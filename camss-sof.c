// SPDX-License-Identifier: GPL-2.0
/*
 * camss-sof - monitor V4L2_EVENT_FRAME_SYNC events from a V4L2 sub-device
 *
 * Subscribes to V4L2_EVENT_FRAME_SYNC on a sub-device, selected either by
 * media entity name or by device node, shows a live frame counter and
 * reports interval / latency statistics on exit.
 *
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>

#include <linux/media.h>
#include <linux/videodev2.h>

#define MAX_IDS			32
#define MAX_CANDIDATES		64
#define FPS_WINDOW		60
#define NSEC_PER_SEC		1000000000ULL
#define OUTLIER_PCT		10.0

struct samples {
	double *v;
	size_t n;
	size_t cap;
};

struct stats {
	double min;
	double max;
	double mean;
	double stddev;
	double median;
	double p95;
	double p99;
	size_t outliers;
};

struct stream {
	unsigned long events;
	uint32_t first_seq;
	uint32_t last_seq;
	unsigned long long seq_gaps;
	uint64_t first_ts;
	uint64_t last_ts;
	struct samples interval;
	struct samples latency;
};

struct candidate {
	char name[64];
	char path[PATH_MAX];
	char media[PATH_MAX];
	unsigned int pads;
};

static volatile sig_atomic_t stop_requested;

static void handle_signal(int sig)
{
	(void)sig;
	stop_requested = 1;
}

static void die(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	fprintf(stderr, "camss-sof: ");
	vfprintf(stderr, fmt, ap);
	va_end(ap);

	exit(EXIT_FAILURE);
}

static uint64_t now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

/* --- sample collection ------------------------------------------------- */

static void samples_add(struct samples *s, double val)
{
	if (s->n == s->cap) {
		size_t cap = s->cap ? s->cap * 2 : 1024;
		double *v = realloc(s->v, cap * sizeof(*v));

		if (!v)
			die("out of memory\n");

		s->v = v;
		s->cap = cap;
	}

	s->v[s->n++] = val;
}

static int cmp_double(const void *a, const void *b)
{
	double x = *(const double *)a;
	double y = *(const double *)b;

	if (x < y)
		return -1;
	if (x > y)
		return 1;

	return 0;
}

static double percentile(const double *sorted, size_t n, double p)
{
	double pos;
	size_t lo;

	if (!n)
		return 0.0;
	if (n == 1)
		return sorted[0];

	pos = (p / 100.0) * (double)(n - 1);
	lo = (size_t)pos;
	if (lo + 1 >= n)
		return sorted[n - 1];

	return sorted[lo] + (pos - (double)lo) * (sorted[lo + 1] - sorted[lo]);
}

static void samples_stats(const struct samples *s, struct stats *out)
{
	double *sorted;
	double sum = 0.0;
	double sq = 0.0;
	size_t i;

	memset(out, 0, sizeof(*out));
	if (!s->n)
		return;

	for (i = 0; i < s->n; i++)
		sum += s->v[i];
	out->mean = sum / (double)s->n;

	for (i = 0; i < s->n; i++) {
		double d = s->v[i] - out->mean;

		sq += d * d;
	}
	out->stddev = s->n > 1 ? sqrt(sq / (double)(s->n - 1)) : 0.0;

	sorted = malloc(s->n * sizeof(*sorted));
	if (!sorted)
		die("out of memory\n");

	memcpy(sorted, s->v, s->n * sizeof(*sorted));
	qsort(sorted, s->n, sizeof(*sorted), cmp_double);

	out->min = sorted[0];
	out->max = sorted[s->n - 1];
	out->median = percentile(sorted, s->n, 50.0);
	out->p95 = percentile(sorted, s->n, 95.0);
	out->p99 = percentile(sorted, s->n, 99.0);

	/*
	 * Count intervals more than OUTLIER_PCT away from the median. The
	 * median is used rather than the mean so that a handful of long
	 * stalls does not hide the rest.
	 */
	if (out->median > 0.0) {
		double limit = out->median * (OUTLIER_PCT / 100.0);

		for (i = 0; i < s->n; i++)
			if (fabs(s->v[i] - out->median) > limit)
				out->outliers++;
	}

	free(sorted);
}

/* --- sub-device discovery ---------------------------------------------- */

static bool devnode_path(unsigned int maj, unsigned int min, char *buf, size_t sz)
{
	char link[64];
	char resolved[PATH_MAX];
	struct dirent *ent;
	dev_t want = makedev(maj, min);
	DIR *dir;

	/* udev keeps /dev/char/<major>:<minor> symlinks around */
	snprintf(link, sizeof(link), "/dev/char/%u:%u", maj, min);
	if (realpath(link, resolved)) {
		snprintf(buf, sz, "%s", resolved);
		return true;
	}

	dir = opendir("/dev");
	if (!dir)
		return false;

	while ((ent = readdir(dir))) {
		char path[PATH_MAX];
		struct stat st;

		snprintf(path, sizeof(path), "/dev/%s", ent->d_name);
		if (stat(path, &st))
			continue;
		if (!S_ISCHR(st.st_mode) || st.st_rdev != want)
			continue;

		snprintf(buf, sz, "%s", path);
		closedir(dir);

		return true;
	}

	closedir(dir);

	return false;
}

static int read_topology(int fd, struct media_v2_topology *topo,
			 struct media_v2_entity **entities,
			 struct media_v2_interface **interfaces,
			 struct media_v2_pad **pads,
			 struct media_v2_link **links)
{
	int attempt;

	for (attempt = 0; attempt < 5; attempt++) {
		memset(topo, 0, sizeof(*topo));
		if (ioctl(fd, MEDIA_IOC_G_TOPOLOGY, topo))
			return -1;

		/* calloc(0) may return NULL, always ask for at least one */
		*entities = calloc(topo->num_entities + 1, sizeof(**entities));
		*interfaces = calloc(topo->num_interfaces + 1, sizeof(**interfaces));
		*pads = calloc(topo->num_pads + 1, sizeof(**pads));
		*links = calloc(topo->num_links + 1, sizeof(**links));
		if (!*entities || !*interfaces || !*pads || !*links)
			die("out of memory\n");

		topo->ptr_entities = (uintptr_t)*entities;
		topo->ptr_interfaces = (uintptr_t)*interfaces;
		topo->ptr_pads = (uintptr_t)*pads;
		topo->ptr_links = (uintptr_t)*links;

		if (!ioctl(fd, MEDIA_IOC_G_TOPOLOGY, topo))
			return 0;

		/* topology changed under us, start over */
		free(*entities);
		free(*interfaces);
		free(*pads);
		free(*links);
	}

	errno = EAGAIN;

	return -1;
}

static unsigned int count_pads(const struct media_v2_pad *pads,
			       unsigned int npads, unsigned int entity_id)
{
	unsigned int i, n = 0;

	for (i = 0; i < npads; i++)
		if (pads[i].entity_id == entity_id)
			n++;

	return n;
}

static bool entity_devnode(const struct media_v2_interface *interfaces,
			   unsigned int nintf, const struct media_v2_link *links,
			   unsigned int nlinks, unsigned int entity_id,
			   char *buf, size_t sz)
{
	unsigned int i, j;

	for (i = 0; i < nlinks; i++) {
		if ((links[i].flags & MEDIA_LNK_FL_LINK_TYPE) !=
		    MEDIA_LNK_FL_INTERFACE_LINK)
			continue;
		if (links[i].sink_id != entity_id)
			continue;

		for (j = 0; j < nintf; j++) {
			if (interfaces[j].id != links[i].source_id)
				continue;

			return devnode_path(interfaces[j].devnode.major,
					    interfaces[j].devnode.minor,
					    buf, sz);
		}
	}

	return false;
}

/*
 * Walk every /dev/media* node and collect the sub-devices, optionally
 * filtered by a case-insensitive substring of the entity name.
 */
static unsigned int find_subdevs(const char *want, struct candidate *out,
				 unsigned int max)
{
	struct dirent *ent;
	unsigned int found = 0;
	DIR *dir;

	dir = opendir("/dev");
	if (!dir)
		die("cannot open /dev: %s\n", strerror(errno));

	while ((ent = readdir(dir)) && found < max) {
		struct media_v2_entity *entities = NULL;
		struct media_v2_interface *interfaces = NULL;
		struct media_v2_pad *pads = NULL;
		struct media_v2_link *links = NULL;
		struct media_v2_topology topo;
		char path[PATH_MAX];
		unsigned int i;
		int fd;

		if (strncmp(ent->d_name, "media", 5))
			continue;

		snprintf(path, sizeof(path), "/dev/%s", ent->d_name);
		fd = open(path, O_RDWR);
		if (fd < 0)
			fd = open(path, O_RDONLY);
		if (fd < 0)
			continue;

		if (read_topology(fd, &topo, &entities, &interfaces, &pads,
				  &links)) {
			close(fd);
			continue;
		}

		for (i = 0; i < topo.num_entities && found < max; i++) {
			struct candidate *c = &out[found];

			if (want && !strcasestr(entities[i].name, want))
				continue;

			if (!entity_devnode(interfaces, topo.num_interfaces,
					    links, topo.num_links,
					    entities[i].id, c->path,
					    sizeof(c->path)))
				continue;

			/* only sub-device nodes can carry frame sync events */
			if (strncmp(c->path, "/dev/v4l-subdev", 15))
				continue;

			snprintf(c->name, sizeof(c->name), "%s",
				 entities[i].name);
			snprintf(c->media, sizeof(c->media), "%s", path);
			c->pads = count_pads(pads, topo.num_pads,
					     entities[i].id);
			found++;
		}

		free(entities);
		free(interfaces);
		free(pads);
		free(links);
		close(fd);
	}

	closedir(dir);

	return found;
}

static void list_subdevs(void)
{
	struct candidate *cand;
	unsigned int i, n;

	cand = calloc(MAX_CANDIDATES, sizeof(*cand));
	if (!cand)
		die("out of memory\n");

	n = find_subdevs(NULL, cand, MAX_CANDIDATES);
	if (!n) {
		printf("no V4L2 sub-devices found\n");
		free(cand);
		return;
	}

	printf("%-28s %-20s %-16s %s\n", "ENTITY", "DEVICE", "MEDIA", "PADS");
	for (i = 0; i < n; i++)
		printf("%-28s %-20s %-16s %u\n", cand[i].name, cand[i].path,
		       cand[i].media, cand[i].pads);

	free(cand);
}

/* --- id list parsing --------------------------------------------------- */

static unsigned int parse_ids(const char *spec, unsigned int *ids,
			      unsigned int max)
{
	unsigned int n = 0;
	const char *p = spec;

	while (*p && n < max) {
		char *end;
		long lo, hi;

		lo = strtol(p, &end, 10);
		if (end == p)
			die("bad id list '%s'\n", spec);
		hi = lo;

		if (*end == '-') {
			p = end + 1;
			hi = strtol(p, &end, 10);
			if (end == p)
				die("bad id range in '%s'\n", spec);
		}

		if (lo < 0 || hi < lo || hi >= MAX_IDS)
			die("id out of range in '%s' (0..%u)\n", spec,
			    MAX_IDS - 1);

		for (; lo <= hi && n < max; lo++) {
			unsigned int j;

			for (j = 0; j < n; j++)
				if (ids[j] == (unsigned int)lo)
					break;
			if (j == n)
				ids[n++] = (unsigned int)lo;
		}

		p = end;
		if (*p == ',')
			p++;
		else if (*p)
			die("bad id list '%s'\n", spec);
	}

	if (!n)
		die("empty id list\n");

	return n;
}

/* --- event handling ---------------------------------------------------- */

static void handle_event(struct stream *st, const struct v4l2_event *ev,
			 uint64_t now)
{
	uint64_t ts = (uint64_t)ev->timestamp.tv_sec * NSEC_PER_SEC +
		      ev->timestamp.tv_nsec;
	uint32_t seq = ev->u.frame_sync.frame_sequence;

	if (st->events) {
		samples_add(&st->interval, (double)(ts - st->last_ts) / 1e6);
		if (seq > st->last_seq + 1)
			st->seq_gaps += seq - st->last_seq - 1;
	} else {
		st->first_ts = ts;
		st->first_seq = seq;
	}

	samples_add(&st->latency, (double)((int64_t)now - (int64_t)ts) / 1e6);

	st->last_ts = ts;
	st->last_seq = seq;
	st->events++;
}

static double window_fps(const struct stream *st)
{
	size_t n = st->interval.n;
	size_t i, start;
	double sum = 0.0;

	if (!n)
		return 0.0;

	start = n > FPS_WINDOW ? n - FPS_WINDOW : 0;
	for (i = start; i < n; i++)
		sum += st->interval.v[i];

	if (sum <= 0.0)
		return 0.0;

	return 1000.0 * (double)(n - start) / sum;
}

static void print_live(const struct stream *streams, const unsigned int *ids,
		       unsigned int nids)
{
	static int prev_len;
	char line[512];
	int len = 0;
	unsigned int i;

	for (i = 0; i < nids; i++) {
		const struct stream *st = &streams[ids[i]];
		double last = st->interval.n ?
			      st->interval.v[st->interval.n - 1] : 0.0;

		len += snprintf(line + len, sizeof(line) - (size_t)len,
				"%sid%u %6lu ev  %6.2f fps  %7.3f ms",
				i ? " | " : "", ids[i], st->events,
				window_fps(st), last);
		if (len >= (int)sizeof(line))
			break;
	}

	printf("\r%-*s", prev_len > len ? prev_len : len, line);
	fflush(stdout);
	prev_len = len;
}

static void report_stream(const struct stream *st, unsigned int id)
{
	struct stats iv, lat;
	double secs;

	printf("\nid %u\n", id);

	if (!st->events) {
		printf("  no events received\n");
		return;
	}

	samples_stats(&st->interval, &iv);
	samples_stats(&st->latency, &lat);

	secs = (double)(st->last_ts - st->first_ts) / 1e9;

	printf("  events           : %lu\n", st->events);
	printf("  timestamp (mono) : %llu.%09llu .. %llu.%09llu\n",
	       (unsigned long long)(st->first_ts / NSEC_PER_SEC),
	       (unsigned long long)(st->first_ts % NSEC_PER_SEC),
	       (unsigned long long)(st->last_ts / NSEC_PER_SEC),
	       (unsigned long long)(st->last_ts % NSEC_PER_SEC));
	printf("  duration         : %.3f s\n", secs);
	if (secs > 0.0)
		printf("  frame rate       : %.3f fps\n",
		       (double)(st->events - 1) / secs);
	printf("  frame_sequence   : %u .. %u (missing: %llu)\n",
	       st->first_seq, st->last_seq, st->seq_gaps);

	if (!st->interval.n) {
		printf("  interval         : need at least 2 events\n");
		return;
	}

	printf("  interval (ms)    : mean %.3f  median %.3f  min %.3f  max %.3f\n",
	       iv.mean, iv.median, iv.min, iv.max);
	printf("                     p95 %.3f  p99 %.3f\n", iv.p95, iv.p99);
	printf("  jitter (stddev)  : %.3f ms", iv.stddev);
	if (iv.mean > 0.0)
		printf(" (%.2f%% of mean)", 100.0 * iv.stddev / iv.mean);
	printf("\n");
	printf("  peak-to-peak     : %.3f ms\n", iv.max - iv.min);
	printf("  outliers (>%.0f%%) : %zu of %zu (%.2f%%)\n", OUTLIER_PCT,
	       iv.outliers, st->interval.n,
	       100.0 * (double)iv.outliers / (double)st->interval.n);
	printf("  latency (ms)     : mean %.3f  median %.3f  max %.3f\n",
	       lat.mean, lat.median, lat.max);
}

static void usage(const char *argv0)
{
	printf("Usage: %s [options]\n\n", argv0);
	printf("Monitor V4L2_EVENT_FRAME_SYNC events from a V4L2 sub-device.\n\n");
	printf("Options:\n");
	printf("  -e, --entity NAME   select sub-device by media entity name (substring)\n");
	printf("  -d, --device PATH   select sub-device node directly (/dev/v4l-subdevN)\n");
	printf("  -i, --id LIST       event ids to subscribe, e.g. 0, 0,2 or 0-3 (default 0)\n");
	printf("  -n, --count N       stop after N events in total (one line per event)\n");
	printf("  -t, --time SEC      stop after SEC seconds\n");
	printf("  -l, --list          list sub-devices and their entity names, then exit\n");
	printf("  -v, --verbose       print one line per event\n");
	printf("  -q, --quiet         no live counter\n");
	printf("  -h, --help          this text\n\n");
	printf("For the Qualcomm CAMSS CSID, id N corresponds to source pad N+1 (RDI N).\n");
	printf("Press Ctrl-C to stop and print the statistics.\n");
}

int main(int argc, char *argv[])
{
	static const struct option opts[] = {
		{ "entity",	required_argument,	NULL, 'e' },
		{ "device",	required_argument,	NULL, 'd' },
		{ "id",		required_argument,	NULL, 'i' },
		{ "count",	required_argument,	NULL, 'n' },
		{ "time",	required_argument,	NULL, 't' },
		{ "list",	no_argument,		NULL, 'l' },
		{ "verbose",	no_argument,		NULL, 'v' },
		{ "quiet",	no_argument,		NULL, 'q' },
		{ "help",	no_argument,		NULL, 'h' },
		{ }
	};
	struct stream *streams;
	const char *entity = NULL;
	const char *device = NULL;
	char path[PATH_MAX];
	char name[64] = "";
	unsigned int ids[MAX_IDS];
	unsigned int nids = 0;
	unsigned long long lost = 0;
	unsigned long total = 0;
	unsigned long limit = 0;
	uint32_t last_ev_seq = 0;
	bool have_ev_seq = false;
	bool verbose = false;
	bool quiet = false;
	bool live;
	double runtime = 0.0;
	uint64_t deadline = 0;
	unsigned int i;
	int fd, opt;

	while ((opt = getopt_long(argc, argv, "e:d:i:n:t:lvqh", opts,
				  NULL)) != -1) {
		switch (opt) {
		case 'e':
			entity = optarg;
			break;
		case 'd':
			device = optarg;
			break;
		case 'i':
			nids = parse_ids(optarg, ids, MAX_IDS);
			break;
		case 'n':
			limit = strtoul(optarg, NULL, 10);
			break;
		case 't':
			runtime = strtod(optarg, NULL);
			break;
		case 'l':
			list_subdevs();
			return EXIT_SUCCESS;
		case 'v':
			verbose = true;
			break;
		case 'q':
			quiet = true;
			break;
		case 'h':
			usage(argv[0]);
			return EXIT_SUCCESS;
		default:
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	if (!nids) {
		ids[0] = 0;
		nids = 1;
	}

	if (device && entity)
		die("use either --device or --entity, not both\n");

	if (device) {
		snprintf(path, sizeof(path), "%s", device);
	} else {
		struct candidate *cand;
		unsigned int n;

		cand = calloc(MAX_CANDIDATES, sizeof(*cand));
		if (!cand)
			die("out of memory\n");

		n = find_subdevs(entity, cand, MAX_CANDIDATES);
		if (!n)
			die("no sub-device matching '%s', try --list\n",
			    entity ? entity : "*");
		if (n > 1 && entity) {
			fprintf(stderr,
				"camss-sof: '%s' matches %u entities, using '%s'\n",
				entity, n, cand[0].name);
			for (i = 0; i < n; i++)
				fprintf(stderr, "    %s (%s)\n", cand[i].name,
					cand[i].path);
		} else if (n > 1) {
			die("several sub-devices present, pick one with --entity or --device\n"
			    "       (see --list)\n");
		}

		snprintf(path, sizeof(path), "%s", cand[0].path);
		snprintf(name, sizeof(name), "%s", cand[0].name);
		free(cand);
	}

	fd = open(path, O_RDWR | O_NONBLOCK);
	if (fd < 0)
		die("cannot open %s: %s\n", path, strerror(errno));

	streams = calloc(MAX_IDS, sizeof(*streams));
	if (!streams)
		die("out of memory\n");

	for (i = 0; i < nids; i++) {
		struct v4l2_event_subscription sub = {
			.type = V4L2_EVENT_FRAME_SYNC,
			.id = ids[i],
		};

		if (ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub)) {
			if (errno == EINVAL)
				die("%s does not support frame sync on id %u\n"
				    "       (driver may not implement V4L2_EVENT_FRAME_SYNC for this pad)\n",
				    path, ids[i]);
			die("cannot subscribe id %u: %s\n", ids[i],
			    strerror(errno));
		}
	}

	signal(SIGINT, handle_signal);
	signal(SIGTERM, handle_signal);

	/*
	 * A bounded run is meant to be read back afterwards, so scroll the
	 * events instead of overwriting a single live counter line.
	 */
	if (limit && !quiet)
		verbose = true;

	live = !quiet && !verbose && isatty(STDOUT_FILENO);

	printf("watching %s", path);
	if (name[0])
		printf(" (%s)", name);
	printf(", id");
	for (i = 0; i < nids; i++)
		printf("%s%u", i ? "," : " ", ids[i]);
	printf("\n");

	if (runtime > 0.0)
		deadline = now_ns() + (uint64_t)(runtime * 1e9);

	while (!stop_requested) {
		struct pollfd pfd = { .fd = fd, .events = POLLPRI };
		int ret;

		ret = poll(&pfd, 1, 200);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			die("poll: %s\n", strerror(errno));
		}

		if (ret && (pfd.revents & POLLPRI)) {
			struct v4l2_event ev;

			while (!ioctl(fd, VIDIOC_DQEVENT, &ev)) {
				uint64_t now = now_ns();
				struct stream *st;

				if (ev.type != V4L2_EVENT_FRAME_SYNC ||
				    ev.id >= MAX_IDS)
					continue;

				/*
				 * fh->sequence counts every event queued on
				 * this handle, so a gap means the kernel
				 * dropped one before we got to it.
				 */
				if (have_ev_seq && ev.sequence > last_ev_seq + 1)
					lost += ev.sequence - last_ev_seq - 1;
				last_ev_seq = ev.sequence;
				have_ev_seq = true;

				st = &streams[ev.id];
				handle_event(st, &ev, now);
				total++;

				if (verbose)
					printf("id %u seq %u frame %u ts %llu.%09llu pend %u lat %.3f ms\n",
					       ev.id, ev.sequence,
					       ev.u.frame_sync.frame_sequence,
					       (unsigned long long)(st->last_ts /
								    NSEC_PER_SEC),
					       (unsigned long long)(st->last_ts %
								    NSEC_PER_SEC),
					       ev.pending,
					       st->latency.v[st->latency.n - 1]);
			}

			if (errno != ENOENT)
				die("VIDIOC_DQEVENT: %s\n", strerror(errno));
		}

		if (live)
			print_live(streams, ids, nids);

		if (limit && total >= limit)
			break;
		if (deadline && now_ns() >= deadline)
			break;
	}

	if (live)
		printf("\n");

	printf("\n--- %s", path);
	if (name[0])
		printf(" (%s)", name);
	printf(" ---\n");
	printf("  total events     : %lu\n", total);
	printf("  events dropped   : %llu", lost);
	if (lost)
		printf("  (kernel event queue overran, userspace too slow)");
	printf("\n");

	for (i = 0; i < nids; i++)
		report_stream(&streams[ids[i]], ids[i]);

	for (i = 0; i < nids; i++) {
		free(streams[ids[i]].interval.v);
		free(streams[ids[i]].latency.v);
	}
	free(streams);
	close(fd);

	return EXIT_SUCCESS;
}
