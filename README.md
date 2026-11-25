# camss-sof

A small tool to watch `V4L2_EVENT_FRAME_SYNC` (SOF) events on a V4L2
sub-device. Written for bringing up and sanity-checking the SOF/frame-sync
support in the Qualcomm CAMSS CSID driver, but it is not CAMSS-specific: any
sub-device that implements the event works.

It shows a live, inline-updating frame counter while running, and prints
interval / jitter / latency statistics when it stops.

## Build

```
make
```

Native build, no dependencies beyond libc and libm. To build against a
kernel tree's uapi headers instead of the host's:

```
make KDIR=/path/to/linux
```

(after `make headers_install` in that tree).

## Usage

```
camss-sof [options]

  -e, --entity NAME   select sub-device by media entity name (substring)
  -d, --device PATH   select sub-device node directly (/dev/v4l-subdevN)
  -i, --id LIST       event ids to subscribe, e.g. 0, 0,2 or 0-3 (default 0)
  -n, --count N       stop after N events in total (one line per event)
  -t, --time SEC      stop after SEC seconds
  -l, --list          list sub-devices and their entity names, then exit
  -v, --verbose       print one line per event
  -q, --quiet         no live counter
  -h, --help          help text
```

Devnode numbering is not stable across boots, so selecting by entity name is
usually what you want. Find the names first:

```
# ./camss-sof --list
ENTITY                       DEVICE               MEDIA            PADS
msm_csiphy0                  /dev/v4l-subdev0     /dev/media0      2
msm_csid0                    /dev/v4l-subdev4     /dev/media0      5
msm_vfe0_rdi0                /dev/v4l-subdev8     /dev/media0      2
imx412 4-001a                /dev/v4l-subdev12    /dev/media0      1
```

Then watch one, in parallel with a capture:

```
# ./camss-sof -e msm_csid0 -i 0
watching /dev/v4l-subdev4 (msm_csid0), id 0
id0    871 ev   29.97 fps   33.368 ms
```

The live line is only drawn on a tty; piping to a file or using `-q`/`-v`
suppresses it, so logging is clean.

Multiple RDIs at once:

```
# ./camss-sof -e msm_csid0 -i 0-2
```

Ctrl-C (or `-n`/`-t`) stops the run and prints the report:

```
--- /dev/v4l-subdev4 (msm_csid0) ---
  total events     : 871
  events dropped   : 0

id 0
  events           : 871
  duration         : 29.062 s
  frame rate       : 29.936 fps
  frame_sequence   : 0 .. 870 (missing: 0)
  interval (ms)    : mean 33.405  median 33.368  min 32.904  max 34.112
                     p95 33.712  p99 33.904
  jitter (stddev)  : 0.187 ms (0.56% of mean)
  peak-to-peak     : 1.208 ms
  outliers (>10%)  : 0 of 870 (0.00%)
  latency (ms)     : mean 0.214  median 0.198  max 1.431
```

## Reading the output

* **interval** — time between consecutive SOFs, from the kernel timestamps
  carried in the event (`CLOCK_MONOTONIC`), so it measures the hardware, not
  this process' scheduling.
* **jitter** — standard deviation of the interval. A stable sensor at a fixed
  link rate should be well under 1% of the mean; a large value means dropped
  frames, a varying blanking interval, or interrupt coalescing.
* **peak-to-peak / p99 / outliers** — where the instability actually lives.
  `outliers` counts intervals more than 10% away from the *median*, so a few
  long stalls cannot hide behind a skewed mean. Roughly double the median
  interval in `max` with a matching outlier count is the signature of SOF
  interrupts being missed.
* **frame_sequence** — the counter the driver puts in the event payload.
  `missing` is the number of values that never arrived, i.e. frames the
  hardware signalled but we never saw. For the CAMSS CSID this counter starts
  at 0 on every stream-on.
* **events dropped** — gaps in the per-filehandle `v4l2_event.sequence`,
  meaning the kernel's event queue overran before we dequeued. The V4L2 core
  clamps a frame-sync subscription to a single queued event, so this will tick
  up if the process is descheduled for longer than a frame period; it is a
  property of this tool's scheduling, not of the hardware. Contrast with
  `missing` above, which counts frames lost upstream of the event queue.
* **latency** — kernel event timestamp to the moment this process dequeued it.
  Pure userspace wakeup latency, useful when deciding whether SOF events are
  good enough to drive frame pacing from userspace.

## Notes

* Requires read/write access to the media and sub-device nodes; normally run
  as root or with the `video` group.
* `--entity` does a case-insensitive substring match. If it matches more than
  one entity the first is used and the rest are listed on stderr.
* For the CAMSS CSID, event id *N* is source pad *N+1*, i.e. RDI *N*. A driver
  that does not implement frame sync for that pad fails the subscribe with
  `EINVAL`, which is reported as such.
* Events only flow while the pipeline is streaming, and the CSID only queues
  them while at least one subscriber exists — so start this tool before or
  during the capture, not after.
