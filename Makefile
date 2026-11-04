# SPDX-License-Identifier: GPL-2.0

CC ?= gcc
CFLAGS ?= -O2 -g -Wall -Wextra
LDLIBS = -lm

# Point KDIR at a kernel tree with installed headers (make headers_install)
# to build against a newer uapi than the host provides.
ifneq ($(KDIR),)
CPPFLAGS += -I$(KDIR)/usr/include
endif

TARGET = camss-sof

all: $(TARGET)

$(TARGET): camss-sof.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
