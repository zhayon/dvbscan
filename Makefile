CC ?= cc
CFLAGS ?= -O2 -g
CFLAGS += -std=gnu99 -Wall -Wextra
PREFIX ?= /usr/local

all: dvbscan

dvbscan: dvbscan.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

static: dvbscan.c
	$(CC) $(CFLAGS) $(LDFLAGS) -static -o dvbscan $<

install: dvbscan
	install -D -m 755 dvbscan $(DESTDIR)$(PREFIX)/bin/dvbscan

check:
	sh tests/run.sh

clean:
	rm -f dvbscan tests/fakedvb.so

.PHONY: all static install check clean
