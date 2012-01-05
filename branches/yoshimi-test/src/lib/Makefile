#
# Makefile for OpenVGR libraries
#
.PHONY: all clean clean-recursive clean-local

CC = gcc
CXX = g++
CFLAGS = -O3 -march=native -Wall -pipe -fPIC
LDFLAGS =

DEFINES = CC='$(CC)' CXX='$(CXX)' CFLAGS='$(CFLAGS)' LDFLAGS='$(LDFLAGS)'

SUBDIRS = libcapture \
          libopenvgr

all :
	@for subdir in $(SUBDIRS); do \
	  echo making $@ in $$subdir; \
	  (cd $$subdir && $(MAKE) $(DEFINES) $@) || exit 1; \
	done

clean: clean-recursive clean-local

clean-recursive:
	@for subdir in $(SUBDIRS); do \
	  target=`echo $@ | sed 's/-recursive//'`; \
	  echo making $$target in $$subdir; \
	  (cd $$subdir && $(MAKE) $$target) || exit 1; \
	done

clean-local:
	rm -f *~
