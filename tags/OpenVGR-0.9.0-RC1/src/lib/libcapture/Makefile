#
# Makefile for libcapture
#
.SUFFIXES:
.SUFFIXES: .c .h .o .a

.c.o:
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@ $<


CC = gcc
CFLAGS = -O3 -march=native -Wall -pipe

TARGETS = libcapture.a


all: $(TARGETS)


libcapture.a: capture.o
	$(AR) $(ARFLAGS) $@ $^

clean:
	$(RM) $(TARGETS) *.o *~


capture.o: capture.h


.PHONY: all clean
