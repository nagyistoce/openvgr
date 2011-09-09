#
# Makefile for multicalib
#
.SUFFIXES:
.SUFFIXES: .c .h .o

.c.o:
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@ $<

LIBCAPTURE_DIR = ../../../lib/libcapture

CC  = gcc
CXX = g++

CFLAGS = -O3 -Wall -march=native -fopenmp -pipe
CXXFLAGS = $(CFLAGS)

LDFLAGS = -lpthread -lgomp
LDLIBS = -L$(LIBCAPTURE_DIR) $(OPENCV) -lm
INCLUDES = -I$(LIBCAPTURE_DIR) -I/usr/include/opencv

LIBDC1394 = -ldc1394
OPENCV = -lcvaux -lhighgui -lcv -lcxcore

TARGETS = sample genconf multicalib


all: $(TARGETS)

sample: LDLIBS = -L$(LIBCAPTURE_DIR) -lcapture $(OPENCV) $(LIBDC1394) -lm
sample: sample.o checker_data.o detect_checker.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS) 

genconf: LDLIBS += -lcapture
genconf: genconf.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS) 

multicalib: multicalib.o calib_proc.o calib_data.o camera_param.o quaternion.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS) 

multicalib.o: multicalib.h calib_proc.h
calib_proc.o: calib_proc.h
calib_data.o: calib_data.h
camera_param.o: camera_param.h local.h
quaternion.o: quaternion.h local.h

clean:
	$(RM) $(TARGETS) *.o *~


.PHONY: all clean
