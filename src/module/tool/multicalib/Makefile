#
# Makefile for multicalib
#
LIBCAPTURE_DIR = ../../../lib/libcapture

CC  = gcc
CXX = g++

CPPFLAGS = $(INCLUDES)

CFLAGS = -O3 -Wall -march=native -fopenmp -pipe
CXXFLAGS = $(CFLAGS)

LDFLAGS = -L$(LIBCAPTURE_DIR) 
LDLIBS = $(OPENCV) -lm -lpthread -lgomp
INCLUDES = -I$(LIBCAPTURE_DIR) $(shell pkg-config --cflags opencv)

LIBDC1394 = -ldc1394
OPENCV = $(shell pkg-config --libs opencv)

TARGETS = ichimatsu genconf reset_bus multicalib


all: $(TARGETS)

ichimatsu: LDLIBS = -L$(LIBCAPTURE_DIR) -lcapture $(OPENCV) $(LIBDC1394) -lm
ichimatsu: ichimatsu.o checker_data.o detect_checker.o

genconf: LDLIBS += $(LIBDC1394)
reset_bus: LDLIBS += $(LIBDC1394)

multicalib: multicalib.o calib_proc.o calib_data.o camera_param.o quaternion.o optimizer.o

multicalib.o: multicalib.hpp calib_proc.hpp
calib_proc.o: calib_proc.hpp local.h
calib_data.o: calib_data.h
camera_param.o: camera_param.h local.h
quaternion.o: quaternion.h local.h
optimizer.o: optimizer.h

clean:
	$(RM) $(TARGETS) *.o *~


.PHONY: all clean
