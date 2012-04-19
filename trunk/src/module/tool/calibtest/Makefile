#
# Makefile for calibtest
#
LIBCAPTURE_DIR = ../../../lib/libcapture
LIBMISC_DIR = ../../../lib/libopenvgr

CC  = gcc
CXX = g++

CPPFLAGS = $(INCLUDES)

CFLAGS = -O3 -Wall -march=native -fopenmp -pipe
CXXFLAGS = $(CFLAGS)

LDFLAGS = -L$(LIBCAPTURE_DIR) -L$(LIBMISC_DIR)
LDLIBS = $(OPENCV) -lm -lpthread -lgomp
INCLUDES = -I$(LIBCAPTURE_DIR) -I$(LIBMISC_DIR) $(shell pkg-config --cflags opencv)

LIBDC1394 = -ldc1394
OPENCV = $(shell pkg-config --libs opencv)
GLLIB = -lglut -lGLU -lGL

TARGETS = calibtest


all: $(TARGETS)

calibtest: LDLIBS = -L$(LIBCAPTURE_DIR) -lcapture -L$(LIBMISC_DIR) -lopenvgr_misc $(OPENCV) $(LIBDC1394) $(GLLIB) -lm
calibtest: calibtest.o glwindow.o cv3d.o globj.o capture.o

%.o: %.hpp
calibtest.o: glwindow.hpp
glwindow.o: calibtest.hpp globj.hpp cv3d.hpp
cv3d.o: cv3d.hpp
globj.o: globj.hpp
capture.o: capture.hpp

clean:
	$(RM) $(TARGETS) *.o *~


.PHONY: all clean
