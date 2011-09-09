#
# Makefile for libopenvgr
#
.SUFFIXES:
.SUFFIXES: .cpp .c .h .o .a

.c.o:
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@ $<

.cpp.o:
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -o $@ $<

OPENCVINC = -I/usr/include/opencv
RTMINC = -I`rtm-config --prefix`/include/rtm/idl
INCLUDES = $(OPENCVINC) $(RTMINC)

CC = gcc
CFLAGS = -O3 -march=native -pipe -Wall
#CFLAGS = -g -O0 -march=native -pipe -Wall

CXX = g++
CXXFLAGS = $(CFLAGS)

IDLDIR   = ../../idl

IDLC     = `rtm-config --idlc`
IDLFLAGS = `rtm-config --idlflags` $(RTMINC)

OBJS = quaternion.o
COMOBJS = calib.o calibUtil.o recogParameter.o\
          recogImage.o imageUtil.o \
          extractEdge.o extractFeature.o \
          vertex.o circle.o conic.o stereo.o \
          pairedcircle.o match3Dfeature.o \
          modelpoints.o score2d.o \
          modelFileio.o modelListFileIO.o \
          rtvcm.o debugutil.o vectorutil.o quaternion.o

all: libopenvgr_misc.a libopenvgr.a

libopenvgr_misc.a: $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

libopenvgr.a: $(COMOBJS)
	$(AR) $(ARFLAGS) $@ $^

calibUtil.o : Img.hh

Img.hh :
	$(IDLC) $(IDLFLAGS) $(IDLDIR)/Img.idl
	/bin/rm Img*.cc

clean:
	$(RM) libopenvgr_misc.a libopenvgr.a $(OBJS) *.o *.hh *~

.PHONY: all clean
