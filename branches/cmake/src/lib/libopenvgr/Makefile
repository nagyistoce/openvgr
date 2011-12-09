#
# Makefile for libopenvgr
#
.SUFFIXES:
.SUFFIXES: .cpp .c .h .o .a

.c.o:
	$(CC) -c $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@ $<

.cpp.o:
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) $(DEFS) -o $@ $<

OPENCVINC = -I/usr/include/opencv
RTMINC = -I`rtm-config --prefix`/include/rtm/idl
INCLUDES = $(OPENCVINC) $(RTMINC)

CC = gcc

ifeq ($(CC),icc)
  CFLAGS = -O3 -Wall -pipe -fPIC -fopenmp
  CXX = icpc
  AR = xiar
else
  CFLAGS = -O3 -march=native -Wall -pipe -fPIC
  #CFLAGS = -O3 -march=native -Wall -pipe -fPIC -fopenmp
  #CFLAGS = -g -O0 -march=native -Wall -pipe -fPIC

  CXX = g++
endif

CXXFLAGS = $(CFLAGS)

IDLDIR   = ../../idl

IDLC     = `rtm-config --idlc`
IDLFLAGS = `rtm-config --idlflags` $(RTMINC)

OBJS = quaternion.o
COMOBJS = calib.o calibUtil.o recogParameter.o\
          recogImage.o imageUtil.o \
          extractEdge.o extractFeature_old.o \
          vertex.o circle.o conic.o stereo.o \
          pairedcircle.o match3Dfeature.o \
          modelpoints.o score2d.o \
          modelFileio.o modelListFileIO.o \
          rtvcm.o debugutil.o vectorutil.o quaternion.o \
          geometry.o extractFeature.o correspondence.o \
          drawing.o mathmisc.o \
          searchEllipse_IW.o RecognitionKernel.o \
          merge_ellipse.o

DEFS =

all: libopenvgr_misc.a libopenvgr.a

libopenvgr_misc.a: $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

libopenvgr.a: $(COMOBJS)
	$(AR) $(ARFLAGS) $@ $^

calibUtil.o : Img.hh
extractFeature.o: extractFeature.hpp
match3Dfeature.o pairedcircle.o score2d.o: common.h
modelpoints.o score2d.o: modelpoints.h

Img.hh :
	$(IDLC) $(IDLFLAGS) $(IDLDIR)/Img.idl
	/bin/rm Img*.cc

test: clean
	$(MAKE) all "DEFS=-DRECOGNITION_TEST"

clean:
	$(RM) libopenvgr_misc.a libopenvgr.a $(OBJS) *.o *.hh *~

.PHONY: all clean
