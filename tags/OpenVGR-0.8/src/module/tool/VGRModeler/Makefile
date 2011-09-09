#
# Makefile for VGRModeler
#
.SUFFIXES:
.SUFFIXES: .cpp .h .o

.cpp.o:
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -o $@ $<

CXX = g++
CXXFLAGS = -Wall -g -O2
LDFLAGS  =

INCLUDES =  -I../../../lib/libopenvgr
LIBS = -L../../../lib/libopenvgr -lopenvgr_misc -lm

objs = VGRModeler.o ModelInformation.o trans.o VRMLWriter.o

all: VGRModeler

VGRModeler: $(objs)
	$(CXX) -o $@ $(objs) $(LDFLAGS) $(LIBS)

clean:
	$(RM) VGRModeler $(objs) *~

.PHONY: all clean
