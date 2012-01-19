# Makefile for transmat

SHELL         = /bin/sh

SRCDIR        = .
OPENCVINCDIR  = /usr/include/opencv

INCLUDES      = -I$(SRCDIR) -I$(OPENCVINCDIR)
LIBS          = -lcxcore

CC            = gcc -pipe
CXX           = g++
LINKER        = gcc -pipe
CFLAGS        = -O2 -Wall  
CPPFLAGS      = -O2 -Wall
#CPPFLAGS      = -g -Wall
DEFS          =
LDFLAGS       = 
COMPILE       = $(CC) -c $(DEFS) $(INCLUDES) $(CPPFLAGS) $(CFLAGS)
CXXCOMPILE    = $(CXX) -c $(DEFS) $(INCLUDES) $(CPPFLAGS)
LINK          = $(LINKER) $(LDFLAGS) -o $@

PROGRAM       = transmat

HDRS          = 

SRCS          = transmat.cpp \
	findmatrix.cpp \
	readdata.cpp

OBJS          = transmat.o \
	findmatrix.o \
	readdata.o \
	libopenvgr_misc.a

.SUFFIXES:
.SUFFIXES: .cpp .c .h .o
.c.o:
	$(COMPILE) $<
.cpp.o:
	$(CXXCOMPILE) $<

all: $(PROGRAM)

$(PROGRAM): $(OBJS)
	$(LINK) $(OBJS) $(LIBS)

$(OBJS):

clean:
	rm -f *.o *~ core $(PROGRAM)

###
