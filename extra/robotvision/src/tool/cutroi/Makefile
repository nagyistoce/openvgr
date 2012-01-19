# Makefile for cutroi

SHELL         = /bin/sh

SRCDIR        = .
OPENCVINCDIR  = /usr/include/opencv

INCLUDES      = -I$(SRCDIR) -I$(OPENCVINCDIR)
LIBS          = -lhighgui

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

PROGRAM       = cutroi

HDRS          = 

SRCS          = cutroi.cpp

OBJS          = cutroi.o

.SUFFIXES:
.SUFFIXES: .cpp .o
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
