#
# Makefile
#
.SUFFIXES:

LN = ln -s
RM = rm -f

CURDIR = $(shell pwd)
TOPDIR = $(shell dirname $(CURDIR))

COMPDIR=$(TOPDIR)/src/module/component
TOOLDIR=$(TOPDIR)/src/module/tool
EXDIR=$(TOPDIR)/example/component

COMP = MultiCameraComp MultiDispComp Measure3DComp \
       RecognitionComp RecognitionResultViewerComp

TOOL = multicalib sample genconf \
       VGRModeler

EX   = MultiCameraTriggerComp SendImageComp SetModelIDComp

all: $(COMP) $(TOOL) $(EX)

MultiCameraComp: $(COMPDIR)/MultiCamera/MultiCameraComp
	$(LN) $<

MultiDispComp: $(COMPDIR)/MultiDisp/MultiDispComp
	$(LN) $<

Measure3DComp: $(COMPDIR)/Measure3D/Measure3DComp
	$(LN) $<

RecognitionComp: $(COMPDIR)/Recognition/RecognitionComp
	$(LN) $<

RecognitionResultViewerComp: $(COMPDIR)/RecognitionResultViewer/RecognitionResultViewerComp
	$(LN) $<

multicalib: $(TOOLDIR)/multicalib/multicalib
	$(LN) $<

sample: $(TOOLDIR)/multicalib/sample
	$(LN) $<

genconf: $(TOOLDIR)/multicalib/genconf
	$(LN) $<

VGRModeler: $(TOOLDIR)/VGRModeler/VGRModeler
	$(LN) $<

MultiCameraTriggerComp: $(EXDIR)/MultiCameraTrigger/MultiCameraTriggerComp
	$(LN) $<

SendImageComp: $(EXDIR)/SendImage/SendImageComp
	$(LN) $<

SetModelIDComp: $(EXDIR)/SetModelID/SetModelIDComp
	$(LN) $<

clean: 
	-$(RM) $(COMP) $(TOOL) $(EX)
