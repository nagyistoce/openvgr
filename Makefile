#
# Main Makefile for OpenVGR
#
.PHONY: all clean clean-recursive clean-local

SUBDIRS = src example

all:
	@for subdir in $(SUBDIRS); do \
	  echo making $@ in $$subdir; \
	  (cd $$subdir && $(MAKE) $@) || exit 1; \
	done

test: clean
	$(MAKE) all "DEFS=-DRECOGNITION_TEST"

clean: clean-recursive clean-local

clean-recursive:
	@for subdir in $(SUBDIRS); do \
	  target=`echo $@ | sed 's/-recursive//'`; \
	  echo making $$target in $$subdir; \
	  (cd $$subdir && $(MAKE) $$target) || exit 1; \
	done

clean-local:
	rm -f *~
