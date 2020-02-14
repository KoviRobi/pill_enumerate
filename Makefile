TARGETS=stm32/f1

ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

.PHONY: all clean program
all:
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 TARGETS=$(TARGETS) lib
	$(Q)$(MAKE) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@

program: src/pill_enumerate.hex
	# Waiting for STLink programmer
	while ! grep 'STM32 STLink' /sys/bus/usb/devices/*/product; do sleep 1s; done
	# Programming
	sudo st-flash --format ihex write $<
