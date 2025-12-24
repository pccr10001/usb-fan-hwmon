ifneq ($(KERNELRELEASE),)
# Kbuild part of makefile
obj-m  := usb_fan_hwmon.o

else
# Normal makefile
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

endif
