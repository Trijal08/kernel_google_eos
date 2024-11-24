#
# Makefile for the Novatek NT383x0 touchscreen driver.
#

EXTRA_INCLUDE ?= -I$(KERNEL_SRC)/../google-modules/nanohub/
EXTRA_SYMBOLS ?= $(OUT_DIR)/../google-modules/nanohub/Module.symvers

default: all

KBUILD_OPTIONS += CONFIG_TOUCHSCREEN_NT383x0=m \
		  ccflags-y=$(EXTRA_INCLUDE) \
		  KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)"

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean::
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions
