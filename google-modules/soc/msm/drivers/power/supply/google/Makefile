# SPDX-License-Identifier: GPL-2.0
#
# Makefile for HVDCP controller
#

# Used for gvotable.h and smblite includes
EXTRA_CFLAGS += -I$(KERNEL_SRC)/../google-modules/bms
EXTRA_CFLAGS += -I$(KERNEL_SRC)/drivers/power/supply/qcom

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE

# Used for gvotable.ko and qpnp-smblite-main.ko
EXTRA_SYMBOLS += $(OUT_DIR)/../google-modules/bms/misc/Module.symvers
EXTRA_SYMBOLS += $(OUT_DIR)/drivers/power/supply/qcom/Module.symvers

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" modules

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
