# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Google bootloader log
#

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(@)
