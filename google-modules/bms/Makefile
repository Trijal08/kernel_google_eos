# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Google Battery Management System
#
GBMS_MODULES =	GOOGLE_BMS \
		GOOGLE_BATTERY \
		GOOGLE_CHARGER \
		GOOGLE_BMS_SW5100

obj-$(CONFIG_GOOGLE_BMS)	+= google-bms.o
google-bms-objs += google_bms.o
google-bms-objs += gbms_storage.o

# TODO(166536889): enable bee only on the devices supporting it. This will
# require a change in the API since right now storage call into eeprom that
# calls back into storage.
# KBUILD_OPTIONS += CONFIG_GOOGLE_BEE=m \

obj-$(CONFIG_GOOGLE_BEE)	+= google_eeprom.o
obj-$(CONFIG_GOOGLE_BEE)	+= google_eeprom_01.o
obj-$(CONFIG_GOOGLE_BEE)	+= gs101_usecase.o

# Battery
obj-$(CONFIG_GOOGLE_BATTERY) += google-battery.o
google-battery-objs += google_battery.o
google-battery-objs += google_ttf.o

# google_charger
obj-$(CONFIG_GOOGLE_CHARGER) += google-charger.o
google-charger-objs += google_charger.o

# google_dual_batt_gauge
obj-$(CONFIG_GOOGLE_DUAL_BATT_GAUGE)	+= google_dual_batt_gauge.o

# charging policy manager, for devices that have more HW chargers
# requires google_dc_pps
obj-$(CONFIG_GOOGLE_CPM)	+= google-cpm.o
google-cpm-objs += google_cpm.o
google-cpm-objs += google_dc_pps.o

# google_dock
obj-$(CONFIG_GOOGLE_DOCK)	+= google_dock.o

# Overheat mitigation driver
obj-$(CONFIG_USB_OVERHEAT_MITIGATION)	+= overheat_mitigation.o

# max7729f drivers for the single SSID
obj-$(CONFIG_PMIC_MAX77729)	+= max77729-pmic.o
max77729-pmic-objs += max77729_pmic.o
max77729-pmic-objs += max77759_maxq.o

obj-$(CONFIG_UIC_MAX77729)	+= max77729_uic.o
obj-$(CONFIG_CHARGER_MAX77729)	+= max77729_charger.o
# Muirwoods drivers for the single SSID (max77729_pmic is shared)
obj-$(CONFIG_CHARGER_MAX77759)	+= max77759_charger.o

# Wireless charging
obj-$(CONFIG_CHARGER_P9221)	+= p9221.o
p9221-objs += p9221_charger.o
p9221-objs += p9221_chip.o

# Standalone for pca9468
obj-$(CONFIG_PCA9468)		+= pca9468.o
pca9468-objs += pca9468_charger.o
pca9468-objs += pca9468_gbms_pps.o
pca9468-objs += google_dc_pps.o

obj-$(CONFIG_PCA9468_GOOGLE)  += pca9468-google.o
pca9468-google-objs += pca_charger.o
pca9468-google-objs += pca9468_gbms_pps.o
pca9468-google-objs += google_dc_pps.o

# Alternate (untested) standalone for max77729f sans FG
obj-$(CONFIG_MAX77729)		+= max77729.o
max77729-objs += max77729_pmic.o
max77729-objs += max77729_uic.o
max77729-objs += max77729_charger.o

# Alternate (untested) standalone for max77759 sans FG
obj-$(CONFIG_MAX77759)		+= max77759.o
max77759-objs += max77729_pmic.o
max77759-objs += max77729_uic.o
max77759-objs += max77729_charger.o
max77759-objs += max77759_maxq.o

obj-$(CONFIG_MAX1720X_BATTERY)  += max1720x-battery.o
max1720x-battery-objs += max1720x_battery.o
max1720x-battery-objs += max1720x_outliers.o
max1720x-battery-objs += max_m5.o

# OVP
obj-$(CONFIG_MAX20339)	+= max20339.o

#SW5100 BMS
obj-$(CONFIG_GOOGLE_BMS_SW5100) += sw5100_bms.o

obj-$(CONFIG_GOOGLE_BMS_VIRT_STORAGE) += gbms_virt_storage.o

# WLC98 Wireless charging
obj-$(CONFIG_STWLC98)	+= wlc98_driver.o

# WC68 DC Charge pump
obj-$(CONFIG_STWC68)	+= wc68.o
wc68-objs += wc68_driver.o
wc68-objs += wc68_gbms_pps.o
wc68-objs += google_dc_pps.o

# prevent warnings
WENUMS=-Wno-enum-conversion -Wno-switch

CFLAGS_max77759_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max77729_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max1720x_battery.o += $(WENUMS)
CFLAGS_pca9468_charger.o += $(WENUMS)
CFLAGS_pca9468_gbms_pps.o += $(WENUMS)
CFLAGS_pca_charger.o += $(WENUMS)
CFLAGS_google_battery.o += $(WENUMS)
CFLAGS_google_ttf.o += -Wno-format
CFLAGS_google_charger.o += -Wno-enum-conversion
CFLAGS_google_bms.o += -Wno-enum-conversion
CFLAGS_google_cpm.o += $(WENUMS)
CFLAGS_google_dual_batt_gauge.o += $(WENUMS)
CFLAGS_google_dock.o += $(WENUMS)
CFLAGS_wc68_driver.o += $(WENUMS)
CFLAGS_wc68_gbms_pps.o += $(WENUMS)

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

subdir-ccflags-y += \
		-I$(KERNEL_SRC)/../google-modules/bms \

KBUILD_OPTIONS += $(foreach m,$(GBMS_MODULES),CONFIG_$(m)=m )

EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/drivers/power/supply/qcom
EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS += $(foreach m,$(GBMS_MODULES),-DCONFIG_$(m)_MODULE)
EXTRA_CFLAGS	+= -DCONFIG_DISABLE_GOOGLE_DC_PPS

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		$(@)

EXTRA_SYMBOLS += $(OUT_DIR)/../google-modules/bms/misc/Module.symvers
EXTRA_SYMBOLS += $(OUT_DIR)/../msm-google/drivers/power/supply/qcom/Module.symvers
include $(KERNEL_SRC)/../google-modules/soc/msm/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

print-%:
	@echo $* = $($*)

value-%:
	@echo $($*)
