/*
 * Copyright 2022 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define BMS_DEV_NAME	"sw5100_bms"
#define pr_fmt(fmt) BMS_DEV_NAME": " fmt

#include <linux/of.h>
#include <linux/of_platform.h>
//#include <linux/of_batterydata.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/pmic-voter.h>
#include <linux/regmap.h>
#include <linux/bitops.h>
#include <linux/iio/consumer.h>
#include <linux/regulator/consumer.h>
#include "google_bms.h"
/* hackaroo... */
#include "battery-profile-loader.h"
#include "smblite-reg.h"
#include "smblite-lib.h"
#include "smb5-iio.h"

#define CAPACITY_OFFSET 5
#define BIAS_STS_READY	BIT(0)

#define CHARGE_DISABLE_VOTER	"charge_disable"
#define USBIN_DISABLE_VOTER	"USBIN_DISABLE"

struct bms_dev {
	struct	device			*dev;
	struct	power_supply		*psy;
	struct	regmap			*pmic_regmap;
	struct	votable			*fv_votable;
	struct	votable			*fcc_votable;
	struct	votable			*dc_suspend_votable;
	struct	votable			*icl_votable;
	struct	notifier_block		nb;
	int				batt_id_ohms;
	u32				rradc_base;
	int				chg_term_voltage;
	int				chg_term_voltage_debounce;
	/* Amount of SOC percentage points to offset to 0% UI SOC. */
	int				soc_shutdown_offset;
	struct iio_channel		*batt_therm_chan;
	struct iio_channel		*batt_id_chan;
	struct iio_channel		**iio_chan_list_qg;

};

struct bias_config {
	u16	status_reg;
	u16	lsb_reg;
	int	bias_kohms;
};

//CHARGER_STATUS_1
#define CHGR_BATTERY_CHARGER_STATUS_REG		0x2606
#define CHG_ERR_STATUS_SFT_EXPIRE		BIT(5)
#define CHG_ERR_STATUS_BAT_OV			BIT(4)

//CHARGER_STATUS_2
#define CHGR_BATTERY_CHARGER_EN_STATUS_REG	0x2607
#define ENABLE_TRICKLE_BIT			BIT(1)
#define ENABLE_PRE_CHARGING_BIT			BIT(2)
#define ENABLE_FULLON_MODE_BIT			BIT(3)

//CHARGER_STATUS_7
#define BATIF_BAT_TEMP_STATUS_REG		0x280D
#define BAT_TEMP_WARM				BIT(3)
#define BAT_TEMP_COOL				BIT(2)
#define BAT_TEMP_TOO_HOT			BIT(4)
#define BAT_TEMP_TOO_COLD			BIT(1)

#define CHGR_FLOAT_VOLTAGE_NOW			0x260B
#define CHGR_CHG_EN				0x2646
#define CHARGING_ENABLE_CMD_BIT			BIT(0)

#define CHGR_CHARGING_PAUSE_CMD			0x2646
#define CHARGING_PAUSE_CMD_BIT			BIT(4)

#define CHGR_FAST_CHARGE_CURRENT_SETTING	0x2654
#define CHGR_ADC_ITERM_UP_THD_MSB		0x2664
#define CHGR_FLOAT_VOLTAGE_SETTING		0x2658
#define CHGR_CHG_TERM_CFG_REG			0x2660
#define CHGR_ITERM_USE_ANALOG_BIT		BIT(3)

#define DCDC_ICL_STATUS_REG			0x2709
#define DCDC_AICL_ICL_STATUS_REG		0x2707
#define DCDC_AICL_STATUS_REG			0x2C06
#define DCDC_SOFT_ILIMIT_BIT			BIT(6)

#define DCDC_POWER_PATH_STATUS_REG		0x270B
#define USE_USBIN_BIT				BIT(5)
#define USE_DCIN_BIT				BIT(4)
#define VALID_INPUT_POWER_SOURCE_STS_BIT	BIT(7)

#define MISC_AICL_CMD_REG			0x2C50

#define BATIF_INT_RT_STS			0x2810
#define BATIF_THERM_OR_ID_MISSING_RT_STS_BIT	BIT(1)

#define CHGR_BATTERY_CHARGER_STATUS_MASK	GENMASK(2, 0)

#define CHGR_USB_SUSPEND			0x2954
#define USBIN_SUSPEND				BIT(0)
#define SUSPEND_ON_COLLAPSE_USBIN		BIT(7)

#define QBG_MAIN_QBG_STATE_FORCE_CMD		0x4F41
#define FORCE_HIGH_POWER_SHIFT			2

#define CHGR_FLOAT_VOLTAGE_BASE			3600000
#define CHGR_CHARGE_CURRENT_STEP		25000

#define CHG_TERM_VOLTAGE			4350
#define CHG_TERM_VOLT_DEBOUNCE			200

#define PM5100_ADC_CHG_ITERM_MULT		16384

/* sync from google_battery.c */
#define DEFAULT_BATT_DRV_RL_SOC_THRESHOLD	97

enum sw5100_chg_status {
	SW5100_INHIBIT_CHARGE		= 0,
	SW5100_TRICKLE_CHARGE		= 1,
	SW5100_PRE_CHARGE		= 2,
	SW5100_FULLON_CHARGE		= 3,
	SW5100_TAPER_CHARGE		= 4,
	SW5100_TERMINATE_CHARGE	= 5,
	SW5100_PAUSE_CHARGE		= 6,
	SW5100_DISABLE_CHARGE		= 7,
};

static int sw5100_read(struct regmap *pmic_regmap, int addr, u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_read(pmic_regmap, addr, val, len);
	if (rc < 0) {
		pr_err("Failed regmap_read for address %04x rc=%d\n", addr, rc);
		return rc;
	}

	return 0;
}

static int sw5100_write(struct regmap *pmic_regmap, int addr, u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_write(pmic_regmap, addr, val, len);

	if (rc < 0) {
		pr_err("Failed regmap_write for address %04x rc=%d\n",
				addr, rc);
		return rc;
	}

	return 0;
}

static int sw5100_masked_write(struct regmap *pmic_regmap,
			       u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(pmic_regmap, addr, mask, val);
}

static int sw5100_rd8(struct regmap *pmic_regmap, int addr, u8 *val)
{
	return sw5100_read(pmic_regmap, addr, val, 1);
}

/* ------------------------------------------------------------------------- */

enum sw5100_qbg_iio_channels {
	SW5100_QBG_DEBUG_BATTERY,
	SW5100_QBG_CAPACITY,
	SW5100_QBG_REAL_CAPACITY,
	SW5100_QBG_CURRENT_NOW,
	SW5100_QBG_VOLTAGE_NOW,
	SW5100_QBG_VOLTAGE_MAX,
	SW5100_QBG_CHARGE_FULL,
	SW5100_QBG_RESISTANCE_ID,
	SW5100_QBG_TEMP,
	SW5100_QBG_CHARGE_COUNTER,
	SW5100_QBG_CYCLE_COUNT,
	SW5100_QBG_CHARGE_FULL_DESIGN,
	SW5100_QBG_TIME_TO_FULL_NOW,
	SW5100_QBG_TIME_TO_EMPTY_AVG,
	SW5100_QBG_VOLTAGE_AVG,
	SW5100_QBG_VOLTAGE_OCV,
	SW5100_QBG_MAX,
};

/* QBG/FG channels */
static const char * const sw5100_qbg_ext_iio_chan[] = {
	[SW5100_QBG_DEBUG_BATTERY] = "debug_battery",
	[SW5100_QBG_CAPACITY] = "capacity",
	[SW5100_QBG_REAL_CAPACITY] = "real_capacity",
	[SW5100_QBG_CURRENT_NOW] = "current_now",
	[SW5100_QBG_VOLTAGE_NOW] = "voltage_now",
	[SW5100_QBG_VOLTAGE_MAX] = "voltage_max",
	[SW5100_QBG_CHARGE_FULL] = "charge_full",
	[SW5100_QBG_RESISTANCE_ID] = "resistance_id",
	[SW5100_QBG_TEMP] = "temp",
	[SW5100_QBG_CHARGE_COUNTER] = "charge_counter",
	[SW5100_QBG_CYCLE_COUNT] = "cycle_count",
	[SW5100_QBG_CHARGE_FULL_DESIGN] = "charge_full_design",
	[SW5100_QBG_TIME_TO_FULL_NOW] = "time_to_full_now",
	[SW5100_QBG_TIME_TO_EMPTY_AVG] = "time_to_empty_avg",
	[SW5100_QBG_VOLTAGE_AVG] = "voltage_avg",
	[SW5100_QBG_VOLTAGE_OCV] = "voltage_ocv",
};

static int sw5100_get_prop_from_bms(struct bms_dev *bms, int channel, int *val)
{
	int rc;

	if (IS_ERR_OR_NULL(bms->iio_chan_list_qg))
		return -ENODEV;

	rc = iio_read_channel_processed(bms->iio_chan_list_qg[channel],
					val);

	return rc < 0 ? rc : 0;
}

static struct iio_channel **sw5100_get_ext_channels(struct device *dev,
		 const char *const *channel_map, int size)
{
	int i, rc = 0;
	struct iio_channel **iio_ch_ext;

	iio_ch_ext = devm_kcalloc(dev, size, sizeof(*iio_ch_ext), GFP_KERNEL);
	if (!iio_ch_ext)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < size; i++) {
		iio_ch_ext[i] = devm_iio_channel_get(dev, channel_map[i]);

		if (IS_ERR(iio_ch_ext[i])) {
			rc = PTR_ERR(iio_ch_ext[i]);
			if (rc != -EPROBE_DEFER)
				dev_err(dev, "%s channel unavailable, %d\n", channel_map[i], rc);
			return ERR_PTR(rc);
		}
	}

	return iio_ch_ext;
}

/* ------------------------------------------------------------------------- */

static irqreturn_t sw5100_chg_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct bms_dev *chg = irq_data->parent_data;
	u8 stat;
	int rc;
	u8 val;

	dev_dbg(chg->dev, "IRQ: %s\n", irq_data->name);

	rc = sw5100_read(chg->pmic_regmap, CHGR_BATTERY_CHARGER_STATUS_REG, &stat, 1);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	power_supply_changed(chg->psy);

	/* Modify QBG update rate for different charge states. */
	switch (stat & CHGR_BATTERY_CHARGER_STATUS_MASK) {
		case SW5100_FULLON_CHARGE:
		case SW5100_TAPER_CHARGE:
			/* Force fuel gauge update interval to high power mode. */
			val = (1 << FORCE_HIGH_POWER_SHIFT);
			rc = sw5100_write(chg->pmic_regmap, QBG_MAIN_QBG_STATE_FORCE_CMD, &val, 1);
			if (rc < 0) {
				dev_err(chg->dev, "Failure to force QBG HPM rc=%d\n", rc);
			}
			break;
		case SW5100_TERMINATE_CHARGE:
		case SW5100_PAUSE_CHARGE:
		case SW5100_DISABLE_CHARGE:
		case SW5100_INHIBIT_CHARGE:
			/* Return to dynamic fuel gauge update interval. */
			val = 0;
			rc = sw5100_write(chg->pmic_regmap, QBG_MAIN_QBG_STATE_FORCE_CMD, &val, 1);
			if (rc < 0) {
				dev_err(chg->dev, "Failure to restore dynamic QBG update rc=%d\n", rc);
			}
			break;
	}

	/* Monitor soc discrepancies at charge termination. */
	if ((stat & CHGR_BATTERY_CHARGER_STATUS_MASK) == SW5100_TERMINATE_CHARGE) {
		int sys_soc = 100;

		rc = sw5100_get_prop_from_bms(chg, SW5100_QBG_REAL_CAPACITY, &sys_soc);
		if (rc != 0) {
			dev_err(chg->dev, "Failed to read system soc, rc=%d\n", rc);
		} else if (sys_soc != 100) {
			dev_err(chg->dev, "Battery full charge termination, sys_soc of %d != 100%%\n",
				sys_soc);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t sw5100_batt_temp_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct bms_dev *chg = irq_data->parent_data;

	dev_dbg(chg->dev, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->psy);
	return IRQ_HANDLED;
}

static irqreturn_t sw5100_batt_psy_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct bms_dev *chg = irq_data->parent_data;

	dev_dbg(chg->dev, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->psy);
	return IRQ_HANDLED;
}

static irqreturn_t sw5100_default_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	dev_dbg(chg->dev, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

/* TODO: sparse, consider adding .irqno */
static struct smb_irq_info sw5100_bms_irqs[] = {
	/* CHARGER IRQs */
	[CHGR_ERROR_IRQ] = {
		.name		= "chgr-error",
		.handler	= sw5100_default_irq_handler,
	},
	[CHG_STATE_CHANGE_IRQ] = {
		.name		= "chg-state-change",
		.handler	= sw5100_chg_state_change_irq_handler,
		.wake		= true,
	},
	[VPH_OV_IRQ] = {
		.name		= "vph-ov",
	},
	[BUCK_OC_IRQ] = {
		.name		= "buck-oc",
	},
	/* BATTERY IRQs */
	[BAT_TEMP_IRQ] = {
		.name		= "bat-temp",
		.handler	= sw5100_batt_temp_changed_irq_handler,
		.wake		= true,
	},
	[BAT_THERM_OR_ID_MISSING_IRQ] = {
		.name		= "bat-therm-or-id-missing",
		.handler	= sw5100_batt_psy_changed_irq_handler,
	},
	[BAT_LOW_IRQ] = {
		.name		= "bat-low",
		.handler	= sw5100_batt_psy_changed_irq_handler,
	},
	[BAT_OV_IRQ] = {
		.name		= "bat-ov",
		.handler	= sw5100_batt_psy_changed_irq_handler,
	},
	[BSM_ACTIVE_IRQ] = {
		.name		= "bsm-active",
	},
};

static int sw5100_get_irq_index_byname(const char *irq_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sw5100_bms_irqs); i++) {
		if (!sw5100_bms_irqs[i].name)
			continue;

		if (strcmp(sw5100_bms_irqs[i].name, irq_name) == 0)
			return i;
	}

	return -ENOENT;
}

static int sw5100_request_interrupt(struct bms_dev *bms,
				    struct device_node *node,
				    const char *irq_name)
{
	int rc, irq, irq_index;
	struct smb_irq_data *irq_data;

	irq = of_irq_get_byname(node, irq_name);
	if (irq < 0) {
		pr_err("Couldn't get irq %s byname\n", irq_name);
		return irq;
	}

	irq_index = sw5100_get_irq_index_byname(irq_name);
	if (irq_index < 0) {
		pr_err("%s is not a defined irq\n", irq_name);
		return irq_index;
	}

	if (!sw5100_bms_irqs[irq_index].handler)
		return 0;

	irq_data = devm_kzalloc(bms->dev, sizeof(*irq_data), GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	irq_data->parent_data = bms;
	irq_data->name = irq_name;
	irq_data->storm_data = sw5100_bms_irqs[irq_index].storm_data;
	mutex_init(&irq_data->storm_data.storm_lock);

	rc = devm_request_threaded_irq(bms->dev, irq, NULL,
					sw5100_bms_irqs[irq_index].handler,
					IRQF_ONESHOT, irq_name, irq_data);
	if (rc < 0) {
		pr_err("Couldn't request irq %d\n", irq);
		return rc;
	}

	sw5100_bms_irqs[irq_index].irq = irq;
	sw5100_bms_irqs[irq_index].irq_data = irq_data;
	if (sw5100_bms_irqs[irq_index].wake)
		enable_irq_wake(irq);

	return rc;
}

static void sw5100_free_interrupts(struct bms_dev *bms)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sw5100_bms_irqs); i++) {
		if (sw5100_bms_irqs[i].irq > 0) {
			if (sw5100_bms_irqs[i].wake)
				disable_irq_wake(sw5100_bms_irqs[i].irq);

			devm_free_irq(bms->dev, sw5100_bms_irqs[i].irq, sw5100_bms_irqs[i].irq_data);
		}
	}
}

static void sw5100_disable_interrupts(struct bms_dev *bms)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sw5100_bms_irqs); i++) {
		if (sw5100_bms_irqs[i].irq > 0)
			disable_irq(sw5100_bms_irqs[i].irq);
	}
}

static int sw5100_request_interrupts(struct bms_dev *bms)
{
	struct device_node *node = bms->dev->of_node;
	struct device_node *child;
	int rc = 0;
	const char *name;
	struct property *prop;

	for_each_available_child_of_node(node, child) {
		of_property_for_each_string(child, "interrupt-names",  prop, name) {
			rc = sw5100_request_interrupt(bms, child, name);
			if (rc < 0)
				return rc;
		}
	}
	return 0;
}

#define BID_RPULL_OHM		100000
#define BID_VREF_MV		1875
static void sw5100_get_batt_id(const struct bms_dev *bms, int *batt_id_ohm)
{
	int rc, batt_id_mv;

	/* Read battery-id */
	rc = iio_read_channel_processed(bms->batt_id_chan, &batt_id_mv);
	if (rc < 0) {
		pr_err("Failed to read BATT_ID over ADC, rc=%d\n", rc);
		return;
	}

	batt_id_mv = div_s64(batt_id_mv, 1000);
	if (batt_id_mv == 0) {
		pr_info("batt_id_mv = 0 from ADC\n");
		return;
	}

	*batt_id_ohm = (u32)batt_id_mv;

	pr_info("batt_id = %d\n", *batt_id_ohm);
}

#define QBG_MAIN_LAST_BURST_AVG_ACC2_DATA0		0xA4
#define IBATT_10A_LSB					6103
#define ICHG_FS_10A					1
#define TEN_NANO_TO_MICRO	100
static int sw5100_get_battery_current(const struct bms_dev *bms, int *val)
{
	int rc = 0;
	unsigned short acc2_data;
	u8 buf[2];
	const unsigned long addr = bms->rradc_base + QBG_MAIN_LAST_BURST_AVG_ACC2_DATA0;

	rc = sw5100_read(bms->pmic_regmap, addr, buf, 2);
	if (rc < 0) {
		pr_err("Failed to read LAST_BURST_AVG_I reg, rc=%d\n", rc);
		return rc;
	}
	acc2_data = buf[0] | (buf[1] << 8);
	*val = ((int16_t)acc2_data) * IBATT_10A_LSB * ICHG_FS_10A;
	*val = *val / TEN_NANO_TO_MICRO;

	return rc;
}

#define QBG_MAIN_LAST_BURST_AVG_ACC0_DATA0		0xA0
#define VBATT_1S_LSB					19463
static int sw5100_get_battery_voltage(const struct bms_dev *bms, int *val)
{
	int rc = 0;
	unsigned short acc0_data;
	u8 buf[2];
	const unsigned long addr = bms->rradc_base + QBG_MAIN_LAST_BURST_AVG_ACC0_DATA0;

	rc = sw5100_read(bms->pmic_regmap, addr, buf, 2);
	if (rc < 0) {
		pr_err("Failed to read LAST_ADV_V reg, rc=%d\n", rc);
		return rc;
	}
	acc0_data = buf[0] | (buf[1] << 8);
	*val = acc0_data * VBATT_1S_LSB;
	*val = *val / TEN_NANO_TO_MICRO;

	return rc;
}

static int sw5100_get_battery_temp(const struct bms_dev *bms, int *val)
{
	int rc = 0;

	if (!bms->rradc_base)
		return -EIO;

	rc = iio_read_channel_processed(bms->batt_therm_chan, val);
	if (rc < 0) {
		pr_err("Failed reading BAT_TEMP over ADC rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#define sw5100_IS_ONLINE(stat)	\
	(((stat) & (USE_DCIN_BIT | USE_USBIN_BIT)) && \
	((stat) & VALID_INPUT_POWER_SOURCE_STS_BIT))

/* charger online when connected */
static bool sw5100_is_online(const struct bms_dev *bms)
{
	u8 stat;
	const int rc = sw5100_read(bms->pmic_regmap, DCDC_POWER_PATH_STATUS_REG, &stat, 1);

	return (rc == 0) && sw5100_IS_ONLINE(stat);
}

static int sw5100_is_limited(const struct bms_dev *bms)
{
	int rc;
	u8 val;

	rc = sw5100_read(bms->pmic_regmap, DCDC_AICL_STATUS_REG, &val, 1);
	return (rc < 0) ? -EIO : ((val & DCDC_SOFT_ILIMIT_BIT) != 0);
}

static int sw5100_get_chg_type(const struct bms_dev *bms)
{
	u8 val;
	int chg_type, rc;

	rc = sw5100_read(bms->pmic_regmap, CHGR_BATTERY_CHARGER_STATUS_REG, &val, 1);
	if (rc < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	switch (val & CHGR_BATTERY_CHARGER_STATUS_MASK) {
	case SW5100_TRICKLE_CHARGE:
	case SW5100_PRE_CHARGE:
		chg_type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case SW5100_FULLON_CHARGE:
		chg_type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case SW5100_TAPER_CHARGE:
		chg_type = POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
		break;
	default:
		chg_type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return chg_type;
}

static int sw5100_get_chg_status(const struct bms_dev *bms,
				 bool *dc_valid, bool *usb_valid)
{
	bool plugged, valid;
	int rc, ret;
	int vchrg = 0;
	int vlimit = bms->chg_term_voltage;
	u8 pstat, stat1, stat2;
	u8 suspend;

	rc = sw5100_rd8(bms->pmic_regmap, DCDC_POWER_PATH_STATUS_REG, &pstat);
	if (rc < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	valid = (pstat & VALID_INPUT_POWER_SOURCE_STS_BIT);
	plugged = (pstat & USE_DCIN_BIT) || (pstat & USE_USBIN_BIT);

	*dc_valid = valid && (pstat & USE_DCIN_BIT);
	*usb_valid = valid && (pstat & USE_USBIN_BIT);

	rc = sw5100_rd8(bms->pmic_regmap, CHGR_BATTERY_CHARGER_STATUS_REG, &stat1);
	if (rc < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	rc = sw5100_rd8(bms->pmic_regmap, CHGR_BATTERY_CHARGER_EN_STATUS_REG, &stat2);
	if (rc < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	pr_debug("pmic: pstat=%x stat=%x enstat=%x\n",
		pstat, stat1, stat2);

	stat1 = stat1 & CHGR_BATTERY_CHARGER_STATUS_MASK;

	if (!plugged)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	switch (stat1) {
	case SW5100_TRICKLE_CHARGE:
	case SW5100_PRE_CHARGE:
	case SW5100_FULLON_CHARGE:
	case SW5100_TAPER_CHARGE:
		ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	/* pause on FCC=0, JEITA, USB/DC suspend or on INPUT UV/OV */
	case SW5100_PAUSE_CHARGE:
	case SW5100_INHIBIT_CHARGE:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case SW5100_TERMINATE_CHARGE:
		/* flag full only at the correct voltage */
		rc = sw5100_get_battery_voltage(bms, &vchrg);
		if (rc == 0)
			vchrg = (vchrg / 1000);
		if (stat1 == SW5100_TERMINATE_CHARGE)
			vlimit -= bms->chg_term_voltage_debounce;
		if (vchrg < vlimit)
			ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			ret = POWER_SUPPLY_STATUS_FULL;
		break;
	/* disabled disconnect */
	case SW5100_DISABLE_CHARGE:
		rc = sw5100_rd8(bms->pmic_regmap, CHGR_USB_SUSPEND, &suspend);
		if (rc < 0)
			return POWER_SUPPLY_STATUS_UNKNOWN;
		if ((suspend & USBIN_SUSPEND) == USBIN_SUSPEND) {
			ret = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	if (ret != POWER_SUPPLY_STATUS_CHARGING)
		return ret;

	if (valid) {
		u8 stat;

		rc = sw5100_rd8(bms->pmic_regmap,
				CHGR_BATTERY_CHARGER_EN_STATUS_REG,
				&stat);
		if (rc < 0)
			return POWER_SUPPLY_STATUS_UNKNOWN;

		stat &= ENABLE_TRICKLE_BIT | ENABLE_PRE_CHARGING_BIT |
					ENABLE_FULLON_MODE_BIT;
		if (stat)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}
static int sw5100_get_chg_chgr_state(const struct bms_dev *bms,
			  union gbms_charger_state *chg_state)
{
	int vchrg, rc;
	bool usb_valid, dc_valid;
	u8 icl = 0;

	chg_state->v = 0;
	chg_state->f.chg_status = sw5100_get_chg_status(bms, &dc_valid, &usb_valid);
	chg_state->f.chg_type = sw5100_get_chg_type(bms);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status, chg_state->f.chg_type);

	rc = sw5100_is_limited(bms);
	if (rc > 0)
		chg_state->f.flags |= GBMS_CS_FLAG_ILIM;

	rc = sw5100_get_battery_voltage(bms, &vchrg);
	if (rc == 0)
		chg_state->f.vchrg = (vchrg / 1000);

	if (usb_valid) {
		(void)sw5100_rd8(bms->pmic_regmap, DCDC_ICL_STATUS_REG, &icl);
	}

	chg_state->f.icl = (icl * 100);

	pr_info("MSC_PCS chg_state=%lx [0x%x:%d:%d:%d:%d] chg=%c\n",
		(unsigned long)chg_state->v,
		chg_state->f.flags,
		chg_state->f.chg_type,
		chg_state->f.chg_status,
		chg_state->f.vchrg,
		chg_state->f.icl,
		usb_valid ? 'u' : dc_valid ? 'w' : ' ');

	return 0;
}

static int sw5100_get_batt_health(struct bms_dev *bms)
{
	int vchrg, effective_fv_uv, rc, ret;
	u8 stat;

	rc = sw5100_rd8(bms->pmic_regmap, CHGR_BATTERY_CHARGER_STATUS_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read CHGR_BATTERY_CHARGER_STATUS_REG rc=%d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & CHG_ERR_STATUS_BAT_OV) {
		rc = sw5100_get_battery_voltage(bms, &vchrg);
		if (rc == 0) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			if (!bms->fv_votable)
				bms->fv_votable = find_votable(VOTABLE_MSC_FV);
			if (bms->fv_votable) {
				effective_fv_uv = get_effective_result(bms->fv_votable);
				if (vchrg >= effective_fv_uv + 40000) {
					ret = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
					pr_err("battery over-voltage vbat_fg = %duV, fv = %duV\n",
							vchrg, effective_fv_uv);
					goto done;
				}
			}
		}
	}

	rc = sw5100_rd8(bms->pmic_regmap, BATIF_BAT_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read BATIF_BAT_TEMP_STATUS_REG rc=%d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	if (stat & BAT_TEMP_TOO_COLD)
		ret = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_TOO_HOT)
		ret = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_COOL)
		ret = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_WARM)
		ret = POWER_SUPPLY_HEALTH_WARM;
	else
		ret = POWER_SUPPLY_HEALTH_GOOD;

done:
	return ret;
}

static int sw5100_get_batt_iterm(struct bms_dev *bms)
{
	int rc, temp;
	u8 stat, buf[2];

	rc = sw5100_rd8(bms->pmic_regmap, CHGR_CHG_TERM_CFG_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read CHGR_CHG_TERM_CFG_REG rc=%d\n", rc);
		return rc;
	}

	if (stat & CHGR_ITERM_USE_ANALOG_BIT)
		return -EINVAL;

	rc = sw5100_read(bms->pmic_regmap, CHGR_ADC_ITERM_UP_THD_MSB, buf, 2);
	if (rc < 0) {
		pr_err("Couldn't read CHGR_ADC_ITERM_UP_THD_MSB rc=%d\n", rc);
		return rc;
	}

	temp = buf[1] | (buf[0] << 8);
	temp = sign_extend32(temp, 15);

	temp = DIV_ROUND_CLOSEST(temp * 1000, PM5100_ADC_CHG_ITERM_MULT);

	return temp;
}

static int sw5100_get_batt_present(struct bms_dev *bms)
{
	int rc, ret;
	u8 stat;

	rc = sw5100_rd8(bms->pmic_regmap,
			BATIF_INT_RT_STS,
			&stat);
	if (rc < 0) {
		pr_err("Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	ret = !(stat & (BATIF_THERM_OR_ID_MISSING_RT_STS_BIT));
	return ret;
}

/** Given a SOC percentage aka capacity we're going to scale 5-100 to 0-100. */
static int scale_capacity(struct bms_dev const *bms, int capacity)
{
	if ((bms->soc_shutdown_offset > 0) && (bms->soc_shutdown_offset < 100)) {
		if (capacity >= 100) {
			return 100;
		} else if (capacity >= bms->soc_shutdown_offset) {
			return DIV_ROUND_UP(((capacity - bms->soc_shutdown_offset) * 100),
				(100 - bms->soc_shutdown_offset));
		} else {
			return 0;
		}
	} else {
		return capacity;
	}
}

static int sw5100_psy_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *pval)
{
	struct bms_dev *bms = (struct bms_dev *)power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	u8 val;
	int ivalue = 0;
	int rc = 0;
	bool usb_valid, dc_valid;

	if (!bms->psy) {
		pr_err("failed to register power supply\n");
		return -EAGAIN;
	}

	switch ((int) psp) {
	/*
	 * called from power_supply_update_leds(), not using it on this
	 * platform. Could return the state of the charge buck (BUCKEN)
	 */
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = sw5100_is_online(bms);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = sw5100_get_chg_status(bms, &dc_valid, &usb_valid);
		break;
	/* pixel battery management subsystem */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		/*
		 * CHGR_FAST_CHARGE_CURRENT_SETTING, 0x2654
		 * 7 : 0 => FAST_CHARGE_CURRENT_SETTING:
		 * Fast Charge Current = DATA x 25mA
		 */
		rc = sw5100_read(bms->pmic_regmap, CHGR_FAST_CHARGE_CURRENT_SETTING, &val, 1);
		if (rc == 0)
			pval->intval = val * CHGR_CHARGE_CURRENT_STEP;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		/*
		 * CHGR_FLOAT_VOLTAGE_SETTING  0x2658
		 * 7 : 0 => FLOAT_VOLTAGE_SETTING:
		 * Float voltage setting = 3.6V + (DATA x 10mV)
		 */
		rc = sw5100_read(bms->pmic_regmap, CHGR_FLOAT_VOLTAGE_SETTING, &val, 1);
		if (rc == 0)
			pval->intval = val * 10000 + CHGR_FLOAT_VOLTAGE_BASE;
		break;
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		rc = sw5100_get_chg_chgr_state(bms, &chg_state);
		if (rc == 0)
			gbms_propval_int64val(pval) = chg_state.v;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		pval->intval = sw5100_get_chg_type(bms);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = sw5100_get_battery_current(bms, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
		rc = sw5100_is_limited(bms);
		if (rc < 0)
			break;
		pval->intval = (rc > 0);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		rc = sw5100_get_battery_temp(bms, &ivalue);
		if (rc < 0)
			break;
		pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		/*
		 * CHGR_FLOAT_VOLTAGE_NOW 0x260B
		 * 7 : 0 => FLOAT_VOLTAGE:
		 * Float voltage after JEITA compensation
		 */
		rc = sw5100_read(bms->pmic_regmap, CHGR_FLOAT_VOLTAGE_NOW,
				&val, 1);
		if (rc == 0)
			pval->intval = val * 10000 + CHGR_FLOAT_VOLTAGE_BASE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = sw5100_get_battery_voltage(bms, &ivalue);
		if (!rc)
			pval->intval = ivalue;
		break;
	case GBMS_PROP_CHARGE_DISABLE:
		rc = sw5100_read(bms->pmic_regmap, CHGR_CHG_EN, &val, 1);
		if (rc == 0)
			pval->intval = !(val & CHARGING_ENABLE_CMD_BIT);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		pval->intval = sw5100_get_batt_health(bms);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		pval->intval = sw5100_get_batt_iterm(bms);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = sw5100_get_batt_present(bms);
		break;
	case GBMS_PROP_CAPACITY_RAW:
		// First query for monotonic SOC value.
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CAPACITY, &ivalue);
		if (rc == 0) {
			// By default we use the monotonic SOC; this prevents
			// any erroneous 0 SOC values due to any of the
			// following calls failing.
			pval->intval = (scale_capacity(bms, ivalue) << 8);
			if (ivalue == 100) {
				// monotonic SOC is 100%, now check if
				// battery offline and idling
				rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CURRENT_NOW, &ivalue);
				if (rc == 0 && ivalue == 0) {
					// use sys_soc as our SOC for
					// recharge tracking.
					rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_REAL_CAPACITY, &ivalue);
					if (rc == 0) {
						pval->intval = (scale_capacity(bms, ivalue) << 8);
					}
				}
			}
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CAPACITY, &ivalue);
		if (rc == 0)
			pval->intval = scale_capacity(bms, ivalue);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CYCLE_COUNT, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		/* TODO(b/243407602): fix cycle count, but for now set it to 0 as a workaround */
		if (pval->intval < 0)
			pval->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_VOLTAGE_AVG, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_VOLTAGE_OCV, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CHARGE_FULL, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CHARGE_FULL_DESIGN, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_CHARGE_COUNTER, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_TIME_TO_FULL_NOW, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		rc = sw5100_get_prop_from_bms(bms, SW5100_QBG_TIME_TO_EMPTY_AVG, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		rc = sw5100_get_battery_current(bms, &ivalue);
		if (rc == 0)
			pval->intval = ivalue;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		pval->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		pval->strval = "";
		break;
	case GBMS_PROP_HEALTH_ACT_IMPEDANCE:
		pval->intval = -EINVAL;
		break;
	case GBMS_PROP_RESISTANCE:
		pval->intval = 0;
		break;
	default:
		pr_debug("getting unsupported property: %d\n", psp);
		return -EINVAL;
	}

	if (rc < 0)
		return -ENODATA;

	return 0;
}

static int sw5100_usbin_disable(struct bms_dev *bms, bool disable)
{
	int rc;

	if (!bms->icl_votable) {
		bms->icl_votable = find_votable("USB_ICL");
		if (bms->icl_votable == NULL) {
			pr_err("USBIN_DISABLE: disable failed\n");
			return -EINVAL;
		}
	}

	rc = vote(bms->icl_votable, USBIN_DISABLE_VOTER, disable, 0);

	pr_debug("USBIN_DISABLE : disable=%d, rc=%d)\n", disable, rc);

	if (rc > 0) {
		/* vote returns positive number on success */
		rc = 0;
	}
	return rc;
}

static int sw5100_charge_disable(struct bms_dev *bms, bool disable)
{
	const u8 val = disable ? 0 : CHARGING_ENABLE_CMD_BIT;
	int rc;

	rc = sw5100_masked_write(bms->pmic_regmap, CHGR_CHG_EN, CHARGING_ENABLE_CMD_BIT, val);

	if (!disable) {
		/* Make sure charging is restarted by toggling usbin */
		if (rc == 0)
			rc = sw5100_usbin_disable(bms, true);
		if (rc == 0)
			rc = sw5100_usbin_disable(bms, false);

	}
	pr_debug("CHARGE_DISABLE : disable=%d -> val=%d (%d)\n", disable, val, rc);

	return rc;
}

static int sw5100_charge_pause(struct bms_dev *bms, bool pause)
{
	const u8 val = pause ? CHARGING_PAUSE_CMD_BIT : 0;
	int rc;

	rc = sw5100_masked_write(bms->pmic_regmap, CHGR_CHARGING_PAUSE_CMD, CHARGING_PAUSE_CMD_BIT, val);

	pr_debug("CHARGE_PAUSE : pause=%d -> val=%d (%d)\n",
		pause, val, rc);

	return rc;
}

static ssize_t soc_shutdown_offset_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct bms_dev *bms = power_supply_get_drvdata(psy);

	sscanf(buf, "%d", &bms->soc_shutdown_offset);
	return count;
}

static ssize_t soc_shutdown_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct bms_dev *bms = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bms->soc_shutdown_offset);
}

static const DEVICE_ATTR_RW(soc_shutdown_offset);

static int sw5100_psy_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *pval)
{
	struct bms_dev *bms = (struct bms_dev *)power_supply_get_drvdata(psy);
	u8 val;
	int ivalue = 0;
	int rc = 0;

	switch ((int) psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		/*
		 * CHGR_FAST_CHARGE_CURRENT_SETTING, 0x2654
		 * 7 : 0 => FAST_CHARGE_CURRENT_SETTING:
		 * Fast Charge Current = DATA x 25mA
		 */
		ivalue = pval->intval;
		if (ivalue < CHGR_CHARGE_CURRENT_STEP)
			val = 0;
		else
			val = ivalue / CHGR_CHARGE_CURRENT_STEP;

		if (ivalue == 0)
			rc = sw5100_charge_pause(bms, true);

		rc = sw5100_write(bms->pmic_regmap, CHGR_FAST_CHARGE_CURRENT_SETTING, &val, 1);

		if (ivalue != 0) {
			u8 paused;

			rc = sw5100_read(bms->pmic_regmap, CHGR_CHARGING_PAUSE_CMD, &paused, 1);
			if (rc == 0 && (paused & CHARGING_PAUSE_CMD_BIT)) {
				rc = sw5100_charge_pause(bms, false);

				/* make sure charging restart */
				if (rc == 0)
					rc = sw5100_charge_disable(bms, true);
				if (rc == 0)
					rc = sw5100_charge_disable(bms, false);
				if (rc < 0)
					pr_err("Failed to toggle charging during charging restart\n");
			}
		}

		pr_debug("CONSTANT_CHARGE_CURRENT_MAX : ivalue=%d, val=%d pause=%d (%d)\n",
			ivalue, val, ivalue == 0, rc);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		/*
		 * CHGR_FLOAT_VOLTAGE_SETTING  0x2658
		 * 7 : 0 => FLOAT_VOLTAGE_SETTING:
		 * Float voltage setting = 3.6V + (DATA x 10mV)
		 */
		ivalue = pval->intval;
		if (ivalue < CHGR_FLOAT_VOLTAGE_BASE) {
			pr_err("CONSTANT_CHARGE_VOLTAGE_MAX : %d (ivalue) < %d (base). Ignoring\n",
				ivalue, CHGR_FLOAT_VOLTAGE_BASE);
			rc = -EINVAL;
			break;
		}
		else
			val = (ivalue - CHGR_FLOAT_VOLTAGE_BASE) / 10000;

		rc = sw5100_write(bms->pmic_regmap, CHGR_FLOAT_VOLTAGE_SETTING, &val, 1);
		pr_debug("CONSTANT_CHARGE_VOLTAGE_MAX : ivalue=%d, val=%d (%d)\n",
							ivalue, val, rc);
		break;
	case GBMS_PROP_CHARGE_DISABLE:
		if (!bms->fcc_votable)
			bms->fcc_votable = find_votable(VOTABLE_MSC_FCC);
		if (bms->fcc_votable)
			vote(bms->fcc_votable, CHARGE_DISABLE_VOTER,
			     pval->intval, 0);
		rc = sw5100_charge_disable(bms, pval->intval != 0);
		break;
	default:
		pr_debug("setting unsupported property: %d\n", psp);
		break;
	}

	return rc;
}

static int sw5100_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch ((int) psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property sw5100_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	/* pixel battery management subsystem */
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,		/* compat */
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static struct power_supply_desc sw5100_psy_desc = {
	.name = "sw5100_bms",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = sw5100_psy_props,
	.num_properties = ARRAY_SIZE(sw5100_psy_props),
	.get_property = sw5100_psy_get_property,
	.set_property = sw5100_psy_set_property,
	.property_is_writeable = sw5100_property_is_writeable,
};

/* All callback functions below */

static int sw5100_notifier_cb(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;
	/* TBD: notification?*/
	return NOTIFY_OK;
}

static int sw5100_dc_suspend_vote_callback(struct votable *votable, void *data,
					     int disable, const char *client)
{
	/* DC suspend isn't supported but function is needed for compatibility */

	return 0;
}

/* All init functions below this */
#define PERPH_TYPE_REG				0x04
#define QG_TYPE					0x0D
static int sw5100_parse_dt_fg(struct bms_dev *bms, struct device_node *node)
{

	int rc = 0;
	u32 val;

	rc = of_property_read_u32(node, "reg", &val);
	if (rc < 0) {
		pr_err("Failed to get base address for QBG, rc = %d\n", rc);
		return rc;
	}
	bms->rradc_base = val;
	return rc;
}

static int sw5100_parse_dt(struct bms_dev *bms)
{
	struct device_node *fg_node, *node = bms->dev->of_node;
	const char *psy_name = NULL;
	int ret;

	if (!node)  {
		pr_err("device tree node missing\n");
		return -ENXIO;
	}

	fg_node = of_get_parent(node);
	if (fg_node)
		fg_node = of_get_child_by_name(fg_node, "qpnp,qbg");
	if (fg_node)
		sw5100_parse_dt_fg(bms, fg_node);
	else
		pr_err("cannot find qpnp,qbg, rradc not available\n");

	ret = of_property_read_u32(node, "google,chg-term-voltage", &bms->chg_term_voltage);
	if (ret < 0)
		bms->chg_term_voltage = CHG_TERM_VOLTAGE;

	ret = of_property_read_u32(node, "google,chg-term-voltage-debounce", &bms->chg_term_voltage_debounce);
	if (ret < 0)
		bms->chg_term_voltage_debounce = CHG_TERM_VOLT_DEBOUNCE;

	ret = of_property_read_string(node, "google,psy-name", &psy_name);
	if (ret == 0)
		sw5100_psy_desc.name =
			devm_kstrdup(bms->dev, psy_name, GFP_KERNEL);

	ret = of_property_read_u32(node, "google,soc_shutdown_offset", &bms->soc_shutdown_offset);
	if (ret < 0)
		bms->soc_shutdown_offset = 0;

	if (sw5100_psy_desc.name == NULL)
		return -EINVAL;

	return 0;
}

static int bms_probe(struct platform_device *pdev)
{
	struct bms_dev *bms;
	struct power_supply_config bms_psy_cfg = {};
	struct iio_channel **iio_list;
	int rc = 0;

	bms = devm_kzalloc(&pdev->dev, sizeof(*bms), GFP_KERNEL);
	if (!bms) {
		pr_info("kalloc error\n");
		return -ENOMEM;
	}

	bms->dev = &pdev->dev;
	bms->batt_id_ohms = -EINVAL;
	bms->pmic_regmap = dev_get_regmap(bms->dev->parent, NULL);
	if (!bms->pmic_regmap) {
		pr_err("Parent regmap is unavailable\n");
	} else {
		/* ADC for BID & THERM */
		bms->batt_id_chan = iio_channel_get(&pdev->dev, "batt-id");
		if (IS_ERR(bms->batt_id_chan)) {
			rc = PTR_ERR(bms->batt_id_chan);
			if (rc != -EPROBE_DEFER)
				pr_err("batt-id channel unavailable, rc=%d\n", rc);
			bms->batt_id_chan = NULL;
			return rc;
		}

		bms->batt_therm_chan = iio_channel_get(&pdev->dev, "batt-therm");
		if (IS_ERR(bms->batt_therm_chan)) {
			rc = PTR_ERR(bms->batt_therm_chan);
			if (rc != -EPROBE_DEFER)
				pr_err("batt-therm channel unavailable, rc=%d\n", rc);
			bms->batt_therm_chan = NULL;
			return rc;
		}

		sw5100_get_batt_id(bms, &bms->batt_id_ohms);

		/*
		 * If AICL collapses, do not "latch-off" (i.e. do not require
		 * that a user take the device off-charger and place it back
		 * on-charger in order to attempt charging again).
		 */
		rc = sw5100_masked_write(bms->pmic_regmap, CHGR_USB_SUSPEND,
					SUSPEND_ON_COLLAPSE_USBIN, 0);
		if (rc != 0) {
			dev_err(bms->dev,
				"Could not disable USBIN latch-off, rc=%d\n", rc);
		}
	}

	rc = sw5100_parse_dt(bms);
	if (rc < 0) {
		pr_err("Parse the device tree fail. rc = %d\n", rc);
		goto exit;
	}

	/* Register the power supply */
	bms_psy_cfg.drv_data = bms;
	bms_psy_cfg.of_node = bms->dev->of_node;
	bms_psy_cfg.supplied_to = NULL;
	bms_psy_cfg.num_supplicants = 0;
	bms->psy = devm_power_supply_register(bms->dev, &sw5100_psy_desc,
			&bms_psy_cfg);
	if (IS_ERR(bms->psy)) {
		pr_err("failed to register psy rc = %ld\n", PTR_ERR(bms->psy));
		goto exit;
	}

	rc = device_create_file(&bms->psy->dev, &dev_attr_soc_shutdown_offset);
	if (rc < 0)
		dev_err(&bms->psy->dev, "Failed to create soc scaling offset for shutdown\n");

	iio_list = sw5100_get_ext_channels(bms->dev, sw5100_qbg_ext_iio_chan,
		ARRAY_SIZE(sw5100_qbg_ext_iio_chan));
	if (!IS_ERR(iio_list))
		bms->iio_chan_list_qg = iio_list;


	bms->nb.notifier_call = sw5100_notifier_cb;
	rc = power_supply_reg_notifier(&bms->nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		goto exit;
	}

	rc = sw5100_request_interrupts(bms);
	if (rc < 0) {
		pr_err("Couldn't register the interrupts rc = %d\n", rc);
		goto exit;
	}

	bms->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					sw5100_dc_suspend_vote_callback,
					bms);
	if (IS_ERR(bms->dc_suspend_votable)) {
		rc = PTR_ERR(bms->dc_suspend_votable);
		bms->dc_suspend_votable = NULL;
		return rc;
	}

	pr_info("SW5100 BMS driver probed successfully\n");

	return 0;
exit:
	return rc;
}

static int bms_remove(struct platform_device *pdev)
{
	struct bms_dev *bms = platform_get_drvdata(pdev);

	sw5100_free_interrupts(bms);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void bms_shutdown(struct platform_device *pdev)
{
	struct bms_dev *bms = platform_get_drvdata(pdev);

	/* disable all interrupts */
	sw5100_disable_interrupts(bms);
}

static const struct of_device_id bms_of_match[] = {
	{.compatible = "google,sw5100_bms"},
	{},
};

static struct platform_driver sw5100_bms_driver = {
	.driver = {
		.name = BMS_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bms_of_match,
	},
	.probe		= bms_probe,
	.remove		= bms_remove,
	.shutdown	= bms_shutdown,
};

module_platform_driver(sw5100_bms_driver);
MODULE_DESCRIPTION("sw5100 BMS driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" BMS_DEV_NAME);
MODULE_AUTHOR("Alice Sheng <alicesheng@google.com>");
