/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform data for the NXP WC68 battery charger driver.
 */

#ifndef _WC68_CHARGER_H_
#define _WC68_CHARGER_H_

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>

/* Google integration */
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"

struct wc68_platform_data {
	s32		irq_gpio;	/* GPIO pin that's connected to INT# */
	u32		iin_cfg;	/* Input Current Limit - uA unit */
	u32		iin_cfg_max;	/* from config/dt */
	u32		v_float;	/* V_Float Voltage - uV unit */
	u32		v_float_dt;	/* from config/dt */
	u32		iin_topoff;	/* Input Topoff current -uV unit */
	s32		iin_max_offset;
	s32		iin_cc_comp_offset;
	u32		ta_max_vol;
	u32		ta_max_vol_cp;

	/* irdrop */
	s32		irdrop_limits[3];
	s32		irdrop_limit_cnt;
	u8		wc68_irdrop;

#if IS_ENABLED(CONFIG_THERMAL)
	const char *usb_tz_name;
#endif
};

/* - PPS Integration Shared definitions ---------------------------------- */

/* AC[0] */
#define WC68_CHGS_VER		1
#define WC68_CHGS_VER_MASK	0xff
/* AC[1] APDO */
/* RS[0] */
#define WC68_CHGS_FLAG_SHIFT	0
#define WC68_CHGS_FLAG_MASK	0xff
#define WC68_CHGS_F_STBY	BIT(0)
#define WC68_CHGS_F_SHDN	BIT(1)
#define WC68_CHGS_F_DONE	BIT(2)
#define WC68_CHGS_PRE_SHIFT	8
#define WC68_CHGS_PRE_MASK	(0xff << WC68_CHGS_PRE_SHIFT)
#define WC68_CHGS_RCPC_SHIFT	16
#define WC68_CHGS_RCPC_MASK	(0xff << WC68_CHGS_RCPC_SHIFT)
#define WC68_CHGS_NC_SHIFT	24
#define WC68_CHGS_NC_MASK	(0xff << WC68_CHGS_NC_SHIFT)
/* RS[1] */
#define WC68_CHGS_OVCC_SHIFT	0
#define WC68_CHGS_OVCC_MASK	(0xffff << WC68_CHGS_OVCC_SHIFT)
#define WC68_CHGS_ADJ_SHIFT	16
#define WC68_CHGS_ADJ_MASK	(0xffff << WC68_CHGS_ADJ_MASK)
/* RS[2] */
#define WC68_CHGS_CC_SHIFT	0
#define WC68_CHGS_CC_MASK	(0xffff << WC68_CHGS_CC_SHIFT)
#define WC68_CHGS_CV_SHIFT	16
#define WC68_CHGS_CV_MASK	(0xffff << WC68_CHGS_CV_SHIFT)
/* RS[3] */
#define WC68_CHGS_CA_SHIFT	0
#define WC68_CHGS_CA_MASK	(0xff << WC68_CHGS_CA_SHIFT)


struct wc68_chg_stats {
	u32 adapter_capabilities[2];
	u32 receiver_state[5];

	u8 valid;
	u32 ovc_count;
	u32 ovc_max_ibatt;
	u32 ovc_max_delta;

	u32 rcp_count;
	u32 nc_count;
	u32 pre_count;
	u32 ca_count;
	u32 cc_count;
	u32 cv_count;
	u32 adj_count;
	u32 stby_count;
};

#define wc68_chg_stats_valid(chg_data) ((chg_data)->valid)

static inline void wc68_chg_stats_update_flags(struct wc68_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] |= flags << WC68_CHGS_FLAG_SHIFT;
}

static inline void wc68_chg_stats_set_flags(struct wc68_chg_stats *chg_data, u8 flags)
{
	chg_data->receiver_state[0] &= ~WC68_CHGS_FLAG_MASK;
	wc68_chg_stats_update_flags(chg_data, flags);
}

static inline void wc68_chg_stats_inc_ovcf(struct wc68_chg_stats *chg_data,
					    s32 ibatt, s32 cc_max)
{
	const s32 delta = ibatt - cc_max;

	chg_data->ovc_count++;
	if (delta > chg_data->ovc_max_delta) {
		chg_data->ovc_max_ibatt = ibatt;
		chg_data->ovc_max_delta = delta;
	}
}

/**
 * struct wc68_charger - wc68 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @dc_wq: work queue for the algorithm and monitor timer
 * @timer_work: timer work for charging
 * @timer_id: timer id for timer_work
 * @timer_period: timer period for timer_work
 * @last_update_time: last update time after sleep
 * @pps_index: psy index
 * @tcpm_psy_name: name of TCPM power supply
 * @tcpm_phandle: lookup for tcpm power supply
 * @pps_work: pps work for PPS periodic time
 * @pps_data: internal data for dc_pps
 * @log: logbuffer
 * @pd: phandle for qualcomm PMI usbpd-phy
 * @wlc_psy_name: power supply for wlc DC
 * @wlc_psy: wlc DC ps
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @ret_state: return direct charging state after DC_STATE_ADJUST_TAVOL is done
 * @iin_cc: input current for the direct charging in cc mode, uA
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @ta_max_cur: TA maximum current of APDO, uA
 * @ta_max_vol: TA maximum voltage for the direct charging, uV
 * @ta_max_pwr: TA maximum power, uW
 * @prev_iin: Previous IIN ADC of WC68, uA
 * @prev_inc: Previous TA voltage or current increment factor
 * @fv_uv: requested float voltage
 * @cc_max: requested charge current max
 * @new_iin: New request input current limit, uA
 * @new_vfloat: Request for new vfloat
 * @adc_comp_gain: adc gain for compensation
 * @retry_cnt: retry counter for re-starting charging if charging stop happens
 * @ta_type: TA type for the direct charging, USBPD TA or Wireless Charger.
 * @chg_mode: supported DC charging mode 2:1 or 4:1 mode
 * @pdata: pointer to platform data
 * @usb_tzd: device for thermal zone
 * @debug_root: debug entry
 * @debug_address: debug register address
 * @debug_adc_channel: ADC channel to read
 * @init_done: true when initialization is complete
 * @dc_start_time: start time (sec since boot) of the DC session
 */
struct wc68_charger {
	struct wakeup_source	*monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;

	struct workqueue_struct	*dc_wq;
	struct delayed_work	timer_work;
	u32		timer_id;
	unsigned long		timer_period;
	unsigned long		last_update_time;

	bool			mains_online;
	u32 			charging_state;
	u32			ret_state;

	u32			iin_cc;

	u32			ta_cur;
	u32			ta_vol;
	u32			ta_objpos;

	/* same as pps_data */
	u32			ta_max_cur;
	u32			ta_max_vol;
	unsigned long		ta_max_pwr;

	u32			prev_iin;
	u32			prev_inc;

	u32			new_iin;
	s32 			new_vfloat;

	s32			adc_comp_gain;

	s32			retry_cnt;

	struct wc68_platform_data *pdata;

	/* Google Integration Start */
	s32 			pps_index;		/* 0=disabled, 1=tcpm, 2=wireless */
	bool			init_done;
	bool			hw_init_done;

	/* PPS_wireless */
	const char 		*wlc_psy_name;
	struct power_supply 	*wlc_psy;
	/*  PPS_wired with TCPM */
	u32			tcpm_phandle;
	const char 		*tcpm_psy_name;
	struct power_supply 	*pd;
	struct delayed_work	pps_work;
	struct pd_pps_data	pps_data;
	struct logbuffer	*log;

#if IS_ENABLED(CONFIG_THERMAL)
	struct thermal_zone_device *usb_tzd;
#endif

	/* WIRELESS or WIRED */
	s32			ta_type;
	/*
	 *	0 - No direct charging
	 *	1 - 2:1 charging mode
	 *	2 - 4:1 charging mode
	 */
	s32			chg_mode;

	/* requested charging current and voltage */
	s32			fv_uv;
	s32			cc_max;
	ktime_t			dc_start_time;

	/* monitoring */
	struct power_supply	*batt_psy;

	/* debug */
	struct dentry		*debug_root;
	u32			debug_address;
	s32			debug_adc_channel;


	bool 			wlc_ramp_out_iin;
	u32 			wlc_ramp_out_delay;
	u32 			wlc_ramp_out_vout_target;

	struct wc68_chg_stats	chg_data;
	struct gvotable_election *dc_avail;

	u32 			debug_count;
	struct i2c_client 	*client;
	struct attribute_group 	attrs;    /* SysFS attributes */
	struct delayed_work 	init_hw_work;
	/* Google Integration END */
};

/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,	/* No charging */
	DC_STATE_CHECK_VBAT,	/* Check min battery level */
	DC_STATE_PRESET_DC, 	/* Preset TA voltage/current for DC */
	DC_STATE_CHECK_ACTIVE,	/* Check active status before Adjust CC mode */
	DC_STATE_ADJUST_CC,	/* Adjust CC mode */
	DC_STATE_CC_MODE,	/* Check CC mode status */
	DC_STATE_START_CV,	/* Start CV mode */
	DC_STATE_CV_MODE,	/* Check CV mode status */
	DC_STATE_CHARGING_DONE,	/* Charging Done */
	DC_STATE_ADJUST_TAVOL,	/* Adjust TA voltage, new TA current < 1000mA */
	DC_STATE_ADJUST_TACUR,	/* Adjust TA current, new TA current < 1000mA */
	DC_STATE_MAX,
};

/* PD Message Type */
enum {
	PD_MSG_REQUEST_APDO,
	MSG_REQUEST_FIXED_PDO,
	WCRX_REQUEST_VOLTAGE,
};

/* TA Type for the direct charging */
enum {
	TA_TYPE_UNKNOWN,
	TA_TYPE_USBPD,
	TA_TYPE_WIRELESS,
};

/* Direct Charging Mode for the direct charging */
enum {
	CHG_NO_DC_MODE,
	CHG_2TO1_DC_MODE,
	CHG_4TO1_DC_MODE,
};

/* PPS timers */
#define WC68_PDMSG_WAIT_T		250	/* 250ms */
#define WC68_PDMSG_RETRY_T		1000	/* 1000ms */
#define WC68_PDMSG_WLC_WAIT_T	5000	/* 5000ms */
#define WC68_PPS_PERIODIC_T		10000	/* 10000ms */

/* - Core driver  ---------------------------- */

s32 wc68_read_adc(struct wc68_charger *wc68, u8 adc_ch);
s32 wc68_input_current_limit(struct wc68_charger *wc68);

/* - PPS Integration (move to a separate file) ---------------------------- */

/* */
enum {
	PPS_INDEX_DISABLED = 0,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC,
	PPS_INDEX_MAX,
};

s32 wc68_probe_pps(struct wc68_charger *wc68_chg);

s32 wc68_request_pdo(struct wc68_charger *wc68);
s32 wc68_usbpd_setup(struct wc68_charger *wc68);
s32 wc68_send_pd_message(struct wc68_charger *wc68, u32 msg_type);
s32 wc68_get_apdo_max_power(struct wc68_charger *wc68,
			    u32 ta_max_vol, u32 ta_max_cur);
s32 wc68_send_rx_voltage(struct wc68_charger *wc68, u32 msg_type);
s32 wc68_get_rx_max_power(struct wc68_charger *wc68);
s32 wc68_set_ta_type(struct wc68_charger *wc68, s32 pps_index);

/* GBMS integration */
struct power_supply *wc68_get_rx_psy(struct wc68_charger *wc68);
s32 wc68_get_chg_chgr_state(struct wc68_charger *wc68,
			    union gbms_charger_state *chg_state);
s32 wc68_is_present(struct wc68_charger *wc68);
s32 wc68_get_status(struct wc68_charger *wc68);
s32 wc68_get_charge_type(struct wc68_charger *wc68);

extern s32 debug_printk_prlog;
extern s32 debug_no_logbuffer;

#define logbuffer_prlog(p, level, fmt, ...)	\
    gbms_logbuffer_prlog(p->log, level, debug_no_logbuffer, debug_printk_prlog, fmt, ##__VA_ARGS__)

/* charge stats */
void wc68_chg_stats_init(struct wc68_chg_stats *chg_data);
s32 wc68_chg_stats_update(struct wc68_chg_stats *chg_data,
			  const struct wc68_charger *wc68);
s32 wc68_chg_stats_done(struct wc68_chg_stats *chg_data,
			const struct wc68_charger *wc68);
void wc68_chg_stats_dump(const struct wc68_charger *wc68);
s32 wc68_check_standby(struct wc68_charger *wc68);
s32 wc68_hw_ping(struct wc68_charger *wc68);

#endif