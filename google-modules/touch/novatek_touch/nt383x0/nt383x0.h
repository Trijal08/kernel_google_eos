/*
 * Copyright (C) 2020 Novatek, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/debugfs.h>

#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
#include <drm/drm_panel.h>
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
#include <linux/soc/qcom/panel_event_notifier.h>
#endif
#endif
#elif defined(CONFIG_DRM_MSM)
#include <linux/msm_drm_notify.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nt383x0_mem_map.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
#define HAVE_PROC_OPS
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
#define HAVE_SET_FS
#endif

#define NVT_DEBUG 0
#define CONFIG_OF 1

//---GPIO number---
#define NVTTOUCH_RST_PIN 980
#define NVTTOUCH_INT_PIN 943


//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING


//---I2C driver info.---
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_I2C_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"


//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 416
#define TOUCH_DEFAULT_MAX_HEIGHT 416
#define TOUCH_MAX_FINGER_NUM 2
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000
#define TOUCH_RECOVER_COOLDOWN_JIFFIES msecs_to_jiffies(60000) // 1 min
#define TOUCH_DEFAULT_BASELINE_THRESHOLD 1200

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 0

//---Customerized func.---
#define GOOGLE_TOUCH_EXT_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_EXT_PROC_INIT 0
#define NVT_TOUCH_EXT_SYSFS 1
#define NVT_TOUCH_MP 1
#define NVT_SAVE_TEST_DATA_IN_FILE 0
#define NVT_TOUCH_MP_PROC_INIT 0
#define NVT_TOUCH_MP_SYSFS 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1
#define PALM_GESTURE 1
/* customized gesture id */
#define DATA_PROTOCOL           30
/* function page definition */
#define FUNCPAGE_GESTURE         1
#define FUNCPAGE_PALM            4
#define REPORT_DELAY_SYSTEM_OFF_MS 10

/* first firmware version enable gesture mode by default*/
#define DEFAULT_GESTURE_MODE_BOE_FW_VER	       0x08
#define DEFAULT_GESTURE_MODE_SDC_FW_VER	       0x05
#define DEFAULT_GESTURE_MODE_SELENE_BOE_FW_VER 0x04
#define DEFAULT_GESTURE_MODE_SELENE_SDC_FW_VER 0x04
#define DEFAULT_GESTURE_MODE_LUNA_BOE_FW_VER   0x05

#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif

#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin"

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 0
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */

#define TOUCH_CLUSTER_GAP_JIFFIES msecs_to_jiffies(5000)

/* debugfs */
#define DEBUGFS_CMD_PAYLOAD_SIZE 256
#define DEBUGFS_CMD_RECEIVE_SIZE 1024
#define DEBUGFS_CMD_DELIM	 " "

// Tracks basic information about the most recent
// cluster of touch events
struct nvt_ts_touch_stats {
	unsigned long first; // Time of first touch in jiffies
	unsigned long last; // Time of last touch in jiffies
	uint32_t event_count; // Number of events in cluster
	uint32_t fw_i2c_error_count; // Number of fw I2C error events
	uint32_t fw_recovery_error_count; // Number of fw recovery error events
	uint32_t suppress_count; // Number of suppressed events
	uint32_t palm_count; // Number of palm events
	uint32_t wakeup_gesture_count; // Number of wakeup gesture events
	uint32_t unknown_gesture_count; // Number of unknown gesture events
	uint32_t event_reported_count; // Number of events reported to evdev
	uint32_t ignore_count; // Number of INTs ignored
};

#define NVT_TRIMID_LEN 9

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	struct notifier_block drm_panel_notif;
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
	struct notifier_block drm_notif;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct nvt_ts_touch_stats stats;
	spinlock_t stats_lock;
	struct timer_list stats_timer;
	ktime_t timestamp; /* time that the event was first received from the touch IC */
	int32_t nfc_gpio;
	uint32_t nfc_active_jiffies;
	uint32_t nfc_debounce_jiffies;
	atomic64_t touch_inactive_time;
	atomic_t log_skipped_touch;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint16_t nvt_pid;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	bool idle_mode;
	char trimid[NVT_TRIMID_LEN+1];
	const char *fw_name;
	bool standby;
	bool gesture_by_default;
	atomic64_t recover_cooldown;
	uint16_t baseline_threshold;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct mutex debugfs_lock;
	struct dentry *debugfs_root;
	uint16_t debugfs_rd_addr;
	uint8_t debugfs_rd_reg;
	int32_t debugfs_rd_len;
#endif
	struct delayed_work delay_touch_event_work;
};

struct nvt_ts_input_event {
	uint16_t type;
	uint16_t code;
	int32_t value;
	ktime_t timestamp;
	struct list_head list;
};

enum INPUT_EVENT_TYPE {
	EV_TIMESTAMP = 0x18,
	EV_SLOT_STATE = 0x19,
	EV_SYNC_FRAME = 0x1a,
};

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} I2C_EVENT_MAP;

#define NVT_XBUF_LEN	(1025)

//---Novatek touchscreen power mode---
typedef enum {
	NORMAL_MODE             = 0x00,
	DEEP_SLEEP_MODE         = 0x11,
	POWER_DOWN_MODE         = 0x12,
	WAKEUP_GESTURE_MODE     = 0x13,
	TEST_MODE_2             = 0x22,
} TOUCH_POWER_MODE;

#define HANDSHAKING_HOST_READY 0xBB

//---extern structures---
extern struct nvt_ts_data *ts;

//---extern functions---
extern int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern void nvt_bootloader_reset(void);
extern void nvt_change_mode(uint8_t mode);
extern void nvt_sw_reset_idle(void);
extern int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
extern int32_t nvt_get_fw_info(void);
extern int32_t nvt_clear_fw_status(void);
extern int32_t nvt_check_fw_status(void);
extern int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr);
extern int32_t nvt_chip_id_info(uint8_t proc_chip_id[]);
extern int32_t nvt_ts_enter_power_down(void);
extern int32_t nvt_ts_enter_sleep(void);
extern void nvt_change_mode(uint8_t mode);
extern void nvt_ts_touch_suprress(unsigned int latency_jiffies);

//---for sysfs---
extern void nvt_touch_sysfs_deinit(void);
extern void nvt_touch_mp_sysfs_deinit(void);
extern int32_t nvt_touch_sysfs_init(void);
extern int32_t nvt_touch_mp_sysfs_init(void);
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
extern void nvt_stop_crc_reboot(void);

#endif /* _LINUX_NVT_TOUCH_H */
