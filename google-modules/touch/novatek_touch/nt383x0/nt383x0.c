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
#include <asm/unaligned.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/timer.h>

#include "nt383x0.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */
#include "nanohub_exports.h"

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
extern void nvt_extra_proc_deinit(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
extern void nvt_mp_proc_deinit(void);
#endif

struct nvt_ts_data *ts;
struct nvt_ts_input_event touch_event_list_head;
static struct workqueue_struct *nvt_touch_event_delay_wq;
#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static struct drm_panel *active_panel;
static int nvt_drm_panel_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static struct drm_panel *active_panel;
static void *notifier_cookie;
static void nvt_panel_notifier_callback(enum panel_event_notifier_tag tag,
		struct panel_event_notification *event, void *client_data);
#endif
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_FB)
static int nvt_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

static int32_t nvt_ts_display_resume_locked(struct device *dev);

static int nvt_init_debugfs(void);
static void nvt_deinit_debugfs(void);

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
// Slide gestures produce a key press in the opposite direction of finger travel
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_WAKEUP, //GESTURE_TAP_TO_WAKE
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_DOWN,   //GESTURE_SLIDE_UP
	KEY_UP,     //GESTURE_SLIDE_DOWN
	KEY_RIGHT,  //GESTURE_SLIDE_LEFT
	KEY_LEFT,   //GESTURE_SLIDE_RIGHT
};
#endif

static uint8_t bTouchIsAwake = 0;
static uint8_t chip_id[8] = {0};
static bool bQueueTouchEvents;
/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	struct irq_desc *desc;

	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}

	desc = irq_to_desc(ts->client->irq);
	NVT_LOG("enable=%d, desc->depth=%d\n", enable, desc->depth);
}
static void nvt_ts_release_queued_input_event(struct work_struct *unused)
{
	struct nvt_ts_input_event *event;
	struct nvt_ts_input_event *next_event;

	mutex_lock(&ts->lock);
	if (bQueueTouchEvents == 1) {
		NVT_LOG("releasing queued input events\n");
		list_for_each_entry_safe(event, next_event, &touch_event_list_head.list, list) {
			if (event->type == EV_SYNC_FRAME)
				input_mt_sync_frame(ts->input_dev);
			else if (event->type == EV_TIMESTAMP)
				input_set_timestamp(ts->input_dev, event->timestamp);
			else if (event->type == EV_SLOT_STATE)
				input_mt_report_slot_state(ts->input_dev, event->code,
							   event->value);
			else
				input_event(ts->input_dev, event->type, event->code, event->value);

			list_del(&event->list);
			kfree(event);
		}
		bQueueTouchEvents = 0;
	}
	mutex_unlock(&ts->lock);
}
/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = ts->xbuf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy(buf + 1, ts->xbuf, len - 1);
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	memcpy(ts->xbuf, buf, len);
	msg.buf   = ts->xbuf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	if (mode == NORMAL_MODE) {
		usleep_range(20000, 20000);
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
		usleep_range(20000, 20000);
	}
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint16_t i2c_addr, uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;

	return CTP_I2C_WRITE(ts->client, i2c_addr, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	NVT_LOG("start nvt_bootloader_reset\n");

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);

	NVT_LOG("end nvt_bootloader_reset\n");
}

/******************************************************
Description:
        Novatek touchscreen Power Dwon function.

return:
        n.a.
*******************************************************/
int32_t nvt_ts_enter_power_down(void)
{
	NVT_LOG("start enter power down in\n");

	//---write command to enter "power down mode"---
	nvt_change_mode(POWER_DOWN_MODE);

	msleep(50);

	NVT_LOG("end power down out \n");
	return 0;
}

/******************************************************
Description:
        Novatek touchscreen Sleep function.

return:
        n.a.
*******************************************************/
int32_t nvt_ts_enter_sleep(void)
{
	NVT_LOG("start enter sleep\n");

	//---write command to enter "deep sleep mode"---
	nvt_change_mode(DEEP_SLEEP_MODE);

	msleep(50);

	NVT_LOG("end sleep\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR |
						     EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	usleep_range(20000, 20000);

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR |
						     EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		usleep_range(10000, 10000);

		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 100)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}

	return ret;
}

int32_t nvt_chip_id_info(uint8_t proc_chip_id[])
{
	uint32_t i;

	for(i=0;i<8;i++)
	{
		proc_chip_id[i] = chip_id[i];
	}

	NVT_LOG("nvt_chip_id_info chip id[1]=0x%02X, id[2]=0x%02X, id[3]=0x%02X, id[4]=0x%02X, id[5]=0x%02X, id[6]=0x%02X\n",
	chip_id[1], chip_id[2], chip_id[3], chip_id[4], chip_id[5], chip_id[6]);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 39);
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		if (retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			ts->fw_ver = 0;
			ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
			ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
			ts->max_button_num = TOUCH_KEY_NUM;
			NVT_ERR("Set default fw_ver=%d, abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
			goto out;
		}
	}
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];
	ts->nvt_pid = (uint16_t)((buf[36] << 8) | buf[35]);
	NVT_LOG("fw_ver=0x%02X, fw_type=0x%02X, PID=0x%04X\n", ts->fw_ver, buf[14], ts->nvt_pid);

	ret = 0;
out:

	return ret;
}

#if WAKEUP_GESTURE
#define TOUCH_ID1                1
#define TOUCH_ID10              10
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_TAP_TO_WAKE     15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24

/*******************************************************
Description:
	Generate key down & key up report.

return:
	n.a.
*******************************************************/
static void nvt_send_key(struct input_dev *dev, uint32_t keycode)
{
	input_report_key(dev, keycode, 1);
	input_sync(dev);
	input_report_key(dev, keycode, 0);
	input_sync(dev);
}

/*******************************************************
Description:
	Release all active touches on screen.

return:
	n.a.
*******************************************************/
static void nvt_release_touches(void)
{
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_mt_sync_frame(ts->input_dev);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
#endif

	input_sync(ts->input_dev);
}

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
int nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];
	int ret = 0;

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return -EINVAL;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_TAP_TO_WAKE: {
			uint32_t input_x = (uint32_t)(data[4] << 4) + (uint32_t)(data[6] >> 4);
			uint32_t input_y = (uint32_t)(data[5] << 4) + (uint32_t)(data[6] & 0x0F);

			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max)) {
				NVT_LOG("Gesture : Tap to Wake, invalid coordinates\n");
				keycode = gesture_key_array[3];
				break;
			}

			NVT_LOG("Gesture : Tap to Wake, coordinates (%d,%d) instead of keycode.\n",
				input_x, input_y);
			/* Send finger down */
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			/* Placeholder pressure value, without this the
			 * touch is interpreted as a hover.
			 */
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0x300);
			input_mt_sync_frame(ts->input_dev);
			input_sync(ts->input_dev);

			/* Send finger up */
			input_mt_slot(ts->input_dev, 0);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			input_mt_sync_frame(ts->input_dev);
			input_sync(ts->input_dev);
			break;
		}
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			NVT_ERR("unknown gesture_id: %d\n", gesture_id);
			ret = -EINVAL;
			break;
	}

	if (keycode > 0) {
		nvt_send_key(ts->input_dev, keycode);
	}

	return ret;
}
#endif

struct mcu_input_message {
	uint8_t type;
	uint32_t code;
	int32_t value;
} __packed;

static inline void assign_input_message(uint8_t type, uint32_t code, int32_t value,
					struct mcu_input_message *input)
{
	if (unlikely(input == NULL))
		return;

	memcpy(&input->type, &type, sizeof(input->type));
	memcpy(&input->code, &code, sizeof(input->code));
	memcpy(&input->value, &value, sizeof(input->value));
}

static int nvt_send_mcu_message(unsigned int type, unsigned int code, int value)
{
	ssize_t bytes;
	struct mcu_input_message input;

	assign_input_message(type, code, value, &input);

	bytes = nanohub_send_message(NANOHUB_TOUCH_CHANNEL_ID, (const char *)&input, sizeof(input));
	if (bytes != sizeof(input)) {
		NVT_ERR("MCU Bytes sent expected = %zd, actual = %zd\n", sizeof(input), bytes);
		return -EIO;
	}
	return 0;
}

static inline void nvt_mcu_report_key(unsigned int code, int value)
{
	nvt_send_mcu_message(EV_KEY, code, value);
}

static inline void nvt_mcu_report_abs(unsigned int code, int value)
{
	nvt_send_mcu_message(EV_ABS, code, value);
}

static inline void nvt_mcu_sync(void)
{
	nvt_send_mcu_message(EV_SYN, SYN_REPORT, 0);
}

static void nvt_mcu_send_key(uint32_t keycode)
{
	nvt_mcu_report_key(keycode, 1);
	nvt_mcu_sync();
	nvt_mcu_report_key(keycode, 0);
	nvt_mcu_sync();
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static void nvt_parse_dt(struct device *dev)
{
	uint32_t nfc_active_ms;
	uint32_t nfc_debounce_ms;
	struct device_node *np = dev->of_node;

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

	ts->nfc_gpio = of_get_named_gpio(np, "novatek,nfc-active-gpio", 0);
	NVT_LOG("novatek,nfc-active-gpio=%d\n", ts->nfc_gpio);
	of_property_read_u32(np, "novatek,nfc-active-ms", &nfc_active_ms);
	NVT_LOG("novatek,nfc-active-ms=%u\n", nfc_active_ms);
	of_property_read_u32(np, "novatek,nfc-debounce-ms", &nfc_debounce_ms);
	NVT_LOG("novatek,nfc-debounce-ms=%u\n", nfc_debounce_ms);

	ts->nfc_active_jiffies = msecs_to_jiffies(nfc_active_ms);
	ts->nfc_debounce_jiffies = msecs_to_jiffies(nfc_debounce_ms);
}
#else
static void nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
}
#endif

static void nvt_parse_baseline_dt(struct device *dev)
{
	static const char fmt_str[] = "novatek,baseline-threshold-%04X";
	/* "%04X" will be replaced by e.g. "ABCD" (same length) */
	char prop_name[sizeof(fmt_str)];

	snprintf(prop_name, sizeof(prop_name), fmt_str, ts->nvt_pid);

	if (of_property_read_u16(dev->of_node, prop_name, &ts->baseline_threshold) &&
		of_property_read_u16(dev->of_node, "novatek,baseline-threshold",
				&ts->baseline_threshold)) {
		NVT_ERR("Failed to get baseline-threshold, using default value\n");
		ts->baseline_threshold = TOUCH_DEFAULT_BASELINE_THRESHOLD;
	}

	NVT_LOG("novatek,baseline-threshold=%hu\n", ts->baseline_threshold);
}

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen deconfig gpio

return:
	n.a.
*******************************************************/
static void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
#endif
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_ERR("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, bootloader reset */
		nvt_bootloader_reset();
		mutex_unlock(&ts->lock);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

static void report_touch_stats(void)
{
	unsigned long flags;
	struct nvt_ts_touch_stats local;

	spin_lock_irqsave(&ts->stats_lock, flags);
	local = ts->stats;
	ts->stats.event_count = 0;
	ts->stats.ignore_count = 0;
	ts->stats.fw_i2c_error_count = 0;
	ts->stats.fw_recovery_error_count = 0;
	ts->stats.suppress_count = 0;
	ts->stats.palm_count = 0;
	ts->stats.event_reported_count = 0;
	ts->stats.wakeup_gesture_count = 0;
	ts->stats.unknown_gesture_count = 0;
	spin_unlock_irqrestore(&ts->stats_lock, flags);

	if (local.event_count || local.ignore_count) {
		NVT_LOG("%u events (= %u fw_i2c_error + %u fw_recovery_error + %u suppress + "
			"%u palm + %u reported_events + %u wakeup_gesture + %u unknown_gesture) + "
			"%u ignored events in %u ms (last was %u ms ago)\n",
			local.event_count, local.fw_i2c_error_count, local.fw_recovery_error_count,
			local.suppress_count, local.palm_count, local.event_reported_count,
			local.wakeup_gesture_count, local.unknown_gesture_count, local.ignore_count,
			jiffies_to_msecs((long)local.last - (long)local.first),
			jiffies_to_msecs((long)jiffies - (long)local.last));
	}
}

static void stats_timer_func(struct timer_list *arg)
{
	report_touch_stats();
}

static void nvt_try_recover_fw_locked(void)
{
	if (time_is_before_jiffies64((u64)atomic64_read(&ts->recover_cooldown))) {
		NVT_LOG("Trying to recover by bootloader reset...\n");
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_REK);
		atomic64_set(&ts->recover_cooldown, jiffies_64 + TOUCH_RECOVER_COOLDOWN_JIFFIES);
	} else {
		NVT_LOG("Skip this recovery as it's still in cooldown.\n");
	}
}

static void nvt_check_baseline_health_locked(void)
{
	struct {
		uint8_t addr;
		uint16_t value;
	} __packed baseline_check;
	uint16_t baseline;
	int i;

	if (!ts->idle_mode)
		return;

	// Check the baseline twice. If the value is too small both times,
	// reset the firmware.
	nvt_set_page(I2C_FW_Address, ts->mmap->BASELINE_CHECK_ADDR);
	for (i = 0; i < 2; i++) {
		baseline_check.addr = ts->mmap->BASELINE_CHECK_ADDR & 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, (uint8_t *)&baseline_check,
			     sizeof(baseline_check));
		baseline = le16_to_cpu(get_unaligned(&baseline_check.value));

		if (baseline > ts->baseline_threshold)
			goto out;
	}

	NVT_ERR("baseline value is abnormal (%hu), try to recover firmware\n", baseline);
	nvt_try_recover_fw_locked();

out:
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR);
}

static irqreturn_t nvt_ts_irq_func(int irq, void *data)
{
	unsigned long flags;
	struct irq_desc *desc = irq_to_desc(irq);
	irqreturn_t ret = IRQ_NONE;

	raw_spin_lock_irqsave(&desc->lock, flags);
	ret = (desc->threads_oneshot) ? IRQ_HANDLED : IRQ_WAKE_THREAD;
	raw_spin_unlock_irqrestore(&desc->lock, flags);

	if (ret == IRQ_WAKE_THREAD) {
		ts->timestamp = ktime_get();
	} else {
		spin_lock_irqsave(&ts->stats_lock, flags);
		++ts->stats.ignore_count;
		spin_unlock_irqrestore(&ts->stats_lock, flags);
	}
	return ret;
}

static void queue_input_event(uint16_t type, uint16_t code, int32_t value, ktime_t timestamp)
{
	struct nvt_ts_input_event *input_report_event =
		kmalloc(sizeof(struct nvt_ts_input_event), GFP_KERNEL);
	input_report_event->type = type;
	input_report_event->code = code;
	input_report_event->value = value;
	input_report_event->timestamp = timestamp;
	list_add_tail(&input_report_event->list, &touch_event_list_head.list);
}

static void nvt_dump_i2c_point_data(uint8_t *point_data)
{
	int i;

	for (i = 0; i < 2; i++) {
		NVT_LOG("%02X %02X %02X %02X %02X %02X  ",
			point_data[1 + i * 6], point_data[2 + i * 6], point_data[3 + i * 6],
			point_data[4 + i * 6], point_data[5 + i * 6], point_data[6 + i * 6]);
	}
	NVT_LOG("\n");
}

static void nvt_handle_unknown_gesture(uint8_t *point_data)
{
	nvt_dump_i2c_point_data(point_data);
	++ts->stats.unknown_gesture_count;

	nvt_try_recover_fw_locked();
}

#define POINT_DATA_LEN 17
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint32_t keycode = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;
	unsigned long flags;
	unsigned long current_time;

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		pm_wakeup_event(&ts->input_dev->dev, 5000);
	}
#endif

	current_time = jiffies;
	spin_lock_irqsave(&ts->stats_lock, flags);

	if (ts->stats.event_count == 0)
		ts->stats.first = current_time;

	ts->stats.last = current_time;
	++ts->stats.event_count;

	spin_unlock_irqrestore(&ts->stats_lock, flags);

	mod_timer(&ts->stats_timer, jiffies + TOUCH_CLUSTER_GAP_JIFFIES);

	mutex_lock(&ts->lock);

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
		++ts->stats.fw_i2c_error_count;
		goto OUT;
	}

	if (time_is_after_jiffies64((u64)atomic64_read(&ts->touch_inactive_time))) {
		if (atomic_xchg(&ts->log_skipped_touch, false))
			NVT_LOG("Touch events are discarded from suppress\n");

		++ts->stats.suppress_count;
		goto OUT;
	}

#if defined(NVT_TOUCH_DEBUG_I2C_BUFFER)
	//--- dump I2C buf ---
	nvt_dump_i2c_point_data(point_data);
#endif

	if (nvt_fw_recovery(point_data)) {
#if NVT_TOUCH_ESD_PROTECT
		nvt_esd_check_enable(true);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		++ts->stats.fw_recovery_error_count;
		goto OUT;
	}

	if (bQueueTouchEvents)
		queue_input_event(EV_TIMESTAMP, 0, 0, ts->timestamp);
	else
		input_set_timestamp(ts->input_dev, ts->timestamp);

#if PALM_GESTURE
	input_id = (uint8_t)(point_data[1] >> 3);
	if ((input_id == DATA_PROTOCOL) && (point_data[2] == FUNCPAGE_PALM)) {
		if (point_data[3] == 1) {
			NVT_LOG("Palm On. idle mode %d\n", ts->idle_mode);
			if (!ts->idle_mode)
				nvt_send_key(ts->input_dev, KEY_SLEEP);
			nvt_mcu_send_key(KEY_SLEEP);
			++ts->stats.palm_count;
		} else {
			NVT_ERR("unknown palm data: %hhu\n", point_data[3]);
			nvt_handle_unknown_gesture(point_data);
		}
		goto OUT;
	}
#endif

#if WAKEUP_GESTURE
	if ((bTouchIsAwake == 0) || (ts->idle_mode)) {
		input_id = (uint8_t)(point_data[1] >> 3);
		if (input_id >= TOUCH_ID1 && input_id <= TOUCH_ID10) {
			/* Touch must have automatically transitioned to
			 * active mode due to external display mode change.
			 */
			NVT_LOG("Resuming due to active touch\n");
			if (bTouchIsAwake == 0) {
				NVT_LOG("Starting to delay reporting touch events\n");
				bQueueTouchEvents = 1;
				queue_input_event(EV_TIMESTAMP, 0, 0, ts->timestamp);
			}
			nvt_ts_display_resume_locked(&ts->client->dev);
			keycode = gesture_key_array[3];
			nvt_send_key(ts->input_dev, keycode);
		}
		else {
			if (nvt_ts_wakeup_gesture_report(input_id, point_data) == 0)
				++ts->stats.wakeup_gesture_count;
			else
				nvt_handle_unknown_gesture(point_data);
			goto OUT;
		}
	}
#endif

	finger_cnt = 0;

	++ts->stats.event_reported_count;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 15] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

			press_id[input_id - 1] = 1;
			if (bQueueTouchEvents) {
				queue_input_event(EV_ABS, ABS_MT_SLOT, input_id - 1, 0);
				queue_input_event(EV_SLOT_STATE, MT_TOOL_FINGER, true, 0);
				queue_input_event(EV_ABS, ABS_MT_POSITION_X, input_x, 0);
				queue_input_event(EV_ABS, ABS_MT_POSITION_Y, input_y, 0);
				queue_input_event(EV_ABS, ABS_MT_TOUCH_MAJOR, input_w, 0);
				queue_input_event(EV_ABS, ABS_MT_PRESSURE, input_p, 0);
				queue_input_event(EV_SYNC_FRAME, 0, 0, 0);
			} else {
#if MT_PROTOCOL_B
				input_mt_slot(ts->input_dev, input_id - 1);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);
#if MT_PROTOCOL_B
				input_mt_sync_frame(ts->input_dev);
#else /* MT_PROTOCOL_B */
				input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */
			}
			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			if (bQueueTouchEvents) {
				queue_input_event(EV_ABS, ABS_MT_SLOT, i, 0);
				queue_input_event(EV_ABS, ABS_MT_TOUCH_MAJOR, 0, 0);
				queue_input_event(EV_ABS, ABS_MT_PRESSURE, 0, 0);
				queue_input_event(EV_SLOT_STATE, MT_TOOL_FINGER, false, 0);
			} else {
				input_mt_slot(ts->input_dev, i);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	if (bQueueTouchEvents)
		queue_input_event(EV_SYNC_FRAME, 0, 0, ts->timestamp);
	else
		input_mt_sync_frame(ts->input_dev);

#else /* MT_PROTOCOL_B */
	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
	if (finger_cnt == 0) {
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[13] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[14] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	if (bQueueTouchEvents)
		queue_input_event(EV_SYN, SYN_REPORT, 0, 0);
	else
		input_sync(ts->input_dev);

OUT:
	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}

void nvt_ts_touch_suprress(unsigned int latency_jiffies)
{
	atomic64_set(&ts->touch_inactive_time, jiffies_64 + latency_jiffies);
	atomic_set(&ts->log_skipped_touch, latency_jiffies > 0);
}

static irqreturn_t nvt_ts_nfc_irq_handler(int irq, void *data)
{
	/* Ignore any ghost touches introduced by NFC */
	if (gpio_get_value(ts->nfc_gpio)) {
		unsigned int nfc_active_ms = jiffies_to_msecs(ts->nfc_active_jiffies);
		dev_info(&ts->client->dev,
			"NFC active, touch suppressed for up to %u ms\n", nfc_active_ms);
		nvt_ts_touch_suprress(ts->nfc_active_jiffies);
	} else {
		if (time_is_after_jiffies64((u64)atomic64_read(&ts->touch_inactive_time))) {
			/* If touch reporting has not already resumed, do so now */
			dev_info(&ts->client->dev,
				 "NFC inactive, touch reporting will resume in %u ms\n",
				 jiffies_to_msecs(ts->nfc_debounce_jiffies));
			nvt_ts_touch_suprress(ts->nfc_debounce_jiffies);
		}
	}

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check and stop crc reboot loop.

return:
	n.a.
*******************************************************/
void nvt_stop_crc_reboot(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;

	//read dummy buffer to check CRC fail reboot is happening or not

	//---change I2C index to prevent geting 0xFF, but not 0xFC---
	nvt_set_page(I2C_BLDR_Address, CHIP_VER_TRIM_ADDR);

	//---read to check if buf is 0xFC which means IC is in CRC reboot ---
	buf[0] = CHIP_VER_TRIM_ADDR & 0xFF;
	CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 4);

	if ((buf[1] == 0xFC) ||
		((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {

		//IC is in CRC fail reboot loop, needs to be stopped!
		for (retry = 5; retry > 0; retry--) {

			//---write i2c cmds to reset idle : 1st---
			buf[0]=0x00;
			buf[1]=0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

			//---write i2c cmds to reset idle : 2rd---
			buf[0]=0x00;
			buf[1]=0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
			msleep(1);

			//---clear CRC_ERR_FLAG---
			nvt_set_page(I2C_BLDR_Address, 0x3F135);

			buf[0] = 0x35;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 2);

			//---check CRC_ERR_FLAG---
			nvt_set_page(I2C_BLDR_Address, 0x3F135);

			buf[0] = 0x35;
			buf[1] = 0x00;
			CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 2);

			if (buf[1] == 0xA5)
				break;
		}
		if (retry == 0)
			NVT_ERR("CRC auto reboot is not able to be stopped! buf[1]=0x%02X\n", buf[1]);
	}

	return;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(uint32_t chip_ver_trim_addr)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	nvt_bootloader_reset(); // NOT in retry loop

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		nvt_set_page(I2C_BLDR_Address, chip_ver_trim_addr);

		buf[0] = chip_ver_trim_addr & 0xFF;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		//---Stop CRC check to prevent IC auto reboot---
		for (i = 0; i < 8; i++) {
			chip_id[i] = buf[i];
		}
		NVT_LOG("chip id[1]=0x%02X, id[2]=0x%02X, id[3]=0x%02X, id[4]=0x%02X, id[5]=0x%02X, id[6]=0x%02X\n",
			chip_id[1], chip_id[2], chip_id[3], chip_id[4], chip_id[5], chip_id[6]);

		//---Stop CRC check to prevent IC auto reboot---
		if ((buf[1] == 0xFC) ||
			((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot();
			continue;
		}

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				strncpy(ts->trimid, trim_id_table[list].name, NVT_TRIMID_LEN);
				ts->trimid[NVT_TRIMID_LEN] = '\0';
				ts->mmap = trim_id_table[list].mmap;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

#if defined(CONFIG_DRM_PANEL) && IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static int nvt_ts_check_dt(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		if (!IS_ERR(panel)) {
			ts->fw_name = of_get_property(node, "touch,firmware", NULL);
			if (ts->fw_name) {
				NVT_LOG("touch,firmware=%s\n", ts->fw_name);
			}
			of_node_put(node);
			active_panel = panel;
			return 0;
		}
		of_node_put(node);
	}

	return PTR_ERR(panel);
}
#endif

static int nvt_ts_input_open(struct input_dev *dev) {
	int irq = ts->client->irq;

	NVT_LOG("enabling wake on irq %d\n", irq);
	enable_irq_wake(irq);

	return 0;
}

static void nvt_ts_input_close(struct input_dev *dev) {
	int irq = ts->client->irq;

	NVT_LOG("disabling wake on irq %d\n", irq);
	disable_irq_wake(irq);
}

static bool nvt_check_default_gesture_mode_enable(void)
{
	return ts->fw_name && ((strcmp(ts->fw_name, "novatek_ts_fw_boe.bin") == 0 &&
				ts->fw_ver >= DEFAULT_GESTURE_MODE_BOE_FW_VER) ||
			       (strcmp(ts->fw_name, "novatek_ts_fw_sdc.bin") == 0 &&
				ts->fw_ver >= DEFAULT_GESTURE_MODE_SDC_FW_VER) ||
			       (strcmp(ts->fw_name, "novatek_ts_fw_boe_selene.bin") == 0 &&
				ts->fw_ver >= DEFAULT_GESTURE_MODE_SELENE_BOE_FW_VER) ||
			       (strcmp(ts->fw_name, "novatek_ts_fw_sdc_selene.bin") == 0 &&
				ts->fw_ver >= DEFAULT_GESTURE_MODE_SELENE_SDC_FW_VER) ||
			       (strcmp(ts->fw_name, "novatek_ts_fw_boe_luna.bin") == 0 &&
				ts->fw_ver >= DEFAULT_GESTURE_MODE_LUNA_BOE_FW_VER));
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if defined(CONFIG_DRM_PANEL) && IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	struct device_node *dp = NULL;
#endif
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	void *cookie = NULL;
#endif
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif
	int nfc_irq;

	NVT_LOG("nvt_ts_probe start\n");

	ts = (struct nvt_ts_data *)kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

#if defined(CONFIG_DRM_PANEL) && IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	dp = client->dev.of_node;

	ret = nvt_ts_check_dt(dp);
	if (ret == -EPROBE_DEFER) {
		goto err_malloc_xbuf;
	}

	if (ret) {
		ret = -ENODEV;
		goto err_malloc_xbuf;
	}
#endif

#if defined(WAKEUP_GESTURE)
	ts->standby = true;
#else
	ts->standby = false;
#endif
	bQueueTouchEvents = 0;
	INIT_LIST_HEAD(&touch_event_list_head.list);
	ts->xbuf = (uint8_t *)kzalloc(NVT_XBUF_LEN, GFP_KERNEL);
	if (ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	NVT_LOG("nvt_ts_probe parse dt line = %d\n",__LINE__);
	//---parse dts---
	nvt_parse_dt(&client->dev);

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NVT_ERR("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	timer_setup(&ts->stats_timer, stats_timer_func, 0);
	INIT_DELAYED_WORK(&ts->delay_touch_event_work, nvt_ts_release_queued_input_event);
	nvt_touch_event_delay_wq =
		alloc_workqueue("nvt_touch_event_delay_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_touch_event_delay_wq) {
		NVT_ERR("nvt_touch_event_delay_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_touch_event_delay_wq_failed;
	}

	spin_lock_init(&ts->stats_lock);
	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_ADDR);
	if (ret) {
		NVT_LOG("try to check from old chip ver trim address\n");
		ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_OLD_ADDR);
		if (ret) {
			NVT_ERR("chip is not identified\n");
			ret = -EINVAL;
			goto err_chipvertrim_failed;
		}
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();
	nvt_parse_baseline_dt(&client->dev);
	ts->gesture_by_default = nvt_check_default_gesture_mode_enable();
	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array) / sizeof(gesture_key_array[0])); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

#if PALM_GESTURE
	input_set_capability(ts->input_dev, EV_KEY, KEY_SLEEP);
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	ts->input_dev->open = nvt_ts_input_open;
	ts->input_dev->close = nvt_ts_input_close;

	//---register input device---
	// Be sure to initialize client->irq prior to registering the input device
	// Once the device is registered nvt_ts_input_open can be called, which
	// makes use of client->irq
	client->irq = gpio_to_irq(ts->irq_gpio);
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	if (gpio_is_valid(ts->nfc_gpio) && ts->nfc_active_jiffies > 0) {
		nfc_irq = gpio_to_irq(ts->nfc_gpio);
		ret = request_irq(nfc_irq, nvt_ts_nfc_irq_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, NVT_I2C_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq (NFC idle) failed. ret = %d\n", ret);
		}
	}

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif

	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = alloc_workqueue("nvt_esd_check_wq", WQ_MEM_RECLAIM, 1);
	if (!nvt_esd_check_wq) {
		NVT_ERR("nvt_esd_check_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_esd_check_wq_failed;
	}
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---set device node---
#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_extra_proc_init_failed;
	}
#endif

#if NVT_TOUCH_EXT_SYSFS
	ret = nvt_touch_sysfs_init();
	if (ret != 0) {
		NVT_ERR("nvt touch sysfs init failed. ret=%d\n", ret);
		goto err_touch_sysfs_init_failed;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_mp_proc_init_failed;
	}
#endif

#if NVT_TOUCH_MP_SYSFS

	ret = nvt_touch_mp_sysfs_init();
	if (ret != 0) {
		NVT_ERR("nvt touch mp sysfs init failed. ret=%d\n", ret);
		goto err_touch_mp_sysfs_init_failed;
	}
#endif

#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	ts->drm_panel_notif.notifier_call = nvt_drm_panel_notifier_callback;
	if (active_panel) {
		ret = drm_panel_notifier_register(active_panel, &ts->drm_panel_notif);
		if (ret < 0) {
			NVT_ERR("register drm_panel_notifier failed. ret=%d\n", ret);
			goto err_register_drm_panel_notif_failed;
		}
	}
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (active_panel) {
		cookie = panel_event_notifier_register(PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH, active_panel,
				&nvt_panel_notifier_callback, ts);
	}

	if (IS_ERR_OR_NULL(cookie)) {
		NVT_ERR("Failed to register for panel events\n");
		goto err_register_suspend_resume_failed;
	}

	notifier_cookie = cookie;
#endif
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
	ts->drm_notif.notifier_call = nvt_drm_notifier_callback;
	ret = msm_drm_register_client(&ts->drm_notif);
	if(ret) {
		NVT_ERR("register drm_notifier failed. ret=%d\n", ret);
		goto err_register_drm_notif_failed;
	}
#elif defined(CONFIG_FB)
	ts->fb_notif.notifier_call = nvt_fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret) {
		NVT_ERR("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	ret = register_early_suspend(&ts->early_suspend);
	if(ret) {
		NVT_ERR("register early suspend failed. ret=%d\n", ret);
		goto err_register_early_suspend_failed;
	}
#endif

	bTouchIsAwake = 1;
	ts->idle_mode = false;

	//---set int-pin & request irq---
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ts->irq_enabled = true;
		ret = request_threaded_irq(client->irq, nvt_ts_irq_func, nvt_ts_work_func,
					   ts->int_trigger_type | IRQF_ONESHOT, NVT_I2C_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

	nvt_init_debugfs();

	NVT_LOG("end\n");

	return 0;

err_int_request_failed:
#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
err_register_drm_panel_notif_failed:
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (active_panel && notifier_cookie)
		panel_event_notifier_unregister(notifier_cookie);
#endif
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
err_register_drm_notif_failed:
#elif defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
err_register_fb_notif_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
err_register_early_suspend_failed:
#endif
err_register_suspend_resume_failed:
#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
err_mp_proc_init_failed:
#endif

#if NVT_TOUCH_EXT_SYSFS
	nvt_touch_mp_sysfs_deinit();
err_touch_mp_sysfs_init_failed:
#endif

#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
err_extra_proc_init_failed:
#endif

#if NVT_TOUCH_MP_SYSFS
	nvt_touch_sysfs_deinit();
err_touch_sysfs_init_failed:
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
err_create_nvt_esd_check_wq_failed:
#endif
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
#endif
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_dev_alloc_failed:
err_chipvertrim_failed:
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
err_create_nvt_touch_event_delay_wq_failed:
err_check_functionality_failed:
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
	i2c_set_clientdata(client, NULL);
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	NVT_LOG("Removing driver...\n");

	nvt_deinit_debugfs();

#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	if (active_panel) {
		if (drm_panel_notifier_unregister(active_panel, &ts->drm_panel_notif))
			NVT_ERR("Error occurred while unregistering drm_panel_notifier.\n");
	}
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (active_panel && notifier_cookie)
		panel_event_notifier_unregister(notifier_cookie);
#endif
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
#elif defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	nvt_gpio_deconfig(ts);

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	i2c_set_clientdata(client, NULL);

	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}

	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return 0;
}

static void nvt_ts_shutdown(struct i2c_client *client)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	if (active_panel) {
		if (drm_panel_notifier_unregister(active_panel, &ts->drm_panel_notif))
			NVT_ERR("Error occurred while unregistering drm_panel_notifier.\n");
	}
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
	if (active_panel && notifier_cookie)
		panel_event_notifier_unregister(notifier_cookie);
#endif
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
#elif defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#if NVT_TOUCH_MP
	nvt_mp_proc_deinit();
#endif
#if NVT_TOUCH_EXT_PROC
	nvt_extra_proc_deinit();
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver idle function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_display_idle(void)
{
	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	ts->idle_mode = true;

	NVT_LOG("end\n");

	mutex_unlock(&ts->lock);

	nvt_release_touches();

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_display_suspend(struct device *dev)
{
	uint8_t buf[4] = { 0 };

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(false);
#endif

#if NVT_TOUCH_ESD_PROTECT
	NVT_LOG("cancel delayed work sync\n");
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	mutex_lock(&ts->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;
	ts->idle_mode = false;

	if (ts->standby) {
		if (ts->gesture_by_default) {
			NVT_LOG("Enabled touch wakeup gesture by default, fw version:%02X\n",
				ts->fw_ver);
		} else {
			//---write command to enter "wakeup gesture mode"---
			buf[0] = EVENT_MAP_HOST_CMD;
			buf[1] = WAKEUP_GESTURE_MODE;
			CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
			NVT_LOG("Enabled touch wakeup gesture by i2c commands, fw version:%02X\n",
				ts->fw_ver);
		}
	}

	mutex_unlock(&ts->lock);

	nvt_release_touches();

	msleep(50);

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function (locked).
	ts lock must be held before calling this.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_display_resume_locked(struct device *dev)
{
	ts->idle_mode = false;
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	// need to uncomment the following code for NT36672, NT36772 IC due to no boot-load when RESX/TP_RESX
	//nvt_bootloader_reset();
	if (nvt_check_fw_reset_state(RESET_STATE_REK)) {
		NVT_ERR("FW is not ready! Try to bootloader reset...\n");
		nvt_bootloader_reset();
		nvt_check_fw_reset_state(RESET_STATE_REK);
	}

#if !WAKEUP_GESTURE
	nvt_irq_enable(true);
#endif

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_display_resume(struct device *dev)
{
	int32_t ret;

	// Release the queued events again in the display post resume
	// to avoid queuing the events that are received during the
	// transition from display OFF to ON.
	nvt_ts_release_queued_input_event(NULL);

	mutex_lock(&ts->lock);
	nvt_check_baseline_health_locked();
	ret = nvt_ts_display_resume_locked(dev);
	mutex_unlock(&ts->lock);

	return ret;
}

static int nvt_ts_suspend(struct device *dev)
{
	NVT_LOG("disabling IRQ\n");
	nvt_irq_enable(false);
	del_timer(&ts->stats_timer);

	report_touch_stats();

	return 0;
}

static int nvt_ts_resume(struct device *dev) {
	NVT_LOG("enabling IRQ\n");
	nvt_irq_enable(true);

	return 0;
}

#if defined(CONFIG_DRM_PANEL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static int nvt_drm_panel_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct drm_panel_notifier *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, drm_panel_notif);

	if (!evdata)
		return 0;

	if (!(event == DRM_PANEL_EARLY_EVENT_BLANK ||
		event == DRM_PANEL_EVENT_BLANK)) {
		//NVT_LOG("event(%lu) not need to process\n", event);
		return 0;
	}

	if (evdata->data && ts) {
		blank = evdata->data;
		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			if (*blank == DRM_PANEL_BLANK_POWERDOWN) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				nvt_ts_display_suspend(&ts->client->dev);
			} else if (*blank == DRM_PANEL_BLANK_LP) {
				ts->idle_mode = true;
			}
		} else if (event == DRM_PANEL_EVENT_BLANK) {
			if (*blank == DRM_PANEL_BLANK_UNBLANK) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				if(ts->idle_mode) {
					ts->idle_mode = false;
				}
				nvt_ts_display_resume(&ts->client->dev);
			}
		}
	}

	return 0;
}
#else
#if IS_ENABLED(CONFIG_QCOM_PANEL_EVENT_NOTIFIER)
static void nvt_panel_notifier_callback(enum panel_event_notifier_tag tag,
		struct panel_event_notification *notification, void *client_data)
{
	struct nvt_ts_data *ts = client_data;

	if (!notification) {
		NVT_ERR("Invalid notification\n");
		return;
	}

	if (!client_data) {
		NVT_ERR("Invalid client data\n");
		return;
	}
	NVT_LOG("Notification type:%d, early_trigger:%d",
			notification->notif_type,
			notification->notif_data.early_trigger);

	switch (notification->notif_type) {
		case DRM_PANEL_EVENT_UNBLANK:
			if (notification->notif_data.early_trigger) {
				NVT_LOG("resume notification pre commit\n");
				queue_delayed_work(nvt_touch_event_delay_wq,
						   &ts->delay_touch_event_work,
						   msecs_to_jiffies(REPORT_DELAY_SYSTEM_OFF_MS));
			} else {
				NVT_LOG("resume notification post commit\n");
				nvt_ts_display_resume(&ts->client->dev);
			}
			break;

		case DRM_PANEL_EVENT_BLANK:
			if (notification->notif_data.early_trigger) {
				NVT_LOG("suspend notification pre commit\n");
				nvt_ts_display_suspend(&ts->client->dev);
			} else {
				NVT_LOG("suspend notification post commit\n");
			}
			break;

		case DRM_PANEL_EVENT_BLANK_LP:
			NVT_LOG("display idle notification\n");
			nvt_ts_display_idle();
			break;

		case DRM_PANEL_EVENT_FPS_CHANGE:
			NVT_LOG("Received fps change old fps:%d new fps:%d\n",
					notification->notif_data.old_fps,
					notification->notif_data.new_fps);
			break;
		case DRM_PANEL_EVENT_RESET: {
			NVT_LOG("Received Panel reset event");
			nvt_send_key(ts->input_dev, gesture_key_array[3]);
			break;
		}
		default:
			NVT_LOG("notification serviced :%d\n",
					notification->notif_type);
			break;
	}
}
#endif
#endif
#elif defined(_MSM_DRM_NOTIFY_H_)
static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, drm_notif);

	if (!evdata || (evdata->id != 0))
		return 0;

	if (evdata->data && ts) {
		blank = evdata->data;
		if (event == MSM_DRM_EARLY_EVENT_BLANK) {
			if (*blank == MSM_DRM_BLANK_POWERDOWN) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				nvt_ts_display_suspend(&ts->client->dev);
			}
		} else if (event == MSM_DRM_EVENT_BLANK) {
			if (*blank == MSM_DRM_BLANK_UNBLANK) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				nvt_ts_display_resume(&ts->client->dev);
			}
		}
	}

	return 0;
}
#elif defined(CONFIG_FB)
static int nvt_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
			nvt_ts_display_suspend(&ts->client->dev);
		} else if (*blank == FB_BLANK_VSYNC_SUSPEND) {
			ts->idle_mode = true;
		}
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
			if(ts->idle_mode) {
				ts->idle_mode = false;
			} else {
				nvt_ts_display_resume(&ts->client->dev);
			}
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver early suspend function.

return:
	n.a.
*******************************************************/
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	nvt_ts_display_suspend(ts->client, PMSG_SUSPEND);
}

/*******************************************************
Description:
	Novatek touchscreen driver late resume function.

return:
	n.a.
*******************************************************/
static void nvt_ts_late_resume(struct early_suspend *h)
{
	nvt_ts_display_resume(ts->client);
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts",},
	{ },
};
#endif

// This should be equivalent to the expansion of
// static SIMPLE_DEV_PM_OPS(nvt_ts_pm_ops, nvt_ts_suspend, nvt_ts_resume);
// with the addition of 'resume_noirq'
static const struct dev_pm_ops nvt_ts_pm_ops = {
	.suspend = nvt_ts_suspend,
	.resume = nvt_ts_resume,
	.freeze = nvt_ts_suspend,
	.thaw = nvt_ts_resume,
	.poweroff = nvt_ts_suspend,
	.restore = nvt_ts_resume,
};

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
		.pm = &nvt_ts_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("nvt start\n");
	//---add i2c driver---
	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		NVT_ERR("nvt failed to add i2c driver");
		goto err_driver;
	}

	NVT_LOG("nvt finished\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);
}

#if defined(CONFIG_DRM_PANEL)
late_initcall(nvt_driver_init);
#else
module_init(nvt_driver_init);
#endif
module_exit(nvt_driver_exit);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif

#if IS_ENABLED(CONFIG_DEBUG_FS)
static ssize_t debugfs_i2c_write_cmd_w(struct file *file, const char __user *p, size_t count,
				       loff_t *ppos)
{
	char *input, *input_tmp, *token;
	uint8_t buf[DEBUGFS_CMD_PAYLOAD_SIZE] = { 0 };
	size_t buf_len = 0;
	ssize_t ret = 0;
	int addr32 = 0, payload32 = 0;
	uint16_t addr;

	if (*ppos) {
		NVT_ERR("invalid argument\n");
		return -EINVAL;
	}

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input) {
		NVT_ERR("allocate memory failed\n");
		return -ENOMEM;
	}

	if (copy_from_user(input, p, count)) {
		NVT_ERR("copy from user failed\n");
		ret = -EFAULT;
		goto END;
	}
	input[count] = '\0';

	NVT_LOG("Command requested for write to touch device: %s\n", input);
	input_tmp = input;

	token = strsep(&input_tmp, DEBUGFS_CMD_DELIM);
	if (!token) {
		NVT_ERR("invalid input data format\n");
		ret = -EINVAL;
		goto END;
	}

	ret = kstrtoint(token, 0, &addr32);
	if (ret) {
		NVT_ERR("input buffer conversion failed\n");
		goto END;
	}
	addr = (uint16_t)(addr32 & 0xff);

	token = strsep(&input_tmp, DEBUGFS_CMD_DELIM);
	while (token) {
		ret = kstrtoint(token, 0, &payload32);
		if (ret) {
			NVT_ERR("input buffer conversion failed\n");
			goto END;
		}

		buf[buf_len++] = (payload32 & 0xff);
		if (buf_len >= DEBUGFS_CMD_PAYLOAD_SIZE) {
			NVT_ERR("buffer size exceeding the limit\n");
			ret = -EFAULT;
			goto END;
		}
		token = strsep(&input_tmp, DEBUGFS_CMD_DELIM);
	}

	if (!buf_len) {
		NVT_ERR("parse write data failed\n");
		ret = -EFAULT;
		goto END;
	}

	NVT_LOG("Command requested for transfer to address: 0x%x\n", addr);

	ret = CTP_I2C_WRITE(ts->client, addr, buf, buf_len);
	if (ret == 1)
		ret = count;
	else
		NVT_ERR("i2c write failed\n");

END:
	kfree(input);
	return ret;
}

static const struct file_operations debugfs_i2c_write_cmd_fops = {
	.open = simple_open,
	.write = debugfs_i2c_write_cmd_w,
};

static ssize_t debugfs_i2c_read_cmd_r(struct file *file, char __user *buf, size_t count,
				      loff_t *ppos)
{
	char *output = NULL;
	char *output_tmp = NULL;
	int i = 0, n = 0, left_size = 0;
	uint8_t *rd_buf = NULL;
	int32_t rd_len;
	uint16_t addr;
	uint8_t reg_addr;
	ssize_t ret = 0;

	if (*ppos)
		return 0;

	mutex_lock(&ts->debugfs_lock);
	rd_len = ts->debugfs_rd_len;
	addr = ts->debugfs_rd_addr;
	reg_addr = ts->debugfs_rd_reg;
	mutex_unlock(&ts->debugfs_lock);

	if (rd_len <= 0) {
		NVT_ERR("no valid read command\n");
		return -EINVAL;
	}

	rd_buf = kzalloc(rd_len + 1, GFP_KERNEL);
	if (!rd_buf) {
		NVT_ERR("allocate read buffer failed\n");
		return -ENOMEM;
	}

	rd_buf[0] = reg_addr;
	ret = CTP_I2C_READ(ts->client, addr, rd_buf, rd_len + 1);
	if (ret < 0) {
		NVT_ERR("i2c read failed\n");
		goto ERR_FREE_READBUF;
	}

	left_size = (rd_len / 10 + rd_len + 1) * 5 + 1;
	output = kzalloc(left_size, GFP_KERNEL);
	if (!output) {
		NVT_ERR("allocate output memory failed\n");
		ret = -ENOMEM;
		goto ERR_FREE_READBUF;
	}
	output_tmp = output;

	for (i = 1; i <= rd_len; i++) {
		if (i % 10 == 1) {
			n = scnprintf(output_tmp, left_size, "%2d0: ", i / 10);
			if (!n)
				break;
			output_tmp += n;
			left_size -= n;
		}

		n = scnprintf(output_tmp, left_size, "0x%02X ", rd_buf[i]);
		if (!n)
			break;
		output_tmp += n;
		left_size -= n;

		if (i % 10 == 0)
			*(output_tmp - 1) = '\n';
	}
	*(output_tmp - 1) = '\n';

	ret = min(strlen(output) + 1, count);
	if (ret <= 0) {
		NVT_ERR("scnprintf failed, err: %d\n", ret);
		ret = -EFAULT;
		goto ERR_FREE_OUTPUT;
	}

	if (copy_to_user(buf, output, ret)) {
		NVT_ERR("copy to user buffer failed\n");
		ret = -EFAULT;
		goto ERR_FREE_OUTPUT;
	}

	*ppos += ret;

ERR_FREE_OUTPUT:
	kfree(output);

ERR_FREE_READBUF:
	kfree(rd_buf);
	return ret;
}

static ssize_t debugfs_i2c_read_cmd_w(struct file *file, const char __user *p, size_t count,
				      loff_t *ppos)
{
	char *input;
	ssize_t ret = 0;
	uint16_t addr;
	uint8_t reg_addr;
	int32_t rd_size;

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input) {
		NVT_ERR("allocate memory failed\n");
		return -ENOMEM;
	}

	if (copy_from_user(input, p, count)) {
		NVT_ERR("copy from user failed\n");
		ret = -EFAULT;
		goto END;
	}
	input[count] = '\0';

	NVT_LOG("Command requested for read from touch device: %s\n", input);

	ret = sscanf(input, "%hx %hhx %d", &addr, &reg_addr, &rd_size);
	if (ret != 3) {
		NVT_ERR("invalid input data format\n");
		ret = -EINVAL;
		goto END;
	}

	if (rd_size <= 0 || rd_size > DEBUGFS_CMD_RECEIVE_SIZE) {
		ret = -EFAULT;
		NVT_ERR("invalid read length\n");
		goto END;
	}

	mutex_lock(&ts->debugfs_lock);
	if (ret < 0) {
		ret = -EINVAL;

		ts->debugfs_rd_len = 0;
	} else {
		ret = count;

		ts->debugfs_rd_len = rd_size;
		ts->debugfs_rd_addr = addr;
		ts->debugfs_rd_reg = reg_addr;
	}
	mutex_unlock(&ts->debugfs_lock);

END:
	kfree(input);
	return ret;
}

static const struct file_operations debugfs_i2c_read_cmd_fops = {
	.open = simple_open,
	.read = debugfs_i2c_read_cmd_r,
	.write = debugfs_i2c_read_cmd_w,
};

int nvt_init_debugfs(void)
{
	ts->debugfs_root = debugfs_create_dir(NVT_I2C_NAME, NULL);
	if (!ts->debugfs_root) {
		NVT_ERR("failed to create debugfs dir\n");
		return -ENOMEM;
	}

	if (!debugfs_create_file("i2c_write_cmd", 0600, ts->debugfs_root, NULL,
				 &debugfs_i2c_write_cmd_fops)) {
		NVT_ERR("failed to create i2c_write_cmd\n");
		return -ENOMEM;
	}

	if (!debugfs_create_file("i2c_read_cmd", 0600, ts->debugfs_root, NULL,
				 &debugfs_i2c_read_cmd_fops)) {
		NVT_ERR("failed to create i2c_read_cmd\n");
		return -ENOMEM;
	}

	return 0;
}

void nvt_deinit_debugfs(void)
{
	debugfs_remove_recursive(ts->debugfs_root);
	ts->debugfs_root = NULL;
}

#else
int nvt_init_debugfs(void)
{
	return 0;
}

void nvt_deinit_debugfs(void)
{
}
#endif //IS_ENABLED(CONFIG_DEBUG_FS)

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
