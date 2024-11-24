/*
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/sched/signal.h>
#include <linux/sched/types.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>

#include "nanohub.h"
#include "nanohub_exports.h"
#include "main.h"
#include "comms.h"
#include "spi.h"

#ifdef CONFIG_NANOHUB_BL_ST
#include "bl_st.h"
#endif

#ifdef CONFIG_NANOHUB_BL_NXP
#include "bl_nxp.h"
#endif

#define READ_QUEUE_DEPTH	20
#define APP_FROM_HOST_EVENTID	0x000000F8
#define FIRST_SENSOR_EVENTID	0x00000200
#define LAST_SENSOR_EVENTID	0x000002FF
#define APP_TO_HOST_EVENTID	0x00000401
#define OS_LOG_EVENTID		0x3B474F4C
#define WAKEUP_INTERRUPT	1
#define WAKEUP_TIMEOUT_MS	1000
#define SUSPEND_TIMEOUT_MS	100
#define RETRY_INT_WIDTH_US      50
#define KTHREAD_ERR_TIME_NS	(60LL * NSEC_PER_SEC)
#define KTHREAD_ERR_CNT		70
#define KTHREAD_WARN_CNT	10
#define WAKEUP_ERR_TIME_NS	(60LL * NSEC_PER_SEC)
#define WAKEUP_ERR_CNT		4
#define BL_MAX_SPEED_HZ		1000000
#define DT_MAX_PROP_SIZE	32

#define GPIO05_INT_SET_TYPE			0x00008C11
#define GPIO05_INT_POLARITY_HIGH		0x00008C12
#define GPIO05_INT_EN_SET 			0x00008C15
#define GPIO05_INT_EN_CLR 			0x00008C16
#define GPIO05_INT_MID_SEL 			0x00008C1A
#define GPIO05_OUTPUT_CTRL 			0x00008C44

/**
 * struct gpio_config - this is a binding between platform data and driver data
 * @label:     for diagnostics
 * @flags:     to pass to gpio_request_one()
 * @options:   one or more of GPIO_OPT_* flags, below
 * @pdata_off: offset of u32 field in platform data with gpio #
 * @data_off:  offset of int field in driver data with irq # (optional)
 */
struct gpio_config {
	const char *label;
	u16 flags;
	u16 options;
	u16 pdata_off;
	u16 data_off;
};

struct device_config {
	const char *name;
	const uint8_t channels;
	const bool is_kernel;
};

#define GPIO_OPT_HAS_IRQ	0x0001
#define GPIO_OPT_OPTIONAL	0x8000

#define PLAT_GPIO_DEF(name, _flags) \
	.pdata_off = offsetof(struct nanohub_platform_data, name ## _gpio), \
	.label = "nanohub_" #name, \
	.flags = _flags \

#define PLAT_GPIO_DEF_IRQ(name, _flags, _opts) \
	PLAT_GPIO_DEF(name, _flags), \
	.data_off = offsetof(struct nanohub_data, name), \
	.options = GPIO_OPT_HAS_IRQ | (_opts) \

static int nanohub_open(struct inode *, struct file *);
static ssize_t nanohub_read(struct file *, char *, size_t, loff_t *);
static ssize_t nanohub_write(struct file *, const char *, size_t, loff_t *);
static unsigned int nanohub_poll(struct file *, poll_table *);
static int nanohub_release(struct inode *, struct file *);
static int nanohub_hw_reset(struct nanohub_data *data, int boot_mode);
static int nanohub_pmic_irq_config(struct regmap *pmic_regmap);

// Kernel client support.
static struct nanohub_data *priv_nanohub_data;
EXPORT_SYMBOL(nanohub_send_message);
EXPORT_SYMBOL(nanohub_register_listener);
EXPORT_SYMBOL(nanohub_unregister_listener);

static struct class *sensor_class;
static int major;

static const struct gpio_config gconf[] = {
	{ PLAT_GPIO_DEF(wakeup, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(nreset, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(boot0, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF(boot2, GPIOF_OUT_INIT_HIGH) },
	{ PLAT_GPIO_DEF_IRQ(irq1, GPIOF_DIR_IN, 0) },
	{ PLAT_GPIO_DEF_IRQ(irq2, GPIOF_DIR_IN, GPIO_OPT_OPTIONAL) },
};

static const struct iio_info nanohub_iio_info = {
};

static const struct file_operations nanohub_fileops = {
	.owner = THIS_MODULE,
	.open = nanohub_open,
	.read = nanohub_read,
	.write = nanohub_write,
	.poll = nanohub_poll,
	.release = nanohub_release,
};

enum {
	ST_RESET,
	ST_IDLE,
	ST_ERROR,
	ST_RUNNING
};

static inline bool gpio_is_optional(const struct gpio_config *_cfg)
{
	return _cfg->options & GPIO_OPT_OPTIONAL;
}

static inline bool gpio_has_irq(const struct gpio_config *_cfg)
{
	return _cfg->options & GPIO_OPT_HAS_IRQ;
}

static inline bool nanohub_has_priority_lock_locked(struct nanohub_data *data)
{
	return  atomic_read(&data->wakeup_lock_cnt) >
		atomic_read(&data->wakeup_cnt);
}

static inline void nanohub_notify_thread(struct nanohub_data *data)
{
	atomic_set(&data->kthread_run, 1);
	/* wake_up implementation works as memory barrier */
	wake_up_interruptible_sync(&data->kthread_wait);
}

static inline void nanohub_io_init(struct nanohub_io *io,
				   struct nanohub_data *data,
				   struct device *dev,
				   int id,
				   bool is_kernel)
{
	init_waitqueue_head(&io->buf_wait);
	INIT_LIST_HEAD(&io->buf_list);
	io->data = data;
	io->dev = dev;
	io->id = id;
	io->is_kernel = is_kernel;
	atomic_set(&io->busy, 0);
}

static inline bool nanohub_io_has_buf(struct nanohub_io *io)
{
	return !list_empty(&io->buf_list);
}

static struct nanohub_buf *nanohub_io_get_buf(struct nanohub_io *io,
					      bool wait)
{
	struct nanohub_buf *buf = NULL;
	int ret;

	spin_lock(&io->buf_wait.lock);
	if (wait) {
		ret = wait_event_interruptible_locked(io->buf_wait,
						      nanohub_io_has_buf(io));
		if (ret < 0) {
			spin_unlock(&io->buf_wait.lock);
			return ERR_PTR(ret);
		}
	}

	if (nanohub_io_has_buf(io)) {
		buf = list_first_entry(&io->buf_list, struct nanohub_buf, list);
		list_del(&buf->list);
	}
	spin_unlock(&io->buf_wait.lock);

	return buf;
}

static void nanohub_io_put_buf(struct nanohub_io *io,
			       struct nanohub_buf *buf)
{
	bool was_empty;

	spin_lock(&io->buf_wait.lock);
	was_empty = !nanohub_io_has_buf(io);
	list_add_tail(&buf->list, &io->buf_list);
	spin_unlock(&io->buf_wait.lock);

	if (was_empty) {
		if (&io->data->free_pool == io)
			nanohub_notify_thread(io->data);
		else
			wake_up_interruptible(&io->buf_wait);
	}
}

static inline int plat_gpio_get(struct nanohub_data *data,
				const struct gpio_config *_cfg)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	return *(u32 *)(((char *)pdata) + (_cfg)->pdata_off);
}

static inline void nanohub_set_irq_data(struct nanohub_data *data,
					const struct gpio_config *_cfg, int val)
{
	int *data_addr = ((int *)(((char *)data) + _cfg->data_off));

	if ((void *)data_addr > (void *)data &&
	    (void *)data_addr < (void *)(data + 1))
		*data_addr = val;
	else
		WARN(1, "No data binding defined for %s", _cfg->label);
}

static inline void mcu_wakeup_gpio_set_value(struct nanohub_data *data,
					     int val)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	gpio_set_value(pdata->wakeup_gpio, val);
}

static inline void mcu_wakeup_gpio_get_locked(struct nanohub_data *data,
					      int priority_lock)
{
	atomic_inc(&data->wakeup_lock_cnt);
	if (!priority_lock && atomic_inc_return(&data->wakeup_cnt) == 1 &&
	    !nanohub_has_priority_lock_locked(data))
		mcu_wakeup_gpio_set_value(data, 0);
}

static inline bool mcu_wakeup_gpio_put_locked(struct nanohub_data *data,
					      int priority_lock)
{
	bool gpio_done = priority_lock ?
			 atomic_read(&data->wakeup_cnt) == 0 :
			 atomic_dec_and_test(&data->wakeup_cnt);
	bool done = atomic_dec_and_test(&data->wakeup_lock_cnt);

	if (!nanohub_has_priority_lock_locked(data))
		mcu_wakeup_gpio_set_value(data, gpio_done ? 1 : 0);

	return done;
}

static inline bool mcu_wakeup_gpio_is_locked(struct nanohub_data *data)
{
	return atomic_read(&data->wakeup_lock_cnt) != 0;
}

static inline void nanohub_handle_irq1(struct nanohub_data *data)
{
	bool locked;

	spin_lock(&data->wakeup_wait.lock);
	locked = mcu_wakeup_gpio_is_locked(data);
	spin_unlock(&data->wakeup_wait.lock);
	if (!locked)
		nanohub_notify_thread(data);
	else
		wake_up_interruptible_sync(&data->wakeup_wait);
}

static inline void nanohub_handle_irq2(struct nanohub_data *data)
{
	nanohub_notify_thread(data);
}

static inline bool mcu_wakeup_try_lock(struct nanohub_data *data, int key)
{
	/* implementation contains memory barrier */
	return atomic_cmpxchg(&data->wakeup_acquired, 0, key) == 0;
}

static inline void mcu_wakeup_unlock(struct nanohub_data *data, int key)
{
	WARN(atomic_cmpxchg(&data->wakeup_acquired, key, 0) != key,
	     "%s: failed to unlock with key %d; current state: %d",
	     __func__, key, atomic_read(&data->wakeup_acquired));
}

static inline void nanohub_set_state(struct nanohub_data *data, int state)
{
	atomic_set(&data->thread_state, state);
	smp_mb__after_atomic(); /* updated thread state is now visible */
}

static inline int nanohub_get_state(struct nanohub_data *data)
{
	smp_mb__before_atomic(); /* wait for all updates to finish */
	return atomic_read(&data->thread_state);
}

static inline void nanohub_clear_err_cnt(struct nanohub_data *data)
{
	data->kthread_err_cnt = data->wakeup_err_cnt = 0;
}

static long do_wait_intr_timeout(wait_queue_head_t *wq,
			wait_queue_entry_t *wait, long tmo)
{
	long ret;

	if (likely(list_empty(&wait->entry)))
		__add_wait_queue_entry_tail(wq, wait);

	set_current_state(TASK_INTERRUPTIBLE);
	if (signal_pending(current))
		return -ERESTARTSYS;

	spin_unlock(&wq->lock);
	ret = schedule_timeout(tmo);
	spin_lock(&wq->lock);
	return ret;
}

/* the following fragment is based on wait_event_* code from wait.h */
#define wait_event_interruptible_timeout_locked(wq, cond, tmo)		\
({									\
	long __ret = (tmo);						\
	DEFINE_WAIT(__wait);						\
	if (!(cond)) {							\
		do {							\
			__ret = do_wait_intr_timeout(&(wq), &__wait, __ret); \
			if (!__ret) {					\
				if ((cond))				\
					__ret = 1;			\
				break;					\
			} else if (__ret < 0)				\
				break;					\
		} while (!(cond));					\
		__remove_wait_queue(&(wq), &__wait);			\
		__set_current_state(TASK_RUNNING);			\
	} else if (__ret == 0) {					\
		__ret = 1;						\
	}								\
	__ret;								\
})

int request_wakeup_ex(struct nanohub_data *data, long timeout_ms,
		      int key, int lock_mode)
{
	long timeout;
	bool priority_lock = lock_mode > LOCK_MODE_NORMAL;
	struct device *dev = data->io[ID_NANOHUB_COMMS].dev;
	ktime_t ktime_delta;
	ktime_t wakeup_ktime;

	spin_lock(&data->wakeup_wait.lock);
	mcu_wakeup_gpio_get_locked(data, priority_lock);
	timeout = (timeout_ms != MAX_SCHEDULE_TIMEOUT) ?
		   msecs_to_jiffies(timeout_ms) :
		   MAX_SCHEDULE_TIMEOUT;

	if (!priority_lock && !data->wakeup_err_cnt)
		wakeup_ktime = ktime_get_boottime();
	timeout = wait_event_interruptible_timeout_locked(
			data->wakeup_wait,
			((priority_lock || nanohub_irq1_fired(data)) &&
			 mcu_wakeup_try_lock(data, key)),
			timeout
		  );

	if (timeout <= 0) {
		if (!timeout && !priority_lock) {
			if (!data->wakeup_err_cnt)
				data->wakeup_err_ktime = wakeup_ktime;
			ktime_delta = ktime_sub(ktime_get_boottime(),
						data->wakeup_err_ktime);
			data->wakeup_err_cnt++;
			if (ktime_to_ns(ktime_delta) > WAKEUP_ERR_TIME_NS
				&& data->wakeup_err_cnt > WAKEUP_ERR_CNT) {
				mcu_wakeup_gpio_put_locked(data, priority_lock);
				spin_unlock(&data->wakeup_wait.lock);
				dev_info(dev, "wakeup: consistent error\n");
				// dev_info(dev,
				// 	"wakeup: hard reset due to consistent error\n");
				// ret = nanohub_hw_reset(data);
				// if (ret) {
				// 	dev_info(dev,
				// 		"%s: failed to reset nanohub: ret=%d\n",
				// 		__func__, ret);
				// }
				return -ETIME;
			}
		}
		mcu_wakeup_gpio_put_locked(data, priority_lock);

		if (timeout == 0)
			timeout = -ETIME;
	} else {
		data->wakeup_err_cnt = 0;
		timeout = 0;
	}
	spin_unlock(&data->wakeup_wait.lock);

	return timeout;
}

void release_wakeup_ex(struct nanohub_data *data, int key, int lock_mode)
{
	bool done;
	bool priority_lock = lock_mode > LOCK_MODE_NORMAL;

	spin_lock(&data->wakeup_wait.lock);
	done = mcu_wakeup_gpio_put_locked(data, priority_lock);
	mcu_wakeup_unlock(data, key);
	spin_unlock(&data->wakeup_wait.lock);

	if (!done)
		wake_up_interruptible_sync(&data->wakeup_wait);
	else if (nanohub_irq1_fired(data) || nanohub_irq2_fired(data))
		nanohub_notify_thread(data);
}

int nanohub_wait_for_interrupt(struct nanohub_data *data)
{
	int ret = -EFAULT;

	/* release the wakeup line, and wait for nanohub to send
	 * us an interrupt indicating the transaction completed.
	 */
	spin_lock(&data->wakeup_wait.lock);
	if (mcu_wakeup_gpio_is_locked(data)) {
		mcu_wakeup_gpio_set_value(data, 1);
		ret = wait_event_interruptible_locked(data->wakeup_wait,
						      nanohub_irq1_fired(data));
		mcu_wakeup_gpio_set_value(data, 0);
	}
	spin_unlock(&data->wakeup_wait.lock);

	return ret;
}

int nanohub_wakeup_eom(struct nanohub_data *data, bool repeat)
{
	int ret = -EFAULT;

	spin_lock(&data->wakeup_wait.lock);
	if (mcu_wakeup_gpio_is_locked(data)) {
		mcu_wakeup_gpio_set_value(data, 1);
		if (repeat) {
			udelay(RETRY_INT_WIDTH_US);
			mcu_wakeup_gpio_set_value(data, 0);
		}
		ret = 0;
	}
	spin_unlock(&data->wakeup_wait.lock);

	return ret;
}

static void __nanohub_interrupt_cfg(struct nanohub_data *data,
				    u8 interrupt, bool mask)
{
	int ret;
	uint8_t mask_ret;
	int cnt = 10;
	struct device *dev = data->io[ID_NANOHUB_COMMS].dev;
	int cmd = mask ? CMD_COMMS_MASK_INTR : CMD_COMMS_UNMASK_INTR;

	// interrupt config is not supported in r11, keeping for later use
	if (true)
		return;

	do {
		ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
		if (ret) {
			dev_err(dev,
				"%s: interrupt %d %smask failed: ret=%d\n",
				__func__, interrupt, mask ? "" : "un", ret);
			return;
		}

		ret =
		    nanohub_comms_tx_rx_retrans(data, cmd, ID_NANOHUB_COMMS,
						&interrupt, sizeof(interrupt),
						NULL, &mask_ret,
						sizeof(mask_ret), false, 10, 0);
		release_wakeup(data);
		dev_dbg(dev,
			"%smasking interrupt %d, ret=%d, mask_ret=%d\n",
			mask ? "" : "un",
			interrupt, ret, mask_ret);
	} while ((ret != 1 || mask_ret != 1) && --cnt > 0);
}

static inline void nanohub_mask_interrupt(struct nanohub_data *data,
					  u8 interrupt)
{
	__nanohub_interrupt_cfg(data, interrupt, true);
	return;
}

static inline void nanohub_unmask_interrupt(struct nanohub_data *data,
					    u8 interrupt)
{
	__nanohub_interrupt_cfg(data, interrupt, false);
	return;
}

static ssize_t nanohub_wakeup_query(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;

	nanohub_clear_err_cnt(data);
	if (nanohub_irq1_fired(data) || nanohub_irq2_fired(data))
		wake_up_interruptible(&data->wakeup_wait);

	return scnprintf(buf, PAGE_SIZE, "WAKEUP: %d INT1: %d INT2: %d\n",
			 gpio_get_value(pdata->wakeup_gpio),
			 gpio_get_value(pdata->irq1_gpio),
			 data->irq2 ? gpio_get_value(pdata->irq2_gpio) : -1);
}

static ssize_t nanohub_pin_nreset_get(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->nreset_gpio));
}

static ssize_t nanohub_pin_boot0_get(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->boot0_gpio));
}

static ssize_t nanohub_pin_boot2_get(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->boot2_gpio));
}

static ssize_t nanohub_pin_nreset_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	if (count >= 1 && buf[0] == '0') {
		gpio_set_value(pdata->nreset_gpio, 0);
	} else if (count >= 1 && buf[0] == '1') {
		gpio_set_value(pdata->nreset_gpio, 1);
	}
	return count;
}

static ssize_t nanohub_pin_boot0_set(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	if (count >= 1 && buf[0] == '0') {
		gpio_set_value(pdata->boot0_gpio, 0);
	} else if (count >= 1 && buf[0] == '1') {
		gpio_set_value(pdata->boot0_gpio, 1);
	}
	return count;
}

static ssize_t nanohub_pin_boot2_set(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	if (count >= 1 && buf[0] == '0') {
		gpio_set_value(pdata->boot2_gpio, 0);
	} else if (count >= 1 && buf[0] == '1') {
		gpio_set_value(pdata->boot2_gpio, 1);
	}
	return count;
}

static ssize_t nanohub_app_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	struct {
		uint64_t appId;
		uint32_t appVer;
		uint32_t appSize;
	} __packed buffer;
	uint32_t i = 0;
	int ret;
	ssize_t len = 0;

	do {
		if (request_wakeup(data))
			return -ERESTARTSYS;

		if (nanohub_comms_tx_rx_retrans
		    (data, CMD_COMMS_QUERY_APP_INFO, ID_NANOHUB_COMMS,
		     (uint8_t *)&i, sizeof(i), NULL, (u8 *)&buffer,
		     sizeof(buffer), false, 10, 10) == sizeof(buffer)) {
			ret =
			    scnprintf(buf + len, PAGE_SIZE - len,
				      "app: %d id: %016llx ver: %08x size: %08x\n",
				      i, buffer.appId, buffer.appVer,
				      buffer.appSize);
			if (ret > 0) {
				len += ret;
				i++;
			}
		} else {
			ret = -1;
		}

		release_wakeup(data);
	} while (ret > 0);

	return len;
}

static ssize_t nanohub_firmware_query(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	uint16_t buffer[6];

	if (request_wakeup(data))
		return -ERESTARTSYS;

	if (nanohub_comms_tx_rx_retrans
	    (data, CMD_COMMS_GET_OS_HW_VERSIONS, ID_NANOHUB_COMMS, NULL,
	     0, NULL, (uint8_t *)&buffer, sizeof(buffer), false, 10,
	     10) == sizeof(buffer)) {
		release_wakeup(data);
		return scnprintf(buf, PAGE_SIZE,
				 "hw type: %04x hw ver: %04x bl ver: %04x os ver: %04x variant ver: %08x\n",
				 buffer[0], buffer[1], buffer[2], buffer[3],
				 buffer[5] << 16 | buffer[4]);
	} else {
		release_wakeup(data);
		return 0;
	}
}

static ssize_t nanohub_time_sync(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	struct nanohub_time_sync {
		uint64_t ap_time_ns;
		uint64_t mcu_time_ns;
	} __packed time_sync_result;
	int ret;

	if (request_wakeup(data))
		return -ERESTARTSYS;

	ret = nanohub_comms_rx_retrans_boottime
	    (data, CMD_COMMS_TIME_SYNC, ID_NANOHUB_COMMS,
	     NULL, (uint8_t *)&time_sync_result, sizeof(time_sync_result),
	     10, 10);

	release_wakeup(data);
	if (ret == sizeof(time_sync_result)) {
		return scnprintf(buf, PAGE_SIZE,
				 "ap time: %llu mcu time: %llu\n",
				 time_sync_result.ap_time_ns, time_sync_result.mcu_time_ns);
	} else {
		return scnprintf(buf, PAGE_SIZE, "time sync error: %d\n", ret);
	}
}

static ssize_t nanohub_mcu_wake_lock(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	uint8_t lock;
	uint8_t result;
	int ret;

	if (count >= 1 && buf[0] == '0') {
		lock = 0;
	} else if (count >= 1 && buf[0] == '1') {
		lock = 1;
	} else {
		return count;
	}

	if (request_wakeup(data))
		return -ERESTARTSYS;

	ret = nanohub_comms_tx_rx_retrans(
		data, CMD_COMMS_MCU_WAKE_LOCK, ID_NANOHUB_COMMS,
		(uint8_t *)&lock, sizeof(lock), NULL,
		(uint8_t *)&result, sizeof(result), false, 10, 10);

	release_wakeup(data);
	return count;
}

static inline int nanohub_wakeup_lock(struct nanohub_data *data, int mode)
{
	int ret;

	if (data->irq2)
		disable_irq(data->irq2);
	else
		nanohub_mask_interrupt(data, 2);

	ret = request_wakeup_ex(data,
				mode == LOCK_MODE_SUSPEND_RESUME ?
				SUSPEND_TIMEOUT_MS : WAKEUP_TIMEOUT_MS,
				KEY_WAKEUP_LOCK, mode);
	if (ret < 0) {
		if (data->irq2)
			enable_irq(data->irq2);
		else
			nanohub_unmask_interrupt(data, 2);
		return ret;
	}

#if defined(CONFIG_NANOHUB_BL_ST) || defined(CONFIG_NANOHUB_BL_NXP)
	if (mode == LOCK_MODE_IO || mode == LOCK_MODE_IO_BL)
		ret = nanohub_bl_open(data);
	if (ret < 0) {
		release_wakeup_ex(data, KEY_WAKEUP_LOCK, mode);
		return ret;
	}
#endif
	if (mode != LOCK_MODE_SUSPEND_RESUME)
		disable_irq(data->irq1);

	atomic_set(&data->lock_mode, mode);
	mcu_wakeup_gpio_set_value(data, mode != LOCK_MODE_IO_BL);

	return 0;
}

/* returns lock mode used to perform this lock */
static inline int nanohub_wakeup_unlock(struct nanohub_data *data)
{
	int mode = atomic_read(&data->lock_mode);

	atomic_set(&data->lock_mode, LOCK_MODE_NONE);
	if (mode != LOCK_MODE_SUSPEND_RESUME)
		enable_irq(data->irq1);
#if defined(CONFIG_NANOHUB_BL_ST) || defined(CONFIG_NANOHUB_BL_NXP)
	if (mode == LOCK_MODE_IO || mode == LOCK_MODE_IO_BL)
		nanohub_bl_close(data);
#endif
	if (data->irq2)
		enable_irq(data->irq2);
	release_wakeup_ex(data, KEY_WAKEUP_LOCK, mode);
	if (!data->irq2)
		nanohub_unmask_interrupt(data, 2);
	nanohub_notify_thread(data);

	return mode;
}

// boot_mode represents the ISP mode for the NXP MCU.
static void __nanohub_hw_reset(struct nanohub_data *data, int boot_mode)
{
	const struct nanohub_platform_data *pdata = data->pdata;
	int isp0 = boot_mode & 0x1;
	int isp1 = (boot_mode & 0x2) >> 1;
	int isp2 = (boot_mode & 0x4) >> 2;
	int ret;

	// Set the ISP mode
	if (isp1 != 1) {
		// ISP1 is not connected and is pulled high in ISP mode.
		pr_err("nanohub_hw_reset: invalid boot mode (%d), ISP pin1 must be high\n",
		       boot_mode);
		return;
	}
	pr_info("nanohub_hw_reset: boot_mode=%d\n", boot_mode);
	gpio_set_value(pdata->boot0_gpio, isp0);
	gpio_set_value(pdata->boot2_gpio, isp2);

	// Disable AFE_TX_SUP prior to MCU boot
	ret = regmap_write(pdata->pmic_regmap, pdata->afe_control_reg, 0);
	if (ret != 0)
		pr_warn("nanohub: failed to disable afe_tx_sup ret=%x\n", ret);

	// Disable BOB EXT CTRL1 prior to MCU boot
	ret = regmap_write(pdata->pmic_regmap, pdata->bob_ext_ctrl1_control_reg, 0);
	if (ret != 0)
		pr_warn("nanohub: failed to disable bob ext ctrl1 ret=%x\n", ret);

	// Disable 32KHz clock prior to MCU boot
	ret = regmap_write(pdata->pmic_regmap, pdata->clk32_control_reg, 0);
	if (ret != 0)
		pr_warn("nanohub: failed to disable 32KHz clock ret=%x\n", ret);

	// Reset MCU
	gpio_set_value(pdata->nreset_gpio, 0);
	usleep_range(1000, 2000);

	// Reset MCU vddcore voltage after reset, but prior to MCU boot
	ret = regmap_write(pdata->pmic_regmap, pdata->vddcore_control_reg,
			   pdata->vddcore_voltage / 1000 & 0xff);
	if (ret == 0)
		ret = regmap_write(pdata->pmic_regmap,
				   pdata->vddcore_control_reg + 1,
				   pdata->vddcore_voltage / 1000 >> 8);
	if (ret != 0)
		pr_warn("nanohub: failed to reset vddcore ret=%x\n", ret);
        // Make sure MCU vddcore rail is enabled since the MCU can disable it
	ret = regmap_write(pdata->pmic_regmap, pdata->vddcore_enable_reg, 0x80);
	if (ret != 0)
		pr_warn("nanohub: failed to enable MCU vddcore ret=%x\n", ret);
	usleep_range(1000, 2000);

	// Force S5A PGOOD high prior to MCU boot (Work-around for QCOM Case #06525336)
	ret = regmap_write(pdata->pmic_regmap, pdata->s5a_pgood_state_reg, 0x80);
	if (ret != 0)
		pr_warn("nanohub: failed to set S5A PGOOD state ret=%x\n", ret);

        // Enable external regulator (if present)
	if (gpio_is_valid(pdata->reg_en_gpio))
		gpio_set_value(pdata->reg_en_gpio, 1);

	gpio_set_value(pdata->nreset_gpio, 1);
	usleep_range(14000, 24000);
	nanohub_clear_err_cnt(data);
}

static int nanohub_hw_reset(struct nanohub_data *data, int boot_mode)
{
	int ret;
	ret = nanohub_wakeup_lock(data, LOCK_MODE_RESET);

	if (!ret) {
		__nanohub_hw_reset(data, boot_mode);
		nanohub_wakeup_unlock(data);
	}

	return ret;
}

// Configure PM_GPIO_05 to function as PMIC_IRQ
static int nanohub_pmic_irq_config(struct regmap *pmic_regmap)
{
	int ret;
	// Command seqeuence provided by QCOM in Case #06209787
	// GPIO05 kernel write permission enabled by gpar/529000
	ret = regmap_write(pmic_regmap, GPIO05_INT_SET_TYPE, 0x1);
	ret |= regmap_write(pmic_regmap, GPIO05_INT_POLARITY_HIGH, 0x1);
	ret |= regmap_write(pmic_regmap, GPIO05_INT_EN_SET, 0x1);
	ret |= regmap_write(pmic_regmap, GPIO05_INT_EN_CLR, 0x1);
	ret |= regmap_write(pmic_regmap, GPIO05_OUTPUT_CTRL, 0x82);
	if (ret) {
		pr_warn("nanohob: a PMIC_IRQ write operation failed, ret=%x\n", ret);
	}

	return ret;
}

static ssize_t nanohub_try_hw_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	#define BOOT_MODE_ARG_SIZE 10
	char boot_mode_buf[BOOT_MODE_ARG_SIZE + 1];
	int boot_mode;
	int ret;

	// Read the boot mode argument. (ascii, possibly unterminated)
	if (count == 0 || count > BOOT_MODE_ARG_SIZE) {
		pr_err("nanohub_try_hw_reset: invalid input\n");
		return -EINVAL;
	}
	memcpy(boot_mode_buf, buf, count);
	boot_mode_buf[count] = 0;
	if (kstrtoint(boot_mode_buf, 0, &boot_mode) != 0) {
		pr_err("nanohub_try_hw_reset: invalid boot mode %s\n",
		       boot_mode_buf);
		return -EINVAL;
	}

	ret = nanohub_hw_reset(data, boot_mode);

	return ret < 0 ? ret : count;
}

#ifdef CONFIG_NANOHUB_BL_ST
static ssize_t nanohub_erase_shared(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	uint8_t status = CMD_ACK;
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	status = nanohub_bl_erase_shared(data);
	dev_info(dev, "nanohub_bl_erase_shared: status=%02x\n",
		 status);

	__nanohub_hw_reset(data, 0);
	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_erase_shared_bl(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	uint8_t status = CMD_ACK;
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO_BL);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, -1);

	status = nanohub_bl_erase_shared_bl(data);
	dev_info(dev, "%s: status=%02x\n", __func__, status);

	__nanohub_hw_reset(data, 0);
	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_download_bl(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	const struct firmware *fw_entry;
	int ret;
	uint8_t status = CMD_ACK;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	ret = request_firmware(&fw_entry, "nanohub.full.bin", dev);
	if (ret) {
		dev_err(dev, "%s: err=%d\n", __func__, ret);
	} else {
		status = nanohub_bl_download(data, pdata->bl_addr,
					     fw_entry->data, fw_entry->size);
		dev_info(dev, "%s: status=%02x\n", __func__, status);
		release_firmware(fw_entry);
	}

	__nanohub_hw_reset(data, 0);
	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}
#endif

static ssize_t nanohub_download_kernel(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	int ret;

	ret = request_firmware(&fw_entry, "nanohub.update.bin", dev);
	if (ret) {
		dev_err(dev, "nanohub_download_kernel: err=%d\n", ret);
		return -EIO;
	} else {
		ret =
		    nanohub_comms_kernel_download(data, fw_entry->data,
						  fw_entry->size);

		release_firmware(fw_entry);

		return count;
	}

}

#ifdef CONFIG_NANOHUB_BL_ST
static ssize_t nanohub_download_kernel_bl(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	int ret;
	uint8_t status = CMD_ACK;

	ret = request_firmware(&fw_entry, "nanohub.kernel.signed", dev);
	if (ret) {
		dev_err(dev, "%s: err=%d\n", __func__, ret);
	} else {
		ret = nanohub_wakeup_lock(data, LOCK_MODE_IO_BL);
		if (ret < 0)
			return ret;

		__nanohub_hw_reset(data, -1);

		status = nanohub_bl_erase_shared_bl(data);
		dev_info(dev, "%s: (erase) status=%02x\n", __func__, status);
		if (status == CMD_ACK) {
			status = nanohub_bl_write_memory(data, 0x50000000,
							 fw_entry->size,
							 fw_entry->data);
			mcu_wakeup_gpio_set_value(data, 1);
			dev_info(dev, "%s: (write) status=%02x\n", __func__, status);
			if (status == CMD_ACK) {
				status = nanohub_bl_update_finished(data);
				dev_info(dev, "%s: (finish) status=%02x\n", __func__, status);
			}
		} else {
			mcu_wakeup_gpio_set_value(data, 1);
		}

		__nanohub_hw_reset(data, 0);
		nanohub_wakeup_unlock(data);

		release_firmware(fw_entry);
	}

	return ret < 0 ? ret : count;
}
#endif

static ssize_t nanohub_firmware_name_query(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct nanohub_platform_data *pdata = data->pdata;

	if (pdata->firmware_name) {
		strcpy(buf, pdata->firmware_name);
		return strlen(pdata->firmware_name);
	}
	return 0;
}

#ifdef CONFIG_NANOHUB_BL_NXP

static ssize_t nanohub_download_firmware_request(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	struct firmware_request *fw_request = &data->firmware_request;
	char *fw_name = fw_request->fw_name;
	uint32_t* fw_image_offset = &fw_request->fw_image_offset;
	int i, ret;

	if (atomic_cmpxchg(&fw_request->state, FW_IDLE, FW_PENDING) !=
	    FW_IDLE) {
		ret = -EBUSY;
		goto out;
	}

	if (count == 0 || count >= FW_NAME_SIZE) {
		ret = -ENAMETOOLONG;
		goto out;
	}

	// Handle \n or unterminated name.
	memcpy(fw_name, buf, count);
	if (fw_name[count - 1] == '\n')
		fw_name[count - 1] = 0;
	fw_name[count] = 0;

	// Parse firmware name then firmware image offset which are expected
	// to be separated by a space. Offset is optional.
	*fw_image_offset = 0;
	for (i = 0; i < strlen(fw_name); i++) {
		if (fw_name[i] == ' ') {
			ret = kstrtou32(fw_name + i + 1, 10, fw_image_offset);
			fw_name[i] = 0;
			break;
		}
	}

	atomic_set(&fw_request->state, FW_REQUESTED);
	nanohub_notify_thread(data);
	wait_for_completion(&fw_request->fw_complete);
	ret = fw_request->result;

out:
	if (ret < 0)
		pr_err("nanohub: firmware request failed: %s err=%d\n", fw_name,
		       ret);

	atomic_set(&fw_request->state, FW_IDLE);
	return ret < 0 ? ret : count;
}

static void nanohub_download_firmware(struct device *dev)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	struct firmware_request *fw_request = &data->firmware_request;
	const struct firmware *fw_entry;
	int ret;

	pr_info("nanohub: firmware download: %s\n", fw_request->fw_name);
	pr_info("nanohub: firmware image offset: %lu\n",
		fw_request->fw_image_offset);

	atomic_set(&fw_request->state, FW_IN_PROGRESS);
	ret = request_firmware(&fw_entry, fw_request->fw_name, dev);
	if (ret < 0)
		goto out;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO_BL);
	if (ret < 0)
		goto release_firmware;

	// Disable irq1 as its GPIO is used by the bootloader.
	if (data->irq1)
		disable_irq(data->irq1);

	// FW download uses SERIAL_ISP mode 110.
	__nanohub_hw_reset(data, 0b110);

	if (fw_request->fw_image_offset != 0) {
		ret = nanohub_bl_download_firmware_buffer(
		    data, fw_entry->data + fw_request->fw_image_offset,
		    fw_entry->size - fw_request->fw_image_offset);
	} else {
		ret = nanohub_bl_download_firmware(data, fw_entry->data,
						   fw_entry->size);
	}

	mcu_wakeup_gpio_set_value(data, 1);
	if (data->irq1)
		enable_irq(data->irq1);
	nanohub_wakeup_unlock(data);

release_firmware:
	release_firmware(fw_entry);
out:
	pr_info("nanohub: firmware download complete: %s err=%d\n",
		fw_request->fw_name, ret);
	fw_request->result = ret;
	atomic_set(&fw_request->state, FW_COMPLETE);
	complete(&fw_request->fw_complete);
}

#endif

static ssize_t nanohub_download_app(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	const struct firmware *fw_entry;
	char buffer[70];
	int i, ret, ret1, ret2, file_len = 0, appid_len = 0, ver_len = 0;
	const char *appid = NULL, *ver = NULL;
	unsigned long version;
	uint64_t id;
	uint32_t cur_version;
	bool update = true;

	for (i = 0; i < count; i++) {
		if (buf[i] == ' ') {
			if (i + 1 == count) {
				break;
			} else {
				if (appid == NULL)
					appid = buf + i + 1;
				else if (ver == NULL)
					ver = buf + i + 1;
				else
					break;
			}
		} else if (buf[i] == '\n' || buf[i] == '\r') {
			break;
		} else {
			if (ver)
				ver_len++;
			else if (appid)
				appid_len++;
			else
				file_len++;
		}
	}

	if (file_len > 64 || appid_len > 16 || ver_len > 8 || file_len < 1)
		return -EIO;

	memcpy(buffer, buf, file_len);
	memcpy(buffer + file_len, ".napp", 5);
	buffer[file_len + 5] = '\0';

	ret = request_firmware(&fw_entry, buffer, dev);
	if (ret) {
		dev_err(dev, "nanohub_download_app(%s): err=%d\n",
			buffer, ret);
		return -EIO;
	}
	if (appid_len > 0 && ver_len > 0) {
		memcpy(buffer, appid, appid_len);
		buffer[appid_len] = '\0';

		ret1 = kstrtoull(buffer, 16, &id);

		memcpy(buffer, ver, ver_len);
		buffer[ver_len] = '\0';

		ret2 = kstrtoul(buffer, 16, &version);

		if (ret1 == 0 && ret2 == 0) {
			if (request_wakeup(data))
				return -ERESTARTSYS;
			if (nanohub_comms_tx_rx_retrans
			    (data, CMD_COMMS_GET_APP_VERSIONS, ID_NANOHUB_COMMS,
			     (uint8_t *)&id, sizeof(id), NULL,
			     (uint8_t *)&cur_version,
			     sizeof(cur_version), false, 10,
			     10) == sizeof(cur_version)) {
				if (cur_version == version)
					update = false;
			}
			release_wakeup(data);
		}
	}

	if (update)
		ret =
		    nanohub_comms_app_download(data, fw_entry->data,
					       fw_entry->size);

	release_firmware(fw_entry);

	return count;
}

#ifdef CONFIG_NANOHUB_BL_ST
static ssize_t nanohub_lock_bl(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	int ret;
	uint8_t status = CMD_ACK;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	gpio_set_value(data->pdata->boot0_gpio, 0);
	/* this command reboots itself */
	status = nanohub_bl_lock(data);
	dev_info(dev, "%s: status=%02x\n", __func__, status);
	msleep(350);

	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}

static ssize_t nanohub_unlock_bl(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	int ret;
	uint8_t status = CMD_ACK;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_IO);
	if (ret < 0)
		return ret;

	__nanohub_hw_reset(data, 1);

	gpio_set_value(data->pdata->boot0_gpio, 0);
	/* this command reboots itself (erasing the flash) */
	status = nanohub_bl_unlock(data);
	dev_info(dev, "%s: status=%02x\n", __func__, status);
	msleep(20);

	nanohub_wakeup_unlock(data);

	return ret < 0 ? ret : count;
}
#endif

static ssize_t nanohub_wakeup_event_msec_get(struct device *dev,
					     struct device_attribute *attr, char *buf)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", data->wakeup_event_msec);
}

static ssize_t nanohub_wakeup_event_msec_set(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct nanohub_data *data = dev_get_drvdata(dev);
	u32 value;

	if (kstrtou32(buf, 0, &value))
		return -EINVAL;

	data->wakeup_event_msec = value;
	return count;
}

static struct device_attribute attributes[] = {
	__ATTR(wakeup, 0440, nanohub_wakeup_query, NULL),
	__ATTR(app_info, 0440, nanohub_app_info, NULL),
	__ATTR(firmware_version, 0440, nanohub_firmware_query, NULL),
	__ATTR(time_sync, 0440, nanohub_time_sync, NULL),
	__ATTR(wake_lock, 0220, NULL, nanohub_mcu_wake_lock),
#ifdef CONFIG_NANOHUB_BL_ST
	__ATTR(download_bl, 0220, NULL, nanohub_download_bl),
#endif
	__ATTR(download_kernel, 0220, NULL, nanohub_download_kernel),
#ifdef CONFIG_NANOHUB_BL_ST
	__ATTR(download_kernel_bl, 0220, NULL, nanohub_download_kernel_bl),
#endif
	__ATTR(download_app, 0220, NULL, nanohub_download_app),
#ifdef CONFIG_NANOHUB_BL_ST
	__ATTR(erase_shared, 0220, NULL, nanohub_erase_shared),
	__ATTR(erase_shared_bl, 0220, NULL, nanohub_erase_shared_bl),
#endif
	__ATTR(hw_reset, 0220, NULL, nanohub_try_hw_reset),
	__ATTR(nreset, 0660, nanohub_pin_nreset_get, nanohub_pin_nreset_set),
	__ATTR(boot0, 0660, nanohub_pin_boot0_get, nanohub_pin_boot0_set),
	__ATTR(boot2, 0660, nanohub_pin_boot2_get, nanohub_pin_boot2_set),
#ifdef CONFIG_NANOHUB_BL_ST
	__ATTR(lock, 0220, NULL, nanohub_lock_bl),
	__ATTR(unlock, 0220, NULL, nanohub_unlock_bl),
#endif
	__ATTR(firmware_name, 0440, nanohub_firmware_name_query, NULL),
#ifdef CONFIG_NANOHUB_BL_NXP
	__ATTR(download_firmware, 0220, NULL, nanohub_download_firmware_request),
#endif
	__ATTR(wakeup_event_msec, 0660, nanohub_wakeup_event_msec_get,
	       nanohub_wakeup_event_msec_set),
};

static inline int nanohub_create_sensor(struct nanohub_data *data)
{
	int i, ret;
	struct device *dev = data->io[ID_NANOHUB_COMMS].dev;

	for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(dev, &attributes[i]);
		if (ret) {
			dev_err(dev,
				"create sysfs attr %d [%s] failed; err=%d\n",
				i, attributes[i].attr.name, ret);
			goto fail_attr;
		}
	}

	ret = sysfs_create_link(&dev->kobj,
				&data->iio_dev->dev.kobj, "iio");
	if (ret) {
		dev_err(dev,
			"sysfs_create_link failed; err=%d\n", ret);
		goto fail_attr;
	}
	goto done;

fail_attr:
	for (i--; i >= 0; i--)
		device_remove_file(dev, &attributes[i]);
done:
	return ret;
}

static int nanohub_create_devices(struct nanohub_data *data)
{
	int i, j, k, ret;
	static const struct device_config device_configs[] = {
		{ "nanohub_comms", 1, false },
		{ "nanohub", ID_NANOHUB_CLIENT_NUM_IDS, false },
		{ "nanohub_audio", 1, true },
		{ "nanohub_display", 1, false },
		{ "nanohub_render", 1, false },
		{ "nanohub_debug_log", 1, false },
		{ "nanohub_metrics", 1, false },
		{ "nanohub_console", 1, false },
		{ "nanohub_rpc0", 1, false },
		{ "nanohub_rpc1", 1, false },
		{ "nanohub_brightness", 1, false },
		{ "nanohub_touch", 1, true },
		{ "nanohub_display_kernel", 1, true },
		{ "nanohub_pele", 1, false },
		{ "nanohub_bt", 1, false },
		{ "nanohub_touch_kernel", 1, true},
	};
	static_assert(ARRAY_SIZE(device_configs) == ID_NANOHUB_MAX - ID_NANOHUB_CLIENT_NUM_IDS + 1);

	for (i = 0, j = 0; j < ID_NANOHUB_MAX - ID_NANOHUB_CLIENT_NUM_IDS + 1; ++j) {
		struct device *dev = NULL;
		if (!device_configs[j].is_kernel) {
			dev = device_create(sensor_class, NULL, MKDEV(major, i), data,
						device_configs[j].name);
			if (IS_ERR(dev)) {
				ret = PTR_ERR(dev);
				pr_err("nanohub: device_create failed for %s; err=%d\n",
					device_configs[j].name, ret);
				goto fail_dev;
			}
		}

		for (k = 0; k < device_configs[j].channels; ++i, ++k) {
			struct nanohub_io *io = &data->io[i];

			nanohub_io_init(io, data, dev, i, device_configs[j].is_kernel);
		}
	}

	ret = nanohub_create_sensor(data);
	if (!ret)
		goto done;

fail_dev:
	for (--i; i >= 0; --i)
		device_destroy(sensor_class, MKDEV(major, i));
done:
	return ret;
}

static int nanohub_match_devt(struct device *dev, const void *data)
{
	const dev_t *devt = data;

	return dev->devt == *devt;
}

static int nanohub_open(struct inode *inode, struct file *file)
{
	dev_t devt = inode->i_rdev;
	struct device *dev;
	int i;

	dev = class_find_device(sensor_class, NULL, &devt, nanohub_match_devt);
	if (dev) {
		struct nanohub_data *data = dev_get_drvdata(dev);
		if (MINOR(devt) == ID_NANOHUB_CLIENT) {
			for (i = 0; i<ID_NANOHUB_CLIENT_NUM_IDS; ++i) {
				struct nanohub_io *io =
					&data->io[ID_NANOHUB_CLIENT + i];
				if (atomic_cmpxchg(&io->busy, 0, 1) == 0) {
					file->private_data = io;
					nonseekable_open(inode, file);
					return 0;
				}
			}
			return -EBUSY;
		} else {
			file->private_data = &data->io[MINOR(devt)];
			nonseekable_open(inode, file);
			return 0;
		}
	}

	return -ENODEV;
}

static ssize_t nanohub_read(struct file *file, char *buffer, size_t length,
			    loff_t *offset)
{
	struct nanohub_io *io = file->private_data;
	struct nanohub_data *data = io->data;
	struct nanohub_buf *buf;
	int ret;

	if (!nanohub_io_has_buf(io) && (file->f_flags & O_NONBLOCK))
		return -EAGAIN;

	buf = nanohub_io_get_buf(io, true);
	if (IS_ERR_OR_NULL(buf))
		return PTR_ERR(buf);

	ret = copy_to_user(buffer, buf->buffer, buf->length);
	if (ret != 0)
		ret = -EFAULT;
	else
		ret = buf->length;

	nanohub_io_put_buf(&data->free_pool, buf);

	return ret;
}

static ssize_t nanohub_write(struct file *file, const char *buffer,
			     size_t length, loff_t *offset)
{
	struct nanohub_io *io = file->private_data;
	struct nanohub_data *data = io->data;
	int ret;

	ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
	if (ret)
		return ret;

	ret = nanohub_comms_write(data, io->id, buffer, length, true);

	release_wakeup(data);

	return ret;
}

ssize_t nanohub_send_message(int channel_id, const char *buffer, size_t length)
{
	struct nanohub_data *data = priv_nanohub_data;
	int ret;

	if (!data) {
		pr_warn("%s: nanohub is not ready for kernel access\n", __func__);
		return -EAGAIN;
	}

	ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
	if (ret)
		return ret;

	ret = nanohub_comms_write(data, channel_id, buffer, length, false);

	release_wakeup(data);

	return ret;
}

bool nanohub_register_listener(int channel_id,
				void (*on_message_received)(const char *buffer,
				size_t length))
{
	struct nanohub_data *data = priv_nanohub_data;
	struct nanohub_io *io;
	if (!data) {
		pr_warn("%s: nanohub is not ready for kernel access\n", __func__);
		return false;
	}
	io = &data->io[channel_id];
	io->on_message_received = on_message_received;
	return true;
}

void nanohub_unregister_listener(int channel_id)
{
	struct nanohub_data *data = priv_nanohub_data;
	struct nanohub_io *io;
	if (!data) {
		pr_warn("%s: nanohub is not ready for kernel access\n", __func__);
		return;
	}
	io = &data->io[channel_id];
	io->on_message_received = NULL;
}

static unsigned int nanohub_poll(struct file *file, poll_table *wait)
{
	struct nanohub_io *io = file->private_data;
	unsigned int mask = POLLOUT | POLLWRNORM;

	poll_wait(file, &io->buf_wait, wait);

	if (nanohub_io_has_buf(io))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int nanohub_release(struct inode *inode, struct file *file)
{
	dev_t devt = inode->i_rdev;
	if (MINOR(devt) == ID_NANOHUB_CLIENT) {
		int drop_cnt = 0;
		struct nanohub_buf *buf;
		struct nanohub_io *io = file->private_data;
		struct nanohub_data *data = io->data;

		/* discard packets on release */
		atomic_set(&io->busy, 2);
		while ((buf = nanohub_io_get_buf(io, false)) != NULL) {
			nanohub_io_put_buf(&data->free_pool, buf);
			drop_cnt++;
		}
		if (drop_cnt > 0) {
			pr_info("nanohub: Discarded %d buffers on release",
				drop_cnt);
		}

		atomic_set(&io->busy, 0);
	}
	file->private_data = NULL;
	return 0;
}

static void nanohub_destroy_devices(struct nanohub_data *data)
{
	int i;
	struct device *dev = data->io[ID_NANOHUB_COMMS].dev;

	sysfs_remove_link(&dev->kobj, "iio");
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, &attributes[i]);
	for (i = 0; i < ID_NANOHUB_MAX; ++i)
		device_destroy(sensor_class, MKDEV(major, i));
}

static irqreturn_t nanohub_irq1(int irq, void *dev_id)
{
	struct nanohub_data *data = (struct nanohub_data *)dev_id;

	nanohub_handle_irq1(data);

	return IRQ_HANDLED;
}

static irqreturn_t nanohub_irq2(int irq, void *dev_id)
{
	struct nanohub_data *data = (struct nanohub_data *)dev_id;

	nanohub_handle_irq2(data);

	return IRQ_HANDLED;
}

static bool nanohub_os_log(char *buffer, int len)
{
	if (le32_to_cpu((((uint32_t *)buffer)[0]) & 0x7FFFFFFF) ==
	    OS_LOG_EVENTID) {
		char *mtype, *mdata = &buffer[5];

		buffer[len] = 0x00;

		switch (buffer[4]) {
		case 'E':
			mtype = KERN_ERR;
			break;
		case 'W':
			mtype = KERN_WARNING;
			break;
		case 'I':
			mtype = KERN_INFO;
			break;
		case 'D':
			mtype = KERN_DEBUG;
			break;
		default:
			mtype = KERN_DEFAULT;
			mdata--;
			break;
		}
		printk("%snanohub: %s", mtype, mdata);
		return true;
	} else {
		return false;
	}
}

static void nanohub_process_buffer(struct nanohub_data *data, uint8_t id,
				   struct nanohub_buf **buf, int ret)
{
	bool wakeup = false;
	struct nanohub_io *io;

	data->kthread_err_cnt = 0;
	if (id == ID_NANOHUB_COMMS && (ret < 4 || nanohub_os_log((*buf)->buffer, ret))) {
		release_wakeup(data);
		return;
	} else if (id >= ID_NANOHUB_MAX) {
		pr_err("nanohub_process_buffer: id %d >= max %d\n", id, ID_NANOHUB_MAX);
		release_wakeup(data);
		return;
	}

	(*buf)->length = ret;
	wakeup = true;
	io = &data->io[id];

	if (id == ID_NANOHUB_CLIENT && atomic_read(&io->busy) != 1) {
		pr_info("nanohub: dropping packet for /dev/nanohub/%d\n", id);
	} else if (!io->is_kernel) {
		nanohub_io_put_buf(io, *buf);
		*buf = NULL;
	}

	if (wakeup)
		__pm_wakeup_event(&data->wakesrc_read, data->wakeup_event_msec);
	release_wakeup(data);
	if (io->is_kernel && io->on_message_received) {
		io->on_message_received((*buf)->buffer, (*buf)->length);
	}
}

static int nanohub_kthread(void *arg)
{
	struct nanohub_data *data = (struct nanohub_data *)arg;
	struct nanohub_buf *buf = NULL;
	int ret;
	ktime_t ktime_delta;
	uint32_t clear_interrupts[8] = { 0x00000006 };
	uint8_t rx_id;
	struct device *dev = data->io[ID_NANOHUB_COMMS].dev;
	static const struct sched_param param = {
		.sched_priority = (MAX_RT_PRIO/2)-1,
	};

	data->kthread_err_cnt = 0;
	sched_setscheduler(current, SCHED_FIFO, &param);
	nanohub_set_state(data, ST_RESET);

	while (!kthread_should_stop()) {
		switch (nanohub_get_state(data)) {
		case ST_RESET:
			nanohub_reset(data);
			nanohub_wakeup_unlock(data);
			nanohub_set_state(data, ST_IDLE);
			break;
		case ST_IDLE:
			wait_event_interruptible(data->kthread_wait,
						 atomic_read(&data->kthread_run)
						 );
			nanohub_set_state(data, ST_RUNNING);
			break;
		case ST_ERROR:
			ktime_delta = ktime_sub(ktime_get_boottime(),
						data->kthread_err_ktime);
			if (ktime_to_ns(ktime_delta) > KTHREAD_ERR_TIME_NS
				&& data->kthread_err_cnt > KTHREAD_ERR_CNT) {
				dev_info(dev, "kthread: consistent error\n");
				// dev_info(dev,
				// 	"kthread: hard reset due to consistent error\n");
				// ret = nanohub_hw_reset(data); //
				// if (ret) {
				// 	dev_info(dev,
				// 		"%s: failed to reset nanohub: ret=%d\n",
				// 		__func__, ret);
				// }
			}
			msleep_interruptible(WAKEUP_TIMEOUT_MS);
			nanohub_set_state(data, ST_RUNNING);
			break;
		case ST_RUNNING:
			break;
		}
		atomic_set(&data->kthread_run, 0);

#ifdef CONFIG_NANOHUB_BL_NXP
		if (atomic_read(&data->firmware_request.state) == FW_REQUESTED) {
			nanohub_download_firmware(dev);
			nanohub_set_state(data, ST_IDLE);
			continue;
		}
#endif

		if (!buf)
			buf = nanohub_io_get_buf(&data->free_pool,
						 false);
		if (buf) {
			ret = request_wakeup_timeout(data, WAKEUP_TIMEOUT_MS);
			if (ret) {
				dev_info(dev,
					 "%s: request_wakeup_timeout: ret=%d\n",
					 __func__, ret);
				continue;
			}

			ret = nanohub_comms_rx_retrans_boottime(
			    data, CMD_COMMS_READ, ID_NANOHUB_COMMS, &rx_id,
			    buf->buffer, sizeof(buf->buffer), 10, 0);

			if (ret > 0) {
				nanohub_process_buffer(data, rx_id, &buf, ret);
				if (!nanohub_irq1_fired(data) &&
				    !nanohub_irq2_fired(data)) {
					nanohub_set_state(data, ST_IDLE);
					continue;
				}
			} else if (ret == 0) {
				/* queue empty, go to sleep */
				data->kthread_err_cnt = 0;
				data->interrupts[0] &= ~0x00000006;
				release_wakeup(data);
				nanohub_set_state(data, ST_IDLE);
				continue;
			} else {
				release_wakeup(data);
				if (data->kthread_err_cnt == 0)
					data->kthread_err_ktime =
						ktime_get_boottime();

				data->kthread_err_cnt++;
				if (data->kthread_err_cnt >= KTHREAD_WARN_CNT) {
					dev_err(dev,
						"%s: kthread_err_cnt=%d\n",
						__func__,
						data->kthread_err_cnt);
					nanohub_set_state(data, ST_ERROR);
					continue;
				}
			}
		} else {
			if (!nanohub_irq1_fired(data) &&
			    !nanohub_irq2_fired(data)) {
				nanohub_set_state(data, ST_IDLE);
				continue;
			}
			pr_warn_ratelimited("nanohub: buffers exhausted");
			/* pending interrupt, but no room to read data -
			 * clear interrupts */
			if (request_wakeup(data))
				continue;
			nanohub_comms_tx_rx_retrans(data,
						    CMD_COMMS_CLR_GET_INTR,
						    ID_NANOHUB_COMMS,
						    (uint8_t *)
						    clear_interrupts,
						    sizeof(clear_interrupts),
						    NULL,
						    (uint8_t *) data->
						    interrupts,
						    sizeof(data->interrupts),
						    false, 10, 0);
			release_wakeup(data);
			nanohub_set_state(data, ST_IDLE);
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static int nanohub_parse_vreg(struct device *dev, struct regulator **supply, const char *name)
{
	int ret = 0;
	char prop_name[DT_MAX_PROP_SIZE];
	u32 val;
	u32 vals[2];

	snprintf(prop_name, DT_MAX_PROP_SIZE, "sensorhub,%s", name);
	*supply = devm_regulator_get(dev, prop_name);
	if (IS_ERR(*supply)) {
		pr_err("nanohub: failed to get %s\n", name);
		goto out;
	}

	snprintf(prop_name, DT_MAX_PROP_SIZE, "sensorhub,%s-current", name);
	ret = of_property_read_u32(dev->of_node, prop_name, &val);
	if (!ret) {
		ret = regulator_set_load(*supply, val);
		if (ret < 0) {
			pr_err("nanohub: error setting %s load\n", name);
			goto out;
		}
	}
	snprintf(prop_name, DT_MAX_PROP_SIZE, "sensorhub,%s-voltage", name);
	ret = of_property_read_u32_array(dev->of_node, prop_name, vals, 2);
	if (!ret) {
		ret = regulator_set_voltage(*supply, vals[0], vals[1]);
		if (ret < 0) {
			pr_err("nanohub: error setting %s voltage\n", name);
			goto out;
		}
	}
	ret = regulator_enable(*supply);
	if (ret < 0) {
		pr_err("nanohub: error enabling %s\n", name);
		goto out;
	}
out:
	return ret;
}

static int nanohub_parse_supplies(struct device *dev, struct regulator *supplies[],
				  const char *list, int cnt)
{
	int i, ret = 0;
	const char *name = NULL;

	for (i = 0; i < cnt; i++) {
		ret = of_property_read_string_index(dev->of_node, list, i, &name);
		if (ret)
			goto out;

		ret = nanohub_parse_vreg(dev, &supplies[i], name);
		if (ret < 0)
			goto out;
	}
out:
	return ret;
}

static struct nanohub_platform_data *nanohub_parse_dt(struct device *dev)
{
	struct nanohub_platform_data *pdata;
	struct device_node *dt = dev->of_node;
	struct device_node *spmi_node;
	struct platform_device *spmi_dev;
#ifdef CONFIG_NANOHUB_BL_ST
	const uint32_t *tmp;
	struct property *prop;
	uint32_t u, i;
#endif
	int ret;
	u32 val = 0;

	if (!dt)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = pdata->irq1_gpio =
	    of_get_named_gpio(dt, "sensorhub,irq1-gpio", 0);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,irq1-gpio in device tree\n");
		goto free_pdata;
	}

	/* optional (strongly recommended) */
	pdata->irq2_gpio = of_get_named_gpio(dt, "sensorhub,irq2-gpio", 0);

	ret = pdata->wakeup_gpio =
	    of_get_named_gpio(dt, "sensorhub,wakeup-gpio", 0);
	if (ret < 0) {
		pr_err
		    ("nanohub: missing sensorhub,wakeup-gpio in device tree\n");
		goto free_pdata;
	}

	ret = pdata->nreset_gpio =
	    of_get_named_gpio(dt, "sensorhub,nreset-gpio", 0);
	if (ret < 0) {
		pr_err
		    ("nanohub: missing sensorhub,nreset-gpio in device tree\n");
		goto free_pdata;
	}

	ret = pdata->boot0_gpio =
		of_get_named_gpio(dt, "sensorhub,boot0-gpio", 0);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,boot0-gpio in device tree\n");
		goto free_pdata;
	}

	ret = pdata->boot2_gpio =
		of_get_named_gpio(dt, "sensorhub,boot2-gpio", 0);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,boot2-gpio in device tree\n");
		goto free_pdata;
	}

	/* optional (spi) */
	pdata->spi_cs_gpio = of_get_named_gpio(dt, "sensorhub,spi-cs-gpio", 0);

	/* required spmi controller info */
	spmi_node = of_parse_phandle(dt, "sensorhub,spmi", 0);
	if (!spmi_node) {
		pr_err("nanohub: unable to find sensorhub,spmi node\n");
		ret = -EINVAL;
		goto free_pdata;
	}
	spmi_dev = of_find_device_by_node(spmi_node);
	of_node_put(spmi_node);
	if (!spmi_dev) {
		pr_err("nanohub: unable to find sensorhub,spmi device\n");
		ret = -EINVAL;
		goto free_pdata;
	}
	pdata->pmic_regmap = dev_get_regmap(spmi_dev->dev.parent, NULL);
	if (!pdata->pmic_regmap) {
		pr_err("nanohub: unable to get spmi controller regmap\n");
		ret = -EINVAL;
		goto free_pdata;
	}

	/* required afe_tx_sup control register */
	ret = of_property_read_u32(dt, "sensorhub,afe_control",
				   &pdata->afe_control_reg);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,afe_control in device tree\n");
		goto free_pdata;
	}

	/* required vddcore control register, enable register, and voltage */
	ret = of_property_read_u32(dt, "sensorhub,vddcore_control",
				   &pdata->vddcore_control_reg);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,vddcore_control in device tree\n");
		goto free_pdata;
	}
	ret = of_property_read_u32(dt, "sensorhub,vddcore_enable",
				   &pdata->vddcore_enable_reg);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,vddcore_enable in device tree\n");
		goto free_pdata;
	}
	ret = of_property_read_u32(dt, "sensorhub,vddcore_voltage",
				   &pdata->vddcore_voltage);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,vddcore_voltage in device tree\n");
		goto free_pdata;
	}

	/* required MCU 32KHz clock control register */
	ret = of_property_read_u32(dt, "sensorhub,clk32_control",
				   &pdata->clk32_control_reg);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,clk32_control in device tree\n");
		goto free_pdata;
	}

	/* required bob ext ctrl1 control register */
	ret = of_property_read_u32(dt, "sensorhub,bob_ext_ctrl1_control",
				   &pdata->bob_ext_ctrl1_control_reg);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,bob_ext_ctrl1_control in device tree\n");
		goto free_pdata;
	}

	/* required S5A PGOOD State control register */
	ret = of_property_read_u32(dt, "sensorhub,s5a_pgood_state_control",
				   &pdata->s5a_pgood_state_reg);
	if (ret < 0) {
		pr_err("nanohub: missing sensorhub,s5a_pgood_state_control in device tree\n");
		goto free_pdata;
	}

	ret = of_property_read_u32(dt, "sensorhub,pmic_irq_enable", &val);
	if (!ret && val) {
		nanohub_pmic_irq_config(pdata->pmic_regmap);
	}

	pdata->supplies_cnt = of_property_count_strings(dt, "sensorhub,static-supplies");

	if (pdata->supplies_cnt < 0) {
		pr_err("nanohub: failed to get static supplies(%d)\n", pdata->supplies_cnt);
		goto free_pdata;
	} else if (pdata->supplies_cnt > 0) {
		pdata->supplies = devm_kcalloc(dev, pdata->supplies_cnt, sizeof(struct regulator *),
					       GFP_KERNEL);
		if (!pdata->supplies) {
			ret = -ENOMEM;
			goto free_pdata;
		}
		ret = nanohub_parse_supplies(dev, pdata->supplies, "sensorhub,static-supplies",
					     pdata->supplies_cnt);
		if (ret < 0) {
			goto free_supplies;
		}
	} else {
		pdata->supplies = NULL;
	}

	/* optional (external regulator enable) */
	pdata->reg_en_gpio = of_get_named_gpio(dt, "sensorhub,reg-en-gpio", 0);

#ifdef CONFIG_NANOHUB_BL_ST
	/* optional (bl-max-frequency) */
	pdata->bl_max_speed_hz = BL_MAX_SPEED_HZ;
	ret = of_property_read_u32(dt, "sensorhub,bl-max-frequency", &u);
	if (!ret)
		pdata->bl_max_speed_hz = u;

	/* optional (stm32f bootloader) */
	of_property_read_u32(dt, "sensorhub,bl-addr", &pdata->bl_addr);

	/* optional (stm32f bootloader) */
	tmp = of_get_property(dt, "sensorhub,num-flash-banks", NULL);
	if (tmp) {
		pdata->num_flash_banks = be32_to_cpup(tmp);
		pdata->flash_banks =
		    devm_kzalloc(dev,
				 sizeof(struct nanohub_flash_bank) *
				 pdata->num_flash_banks, GFP_KERNEL);
		if (!pdata->flash_banks)
			goto no_mem;

		/* TODO: investigate replacing with of_property_read_u32_array
		 */
		i = 0;
		of_property_for_each_u32(dt, "sensorhub,flash-banks", prop, tmp,
					 u) {
			if (i / 3 >= pdata->num_flash_banks)
				break;
			switch (i % 3) {
			case 0:
				pdata->flash_banks[i / 3].bank = u;
				break;
			case 1:
				pdata->flash_banks[i / 3].address = u;
				break;
			case 2:
				pdata->flash_banks[i / 3].length = u;
				break;
			}
			i++;
		}
	}

	/* optional (stm32f bootloader) */
	tmp = of_get_property(dt, "sensorhub,num-shared-flash-banks", NULL);
	if (tmp) {
		pdata->num_shared_flash_banks = be32_to_cpup(tmp);
		pdata->shared_flash_banks =
		    devm_kzalloc(dev,
				 sizeof(struct nanohub_flash_bank) *
				 pdata->num_shared_flash_banks, GFP_KERNEL);
		if (!pdata->shared_flash_banks)
			goto no_mem_shared;

		/* TODO: investigate replacing with of_property_read_u32_array
		 */
		i = 0;
		of_property_for_each_u32(dt, "sensorhub,shared-flash-banks",
					 prop, tmp, u) {
			if (i / 3 >= pdata->num_shared_flash_banks)
				break;
			switch (i % 3) {
			case 0:
				pdata->shared_flash_banks[i / 3].bank = u;
				break;
			case 1:
				pdata->shared_flash_banks[i / 3].address = u;
				break;
			case 2:
				pdata->shared_flash_banks[i / 3].length = u;
				break;
			}
			i++;
		}
	}
#endif

	pdata->firmware_name = of_get_property(dt, "sensorhub,firmware", NULL);
	if (!pdata->firmware_name) {
		pr_err("nanohub: missing sensorhub,firmware in device tree\n");
		ret = -EINVAL;
		goto free_supplies;
	}

	return pdata;

#ifdef CONFIG_NANOHUB_BL_ST
no_mem_shared:
	devm_kfree(dev, pdata->flash_banks);
no_mem:
	ret = -ENOMEM;
#endif
free_supplies:
	devm_kfree(dev, pdata->supplies);
free_pdata:
	devm_kfree(dev, pdata);
	return ERR_PTR(ret);
}
#else
static struct nanohub_data *nanohub_parse_dt(struct device *dev)
{
	struct nanohub_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->bl_max_speed_hz = BL_MAX_SPEED_HZ;
	return pdata;
}
#endif

static int nanohub_request_irqs(struct nanohub_data *data)
{
	int ret;

	ret = request_threaded_irq(data->irq1, NULL, nanohub_irq1,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "nanohub-irq1", data);
	if (ret < 0)
		data->irq1 = 0;
	else
		disable_irq(data->irq1);
	if (data->irq2 <= 0 || ret < 0) {
		data->irq2 = 0;
		return ret;
	}

	ret = request_threaded_irq(data->irq2, NULL, nanohub_irq2,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "nanohub-irq2", data);
	if (ret < 0) {
		data->irq2 = 0;
		WARN(1, "failed to request optional IRQ %d; err=%d",
		     data->irq2, ret);
	} else {
		disable_irq(data->irq2);
	}

	/* if 2d request fails, hide this; it is optional IRQ,
	 * and failure should not interrupt driver init sequence.
	 */
	return 0;
}

static int nanohub_request_gpios(struct nanohub_data *data)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(gconf); ++i) {
		const struct gpio_config *cfg = &gconf[i];
		unsigned int gpio = plat_gpio_get(data, cfg);
		const char *label;
		bool optional = gpio_is_optional(cfg);

		ret = 0; /* clear errors on optional pins, if any */

		if (!gpio_is_valid(gpio) && optional)
			continue;

		label = cfg->label;
		ret = gpio_request_one(gpio, cfg->flags, label);
		if (ret && !optional) {
			pr_err("nanohub: gpio %d[%s] request failed;err=%d\n",
			       gpio, label, ret);
			break;
		}
		if (gpio_has_irq(cfg)) {
			int irq = gpio_to_irq(gpio);
			if (irq > 0) {
				nanohub_set_irq_data(data, cfg, irq);
			} else if (!optional) {
				ret = -EINVAL;
				pr_err("nanohub: no irq; gpio %d[%s];err=%d\n",
				       gpio, label, irq);
				break;
			}
		}
	}
	if (i < ARRAY_SIZE(gconf)) {
		for (--i; i >= 0; --i)
			gpio_free(plat_gpio_get(data, &gconf[i]));
	}

	return ret;
}

static void nanohub_release_gpios_irqs(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	if (data->irq2)
		free_irq(data->irq2, data);
	if (data->irq1)
		free_irq(data->irq1, data);
	if (gpio_is_valid(pdata->irq2_gpio))
		gpio_free(pdata->irq2_gpio);
	gpio_free(pdata->irq1_gpio);
	//gpio_set_value(pdata->nreset_gpio, 0);
	gpio_free(pdata->nreset_gpio);
	mcu_wakeup_gpio_set_value(data, 1);
	gpio_free(pdata->wakeup_gpio);
	gpio_set_value(pdata->boot0_gpio, 1);
	gpio_free(pdata->boot0_gpio);
	gpio_free(pdata->boot2_gpio);
	if (gpio_is_valid(pdata->reg_en_gpio))
		gpio_free(pdata->reg_en_gpio);
}

struct iio_dev *nanohub_probe(struct device *dev, struct iio_dev *iio_dev)
{
	int ret, i;
	const struct nanohub_platform_data *pdata;
	struct nanohub_data *data;
	struct nanohub_buf *buf;
	bool own_iio_dev = !iio_dev;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		pdata = nanohub_parse_dt(dev);
		if (IS_ERR(pdata))
			return ERR_PTR(PTR_ERR(pdata));
	}

	if (own_iio_dev) {
		iio_dev = iio_device_alloc(dev, sizeof(struct nanohub_data));
		if (!iio_dev)
			return ERR_PTR(-ENOMEM);
	}

	iio_dev->name = "nanohub";
	iio_dev->dev.parent = dev;
	iio_dev->info = &nanohub_iio_info;
	iio_dev->channels = NULL;
	iio_dev->num_channels = 0;

	data = iio_priv(iio_dev);
	data->iio_dev = iio_dev;
	data->pdata = pdata;

	data->wakeup_event_msec = 100; /* Default 100 msec timed wakelock per event */

	init_waitqueue_head(&data->kthread_wait);

	nanohub_io_init(&data->free_pool, data, dev, 0, false);

	buf = vmalloc(sizeof(*buf) * READ_QUEUE_DEPTH);
	data->vbuf = buf;
	if (!buf) {
		ret = -ENOMEM;
		goto fail_vma;
	}

	for (i = 0; i < READ_QUEUE_DEPTH; i++)
		nanohub_io_put_buf(&data->free_pool, &buf[i]);
	atomic_set(&data->kthread_run, 0);
	data->wakesrc_read.name = "nanohub_wake";
	wakeup_source_add(&data->wakesrc_read);

	/* hold lock until reset completes */
	atomic_set(&data->lock_mode, LOCK_MODE_RESET);
	atomic_set(&data->wakeup_cnt, 0);
	atomic_set(&data->wakeup_lock_cnt, 1);
	atomic_set(&data->wakeup_acquired, KEY_WAKEUP_LOCK);
	init_waitqueue_head(&data->wakeup_wait);

	ret = nanohub_request_gpios(data);
	if (ret)
		goto fail_gpio;

	ret = nanohub_request_irqs(data);
	if (ret)
		goto fail_irq;

	ret = iio_device_register(iio_dev);
	if (ret) {
		pr_err("nanohub: iio_device_register failed\n");
		goto fail_irq;
	}

	ret = nanohub_create_devices(data);
	if (ret)
		goto fail_dev;

#ifdef CONFIG_NANOHUB_BL_NXP
	atomic_set(&data->firmware_request.state, FW_IDLE);
	init_completion(&data->firmware_request.fw_complete);
#endif

	data->thread = kthread_create(nanohub_kthread, data, "nanohub");
	if (!IS_ERR(data->thread)) {
		priv_nanohub_data = data;
		return iio_dev;
	}

fail_dev:
	iio_device_unregister(iio_dev);
fail_irq:
	nanohub_release_gpios_irqs(data);
fail_gpio:
	wakeup_source_remove(&data->wakesrc_read);
	vfree(buf);
fail_vma:
	if (own_iio_dev)
		iio_device_free(iio_dev);

	return ERR_PTR(ret);
}

void nanohub_start(struct nanohub_data *data)
{
	wake_up_process(data->thread);
}

int nanohub_reset(struct nanohub_data *data)
{
	const struct nanohub_platform_data *pdata = data->pdata;

	gpio_set_value(pdata->nreset_gpio, 1);
	usleep_range(650000, 700000);
	return 0;
}

int nanohub_remove(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);

	priv_nanohub_data = NULL;
	nanohub_notify_thread(data);
	kthread_stop(data->thread);

	nanohub_destroy_devices(data);
	iio_device_unregister(iio_dev);
	nanohub_release_gpios_irqs(data);
	wakeup_source_remove(&data->wakesrc_read);
	vfree(data->vbuf);
	iio_device_free(iio_dev);

	return 0;
}

void nanohub_shutdown(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);
	const struct nanohub_platform_data *pdata = data->pdata;

	// Hold the MCU in reset during shutdown. See b/313042435.
	gpio_set_value(pdata->nreset_gpio, 0);
}

int nanohub_suspend(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);
	const struct nanohub_platform_data *pdata = data->pdata;
	int ret;

	ret = nanohub_wakeup_lock(data, LOCK_MODE_SUSPEND_RESUME);
	if (!ret) {
		int cnt;
		const int max_cnt = 10;

		for (cnt = 0; cnt < max_cnt; ++cnt) {
			if (!nanohub_irq1_fired(data))
				break;
			usleep_range(10, 15);
		}
		if (cnt < max_cnt) {
			dev_dbg(&iio_dev->dev, "%s: cnt=%d\n", __func__, cnt);
			enable_irq_wake(data->irq1);
			gpio_set_value(pdata->boot0_gpio, 1);
			return 0;
		}
		ret = -EBUSY;
		dev_info(&iio_dev->dev,
			 "%s: failed to suspend: IRQ1=%d, state=%d\n",
			 __func__, nanohub_irq1_fired(data),
			 nanohub_get_state(data));
		nanohub_wakeup_unlock(data);
	} else {
		dev_info(&iio_dev->dev, "%s: could not take wakeup lock\n",
			 __func__);
	}

	return ret;
}

int nanohub_suspend_noirq(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);
	int ret = 0;

	if (nanohub_irq1_fired(data)) {
		ret = -EBUSY;
	}
	return ret;
}

int nanohub_resume(struct iio_dev *iio_dev)
{
	struct nanohub_data *data = iio_priv(iio_dev);
	const struct nanohub_platform_data *pdata = data->pdata;

	disable_irq_wake(data->irq1);
	gpio_set_value(pdata->boot0_gpio, 0);
	nanohub_wakeup_unlock(data);

	return 0;
}

static int __init nanohub_init(void)
{
	int ret = 0;

	sensor_class = class_create(THIS_MODULE, "nanohub");
	if (IS_ERR(sensor_class)) {
		ret = PTR_ERR(sensor_class);
		pr_err("nanohub: class_create failed; err=%d\n", ret);
	}
	if (!ret)
		major = __register_chrdev(0, 0, ID_NANOHUB_MAX, "nanohub",
					  &nanohub_fileops);

	if (major < 0) {
		ret = major;
		major = 0;
		pr_err("nanohub: can't register; err=%d\n", ret);
	}

#ifdef CONFIG_NANOHUB_SPI
	if (ret == 0)
		ret = nanohub_spi_init();
#endif
	pr_info("nanohub: loaded; ret=%d\n", ret);
	return ret;
}

static void __exit nanohub_cleanup(void)
{
#ifdef CONFIG_NANOHUB_SPI
	nanohub_spi_cleanup();
#endif
	__unregister_chrdev(major, 0, ID_NANOHUB_MAX, "nanohub");
	class_destroy(sensor_class);
	major = 0;
	sensor_class = 0;
}

module_init(nanohub_init);
module_exit(nanohub_cleanup);

MODULE_AUTHOR("Ben Fennema");
MODULE_LICENSE("GPL");
