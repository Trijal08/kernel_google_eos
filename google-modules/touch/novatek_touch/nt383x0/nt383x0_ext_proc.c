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


#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "nt383x0.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#define NVT_CHIP_ID "nvt_chip_id"

#if GOOGLE_TOUCH_EXT_PROC
#define NVT_RESET "nvt_reset"
#define NVT_SCAN_MODE "nvt_scan_mode"
#define NVT_SENSING "nvt_sensing"
#define NVT_HOPPING_FREQ "nvt_hopping_freq"
#define NVT_TRIMID "nvt_trimid"
#define NVT_HISTORY_EVENT "nvt_history_event"
#endif

#define BUS_TRANSFER_LENGTH  64

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static uint8_t chip_id[8] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *NVT_proc_chip_id_entry;

#if GOOGLE_TOUCH_EXT_PROC
static struct proc_dir_entry *NVT_proc_reset_entry;
static struct proc_dir_entry *NVT_proc_scan_mode_entry;
static struct proc_dir_entry *NVT_proc_sensing_entry;
static struct proc_dir_entry *NVT_proc_hopping_freq_entry;
static struct proc_dir_entry *NVT_proc_trimid_entry;
static struct proc_dir_entry *NVT_proc_history_event_entry;
#endif

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address,
		     ts->mmap->EVENT_BUF_BEGIN_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 1] = {0};
	int32_t data_len = 0;
	int32_t loop_cnt = 0;
	int32_t residual_len = 0;

	//---set read data length---
	data_len = ts->x_num * ts->y_num * 2;
	loop_cnt = data_len / BUS_TRANSFER_LENGTH;
	residual_len = data_len % BUS_TRANSFER_LENGTH;

	//printk("data_len=%d, loop_cnt=%d, residual_len=%d\n", data_len, loop_cnt, residual_len);

	//read xdata : step 1
	for (i = 0; i < loop_cnt; i++) {
		//---change xdata index---
		nvt_set_page(I2C_FW_Address, (xdata_addr + i * BUS_TRANSFER_LENGTH));
		//---read data---
		buf[0] = ((xdata_addr + i * BUS_TRANSFER_LENGTH) & 0xFF);
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);
		memcpy((xdata_tmp + i * BUS_TRANSFER_LENGTH), (buf + 1), BUS_TRANSFER_LENGTH);
		//printk("addr=0x%05X\n", (xdata_addr + i * BUS_TRANSFER_LENGTH));
	}

	//read residual xdata : step2
	if (residual_len != 0) {
		//---change xdata index---
		nvt_set_page(I2C_FW_Address, (xdata_addr + loop_cnt * BUS_TRANSFER_LENGTH));
		//---read data---
		buf[0] = ((xdata_addr + loop_cnt * BUS_TRANSFER_LENGTH) & 0xFF);
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, residual_len + 1);
		memcpy((xdata_tmp + loop_cnt * BUS_TRANSFER_LENGTH), (buf + 1), residual_len);
		//printk("addr=0x%05X\n", (xdata_addr + loop_cnt * BUS_TRANSFER_LENGTH));
	}

	//---2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[i * 2] + 256 * xdata_tmp[i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(I2C_FW_Address, xdata_btn_addr);

	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR);
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}


/*******************************************************
Description:
	Novatek touchscreen chip id show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_chip_id_show(struct seq_file *m, void *v)
{
	seq_printf(m, "chip_id=[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]\n", chip_id[1], chip_id[2], chip_id[3], chip_id[4], chip_id[5], chip_id[6]);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

const struct seq_operations nvt_chip_id_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_chip_id_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_fw_version_fops = {
	.proc_open = nvt_fw_version_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_chip_id open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_chip_id_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_chip_id_info(chip_id)) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_chip_id_seq_ops);
}

static const struct file_operations nvt_chip_id_fops = {
	.owner = THIS_MODULE,
	.open = nvt_chip_id_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_baseline_fops = {
	.proc_open = nvt_baseline_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_raw_fops = {
	.proc_open = nvt_raw_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_diff_fops = {
	.proc_open = nvt_diff_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

#if GOOGLE_TOUCH_EXT_PROC
#define CMD_CHANGE_SCAN 0x1F
#define CMD_DOZE_MODE 0x30
#define CMD_GESTURE_MODE 0x31
#define CMD_FDM_MODE 0x32
#define CMD_HOPPING_FREQ1 0x41
#define CMD_HOPPING_FREQ2 0x42
#define CMD_ACTVE_MODE 0x01
#define INIT_MODE 0x00
#define ACTVE_MODE 0x01

#define SCAN_MODE_STATUS_ADDR 0x20051

int32_t nvt_customizeCmd_store(uint8_t u8Cmd, uint8_t u8SubCmd, uint16_t len) {
	uint8_t buf[4] = {0};
	int32_t ret = 0;
	int32_t i, retry = 5;


	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		return ret;
	}

	for(i = 0; i < retry; i++){
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8SubCmd;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, len);

		msleep(20);

		//---read cmd status---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i == retry)) {
		NVT_ERR("send Cmd 0x%02X, 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, u8SubCmd, buf[1]);
		ret = -1;
	} else {
		NVT_LOG("send Cmd 0x%02X, 0x%02X success, tried %d times\n", u8Cmd, u8SubCmd, i);
	}

	return  ret;
}


uint8_t nvt_customizeCmd_show(int32_t start_addr) {
	uint8_t buf[4] = {0};
	uint8_t result = 0;

	//---set xdata index to start_addr---
	nvt_set_page(I2C_FW_Address, start_addr);

	//---read cmd status---
	buf[0] = start_addr & 0xFF;
	buf[1] = 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	result = buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR);

	return  result;
}


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_reset. (Read only)

return:
	Reset OK!
*******************************************************/
static int nvt_reset_show(struct seq_file *sfile, void *v) {
       seq_printf(sfile, "Reset OK!\n");
       return 0;
}

static int32_t nvt_reset_open(struct inode *inode, struct file *file)
{
	NVT_LOG("++\n");

	//---Reset IC---
	nvt_bootloader_reset();

	NVT_LOG("--\n");

	return single_open(file, &nvt_reset_show, NULL);
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_reset_fops = {
	.proc_open = nvt_reset_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_reset_fops = {
	.owner = THIS_MODULE,
	.open = nvt_reset_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_scan_mode.

return:
Read:
	fw current scan mode
Write:
	1 : tp enter active mode
	2 : tp enter doze mode
	3 : tp enter standby mode finger on
	4 : tp enter standby mode finger off
*******************************************************/
static int nvt_scan_mode_show(struct seq_file *sfile, void *v) {
	uint8_t scan_mode;

	scan_mode = nvt_customizeCmd_show(SCAN_MODE_STATUS_ADDR);

	switch (scan_mode) {
		case INIT_MODE:
			seq_printf(sfile, "Initial value\n");
			break;
		case ACTVE_MODE:
			seq_printf(sfile, "Active mode\n");
			break;
		case CMD_DOZE_MODE:
			seq_printf(sfile, "Doze mode\n");
			break;
		case CMD_GESTURE_MODE:
			seq_printf(sfile, "Standby mode finger on\n");
			break;
		case CMD_FDM_MODE:
			seq_printf(sfile, "Standby mode finger off\n");
			break;
		case CMD_HOPPING_FREQ1:
			seq_printf(sfile, "Active mode & Hopping Freq1\n");
			break;
		case CMD_HOPPING_FREQ2:
			seq_printf(sfile, "Active mode & Hopping Freq2\n");
			break;
		default:
			seq_printf(sfile, "%02x Invalid!\n", scan_mode);
			break;
	}

    return 0;
}

static int32_t nvt_scan_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, &nvt_scan_mode_show, NULL);
}

static ssize_t nvt_scan_mode_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[128] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN);

	switch (input) {
		case 1: //Active mode
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_ACTVE_MODE,3);
			break;
		case 2://Doze mode
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_DOZE_MODE,3);
			break;
		case 3://Standby mode finger on
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_GESTURE_MODE,3);
			break;
		case 4://Standby mode finger off
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_FDM_MODE,3);
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;

	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;

}
#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_scan_mode_fops = {
	.proc_open = nvt_scan_mode_open,
	.proc_write = nvt_scan_mode_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_scan_mode_fops = {
	.owner = THIS_MODULE,
	.open = nvt_scan_mode_open,
	.write = nvt_scan_mode_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_hopping_freq. (read/write)

return:
Read:
	fw current scan mode
Write:
	1 : hopping active frequency 1
	2 : hopping active frequency 2
*******************************************************/
static int nvt_hopping_freq_show(struct seq_file *sfile, void *v) {
	uint8_t scan_mode;

	scan_mode = nvt_customizeCmd_show(SCAN_MODE_STATUS_ADDR);

	switch (scan_mode) {
		case INIT_MODE:
			seq_printf(sfile, "Initial value\n");
			break;
		case ACTVE_MODE:
			seq_printf(sfile, "Active mode\n");
			break;
		case CMD_DOZE_MODE:
			seq_printf(sfile, "Doze mode\n");
			break;
		case CMD_GESTURE_MODE:
			seq_printf(sfile, "Standby mode finger on\n");
			break;
		case CMD_FDM_MODE:
			seq_printf(sfile, "Standby mode finger off\n");
			break;
		case CMD_HOPPING_FREQ1:
			seq_printf(sfile, "Active mode & Hopping Freq1\n");
			break;
		case CMD_HOPPING_FREQ2:
			seq_printf(sfile, "Active mode & Hopping Freq2\n");
			break;
		default:
			seq_printf(sfile, "%02x Invalid!\n", scan_mode);
			break;
	}

    return 0;
}

static int32_t nvt_hopping_freq_open(struct inode *inode, struct file *file)
{

	return single_open(file, &nvt_hopping_freq_show, NULL);
}

static ssize_t nvt_hopping_freq_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[128] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN);

	switch (input) {
		case 1:
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_HOPPING_FREQ1,3);
			break;
		case 2:
			nvt_customizeCmd_store(CMD_CHANGE_SCAN,CMD_HOPPING_FREQ2,3);
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;
}
#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_hopping_freq_fops = {
	.proc_open = nvt_hopping_freq_open,
	.proc_write = nvt_hopping_freq_write,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_hopping_freq_fops = {
	.owner = THIS_MODULE,
	.open = nvt_hopping_freq_open,
	.write = nvt_hopping_freq_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_sensing. (write only)

return:
	0 : sensing off, touch enter power down
	1 : sensing on
*******************************************************/
static ssize_t nvt_sensing_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[4] = {0};
	uint8_t buf[4] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	switch (input) {
		case 0:
			//---set xdata index to EVENT BUF ADDR---
			nvt_set_page(I2C_FW_Address,
				     ts->mmap->EVENT_BUF_BEGIN_ADDR | EVENT_MAP_HOST_CMD);

			buf[0] = EVENT_MAP_HOST_CMD;
			buf[1] = 0x12;
			CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
			NVT_LOG("Touch Enter PD\n");
			break;
		case 1:
			nvt_bootloader_reset();
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;
}
#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_sensing_fops = {
	.proc_write = nvt_sensing_write,
};
#else
static const struct file_operations nvt_sensing_fops = {
	.write = nvt_sensing_write,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_trimid. (read only)

return:
	novatek chip id
*******************************************************/
static int nvt_trimid_show(struct seq_file *sfile, void *v) {
	   seq_printf(sfile, "%s\n", ts->trimid);
       return 0;
}

static int32_t nvt_trimid_open(struct inode *inode, struct file *file)
{
	return single_open(file, &nvt_trimid_show, NULL);
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_trimid_fops = {
	.proc_open = nvt_trimid_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_trimid_fops = {
	.owner = THIS_MODULE,
	.open = nvt_trimid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_history_event. (read/write)
return:
Read:
	fw history event log
	@FW_HISTORY_WDT_ADDR 1 bytes
	@FW_HISTORY_BEGIN_ADDR 64 bytes
	@FW_HISTORY_MID_ADDR 64 bytes
Write:
	0:inital fw history log
*******************************************************/
static int32_t c_history_event_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	seq_printf(m, "Addr:[0x%5X]\n", ts->mmap->FW_HISTORY_WDT_ADDR);
	seq_printf(m, "%2X\n", xdata_tmp[0]);

	seq_printf(m, "Addr:[0x%5X]\n", ts->mmap->FW_HISTORY_BEGIN_ADDR);
	for (i = 0; i < 2; i++) {
		if(i == 1) {
			seq_printf(m, "Addr:[0x%5X]\n", ts->mmap->FW_HISTORY_MID_ADDR);
		}
		for (j = 0; j < 64; j++) {
			seq_printf(m, "%2X, ", xdata_tmp[1 + i * 64 + j]);
		}
		seq_puts(m, "\n");
	}

	seq_printf(m, "\n\n");
	return 0;
}

const struct seq_operations nvt_history_event_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_history_event_show
};

static int32_t nvt_history_event_open(struct inode *inode, struct file *file)
{
	int32_t i = 0;
	int32_t j = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 1] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	//---read WDT log---
	nvt_set_page(I2C_FW_Address, ts->mmap->FW_HISTORY_WDT_ADDR);
	buf[0] = ts->mmap->FW_HISTORY_WDT_ADDR & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
	xdata_tmp[j++] = buf[1];

	//---read history event buffer 1---
	nvt_set_page(I2C_FW_Address, ts->mmap->FW_HISTORY_BEGIN_ADDR);
	buf[0] = ts->mmap->FW_HISTORY_BEGIN_ADDR & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);

	//---copy buf to xdata_tmp---
	for (i = 0; i < BUS_TRANSFER_LENGTH; i++) {
		xdata_tmp[j++] = buf[i + 1];
	}

	//---read history event buffer 2---
	buf[0] = ts->mmap->FW_HISTORY_MID_ADDR & 0xFF;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, BUS_TRANSFER_LENGTH + 1);

	//---copy buf to xdata_tmp---
	for (i = 0; i < BUS_TRANSFER_LENGTH; i++) {
		xdata_tmp[j++] = buf[i + 1];
	}

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(I2C_FW_Address, ts->mmap->EVENT_BUF_BEGIN_ADDR);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_history_event_seq_ops);
}

static ssize_t nvt_hitstory_event_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	unsigned int input = 0;
	char cmd[4] = {0};

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

	if(count > 2) {
		NVT_ERR("input value error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if(copy_from_user(cmd, buffer, count)) {
		NVT_ERR("copy value from user error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	if (kstrtouint(cmd, 0, &input)) {
		NVT_ERR("kstrtouint error\n");
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}

	switch (input) {
		case 0:
			nvt_bootloader_reset();
			break;
		default:
			NVT_ERR("%u not supported\n", input);
			break;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return count;
}

#ifdef HAVE_PROC_OPS
static const struct proc_ops nvt_history_event_fops = {
	.proc_write = nvt_hitstory_event_write,
	.proc_open = nvt_history_event_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations nvt_history_event_fops = {
	.owner = THIS_MODULE,
	.write = nvt_hitstory_event_write,
	.open = nvt_history_event_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif
#endif
/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t nvt_extra_proc_init(void)
{
#if NVT_TOUCH_EXT_PROC_INIT
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_FW_VERSION);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_FW_VERSION);
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_BASELINE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_BASELINE);
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_RAW);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_RAW);
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_DIFF);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_DIFF);
	}

	NVT_proc_chip_id_entry = proc_create(NVT_CHIP_ID, 0444, NULL,&nvt_chip_id_fops);
	if (NVT_proc_chip_id_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_CHIP_ID);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_CHIP_ID);
	}
#endif

#if GOOGLE_TOUCH_EXT_PROC
	NVT_proc_reset_entry = proc_create(NVT_RESET, 0444, NULL,&nvt_reset_fops);
	if (NVT_proc_reset_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_RESET);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_RESET);
	}

	NVT_proc_scan_mode_entry = proc_create(NVT_SCAN_MODE, 0644, NULL,&nvt_scan_mode_fops);
	if (NVT_proc_scan_mode_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_SCAN_MODE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_SCAN_MODE);
	}

	NVT_proc_hopping_freq_entry = proc_create(NVT_HOPPING_FREQ, 0644, NULL,&nvt_hopping_freq_fops);
	if (NVT_proc_hopping_freq_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_HOPPING_FREQ);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_HOPPING_FREQ);
	}

	NVT_proc_sensing_entry = proc_create(NVT_SENSING, 0222, NULL,&nvt_sensing_fops);
	if (NVT_proc_sensing_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_SENSING);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_SENSING);
	}

	NVT_proc_trimid_entry = proc_create(NVT_TRIMID, 0444, NULL,&nvt_trimid_fops);
	if (NVT_proc_trimid_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_TRIMID);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_TRIMID);
	}

	NVT_proc_history_event_entry = proc_create(NVT_HISTORY_EVENT, 0644, NULL,&nvt_history_event_fops);
	if (NVT_proc_history_event_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_HISTORY_EVENT);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_HISTORY_EVENT);
	}
#endif
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	deinitial function.

return:
	n.a.
*******************************************************/
void nvt_extra_proc_deinit(void)
{
	if (NVT_proc_fw_version_entry != NULL) {
		remove_proc_entry(NVT_FW_VERSION, NULL);
		NVT_proc_fw_version_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_FW_VERSION);
	}

	if (NVT_proc_baseline_entry != NULL) {
		remove_proc_entry(NVT_BASELINE, NULL);
		NVT_proc_baseline_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_BASELINE);
	}

	if (NVT_proc_raw_entry != NULL) {
		remove_proc_entry(NVT_RAW, NULL);
		NVT_proc_raw_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_RAW);
	}

	if (NVT_proc_diff_entry != NULL) {
		remove_proc_entry(NVT_DIFF, NULL);
		NVT_proc_diff_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_DIFF);
	}

	if (NVT_proc_chip_id_entry != NULL) {
		remove_proc_entry(NVT_CHIP_ID, NULL);
		NVT_proc_chip_id_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_CHIP_ID);
	}
}

/* sysfs */
#if NVT_TOUCH_EXT_SYSFS
static ssize_t nvt_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return scnprintf(buf, PAGE_SIZE, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n",
			 ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
}

static ssize_t dump_touch_frame(char *buf, uint32_t max_size, char *dump_data_name)
{
	int32_t x = 0;
	int32_t y = 0;
	int32_t iArrayIndex = 0;
	uint32_t ret = 0;

	ret += scnprintf(buf + ret, max_size - ret, "%s\n", dump_data_name);
	for (y = 0; y < ts->y_num; y++) {
		for (x = 0; x < ts->x_num; x++) {
			iArrayIndex = y * ts->x_num + x;
			ret += scnprintf(buf + ret, max_size - ret, "%5d, ", xdata[iArrayIndex]);
		}
		ret += scnprintf(buf + ret, max_size - ret, "\n");
	}
	return ret;
}

static ssize_t nvt_baseline_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = 0;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);

	nvt_change_mode(NORMAL_MODE);
	ret = dump_touch_frame(buf, PAGE_SIZE, "Dump baseline data:");
	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return ret;
}

static ssize_t nvt_raw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = 0;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);
	ret = dump_touch_frame(buf, PAGE_SIZE, "Dump raw data:");
	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return ret;
}

static ssize_t nvt_diff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = 0;

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);
	ret = dump_touch_frame(buf, PAGE_SIZE, "Dump diff data:");
	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return ret;
}

static ssize_t nvt_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
        nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_chip_id_info(chip_id)) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");
	return scnprintf(buf, PAGE_SIZE, "chip_id=[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]\n",
			 chip_id[1], chip_id[2], chip_id[3], chip_id[4], chip_id[5], chip_id[6]);
}

static ssize_t nvt_chip_power_down_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
    nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_ts_enter_power_down()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return 0;
}

static ssize_t nvt_chip_power_up_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	NVT_LOG("nvt_chip_power_up\n");
	nvt_bootloader_reset();
	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return 0;
}

static ssize_t nvt_chip_sleep_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_ts_enter_sleep()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return 0;
}

static ssize_t nvt_chip_wakeup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	NVT_LOG("nvt_chip_wakeup\n");
	nvt_bootloader_reset();
	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return 0;
}

static ssize_t nvt_touch_suppress_ms_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int ret;
	unsigned int latency_ms;

	ret = kstrtouint(buf, 0, &latency_ms);
	if (ret < 0) {
		NVT_LOG("Unknown value: %s is given", buf);
		return ret;
	}

	nvt_ts_touch_suprress(msecs_to_jiffies(latency_ms));

	return count;
}

static ssize_t nvt_touch_suppress_ms_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%lu", atomic64_read(&ts->touch_inactive_time));
}

static ssize_t nvt_standby_store(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int rc;
	bool cmd;

	if (mutex_lock_interruptible(&ts->lock))
		return -ERESTARTSYS;

	rc = kstrtobool(buf, &cmd);
	if (rc) {
		NVT_LOG("kstrtobool failed, rc = %d\n", rc);
		mutex_unlock(&ts->lock);
		return -EINVAL;
	}
	ts->standby = cmd;
	NVT_LOG("Set standby mode to \"%s\"\n", cmd ? "Enabled" : "Disabled");
	mutex_unlock(&ts->lock);
	return count;
}

static ssize_t nvt_standby_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mutex_lock_interruptible(&ts->lock))
		return -ERESTARTSYS;

	NVT_LOG("standby mode %s\n", ts->standby ? "enabled" : "disabled");
	mutex_unlock(&ts->lock);

	return scnprintf(buf, PAGE_SIZE, "standby mode=%d\n", ts->standby);
}

ssize_t nvt_dump_raw(char *buf, uint32_t max_size)
{
	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	return dump_touch_frame(buf, max_size, "Dump raw data:");
}

ssize_t nvt_dump_baseline(char *buf, uint32_t max_size)
{
	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);
	return dump_touch_frame(buf, max_size, "Dump baseline data:");
}

ssize_t nvt_dump_diff(char *buf, uint32_t max_size)
{
	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	return dump_touch_frame(buf, max_size, "Dump diff data:");
}

ssize_t nvt_customized_dump(char *log, uint32_t max_size, uint32_t start_addr, uint32_t end_addr)
{
	int32_t i = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 1] = { 0 };
	int32_t data_len = 0;
	int32_t read_cnt = 0;
	uint32_t ret = 0;

	if (end_addr < start_addr)
		return 0;

	data_len = end_addr - start_addr + 1;
	ret += scnprintf(log + ret, max_size - ret, "Dump starting from %05X to %05X:\n",
			 start_addr, end_addr);

	while (data_len > 0) {
		read_cnt = min(data_len, BUS_TRANSFER_LENGTH);
		nvt_set_page(I2C_FW_Address, start_addr);
		buf[0] = (start_addr & 0xFF);
		if (CTP_I2C_READ(ts->client, I2C_FW_Address, buf, read_cnt + 1) < 0) {
			NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
			return 0;
		}
		for (i = 0; i < read_cnt; i++)
			ret += scnprintf(log + ret, max_size - ret, "%02X, ", buf[i]);

		start_addr += read_cnt;
		data_len -= read_cnt;
	}
	ret += scnprintf(log + ret, max_size - ret, "\n");

	return ret;
}

ssize_t nvt_dump_fw_info(char *buf, uint32_t max_size)
{
	return scnprintf(buf, max_size, "fw_ver=0x%02X, PID=0x%04X\n", ts->fw_ver, ts->nvt_pid);
}

static ssize_t nvt_dump_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t count = 0;

	if (mutex_lock_interruptible(&ts->lock))
		return 0;

	count += nvt_dump_fw_info(buf + count, PAGE_SIZE - count);

	if (nvt_clear_fw_status()) {
		NVT_ERR("clear fw status failed!\n");
		goto dump_exit;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		NVT_ERR("check fw status failed!\n");
		goto dump_exit;
	}

	count += nvt_dump_baseline(buf + count, PAGE_SIZE - count);

	nvt_change_mode(NORMAL_MODE);

	// Collect firmware history event
	count += nvt_customized_dump(buf + count, PAGE_SIZE - count,
				     ts->mmap->FW_HISTORY_BEGIN_ADDR,
				     ts->mmap->FW_HISTORY_END_ADDR);

	// Collect event buffer
	count += nvt_customized_dump(buf + count, PAGE_SIZE - count, ts->mmap->EVENT_BUF_BEGIN_ADDR,
				     ts->mmap->EVENT_BUF_END_ADDR);

	// Collect firmware states
	count += nvt_customized_dump(buf + count, PAGE_SIZE - count, ts->mmap->FW_STATE_BEGIN_ADDR,
				     ts->mmap->FW_STATE_END_ADDR);

	// Collect firmware version
	count += nvt_customized_dump(buf + count, PAGE_SIZE - count, ts->mmap->FW_VERSION_ADDR,
				     ts->mmap->FW_VERSION_ADDR);

	// Collect firmware pid
	count += nvt_customized_dump(buf + count, PAGE_SIZE - count, ts->mmap->FW_PID_ADDR,
				     ts->mmap->FW_PID_ADDR);

dump_exit:
	nvt_bootloader_reset();
	mutex_unlock(&ts->lock);
	return count;
}

static DEVICE_ATTR_RO(nvt_fw_version);
static DEVICE_ATTR_RO(nvt_baseline);
static DEVICE_ATTR_RO(nvt_raw);
static DEVICE_ATTR_RO(nvt_diff);
static DEVICE_ATTR_RO(nvt_chip_id);
static DEVICE_ATTR_RO(nvt_chip_power_down);
static DEVICE_ATTR_RO(nvt_chip_power_up);
static DEVICE_ATTR_RO(nvt_chip_sleep);
static DEVICE_ATTR_RO(nvt_chip_wakeup);
static DEVICE_ATTR_RO(nvt_dump_data);
static DEVICE_ATTR_RW(nvt_touch_suppress_ms);
static DEVICE_ATTR_RW(nvt_standby);

static struct attribute *nvt_ts_attributes[] = {
	&dev_attr_nvt_fw_version.attr,
	&dev_attr_nvt_baseline.attr,
	&dev_attr_nvt_raw.attr,
	&dev_attr_nvt_diff.attr,
	&dev_attr_nvt_chip_id.attr,
	&dev_attr_nvt_chip_power_down.attr,
	&dev_attr_nvt_chip_power_up.attr,
	&dev_attr_nvt_chip_sleep.attr,
	&dev_attr_nvt_chip_wakeup.attr,
	&dev_attr_nvt_touch_suppress_ms.attr,
	&dev_attr_nvt_standby.attr,
	&dev_attr_nvt_dump_data.attr,
	NULL,
};

static struct attribute_group nvt_ts_attribute_group = {
	.attrs = nvt_ts_attributes,
};

#define NVT_TOUCH_SYSFS_LINK "nvt_touch"
int32_t nvt_touch_sysfs_init(void)
{
	int32_t ret = 0;

	NVT_LOG("nvt create sysfs init\n");
	ret = sysfs_create_link(ts->input_dev->dev.kobj.parent, &ts->input_dev->dev.kobj, NVT_TOUCH_SYSFS_LINK);
	if (ret != 0) {
		NVT_ERR("nvt sysfs create link %s failed. ret=%d", NVT_TOUCH_SYSFS_LINK, ret);
		goto exit_nvt_touch_sysfs_init;
	}

	ret = sysfs_create_group(&ts->input_dev->dev.kobj, &nvt_ts_attribute_group);
	if (ret != 0) {
		NVT_ERR("nvt sysfs create group failed. ret=%d\n", ret);
		goto exit_nvt_touch_sysfs_init;
	}

exit_nvt_touch_sysfs_init:
	return ret;
}

void nvt_touch_sysfs_deinit(void)
{
	NVT_LOG("nvt sysfs deinit\n");
	sysfs_remove_group(&ts->input_dev->dev.kobj, &nvt_ts_attribute_group);

	sysfs_remove_link(ts->input_dev->dev.kobj.parent, NVT_TOUCH_SYSFS_LINK);

#if GOOGLE_TOUCH_EXT_PROC
	if (NVT_proc_reset_entry != NULL) {
		remove_proc_entry(NVT_RESET, NULL);
		NVT_proc_reset_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_RESET);
	}

	if (NVT_proc_scan_mode_entry != NULL) {
		remove_proc_entry(NVT_SCAN_MODE, NULL);
		NVT_proc_scan_mode_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_SCAN_MODE);
	}

	if (NVT_proc_hopping_freq_entry != NULL) {
		remove_proc_entry(NVT_HOPPING_FREQ, NULL);
		NVT_proc_hopping_freq_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_HOPPING_FREQ);
	}

	if (NVT_proc_sensing_entry != NULL) {
		remove_proc_entry(NVT_SENSING, NULL);
		NVT_proc_sensing_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_SENSING);
	}

	if (NVT_proc_trimid_entry != NULL) {
		remove_proc_entry(NVT_TRIMID, NULL);
		NVT_proc_trimid_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_TRIMID);
	}

	if (NVT_proc_history_event_entry != NULL) {
		remove_proc_entry(NVT_HISTORY_EVENT, NULL);
		NVT_proc_history_event_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_HISTORY_EVENT);
	}
#endif
}
#endif
#endif
