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

#ifndef _NANOHUB_COMMS_H
#define _NANOHUB_COMMS_H

struct __attribute__ ((__packed__)) nanohub_packet {
	uint8_t sync;
	uint32_t seq;
	uint32_t id : 8;
	uint32_t reason : 24;
	uint8_t len;
	uint8_t data[];
};

struct __attribute__ ((__packed__)) nanohub_packet_pad {
	uint8_t pad[3];
	struct nanohub_packet
	 packet;
};

struct __attribute__ ((__packed__)) nanohub_packet_crc {
	uint32_t crc;
};

struct nanohub_data;

struct nanohub_comms {
	uint32_t seq;
	int timeout_write;
	int timeout_read;
	int timeout_ack;
	int timeout_reply;
	int (*open)(void *);
	void (*close)(void *);
	int (*write)(void *, uint8_t *, int, int);
	int (*read)(void *, uint8_t *, int, int);

	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
};

int nanohub_comms_kernel_download(struct nanohub_data *, const uint8_t *,
				  size_t);
int nanohub_comms_app_download(struct nanohub_data *, const uint8_t *, size_t);
int nanohub_comms_rx_retrans_boottime(struct nanohub_data *, uint32_t, uint8_t,
				      uint8_t *, uint8_t *, size_t, int, int);
int nanohub_comms_tx_rx_retrans(struct nanohub_data *, uint32_t, uint8_t,
				const uint8_t *, uint8_t, uint8_t *, uint8_t *,
				size_t, bool, int, int);

#define ERROR_NACK			-1
#define ERROR_BUSY			-2

#define MAX_UINT8			((1 << (8*sizeof(uint8_t))) - 1)

#define COMMS_SYNC			0x31
#define COMMS_FLASH_KERNEL_ID		0x1
#define COMMS_FLASH_EEDATA_ID		0x2
#define COMMS_FLASH_APP_ID		0x4

#define CMD_COMMS_ACK			0x000000
#define CMD_COMMS_NACK			0x000001
#define CMD_COMMS_BUSY			0x000002

#define CMD_COMMS_GET_OS_HW_VERSIONS	0x001000
#define CMD_COMMS_GET_APP_VERSIONS	0x001001
#define CMD_COMMS_QUERY_APP_INFO	0x001002
#define CMD_COMMS_TIME_SYNC		0x001003
#define CMD_COMMS_MCU_WAKE_LOCK		0x001004

#define CMD_COMMS_START_KERNEL_UPLOAD	0x001040
#define CMD_COMMS_KERNEL_CHUNK		0x001041
#define CMD_COMMS_FINISH_KERNEL_UPLOAD	0x001042

#define CMD_COMMS_START_APP_UPLOAD	0x001050
#define CMD_COMMS_APP_CHUNK		0x001051

#define CMD_COMMS_CLR_GET_INTR		0x001080
#define CMD_COMMS_MASK_INTR		0x001081
#define CMD_COMMS_UNMASK_INTR		0x001082
#define CMD_COMMS_READ			0x001090
#define CMD_COMMS_WRITE			0x001091

#define CHUNK_REPLY_ACCEPTED		0
#define CHUNK_REPLY_WAIT                1
#define CHUNK_REPLY_RESEND              2
#define CHUNK_REPLY_RESTART             3
#define CHUNK_REPLY_CANCEL              4
#define CHUNK_REPLY_CANCEL_NO_RETRY     5

#define UPLOAD_REPLY_SUCCESS			0
#define UPLOAD_REPLY_PROCESSING			1
#define UPLOAD_REPLY_WAITING_FOR_DATA		2
#define UPLOAD_REPLY_APP_SEC_KEY_NOT_FOUND	3
#define UPLOAD_REPLY_APP_SEC_HEADER_ERROR	4
#define UPLOAD_REPLY_APP_SEC_TOO_MUCH_DATA	5
#define UPLOAD_REPLY_APP_SEC_TOO_LITTLE_DATA	6
#define UPLOAD_REPLY_APP_SEC_SIG_VERIFY_FAIL	7
#define UPLOAD_REPLY_APP_SEC_SIG_DECODE_FAIL	8
#define UPLOAD_REPLY_APP_SEC_SIG_ROOT_UNKNOWN	9
#define UPLOAD_REPLY_APP_SEC_MEMORY_ERROR	10
#define UPLOAD_REPLY_APP_SEC_INVALID_DATA	11
#define UPLOAD_REPLY_APP_SEC_BAD		12

static inline int nanohub_comms_write(struct nanohub_data *data, uint8_t id,
				      const uint8_t *buffer, size_t buffer_len, bool user)
{
	uint8_t ret;
	if (nanohub_comms_tx_rx_retrans
	    (data, CMD_COMMS_WRITE, id, buffer, buffer_len, NULL, &ret, sizeof(ret), user,
	     10, 10) == sizeof(ret)) {
		if (ret)
			return buffer_len;
		else
			return 0;
	} else {
		return ERROR_NACK;
	}
}

#endif
