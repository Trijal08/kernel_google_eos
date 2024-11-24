/*
 * Copyright (C) 2022 Google, Inc.
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

#ifndef _NANOHUB_BL_NXP_H
#define _NANOHUB_BL_NXP_H

#include "nanohub.h"

struct nanohub_data;

struct nanohub_bl {
	int (*open)(const void *);
	void (*close)(const void *);
	int (*read_ack)(const void *);
	int (*read_response)(const void *);
	int (*write_ack)(const void *);
	int (*write_cmd)(const void *, uint8_t cmd, uint8_t flags,
			     int n_args, ...);
	int (*write_data)(const void *, const uint8_t *buf, int size);

	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
};

int nanohub_bl_open(struct nanohub_data *);
void nanohub_bl_close(struct nanohub_data *);

int nanohub_bl_set_property(struct nanohub_data *data, uint32_t tag,
			    uint32_t value);
int nanohub_bl_write_memory(struct nanohub_data *data, const uint8_t *firmware,
			    uint32_t size, uint32_t address,
			    uint32_t *data_abort_offset);
int nanohub_bl_execute(struct nanohub_data *data, uint32_t prgm_counter,
		       uint32_t ram_addr, uint32_t stack_ptr);

int nanohub_bl_download_firmware(struct nanohub_data *, const uint8_t *firmware,
				uint32_t size);
int nanohub_bl_download_firmware_buffer(struct nanohub_data *,
					const uint8_t *firmware,
					uint32_t size);

#define BL_COMMAND_WRITE_MEMORY		0x04
#define BL_COMMAND_EXECUTE		0x09
#define BL_COMMAND_SET_PROPERTY		0x0C

#define BL_RESPONSE_GENERIC		0xA0

#define BL_FLAG_HAS_DATA		0x01

#define BL_FRAME_SYNC			0x5A
#define BL_FRAME_ACK			0xA1
#define BL_FRAME_NACK			0xA2
#define BL_FRAME_ABORT			0xA3
#define BL_FRAME_COMMAND		0xA4
#define BL_FRAME_DATA			0xA5
#define BL_FRAME_PING			0xA6
#define BL_FRAME_PING_RESPONSE		0xA7

// XXX find system errors for these, will still clash with nxp status though
// so maybe these should be in a unique scope?
#define BL_ERR_TIMEOUT			52
#define BL_ERR_CRC			53
#define BL_ERR_SYNC			54
#define BL_ERR_BAD_RESPONSE		55

// Optimized for burst transfers (total packet multiple of 4)
#define BL_DATA_CHUNK_SIZE 510

static inline void put_le16(uint8_t *buf, uint16_t i) {
	buf[0] = i & 0xff;
	buf[1] = (i >> 8) & 0xff;
}

static inline void put_le32(uint8_t *buf, uint32_t i) {
	buf[0] = i & 0xff;
	buf[1] = (i >> 8) & 0xff;
	buf[2] = (i >> 16) & 0xff;
	buf[3] = (i >> 24) & 0xff;
}

static inline uint16_t get_le16(uint8_t *buf) {
	return buf[0] | buf[1] << 8;
}

static inline uint32_t get_le32(uint8_t *buf) {
	return buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
}

#endif
