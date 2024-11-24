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
#define CHIP_VER_TRIM_ADDR 0x3F004
#define CHIP_VER_TRIM_OLD_ADDR 0x1F64E

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_BEGIN_ADDR;
	uint32_t EVENT_BUF_END_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t BASELINE_CHECK_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
	uint32_t READ_FLASH_CHECKSUM_ADDR;
	uint32_t RW_FLASH_DATA_ADDR;
	uint32_t FW_HISTORY_BEGIN_ADDR;
	uint32_t FW_HISTORY_END_ADDR;
	uint32_t FW_HISTORY_MID_ADDR;
	uint32_t FW_HISTORY_WDT_ADDR;
	uint32_t FW_STATE_BEGIN_ADDR;
	uint32_t FW_STATE_END_ADDR;
	uint32_t FW_VERSION_ADDR;
	uint32_t FW_PID_ADDR;
};

struct nvt_ts_hw_info {
	uint8_t hw_crc;
};


static const struct nvt_ts_mem_map NT38350_memory_map = {
	.EVENT_BUF_BEGIN_ADDR     = 0x20000,
	.EVENT_BUF_END_ADDR       = 0x20011,
	.RAW_PIPE0_ADDR           = 0x20160,
	.RAW_PIPE1_ADDR           = 0x20660,
	.BASELINE_ADDR            = 0x20268,
	.BASELINE_BTN_ADDR        = 0x20768,
	.BASELINE_CHECK_ADDR      = 0x20286,
	.DIFF_PIPE0_ADDR          = 0x201FC,
	.DIFF_PIPE1_ADDR          = 0x206FC,
	.RAW_BTN_PIPE0_ADDR       = 0x20260,
	.RAW_BTN_PIPE1_ADDR       = 0x20260,
	.DIFF_BTN_PIPE0_ADDR      = 0x20760,
	.DIFF_BTN_PIPE1_ADDR      = 0x20760,
	.READ_FLASH_CHECKSUM_ADDR = 0x21000,
	.RW_FLASH_DATA_ADDR       = 0x21002,
	.FW_HISTORY_BEGIN_ADDR    = 0x20A00,
	.FW_HISTORY_END_ADDR      = 0x20AA0,
	.FW_HISTORY_MID_ADDR      = 0x20A50,
	.FW_HISTORY_WDT_ADDR      = 0x209C8,
	.FW_STATE_BEGIN_ADDR      = 0x20050,
	.FW_STATE_END_ADDR        = 0x20062,
	.FW_VERSION_ADDR          = 0x20AC0,
	.FW_PID_ADDR              = 0x20AD6,
};

static const struct nvt_ts_mem_map NT38360_memory_map = {
	.EVENT_BUF_BEGIN_ADDR     = 0x20000,
	.EVENT_BUF_END_ADDR       = 0x20011,
	.RAW_PIPE0_ADDR           = 0x201E0,
	.RAW_PIPE1_ADDR           = 0x208E0,
	.BASELINE_ADDR            = 0x20328,
	.BASELINE_BTN_ADDR        = 0x20328,
	.BASELINE_CHECK_ADDR      = 0x2035A,
	.DIFF_PIPE0_ADDR          = 0x202A8,
	.DIFF_PIPE1_ADDR          = 0x209A8,
	.RAW_BTN_PIPE0_ADDR       = 0x201E0,
	.RAW_BTN_PIPE1_ADDR       = 0x208E0,
	.DIFF_BTN_PIPE0_ADDR      = 0x202A8,
	.DIFF_BTN_PIPE1_ADDR      = 0x209A8,
	.READ_FLASH_CHECKSUM_ADDR = 0x21000,
	.RW_FLASH_DATA_ADDR       = 0x21002,
	.FW_HISTORY_BEGIN_ADDR    = 0x20E00,
	.FW_HISTORY_END_ADDR      = 0x20EA0,
	.FW_HISTORY_MID_ADDR      = 0x20E50,
	.FW_HISTORY_WDT_ADDR      = 0x20CCC,
	.FW_STATE_BEGIN_ADDR      = 0x20050,
	.FW_STATE_END_ADDR        = 0x20062,
	.FW_VERSION_ADDR          = 0x20EC0,
	.FW_PID_ADDR              = 0x20ED6,
};

static struct nvt_ts_hw_info NT38350_hw_info = {
	.hw_crc         = 1,
};

static struct nvt_ts_hw_info NT38360_hw_info = {
	.hw_crc         = 1,
};

#define NVT_ID_BYTE_MAX 6
struct nvt_ts_trim_id_table {
	const char name[10];
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	const struct nvt_ts_hw_info *hwinfo;
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
	{"NT38350", .id = {0xFF, 0xFF, 0xFF, 0x50, 0x83, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT38350_memory_map,  .hwinfo = &NT38350_hw_info},
	{"NT38360", .id = {0xFF, 0xFF, 0xFF, 0x60, 0x83, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT38360_memory_map,  .hwinfo = &NT38360_hw_info},
};
