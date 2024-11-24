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

#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include "main.h"
#include "comms.h"

#ifdef CONFIG_NANOHUB_BL_ST
#include "bl_st.h"
#endif

#ifdef CONFIG_NANOHUB_BL_NXP
#include "bl_nxp.h"
#endif

#define SPI_TIMEOUT		65535
#define SPI_MIN_DMA		48
#define SPI_MAX_SPEED_HZ	10000000
#define SPI_BITS_PER_WORD	8
#define DMA_BURST_WIDTH		4
#define DMA_PAD			3

struct nanohub_spi_data {
	struct nanohub_data data;
	struct spi_device *device;
	struct semaphore spi_sem;
	int cs;
	uint16_t rx_length;
	uint16_t rx_offset;
};

// Set cs if it is under nanohub control.
static void cs_set_value(const struct nanohub_spi_data *spi_data, int value) {
	if (gpio_is_valid(spi_data->cs)) {
		gpio_set_value(spi_data->cs, value);
	}
}

#ifdef CONFIG_NANOHUB_BL_ST
static uint8_t bl_checksum(const uint8_t *bytes, int length)
{
	int i;
	uint8_t csum;

	if (length == 1) {
		csum = ~bytes[0];
	} else if (length > 1) {
		for (csum = 0, i = 0; i < length; i++)
			csum ^= bytes[i];
	} else {
		csum = 0xFF;
	}

	return csum;
}

static uint8_t spi_bl_write_data(const void *data, uint8_t *tx, int length)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = length + 1,
		.tx_buf = bl->tx_buffer,
		.rx_buf = bl->rx_buffer,
		.cs_change = 1,
	};

	tx[length] = bl_checksum(tx, length);
	memcpy(bl->tx_buffer, tx, length + 1);

	spi_message_init_with_transfers(&msg, &xfer, 1);

	if (spi_sync_locked(spi_data->device, &msg) == 0)
		return bl->rx_buffer[length];
	else
		return CMD_NACK;
}

static uint8_t spi_bl_write_cmd(const void *data, uint8_t cmd)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = bl->tx_buffer,
		.rx_buf = bl->rx_buffer,
		.cs_change = 1,
	};
	bl->tx_buffer[0] = CMD_SOF;
	bl->tx_buffer[1] = cmd;
	bl->tx_buffer[2] = ~cmd;

	spi_message_init_with_transfers(&msg, &xfer, 1);

	if (spi_sync_locked(spi_data->device, &msg) == 0)
		return CMD_ACK;
	else
		return CMD_NACK;
}

static uint8_t spi_bl_read_data(const void *data, uint8_t *rx, int length)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = length + 1,
		.tx_buf = bl->tx_buffer,
		.rx_buf = bl->rx_buffer,
		.cs_change = 1,
	};
	memset(&bl->tx_buffer[0], 0x00, length + 1);

	spi_message_init_with_transfers(&msg, &xfer, 1);

	if (spi_sync_locked(spi_data->device, &msg) == 0) {
		memcpy(rx, &bl->rx_buffer[1], length);
		return CMD_ACK;
	} else {
		return CMD_NACK;
	}
}

static uint8_t spi_bl_read_ack(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	int32_t timeout = SPI_TIMEOUT;
	uint8_t ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 1,
		.tx_buf = bl->tx_buffer,
		.rx_buf = bl->rx_buffer,
		.cs_change = 1,
	};
	bl->tx_buffer[0] = 0x00;

	spi_message_init_with_transfers(&msg, &xfer, 1);

	if (spi_sync_locked(spi_data->device, &msg) == 0) {
		do {
			spi_sync_locked(spi_data->device, &msg);
			timeout--;
			if (bl->rx_buffer[0] != CMD_ACK
			    && bl->rx_buffer[0] != CMD_NACK
			    && timeout % 256 == 0)
				schedule();
		} while (bl->rx_buffer[0] != CMD_ACK
			 && bl->rx_buffer[0] != CMD_NACK && timeout > 0);

		if (bl->rx_buffer[0] != CMD_ACK && bl->rx_buffer[0] != CMD_NACK
		    && timeout == 0)
			ret = CMD_NACK;
		else
			ret = bl->rx_buffer[0];

		bl->tx_buffer[0] = CMD_ACK;
		spi_sync_locked(spi_data->device, &msg);
		return ret;
	} else {
		return CMD_NACK;
	}
}

static int spi_bl_open(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;
	int ret;

	spi_bus_lock(spi_data->device->master);
	spi_data->device->max_speed_hz = spi_data->data.pdata->bl_max_speed_hz;
	spi_data->device->mode = SPI_MODE_0;
	spi_data->device->bits_per_word = 8;
	ret = spi_setup(spi_data->device);
	if (!ret)
		cs_set_value(spi_data, 0);

	return ret;
}

static void spi_bl_close(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;

	cs_set_value(spi_data, 1);
	spi_bus_unlock(spi_data->device->master);
}

static uint8_t spi_bl_sync(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	int32_t timeout = SPI_TIMEOUT;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 1,
		.tx_buf = bl->tx_buffer,
		.rx_buf = bl->rx_buffer,
		.cs_change = 1,
	};
	bl->tx_buffer[0] = CMD_SOF;

	spi_message_init_with_transfers(&msg, &xfer, 1);

	do {
		if (spi_sync_locked(spi_data->device, &msg) != 0)
			return CMD_NACK;
		timeout--;
		if (bl->rx_buffer[0] != CMD_SOF_ACK && timeout % 256 == 0)
			schedule();
	} while (bl->rx_buffer[0] != CMD_SOF_ACK && timeout > 0);

	if (bl->rx_buffer[0] == CMD_SOF_ACK)
		return bl->read_ack(data);
	else
		return CMD_NACK;
}

void nanohub_spi_bl_init(struct nanohub_spi_data *spi_data)
{
	struct nanohub_bl *bl = &spi_data->data.bl;

	bl->open = spi_bl_open;
	bl->sync = spi_bl_sync;
	bl->write_data = spi_bl_write_data;
	bl->write_cmd = spi_bl_write_cmd;
	bl->read_data = spi_bl_read_data;
	bl->read_ack = spi_bl_read_ack;
	bl->close = spi_bl_close;
}

#endif


#ifdef CONFIG_NANOHUB_BL_NXP

#define CRC_POLY 0x1021

static uint16_t crc_update(uint16_t crc_in, int incr)
{
        uint16_t xor = crc_in >> 15;
        uint16_t out = crc_in << 1;
        if (incr)
                out++;
        if (xor)
                out ^= CRC_POLY;
        return out;
}

// Calculate a packet CRC, ignoring the CRC bytes at offset 4 & 5.
static uint16_t bl_crc16(const uint8_t *data, uint16_t size)
{
        uint16_t crc, i, j;
        for (crc = 0, j = 0; j < size; j++, data++) {
		if (j == 4 || j == 5)
			continue; // skip the CRC word
                for (i = 0x80; i; i >>= 1)
                        crc = crc_update(crc, *data & i);
	}
        for (i = 0; i < 16; i++)
                crc = crc_update(crc, 0);
        return crc;
}

enum bl_readiness {
	BL_READY_READ = 0,
	BL_READY_WRITE = 1,
	BL_READY_NONE = 2,
};

static int spi_bl_wait(const void *data, enum bl_readiness bl_ready_state)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_platform_data *pdata = spi_data->data.pdata;
	int retry, delay;

	if (bl_ready_state == BL_READY_NONE)
		return 0;
	for (retry = 0, delay = 0; retry < 20; retry++) {
		if (gpio_get_value(pdata->irq1_gpio) == bl_ready_state)
			return 0;
		delay += 10;
		udelay(delay);
	}
	pr_warn("nanohub: %s timed out waiting for bootloader (%s)\n", __func__,
		bl_ready_state ? "write" : "read");
	return -BL_ERR_TIMEOUT;
}

static int spi_bl_tx_rx(const void *data, struct spi_transfer *xfer,
			enum bl_readiness bl_ready_state)
{
	const struct nanohub_spi_data *spi_data = data;
	struct spi_message msg;
	int ret;

	spi_message_init_with_transfers(&msg, xfer, 1);
	ret = spi_bl_wait(data, bl_ready_state);
	if (ret < 0)
		return ret;
	cs_set_value(spi_data, 0);
	ret = spi_sync_locked(spi_data->device, &msg);
	cs_set_value(spi_data, 1);
	return ret;
}

static int spi_bl_write_ack(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_transfer xfer = {
		.len = 2,
		.tx_buf = bl->tx_buffer,
		.rx_buf = NULL,
		.cs_change = 1,
	};

	bl->tx_buffer[0] = BL_FRAME_SYNC;
	bl->tx_buffer[1] = BL_FRAME_ACK;

	return spi_bl_tx_rx(data, &xfer, BL_READY_WRITE);
}

// Read a packet beginning with a sync byte, eg, ACK and Frame packets.
// Works around some bootloader issues that can cause random errors.
static int spi_bl_read_sync_packet(const void *data, int length)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_transfer xfer = {
		.len = length,
		.tx_buf = NULL,
		.rx_buf = bl->rx_buffer,
		.cs_change = 1,
	};
	int ret;

	ret = spi_bl_tx_rx(data, &xfer, BL_READY_READ);
	if (ret < 0)
		return ret;

	if (bl->rx_buffer[0] != BL_FRAME_SYNC)
		return -BL_ERR_SYNC;

	if (bl->rx_buffer[1] == BL_FRAME_SYNC) {
		// The NXP bootloader can sometimes return an extra sync byte.
		// Discard the extra sync and refetch the last content byte.
		// This also removes the need to wait 50us b/w sync and ack.
		// See ref manual 18.7.2.6.7
		memmove(bl->rx_buffer, bl->rx_buffer + 1, length - 1);
		xfer.len = 1;
		xfer.rx_buf = bl->rx_buffer + length - 1;
		ret = spi_bl_tx_rx(data, &xfer, BL_READY_NONE);
	}

	return ret;
}

static int spi_bl_read_ack(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	uint8_t ack;
	int ret = spi_bl_read_sync_packet(data, 2);
	if (ret < 0)
		return ret;
	ack = bl->rx_buffer[1];
	return ack == BL_FRAME_ACK ? 0 : -ack;
}

static int spi_bl_read_frame(const void *data)
{
	return spi_bl_read_sync_packet(data, 6);
}

// NXP bootloader command and response packet format
// Frame
//    0   sync      5A
//    1   type      A4
//    2   length    2 bytes, length of cmd packet, little endian
//    4   crc       2 bytes, crc of entire packet, not including crc, little endian
// Command or Response
//    6   cmd/resp  1 byte
//    7   flags     1 byte
//    8   reserved  00
//    9   n_args    1 byte
//   10   arg1      4 bytes, little endian
//   14   ...
static int spi_bl_write_cmd(const void *data, uint8_t cmd, uint8_t flags,
			    int n_args, ...)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_transfer xfer = {
		.len = 10 + 4 * n_args,
		.tx_buf = bl->tx_buffer,
		.rx_buf = NULL,
		.cs_change = 1,
	};
	int i, ret;
	uint16_t crc = 0;
	va_list args;

	bl->tx_buffer[0] = BL_FRAME_SYNC;
	bl->tx_buffer[1] = BL_FRAME_COMMAND;
	put_le16(bl->tx_buffer + 2, 4 + 4 * n_args);
	bl->tx_buffer[6] = cmd;
	bl->tx_buffer[7] = flags;
	bl->tx_buffer[8] = 0;
	bl->tx_buffer[9] = n_args;
	va_start(args, n_args);
	for (i = 0; i < n_args; i++) {
		put_le32(bl->tx_buffer + 10 + i * 4, va_arg(args, uint32_t));
	}
	va_end(args);
	crc = bl_crc16(bl->tx_buffer, 10 + 4 * n_args);
	put_le16(bl->tx_buffer + 4, crc);

	ret = spi_bl_tx_rx(data, &xfer, BL_READY_WRITE);
	if (ret < 0)
		return ret;

	return spi_bl_read_ack(data);
}

static int spi_bl_write_data(const void *data, const uint8_t *buf, int length)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_transfer xfer = {
		.len = 6 + length,
		.tx_buf = bl->tx_buffer,
		.rx_buf = NULL,
		.cs_change = 1,
	};
	uint16_t crc = 0;
	int ret;

	bl->tx_buffer[0] = BL_FRAME_SYNC;
	bl->tx_buffer[1] = BL_FRAME_DATA;
	put_le16(bl->tx_buffer + 2, length);
	memcpy(bl->tx_buffer + 6, buf, length);
	crc = bl_crc16(bl->tx_buffer, 6 + length);
	put_le16(bl->tx_buffer + 4, crc);

	ret = spi_bl_tx_rx(data, &xfer, BL_READY_WRITE);
	if (ret < 0)
		return ret;

	return spi_bl_read_ack(data);
}

static int spi_bl_read_response(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;
	const struct nanohub_bl *bl = &spi_data->data.bl;
	struct spi_transfer xfer = {
		.tx_buf = NULL,
		.cs_change = 1,
	};
	int ret;
	uint16_t length, expected_crc, actual_crc;

	// Read the frame
	// This delay allows the MCU to begin a new data ready notification.
	// It could be removed if we changed to an edge-driven interrupt.
	udelay(60);
	ret = spi_bl_read_frame(data);
	if (ret < 0)
		return ret;

	length = get_le16(bl->rx_buffer + 2);
	expected_crc = get_le16(bl->rx_buffer + 4);

	if (length > 32)
		return -BL_ERR_BAD_RESPONSE;

	// Read the response
	xfer.len = length;
	xfer.rx_buf = bl->rx_buffer + 6;

	ret = spi_bl_tx_rx(data, &xfer, BL_READY_NONE);
	if (ret < 0)
		return ret;

	actual_crc = bl_crc16(bl->rx_buffer, 6 + length);
	if (actual_crc != expected_crc)
		return -BL_ERR_CRC;

	return spi_bl_write_ack(data);
}

static int spi_bl_open(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;

	spi_bus_lock(spi_data->device->master);
	return 0;
}

static void spi_bl_close(const void *data)
{
	const struct nanohub_spi_data *spi_data = data;

	spi_bus_unlock(spi_data->device->master);
}

void nanohub_spi_bl_init(struct nanohub_spi_data *spi_data)
{
	struct nanohub_bl *bl = &spi_data->data.bl;

	bl->open = spi_bl_open;
	bl->write_ack = spi_bl_write_ack;
	bl->write_cmd = spi_bl_write_cmd;
	bl->write_data = spi_bl_write_data;
	bl->read_ack = spi_bl_read_ack;
	bl->read_response = spi_bl_read_response;
	bl->close = spi_bl_close;
}

#endif

int nanohub_spi_write(void *data, uint8_t *tx, int length, int timeout)
{
	struct nanohub_spi_data *spi_data = data;
	const struct nanohub_comms *comms = &spi_data->data.comms;
	int max_len = sizeof(struct nanohub_packet) + MAX_UINT8 +
		      sizeof(struct nanohub_packet_crc);
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = max_len + timeout,
		.tx_buf = comms->tx_buffer,
		.rx_buf = comms->rx_buffer,
		.cs_change = 1,
	};
	xfer.len = (xfer.len + DMA_PAD) & ~DMA_PAD;
	spi_data->rx_offset = max_len;
	spi_data->rx_length = xfer.len;
	memcpy(comms->tx_buffer, tx, length);
	memset(comms->tx_buffer + length, 0xFF, xfer.len - length);

	spi_message_init_with_transfers(&msg, &xfer, 1);

	if (spi_sync_locked(spi_data->device, &msg) == 0)
		return length;
	else
		return ERROR_NACK;
}

int nanohub_spi_read(void *data, uint8_t *rx, int max_length, int timeout)
{
	struct nanohub_spi_data *spi_data = data;
	struct nanohub_comms *comms = &spi_data->data.comms;
	const int min_size = sizeof(struct nanohub_packet) +
	    sizeof(struct nanohub_packet_crc);
	int i, ret;
	int offset = 0;
	struct nanohub_packet *packet = NULL;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = timeout,
		.tx_buf = comms->tx_buffer,
		.rx_buf = comms->rx_buffer,
		.cs_change = 1,
	};

	if (max_length < min_size)
		return ERROR_NACK;

	/* consume leftover bytes, if any */
	if (spi_data->rx_offset < spi_data->rx_length) {
		for (i = spi_data->rx_offset; i < spi_data->rx_length; i++) {
			if (comms->rx_buffer[i] != 0xFF) {
				offset = spi_data->rx_length - i;

				if (offset <
				    offsetof(struct nanohub_packet,
					     len) + sizeof(packet->len)) {
					memcpy(rx, &comms->rx_buffer[i],
					       offset);
					xfer.len =
					    min_size + MAX_UINT8 - offset;
					break;
				} else {
					packet =
					    (struct nanohub_packet *)&comms->
					    rx_buffer[i];
					if (offset < min_size + packet->len) {
						memcpy(rx, packet, offset);
						xfer.len =
						    min_size + packet->len -
						    offset;
						break;
					} else {
						memcpy(rx, packet,
						       min_size + packet->len);
						spi_data->rx_offset = i +
						     min_size + packet->len;
						return min_size + packet->len;
					}
				}
			}
		}
	}

	xfer.len = (xfer.len + DMA_PAD) & ~DMA_PAD;
	if (xfer.len != 1 && xfer.len < SPI_MIN_DMA)
		xfer.len = SPI_MIN_DMA;
	memset(comms->tx_buffer, 0xFF, xfer.len);

	spi_message_init_with_transfers(&msg, &xfer, 1);

	ret = spi_sync_locked(spi_data->device, &msg);
	if (ret == 0) {
		if (offset > 0) {
			packet = (struct nanohub_packet *)rx;
			if (offset + xfer.len > max_length)
				memcpy(&rx[offset], comms->rx_buffer,
					max_length - offset);
			else
				memcpy(&rx[offset], comms->rx_buffer, xfer.len);
			spi_data->rx_length = xfer.len;
			spi_data->rx_offset = min_size + packet->len - offset;
		} else {
			for (i = 0; i < xfer.len; i++) {
				if (comms->rx_buffer[i] != 0xFF) {
					spi_data->rx_length = xfer.len;

					if (xfer.len - i < min_size) {
						spi_data->rx_offset = i;
						break;
					} else {
						packet =
						    (struct nanohub_packet *)
						    &comms->rx_buffer[i];
						if (xfer.len - i <
						    min_size + packet->len) {
							packet = NULL;
							spi_data->rx_offset = i;
						} else {
							memcpy(rx, packet,
							       min_size +
							       packet->len);
							spi_data->rx_offset =
							    i + min_size +
							    packet->len;
						}
					}
					break;
				}
			}
		}
	}

	if (ret < 0)
		return ret;
	else if (!packet)
		return 0;
	else
		return min_size + packet->len;
}

static int nanohub_spi_open(void *data)
{
	struct nanohub_spi_data *spi_data = data;

	down(&spi_data->spi_sem);
	spi_bus_lock(spi_data->device->master);
	cs_set_value(spi_data, 0);
	return 0;
}

static void nanohub_spi_close(void *data)
{
	struct nanohub_spi_data *spi_data = data;

	cs_set_value(spi_data, 1);
	spi_bus_unlock(spi_data->device->master);
	up(&spi_data->spi_sem);
}

void nanohub_spi_comms_init(struct nanohub_spi_data *spi_data)
{
	struct nanohub_comms *comms = &spi_data->data.comms;
	int max_write, max_read;
	int max_len = sizeof(struct nanohub_packet) + MAX_UINT8 +
		      sizeof(struct nanohub_packet_crc);

	comms->seq = 1;
	comms->timeout_write = 470;
	comms->timeout_read = 740;
	comms->timeout_ack = 272;
	comms->timeout_reply = 512;
	comms->open = nanohub_spi_open;
	comms->close = nanohub_spi_close;
	comms->write = nanohub_spi_write;
	comms->read = nanohub_spi_read;

	// nanohub_spi_write transactions
	max_write = max_len + comms->timeout_write;
	max_read = max_len + comms->timeout_read;
	max_len = max(max_read, max_write);
	// nanohub_spi_read transactions
	max_len = max(max_len, comms->timeout_ack);
	max_len = max(max_len, comms->timeout_reply);
	max_len = (max_len + DMA_PAD) & ~DMA_PAD;
	comms->tx_buffer = kmalloc(max_len, GFP_KERNEL | GFP_DMA);
	comms->rx_buffer = kmalloc(max_len, GFP_KERNEL | GFP_DMA);

	spi_data->rx_length = 0;
	spi_data->rx_offset = 0;

	sema_init(&spi_data->spi_sem, 1);
}

static int nanohub_spi_probe(struct spi_device *spi)
{
	struct nanohub_spi_data *spi_data;
	struct iio_dev *iio_dev;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 0,
		.tx_buf = NULL,
		.rx_buf = NULL
	};
	int error;

	iio_dev = iio_device_alloc(&spi->dev, sizeof(struct nanohub_spi_data));

	iio_dev = nanohub_probe(&spi->dev, iio_dev);

	if (IS_ERR(iio_dev))
		return PTR_ERR(iio_dev);

	spi_data = iio_priv(iio_dev);

	spi_set_drvdata(spi, iio_dev);

	spi_data->cs = spi_data->data.pdata->spi_cs_gpio;
	if (gpio_is_valid(spi_data->cs)) {
		error = gpio_request(spi_data->cs, "nanohub_spi_cs");
		if (error) {
			spi_data->cs = -1;
			pr_err("nanohub: spi_cs_gpio request failed\n");
		} else {
			gpio_direction_output(spi_data->cs, 1);
		}
	}

	spi_data->device = spi;
	spi_data->data.max_speed_hz = spi->max_speed_hz ? : SPI_MAX_SPEED_HZ;
	nanohub_spi_comms_init(spi_data);

#ifdef CONFIG_NANOHUB_BL_ST
	spi_data->data.bl.cmd_erase = CMD_ERASE;
	spi_data->data.bl.cmd_read_memory = CMD_READ_MEMORY;
	spi_data->data.bl.cmd_write_memory = CMD_WRITE_MEMORY;
	spi_data->data.bl.cmd_get_version = CMD_GET_VERSION;
	spi_data->data.bl.cmd_get_id = CMD_GET_ID;
	spi_data->data.bl.cmd_readout_protect = CMD_READOUT_PROTECT;
	spi_data->data.bl.cmd_readout_unprotect = CMD_READOUT_UNPROTECT;
	spi_data->data.bl.cmd_update_finished = CMD_UPDATE_FINISHED;
	nanohub_spi_bl_init(spi_data);
#endif

#ifdef CONFIG_NANOHUB_BL_NXP
	nanohub_spi_bl_init(spi_data);
#endif

	// Set up the SPI bus and run an empty message to set the idle state.
	// This setup can overridden in nanohub_spi_open and spi_bl_open.
	spi_data->device->max_speed_hz = spi_data->data.max_speed_hz;
	spi_data->device->mode = SPI_MODE_3;
	spi_data->device->bits_per_word = SPI_BITS_PER_WORD;
	spi_setup(spi_data->device);
	spi_message_init_with_transfers(&msg, &xfer, 1);
	spi_sync_locked(spi_data->device, &msg);

	nanohub_start(&spi_data->data);

	return 0;
}

static int nanohub_spi_remove(struct spi_device *spi)
{
	struct nanohub_spi_data *spi_data;
	struct iio_dev *iio_dev;

	iio_dev = spi_get_drvdata(spi);
	spi_data = iio_priv(iio_dev);

	if (gpio_is_valid(spi_data->cs)) {
		gpio_direction_output(spi_data->cs, 1);
		gpio_free(spi_data->cs);
	}

	return nanohub_remove(iio_dev);
}

static int nanohub_spi_suspend(struct device *dev)
{
	struct iio_dev *iio_dev = spi_get_drvdata(to_spi_device(dev));
	struct nanohub_spi_data *spi_data = iio_priv(iio_dev);
	int ret;

	ret = nanohub_suspend(iio_dev);

	if (!ret) {
		ret = down_interruptible(&spi_data->spi_sem);
		if (ret)
			up(&spi_data->spi_sem);
	}

	return ret;
}

static int nanohub_spi_suspend_noirq(struct device *dev)
{
	struct iio_dev *iio_dev = spi_get_drvdata(to_spi_device(dev));
	return nanohub_suspend_noirq(iio_dev);
}

static int nanohub_spi_resume(struct device *dev)
{
	struct iio_dev *iio_dev = spi_get_drvdata(to_spi_device(dev));
	struct nanohub_spi_data *spi_data = iio_priv(iio_dev);

	up(&spi_data->spi_sem);

	return nanohub_resume(iio_dev);
}

static void nanohub_spi_shutdown(struct spi_device *spi)
{
	struct iio_dev *iio_dev = spi_get_drvdata(spi);

	nanohub_shutdown(iio_dev);
}

static struct spi_device_id nanohub_spi_id[] = {
	{NANOHUB_NAME, 0},
	{},
};

static const struct dev_pm_ops nanohub_spi_pm_ops = {
	.suspend = nanohub_spi_suspend,
	.suspend_noirq = nanohub_spi_suspend_noirq,
	.resume = nanohub_spi_resume,
};

static struct spi_driver nanohub_spi_driver = {
	.driver = {
		   .name = NANOHUB_NAME,
		   .owner = THIS_MODULE,
		   .pm = &nanohub_spi_pm_ops,
		   },
	.probe = nanohub_spi_probe,
	.remove = nanohub_spi_remove,
	.shutdown = nanohub_spi_shutdown,
	.id_table = nanohub_spi_id,
};

int __init nanohub_spi_init(void)
{
	return spi_register_driver(&nanohub_spi_driver);
}

void nanohub_spi_cleanup(void)
{
	spi_unregister_driver(&nanohub_spi_driver);
}

MODULE_DEVICE_TABLE(spi, nanohub_spi_id);
