// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Google, Inc.
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
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>

#include "mcu_display_nanohub.h"
#include "nanohub_exports.h"

#define NANOHUB_DISPLAY_COMMAND_VERSION		0x01
#define NANOHUB_DISPLAY_GET_STATE_TIMEOUT_MS	100
#define DISPLAY_STATE_MESSAGE_SIZE		3

static DEFINE_MUTEX(nanohub_display_mutex);
static DECLARE_COMPLETION(message_callback);

static int display_state;

enum nanohub_display_command_type {
	NANOHUB_DISPLAY_COMMAND_GET_STATE = 0x06,
	NANOHUB_DISPLAY_COMMAND_GET_STATE_NO_CHECK = 0x07,
};

static void on_message_received(const char *buffer, size_t length)
{
	if (length == DISPLAY_STATE_MESSAGE_SIZE && buffer[0] == NANOHUB_DISPLAY_COMMAND_VERSION &&
	    buffer[1] == NANOHUB_DISPLAY_COMMAND_GET_STATE) {
		display_state = buffer[2];
	}
	complete(&message_callback);
}

bool mcu_display_register_nanohub(void)
{
	return nanohub_register_listener(NANOHUB_DISPLAY_KERNEL_CHANNEL_ID, on_message_received);
}

void mcu_display_unregister_nanohub(void)
{
	nanohub_unregister_listener(NANOHUB_DISPLAY_KERNEL_CHANNEL_ID);
}

int mcu_display_nanohub_get_display_state(enum mcu_display_ownership check_ownership)
{
	const char message[] = { NANOHUB_DISPLAY_COMMAND_VERSION,
				 check_ownership == CHECK_DISPLAY_OWNERSHIP ? NANOHUB_DISPLAY_COMMAND_GET_STATE :
						   NANOHUB_DISPLAY_COMMAND_GET_STATE_NO_CHECK };
	long timeout;

	mutex_lock(&nanohub_display_mutex);

	reinit_completion(&message_callback);
	display_state = -ENODATA;

	nanohub_send_message(NANOHUB_DISPLAY_CHANNEL_ID, message, sizeof(message));

	timeout = wait_for_completion_interruptible_timeout(
		&message_callback, msecs_to_jiffies(NANOHUB_DISPLAY_GET_STATE_TIMEOUT_MS));

	mutex_unlock(&nanohub_display_mutex);

	if (timeout <= 0) {
		pr_err("mcu_display: failed to query disp, ret = %d\n", timeout);
		if (timeout == 0)
			return -ETIMEDOUT;
		else
			return timeout;
	}

	return display_state;
}
