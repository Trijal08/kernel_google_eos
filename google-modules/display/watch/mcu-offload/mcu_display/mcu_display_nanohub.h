/* SPDX-License-Identifier: GPL-2.0-only */
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
#ifndef _MCU_DISPLAY_NANOHUB_H_
#define _MCU_DISPLAY_NANOHUB_H_

enum mcu_display_ownership {
	SKIP_DISPLAY_OWNERSHIP_CHECKING,
	CHECK_DISPLAY_OWNERSHIP,
};

bool mcu_display_register_nanohub(void);

void mcu_display_unregister_nanohub(void);

int mcu_display_nanohub_get_display_state(enum mcu_display_ownership check_ownership);

#endif /* _MCU_DISPLAY_NANOHUB_H_ */
