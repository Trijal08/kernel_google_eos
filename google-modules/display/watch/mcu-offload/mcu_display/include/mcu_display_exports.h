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
#ifndef _MCU_DISPLAY_EXPORTS_H_
#define _MCU_DISPLAY_EXPORTS_H_

/**
 * MCU display power state returned from mcu_display_query_display_state
 */
enum mcu_display_mode {
	MCU_DISPLAY_NONE = 0,
	MCU_DISPLAY_INIT = 1,
	MCU_DISPLAY_OFF = 2,
	MCU_DISPLAY_IDLE = 3,
	MCU_DISPLAY_ON = 4,
	MCU_DISPLAY_HIGH_BRIGHTNESS = 5,
};

/**
 * Query the display power state from MCU when display is controlled by MCU.
 * MCU_DISPLAY_NONE, when MCU doesn't control display.
 */
extern int mcu_display_query_display_state(void);

/**
 * Registration function for panel_power_notifier
 **/
extern int mcu_display_register_panel_power_notifier(struct notifier_block *nb);

/**
 * Unregistration function for panel_power_notifier
 **/
extern int mcu_display_unregister_panel_power_notifier(struct notifier_block *nb);

#endif /* _MCU_DISPLAY_EXPORTS_H_ */
