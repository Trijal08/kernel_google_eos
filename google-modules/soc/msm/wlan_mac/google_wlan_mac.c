// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <asm/setup.h>
#include <linux/export.h>
#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "cnss_utils.h"

#define CDB_PATH "/chosen/config"
#define WIFI_MAC "wlan_mac1"

static int set_wifi_mac(void)
{
	const struct device_node *node;
	const char *mac_addr = NULL;
	int size;
	u8 mac[ETH_ALEN] = { 0 };
	bool mac_found = false;
	int ret = 0;

	node = of_find_node_by_path(CDB_PATH);
	if (!node) {
		pr_err("Unable to locate %s in device-tree\n", CDB_PATH);
		return -ENOENT;
	} else {
		mac_addr = (const char *)
			of_get_property(node, WIFI_MAC, &size);
		if (!mac_addr) {
			pr_err("No MAC address found in device-tree\n");
			return -ENOENT;
		}
		pr_info("MAC address from device-tree: %s\n", mac_addr);
	}

	/* Parse MAC address. Two formats are supported:
	 * AA:BB:CC:DD:EE:FF and AABBCCDDEEFF.
	 */
	if (sscanf(mac_addr,
			"%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
			&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
		mac_found = true;
	} else if (sscanf(mac_addr,
				"%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
				&mac[0], &mac[1], &mac[2], &mac[3], &mac[4],
				&mac[5]) == 6) {
		mac_found = true;
	}

	if (!mac_found) {
		pr_err("Invalid format for provisioned MAC address\n");
		return -EINVAL;
	}

	/* Ensure sure provisioned MAC address is globally administered */
	if (mac[0] & 2) {
		pr_err("Invalid provisioned MAC address\n");
		return -EINVAL;
	}

	ret = cnss_utils_set_wlan_mac_address(mac, sizeof(mac));
	if (ret != 0) {
		pr_err("setting WLAN MAC address failed\n");
		return ret;
	}

	/* Make derived mac address by flipping the locally administered bit */
	mac[0] = mac[0] | 2;

	ret = cnss_utils_set_wlan_derived_mac_address(mac, sizeof(mac));
	if (ret != 0) {
		pr_err("Setting WLAN derived MAC address failed\n");
		return ret;
	}

	pr_info("WLAN MAC address set to %s\n", mac_addr);
	return 0;
}

static int __init wifi_mac_init(void)
{
	return set_wifi_mac();
}

late_initcall(wifi_mac_init);

MODULE_AUTHOR("Google");
MODULE_DESCRIPTION("Sets QCOM WLAN MAC from device-tree");
MODULE_LICENSE("GPL v2");
