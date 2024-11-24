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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/sched/signal.h>
#include <linux/types.h>

#include "mcu_display_exports.h"
#include "mcu_display_nanohub.h"

#define MCU_DISPLAY_DEVICE_NAME		"display"
#define MCU_DISPLAY_MAX_CHANNEL		1

#define DISPLAY_COMMS_NAME		"display_comms"
#define DISPLAY_COMMS_CHANNEL_ID	0

#define DISPLAY_SELECT_GPIO_LABEL	"mcu_display_display_select"

struct mcu_display_data {
	struct device *display_comms_dev;
	struct blocking_notifier_head panel_power_notifier;
	u32 display_select_gpio;
};

static struct class *mcu_display_class;
static int dev_number_major;

static struct mcu_display_data *priv_mcu_display_data;

static const struct file_operations mcu_display_fops = {
	.owner = THIS_MODULE,
};

int mcu_display_register_panel_power_notifier(struct notifier_block *nb) {
	int rc;

	if (!priv_mcu_display_data) {
		pr_err("mcu_display: priv_mcu_display_data not ready!\n");
		return -ENODEV;
	}

	rc = blocking_notifier_chain_register(&priv_mcu_display_data->panel_power_notifier, nb);
	pr_info("mcu_display: blocking_notifier_chain_register rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(mcu_display_register_panel_power_notifier);

int mcu_display_unregister_panel_power_notifier(struct notifier_block *nb) {
	int rc;

	if (!priv_mcu_display_data) {
		pr_err("mcu_display: priv_mcu_display_data not ready!\n");
		return -ENODEV;
	}

	rc = blocking_notifier_chain_unregister(&priv_mcu_display_data->panel_power_notifier, nb);
	pr_info("mcu_display: blocking_notifier_chain_unregister rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(mcu_display_unregister_panel_power_notifier);

static int mcu_display_query_display_state_internal(enum mcu_display_ownership check_ownership)
{
	struct mcu_display_data *pdata = priv_mcu_display_data;

	if (pdata == NULL)
		return -ENODEV;

	if (check_ownership == CHECK_DISPLAY_OWNERSHIP) {
		if (gpio_get_value(pdata->display_select_gpio) == 1)
			return MCU_DISPLAY_NONE;
	}

	return mcu_display_nanohub_get_display_state(check_ownership);
}

int mcu_display_query_display_state(void)
{
	return mcu_display_query_display_state_internal(CHECK_DISPLAY_OWNERSHIP);
}
EXPORT_SYMBOL(mcu_display_query_display_state);

static int mcu_display_query_display_state_no_check(void)
{
	return mcu_display_query_display_state_internal(SKIP_DISPLAY_OWNERSHIP_CHECKING);
}

static ssize_t display_select_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct mcu_display_data *pdata = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->display_select_gpio));
}

static ssize_t display_select_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 0);
	struct mcu_display_data *pdata = dev_get_drvdata(dev);

	if (gpio_get_value(pdata->display_select_gpio) == value) {
		return count;
	}

	if (value == 0) {
		gpio_set_value(pdata->display_select_gpio, 0);
	} else if (value == 1) {
		int power_mode = mcu_display_query_display_state_no_check();

		blocking_notifier_call_chain(&pdata->panel_power_notifier, power_mode, NULL);
		gpio_set_value(pdata->display_select_gpio, 1);
	}
	return count;
}

static ssize_t display_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int state = mcu_display_query_display_state_internal(CHECK_DISPLAY_OWNERSHIP);
	return scnprintf(buf, PAGE_SIZE, "%d", state);
}

static ssize_t display_state_no_check_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	int state = mcu_display_query_display_state_internal(SKIP_DISPLAY_OWNERSHIP_CHECKING);
	return scnprintf(buf, PAGE_SIZE, "%d", state);
}

DEVICE_ATTR(display_select, 0660, display_select_show, display_select_store);
DEVICE_ATTR(display_state, 0440, display_state_show, NULL);
DEVICE_ATTR(display_state_no_check, 0440, display_state_no_check_show, NULL);

static struct attribute *dev_attrs[] = {
	&dev_attr_display_select.attr,
	&dev_attr_display_state.attr,
	&dev_attr_display_state_no_check.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dev);

static int mcu_display_create_device(struct mcu_display_data *pdata)
{
	int ret = 0;
	struct device *dev;

	dev = device_create_with_groups(mcu_display_class, NULL, MKDEV(dev_number_major, DISPLAY_COMMS_CHANNEL_ID), pdata, dev_groups, DISPLAY_COMMS_NAME);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("mcu_display: device create failed; err=%d\n", ret);
		return ret;
	}

	pdata->display_comms_dev = dev;
	return 0;
}

static void mcu_display_remove_device(void)
{
	device_destroy(mcu_display_class, MKDEV(dev_number_major, DISPLAY_COMMS_CHANNEL_ID));
}

static int mcu_display_init_gpio(struct mcu_display_data *pdata, struct device *dev)
{
	int ret = 0;
	struct device_node *pnode = dev->of_node;

	pdata->display_select_gpio =
		of_get_named_gpio(pnode, "display_pins,display-mux-select-gpio", 0);

	if (gpio_is_valid(pdata->display_select_gpio)) {
		ret = gpio_request_one(pdata->display_select_gpio, GPIOF_OUT_INIT_HIGH,
				       DISPLAY_SELECT_GPIO_LABEL);
		if (ret) {
			dev_err(dev, "gpio request failed;err=%d\n", ret);
			return ret;
		}
	} else {
		dev_err(dev, "invalid gpio: %u", pdata->display_select_gpio);
		return -EINVAL;
	}

	return 0;
}

static void mcu_display_release_gpio(struct mcu_display_data *pdata)
{
	gpio_free(pdata->display_select_gpio);
}

static int mcu_display_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mcu_display_data *pdata;

	if (!mcu_display_register_nanohub())
		return -EPROBE_DEFER;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto fail_kzalloc;
	}

	ret = mcu_display_init_gpio(pdata, &pdev->dev);
	if (ret)
		goto fail_kzalloc;

	ret = mcu_display_create_device(pdata);
	if (ret)
		goto fail_create_device;

	priv_mcu_display_data = pdata;

	platform_set_drvdata(pdev, pdata);
	BLOCKING_INIT_NOTIFIER_HEAD(&pdata->panel_power_notifier);
	return 0;

fail_create_device:
	mcu_display_release_gpio(pdata);
fail_kzalloc:
	mcu_display_unregister_nanohub();
	return ret;
}

static int mcu_display_remove(struct platform_device *pdev)
{
	struct mcu_display_data *pdata = platform_get_drvdata(pdev);

	mcu_display_release_gpio(pdata);
	mcu_display_remove_device();
	mcu_display_unregister_nanohub();
	priv_mcu_display_data = NULL;
	return 0;
}

static const struct of_device_id mcu_display_of_match[] = {
	{
		.compatible = "google,mcu_display",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mcu_display_of_match);

static struct platform_driver mcu_display_driver = {
	.driver = {
			.name = "mcu_display",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(mcu_display_of_match),
		},
	.probe = mcu_display_probe,
	.remove = mcu_display_remove,
};

static int __init mcu_display_init(void)
{
	int ret = 0;

	mcu_display_class = class_create(THIS_MODULE, MCU_DISPLAY_DEVICE_NAME);
	if (IS_ERR(mcu_display_class)) {
		ret = PTR_ERR(mcu_display_class);
		pr_err("mcu_display: class_create failed; err=%d\n", ret);
		return ret;
	}

	dev_number_major = __register_chrdev(0, 0, MCU_DISPLAY_MAX_CHANNEL,
					MCU_DISPLAY_DEVICE_NAME, &mcu_display_fops);

	if (dev_number_major < 0) {
		ret = dev_number_major;
		pr_err("mcu_display: can't register character dev; err=%d\n", ret);
		class_destroy(mcu_display_class);
		return ret;
	}

	return platform_driver_register(&mcu_display_driver);
}

static void __exit mcu_display_exit(void)
{
	platform_driver_unregister(&mcu_display_driver);
	__unregister_chrdev(dev_number_major, 0, MCU_DISPLAY_MAX_CHANNEL,
			    MCU_DISPLAY_DEVICE_NAME);
	class_destroy(mcu_display_class);
}

module_init(mcu_display_init);
module_exit(mcu_display_exit);

MODULE_DESCRIPTION("MCU display module");
MODULE_LICENSE("GPL v2");
