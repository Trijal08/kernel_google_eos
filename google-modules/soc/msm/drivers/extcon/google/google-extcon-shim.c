/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Google LLC */

#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <dt-bindings/extcon/google,extcon-shim.h>

#define NUM_SUPPORTED_CABLES 2
#define SUPPLIER_EXTCON_NAME "google,supplier-extcon"

struct conn_extcon_mapping {
	int conn_type;
	int id;
};

static const struct conn_extcon_mapping conn_extcon_map[] = {
	{
		.conn_type=CONN_USB_2_0,
		.id=EXTCON_USB
	},
	{
		.conn_type=CONN_CHG,
		.id=EXTCON_CHG_USB_FAST
	},
};

struct extcon_shim {
	struct mutex lock;
	struct device *dev;
	struct extcon_dev *extcon;
	const struct conn_extcon_mapping *conn_extcon_mapping;

	int supported_cables[NUM_SUPPORTED_CABLES];

	/* The "supplier" is the extcon device we receive state updates from.
	 * i.e. the "real" extcon device.
	 */
	struct extcon_dev *supplier;
	struct notifier_block supplier_notifier;
	const struct conn_extcon_mapping *supplier_conn_extcon_mapping;
	struct work_struct supplier_update_work;
	bool supplier_connected;

	/* Always indicate no connection present */
	bool conn_force_disable;

	bool spoof_connected;
};

static void update(struct extcon_shim *shim, bool *state, bool newval)
{
	bool report_connected;
	int ret;

	mutex_lock(&shim->lock);
	if (state)
		*state = newval;

	report_connected = shim->supplier_connected
			&& !shim->conn_force_disable;
	report_connected |= !!shim->spoof_connected;

	dev_info(shim->dev, "Report %s\n",
		report_connected ? "connected" : "disconnected");

	ret = extcon_set_state_sync(shim->extcon, shim->conn_extcon_mapping->id,
				report_connected);

	if (ret != 0)
		dev_err(shim->dev, "Could not set extcon state: %d\n", ret);

	mutex_unlock(&shim->lock);
}

static void supplier_update_work(struct work_struct *work)
{
	struct extcon_shim *shim = container_of(work, struct extcon_shim,
						supplier_update_work);

	bool supplier_connected =
		(extcon_get_state(shim->supplier,
				shim->supplier_conn_extcon_mapping->id) != 0);

	update(shim, &shim->supplier_connected, supplier_connected);
}

static int supplier_change(struct notifier_block *notifier,
			unsigned long supplier_connected, void *unused)
{
	struct extcon_shim *shim = container_of(notifier,
						struct extcon_shim,
						supplier_notifier);
	schedule_work(&shim->supplier_update_work);
	return NOTIFY_DONE;
}

static ssize_t force_disable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct extcon_shim *shim = dev_get_drvdata(dev);
	bool force_disable;
	int ret;

	ret = kstrtobool(buf, &force_disable);
	if (ret < 0) {
		return ret;
	}

	update(shim, &shim->conn_force_disable, force_disable);

	return count;
}

static ssize_t force_disable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct extcon_shim *shim = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", shim->conn_force_disable);
}

static const DEVICE_ATTR_RW(force_disable);

static ssize_t spoof_connected_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct extcon_shim *shim = dev_get_drvdata(dev);
	bool spoof_connected;
	int ret;

	ret = kstrtobool(buf, &spoof_connected);
	if (ret < 0) {
		return ret;
	}

	update(shim, &shim->spoof_connected, spoof_connected);

	return count;
}

static ssize_t spoof_connected_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct extcon_shim *shim = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", shim->spoof_connected);
}

static const DEVICE_ATTR_RW(spoof_connected);

static const struct conn_extcon_mapping *get_mapping(int conn_type)
{
	int x;
	for (x = 0; x < ARRAY_SIZE(conn_extcon_map); x++) {
		if(conn_extcon_map[x].conn_type == conn_type)
			return &conn_extcon_map[x];
	}

	return NULL;
}

/* See extcon_get_edev_by_phandle() in extcon.c for reference */
static struct extcon_dev *get_supplier_extcon(struct device *dev)
{
	struct device_node *node;
	struct extcon_dev *supplier;

	node = of_parse_phandle(dev->of_node, SUPPLIER_EXTCON_NAME, 0);
	if (!node) {
		dev_err(dev, "Could not parse supplier extcon node\n");
		return ERR_PTR(-ENODEV);
	}

	supplier = extcon_find_edev_by_node(node);
	of_node_put(node);

	return supplier;
}

static int supplier_notification_setup(struct extcon_shim *shim)
{
	struct device *dev = shim->dev;
	struct device_node *node = dev->of_node;
	struct extcon_dev *supplier;
	int num_extcon;
	unsigned int supplier_type;
	const struct conn_extcon_mapping *mapping;
	int ret;

	num_extcon = of_count_phandle_with_args(node, SUPPLIER_EXTCON_NAME,
						NULL);
	if (num_extcon < 0) {
		dev_err(dev, "Extcon count failed\n");
		return -ENODEV;
	}

	if (num_extcon != 1) {
		dev_err(dev, "Only one extcon allowed, %d provided\n",
			num_extcon);
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "google,supplier-connection-type",
				&supplier_type);
	if (ret < 0) {
		dev_err(dev, "Must provide a supplier connection type\n");
		return ret;
	}

	mapping = get_mapping(supplier_type);
	if (!mapping) {
		dev_err(dev, "Invalid supplier connection type %u\n",
			supplier_type);
		return -EINVAL;
	}

	supplier = get_supplier_extcon(dev);
	if (IS_ERR(supplier)) {
		return PTR_ERR(supplier);
	}

	INIT_WORK(&shim->supplier_update_work, supplier_update_work);
	shim->supplier = supplier;
	shim->supplier_conn_extcon_mapping = mapping;

	shim->supplier_notifier.notifier_call = supplier_change;
	ret = devm_extcon_register_notifier(shim->dev, supplier, mapping->id,
					&shim->supplier_notifier);
	if (ret < 0)
		dev_err(dev,
			"Failed to register notifier for supplier: %d\n", ret);

	return ret;
}

static int setup_own_extcon(struct extcon_shim *shim)
{
	struct device *dev = shim->dev;
	struct device_node *node = dev->of_node;
	unsigned int conn_type;
	const struct conn_extcon_mapping *mapping;
	int ret = 0;

	ret = of_property_read_u32(node, "google,connection-type", &conn_type);
	if (ret < 0) {
		dev_err(dev, "Must provide a connection type\n");
		return ret;
	}

	mapping = get_mapping(conn_type);
	if (!mapping) {
		dev_err(dev, "Invalid connection type %u\n",
			conn_type);
		return -EINVAL;
	}

	shim->conn_extcon_mapping = mapping;

	shim->supported_cables[0] = shim->conn_extcon_mapping->id;
	/* Last value is sentinel */
	shim->supported_cables[NUM_SUPPORTED_CABLES - 1] = EXTCON_NONE;

	shim->extcon = devm_extcon_dev_allocate(dev, shim->supported_cables);
	if (IS_ERR(shim->extcon)) {
		ret = PTR_ERR(shim->extcon);
		dev_err(dev, "Failed to allocate extcon device: %d\n", ret);
		return ret;
	}

	ret = devm_extcon_dev_register(dev, shim->extcon);
	if (ret < 0) {
		dev_err(dev, "Failed to register extcon device: %d\n", ret);
		return ret;
	}

	return ret;
}

static int setup_sysfs(struct extcon_shim *shim)
{
	struct device *dev = shim->dev;
	int ret;

	ret = device_create_file(shim->dev, &dev_attr_force_disable);
	if (ret < 0) {
		dev_err(dev, "Failed to create %s: %d\n",
			dev_attr_force_disable.attr.name, ret);

		/* Nonfatal */
		ret = 0;
	}

	ret = device_create_file(shim->dev, &dev_attr_spoof_connected);
	if (ret < 0) {
		dev_err(dev, "Failed to create %s: %d\n",
			dev_attr_spoof_connected.attr.name, ret);

		/* Nonfatal */
		ret = 0;
	}

	return ret;
}

static void teardown_sysfs(struct extcon_shim *shim)
{
	device_remove_file(shim->dev, &dev_attr_force_disable);
	device_remove_file(shim->dev, &dev_attr_spoof_connected);
}

static void setup_force_disabled_at_init(struct extcon_shim *shim)
{
	struct device *dev = shim->dev;
	struct device_node *node = dev->of_node;
	bool start_force_disabled;

	start_force_disabled =
		of_property_read_bool(node, "google,force-disabled-at-init");

	shim->conn_force_disable = start_force_disabled;

	dev_info(dev, "Shim start force disabled:%u\n", start_force_disabled);
}

static int extcon_shim_probe(struct platform_device *pdev)
{
	struct extcon_shim *shim;
	int ret = 0;

	shim = devm_kzalloc(&pdev->dev, sizeof(*shim), GFP_KERNEL);
	if (!shim)
		return -ENOMEM;

	platform_set_drvdata(pdev, shim);
	dev_set_drvdata(&pdev->dev, shim);
	shim->dev = &pdev->dev;

	mutex_init(&shim->lock);

	ret = setup_own_extcon(shim);
	if (ret < 0) {
		dev_err(shim->dev, "Own extcon setup failed: %d\n", ret);
		return ret;
	}

	setup_force_disabled_at_init(shim);

	ret = supplier_notification_setup(shim);
	if (ret < 0) {
		if (ret == -EPROBE_DEFER)
			return ret;

		/*
		 * Spoof the connection so that this path turns on to facilitate
		 * debug.
		 */
		dev_err(shim->dev,
			"Supplier notification setup failed: %d\n", ret);
		dev_err(shim->dev, "Extcon will report as connected\n");
		shim->spoof_connected = true;
	}

	ret = setup_sysfs(shim);
	if (ret < 0) {
		dev_err(shim->dev, "Failed to set up sysfs: %d\n", ret);
		dev_err(shim->dev, "Extcon will report as connected\n");
		shim->spoof_connected = true;
	}

	schedule_work(&shim->supplier_update_work);

	return 0;
}

static int extcon_shim_remove(struct platform_device *pdev)
{
	struct extcon_shim *shim = platform_get_drvdata(pdev);

	teardown_sysfs(shim);
	mutex_destroy(&shim->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "google,extcon-shim" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver extcon_shim = {
	.driver	= {
		.name = "extcon-shim",
		.of_match_table	= of_match_ptr(match_table),
		.pm = NULL,
	},
	.probe		= extcon_shim_probe,
	.remove		= extcon_shim_remove,
};
module_platform_driver(extcon_shim);

MODULE_DESCRIPTION("Extcon shim");
MODULE_LICENSE("GPL v2");
