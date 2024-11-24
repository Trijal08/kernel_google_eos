/*
 * Google bootloader log module
 *
 * Copyright (C) 2023 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fsnotify.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#pragma pack(1)
struct bldr_log_header {
	unsigned int magic;
	unsigned int offset;
	unsigned int rotate_flag;
	unsigned int buf_size;
	char         *buf_start;
};
#pragma pack()

#define BOOT_DEBUG_MAGIC	0xAACCBBDD

struct bldr_log_region {
	phys_addr_t addr;
	size_t size;
};

static struct dentry *bl_debugfs_root;
static struct bldr_log_region cur_bl_log_region;

static int bldr_log_check_header(struct bldr_log_header *header,
				 unsigned long bldr_log_size)
{
	if (header->magic == BOOT_DEBUG_MAGIC) {
		if (header->offset >= sizeof(struct bldr_log_header) &&
			header->offset < sizeof(struct bldr_log_header) +
			bldr_log_size)
			return 0;
	}
	return -EINVAL;
}

static int bldr_log_show(struct seq_file *m, void *v)
{
	char *bldr_base;
	phys_addr_t log_phy_addr = cur_bl_log_region.addr;
	size_t log_size = cur_bl_log_region.size;
	struct bldr_log_header *header;

	bldr_base = memremap(log_phy_addr, log_size, MEMREMAP_WB);
	if (!bldr_base) {
		pr_err("%s: failed to memremap bl log address\n",
			__func__);
		return -ENOMEM;
	}

	header = (struct bldr_log_header *)bldr_base;

	if (bldr_log_check_header(header, log_size)) {
		pr_err("%s: invalid bl log header\n", __func__);
		goto out;
	}

	if (header->rotate_flag) {
		seq_write(m, bldr_base + sizeof(*header) + header->offset,
			header->buf_size - header->offset);
		seq_write(m, bldr_base + sizeof(*header), header->offset - 1);
	} else {
		seq_write(m, bldr_base + sizeof(*header), header->offset - 1);
	}

out:
	memunmap(bldr_base);
	return 0;
}

static int bldr_log_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bl_log");
	if (!res) {
		dev_err(&pdev->dev, "%s: failed to get bl_log resource\n", __func__);
		return -EINVAL;
	}

	if (!resource_size(res)) {
		dev_err(&pdev->dev, "%s: invalid bl_log resource size\n", __func__);
		return -EINVAL;
	}

	cur_bl_log_region.addr = res->start;
	cur_bl_log_region.size = resource_size(res);

	bl_debugfs_root = debugfs_create_dir("bldr_log", 0);
	if (IS_ERR(bl_debugfs_root)) {
		dev_err(&pdev->dev, "%s: failed to create debugs dir\n", __func__);
		return PTR_ERR(bl_debugfs_root);
	}

	debugfs_create_devm_seqfile(&pdev->dev, "cur_bldr_log", bl_debugfs_root, bldr_log_show);
	return 0;
}

static int bldr_log_remove(struct platform_device *pdev)
{
	debugfs_remove_recursive(bl_debugfs_root);
	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "google,bldr_log" },
	{},
};

MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver bldr_log_driver = {
	.driver		= {
		.name	= "google,bldr_log",
		.of_match_table	= dt_match,
	},
	.probe		= bldr_log_probe,
	.remove		= __exit_p(bldr_log_remove),
};

module_platform_driver(bldr_log_driver);

MODULE_DESCRIPTION("Google bootloader log driver");
MODULE_LICENSE("GPL");
