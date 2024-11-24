// SPDX-License-Identifier: GPL-2.0
/*
 * debug-snapshot-debug-kinfo.c - use DPM to enable/disable kernel information backup
 *
 * Copyright 2020 Google LLC
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <generated/utsrelease.h>
#include "debug_snapshot_debug_kinfo.h"


static int debug_kinfo_rmem_setup(struct reserved_mem *rmem)
{
	/* Create mappings for reserved memory */
	int i;
	phys_addr_t base, size;
	unsigned int num_pages;
	unsigned long flags = VM_NO_GUARD | VM_MAP;
	pgprot_t prot = __pgprot(PROT_NORMAL_NC);
	struct page **pages;
	void *vaddr;


	base = rmem->base;
	size = rmem->size;

	if (!base || !size) {
		pr_err("debug_kinfo_mem_setup: unexpected reserved memory\n");
		return 0;
	}

	num_pages = (size >> PAGE_SHIFT);
	if (size > (num_pages << PAGE_SHIFT))
		num_pages++;

	pages = kcalloc(num_pages, sizeof(struct page *), GFP_KERNEL);
	for (i = 0; i < num_pages; i++) {
		pages[i] = phys_to_page(base);
		base += PAGE_SIZE;
	}

	vaddr = vmap(pages, num_pages, flags, prot);
	kfree(pages);
	if (!vaddr) {
		pr_err("debug_kinfo_mem_setup: paddr:%pK page_size:0x%x failed to vmap\n",
				(void *)rmem->base, (unsigned int)rmem->size);
		return 0;
	}

	rmem->priv = vaddr;

	return 0;
}

static void update_vendor_kernel_all_info(struct vendor_kernel_all_info *all_info)
{
	/* Write magic number and checksum value for vendor kernel info */
	int index;
	struct vendor_kernel_info *info;
	unsigned int *vendor_checksum_info;

	all_info->magic_number = DEBUG_KINFO_MAGIC;
	all_info->combined_checksum = 0;

	info = &all_info->info;
	vendor_checksum_info = (unsigned int *)info;
	for (index = 0; index < sizeof(*info) / sizeof(unsigned int); index++)
		all_info->combined_checksum ^= vendor_checksum_info[index];
}

static void debug_kinfo_write_vendor_kernel_info(struct vendor_kernel_all_info *all_info)
{
	struct vendor_kernel_info *info;
	memset(all_info, 0, sizeof(*all_info));
	info = &all_info->info;
	strscpy(info->uts_release, UTS_RELEASE, sizeof(info->uts_release));
	update_vendor_kernel_all_info(all_info);
}

static int debug_snapshot_debug_kinfo_probe(struct platform_device *pdev)
{

	struct reserved_mem *rmem;
	struct device_node *mem_region;
	void *addr;

	mem_region = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!mem_region) {
		dev_err(&pdev->dev, "no such memory-region\n");
		return 0;
	}

	rmem = of_reserved_mem_lookup(mem_region);
	if (!rmem) {
		dev_err(&pdev->dev, "no such reserved mem of node name %s\n",
				pdev->dev.of_node->name);
		return 0;
	}
	debug_kinfo_rmem_setup(rmem);
	addr = rmem->priv + DEBUG_KINFO_VENDOR_OFFSET;
	debug_kinfo_write_vendor_kernel_info((struct vendor_kernel_all_info *)addr);

	return 0;
}

static const struct of_device_id debug_snapshot_debug_kinfo_of_match[] = {
	{ .compatible	= "google,debug-snapshot-debug-kinfo" },
	{},
};
MODULE_DEVICE_TABLE(of, debug_snapshot_debug_kinfo_of_match);

static struct platform_driver debug_snapshot_debug_kinfo_driver = {
	.probe = debug_snapshot_debug_kinfo_probe,
	.driver = {
		.name = "debug-snapshot-debug-kinfo",
		.of_match_table = of_match_ptr(debug_snapshot_debug_kinfo_of_match),
	},
};
module_platform_driver(debug_snapshot_debug_kinfo_driver);

MODULE_AUTHOR("Jone Chou <jonechou@google.com>");
MODULE_DESCRIPTION("Debug Snapshot Debug Kinfo");
MODULE_LICENSE("GPL v2");