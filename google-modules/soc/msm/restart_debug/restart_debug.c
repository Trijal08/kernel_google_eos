// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2022 The Linux Foundation. All rights reserved.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/of_address.h>
#include <linux/kdebug.h>
#include <linux/kallsyms.h>
#include <linux/slab.h>
#include <linux/panic_notifier.h>
#include <linux/ktime.h>

static char *reset_message;
static time64_t *reset_timestamp;
static size_t reset_message_size;
static struct die_args tombstone;
static bool tombstone_written;

static void set_restart_msg(char *msg)
{
	time64_t current_timestamp;
	if (reset_message_size == 0 || !reset_message)
		return;

	pr_err("%s: set restart msg = `%s'\r\n", __func__, msg?:"<null>");
	memset_io(reset_message, 0, reset_message_size);
	memcpy_toio(reset_message, msg, min(strlen(msg), reset_message_size));

	current_timestamp = ktime_get_real_seconds();
	if (reset_timestamp) {
		memcpy_toio(reset_timestamp, &current_timestamp, sizeof(time64_t));
	}
}

static int panic_notify(struct notifier_block *this,
			unsigned long event, void *ptr)
{
	char *kernel_panic_msg;

	if (reset_message_size == 0 || !reset_message)
		goto out;

	kernel_panic_msg = kmalloc(reset_message_size, GFP_KERNEL);

	if (!kernel_panic_msg) {
		pr_err("%s: failed to allocate kernel panic message", __func__);
		return -ENOMEM;
	}

	if (tombstone_written) {
		char pc_symn[KSYM_SYMBOL_LEN] = "<unknown>";
		char lr_symn[KSYM_SYMBOL_LEN] = "<unknown>";
#if defined(CONFIG_ARM)
		sprint_symbol(pc_symn, tombstone.regs->ARM_pc);
		sprint_symbol(lr_symn, tombstone.regs->ARM_lr);
#elif defined(CONFIG_ARM64)
		sprint_symbol(pc_symn, tombstone.regs->pc);
		sprint_symbol(lr_symn, tombstone.regs->regs[30]);
#endif
		scnprintf(kernel_panic_msg, reset_message_size,
			"KP: %s PC:%s LR:%s",
			current->comm, pc_symn, lr_symn);
	} else {
		scnprintf(kernel_panic_msg, reset_message_size,
			"KP: %s", (char *)ptr);
	}

	set_restart_msg(kernel_panic_msg);
	kfree(kernel_panic_msg);

out:
	return NOTIFY_DONE;
}

static int die_notify(struct notifier_block *this,
			unsigned long val, void *data)
{
	memcpy(&tombstone, data, sizeof(struct die_args));
	tombstone_written = 1;

	return NOTIFY_DONE;
}

static struct notifier_block die_nb = {
	.notifier_call  = die_notify,
};

static struct notifier_block panic_nb = {
	.notifier_call = panic_notify,
};

static int restart_debug_probe(struct platform_device *pdev)
{

	struct resource res;
	struct device_node *np;

	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);

	if (!np) {
		dev_err(&pdev->dev, "no such memory-region\n");
		return -ENOENT;
	}

	reset_message = of_iomap(np, 0);
	if (!reset_message) {
		dev_err(&pdev->dev, "failed to iomap reset message");
		return -ENOMEM;
	}

	of_address_to_resource(np, 0, &res);
	reset_message_size = resource_size(&res);
	if (reset_message_size == 0) {
		dev_err(&pdev->dev, "reset message size is 0\n");
		return -EINVAL;
	}

	reset_timestamp = of_iomap(np, 1);
	if (!reset_timestamp) {
		dev_err(&pdev->dev, "failed to iomap reset timestamp");
		return -ENOMEM;
	}

	of_address_to_resource(np, 1, &res);
	if (resource_size(&res) < sizeof(time64_t)) {
		dev_err(&pdev->dev, "reset time memory size is too small\n");
		return -EINVAL;
	}

	set_restart_msg("Unknown");
	tombstone_written = 0;

	register_die_notifier(&die_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_nb);
	return 0;
}

static const struct of_device_id restart_debug_of_match[] = {
	{ .compatible	= "google,debug-reset-msg" },
	{},
};
MODULE_DEVICE_TABLE(of, restart_debug_of_match);

static struct platform_driver restart_debug_driver = {
	.probe = restart_debug_probe,
	.driver = {
		.name = "restart-debug",
		.of_match_table = of_match_ptr(restart_debug_of_match),
	},
};
module_platform_driver(restart_debug_driver);

MODULE_DESCRIPTION("Restart Debug");
MODULE_LICENSE("GPL v2");
