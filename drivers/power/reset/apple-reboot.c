/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple SoC power-off and reboot code
 *
 * Copyright (C) 2020-1 Corellium LLC
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

#define REG_RESET_0			0x00
#define REG_UNKNOWN_0			0x04
#define REG_INTERVAL_0			0x08
#define REG_CTRL_0			0x0c
#define REG_RESET_1			0x10
#define REG_UNKNOWN_1			0x14
#define REG_INTERVAL_1			0x18
#define REG_CTRL_1			0x1c
#define  REG_CTRL_ENABLE_IRQ		1
#define  REG_CTRL_ACK_IRQ		2
#define  REG_CTRL_ENABLE_REBOOT 	4

struct apple_reboot {
	struct device *dev;
	void __iomem *base;

	struct notifier_block restart_nb;
};

static struct apple_reboot *apple_reboot = NULL;

static int apple_reboot_restart(struct notifier_block *this, unsigned long mode, void *cmd)
{
	if(!apple_reboot)
		return NOTIFY_DONE;

	writel(0, apple_reboot->base + REG_RESET_1);
	writel(0, apple_reboot->base + REG_UNKNOWN_1);
	writel(REG_CTRL_ENABLE_REBOOT, apple_reboot->base + REG_CTRL_1);
	writel(0, apple_reboot->base + REG_RESET_1);

	mdelay(500);
	pr_emerg("apple-reboot: Unable to restart!\n");
	return NOTIFY_DONE;
}

static int apple_reboot_probe(struct platform_device *pdev)
{
	struct apple_reboot *ar;
	struct resource *r;
	int err;

	if(apple_reboot)
		return -EEXIST;

	ar = devm_kzalloc(&pdev->dev, sizeof(struct apple_reboot), GFP_KERNEL);
	if(!ar)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ar->base = devm_ioremap_resource(&pdev->dev, r);
	if(IS_ERR(ar->base))
		return PTR_ERR(ar->base);

	ar->restart_nb.notifier_call = apple_reboot_restart;
	ar->restart_nb.priority = 128;
	err = register_restart_handler(&ar->restart_nb);
	if(err) {
		dev_err(&pdev->dev, "Registering restart handler failed: %d.\n", err);
		return err;
	}

	writel(0, ar->base + REG_CTRL_0);
	writel(0, ar->base + REG_CTRL_1);

	apple_reboot = ar;
	return 0;
}

static const struct of_device_id apple_reboot_of_match[] = {
	{ .compatible = "apple,reboot-v0" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_reboot_of_match);

static struct platform_driver apple_reboot_driver = {
	.probe = apple_reboot_probe,
	.driver = {
		.name = "apple-reboot",
		.of_match_table = apple_reboot_of_match,
	},
};
module_platform_driver(apple_reboot_driver);

MODULE_DESCRIPTION("Apple SoC reset driver");
MODULE_LICENSE("GPL v2");
