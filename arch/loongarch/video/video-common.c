// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Loongson Technology Corporation Limited
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/vgaarb.h>

#include <asm/video.h>

bool video_is_primary_device(struct device *dev)
{
	struct pci_dev *pdev;

	if (!dev_is_pci(dev))
		return false;

	pdev = to_pci_dev(dev);

	if (!pci_is_display(pdev))
		return false;

	return pdev == vga_default_device();
}
EXPORT_SYMBOL(video_is_primary_device);

MODULE_DESCRIPTION("LoongArch video helpers");
MODULE_LICENSE("GPL");
