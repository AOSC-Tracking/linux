// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson-2K Board Management Controller (BMC) MFD Core Driver.
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited.
 *
 * Originally written by Chong Qiao <qiaochong@loongson.cn>
 * Rewritten for mainline by Binbin Zhou <zhoubinbin@loongson.cn>
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/platform_device.h>

static struct resource ls2k_display_resources[] = {
	{
		.name	= "ls2kbmc-simplebuf-res",
		.start	= SZ_16M + SZ_2M,
		.end	= SZ_16M + SZ_2M + SZ_4M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ls2k_ipmi0_resources[] = {
	{
		.name	= "ipmi-res0",
		.start	= SZ_16M + 0x00f00000,
		.end	= SZ_16M + 0x00f00000 + 0x1c - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ls2k_ipmi1_resources[] = {
	{
		.name	= "ipmi-res1",
		.start	= SZ_16M + 0x00f00000 + 0x1c,
		.end	= SZ_16M + 0x00f00000 + 0x1c * 2 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ls2k_ipmi2_resources[] = {
	{
		.name	= "ipmi-res2",
		.start	= SZ_16M + 0x00f00000 + 0x1c * 2,
		.end	= SZ_16M + 0x00f00000 + 0x1c * 3 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ls2k_ipmi3_resources[] = {
	{
		.name	= "ipmi-res3",
		.start	= SZ_16M + 0x00f00000 + 0x1c * 3,
		.end	= SZ_16M + 0x00f00000 + 0x1c * 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ls2k_ipmi4_resources[] = {
	{
		.name	= "ipmi-res4",
		.start	= SZ_16M + 0x00f00000 + 0x1c * 4,
		.end	= SZ_16M + 0x00f00000 + 0x1c * 5 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct mfd_cell ls2k_bmc_cells[] = {
	{
		.name = "ls2kbmc-framebuffer",
		.num_resources = ARRAY_SIZE(ls2k_display_resources),
		.resources = ls2k_display_resources,
	},
	{
		.name = "ls2k-ipmi-si",
		.num_resources = ARRAY_SIZE(ls2k_ipmi0_resources),
		.resources = ls2k_ipmi0_resources,
	},
	{
		.name = "ls2k-ipmi-si",
		.num_resources = ARRAY_SIZE(ls2k_ipmi1_resources),
		.resources = ls2k_ipmi1_resources,
	},
	{
		.name = "ls2k-ipmi-si",
		.num_resources = ARRAY_SIZE(ls2k_ipmi2_resources),
		.resources = ls2k_ipmi2_resources,
	},
	{
		.name = "ls2k-ipmi-si",
		.num_resources = ARRAY_SIZE(ls2k_ipmi3_resources),
		.resources = ls2k_ipmi3_resources,
	},
	{
		.name = "ls2k-ipmi-si",
		.num_resources = ARRAY_SIZE(ls2k_ipmi4_resources),
		.resources = ls2k_ipmi4_resources,
	},
};

static int ls2k_bmc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	ls2k_bmc_cells[0].platform_data = &dev;
	ls2k_bmc_cells[0].pdata_size = sizeof(dev);

	return devm_mfd_add_devices(&dev->dev, PLATFORM_DEVID_AUTO,
				    ls2k_bmc_cells, ARRAY_SIZE(ls2k_bmc_cells),
				    &dev->resource[0], 0, NULL);
}

static void ls2k_bmc_remove(struct pci_dev *dev)
{
	pci_disable_device(dev);
}

static struct pci_device_id ls2k_bmc_devices[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, 0x1a05) },
	{ }
};
MODULE_DEVICE_TABLE(pci, ls2k_bmc_devices);

static struct pci_driver ls2k_bmc_driver = {
	.name = "ls2k-bmc",
	.id_table = ls2k_bmc_devices,
	.probe = ls2k_bmc_probe,
	.remove = ls2k_bmc_remove,
};

module_pci_driver(ls2k_bmc_driver);

MODULE_DESCRIPTION("Loongson-2K BMC driver");
MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_LICENSE("GPL");
