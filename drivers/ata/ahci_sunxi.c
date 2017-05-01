/*
 * Allwinner sunxi AHCI SATA platform driver
 * Copyright 2013 Olliver Schinagl <oliver@schinagl.nl>
 * Copyright 2014 Hans de Goede <hdegoede@redhat.com>
 *
 * based on the AHCI SATA platform driver by Jeff Garzik and Anton Vorontsov
 * Based on code from Allwinner Technology Co., Ltd. <www.allwinnertech.com>,
 * Daniel Wang <danielwang@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/ahci_platform.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include "ahci.h"

#define DRV_NAME "ahci-sunxi"

/* Insmod parameters */
static bool enable_pmp;
module_param(enable_pmp, bool, 0);
MODULE_PARM_DESC(enable_pmp,
	"Enable support for sata port multipliers, only use if you use a pmp!");

#define AHCI_BISTAFR	0x00a0
#define AHCI_BISTCR	0x00a4
#define AHCI_BISTFCTR	0x00a8
#define AHCI_BISTSR	0x00ac
#define AHCI_BISTDECR	0x00b0
#define AHCI_DIAGNR0	0x00b4
#define AHCI_DIAGNR1	0x00b8
#define AHCI_OOBR	0x00bc
#define AHCI_PHYCS0R	0x00c0
#define AHCI_PHYCS1R	0x00c4
#define AHCI_PHYCS2R	0x00c8
#define AHCI_TIMER1MS	0x00e0
#define AHCI_GPARAM1R	0x00e8
#define AHCI_GPARAM2R	0x00ec
#define AHCI_PPARAMR	0x00f0
#define AHCI_TESTR	0x00f4
#define AHCI_VERSIONR	0x00f8
#define AHCI_IDR	0x00fc
#define AHCI_RWCR	0x00fc
#define AHCI_P0DMACR	0x0170
#define AHCI_P0PHYCR	0x0178
#define AHCI_P0PHYSR	0x017c

struct ahci_sunxi_quirks {
	bool has_reset;
	bool has_vdd1v2;
	bool has_vdd2v5;
};

struct ahci_sunxi_data {
	const struct ahci_sunxi_quirks *quirks;
	struct reset_control *reset;
	struct regulator *vdd1v2;
	struct regulator *vdd2v5;
};

static void sunxi_clrbits(void __iomem *reg, u32 clr_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val &= ~(clr_val);
	writel(reg_val, reg);
}

static void sunxi_setbits(void __iomem *reg, u32 set_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val |= set_val;
	writel(reg_val, reg);
}

static void sunxi_clrsetbits(void __iomem *reg, u32 clr_val, u32 set_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val &= ~(clr_val);
	reg_val |= set_val;
	writel(reg_val, reg);
}

static u32 sunxi_getbits(void __iomem *reg, u8 mask, u8 shift)
{
	return (readl(reg) >> shift) & mask;
}

static int ahci_sunxi_phy_init(struct device *dev, void __iomem *reg_base)
{
	u32 reg_val;
	int timeout;

	/* This magic is from the original code */
	writel(0, reg_base + AHCI_RWCR);
	msleep(5);

	sunxi_setbits(reg_base + AHCI_PHYCS1R, BIT(19));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS0R,
			 (0x7 << 24),
			 (0x5 << 24) | BIT(23) | BIT(18));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS1R,
			 (0x3 << 16) | (0x1f << 8) | (0x3 << 6),
			 (0x2 << 16) | (0x6 << 8) | (0x2 << 6));
	sunxi_setbits(reg_base + AHCI_PHYCS1R, BIT(28) | BIT(15));
	sunxi_clrbits(reg_base + AHCI_PHYCS1R, BIT(19));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS0R,
			 (0x7 << 20), (0x3 << 20));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS2R,
			 (0x1f << 5), (0x19 << 5));
	msleep(5);

	sunxi_setbits(reg_base + AHCI_PHYCS0R, (0x1 << 19));

	timeout = 250; /* Power up takes aprox 50 us */
	do {
		reg_val = sunxi_getbits(reg_base + AHCI_PHYCS0R, 0x7, 28);
		if (reg_val == 0x02)
			break;

		if (--timeout == 0) {
			dev_err(dev, "PHY power up failed.\n");
			return -EIO;
		}
		udelay(1);
	} while (1);

	sunxi_setbits(reg_base + AHCI_PHYCS2R, (0x1 << 24));

	timeout = 100; /* Calibration takes aprox 10 us */
	do {
		reg_val = sunxi_getbits(reg_base + AHCI_PHYCS2R, 0x1, 24);
		if (reg_val == 0x00)
			break;

		if (--timeout == 0) {
			dev_err(dev, "PHY calibration failed.\n");
			return -EIO;
		}
		udelay(1);
	} while (1);

	msleep(15);

	writel(0x7, reg_base + AHCI_RWCR);

	return 0;
}

static void ahci_sunxi_start_engine(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_host_priv *hpriv = ap->host->private_data;

	/* Setup DMA before DMA start */
	sunxi_clrsetbits(hpriv->mmio + AHCI_P0DMACR, 0x0000ff00, 0x00004400);

	/* Start DMA */
	sunxi_setbits(port_mmio + PORT_CMD, PORT_CMD_START);
}

static const struct ata_port_info ahci_sunxi_port_info = {
	.flags		= AHCI_FLAG_COMMON | ATA_FLAG_NCQ,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static struct scsi_host_template ahci_platform_sht = {
	AHCI_SHT(DRV_NAME),
};

static int ahci_sunxi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ahci_host_priv *hpriv;
	struct ahci_sunxi_data *data;
	int rc;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->quirks = of_device_get_match_data(dev);
	if (!data->quirks)
		return -EINVAL;

	if (data->quirks->has_reset) {
		data->reset = devm_reset_control_get(dev, NULL);
		if (IS_ERR(data->reset)) {
			dev_err(dev, "Failed to get reset\n");
			return PTR_ERR(data->reset);
		}
	}

	if (data->quirks->has_vdd1v2) {
		data->vdd1v2 = devm_regulator_get(dev, "vdd1v2");
		if (IS_ERR(data->vdd1v2)) {
			dev_err(dev, "Failed to get 1.2v VDD regulator\n");
			return PTR_ERR(data->vdd1v2);
		}
	}

	if (data->quirks->has_vdd2v5) {
		data->vdd2v5 = devm_regulator_get(dev, "vdd2v5");
		if (IS_ERR(data->vdd2v5)) {
			dev_err(dev, "Failed to get 2.5v VDD regulator\n");
			return PTR_ERR(data->vdd2v5);
		}
	}

	hpriv = ahci_platform_get_resources(pdev, 0);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	hpriv->plat_data = data;
	hpriv->start_engine = ahci_sunxi_start_engine;

	if (data->quirks->has_vdd1v2) {
		rc = regulator_enable(data->vdd1v2);
		if (rc)
			return rc;
	}

	if (data->quirks->has_vdd2v5) {
		rc = regulator_enable(data->vdd2v5);
		if (rc)
			goto disable_vdd1v2;
	}

	if (data->quirks->has_reset) {
		rc = reset_control_deassert(data->reset);
		if (rc)
			goto disable_vdd2v5;
	}

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		goto assert_reset;

	rc = ahci_sunxi_phy_init(dev, hpriv->mmio);
	if (rc)
		goto disable_resources;

	hpriv->flags = AHCI_HFLAG_32BIT_ONLY | AHCI_HFLAG_NO_MSI |
		       AHCI_HFLAG_YES_NCQ;

	/*
	 * The sunxi sata controller seems to be unable to successfully do a
	 * soft reset if no pmp is attached, so disable pmp use unless
	 * requested, otherwise directly attached disks do not work.
	 */
	if (!enable_pmp)
		hpriv->flags |= AHCI_HFLAG_NO_PMP;

	rc = ahci_platform_init_host(pdev, hpriv, &ahci_sunxi_port_info,
				     &ahci_platform_sht);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
assert_reset:
	if (data->quirks->has_reset)
		reset_control_assert(data->reset);
disable_vdd2v5:
	if (data->quirks->has_vdd2v5)
		regulator_disable(data->vdd2v5);
disable_vdd1v2:
	if (data->quirks->has_vdd1v2)
		regulator_disable(data->vdd1v2);
	return rc;
}

static int ahci_sunxi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct ahci_sunxi_data *data = hpriv->plat_data;
	int rc;

	rc = ata_platform_remove_one(pdev);

	if (data->quirks->has_reset)
		reset_control_assert(data->reset);
	if (data->quirks->has_vdd2v5)
		regulator_disable(data->vdd2v5);
	if (data->quirks->has_vdd1v2)
		regulator_disable(data->vdd1v2);

	return rc;
}

#ifdef CONFIG_PM_SLEEP
static int ahci_sunxi_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	int rc;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	rc = ahci_sunxi_phy_init(dev, hpriv->mmio);
	if (rc)
		goto disable_resources;

	rc = ahci_platform_resume_host(dev);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}
#endif

static SIMPLE_DEV_PM_OPS(ahci_sunxi_pm_ops, ahci_platform_suspend,
			 ahci_sunxi_resume);

static const struct ahci_sunxi_quirks sun4i_a10_ahci_quirks = {
	.has_reset = false,
};

static const struct ahci_sunxi_quirks sun8i_r40_ahci_quirks = {
	.has_reset = true,
	.has_vdd1v2 = true,
	.has_vdd2v5 = true,
};

static const struct of_device_id ahci_sunxi_of_match[] = {
	{
		.compatible = "allwinner,sun4i-a10-ahci",
		.data = &sun4i_a10_ahci_quirks,
	},
	{
		.compatible = "allwinner,sun8i-r40-ahci",
		.data = &sun8i_r40_ahci_quirks,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ahci_sunxi_of_match);

static struct platform_driver ahci_sunxi_driver = {
	.probe = ahci_sunxi_probe,
	.remove = ahci_sunxi_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ahci_sunxi_of_match,
		.pm = &ahci_sunxi_pm_ops,
	},
};
module_platform_driver(ahci_sunxi_driver);

MODULE_DESCRIPTION("Allwinner sunxi AHCI SATA driver");
MODULE_AUTHOR("Olliver Schinagl <oliver@schinagl.nl>");
MODULE_LICENSE("GPL");
