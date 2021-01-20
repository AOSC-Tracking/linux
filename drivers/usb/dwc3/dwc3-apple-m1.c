// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-apple-m1.c - OF glue layer for Apple M1 SoC
 *
 * Copyright (c) 2021 Corellium LLC
 *
 * Based on dwc3-keystone.c:
 *  Copyright (c) 2015 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Felipe Balbi <balbi@ti.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>

/* for registers not normally configured by Linux */
#include "core.h"

struct dwc3_apple {
	struct device		*dev;
	struct clk_bulk_data	*clks;
	int			num_clocks;
	void __iomem		*atcphy;
	void __iomem		*usbcore;
};

static inline void dwc3_apple_m1_rmwl(u32 clr, u32 set, void __iomem *io)
{
	writel((readl(io) & ~clr) | set, io);
}

static int dwc3_apple_m1_poll(u32 msk, u32 val, int neq, void __iomem *io, struct dwc3_apple *da,
			      const char *name)
{
	int cnt = 100;
	u32 rdv;
	while(cnt --) {
		rdv = readl(io);
		if(((rdv & msk) == val) ^ neq)
			return 0;
		udelay(100);
	}

	pr_err("%pOFn: %s: [%s] timed out (0x%x).\n", da->dev->of_node, __func__, name, rdv);
	return -ETIMEDOUT;
}

/* this plays back tunables passed from bootloader. those are somewhat hardware specific,
   and ultimately provided by the first-stage platform bootloader */
static int dwc3_apple_m1_tunable(struct dwc3_apple *da, const char *name)
{
	struct device_node *np = da->dev->of_node;
	int cnt, idx;
	u32 addr, mask, val, range;

	cnt = of_property_count_elems_of_size(np, name, sizeof(u32));
	if(cnt < 0) {
		pr_warn("%pOFn: %s: missing tunable [%s].\n", np, __func__, name);
		return 0;
	}
	if(cnt % 3) {
		pr_warn("%pOFn: %s: tunable [%s] is not made of <addr, mask, val> triplets.\n", np, __func__, name);
		return -EINVAL;
	}

	for(idx=0; idx<cnt; idx+=3) {
		if(of_property_read_u32_index(np, name, idx, &addr) < 0 ||
		   of_property_read_u32_index(np, name, idx + 1, &mask) < 0 ||
		   of_property_read_u32_index(np, name, idx + 2, &val)) {
			pr_warn("%pOFn: %s: tunable [%s] not readable.\n", np, __func__, name);
			return -EINVAL;
		}
		range = addr >> 28;
		addr &= 0x0FFFFFFF;

		switch(range) {
		case 0:
			dwc3_apple_m1_rmwl(mask, val, da->atcphy + addr);
			break;
		case 1:
			dwc3_apple_m1_rmwl(mask, val, da->usbcore + addr);
			break;
		default:
			pr_warn("%pOFn: %s: tunable [%s] refers to nonexistent range %d.\n", np, __func__, name, range);
		}
	}

	return 0;
}

#define USBCORE_PIPEPHY_STATUS				0x200020
#define   USBCORE_PIPEPHY_STATUS_READY			(1 << 30)
#define USBCORE_FORCE_CLK_ON				0x2000F0
#define USBCORE_DWC3					0x280000
#define USBCORE_AUSBEVT_USB2CTL				0x800000
#define   USBCORE_AUSBEVT_USB2CTL_EVT_EN		(1 << 0)
#define   USBCORE_AUSBEVT_USB2CTL_LOAD_CNT		(1 << 3)
#define USBCORE_AUSBEVT_UTMIACT_EVTCNT			0x800020
#define USBCORE_PIPEHDLR_MUXSEL				0xA8400C
#define   USBCORE_PIPEHDLR_MUXSEL_MODE_MASK		(3 << 0)
#define     USBCORE_PIPEHDLR_MUXSEL_MODE_USB2		(2 << 0)
#define   USBCORE_PIPEHDLR_MUXSEL_CLKEN_MASK		(3 << 3)
#define USBCORE_PIPEHDLR_PIPE_IF_REQ			0xA84010
#define USBCORE_PIPEHDLR_PIPE_IF_ACK			0xA84014
#define USBCORE_PIPEHDLR_AON_GEN			0xA8401C
#define   USBCORE_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN	(1 << 4)
#define   USBCORE_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET	(1 << 0)
#define USBCORE_PIPEHDLR_NONSEL_OVRD			0xA84020
#define   USBCORE_PIPEHDLR_NONSEL_OVRD_DUMMY_PHY_READY	(1 << 15)
#define USBCORE_USB2PHY_USBCTL				0xA90000
#define   USBCORE_USB2PHY_USBCTL_MODE_MASK		(7 << 0)
#define     USBCORE_USB2PHY_USBCTL_MODE_USB2		(2 << 0)
#define USBCORE_USB2PHY_CTL				0xA90004
#define   USBCORE_USB2PHY_CTL_RESET			(1 << 0)
#define   USBCORE_USB2PHY_CTL_PORT_RESET		(1 << 1)
#define   USBCORE_USB2PHY_CTL_APB_RESETN		(1 << 2)
#define   USBCORE_USB2PHY_CTL_SIDDQ			(1 << 3)
#define USBCORE_USB2PHY_SIG				0xA90008
#define   USBCORE_USB2PHY_SIG_VBUSDET_FORCE_VAL		(1 << 0)
#define   USBCORE_USB2PHY_SIG_VBUSDET_FORCE_EN		(1 << 1)
#define   USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL	(1 << 2)
#define   USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN	(1 << 3)
#define   USBCORE_USB2PHY_SIG_MODE_HOST			(7 << 12)
#define USBCORE_USB2PHY_MISCTUNE			0xA9001C
#define   USBCORE_USB2PHY_MISCTUNE_APBCLK_GATE_OFF	(1 << 29)
#define   USBCORE_USB2PHY_MISCTUNE_REFCLK_GATE_OFF	(1 << 30)

#define DWC3_GUSB2PHYCFG0_SUSPENDUSB20			(1 << 6)
#define DWC3_GUSB3PIPECTL0_SUSPENDENABLE		(1 << 17)

static int dwc3_apple_m1_start(struct dwc3_apple *da)
{
	int ret;

	ret = dwc3_apple_m1_tunable(da, "tunable-ATC0AXI2AF");
	if(ret < 0)
		return ret;

	dwc3_apple_m1_rmwl(0, USBCORE_USB2PHY_SIG_MODE_HOST,
		da->usbcore + USBCORE_USB2PHY_SIG);

	/* configure VBUS detection */
	dwc3_apple_m1_rmwl(0, USBCORE_USB2PHY_SIG_VBUSDET_FORCE_VAL,
		da->usbcore + USBCORE_USB2PHY_SIG);
	dwc3_apple_m1_rmwl(0, USBCORE_USB2PHY_SIG_VBUSDET_FORCE_EN,
		da->usbcore + USBCORE_USB2PHY_SIG);
	dwc3_apple_m1_rmwl(0, USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL,
		da->usbcore + USBCORE_USB2PHY_SIG);
	dwc3_apple_m1_rmwl(0, USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN,
		da->usbcore + USBCORE_USB2PHY_SIG);

	/* power on the PHY and take it out of reset */
	dwc3_apple_m1_rmwl(USBCORE_USB2PHY_CTL_SIDDQ, 0, da->usbcore + USBCORE_USB2PHY_CTL);
	udelay(10);
	dwc3_apple_m1_rmwl(USBCORE_USB2PHY_CTL_RESET, 0, da->usbcore + USBCORE_USB2PHY_CTL);
	dwc3_apple_m1_rmwl(USBCORE_USB2PHY_CTL_PORT_RESET, 0, da->usbcore + USBCORE_USB2PHY_CTL);

	dwc3_apple_m1_rmwl(0, USBCORE_AUSBEVT_USB2CTL_LOAD_CNT,
		da->usbcore + USBCORE_AUSBEVT_USB2CTL);
	dwc3_apple_m1_rmwl(0, USBCORE_AUSBEVT_USB2CTL_LOAD_CNT,
		da->usbcore + USBCORE_AUSBEVT_USB2CTL);

	dwc3_apple_m1_rmwl(0, USBCORE_USB2PHY_CTL_APB_RESETN, da->usbcore + USBCORE_USB2PHY_CTL);

	/* enable PHY clocks */
	dwc3_apple_m1_rmwl(USBCORE_USB2PHY_MISCTUNE_APBCLK_GATE_OFF, 0,
		da->usbcore + USBCORE_USB2PHY_MISCTUNE);
	dwc3_apple_m1_rmwl(USBCORE_USB2PHY_MISCTUNE_REFCLK_GATE_OFF, 0,
		da->usbcore + USBCORE_USB2PHY_MISCTUNE);
	udelay(30);
	ret = dwc3_apple_m1_poll(-1u, 0, 1, da->usbcore + USBCORE_AUSBEVT_UTMIACT_EVTCNT, da,
		"UTMI clock active (1)");
	if(ret < 0)
		return ret;

	/* set host mode on PHY */
	dwc3_apple_m1_rmwl(USBCORE_USB2PHY_USBCTL_MODE_MASK, USBCORE_USB2PHY_USBCTL_MODE_USB2,
		da->usbcore + USBCORE_USB2PHY_USBCTL);

	/* power on the USB DRD core */
	dwc3_apple_m1_rmwl(USBCORE_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN, 0,
		da->usbcore + USBCORE_PIPEHDLR_AON_GEN);
	dwc3_apple_m1_rmwl(0, USBCORE_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET,
		da->usbcore + USBCORE_PIPEHDLR_AON_GEN);

	/* set core mode to host */
	dwc3_apple_m1_rmwl(DWC3_GCTL_PRTCAPDIR(3), DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_HOST),
		da->usbcore + USBCORE_DWC3 + DWC3_GCTL);

	/* wait for fast PHY ready */
	if((readl(da->usbcore + USBCORE_PIPEHDLR_MUXSEL) & USBCORE_PIPEHDLR_MUXSEL_MODE_MASK)
	   != USBCORE_PIPEHDLR_MUXSEL_MODE_USB2) {
		ret = dwc3_apple_m1_poll(USBCORE_PIPEPHY_STATUS_READY, USBCORE_PIPEPHY_STATUS_READY,
			0, da->usbcore + USBCORE_PIPEPHY_STATUS, da, "PIPE PHY ready");
		if(ret < 0)
			return ret;
	}
	ret = dwc3_apple_m1_poll(-1u, 0, 1, da->usbcore + USBCORE_AUSBEVT_UTMIACT_EVTCNT, da,
		"UTMI clock active (2)");
	if(ret < 0)
		return ret;

	ret = dwc3_apple_m1_tunable(da, "tunable");
	if(ret < 0)
		return ret;

	/* set up power and clock parameters in DRD core */
	dwc3_apple_m1_rmwl(DWC3_GCTL_PWRDNSCALE(0x1FFF), DWC3_GCTL_PWRDNSCALE(13),
		da->usbcore + USBCORE_DWC3 + DWC3_GCTL);
	dwc3_apple_m1_rmwl(0, DWC3_GCTL_GBLHIBERNATIONEN, da->usbcore + USBCORE_DWC3 + DWC3_GCTL);

	/* unsuspend the PHYs */
	dwc3_apple_m1_rmwl(DWC3_GUSB2PHYCFG0_SUSPENDUSB20, 0,
		da->usbcore + USBCORE_DWC3 + DWC3_GUSB2PHYCFG(0));
	dwc3_apple_m1_rmwl(DWC3_GUSB3PIPECTL0_SUSPENDENABLE, 0,
		da->usbcore + USBCORE_DWC3 + DWC3_GUSB3PIPECTL(0));

	return 0;
}

static int dwc3_apple_m1_probe(struct platform_device *pdev)
{
	struct dwc3_apple	*da;
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;

	int			ret;

	da = devm_kzalloc(dev, sizeof(*da), GFP_KERNEL);
	if (!da)
		return -ENOMEM;

	platform_set_drvdata(pdev, da);
	da->dev = dev;

	da->atcphy = of_iomap(np, 0);
	da->usbcore = of_iomap(np, 1);
	if (!da->atcphy || !da->usbcore) {
		pr_err("%pOFn: %s: failed to map MMIO ranges.", np, __func__);
		return -EINVAL;
	}

	ret = clk_bulk_get_all(da->dev, &da->clks);
	if (ret < 0) {
		pr_err("%pOFn: %s: clk_bulk_get_all failed.", np, __func__);
		return ret;
	}

	da->num_clocks = ret;
	ret = clk_bulk_prepare_enable(da->num_clocks, da->clks);
	if (ret) {
		pr_err("%pOFn: %s: clk_bulk_prepare_enable failed.", np, __func__);
		return ret;
	}

	ret = dwc3_apple_m1_start(da);
	if(ret) {
		pr_err("%pOFn: %s: failed to prepare hardware: %d.", np, __func__, ret);
		return ret;
	}

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		goto err_clk_put;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err_clk_put:
	clk_bulk_disable_unprepare(da->num_clocks, da->clks);
	clk_bulk_put_all(da->num_clocks, da->clks);

	return ret;
}

static int dwc3_apple_m1_remove(struct platform_device *pdev)
{
	struct dwc3_apple	*da = platform_get_drvdata(pdev);
	struct device		*dev = &pdev->dev;

	of_platform_depopulate(dev);

	clk_bulk_disable_unprepare(da->num_clocks, da->clks);
	clk_bulk_put_all(da->num_clocks, da->clks);
	da->num_clocks = 0;

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_set_suspended(dev);

	return 0;
}

static int __maybe_unused dwc3_apple_m1_runtime_suspend(struct device *dev)
{
	struct dwc3_apple	*da = dev_get_drvdata(dev);

	clk_bulk_disable(da->num_clocks, da->clks);

	return 0;
}

static int __maybe_unused dwc3_apple_m1_runtime_resume(struct device *dev)
{
	struct dwc3_apple	*da = dev_get_drvdata(dev);

	return clk_bulk_enable(da->num_clocks, da->clks);
}

static int __maybe_unused dwc3_apple_m1_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused dwc3_apple_m1_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops dwc3_apple_m1_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_apple_m1_suspend, dwc3_apple_m1_resume)
	SET_RUNTIME_PM_OPS(dwc3_apple_m1_runtime_suspend, dwc3_apple_m1_runtime_resume, NULL)
};

static const struct of_device_id of_dwc3_apple_m1_match[] = {
	{ .compatible = "apple,dwc3-m1" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_apple_m1_match);

static struct platform_driver dwc3_apple_m1_driver = {
	.probe		= dwc3_apple_m1_probe,
	.remove		= dwc3_apple_m1_remove,
	.driver		= {
		.name	= "dwc3-of-apple-m1",
		.of_match_table = of_dwc3_apple_m1_match,
		.pm	= &dwc3_apple_m1_dev_pm_ops,
	},
};

module_platform_driver(dwc3_apple_m1_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Apple M1 Glue Layer");
MODULE_AUTHOR("Corellium LLC");
