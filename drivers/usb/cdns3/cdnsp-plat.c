// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence USBSSP DRD Driver - Platform Device Support
 *
 * Based on cdns3-plat.c
 *
 * Copyright (C) 2018-2020 Cadence.
 * Copyright (C) 2017-2018 NXP
 * Copyright (C) 2019 Texas Instruments
 *
 * Author: Peter Chen <peter.chen@nxp.com>
 *         Pawel Laszczak <pawell@cadence.com>
 *         Roger Quadros <rogerq@ti.com>
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>

#include "core.h"
#include "gadget-export.h"
#include "drd.h"

/*
 * Resolve a PHY reference from an ACPI _DSD property.
 *
 * Under DT, devm_phy_optional_get() resolves PHY phandles directly.
 * Under ACPI, _DSD device references (e.g. "cdnsp,usb3-phy" → ^^U3P4.USB0)
 * are not handled by the PHY framework.  Resolve the reference manually:
 * find the referenced fwnode, get its parent device (the PHY provider),
 * and look up the PHY via the provider's lookup table.
 *
 * Based on the vendor kernel's devm_phy_optional_ref_get().
 */
static void cdnsp_phy_release(struct device *dev, void *res)
{
	struct phy *phy = *(struct phy **)res;

	phy_put(dev, phy);
}

static struct phy *cdnsp_acpi_phy_ref_get(struct device *dev, const char *name)
{
	struct fwnode_handle *ref_fwnode, *parent_fwnode;
	const char *phy_con_id;
	struct device *phy_dev;
	struct phy **ptr, *phy;

	ref_fwnode = fwnode_find_reference(dev_fwnode(dev), name, 0);
	if (IS_ERR_OR_NULL(ref_fwnode))
		return NULL;

	phy_con_id = fwnode_get_name(ref_fwnode);

	parent_fwnode = fwnode_get_parent(ref_fwnode);
	fwnode_handle_put(ref_fwnode);
	if (!parent_fwnode)
		return NULL;

	phy_dev = get_dev_from_fwnode(parent_fwnode);
	fwnode_handle_put(parent_fwnode);
	if (!phy_dev)
		return NULL;

	ptr = devres_alloc(cdnsp_phy_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr) {
		put_device(phy_dev);
		return ERR_PTR(-ENOMEM);
	}

	phy = phy_get(phy_dev, phy_con_id);
	put_device(phy_dev);

	if (IS_ERR_OR_NULL(phy)) {
		devres_free(ptr);
		return NULL;
	}

	*ptr = phy;
	devres_add(dev, ptr);
	return phy;
}

static int set_phy_power_on(struct cdns *cdns)
{
	int ret;

	ret = phy_power_on(cdns->usb2_phy);
	if (ret)
		return ret;

	ret = phy_power_on(cdns->usb3_phy);
	if (ret)
		phy_power_off(cdns->usb2_phy);

	return ret;
}

static void set_phy_power_off(struct cdns *cdns)
{
	phy_power_off(cdns->usb3_phy);
	phy_power_off(cdns->usb2_phy);
}

/**
 * cdnsp_plat_probe - probe for cdnsp core device
 * @pdev: Pointer to cdnsp core platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdnsp_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource	*res;
	struct cdns *cdns;
	void __iomem *regs;
	int ret;

	cdns = devm_kzalloc(dev, sizeof(*cdns), GFP_KERNEL);
	if (!cdns)
		return -ENOMEM;

	cdns->dev = dev;
	cdns->pdata = dev_get_platdata(dev);

	platform_set_drvdata(pdev, cdns);

	ret = platform_get_irq_byname(pdev, "host");
	if (ret < 0)
		return ret;

	cdns->xhci_res[0].start = ret;
	cdns->xhci_res[0].end = ret;
	cdns->xhci_res[0].flags = IORESOURCE_IRQ | irq_get_trigger_type(ret);
	cdns->xhci_res[0].name = "host";

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "xhci");
	if (!res) {
		dev_err(dev, "couldn't get xhci resource\n");
		return -ENXIO;
	}

	cdns->xhci_res[1] = *res;

	cdns->dev_irq = platform_get_irq_byname(pdev, "peripheral");

	if (cdns->dev_irq < 0)
		return dev_err_probe(dev, cdns->dev_irq,
				     "Failed to get peripheral IRQ\n");

	regs = devm_platform_ioremap_resource_byname(pdev, "dev");
	if (IS_ERR(regs))
		return dev_err_probe(dev, PTR_ERR(regs),
				     "Failed to get dev base\n");

	cdns->dev_regs	= regs;

	cdns->otg_irq = platform_get_irq_byname(pdev, "otg");
	if (cdns->otg_irq < 0)
		return dev_err_probe(dev, cdns->otg_irq,
				     "Failed to get otg IRQ\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "otg");
	if (!res) {
		dev_err(dev, "couldn't get otg resource\n");
		return -ENXIO;
	}

	cdns->phyrst_a_enable = device_property_read_bool(dev, "cdns,phyrst-a-enable");

	cdns->otg_res = *res;

	cdns->wakeup_irq = platform_get_irq_byname_optional(pdev, "wakeup");
	if (cdns->wakeup_irq == -EPROBE_DEFER)
		return cdns->wakeup_irq;

	if (cdns->wakeup_irq < 0) {
		dev_dbg(dev, "couldn't get wakeup irq\n");
		cdns->wakeup_irq = 0x0;
	}

	/* CDNSP uses different PHY names than CDNS3 */
	cdns->usb2_phy = devm_phy_optional_get(dev, "cdnsp,usb2-phy");
	if (IS_ERR(cdns->usb2_phy))
		return dev_err_probe(dev, PTR_ERR(cdns->usb2_phy),
				     "Failed to get cdnsp,usb2-phy\n");

	/* ACPI: _DSD device references aren't resolved by the PHY framework */
	if (!cdns->usb2_phy) {
		cdns->usb2_phy = cdnsp_acpi_phy_ref_get(dev, "cdnsp,usb2-phy");
		if (IS_ERR(cdns->usb2_phy))
			return PTR_ERR(cdns->usb2_phy);
	}

	ret = phy_init(cdns->usb2_phy);
	if (ret)
		return ret;

	cdns->usb3_phy = devm_phy_optional_get(dev, "cdnsp,usb3-phy");
	if (IS_ERR(cdns->usb3_phy))
		return dev_err_probe(dev, PTR_ERR(cdns->usb3_phy),
				     "Failed to get cdnsp,usb3-phy\n");

	/* ACPI: _DSD device references aren't resolved by the PHY framework */
	if (!cdns->usb3_phy) {
		cdns->usb3_phy = cdnsp_acpi_phy_ref_get(dev, "cdnsp,usb3-phy");
		if (IS_ERR(cdns->usb3_phy))
			return PTR_ERR(cdns->usb3_phy);
	}

	ret = phy_init(cdns->usb3_phy);
	if (ret)
		goto err_phy3_init;

	ret = set_phy_power_on(cdns);
	if (ret)
		goto err_phy_power_on;

	/* Use CDNSP gadget init for SuperSpeedPlus support */
	cdns->gadget_init = cdnsp_gadget_init;

	ret = cdns_init(cdns);
	if (ret)
		goto err_cdns_init;

	device_set_wakeup_capable(dev, true);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	if (!(cdns->pdata && (cdns->pdata->quirks & CDNS3_DEFAULT_PM_RUNTIME_ALLOW)))
		pm_runtime_forbid(dev);

	/*
	 * Allow enough time for USB3 link training and hub port polling
	 * before entering low power mode. USB3 SuperSpeed link training
	 * can take hundreds of milliseconds, and the hub driver polls
	 * at ~256ms intervals. Must be shorter than the platform poll
	 * interval (1s) to allow D3 between polls for power savings.
	 */
	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_use_autosuspend(dev);

	return 0;

err_cdns_init:
	set_phy_power_off(cdns);
err_phy_power_on:
	phy_exit(cdns->usb3_phy);
err_phy3_init:
	phy_exit(cdns->usb2_phy);

	return ret;
}

/**
 * cdnsp_plat_remove() - unbind drd driver and clean up
 * @pdev: Pointer to Linux platform device
 */
static void cdnsp_plat_remove(struct platform_device *pdev)
{
	struct cdns *cdns = platform_get_drvdata(pdev);
	struct device *dev = cdns->dev;

	pm_runtime_get_sync(dev);
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	cdns_remove(cdns);
	set_phy_power_off(cdns);
	phy_exit(cdns->usb2_phy);
	phy_exit(cdns->usb3_phy);
}

#ifdef CONFIG_PM

static int cdnsp_set_platform_suspend(struct device *dev,
				      bool suspend, bool wakeup)
{
	struct cdns *cdns = dev_get_drvdata(dev);
	int ret = 0;

	if (cdns->pdata && cdns->pdata->platform_suspend)
		ret = cdns->pdata->platform_suspend(dev, suspend, wakeup);

	return ret;
}

static int cdnsp_controller_suspend(struct device *dev, pm_message_t msg)
{
	struct cdns *cdns = dev_get_drvdata(dev);
	bool wakeup;
	unsigned long flags;

	if (cdns->in_lpm)
		return 0;

	if (PMSG_IS_AUTO(msg))
		wakeup = true;
	else
		wakeup = device_may_wakeup(dev);

	cdnsp_set_platform_suspend(cdns->dev, true, wakeup);
	set_phy_power_off(cdns);
	spin_lock_irqsave(&cdns->lock, flags);
	cdns->in_lpm = true;
	spin_unlock_irqrestore(&cdns->lock, flags);
	dev_dbg(cdns->dev, "%s ends\n", __func__);

	return 0;
}

static int cdnsp_controller_resume(struct device *dev, pm_message_t msg)
{
	struct cdns *cdns = dev_get_drvdata(dev);
	int ret;
	unsigned long flags;

	if (!cdns->in_lpm)
		return 0;

	if (cdns_power_is_lost(cdns)) {
		phy_exit(cdns->usb2_phy);
		ret = phy_init(cdns->usb2_phy);
		if (ret)
			return ret;

		phy_exit(cdns->usb3_phy);
		ret = phy_init(cdns->usb3_phy);
		if (ret)
			return ret;
	}

	ret = set_phy_power_on(cdns);
	if (ret)
		return ret;

	cdnsp_set_platform_suspend(cdns->dev, false, false);

	spin_lock_irqsave(&cdns->lock, flags);
	cdns_resume(cdns);
	cdns->in_lpm = false;
	spin_unlock_irqrestore(&cdns->lock, flags);
	cdns_set_active(cdns, !PMSG_IS_AUTO(msg));
	if (cdns->wakeup_pending) {
		cdns->wakeup_pending = false;
		enable_irq(cdns->wakeup_irq);
	}
	dev_dbg(cdns->dev, "%s ends\n", __func__);

	return ret;
}

static int cdnsp_plat_runtime_suspend(struct device *dev)
{
	return cdnsp_controller_suspend(dev, PMSG_AUTO_SUSPEND);
}

static int cdnsp_plat_runtime_resume(struct device *dev)
{
	return cdnsp_controller_resume(dev, PMSG_AUTO_RESUME);
}

#ifdef CONFIG_PM_SLEEP

static int cdnsp_plat_suspend(struct device *dev)
{
	struct cdns *cdns = dev_get_drvdata(dev);
	int ret;

	cdns_suspend(cdns);

	ret = cdnsp_controller_suspend(dev, PMSG_SUSPEND);
	if (ret)
		return ret;

	if (device_may_wakeup(dev) && cdns->wakeup_irq)
		enable_irq_wake(cdns->wakeup_irq);

	return ret;
}

static int cdnsp_plat_resume(struct device *dev)
{
	return cdnsp_controller_resume(dev, PMSG_RESUME);
}
#endif /* CONFIG_PM_SLEEP */
#endif /* CONFIG_PM */

static const struct dev_pm_ops cdnsp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cdnsp_plat_suspend, cdnsp_plat_resume)
	SET_RUNTIME_PM_OPS(cdnsp_plat_runtime_suspend,
			   cdnsp_plat_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id of_cdnsp_match[] = {
	{ .compatible = "cdns,usbssp" },
	{ },
};
MODULE_DEVICE_TABLE(of, of_cdnsp_match);
#endif

static const struct acpi_device_id acpi_cdnsp_match[] = {
	{ "CIXH2031" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, acpi_cdnsp_match);

static struct platform_driver cdnsp_driver = {
	.probe		= cdnsp_plat_probe,
	.remove		= cdnsp_plat_remove,
	.driver		= {
		.name	= "cdns-usbssp",
		.of_match_table	= of_match_ptr(of_cdnsp_match),
		.acpi_match_table = acpi_cdnsp_match,
		.pm	= &cdnsp_pm_ops,
	},
};

module_platform_driver(cdnsp_driver);

MODULE_ALIAS("platform:cdnsp");
MODULE_AUTHOR("Peter Chen <peter.chen@nxp.com>");
MODULE_AUTHOR("Pawel Laszczak <pawell@cadence.com>");
MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Cadence USBSSP DRD Controller Platform Driver");
