// SPDX-License-Identifier: GPL-2.0-only
/* Dedicated orientation-specific PIPE PHYs on SpacemiT K3 USB Port A. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb/ch9.h>
#include "phy-k3-common.h"

#define PHY_POWER_SELECT	0x40
#define PHY_STATUS_OVERRIDE	BIT(10)
#define PHY_STATUS_VALUE	BIT(9)

struct k3_usb3 {
	void __iomem *base[2];
	struct clk *clk;
	struct regmap *spare;
	int speed;
	bool initialized;
};

static void k3_usb3_park(struct k3_usb3 *priv)
{
	u32 val;
	int i;

	/* Hold PIPE PHYSTATUS low while the controller uses only UTMI. */
	for (i = 0; i < ARRAY_SIZE(priv->base); i++) {
		val = readl(priv->base[i] + PHY_POWER_SELECT);
		val = (val & ~PHY_STATUS_VALUE) | PHY_STATUS_OVERRIDE;
		writel(val, priv->base[i] + PHY_POWER_SELECT);
	}
	usleep_range(200, 250);
}

static int k3_usb3_init(struct phy *phy)
{
	struct k3_usb3 *priv = phy_get_drvdata(phy);
	u32 val;
	int ret, i;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;
	if (priv->speed == USB_SPEED_HIGH || priv->speed == USB_SPEED_FULL) {
		k3_usb3_park(priv);
		priv->initialized = true;
		return 0;
	}
	ret = k3_phy_calibrate(priv->spare);
	if (ret)
		goto fail;
	for (i = 0; i < ARRAY_SIZE(priv->base); i++) {
		val = readl(priv->base[i] + PHY_POWER_SELECT);
		writel(val & ~PHY_STATUS_OVERRIDE, priv->base[i] + PHY_POWER_SELECT);
		ret = k3_usb3phy_init_single(phy, priv->base[i]);
		if (ret)
			goto fail;
	}
	priv->initialized = true;
	return 0;
fail:
	k3_usb3_park(priv);
	clk_disable_unprepare(priv->clk);
	return ret;
}

static int k3_usb3_exit(struct phy *phy)
{
	struct k3_usb3 *priv = phy_get_drvdata(phy);

	k3_usb3_park(priv);
	priv->initialized = false;
	clk_disable_unprepare(priv->clk);
	return 0;
}

static int k3_usb3_set_speed(struct phy *phy, int speed)
{
	struct k3_usb3 *priv = phy_get_drvdata(phy);

	if (priv->initialized)
		return speed == priv->speed ? 0 : -EBUSY;
	if (speed != USB_SPEED_UNKNOWN && speed != USB_SPEED_FULL &&
	    speed != USB_SPEED_HIGH && speed != USB_SPEED_SUPER &&
	    speed != USB_SPEED_SUPER_PLUS)
		return -EINVAL;
	priv->speed = speed;
	return 0;
}

static const struct phy_ops k3_usb3_ops = {
	.init = k3_usb3_init,
	.exit = k3_usb3_exit,
	.set_speed = k3_usb3_set_speed,
	.owner = THIS_MODULE,
};

static int k3_usb3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct k3_usb3 *priv;
	struct phy *phy;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(priv->base); i++) {
		priv->base[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(priv->base[i]))
			return PTR_ERR(priv->base[i]);
	}
	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk), "missing PHY clock\n");
	priv->spare = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "spacemit,syscon-apb-spare");
	if (IS_ERR(priv->spare))
		return dev_err_probe(dev, PTR_ERR(priv->spare), "missing calibration registers\n");
	phy = devm_phy_create(dev, NULL, &k3_usb3_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, priv);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id k3_usb3_match[] = {
	{ .compatible = "spacemit,k3-usb3-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, k3_usb3_match);

static struct platform_driver k3_usb3_driver = {
	.probe = k3_usb3_probe,
	.driver = {
		.name = "k3-usb3-phy",
		.of_match_table = k3_usb3_match,
	},
};
module_platform_driver(k3_usb3_driver);
MODULE_DESCRIPTION("SpacemiT K3 dedicated USB3 PHY");
MODULE_LICENSE("GPL");
