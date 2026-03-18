// SPDX-License-Identifier: GPL-2.0-only
/*
 * CIX Sky1 Audio Subsystem Reset Controller
 *
 * Copyright 2024 Cix Technology Group Co., Ltd.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/device/bus.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/reset/sky1-reset-audss.h>

#define SKY1_AUDSS_SW_RST		0x78

#define SKY1_RESET_SLEEP_US		10000

struct sky1_audss_reset {
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
};

static int sky1_audss_reset_update(struct sky1_audss_reset *rst,
				   unsigned long id, bool assert)
{
	unsigned int bit = BIT(id);
	unsigned int val = assert ? 0 : bit;

	return regmap_update_bits(rst->regmap, SKY1_AUDSS_SW_RST, bit, val);
}

static int sky1_audss_reset(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct sky1_audss_reset *rst = container_of(rcdev, struct sky1_audss_reset, rcdev);
	int ret;

	ret = sky1_audss_reset_update(rst, id, true);
	if (ret)
		return ret;

	usleep_range(SKY1_RESET_SLEEP_US, SKY1_RESET_SLEEP_US * 2);

	ret = sky1_audss_reset_update(rst, id, false);
	if (ret)
		return ret;

	/* Ensure component is ready after reset release */
	usleep_range(SKY1_RESET_SLEEP_US, SKY1_RESET_SLEEP_US * 2);

	return 0;
}

static int sky1_audss_reset_assert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct sky1_audss_reset *rst = container_of(rcdev, struct sky1_audss_reset, rcdev);

	return sky1_audss_reset_update(rst, id, true);
}

static int sky1_audss_reset_deassert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct sky1_audss_reset *rst = container_of(rcdev, struct sky1_audss_reset, rcdev);

	return sky1_audss_reset_update(rst, id, false);
}

static int sky1_audss_reset_status(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct sky1_audss_reset *rst = container_of(rcdev, struct sky1_audss_reset, rcdev);
	unsigned int val;
	int ret;

	ret = regmap_read(rst->regmap, SKY1_AUDSS_SW_RST, &val);
	if (ret)
		return ret;

	/* Reset is active-low: bit=0 means in reset */
	return !(val & BIT(id));
}

static const struct reset_control_ops sky1_audss_reset_ops = {
	.reset = sky1_audss_reset,
	.assert = sky1_audss_reset_assert,
	.deassert = sky1_audss_reset_deassert,
	.status = sky1_audss_reset_status,
};

static int sky1_audss_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *parent_np;
	struct sky1_audss_reset *rst;
	struct regmap *regmap;
	int ret;

	parent_np = of_get_parent(dev->of_node);
	regmap = syscon_node_to_regmap(parent_np);
	of_node_put(parent_np);

	if (IS_ERR(regmap) && has_acpi_companion(dev)) {
		struct fwnode_handle *fw;
		struct device *syscon_dev;

		fw = fwnode_find_reference(dev_fwnode(dev), "audss_cru", 0);
		if (!IS_ERR(fw)) {
			syscon_dev = bus_find_device_by_fwnode(
					&platform_bus_type, fw);
			fwnode_handle_put(fw);
			if (syscon_dev) {
				regmap = dev_get_regmap(syscon_dev, NULL);
				put_device(syscon_dev);
				if (!regmap)
					regmap = ERR_PTR(-EPROBE_DEFER);
			}
		}
	}

	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to get parent regmap\n");

	rst = devm_kzalloc(dev, sizeof(*rst), GFP_KERNEL);
	if (!rst)
		return -ENOMEM;

	rst->regmap = regmap;
	rst->rcdev.owner = THIS_MODULE;
	rst->rcdev.nr_resets = SKY1_AUDSS_SW_RESET_NUM;
	rst->rcdev.ops = &sky1_audss_reset_ops;
	rst->rcdev.of_node = dev->of_node;
	rst->rcdev.dev = dev;

	ret = devm_reset_controller_register(dev, &rst->rcdev);
	if (ret)
		return ret;

	/* Allow DLKL consumers (DMA, I2S, etc.) to enumerate now */
	if (has_acpi_companion(dev))
		acpi_dev_clear_dependencies(ACPI_COMPANION(dev));

	/*
	 * Under ACPI, consumers find reset lines via lookup table
	 * (equivalent of DT phandle references).  Entries derived
	 * from vendor DSDT RSTL objects on each consumer device.
	 */
	if (has_acpi_companion(dev)) {
		static struct reset_control_lookup lookups[] = {
			{ .index = 15, .dev_id = "CIXH1006:00",
			  .con_id = "dma_reset" },
			{ .index = 14, .dev_id = "CIXH6020:00",
			  .con_id = "hda" },
		};
		int i;

		for (i = 0; i < ARRAY_SIZE(lookups); i++)
			lookups[i].provider = dev_name(dev);
		reset_controller_add_lookup(lookups, ARRAY_SIZE(lookups));
	}

	return 0;
}

static const struct of_device_id sky1_audss_reset_of_match[] = {
	{ .compatible = "cix,sky1-audss-reset" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sky1_audss_reset_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id sky1_audss_reset_acpi_match[] = {
	{ "CIXH6062", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, sky1_audss_reset_acpi_match);
#endif

static struct platform_driver sky1_audss_reset_driver = {
	.probe = sky1_audss_reset_probe,
	.driver = {
		.name = "sky1-audss-reset",
		.of_match_table = sky1_audss_reset_of_match,
		.acpi_match_table = ACPI_PTR(sky1_audss_reset_acpi_match),
	},
};
module_platform_driver(sky1_audss_reset_driver);

MODULE_AUTHOR("Joakim Zhang <joakim.zhang@cixtech.com>");
MODULE_DESCRIPTION("CIX Sky1 Audio Subsystem Reset Controller");
MODULE_LICENSE("GPL");
