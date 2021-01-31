// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Corellium LLC
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

static const struct of_device_id apple_spmi_pmu_id_table[] = {
	{ .compatible = "apple,spmi-pmu-sera" },
	{ .compatible = "apple,spmi-pmu" },
};

static const struct regmap_config spmi_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= true,
};

static int apple_spmi_pmu_probe(struct spmi_device *sdev)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spmi_ext(sdev, &spmi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return devm_of_platform_populate(&sdev->dev);
}

MODULE_DEVICE_TABLE(of, apple_spmi_pmu_id_table);

static struct spmi_driver apple_spmi_pmu_driver = {
	.probe = apple_spmi_pmu_probe,
	.driver = {
		.name = "apple-spmi-pmu",
		.of_match_table = apple_spmi_pmu_id_table,
	},
};
module_spmi_driver(apple_spmi_pmu_driver);

MODULE_DESCRIPTION("Apple SPMI PMU driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Corellium LLC");
