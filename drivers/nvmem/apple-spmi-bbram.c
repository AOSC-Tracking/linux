/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for BBRAM inside PMU of Apple SoC-based devices.
 *
 * Copyright (C) 2020-1 Corellium LLC
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/of.h>

struct apple_spmi_bbram {
	struct device *dev;
	struct regmap *regmap;
	struct nvmem_config cfg;
	struct nvmem_device *nvmem;
	char shortname[64];
	u32 base, size;
};

static int apple_spmi_bbram_write(void *context, unsigned int offset, void *val, size_t bytes)
{
	struct apple_spmi_bbram *bbram = context;
	return regmap_bulk_write(bbram->regmap, bbram->base + offset, val, bytes);
}

static int apple_spmi_bbram_read(void *context, unsigned int offset, void *val, size_t bytes)
{
	struct apple_spmi_bbram *bbram = context;
	return regmap_bulk_read(bbram->regmap, bbram->base + offset, val, bytes);
}

static int apple_spmi_bbram_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct apple_spmi_bbram *bbram;
	struct nvmem_config *cfg;
	char *sep;

	bbram = devm_kzalloc(&pdev->dev, sizeof(struct apple_spmi_bbram), GFP_KERNEL);
	if(!bbram)
		return -ENOMEM;

	bbram->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if(!bbram->regmap) {
		dev_err(&pdev->dev, "Parent regmap unavailable.\n");
		return -ENXIO;
	}

	if(of_property_read_u32_index(node, "base", 0, &bbram->base) < 0 ||
	   of_property_read_u32_index(node, "base", 1, &bbram->size) < 0) {
		dev_err(&pdev->dev, "BBRAM requires a 'base' property that has 2 elements.\n");
		return -EINVAL;
	}

	bbram->dev = &pdev->dev;
	platform_set_drvdata(pdev, bbram);

	cfg = &bbram->cfg;
	cfg->priv = bbram;
	cfg->name = "bbram";
	cfg->dev = &pdev->dev;
	cfg->root_only = 1;
	cfg->type = NVMEM_TYPE_BATTERY_BACKED;
	cfg->stride = 1;
	cfg->word_size = 1;
	cfg->size = bbram->size,
	cfg->owner = THIS_MODULE;
	cfg->reg_read  = apple_spmi_bbram_read;
	cfg->reg_write = apple_spmi_bbram_write;

	if(!node->name)
		return -EINVAL;
	sep = strchr(node->name, '@');
	if(sep) {
		if(sep - node->name >= sizeof(bbram->shortname)) {
			dev_err(&pdev->dev, "BBRAM node name '%s' too long.\n", node->name);
			return -EINVAL;
		}
		memcpy(bbram->shortname, node->name, sep - node->name);
		cfg->name = bbram->shortname;
	} else
		cfg->name = node->name;

	bbram->nvmem = devm_nvmem_register(&pdev->dev, cfg);
	return PTR_ERR_OR_ZERO(bbram->nvmem);
}

static const struct of_device_id apple_spmi_bbram_of_match[] = {
	{ .compatible = "apple,spmi-bbram-v0" },
	{ },
};

static struct platform_driver apple_spmi_bbram_driver = {
	.driver = {
		.name = "apple-spmi-bbram",
		.of_match_table = of_match_ptr(apple_spmi_bbram_of_match),
	},
	.probe = apple_spmi_bbram_probe,
};

module_platform_driver(apple_spmi_bbram_driver);
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL v2");
