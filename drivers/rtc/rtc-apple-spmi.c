/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 Corellium LLC
 */

#include <linux/of.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

struct apple_spmi_rtc {
	struct device *dev;
	struct regmap *regmap;
	struct rtc_device *rtc;
	u32 base, off_base;
	u64 offs;
};

static int apple_spmi_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct apple_spmi_rtc *artc = dev_get_drvdata(dev);
	int ret;
	u8 data[6];
	u64 time;

	ret = regmap_bulk_read(artc->regmap, artc->base, data, sizeof(data));
	if(ret < 0) {
		dev_err(artc->dev, "RTC read time failed: %d\n", ret);
		return ret;
	}

	time = data[0] | ((u64)data[1] << 8) | ((u64)data[2] << 16) |
	       ((u64)data[3] << 24) | ((u64)data[4] << 32) | ((u64)data[5] << 40);
	time += artc->offs << 1; /* offset is stored as 33.15, RTC hardware uses 32.16 */

	rtc_time64_to_tm(time >> 16, tm);
	return 0;
}

static int apple_spmi_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct apple_spmi_rtc *artc = dev_get_drvdata(dev);
	int ret;
	u8 data[6];
	u64 time, offs;

	if(!artc->off_base)
		return -EPERM;

	offs = rtc_tm_to_time64(tm) << 16;

	ret = regmap_bulk_read(artc->regmap, artc->base, data, sizeof(data));
	if(ret < 0) {
		dev_err(artc->dev, "RTC read time failed: %d\n", ret);
		return ret;
	}

	time = data[0] | ((u64)data[1] << 8) | ((u64)data[2] << 16) |
	       ((u64)data[3] << 24) | ((u64)data[4] << 32) | ((u64)data[5] << 40);
	offs = (offs - time) >> 1;

	data[0] = offs;
	data[1] = offs >> 8;
	data[2] = offs >> 16;
	data[3] = offs >> 24;
	data[4] = offs >> 32;
	data[5] = offs >> 40;

	ret = regmap_bulk_write(artc->regmap, artc->off_base, data, sizeof(data));
	if(ret < 0) {
		dev_err(artc->dev, "RTC set time offset failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static const struct rtc_class_ops apple_spmi_rtc_ops = {
	.read_time	= apple_spmi_rtc_read_time,
	.set_time	= apple_spmi_rtc_set_time,
};

static u32 apple_spmi_rtc_get_base(struct platform_device *pdev, int index)
{
	u32 val;
	if(of_property_read_u32_index(pdev->dev.of_node, "base", index * 2, &val) < 0)
		return 0;
	return val;
}

static int apple_spmi_rtc_probe(struct platform_device *pdev)
{
	struct apple_spmi_rtc *artc;
	int ret;
	u8 data[6];

	artc = devm_kzalloc(&pdev->dev, sizeof(*artc), GFP_KERNEL);
	if(!artc)
		return -ENOMEM;
	artc->dev = &pdev->dev;

	artc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if(!artc->regmap) {
		dev_err(artc->dev, "Parent regmap unavailable.\n");
		return -ENXIO;
	}

	platform_set_drvdata(pdev, artc);

	artc->base = apple_spmi_rtc_get_base(pdev, 0);
	artc->off_base = apple_spmi_rtc_get_base(pdev, 1);
	if(!artc->base) {
		dev_err(artc->dev, "Missing RTC base property.\n");
		return -EINVAL;
	}

	if(artc->off_base) {
		ret = regmap_bulk_read(artc->regmap, artc->off_base, data, 6);
		if(ret) {
			dev_err(artc->dev, "Failed reading RTC offset: %d.\n", ret);
			return ret;
		}
		artc->offs = data[0] | ((u64)data[1] << 8) | ((u64)data[2] << 16) |
			     ((u64)data[3] << 24) | ((u64)data[4] << 32) | ((u64)data[5] << 40);
	}

	artc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(artc->rtc))
		return PTR_ERR(artc->rtc);

	artc->rtc->ops = &apple_spmi_rtc_ops;
	artc->rtc->range_max = U32_MAX * 2;
	return devm_rtc_register_device(artc->rtc);
}

static const struct of_device_id apple_spmi_rtc_id_table[] = {
	{ .compatible = "apple,spmi-rtc-v0" },
	{ },
};
MODULE_DEVICE_TABLE(of, apple_spmi_rtc_id_table);

static struct platform_driver apple_spmi_rtc_driver = {
	.probe		= apple_spmi_rtc_probe,
	.driver	= {
		.name		= "rtc-apple_spmi",
		.of_match_table	= apple_spmi_rtc_id_table,
	},
};

module_platform_driver(apple_spmi_rtc_driver);

MODULE_DESCRIPTION("Apple SPMI RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Corellium LLC");
