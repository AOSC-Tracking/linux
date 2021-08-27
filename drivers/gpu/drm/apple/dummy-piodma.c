// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io> */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>

static int dcp_piodma_probe(struct platform_device *pdev)
{
	return dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
}

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,dcp-piodma" },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver dcp_piodma_platform_driver = {
	.probe		= dcp_piodma_probe,
	.driver	= {
		.name = "apple,dcp-piodma",
		.of_match_table	= of_match,
	},
};

module_platform_driver(dcp_piodma_platform_driver);

MODULE_AUTHOR("Alyssa Rosenzweig <alyssa@rosenzweig.io>");
MODULE_DESCRIPTION("[HACK] Apple DCP PIODMA shim");
MODULE_LICENSE("GPL v2");
