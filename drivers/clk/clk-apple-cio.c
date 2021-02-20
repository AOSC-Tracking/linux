// SPDX-License-Identifier: GPL-2.0
/*
 * Apple M1 SoC CIO clock freeze driver
 *
 * Copyright (c) 2021 Corellium LLC
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mailbox_client.h>

struct apple_cio_clk_m1 {
	struct device *dev;
	void __iomem *reg;
	struct clk_bulk_data *clks;
	int num_clks;
	struct mbox_client mbox;
	struct mbox_chan *chan;

	struct clk_hw clkhw;
	unsigned state;
};

#define TIMEOUT_MS		250

static int clk_apple_cio_clk_m1_enable(struct clk_hw *hw)
{
	struct apple_cio_clk_m1 *acc = container_of(hw, struct apple_cio_clk_m1, clkhw);
	int ret;

	if(acc->state & 1)
		return 0;

	ret = clk_bulk_enable(acc->num_clks, acc->clks);
	if(ret < 0)
		return ret;

	if(acc->state == 0) {
		u64 msg[2] = { -1ull, -1ull };

		writel(readl(acc->reg) | 1, acc->reg);
		readl(acc->reg);
		msleep(1000);

		/* this causes the mbox to wait for initial hello sequence to finish */
		ret = mbox_send_message(acc->chan, msg);
		if(ret < 0) {
			dev_err(acc->dev, "CIO mailbox startup failed: %d.\n", ret);
			return 0; /* for now */
		}
	}
	acc->state = 1;

	return 0;
}

static void clk_apple_cio_clk_m1_disable(struct clk_hw *hw)
{
	struct apple_cio_clk_m1 *acc = container_of(hw, struct apple_cio_clk_m1, clkhw);

	if(!(acc->state & 1))
		return;

	clk_bulk_disable(acc->num_clks, acc->clks);
	acc->state = 2;
}

static int clk_apple_cio_clk_m1_is_enabled(struct clk_hw *hw)
{
	struct apple_cio_clk_m1 *acc = container_of(hw, struct apple_cio_clk_m1, clkhw);

	return acc->state & 1;
}

const struct clk_ops clk_apple_cio_clk_m1_ops = {
	.enable = clk_apple_cio_clk_m1_enable,
	.disable = clk_apple_cio_clk_m1_disable,
	.is_enabled = clk_apple_cio_clk_m1_is_enabled,
};

static void apple_cio_clk_m1_mbox_msg(struct mbox_client *cl, void *msg)
{
}

static void apple_cio_clk_m1_free_channel(void *data)
{
	struct apple_cio_clk_m1 *acc = data;
	mbox_free_channel(acc->chan);
}

static int apple_cio_clk_m1_probe(struct platform_device *pdev)
{
	struct clk_init_data clk_init = {};
	struct apple_cio_clk_m1 *acc;
	struct resource *rsrc;
	int err;

	acc = devm_kzalloc(&pdev->dev, sizeof(*acc), GFP_KERNEL);
	if(!acc)
		return -ENOMEM;
	acc->dev = &pdev->dev;

	err = clk_bulk_get_all(acc->dev, &acc->clks);
	if(err < 0) {
		dev_err(&pdev->dev, "clk_bulk_get_all failed.\n");
		return err;
	}
	acc->num_clks = err;

	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	acc->reg = devm_ioremap(acc->dev, rsrc->start, resource_size(rsrc));
	if(IS_ERR(acc->reg)) {
		err = PTR_ERR(acc->reg);
		dev_err(acc->dev, "failed to map MMIO: %d.\n", err);
		return err;
	}

	err = clk_bulk_prepare(acc->num_clks, acc->clks);
	if(err) {
		dev_err(&pdev->dev, "clk_bulk_prepare failed.\n");
		return err;
	}

	platform_set_drvdata(pdev, acc);

	acc->mbox.dev = acc->dev;
	acc->mbox.rx_callback = apple_cio_clk_m1_mbox_msg;
	acc->mbox.tx_block = true;
	acc->mbox.tx_tout = TIMEOUT_MS;
	acc->chan = mbox_request_channel(&acc->mbox, 0);
	if(IS_ERR(acc->chan)) {
		err = PTR_ERR(acc->chan);
		if(err != -EPROBE_DEFER)
			dev_err(acc->dev, "failed to attach to mailbox: %d.\n", err);
		return err;
	}
	err = devm_add_action(acc->dev, apple_cio_clk_m1_free_channel, acc);
	if(err)
		return err;

	err = of_property_read_string(acc->dev->of_node, "clock-output-names", &clk_init.name);
	if(err < 0)
		return err;
	clk_init.num_parents = 0;
	clk_init.ops = &clk_apple_cio_clk_m1_ops;

	acc->clkhw.init = &clk_init;
	err = devm_clk_hw_register(acc->dev, &acc->clkhw);
	if(err)
		return err;

	return of_clk_add_provider(acc->dev->of_node, of_clk_src_simple_get, acc->clkhw.clk);
}

static const struct of_device_id apple_cio_clk_m1_match[] = {
	{ .compatible = "apple,cio-reconfig-m1" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_cio_clk_m1_match);

static struct platform_driver apple_cio_clk_m1_driver = {
	.driver = {
		.name = "clk-apple-cio",
		.of_match_table = apple_cio_clk_m1_match,
	},
	.probe = apple_cio_clk_m1_probe,
};

module_platform_driver(apple_cio_clk_m1_driver);

MODULE_AUTHOR("Corellium LLC");
MODULE_DESCRIPTION("Apple M1 SoC CIO clock freeze driver");
MODULE_LICENSE("GPL");
