/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * Copyright (C) 2020 Corellium LLC
 *
 * Clock gating via Apple SoC PMGR
 */

#include <linux/clk-provider.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <asm/io.h>

#define PMGR_GATE	1

#define MAX_BASES	4

struct clk_apple_pmgr {
	struct clk_hw hw;
	unsigned type;
	const char *name;
	void __iomem *bases[MAX_BASES];
	const unsigned *seq[4];
	unsigned seqn[4];
	u32 freq_target;
};

#define to_clk_apple_pmgr(_hw) container_of(_hw, struct clk_apple_pmgr, hw)

static void clk_apple_pmgr_run_seq(struct clk_apple_pmgr *clk, const unsigned *seq, unsigned n)
{
	if(!n || !seq)
		return;
	while(n --) {
		writel(seq[2], clk->bases[seq[0]] + seq[1]);
		seq += 3;
	}
}

/********************* PMGR_GATE *********************/

static int clk_apple_pmgr_gate_enable(struct clk_hw *hw)
{
	struct clk_apple_pmgr *clk = to_clk_apple_pmgr(hw);
	unsigned max = 10000;
	uint32_t val;

	clk_apple_pmgr_run_seq(clk, clk->seq[1], clk->seqn[1]);

	val = readl(clk->bases[0]);

	val |= 15;
	writel(val, clk->bases[0]);

	while(max --) {
		mb();
		val = readl(clk->bases[0]);

		if(((val >> 4) & 15) == 15) {
			writel(val | 0x10000000, clk->bases[0]);
			clk_apple_pmgr_run_seq(clk, clk->seq[3], clk->seqn[3]);
			return 0;
		}

		if(max < 5000)
			udelay(20);
		cpu_relax();
	}

	pr_err("%s: failed to enable PMGR clock\n", clk->name);

	return -ETIMEDOUT;
}

static void clk_apple_pmgr_gate_disable(struct clk_hw *hw)
{
	struct clk_apple_pmgr *clk = to_clk_apple_pmgr(hw);
	unsigned max = 10000;
	uint32_t val;

	clk_apple_pmgr_run_seq(clk, clk->seq[0], clk->seqn[0]);

	val = readl(clk->bases[0]);
	val |= 0x300;
	val &= ~15;
	writel(val, clk->bases[0]);

	while(max --) {
		val = readl(clk->bases[0]);

		if(((val >> 4) & 15) == 0) {
			clk_apple_pmgr_run_seq(clk, clk->seq[2], clk->seqn[2]);
			return;
		}

		cpu_relax();
	}

	pr_err("%s: failed to disable PMGR clock\n", clk->name);
}

static int clk_apple_pmgr_gate_is_enabled(struct clk_hw *hw)
{
	struct clk_apple_pmgr *clk = to_clk_apple_pmgr(hw);
	uint32_t val;

	val = readl(clk->bases[0]);
	return ((val >> 4) & 15) == 15;
}

const struct clk_ops clk_apple_pmgr_gate_ops = {
	.enable = clk_apple_pmgr_gate_enable,
	.disable = clk_apple_pmgr_gate_disable,
	.is_enabled = clk_apple_pmgr_gate_is_enabled,
};

static int clk_prepare_apple_pmgr_gate(struct clk_apple_pmgr *clk_apple_pmgr, struct clk_init_data *init, struct device *dev, struct device_node *node, const char * const *parent_names, u8 num_parents, void __iomem **bases)
{
	init->ops = &clk_apple_pmgr_gate_ops;
	init->flags = CLK_SET_RATE_PARENT;
	return 0;
}

/********************* shared code *********************/

static int clk_apple_pmgr_driver_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const char **parent_names = NULL;
	unsigned num_parents, type;
	struct clk *clk;
	void __iomem *bases[MAX_BASES] = { NULL };
	int i, n, err;
	unsigned seqn[4] = { 0 }, *seq[4] = { NULL };
	static const char * const seqname[4] = { "pre-down", "pre-up", "post-down", "post-up" };
	struct clk_apple_pmgr *clk_apple_pmgr;
	struct clk_init_data init = {};
	struct clk_hw *hw;

	num_parents = of_clk_get_parent_count(node);
	if(num_parents) {
		parent_names = devm_kcalloc(&pdev->dev, num_parents, sizeof(char *), GFP_KERNEL);
		if(!parent_names)
			return -ENOMEM;
		of_clk_parent_fill(node, parent_names, num_parents);
	}

	n = of_property_count_elems_of_size(node, "reg", sizeof(uint32_t) * (of_n_addr_cells(node) + of_n_size_cells(node)));
	if(n < 0) {
		pr_err("%pOFn: %s: not enough MMIO ranges.\n", node, __func__);
		return -EINVAL;
	}
	if(n > MAX_BASES) {
		pr_err("%pOFn: %s: too many MMIO ranges.\n", node, __func__);
		return -EINVAL;
	}

	for(i=0; i<n; i++) {
		bases[i] = of_iomap(node, i);
		if(!bases[i]) {
			pr_err("%pOFn: %s: unable to map MMIO range %d.\n", node, __func__, i);
			return -EINVAL;
		}
	}

	for(i=0; i<4; i++) {
		n = of_property_count_elems_of_size(node, seqname[i], sizeof(uint32_t));
		if(n > 0) {
			seq[i] = devm_kcalloc(&pdev->dev, n, sizeof(unsigned), GFP_KERNEL);
			if(!seq[i])
				return -ENOMEM;
			seqn[i] = of_property_read_variable_u32_array(node, seqname[i], seq[i], 0, n) / 3;
		}
	}

	type = 0;
	if(of_device_is_compatible(node, "apple,pmgr-clk-gate"))
		type = PMGR_GATE;

	clk_apple_pmgr = devm_kzalloc(&pdev->dev, sizeof(*clk_apple_pmgr), GFP_KERNEL);
	if(!clk_apple_pmgr)
		return -ENOMEM;

	clk_apple_pmgr->type = type;
	for(i=0; i<MAX_BASES; i++)
		clk_apple_pmgr->bases[i] = bases[i];
	for(i=0; i<4; i++) {
		clk_apple_pmgr->seq[i] = seq[i];
		clk_apple_pmgr->seqn[i] = seqn[i];
	}
	clk_apple_pmgr->name = node->name;

	init.name = node->name;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	switch(type) {
	case PMGR_GATE:
		err = clk_prepare_apple_pmgr_gate(clk_apple_pmgr, &init, &pdev->dev, node, parent_names, num_parents, bases);
		break;
	default:
		pr_err("%pOFn: %s: unsupported device type\n", node, __func__);
		return -EINVAL;
	}
	if(err)
		return err;

	clk_apple_pmgr->hw.init = &init;

	hw = &clk_apple_pmgr->hw;
	err = devm_clk_hw_register(&pdev->dev, hw);
	if(err)
		return err;

	clk = hw->clk;
	return of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static const struct of_device_id clk_apple_pmgr_match_table[] = {
	{ .compatible = "apple,pmgr-clk-gate" },
	{ }
};

static struct platform_driver clk_apple_pmgr_driver = {
	.probe = clk_apple_pmgr_driver_probe,
	.driver = {
		.name = "clk-apple-pmgr",
		.of_match_table = clk_apple_pmgr_match_table,
	},
};
builtin_platform_driver(clk_apple_pmgr_driver);
