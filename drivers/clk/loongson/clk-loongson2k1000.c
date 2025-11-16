/*
 * Copyright (c) 2022 Michael <michael5hzg@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <loongson-2k.h>
#include <dt-bindings/clock/loongson2k1000-clock.h>

#include "clk.h"

#define NODE_CLK_FREQ_DEF		900000000

static unsigned long refclk_rate;
static void __iomem *reg;

#ifdef CONFIG_LOONGSON2_SYSCLK_SOFT
struct pll_config {
	unsigned int l1_div_ref;
	unsigned int l1_loopc;
	unsigned int l2_divout;
};

// 系统时钟需满足如下几个限制：
// 1.可配分频器的输出 refclk/L1_div_ref 在 20~40MHz 范围内
// 2.PLL 倍频值 refclk/L1_div_ref*L1_loopc 需要在 1.2GHz~3.2GHz
static int ls2k_caculate_freq(unsigned long freq, struct pll_config *pll_cfg)
{
	unsigned long pll_in_clk;
	unsigned long loopc_min, loopc_max, loopc;
	unsigned long div, div_max;
	int found = 0;

	if (!pll_cfg)
		return -EINVAL;

	// Keep refclk/L1_div_ref in range 20MHz ~ 40Mhz
	pll_cfg->l1_div_ref = 4;
	pll_in_clk = refclk_rate / pll_cfg->l1_div_ref;		// 25MHz
	loopc_min = 1200000000 / pll_in_clk;
	loopc_max = 3200000000 / pll_in_clk;
	div_max = loopc_max; 	// pll_in_clk * loopc / div > pll_in_clk

	for (div = 1; div < div_max; div++) {
		loopc = freq * div / pll_in_clk;
		if (loopc >= loopc_min && loopc <= loopc_max) {
			found = 1;
			break;
		}
	}

	if (!found) {
		pll_cfg->l1_div_ref = 0;
		pll_cfg->l1_loopc = 0;
		pll_cfg->l2_divout = 0;
		return -EINVAL;
	}

	pll_cfg->l1_loopc = loopc;
	pll_cfg->l2_divout = div;
	return 0;
}

static void ls2k_config_pll(void __iomem *pll_base, struct pll_config *pll_cfg)
{
	int timeout = 200;
	u64 val;

	/* set sel_pll_out0 0 */
	val = __raw_readq(pll_base);
	val &= ~BIT(0);
	__raw_writeq(val, pll_base);

	/* pll_pd 1 */
	val = __raw_readq(pll_base);
	val |= BIT(19);
	__raw_writeq(val, pll_base);

	/* set_pll_param 0 */
	val = __raw_readq(pll_base);
	val &= ~BIT(2);
	__raw_writeq(val, pll_base);

	/* set new div ref, loopc, div out */
	/* clear old value first*/
	val = BIT(7) | (BIT(10) | BIT(11)) |
		((u64)(pll_cfg->l1_loopc) << 32) |
		(pll_cfg->l1_div_ref << 26);
	__raw_writeq(val, pll_base);
	__raw_writeq(pll_cfg->l2_divout, pll_base + 8);

	/* set_pll_param 1 */
	val = __raw_readq(pll_base);
	val |= BIT(2);
	__raw_writeq(val, pll_base);

	/* pll_pd 0 */
	val = __raw_readq(pll_base);
	val &= ~BIT(19);
	__raw_writeq(val, pll_base);

	/* wait pll lock */
	while(!(__raw_readq(pll_base) & BIT(16)) && (timeout-- > 0)) {
		udelay(1);
	}

	/* set sel_pll_out0 1 */
	val = __raw_readq(pll_base);
	val |= BIT(0);
	__raw_writeq(val, pll_base);
}


static unsigned long ls2x_node_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	u64 ctrl;
	unsigned int l1div_loopc, l1div_ref, l2div_out;
	unsigned int mult, div;
	unsigned long rate;

	/* node cpu clk */
	ctrl = __raw_readq(reg + PLL_SYS0_OFF);
	l1div_loopc = (ctrl >> NODE_L1DIV_LOOPC_SHIFT) & NODE_L1DIV_LOOPC_MARK;
	l1div_ref = (ctrl >> NODE_L1DIV_REF_SHIFT) & NODE_L1DIV_REF_MARK;
	ctrl = __raw_readq(reg + PLL_SYS1_OFF);
	l2div_out = (ctrl >> NODE_L2DIV_OUT_SHIFT) & NODE_L2DIV_OUT_MARK;
	mult = l1div_loopc;
	div = l1div_ref * l2div_out;
	if (div == 0) {
		rate = NODE_CLK_FREQ_DEF;
	} else {
		rate = refclk_rate * mult / div;
	}

	return rate;
}

static long ls2x_node_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct pll_config pll_cfg;
	unsigned int mult, div;

	if (rate == 0 || *parent_rate == 0)
		return 0;

	if (ls2k_caculate_freq(rate, &pll_cfg)) {
		return NODE_CLK_FREQ_DEF;
	}

	mult = pll_cfg.l1_loopc;
	div = pll_cfg.l1_div_ref * pll_cfg.l2_divout;

	return refclk_rate * mult / div;
}

static int ls2x_node_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct pll_config pll_cfg;

	if (rate == 0 || parent_rate == 0)
		return -EINVAL;

	if (ls2k_caculate_freq(rate, &pll_cfg)) {
		return -EINVAL;
	}

	ls2k_config_pll(reg + PLL_SYS0_OFF, &pll_cfg);
	return 0;
}

static struct clk_ops ls2x_node_clk_ops = {
	.recalc_rate = ls2x_node_recalc_rate,
	.round_rate = ls2x_node_round_rate,
	.set_rate = ls2x_node_set_rate,
};

static unsigned long ls2x_cpu_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	unsigned int freqscale;
	u64 val;

	val = __raw_readq(reg + PLL_FREQ_SC);
	freqscale = (val >> FREQSCALE_NODE_SHIFT) & FREQSCALE_NODE_MARK;

	return parent_rate * (freqscale + 1) / 8;
}

static long ls2x_cpu_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	int scale;
	if (!rate || !*parent_rate)
		return 0;

	scale = rate * 8 / (*parent_rate) - 1;
	return (*parent_rate) * (scale + 1) / 8;
}

static int ls2x_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	int scale;
	u64 val;

	if (!rate || !parent_rate)
		return -EINVAL;

	scale = rate * 8 / parent_rate - 1;
	val = __raw_readq(reg + PLL_FREQ_SC);
	val &= (~FREQSCALE_NODE_MARK) << FREQSCALE_NODE_SHIFT;
	val |= (scale & FREQSCALE_NODE_MARK) << FREQSCALE_NODE_SHIFT;
	__raw_writeq(val, reg + PLL_FREQ_SC);
	return 0;
}

static struct clk_ops ls2x_cpu_clk_ops = {
	.recalc_rate = ls2x_cpu_recalc_rate,
	.round_rate = ls2x_cpu_round_rate,
	.set_rate = ls2x_cpu_set_rate,
};

#ifdef CONFIG_SND_LS1X_SOC_I2S
#if defined(CONFIG_MIPS)
#define REMAP CKSEG1ADDR
#else
#define REMAP TO_UNCAC
#endif
#define LS2X_APB_BASE			0x1fe00000
#define LS2X_I2S_CONFIG_REG		\
			((void __iomem *)REMAP(LS2X_APB_BASE + (0xd004)))
#define LS2X_I2S_CONFIG_MCLK_MASK	(0xFF)
#define LS2X_I2S_CONFIG_MCLK_SHIFT	(0)

static unsigned long ls2x_i2s_mclk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	u32 ratio, val;

	val = __raw_readl(LS2X_I2S_CONFIG_REG);
	ratio = (val >> LS2X_I2S_CONFIG_MCLK_SHIFT) & LS2X_I2S_CONFIG_MCLK_MASK;

	return parent_rate / (2 * (ratio + 1));
}

static long ls2x_i2s_mclk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	u32 ratio;
	if (!rate || !*parent_rate)
		return 0;

	ratio = (*parent_rate) / (2 * rate) - 1;
	return (*parent_rate) / (2 * (ratio + 1));
}

static int ls2x_i2s_mclk_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	u32 ratio, val;

	ratio = parent_rate / (2 * rate) - 1;
	val = __raw_readl(LS2X_I2S_CONFIG_REG);
	val &= ~(LS2X_I2S_CONFIG_MCLK_MASK << LS2X_I2S_CONFIG_MCLK_SHIFT);
	val |= (ratio & LS2X_I2S_CONFIG_MCLK_MASK) << LS2X_I2S_CONFIG_MCLK_SHIFT;
	__raw_writel(val, LS2X_I2S_CONFIG_REG);
	return 0;
}

static struct clk_ops ls2x_i2s_mclk_ops = {
	.recalc_rate = ls2x_i2s_mclk_recalc_rate,
	.round_rate = ls2x_i2s_mclk_round_rate,
	.set_rate = ls2x_i2s_mclk_set_rate,

};
#endif // CONFIG_SND_LS1X_SOC_I2S
#endif // CONFIG_LOONGSON2K_SYSCLK_SOFT


static struct clk ** __init ls2x_clk_setup(struct device_node *node)
{
	struct clk_onecell_data *clk_data;
    struct clk **clks;
    int ndivs = CLK_CNT;

    char *con_ids[] = {"ref_clk", "node_clk", "cpu_clk", "ddr_clk",
     "gpu_clk", "hda_clk", "dc_clk", "pix0_clk", "pix1_clk",
     "gmac_clk", "sata_clk", "usb_clk", "apb_clk", "spi_clk", "i2s_mclk"};

	int err = -EINVAL;

    reg = of_iomap(node, 0);
    if(!reg) {
        pr_err("Could not map registers for divs-clk: %pOF\n", node);
		return NULL;
    }
    pr_debug("lsdbg---> ls2x_clk_setup reg:0x%p 0x%lx", reg, (ulong)(reg - PLL_SYS0_OFF));
	reg -= PLL_SYS0_OFF;

    clk_data = kmalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
    if(!clk_data)
        goto out_unmap;
    clks = kcalloc(ndivs, sizeof(*clks), GFP_KERNEL);
    if(!clks)
        goto free_clkdata;

    clk_data->clks = clks;
    clk_data->clk_num = CLK_CNT;
	clks[CLK_REF] = of_clk_get_by_name(node, con_ids[CLK_REF]);
	if (IS_ERR(clks[CLK_REF])) {
		pr_err("%s: no external clock '%s' provided\n",
				__func__, con_ids[CLK_REF]);
		err = -ENODEV;
		goto free_clks;
	}
	err = clk_register_clkdev(clks[CLK_REF], con_ids[CLK_REF], NULL);
	if (err) {
		goto free_clks;
	}

	refclk_rate = clk_get_rate(clks[CLK_REF]);
	pr_debug("lsdbg--->ls2x_clk_setup refclk rate:%ld", refclk_rate);
	if (!refclk_rate) {
		err = -EINVAL;
		goto free_clks;
	}

#ifdef CONFIG_LOONGSON2_SYSCLK_HW_LOWFREQ
    int fixed_clks[] = {875000000, 875000000, 480000000, 320000000, 24000000,
    200000000, 100000000, 100000000, 125000000, 85000000, 85000000, 85000000};
    for (i = start_; i <= end_; i++) {
        clks[i] = clk_register_fixed_rate(NULL, con_ids[i], NULL, 0, fixed_clks[i-start_]);
        clk_register_clkdev(clks[i], con_ids[i], NULL);
    }
#endif

#ifdef CONFIG_LOONGSON2_SYSCLK_HW_HIGFREQ
    int fixed_clks[] = {1000000000, 1000000000, 600000000, 400000000, 24000000,
    400000000, 200000000, 200000000, 125000000, 150000000, 150000000, 150000000};
    for (i = start_; i <= end_; i++) {
        clks[i] = clk_register_fixed_rate(NULL, con_ids[i], NULL, 0, fixed_clks[i-start_]);
        clk_register_clkdev(clks[i], con_ids[i], NULL);
    }
#endif

#ifdef CONFIG_LOONGSON2_SYSCLK_SOFT
    {
        u64 ctrl;
        unsigned int l1div_out, l1div_loopc, l1div_ref;
        unsigned int l2div_out;
        unsigned int mult, div;
        struct clk_hw *node_clk_hw;
        struct clk_hw *cpu_clk_hw;
		#ifdef CONFIG_SND_LS1X_SOC_I2S
		struct clk_hw *i2s_mclk_hw;
		#endif

        node_clk_hw = clk_hw_register_pll(NULL, "node_clk", "ref_clk", &ls2x_node_clk_ops, 0);
        clks[CLK_NODE] = node_clk_hw->clk;
        clk_register_clkdev(clks[CLK_NODE], "node_clk", NULL);

        cpu_clk_hw = clk_hw_register_pll(NULL, "cpu_clk", "node_clk", &ls2x_cpu_clk_ops, 0);
        clks[CLK_CPU] = cpu_clk_hw->clk;
        clk_register_clkdev(clks[CLK_CPU], "cpu_clk", NULL);

        /* ddr gpu hda clk */
        ctrl = __raw_readq(reg + PLL_DLL0_OFF);
        l1div_out = (ctrl >> DDR_L1DIV_OUT_SHIFT) & DDR_L1DIV_OUT_MARK;
        l1div_loopc = (ctrl >> DDR_L1DIV_LOOPC_SHIFT) & DDR_L1DIV_LOOPC_MARK;
        l1div_ref = (ctrl >> DDR_L1DIV_REF_SHIFT) & DDR_L1DIV_REF_MARK;
        ctrl = __raw_readq(reg + PLL_DLL1_OFF);
        l2div_out = (ctrl >> DDR_L2DIV_OUT_DDR_SHIFT) & DDR_L2DIV_OUT_DDR_MARK;
        mult = l1div_loopc;
        div = l1div_ref * l2div_out;

        clks[CLK_DDR] = clk_register_fixed_factor(NULL, "ddr_clk", "ref_clk", 0, mult, div);
        clk_register_clkdev(clks[CLK_DDR], "ddr_clk", NULL);

        l2div_out = (ctrl >> DDR_L2DIV_OUT_GPU_SHIFT) & DDR_L2DIV_OUT_GPU_MARK;
        div = l1div_ref * l2div_out;
        clks[CLK_GPU] = clk_register_fixed_factor(NULL, "gpu_clk", "ref_clk", 0, mult, div);
        clk_register_clkdev(clks[CLK_GPU], "gpu_clk", NULL);

        l2div_out = (ctrl >> DDR_L2DIV_OUT_HDA_SHIFT) & DDR_L2DIV_OUT_HDA_MARK;
        div = l1div_ref * l2div_out;
        clks[CLK_HDA] = clk_register_fixed_factor(NULL, "hda_clk", "ref_clk", 0, mult, div);
        clk_register_clkdev(clks[CLK_HDA], "hda_clk", NULL);

        /* dc gmac clk */
        ctrl = __raw_readq(reg + PLL_DC0_OFF);
        l1div_out = (ctrl >> DC_L1DIV_OUT_SHIFT) & DC_L1DIV_OUT_MARK;
        l1div_loopc = (ctrl >> DC_L1DIV_LOOPC_SHIFT) & DC_L1DIV_LOOPC_MARK;
        l1div_ref = (ctrl >> DC_L1DIV_REF_SHIFT) & DC_L1DIV_REF_MARK;
        ctrl = __raw_readq(reg + PLL_DC1_OFF);
        l2div_out = (ctrl >> DC_L2DIV_OUT_DC_SHIFT) & DC_L2DIV_OUT_DC_MARK;
        mult = l1div_loopc;
        div = l1div_ref * l2div_out;

        clks[CLK_DC] = clk_register_fixed_factor(NULL, "dc_clk", "ref_clk", 0, mult, div);
        clk_register_clkdev(clks[CLK_DC], "dc_clk", NULL);

        l2div_out = (ctrl >> DC_L2DIV_OUT_GMAC_SHIFT) & DC_L2DIV_OUT_GMAC_MARK;
        div = l1div_ref * l2div_out;
        clks[CLK_GMAC] = clk_register_fixed_factor(NULL, "gmac_clk", "ref_clk", 0, mult, div);
        clk_register_clkdev(clks[CLK_GMAC], "gmac_clk", NULL);

        /* apb usb sata clk */
        ctrl = __raw_readq(reg + PLL_FREQ_SC);
        {
            unsigned int sata_freqscale;
            unsigned int usb_freqscale;
            unsigned int apb_freqscale;

            usb_freqscale = (ctrl >> FREQSCALE_USB_SHIFT) & FREQSCALE_USB_MARK;
            clks[CLK_USB] = clk_register_fixed_factor(NULL, "usb_clk", "gmac_clk", 0, usb_freqscale + 1, 8);
            clk_register_clkdev(clks[CLK_USB], "usb_clk", NULL);
            sata_freqscale = (ctrl >> FREQSCALE_SATA_SHIFT) & FREQSCALE_SATA_MARK;
            clks[CLK_SATA] = clk_register_fixed_factor(NULL, "sata_clk", "gmac_clk", 0, sata_freqscale + 1, 8);
            clk_register_clkdev(clks[CLK_SATA], "sata_clk", NULL);
            apb_freqscale = (ctrl >> FREQSCALE_APB_SHIFT) & FREQSCALE_APB_MARK;
            clks[CLK_APB] = clk_register_fixed_factor(NULL, "apb_clk", "gmac_clk", 0, apb_freqscale + 1, 8);
            clk_register_clkdev(clks[CLK_APB], "apb_clk", NULL);
        }
		#ifdef CONFIG_SND_LS1X_SOC_I2S
		i2s_mclk_hw = clk_hw_register_pll(NULL, "mclk", "apb_clk", &ls2x_i2s_mclk_ops, 0);
		clks[CLK_I2S_MCLK] = i2s_mclk_hw->clk;
		clk_register_clkdev(clks[CLK_I2S_MCLK], "mclk", NULL);
		#endif
	}
#endif

#ifdef CONFIG_LOONGSON2_SYSCLK_HW_BYPASS
    for (i = start_; i <= end_; i++) {
        clks[i] = clks[CLK_REF];
        clk_register_clkdev(clks[i], con_ids[i], NULL);
    }
#endif

    clks[CLK_SPI] = clks[CLK_REF];
	clk_register_clkdev(clks[CLK_SPI], con_ids[CLK_SPI], NULL);

	if (of_clk_add_provider(node, of_clk_src_onecell_get, clk_data)) {
		pr_err("%s: failed to add clock provider for\n",
		       __func__);
		goto free_clks;
	}

    return clks;

free_clks:
    kfree(clks);
free_clkdata:
    kfree(clk_data);
out_unmap:
    iounmap(reg);
    return NULL;
}

static void __init ls2x_clk_init(struct  device_node *node)
{
    ls2x_clk_setup(node);
}

CLK_OF_DECLARE(ls2x_clk, "loongson,ls2x-clk", ls2x_clk_init);