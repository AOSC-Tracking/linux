/*
 * Copyright (c) 2024 Loongson(GD) Inc.
 * Author: loongson-gz
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
#include <loongson-2k300.h>
#include <dt-bindings/clock/loongson2k300-clock.h>

#include "clk.h"

#define REF_CLK_NAME "ref_clk"
#define NODE_CLK_FREQ_DEF 1000000000

static unsigned long refclk_rate;
static void __iomem *reg;

static struct clk** clks = NULL;

#ifdef CONFIG_LOONGSON2_SYSCLK_SOFT

// 还没实现 CPU 变频支持 代码是从 2K500 那里拿过来的
#define CPU_CLK_CHANGE 0

#if 0
#define DEBUG_INFO(fmt, ...) printk(KERN_INFO fmt, ##__VA_ARGS__)
#else
#define DEBUG_INFO(fmt, ...)
#endif

#define PARENT_BRANCH 0
#define PARENT_CLONE 1

struct ls2k_reg_field_desc {
	unsigned int reg_offset;
	unsigned int bit_offset;
	unsigned int bit_mask;
};

typedef unsigned long (*clock_magnification_calu_func)(struct ls2k_reg_field_desc*);

/*
 * 本驱动中用于描述时钟结构的单个时钟的结构体，其组成的数组用于描述整个时钟结构
 * 其中不同的变量的值用于表示这个时钟的一些定义
 * name: 这是时钟的名字
 * parent_name: 这是时钟的上一层时钟的名字，比如DDR PLL的时钟 分频后 就是 CPU的时钟 那么可以说 CPU的 parent_name 是 Node PLL
 * parent_relevant: 和上一层时钟的关系，这个变量用于 fixed 为 1的时候的处理流程，主要是为了区分 clks* 怎么来
 *                  PARENT_BRANCH 代表是一个分支，PARENT_CLONE 代表是同一个时钟，而定义是不是分支主要是看两个时钟之间有没有分频或倍频
 *                  比如 DRR_PLL的时钟 分频后 获得 HDA 的时钟 那么就是 PARENT_BRANCH
 *                  比如 BOOT 的时钟 直接引到 BOOT SPI里面 那么就是 代表是一个分支, PARENT_CLONE
 * dts_desc: 这是一个重要的变量 为1 的时候，意味着要去dts里面找这个时钟的声明，比如参考时钟
 * essential_n: 在 dts_desc 为 1的时候，如果找不到该时钟，如果这个值为 1 将不会报错，只是会提示，不会影响该时钟管理驱动的初始化。例如 PCI的外部时钟，不提供也没什么事
 * fixed: 这个值为1的时候，代表这个时钟在本内核中是固定不变的 不为1的时候，就可以改变
 * *_reg_set 和 *_calu_func: 在 fixed == 1 的时候有效 *_reg_set 用于登记 分频 和 倍频的寄存器位域
 *								*calu_func是怎么处理这些寄存器的值的函数(就是怎么算系数) 不过我们也有提供通用的 ls2k_mul_calu ls2k_div_calu 等函数
 * clk_ops: 这个值就是一个结构体 需要填充 recalc_rate round_rate set_rate 函数的实现(linux这边要求的) 详见 ls2k300_node_clk_ops 和 ls2k300_cpu_clk_ops
 *
 * 所以 需要注意
 * name parent_name 是必填的
 * dts_desc 为 1的时候 相关的要填的变量是 essential_n(可选 不填的话 static 下 自动为0) 其他变量不起效
 * parent_relevant 为 PARENT_BRANCH 的时候
 *                 fixed    为 1的时候 *_reg_set 和 *_calu_func(可选 不填的话 static 下 自动为0) 其他变量不起效
 *                 fixed    为 0的时候 clk_ops(一定要实现) 其他变量不起效
 * parent_relevant 为 PARENT_CLONE
 *                 parent_name 不能为空 其他变量不起效
 *
 * *_reg_set 声明的时候 必须在在末尾添加 {.bit_mask = 0} 这是判断数组结束的标志
 * *_calu_func 可以为空
 *
 * static 声明 ls2k_clock_desc数组时, 可不填的变量 就是 0
 */
struct ls2k_clock_desc {
	char* name;
	char* parent_name;
	int parent_relevant;  //branch or clone if branch reg_set or clk_ops enable if clone just clockA == clockB
	int dts_desc;
	int essential_n;
	int fixed; // if fixed reg_set enable or clk_ops enable

	struct ls2k_reg_field_desc* mul_reg_set;
	clock_magnification_calu_func mul_calu_func;
	struct ls2k_reg_field_desc* div_reg_set;
	clock_magnification_calu_func div_calu_func;
	struct ls2k_reg_field_desc* freqscale_reg_set;
	clock_magnification_calu_func freqscale_mul_calu_func;
	clock_magnification_calu_func freqscale_div_calu_func;

	struct clk_ops* clk_ops;
};

#if (CPU_CLK_CHANGE == 1)

struct pll_config {
	unsigned int l1_div_ref;
	unsigned int l1_loopc;
	unsigned int l1_odiv;
};

// 系统时钟需满足如下几个限制：
// 1.可配分频器的输出 refclk/L1_div_ref 在 20~40MHz 范围内
// 2.PLL 倍频值 refclk/L1_div_ref*L1_loopc 需要在 1.2GHz~3.2GHz
static int ls2k_caculate_freq(unsigned long freq, struct pll_config *pll_cfg)
{
	unsigned long pll_in_clk;
	unsigned long loopc;
	unsigned long div;
	int found = 0;

	if (!pll_cfg)
		return -EINVAL;

	// Keep refclk/L1_div_ref in range 20MHz ~ 40Mhz
	pll_cfg->l1_div_ref = 4;
	pll_in_clk = refclk_rate >> 2;		// 25MHz // 100/4 = 25

	// 2*7 = 14 1.4Ghz 3*7 = 21 2.1GHz
	// 3*6 = 18 1.8GHz
	if(freq >= 700000000)
		div = 2;
	else
		div = 3;

	//keep 1.2 ~ 3.2
	// and keep 1.2 ~ 2.1 will be better
	loopc = (freq * div) / pll_in_clk;

	if (!found) {
		pll_cfg->l1_div_ref = 0;
		pll_cfg->l1_loopc = 0;
		pll_cfg->l1_odiv = 0;
		return -EINVAL;
	}

	pll_cfg->l1_loopc = loopc;
	pll_cfg->l1_odiv = div;
	return 0;
}

//PLL 寄存器设置 用于 时钟频率设置生效
/*
 * 注意手册上写的 3.5.2 软件配置
 * 第二步和第三步是一起的，也就是代码中
 * li.w	t1, (NODE_DIV << 24) | (NODE_LOOPC << 16) | (NODE_REFC << 8)
 * 没有管 pd_pll 会被设置成 0 的原因
 */
static void ls2k_config_pll(void __iomem *pll_base, struct pll_config *pll_cfg)
{
	int timeout = 200;
	u32 val;

	/* pd_pll 1 */
	val = __raw_readl(pll_base);
	val |= BIT(5);
	__raw_writel(val, pll_base);

	/* 除了 pll_sel_* [2:0] 设置为0 pll_soft_set (3) 设置为0 其他的寄存器按需设置*/
	/* pd_pll 0 */
	val = (pll_cfg->l1_odiv << PLL_GENERIC_0_ODIV_SHIFT) |
			(pll_cfg->l1_loopc << PLL_GENERIC_0_LOOPC_SHIFT) |
			(pll_cfg->l1_div_ref << PLL_GENERIC_0_REFC_SHIFT);
	__raw_writel(val, pll_base);

	/* 其他信号不变 pll_soft_set (3) 信号设置为 1 */
	val = __raw_readl(pll_base);
	val |= BIT(3);
	__raw_writel(val, pll_base);

	/* 等待 pll_lock (7) 锁定 也就是变成 1 */
	while(!(__raw_readl(pll_base) & BIT(7)) && (timeout-- > 0)) {
		udelay(1);
	}

	/* pll_sel_* [2:0] 设置为1 */
	val = __raw_readl(pll_base);
	val |= 0x7;
	__raw_writel(val, pll_base);
}


// 这是计算当前频率的
static unsigned long ls2k300_node_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	u32 ctrl;
	unsigned int div_loopc, div_ref, odiv;
	unsigned int mult, div;
	unsigned long rate;

	/* node cpu clk */
	ctrl = __raw_readl(reg + PLL_NODE_0_OFFSET);
	div_loopc = (ctrl >> PLL_NODE_0_LOOPC_SHIFT) & PLL_NODE_0_LOOPC_MASK;
	div_ref = (ctrl >> PLL_NODE_0_REFC_SHIFT) & PLL_NODE_0_REFC_MASK;
	odiv = (ctrl >> PLL_NODE_0_ODIV_SHIFT) & PLL_NODE_0_ODIV_MASK;
	mult = div_loopc;
	// div = div_ref * odiv;
	div = div_ref;
	if (div == 0) {
		rate = NODE_CLK_FREQ_DEF;
	} else {
		rate = refclk_rate * mult / div;
	}

	return rate;
}

static long ls2k300_node_round_rate(struct clk_hw *hw, unsigned long rate,
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
	div = pll_cfg.l1_div_ref * pll_cfg.l1_odiv;

	return refclk_rate * mult / div;
}

static int ls2k300_node_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct pll_config pll_cfg;

	if (rate == 0 || parent_rate == 0)
		return -EINVAL;

	if (ls2k_caculate_freq(rate, &pll_cfg)) {
		return -EINVAL;
	}

	ls2k_config_pll(reg + PLL_NODE_0_OFFSET, &pll_cfg);
	return 0;
}

static struct clk_ops ls2k300_node_clk_ops = {
	.recalc_rate = ls2k300_node_recalc_rate,
	.round_rate = ls2k300_node_round_rate,
	.set_rate = ls2k300_node_set_rate,
};

static unsigned long ls2k300_cpu_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	unsigned int freqscale;
	u32 val;

	val = __raw_readl(reg + PLL_FREQ_SC);
	freqscale = (val >> PLL_FREQSCALE_NODE_SHIFT) & PLL_FREQSCALE_NODE_MASK;
	++freqscale;
	freqscale >>= 3;  // /8

	return parent_rate / freqscale;
}

static long ls2k300_cpu_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	int scale;
	if (!rate || !*parent_rate)
		return 0;

	scale = rate * 8 / (*parent_rate) - 1;
	return (*parent_rate) * (scale + 1) / 8;
}

static int ls2k300_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	int scale;
	u32 val;

	if (!rate || !parent_rate)
		return -EINVAL;

	scale = rate * 8 / parent_rate - 1;
	val = __raw_readl(reg + PLL_FREQ_SC);
	val &= (~PLL_FREQSCALE_NODE_MASK) << PLL_FREQSCALE_NODE_SHIFT;
	val |= (scale & PLL_FREQSCALE_NODE_MASK) << PLL_FREQSCALE_NODE_SHIFT;
	__raw_writel(val, reg + PLL_FREQ_SC);
	return 0;
}

static struct clk_ops ls2k300_cpu_clk_ops = {
	.recalc_rate = ls2k300_cpu_recalc_rate,
	.round_rate = ls2k300_cpu_round_rate,
	.set_rate = ls2k300_cpu_set_rate,
};
#endif // end #if (CPU_CLK_CHANGE == 1)

//通用的倍率计算函数 不管是乘法还是除法
static unsigned int ls2k_generic_magnification_calu(struct ls2k_reg_field_desc* reg_set)
{
	int i = 0;
	unsigned int total = 1;
	while(1) {
		unsigned int reg_value, value;
		if(!reg_set[i].bit_mask)
			break;
		reg_value = __raw_readl(reg + reg_set[i].reg_offset);
		value = (reg_value >> reg_set[i].bit_offset) & reg_set[i].bit_mask;
		total *= value;
		++i;
	}
	return total;
}

static unsigned int ls2k_mul_calu(struct ls2k_reg_field_desc* reg_set)
{
	return ls2k_generic_magnification_calu(reg_set);
}

static unsigned int ls2k_div_calu(struct ls2k_reg_field_desc* reg_set)
{
	return ls2k_generic_magnification_calu(reg_set);
}

//通用的freqscale 乘法部分计算 需要 + 1 (7+1)
// 最高bit 作为 freq_mode
// freq_mode = 0 fout=fin*(freqscale[2:0]+1)/8
// freq_mode = 1 fout=fin/(freqscale[2:0]+1)
static unsigned long ls2k_freqscale_mul_calu(struct ls2k_reg_field_desc* reg_set)
{
	int i = 0;
	unsigned int total = 1;
	while(1) {
		unsigned int reg_value, value, mode;
		if(!reg_set[i].bit_mask)
			break;
		reg_value = __raw_readl(reg + reg_set[i].reg_offset);
		value = (reg_value >> reg_set[i].bit_offset) & reg_set[i].bit_mask;

		// 判断最高bit 长度为 4, 右移 3 bit 就知道mode了
		mode = value >> 3;
		if (mode)
			total *= 1; // fout=fin/(freqscale[2:0]+1) 也就是 x1
		else
			total *= ((value & 0x7) + 1); // fout=fin*(freqscale[2:0]+1)/8 也就是 低 3 bit 有效
		++i;
	}
	return total;
}

//通用的freqscale 除法部分计算 按照位域的最大值 + 1 比如 usb 就是 (freqscale + 1 / 8) 这个8就是返回值
// 这是 最高bit 作为 freq_mode
// freq_mode = 0 fout=fin*(freqscale[2:0]+1)/8
// freq_mode = 1 fout=fin/(freqscale[2:0]+1)
static unsigned long ls2k_freqscale_div_calu(struct ls2k_reg_field_desc* reg_set)
{
	int i = 0;
	unsigned int total = 1;
	while(1) {
		unsigned int reg_value, value, mode;
		if(!reg_set[i].bit_mask)
			break;
		reg_value = __raw_readl(reg + reg_set[i].reg_offset);
		value = (reg_value >> reg_set[i].bit_offset) & reg_set[i].bit_mask;

		// 判断最高bit 长度为 4, 右移 3 bit 就知道mode了
		mode = value >> 3;

		if (mode)
			total *= ((value & 0x7) + 1); // fout=fin/(freqscale[2:0]+1) 也就是 低 3 bit 有效
		else
			total *= 8; // fout=fin*(freqscale[2:0]+1)/8 也就是 / 8
		++i;
	}
	return total;
}

static unsigned long ls2k300_sdio_freqscale_mul_calu(struct ls2k_reg_field_desc* reg_set)
{
	int i = 0;
	unsigned int total = 1;
	while(1) {
		if(!reg_set[i].bit_mask)
			break;

		// CLK/freqscale[27:24] 所以 * 1
		total *= 1;
		++i;
	}
	return total;
}

static unsigned long ls2k300_sdio_freqscale_div_calu(struct ls2k_reg_field_desc* reg_set)
{
	int i = 0;
	unsigned int total = 1;
	while(1) {
		unsigned int reg_value, value;
		if(!reg_set[i].bit_mask)
			break;
		reg_value = __raw_readl(reg + reg_set[i].reg_offset);
		value = (reg_value >> reg_set[i].bit_offset) & reg_set[i].bit_mask;

		// CLK/freqscale[27:24] 是 0 或者 1 都是除以 1，也就是不分频
		if (!value)
			value = 1;

		total /= value;
		++i;
	}
	return total;
}

/*
 * @brief: 倍频系数和分频系数的计算函数
 *         逻辑上就是对*_reg_set分析 把全部倍频系数 乘起来 把全部分频系数 乘起来
 *         然后 赋值到 参数 mul_result 和 div_result
 * @param[in]: 参数1(cur_clock): 这个要计算的时钟的描述结构体 里面包含 *_reg_set
 * @param[out]: 参数2(mul_result): 倍频的总系数 需要传入一个 unsigned long 的变量的地址 作为结果返回
 * @param[out]: 参数3(div_result): 分频的总系数 需要传入一个 unsigned long 的变量的地址 作为结果返回
 * @return: 参数传进来的指针有一个为NULL 返回 -EFAULT 否则返回 0 代表计算成功
 */
static int ls2k_mul_div_total_calu(struct ls2k_clock_desc* cur_clock, unsigned long* mul_result, unsigned long* div_result)
{
	unsigned long mul = 1, div = 1, freq_sc_mul = 1, freq_sc_div = 1;

	if(!mul_result || !div_result)
		return -EFAULT;
	if(!cur_clock)
		return -EFAULT;

	if(cur_clock->mul_reg_set)
		mul = cur_clock->mul_calu_func ? cur_clock->mul_calu_func(cur_clock->mul_reg_set)
				: ls2k_mul_calu(cur_clock->mul_reg_set);
	if(cur_clock->div_reg_set)
		div = cur_clock->div_calu_func ? cur_clock->div_calu_func(cur_clock->div_reg_set)
				: ls2k_div_calu(cur_clock->div_reg_set);
	if(cur_clock->freqscale_reg_set) {
		freq_sc_mul = cur_clock->freqscale_mul_calu_func ? cur_clock->freqscale_mul_calu_func(cur_clock->freqscale_reg_set)
					: ls2k_freqscale_mul_calu(cur_clock->freqscale_reg_set);
		freq_sc_div = cur_clock->freqscale_div_calu_func ? cur_clock->freqscale_div_calu_func(cur_clock->freqscale_reg_set)
					: ls2k_freqscale_div_calu(cur_clock->freqscale_reg_set);
	}
	mul *= freq_sc_mul;
	div *= freq_sc_div;
	mul_result[0] = mul;
	div_result[0] = div;
	return 0;
}

/*
 * 时钟的描述结构体的数组 用于描述 整个时钟结构
 * 注意 必须要让父节点先于子节点声明，否则在处理逻辑中会引发bug
 * 参考时钟 REF_CLK_NAME 必须是第一个变量
 */
static struct ls2k_clock_desc ls2k_clock_desc_set[CLK_CNT] = {
	{
		.name = REF_CLK_NAME,
		.parent_name = NULL,
		.dts_desc = 1,
	},

	// node pll
	{
		.name = "node_pll_clk",
		.parent_name = "ref_clk",
		.parent_relevant = PARENT_BRANCH,
#if (CPU_CLK_CHANGE == 1)
		// 还没实现
		.clk_ops = &ls2k300_node_clk_ops,
#else
		.fixed = 1,
		.mul_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_0_OFFSET, .bit_offset = PLL_NODE_0_LOOPC_SHIFT, .bit_mask = PLL_NODE_0_LOOPC_MASK}, {.bit_mask = 0}},
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_0_OFFSET, .bit_offset = PLL_NODE_0_REFC_SHIFT, .bit_mask = PLL_NODE_0_REFC_MASK}, {.bit_mask = 0}},
#endif
	},
	{
		.name = "cpu_clk",
		.parent_name = "node_pll_clk",
		.parent_relevant = PARENT_BRANCH,
#if (CPU_CLK_CHANGE == 1)
		// 还没实现
		.clk_ops = &ls2k300_cpu_clk_ops,
#else
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_0_OFFSET, .bit_offset = PLL_NODE_0_ODIV_SHIFT, .bit_mask = PLL_NODE_0_ODIV_MASK}, {.bit_mask = 0}},
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_NODE_SHIFT, .bit_mask = PLL_FREQSCALE_NODE_MASK}, {.bit_mask = 0}},
#endif
	},
	{
		.name = "scache_clk",
		.parent_name = "node_pll_clk",
		.parent_relevant = PARENT_BRANCH,
#if (CPU_CLK_CHANGE == 1)
		// 还没实现
		.clk_ops = &ls2k300_cpu_clk_ops,
#else
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_0_OFFSET, .bit_offset = PLL_NODE_0_ODIV_SHIFT, .bit_mask = PLL_NODE_0_ODIV_MASK}, {.bit_mask = 0}},
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_NODE_SHIFT, .bit_mask = PLL_FREQSCALE_NODE_MASK}, {.bit_mask = 0}},
#endif
	},
	{
		.name = "iodma_clk",
		.parent_name = "node_pll_clk",
		.parent_relevant = PARENT_BRANCH,
#if (CPU_CLK_CHANGE == 1)
		// 还没实现
		.clk_ops = &ls2k300_cpu_clk_ops,
#else
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_0_OFFSET, .bit_offset = PLL_NODE_0_ODIV_SHIFT, .bit_mask = PLL_NODE_0_ODIV_MASK}, {.bit_mask = 0}},
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_NODE_SHIFT, .bit_mask = PLL_FREQSCALE_NODE_MASK}, {.bit_mask = 0}},
#endif
	},
	{
		.name = "gmac_clk",
		.parent_name = "node_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_1_OFFSET, .bit_offset = PLL_NODE_1_ODIV_GMAC_SHIFT, .bit_mask = PLL_NODE_1_ODIV_GMAC_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "i2s_clk",
		.parent_name = "node_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_NODE_1_OFFSET, .bit_offset = PLL_NODE_1_ODIV_I2S_SHIFT, .bit_mask = PLL_NODE_1_ODIV_I2S_MASK}, {.bit_mask = 0}},
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_I2S_SHIFT, .bit_mask = PLL_FREQSCALE_I2S_MASK}, {.bit_mask = 0}},
	},

	// ddr pll
	{
		.name = "ddr_pll_clk",
		.parent_name = "ref_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.mul_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_DDR_0_OFFSET, .bit_offset = PLL_DDR_0_LOOPC_SHIFT, .bit_mask = PLL_DDR_0_LOOPC_MASK}, {.bit_mask = 0}},
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_DDR_0_OFFSET, .bit_offset = PLL_DDR_0_REFC_SHIFT, .bit_mask = PLL_DDR_0_REFC_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "ddr_clk",
		.parent_name = "ddr_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_DDR_0_OFFSET, .bit_offset = PLL_DDR_0_ODIV_SHIFT, .bit_mask = PLL_DDR_0_ODIV_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "net_clk",
		.parent_name = "ddr_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_DDR_1_OFFSET, .bit_offset = PLL_DDR_1_ODIV_NET_SHIFT, .bit_mask = PLL_DDR_1_ODIV_NET_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "devs_clk",
		.parent_name = "ddr_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_DDR_1_OFFSET, .bit_offset = PLL_DDR_1_ODIV_DEVS_SHIFT, .bit_mask = PLL_DDR_1_ODIV_DEVS_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "usb_clk",
		.parent_name = "devs_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_USB_SHIFT, .bit_mask = PLL_FREQSCALE_USB_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "apb_clk",
		.parent_name = "devs_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_APB_SHIFT, .bit_mask = PLL_FREQSCALE_APB_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "boot_clk",
		.parent_name = "devs_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_BOOT_SHIFT, .bit_mask = PLL_FREQSCALE_BOOT_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "sdio_clk",
		.parent_name = "devs_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.freqscale_mul_calu_func = ls2k300_sdio_freqscale_mul_calu,
		.freqscale_div_calu_func = ls2k300_sdio_freqscale_div_calu,
		.freqscale_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_FREQ_SC, .bit_offset = PLL_FREQSCALE_SDIO_SHIFT, .bit_mask = PLL_FREQSCALE_SDIO_MASK}, {.bit_mask = 0}},
	},

	// pix pll
	{
		.name = "pix_pll_clk",
		.parent_name = "ref_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.mul_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_PIX_0_OFFSET, .bit_offset = PLL_PIX0_0_LOOPC_SHIFT, .bit_mask = PLL_PIX0_0_LOOPC_MASK}, {.bit_mask = 0}},
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_PIX_0_OFFSET, .bit_offset = PLL_PIX0_0_REFC_SHIFT, .bit_mask = PLL_PIX0_0_REFC_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "pix_clk",
		.parent_name = "pix_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_PIX_0_OFFSET, .bit_offset = PLL_PIX0_0_ODIV_SHIFT, .bit_mask = PLL_PIX0_0_ODIV_MASK}, {.bit_mask = 0}},
	},
	{
		.name = "gmacbp_clk",
		.parent_name = "pix_pll_clk",
		.parent_relevant = PARENT_BRANCH,
		.fixed = 1,
		.div_reg_set = (struct ls2k_reg_field_desc[]){{.reg_offset = PLL_PIX_1_OFFSET, .bit_offset = PLL_PIX0_1_ODIV_GMACBP_SHIFT, .bit_mask = PLL_PIX0_1_ODIV_GMACBP_MASK}, {.bit_mask = 0}},
	},
};

/*
 * @brief: CONFIG_LOONGSON2_SYSCLK_SOFT 的情况下 ls2k300_clk_setup 函数要做的主要任务
 *         扫描上面的 ls2k_clock_desc_set 得到整个时钟结构
 *         然而第一个时钟默认不用扫，在 ls2k300_clk_setup 开头就会对 参数时钟中注册
 * @param[in]: 参数1(node): 这是dts中 始终管理驱动节点的信息 例如这个 clocks:clock-controller@1fe10400
 *                          用于扫描dts中指定的外部时钟
 * @return: 如果成功返回 0
 *          clks 为空则返回 -EFAULT
 *          node 为空 | dts里面没有要扫描的必要的时钟声明 | ls2k_clock_desc_set里面的某一个 PARENT_CLONE 的 父节点不存在 返回 -ENODEV
 */
static int register_clock(struct device_node *node)
{
	int i = 1; // ref clk is registered!
	struct ls2k_clock_desc* cur_clock;
	unsigned long rate;

	if(!clks) {
		pr_err("%s func %s: ls2k300_clks kalloc failed\n", __FILE__, __func__);
		return -EFAULT;
	}

	for(; i < CLK_CNT; ++i) {
		int err;
		cur_clock = ls2k_clock_desc_set + i;
		clks[i] = NULL;
		if(cur_clock->dts_desc) {
			//dts 扫描
			if(!node && !cur_clock->essential_n) {
				pr_err("%s func %s: device node is NULL can't scan dts\n", __FILE__, __func__);
				return -ENODEV;
			}

			clks[i] = of_clk_get_by_name(node, cur_clock->name);
			if (IS_ERR(clks[i])) {
				pr_err("%s func %s: not found %s clock in device tree or maybe you should add this clock in %s clock-names key\n",
						__FILE__, __func__, cur_clock->name, node->full_name);
				if(cur_clock->essential_n) {
					clks[i] = NULL;
					continue;
				} else
					return -ENODEV;
			}
			err = clk_register_clkdev(clks[i], cur_clock->name, NULL);
			if (err && !cur_clock->essential_n)
				return err;
		}
		else if(cur_clock->parent_relevant == PARENT_BRANCH) {
			if(cur_clock->fixed) {
				//从父节点有分频倍频而来的的时钟 且固定不变
				unsigned long mul_total = 1, div_total = 1;
				ls2k_mul_div_total_calu(cur_clock, &mul_total, &div_total);
				clks[i] = clk_register_fixed_factor(NULL, cur_clock->name, cur_clock->parent_name, 0, mul_total, div_total);
				clk_register_clkdev(clks[i], cur_clock->name, NULL);
			}
			else {
				//从父节点有分频倍频而来的的时钟 且会改变
				struct clk_hw *cur_cpu_hw;
				cur_cpu_hw = clk_hw_register_pll(NULL, cur_clock->name, cur_clock->parent_name, cur_clock->clk_ops, 0);
				clks[i] = cur_cpu_hw->clk;
				clk_register_clkdev(clks[i], cur_clock->name, NULL);
			}
		}
		else if(cur_clock->parent_relevant == PARENT_CLONE) {
			//从父节点直接使用的时钟 同一个时钟
			int parent_index = 0;
			//null name not found
			if(!cur_clock->parent_name)
				parent_index = i;
			for(; parent_index < i; ++parent_index) {
				if(!strcmp(cur_clock->parent_name, ls2k_clock_desc_set[parent_index].name)) {
					clks[i] = clks[parent_index];
					clk_register_clkdev(clks[i], cur_clock->name, NULL);
				}
			}
			//not found parent
			if(parent_index == i) {
				//not found
				printk(KERN_INFO "%s func %s: warning: clk not found parent clk, name is %s, parent is %s\n",
						__FILE__, __func__, cur_clock->name, cur_clock->parent_name);
				if(!cur_clock->essential_n)
					return -ENODEV;
			}
		}
		else
			printk(KERN_INFO "%s func %s: warning: clk register type unkonwn! name is %s\n", __FILE__, __func__, cur_clock->name);

		if(clks[i]) {
			rate = clk_get_rate(clks[i]);
			//文件开头 #if 判断开的话 就能看到所有扫描的时钟的信息
			DEBUG_INFO("cur clock info name:%s\tpname:%s\trate:%ld\tdts_desc:%d\tparent_relevant:%d\tfixed:%d\n",
						cur_clock->name, cur_clock->parent_name, rate, cur_clock->dts_desc, cur_clock->parent_relevant, cur_clock->fixed);
		}
	}
	return 0;
}
#endif

static struct clk ** __init ls2k300_clk_setup(struct device_node *node)
{
#ifndef CONFIG_LOONGSON2_SYSCLK_SOFT
	int start_ = CLK_NODE;
	int end_ = CLK_PCI;
	int i = 0;

	char *con_ids[] = { REF_CLK_NAME,
							"node_pll_clk", "cpu_clk", "scache_clk", "iodma_clk", "gmac_clk", "i2s_clk"
							"ddr_pll_clk", "ddr_clk", "net_clk", "devs_clk", "usb_clk", "apb_clk", "boot_clk", "sdio_clk",
							"pix_pll_clk", "pix_clk", "gmacbp_clk"
							};
#endif
	struct clk_onecell_data *clk_data;
	int ndivs = CLK_CNT;
	int err = -EINVAL;

	reg = of_iomap(node, 0);
	if(!reg) {
		pr_err("%s: Could not map registers for divs-clk: %s\n", __func__, node->full_name);
		return NULL;
	}
	pr_debug("lsdbg---> %s reg:0x%lx", __func__, (unsigned long)reg);

	clk_data = kmalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
	if(!clk_data)
		goto out_unmap;
	clks = kcalloc(ndivs, sizeof(*clks), GFP_KERNEL);
	if(!clks)
		goto free_clkdata;

	clk_data->clks = clks;
	clk_data->clk_num = CLK_CNT;

	clks[CLK_REF] = of_clk_get_by_name(node, REF_CLK_NAME);
	if (IS_ERR(clks[CLK_REF])) {
		pr_err("%s func %s: not found %s clock in device tree or maybe you should add this clock in %s clock-names key\n",
				__FILE__, __func__, REF_CLK_NAME, node->full_name);
		err = -ENODEV;
		goto free_clks;
	}
	err = clk_register_clkdev(clks[CLK_REF], REF_CLK_NAME, NULL);
	if (err) {
		goto free_clks;
	}

	refclk_rate = clk_get_rate(clks[CLK_REF]);
	pr_debug("lsdbg--->%s refclk rate:%ld", __func__, refclk_rate);
	if (!refclk_rate) {
		err = -EINVAL;
		goto free_clks;
	}

#ifdef CONFIG_LOONGSON2_SYSCLK_HW_LOWFREQ
	{
		int fixed_clks[] = {120000000,
							750000000, 750000000, 750000000, 750000000, 125000000, 750000000,
							800000000, 800000000, 266000000, 100000000, 100000000, 100000000, 100000000,
							100000000, 100000000, 100000000 };
		for (i = start_; i <= end_; i++) {
			clks[i] = clk_register_fixed_rate(NULL, con_ids[i], NULL, 0, fixed_clks[i-start_]);
			clk_register_clkdev(clks[i], con_ids[i], NULL);
		}
	}
#endif

#ifdef CONFIG_LOONGSON2_SYSCLK_HW_HIGFREQ
	{
		int fixed_clks[] = {120000000,
							1000000000, 1000000000, 1000000000, 1000000000, 125000000, 1000000000,
							1200000000, 1200000000, 400000000, 150000000, 150000000, 150000000, 150000000,
							200000000, 200000000, 200000000
							};
		for (i = start_; i <= end_; i++) {
			clks[i] = clk_register_fixed_rate(NULL, con_ids[i], NULL, 0, fixed_clks[i-start_]);
			clk_register_clkdev(clks[i], con_ids[i], NULL);
		}
	}
#endif

#ifdef CONFIG_LOONGSON2_SYSCLK_SOFT
	if(register_clock(node))
		goto free_clks;
#endif

#ifdef CONFIG_LOONGSON2_SYSCLK_HW_BYPASS
	for (i = start_; i <= end_; i++) {
		clks[i] = clks[CLK_REF];
		clk_register_clkdev(clks[i], con_ids[i], NULL);
	}
#endif

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

static void __init ls2k300_clk_init(struct  device_node *node)
{
	ls2k300_clk_setup(node);
}

CLK_OF_DECLARE(ls2k300_clk, "loongson,ls2k300-clk", ls2k300_clk_init);
