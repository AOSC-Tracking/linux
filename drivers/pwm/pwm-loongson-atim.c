// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 by ilikara 3435193369@qq.com.
 *
 * Loongson ATIM PWM driver
 *
 * For Loongson's ATIM IP block documentation please refer Chapter 18 of
 * Reference Manual: https://loongson.cn/uploads/images/2024120211205269050.%E9%BE%99%E8%8A%AF2K0300%E5%A4%84%E7%90%86%E5%99%A8%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C_V1.0.pdf
 *
 * Author: Ilikara <3435193369@qq.com>
 *
 * Limitations:
 * - ATIM_CNT will be set to 0 when ls_pwm_atim_config is called.
 * - All the 8 channels share the same period.
 * - Channel n and n+4 share the same duty cycle.
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/units.h>
#include <linux/delay.h>

/* counter offest */
#define ATIM_CR1 0x00
#define ATIM_CR2 0x04
#define ATIM_SMCR 0x08
#define ATIM_DIER 0x0C
#define ATIM_SR 0x10
#define ATIM_EGR 0x14
#define ATIM_CCMR(n) 0x18 + (((n) % 4) / 2) * 0x04
#define ATIM_CCER 0x20
#define ATIM_CNT 0x24
#define ATIM_PSC 0x28
#define ATIM_ARR 0x2C
#define ATIM_RCR 0x30
#define ATIM_CCR(n) 0x34 + ((n) % 4) * 0x04
#define ATIM_BDTR 0x44
#define ATIM_INSTA 0x50

/* CR1 each bit */
#define CR1_CEN BIT(0)

/* EGR each bit */
#define EGR_UG BIT(0)

/* CCMR each bit */
#define CCMR_CCnS(n) BIT((n) % 2 * 8 + 0)
#define CCMR_OCnFE(n) BIT((n) % 2 * 8 + 2)
#define CCMR_OCnPE(n) BIT((n) % 2 * 8 + 3)
#define CCMR_OCnM(n) BIT((n) % 2 * 8 + 4)
#define CCMR_OCnCE(n) BIT((n) % 2 * 8 + 7)

/* CCER each bit */
#define CCER_CCnE(n) BIT(((n) % 4) * 4 + 0 + ((n) / 4) * 2)
#define CCER_CCnP(n) BIT(((n) % 4) * 4 + 1 + ((n) / 4) * 2)

/* BDTR each bit */
#define BDTR_MOE BIT(15)

/* default input clk frequency for the ACPI case */
#define LOONGSON_PWM_FREQ_DEFAULT 50000000 /* Hz */

struct pwm_loongson_suspend_store {
	u32 arr_reg;
	u32 ccr_reg[4];
	u32 ccer_reg;
	u32 en_mark;
};

struct pwm_loongson_ddata {
	struct clk *clk;
	void __iomem *base;
	u64 clk_rate;
	struct pwm_loongson_suspend_store lss;
};

static inline struct pwm_loongson_ddata *
to_pwm_loongson_ddata(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static inline u32 pwm_loongson_readl(struct pwm_loongson_ddata *ddata,
				     u32 offset)
{
	return readl(ddata->base + offset);
}

static inline void pwm_loongson_writel(struct pwm_loongson_ddata *ddata,
				       u32 val, u32 offset)
{
	writel(val, ddata->base + offset);
}

static int ls_pwm_atim_set_polarity(struct pwm_chip *chip,
				    struct pwm_device *pwm,
				    enum pwm_polarity polarity)
{
	u16 val;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	val = pwm_loongson_readl(ddata, ATIM_CCER);

	if (polarity == PWM_POLARITY_INVERSED)
		/* Duty cycle defines LOW period of PWM */
		val &= ~CCER_CCnP(pwm->hwpwm);
	else
		/* Duty cycle defines HIGH period of PWM */
		val |= CCER_CCnP(pwm->hwpwm);

	pwm_loongson_writel(ddata, val, ATIM_CCER);

	return 0;
}

static void ls_pwm_atim_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);
	u32 val;

	if (pwm->state.polarity == PWM_POLARITY_NORMAL)
		pwm_loongson_writel(ddata, ddata->lss.arr_reg,
				    ATIM_CCR(pwm->hwpwm));
	else if (pwm->state.polarity == PWM_POLARITY_INVERSED)
		pwm_loongson_writel(ddata, 0, ATIM_CCR(pwm->hwpwm));

	val = pwm_loongson_readl(ddata, ATIM_CCMR(pwm->hwpwm));
	val &= ~(0b111 * CCMR_OCnM(pwm->hwpwm));
	pwm_loongson_writel(ddata, val, ATIM_CCMR(pwm->hwpwm));

	val = pwm_loongson_readl(ddata, ATIM_CCER);
	val &= ~CCER_CCnE(pwm->hwpwm);
	pwm_loongson_writel(ddata, val, ATIM_CCER);

	ddata->lss.en_mark &= ~BIT(pwm->hwpwm);
	if (ddata->lss.en_mark == 0b00000000) {
		val = pwm_loongson_readl(ddata, ATIM_CR1);
		val &= ~CR1_CEN;
		pwm_loongson_writel(ddata, val, ATIM_CR1);
		pwm_loongson_writel(ddata, 0x0, ATIM_CNT);
	}
}

static int ls_pwm_atim_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	pwm_loongson_writel(ddata, ddata->lss.ccr_reg[pwm->hwpwm % 4],
			    ATIM_CCR(pwm->hwpwm));
	pwm_loongson_writel(ddata, ddata->lss.arr_reg, ATIM_ARR);

	// todo:类似config函数中的特殊处理
	val = pwm_loongson_readl(ddata, ATIM_CCMR(pwm->hwpwm));
	val |= 0b111 * CCMR_OCnM(pwm->hwpwm);
	pwm_loongson_writel(ddata, val, ATIM_CCMR(pwm->hwpwm));

	val = pwm_loongson_readl(ddata, ATIM_CCER);
	val |= CCER_CCnE(pwm->hwpwm);
	pwm_loongson_writel(ddata, val, ATIM_CCER);

	if (ddata->lss.en_mark == 0b00000000) {
		pwm_loongson_writel(ddata, 0x0, ATIM_CNT);
		val = pwm_loongson_readl(ddata, ATIM_CR1);
		val |= CR1_CEN;
		pwm_loongson_writel(ddata, val, ATIM_CR1);
	}
	ddata->lss.en_mark |= BIT(pwm->hwpwm);

	return 0;
}

static int ls_pwm_atim_config(struct pwm_chip *chip, struct pwm_device *pwm,
			      u64 duty_ns, u64 period_ns)
{
	u32 duty, period;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	/* duty = duty_ns * ddata->clk_rate / NSEC_PER_SEC */
	duty = mul_u64_u64_div_u64(duty_ns, ddata->clk_rate, NSEC_PER_SEC);
	pwm_loongson_writel(ddata, duty, ATIM_CCR(pwm->hwpwm));

	/* period = period_ns * ddata->clk_rate / NSEC_PER_SEC */
	period = mul_u64_u64_div_u64(period_ns, ddata->clk_rate, NSEC_PER_SEC);
	pwm_loongson_writel(ddata, period, ATIM_ARR);

	pwm_loongson_writel(ddata, 0, ATIM_CNT);

	ddata->lss.ccr_reg[pwm->hwpwm % 4] = duty;
	ddata->lss.arr_reg = period;

	return 0;
}

static int ls_pwm_atim_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	int ret;
	u64 period, duty_cycle;
	bool enabled = pwm->state.enabled;

	if (!state->enabled) {
		if (enabled)
			ls_pwm_atim_disable(chip, pwm);
		return 0;
	}

	ret = ls_pwm_atim_set_polarity(chip, pwm, state->polarity);
	if (ret)
		return ret;

	period = min(state->period, NSEC_PER_SEC);
	duty_cycle = min(state->duty_cycle, NSEC_PER_SEC);

	ret = ls_pwm_atim_config(chip, pwm, state->duty_cycle, state->period);
	if (ret)
		return ret;

	if (!enabled && state->enabled)
		ret = ls_pwm_atim_enable(chip, pwm);

	return ret;
}

static int ls_pwm_atim_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	u32 arr_reg, ccr_reg, ccer_reg;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	arr_reg = pwm_loongson_readl(ddata, ATIM_ARR);
	ccr_reg = pwm_loongson_readl(ddata, ATIM_CCR(pwm->hwpwm));
	ccer_reg = pwm_loongson_readl(ddata, ATIM_CCER);

	/* duty & period have a max of 2^32, so we can't overflow */
	state->duty_cycle = DIV64_U64_ROUND_UP((u64)ccr_reg * NSEC_PER_SEC,
					       ddata->clk_rate);
	state->period = DIV64_U64_ROUND_UP((u64)arr_reg * NSEC_PER_SEC,
					   ddata->clk_rate);
	state->polarity = (ccer_reg & CCER_CCnP(pwm->hwpwm)) ?
				  PWM_POLARITY_INVERSED :
				  PWM_POLARITY_NORMAL;
	state->enabled = (ccer_reg & CCER_CCnE(pwm->hwpwm)) ? true : false;

	return 0;
}

static const struct pwm_ops ls_pwm_atim_ops = {
	.apply = ls_pwm_atim_apply,
	.get_state = ls_pwm_atim_get_state,
};

static int ls_pwm_atim_probe(struct platform_device *pdev)
{
	int ret;
	struct pwm_chip *chip;
	struct pwm_loongson_ddata *ddata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 of_clk_freq = 0;

	chip = devm_pwmchip_alloc(dev, 8, sizeof(*ddata));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ddata = to_pwm_loongson_ddata(chip);

	ddata->lss.en_mark = 0b00000000;

	ddata->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ddata->base))
		return PTR_ERR(ddata->base);

	ddata->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(ddata->clk))
		return dev_err_probe(dev, PTR_ERR(ddata->clk),
				     "failed to get pwm clock\n");
	ddata->clk_rate = LOONGSON_PWM_FREQ_DEFAULT;
	if (ddata->clk) {
		ret = devm_clk_rate_exclusive_get(dev, ddata->clk);
		if (ret)
			return ret;

		ddata->clk_rate = clk_get_rate(ddata->clk);
	} else {
#ifdef CONFIG_OF
		if (!of_property_read_u32(np, "clock-frequency", &of_clk_freq))
			ddata->clk_rate = of_clk_freq;
#endif
	}

	/* Explicitly initialize the CCER register */
	pwm_loongson_writel(ddata, 0, ATIM_CCER);

	/* Additional configuration for ATIM */
	pwm_loongson_writel(ddata, EGR_UG, ATIM_EGR);
	pwm_loongson_writel(ddata, BDTR_MOE, ATIM_BDTR);

	chip->ops = &ls_pwm_atim_ops;
	chip->atomic = true;
	dev_set_drvdata(dev, chip);

	ret = devm_pwmchip_add(dev, chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to add PWM chip\n");

	return 0;
}

static int ls_pwm_atim_suspend(struct device *dev)
{
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	ddata->lss.ccer_reg = pwm_loongson_readl(ddata, ATIM_CCER);
	ddata->lss.ccr_reg[0] = pwm_loongson_readl(ddata, ATIM_CCR(0));
	ddata->lss.ccr_reg[1] = pwm_loongson_readl(ddata, ATIM_CCR(1));
	ddata->lss.ccr_reg[2] = pwm_loongson_readl(ddata, ATIM_CCR(2));
	ddata->lss.ccr_reg[3] = pwm_loongson_readl(ddata, ATIM_CCR(3));
	ddata->lss.arr_reg = pwm_loongson_readl(ddata, ATIM_ARR);

	clk_disable_unprepare(ddata->clk);

	return 0;
}

static int ls_pwm_atim_resume(struct device *dev)
{
	int ret;
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	ret = clk_prepare_enable(ddata->clk);
	if (ret)
		return ret;

	pwm_loongson_writel(ddata, ddata->lss.ccer_reg, ATIM_CCER);
	pwm_loongson_writel(ddata, ddata->lss.ccr_reg[0], ATIM_CCR(0));
	pwm_loongson_writel(ddata, ddata->lss.ccr_reg[1], ATIM_CCR(1));
	pwm_loongson_writel(ddata, ddata->lss.ccr_reg[2], ATIM_CCR(2));
	pwm_loongson_writel(ddata, ddata->lss.ccr_reg[3], ATIM_CCR(3));
	pwm_loongson_writel(ddata, ddata->lss.arr_reg, ATIM_ARR);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ls_pwm_atim_pm_ops, ls_pwm_atim_suspend,
				ls_pwm_atim_resume);

static struct of_device_id ls_pwm_atim_id_table[] = {
	{ .compatible = "loongson,ls2k-pwm-atim" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ls_pwm_atim_id_table);

static struct platform_driver ls_pwm_atim_driver = {
	.driver = {
		.name = "ls-pwm-atim",
		.pm = pm_ptr(&ls_pwm_atim_pm_ops),
		.of_match_table = of_match_ptr(ls_pwm_atim_id_table),
	},
	.probe = ls_pwm_atim_probe,
};
module_platform_driver(ls_pwm_atim_driver);

MODULE_DESCRIPTION("Loongson Atimer Pwm Driver");
MODULE_AUTHOR("Ilikara <3435193369@qq.com>");
MODULE_LICENSE("GPL");
