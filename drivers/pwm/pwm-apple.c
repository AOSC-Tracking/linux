// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Driver for the Apple SoC PWM controller
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/math64.h>

#define PWM_CONTROL     0x00
#define PWM_ON_CYCLES   0x1c
#define PWM_OFF_CYCLES  0x18

#define CTRL_ENABLE        BIT(0)
#define CTRL_MODE          BIT(2)
#define CTRL_UPDATE        BIT(5)
#define CTRL_TRIGGER       BIT(9)
#define CTRL_INVERT        BIT(10)
#define CTRL_OUTPUT_ENABLE BIT(14)

struct apple_pwm {
	struct pwm_chip chip;
	void __iomem *base;
	u64 clkrate;
};

static int apple_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   const struct pwm_state *state)
{
	struct apple_pwm *fpwm;
	u64 on_cycles, off_cycles;

	fpwm = container_of(chip, struct apple_pwm, chip);
	if (state->enabled) {
		on_cycles = mul_u64_u64_div_u64(fpwm->clkrate,
						state->duty_cycle, NSEC_PER_SEC);
		off_cycles = mul_u64_u64_div_u64(fpwm->clkrate,
						 state->period, NSEC_PER_SEC) - on_cycles;
		writel(on_cycles, fpwm->base + PWM_ON_CYCLES);
		writel(off_cycles, fpwm->base + PWM_OFF_CYCLES);
		writel(CTRL_ENABLE | CTRL_OUTPUT_ENABLE | CTRL_UPDATE,
		       fpwm->base + PWM_CONTROL);
	} else {
		writel(0, fpwm->base + PWM_CONTROL);
	}
	return 0;
}

static int apple_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			   struct pwm_state *state)
{
	struct apple_pwm *fpwm;
	u32 on_cycles, off_cycles, ctrl;

	fpwm = container_of(chip, struct apple_pwm, chip);

	ctrl = readl(fpwm->base + PWM_CONTROL);
	on_cycles = readl(fpwm->base + PWM_ON_CYCLES);
	off_cycles = readl(fpwm->base + PWM_OFF_CYCLES);

	state->enabled = (ctrl & CTRL_ENABLE) && (ctrl & CTRL_OUTPUT_ENABLE);
	state->polarity = PWM_POLARITY_NORMAL;
	state->duty_cycle = div_u64(on_cycles, fpwm->clkrate) * NSEC_PER_SEC;
	state->period = div_u64(off_cycles + on_cycles, fpwm->clkrate) * NSEC_PER_SEC;

	return 0;
}

static const struct pwm_ops apple_pwm_ops = {
	.apply = apple_pwm_apply,
	.get_state = apple_pwm_get_state,
	.owner = THIS_MODULE,
};

static int apple_pwm_probe(struct platform_device *pdev)
{
	struct apple_pwm *pwm;
	struct clk *clk;
	int ret;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

	pwm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pwm->base))
		return PTR_ERR(pwm->base);

	platform_set_drvdata(pdev, pwm);

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	pwm->clkrate = clk_get_rate(clk);
	pwm->chip.dev = &pdev->dev;
	pwm->chip.npwm = 1;
	pwm->chip.ops = &apple_pwm_ops;

	ret = devm_pwmchip_add(&pdev->dev, &pwm->chip);
	return ret;
}

static const struct of_device_id apple_pwm_of_match[] = {
	{ .compatible = "apple,s5l-fpwm" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_pwm_of_match);

static struct platform_driver apple_pwm_driver = {
	.probe = apple_pwm_probe,
	.driver = {
		.name = "apple-pwm",
		.of_match_table = apple_pwm_of_match,
	},
};
module_platform_driver(apple_pwm_driver);

MODULE_DESCRIPTION("Apple SoC PWM driver");
MODULE_LICENSE("Dual MIT/GPL");
