// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017-2025 Loongson Technology Corporation Limited.
 *
 * Loongson PWM driver
 *
 * For Loongson's PWM IP block documentation please refer Chapter 11 of
 * Reference Manual: https://loongson.github.io/LoongArch-Documentation/Loongson-7A1000-usermanual-EN.pdf
 *
 * Author: Juxin Gao <gaojuxin@loongson.cn>
 * Further cleanup and restructuring by:
 *         Binbin Zhou <zhoubinbin@loongson.cn>
 *
 * Limitations:
 * - If both DUTY and PERIOD are set to 0, the output is a constant low signal.
 * - When disabled the output is driven to 0 independent of the configured
 *   polarity.
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/units.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

/* Loongson PWM registers */
#define LOONGSON_PWM_REG_DUTY		0x4 /* Low Pulse Buffer Register */
#define LOONGSON_PWM_REG_PERIOD		0x8 /* Pulse Period Buffer Register */
#define LOONGSON_PWM_REG_CTRL		0xc /* Control Register */

/* Control register bits */
#define LOONGSON_PWM_CTRL_EN		BIT(0)  /* Counter Enable Bit */
#define LOONGSON_PWM_CTRL_OE		BIT(3)  /* Pulse Output Enable Control Bit, Valid Low */
#define LOONGSON_PWM_CTRL_SINGLE	BIT(4)  /* Single Pulse Control Bit */
#define LOONGSON_PWM_CTRL_INTE		BIT(5)  /* Interrupt Enable Bit */
#define LOONGSON_PWM_CTRL_INT		BIT(6)  /* Interrupt Bit */
#define LOONGSON_PWM_CTRL_RST		BIT(7)  /* Counter Reset Bit */
#define LOONGSON_PWM_CTRL_CAPTE		BIT(8)  /* Measurement Pulse Enable Bit */
#define LOONGSON_PWM_CTRL_INVERT	BIT(9)  /* Output flip-flop Enable Bit */
#define LOONGSON_PWM_CTRL_DZONE		BIT(10) /* Anti-dead Zone Enable Bit */

/* default input clk frequency for the ACPI case */
#define LOONGSON_PWM_FREQ_DEFAULT	50000000 /* Hz */

struct pwm_loongson_ddata {
	struct clk *clk;
	void __iomem *base;
	u32 irq;
	u32 int_count;
	u64 clk_rate;
	struct pwm_loongson_suspend_store lss;
	struct wait_queue_head capture_wait_queue;
};

static inline struct pwm_loongson_ddata *to_pwm_loongson_ddata(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static inline u32 pwm_loongson_readl(struct pwm_loongson_ddata *ddata, u32 offset)
{
	return readl(ddata->base + offset);
}

static inline void pwm_loongson_writel(struct pwm_loongson_ddata *ddata,
				       u32 val, u32 offset)
{
	writel(val, ddata->base + offset);
}

static irqreturn_t pwm_loongson_isr(int irq, void *dev)
{
	u32 val;
	struct pwm_chip *chip = dev;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);
	if ((val & LOONGSON_PWM_CTRL_INT) == 0) {
		return IRQ_NONE;
	}
	val |= LOONGSON_PWM_CTRL_INT;
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	ddata->int_count++;

	pr_debug("pwm_loongson_isr count %u\n", ddata->int_count);

	return IRQ_HANDLED;
}

static int pwm_loongson_capture(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_capture *result,
				unsigned long timeout)
{
	u32 val;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);
	int ret;

	ddata->int_count = 0;
	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);
	val |= LOONGSON_PWM_CTRL_EN | LOONGSON_PWM_CTRL_CAPTE |
	       LOONGSON_PWM_CTRL_INTE | LOONGSON_PWM_CTRL_OE;
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	ret = wait_event_timeout(ddata->capture_wait_queue,
				 ddata->int_count > 1, timeout);
	if (ret == 0)
		return -ETIMEDOUT;

	result->period = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_PERIOD);
	result->duty_cycle = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_DUTY);

	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);
	val &= ~(LOONGSON_PWM_CTRL_EN | LOONGSON_PWM_CTRL_CAPTE |
		 LOONGSON_PWM_CTRL_INTE | LOONGSON_PWM_CTRL_OE);
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	return 0;
}

static int pwm_loongson_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				     enum pwm_polarity polarity)
{
	u16 val;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);

	if (polarity == PWM_POLARITY_INVERSED)
		/* Duty cycle defines LOW period of PWM */
		val &= ~LOONGSON_PWM_CTRL_INVERT;
	else
		/* Duty cycle defines HIGH period of PWM */
		val |= LOONGSON_PWM_CTRL_INVERT;

	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	return 0;
}

static void pwm_loongson_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	u32 duty;
	u32 period;
	u32 period_1000ns;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	duty = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_DUTY);
	period = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_PERIOD);

	period_1000ns = mul_u64_u64_div_u64(1000, ddata->clk_rate, NSEC_PER_SEC);
	pwm_loongson_writel(ddata, 1, LOONGSON_PWM_REG_DUTY);
	pwm_loongson_writel(ddata, period_1000ns, LOONGSON_PWM_REG_PERIOD);
	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);
	val |= LOONGSON_PWM_CTRL_RST;
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);
	val ^= LOONGSON_PWM_CTRL_RST;
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	ndelay(1000);

	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);
	val &= ~LOONGSON_PWM_CTRL_EN;
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	pwm_loongson_writel(ddata, duty, LOONGSON_PWM_REG_DUTY);
	pwm_loongson_writel(ddata, period, LOONGSON_PWM_REG_PERIOD);
}

static int pwm_loongson_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	val = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);
	val |= LOONGSON_PWM_CTRL_EN;
	pwm_loongson_writel(ddata, val, LOONGSON_PWM_REG_CTRL);

	return 0;
}

static int pwm_loongson_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       u64 duty_ns, u64 period_ns)
{
	u32 duty, period;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	/* duty = duty_ns * ddata->clk_rate / NSEC_PER_SEC */
	duty = mul_u64_u64_div_u64(duty_ns, ddata->clk_rate, NSEC_PER_SEC);
	pwm_loongson_writel(ddata, duty, LOONGSON_PWM_REG_DUTY);

	/* period = period_ns * ddata->clk_rate / NSEC_PER_SEC */
	period = mul_u64_u64_div_u64(period_ns, ddata->clk_rate, NSEC_PER_SEC);
	pwm_loongson_writel(ddata, period, LOONGSON_PWM_REG_PERIOD);

	return 0;
}

static int pwm_loongson_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	int ret;
	u64 period, duty_cycle;
	bool enabled = pwm->state.enabled;

	if (!state->enabled) {
		if (enabled)
			pwm_loongson_disable(chip, pwm);
		return 0;
	}

	ret = pwm_loongson_set_polarity(chip, pwm, state->polarity);
	if (ret)
		return ret;

	period = min(state->period, NSEC_PER_SEC);
	duty_cycle = min(state->duty_cycle, NSEC_PER_SEC);

	ret = pwm_loongson_config(chip, pwm, duty_cycle, period);
	if (ret)
		return ret;

	if (!enabled && state->enabled)
		ret = pwm_loongson_enable(chip, pwm);

	return ret;
}

static int pwm_loongson_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				  struct pwm_state *state)
{
	u32 duty, period, ctrl;
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	duty = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_DUTY);
	period = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_PERIOD);
	ctrl = pwm_loongson_readl(ddata, LOONGSON_PWM_REG_CTRL);

	/* duty & period have a max of 2^32, so we can't overflow */
	state->duty_cycle = DIV64_U64_ROUND_UP((u64)duty * NSEC_PER_SEC, ddata->clk_rate);
	state->period = DIV64_U64_ROUND_UP((u64)period * NSEC_PER_SEC, ddata->clk_rate);
	state->polarity = (ctrl & LOONGSON_PWM_CTRL_INVERT) ? PWM_POLARITY_NORMAL :
			  PWM_POLARITY_INVERSED;
	state->enabled = (ctrl & LOONGSON_PWM_CTRL_EN) ? true : false;

	return 0;
}

static const struct pwm_ops pwm_loongson_ops = {
	.capture = pwm_loongson_capture,
	.apply = pwm_loongson_apply,
	.get_state = pwm_loongson_get_state,
};

static int pwm_loongson_probe(struct platform_device *pdev)
{
	int ret;
	struct pwm_chip *chip;
	struct pwm_loongson_ddata *ddata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 of_clk_freq = 0;
	u32 irq;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
	    dev_err(&pdev->dev, "no irq resource?\n");
	    return -ENODEV;
	}

	chip = devm_pwmchip_alloc(dev, 1, sizeof(*ddata));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ddata = to_pwm_loongson_ddata(chip);

    // 初始化等待队列
    init_waitqueue_head(&ddata->capture_wait_queue);

	ddata->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ddata->base))
		return PTR_ERR(ddata->base);

	ddata->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(ddata->clk))
		return dev_err_probe(dev, PTR_ERR(ddata->clk),
				     "Failed to get pwm clock\n");
	ddata->clk_rate = LOONGSON_PWM_FREQ_DEFAULT;
	if (ddata->clk) {
		ret = devm_clk_rate_exclusive_get(dev, ddata->clk);
		if (ret)
			return dev_err_probe(dev, PTR_ERR(ddata->clk),
					     "Failed to get exclusive rate\n");

		ddata->clk_rate = clk_get_rate(ddata->clk);
	} else {
#ifdef CONFIG_OF
		if (!of_property_read_u32(np, "clock-frequency", &of_clk_freq))
			ddata->clk_rate = of_clk_freq;
#endif
	}

	ddata->irq = irq;

	ret = devm_request_irq(&pdev->dev, ddata->irq, pwm_loongson_isr,
			       IRQF_SHARED,
			       dev_name(&pdev->dev), chip);

	if (ret)
		dev_err(&pdev->dev, "failure requesting irq %d\n", ret);

	chip->ops = &pwm_loongson_ops;
	chip->atomic = true;
	dev_set_drvdata(dev, chip);

	ret = devm_pwmchip_add(dev, chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to add PWM chip\n");

	return 0;
}

static int pwm_loongson_suspend(struct device *dev)
{
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);
	struct pwm_device *pwm = &chip->pwms[0];

	if (pwm->state.enabled)
		return -EBUSY;

	clk_disable_unprepare(ddata->clk);

	return 0;
}

static int pwm_loongson_resume(struct device *dev)
{
	struct pwm_chip *chip = dev_get_drvdata(dev);
	struct pwm_loongson_ddata *ddata = to_pwm_loongson_ddata(chip);

	return clk_prepare_enable(ddata->clk);
}

static DEFINE_SIMPLE_DEV_PM_OPS(pwm_loongson_pm_ops, pwm_loongson_suspend,
				pwm_loongson_resume);

static const struct of_device_id pwm_loongson_of_ids[] = {
	{ .compatible = "loongson,ls7a-pwm" },
	{ .compatible = "loongson,ls2k-pwm" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pwm_loongson_of_ids);

static const struct acpi_device_id pwm_loongson_acpi_ids[] = {
	{ "LOON0006" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, pwm_loongson_acpi_ids);

static struct platform_driver pwm_loongson_driver = {
	.probe = pwm_loongson_probe,
	.driver = {
		.name = "loongson-pwm",
		.pm = pm_ptr(&pwm_loongson_pm_ops),
		.of_match_table = pwm_loongson_of_ids,
		.acpi_match_table = pwm_loongson_acpi_ids,
	},
};
module_platform_driver(pwm_loongson_driver);

MODULE_DESCRIPTION("Loongson PWM driver");
MODULE_AUTHOR("Loongson Technology Corporation Limited.");
MODULE_LICENSE("GPL");
