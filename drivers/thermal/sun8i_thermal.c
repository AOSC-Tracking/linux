// SPDX-License-Identifier: GPL-2.0
/*
 * Thermal sensor driver for Allwinner SOC
 * Copyright (C) 2019 Yangtao Li
 *
 * Based on the work of Icenowy Zheng <icenowy@aosc.io>
 * Based on the work of Ondrej Jirman <megous@megous.com>
 * Based on the work of Josef Gajdusek <atx@atx.name>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#define MAX_SENSOR_NUM	4

#define FT_TEMP_MASK				GENMASK(11, 0)
#define TEMP_CALIB_MASK				GENMASK(11, 0)
#define TEMP_TO_REG				672
#define CALIBRATE_DEFAULT			0x800

#define SUN50I_THS_CTRL0			0x00
#define SUN50I_H6_THS_ENABLE			0x04
#define SUN50I_H6_THS_PC			0x08
#define SUN50I_H6_THS_DIC			0x10
#define SUN50I_H6_THS_DIS			0x20
#define SUN50I_H6_THS_MFC			0x30
#define SUN50I_H6_THS_TEMP_CALIB		0xa0
#define SUN50I_H6_THS_TEMP_DATA			0xc0

#define SUN50I_THS_CTRL0_T_ACQ(x)		((GENMASK(15, 0) & (x)) << 16)
#define SUN50I_THS_FILTER_EN			BIT(2)
#define SUN50I_THS_FILTER_TYPE(x)		(GENMASK(1, 0) & (x))
#define SUN50I_H6_THS_PC_TEMP_PERIOD(x)		((GENMASK(19, 0) & (x)) << 12)
#define SUN50I_H6_THS_DATA_IRQ_STS(x)		BIT(x)

/* millidegree celsius */
#define SUN50I_H6_FT_DEVIATION			7000

struct ths_device;

struct tsensor {
	struct ths_device		*tmdev;
	struct thermal_zone_device	*tzd;
	int				id;
};

struct ths_thermal_chip {
	bool            has_mod_clk;
	int		sensor_num;
	int		offset;
	int		scale;
	int		ft_deviation;
	int		temp_data_base;
	int		(*init)(struct ths_device *tmdev);
	int             (*irq_ack)(struct ths_device *tmdev);
};

struct ths_device {
	const struct ths_thermal_chip		*chip;
	struct device				*dev;
	struct regmap				*regmap;
	struct reset_control			*reset;
	struct clk				*bus_clk;
	struct clk                              *mod_clk;
	struct tsensor				sensor[MAX_SENSOR_NUM];
};

/* Temp Unit: millidegree Celsius */
static int sun8i_ths_reg2temp(struct ths_device *tmdev,
			      int reg)
{
	return (reg + tmdev->chip->offset) * tmdev->chip->scale;
}

static int sun8i_ths_get_temp(void *data, int *temp)
{
	struct tsensor *s = data;
	struct ths_device *tmdev = s->tmdev;
	int val;

	regmap_read(tmdev->regmap, tmdev->chip->temp_data_base +
		    0x4 * s->id, &val);

	/* ths have no data yet */
	if (!val)
		return -EBUSY;

	*temp = sun8i_ths_reg2temp(tmdev, val);
	/*
	 * XX - According to the original sdk, there are some platforms(rarely)
	 * that add a fixed offset value after calculating the temperature
	 * value. We can't simply put it on the formula for calculating the
	 * temperature above, because the formula for calculating the
	 * temperature above is also used when the sensor is calibrated. If
	 * do this, the correct calibration formula is hard to know.
	 */
	*temp += tmdev->chip->ft_deviation;

	return 0;
}

static const struct thermal_zone_of_device_ops ths_ops = {
	.get_temp = sun8i_ths_get_temp,
};

static const struct regmap_config config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
};

static int sun50i_h6_irq_ack(struct ths_device *tmdev)
{
	int i, state, ret = 0;

	regmap_read(tmdev->regmap, SUN50I_H6_THS_DIS, &state);

	for (i = 0; i < tmdev->chip->sensor_num; i++) {
		if (state & SUN50I_H6_THS_DATA_IRQ_STS(i)) {
			regmap_write(tmdev->regmap, SUN50I_H6_THS_DIS,
				     SUN50I_H6_THS_DATA_IRQ_STS(i));
			ret |= BIT(i);
		}
	}

	return ret;
}

static irqreturn_t sun8i_irq_thread(int irq, void *data)
{
	struct ths_device *tmdev = data;
	int i, state;

	state = tmdev->chip->irq_ack(tmdev);

	for (i = 0; i < tmdev->chip->sensor_num; i++) {
		if ((state & BIT(i)) && tmdev->sensor[i].tzd)
			thermal_zone_device_update(tmdev->sensor[i].tzd,
						   THERMAL_EVENT_UNSPECIFIED);
	}

	return IRQ_HANDLED;
}

static int sun50i_ths_calibrate(struct ths_device *tmdev)
{
	struct nvmem_cell *calcell;
	struct device *dev = tmdev->dev;
	u16 *caldata;
	size_t callen;
	int ft_temp;
	int i, ret = 0;

	calcell = devm_nvmem_cell_get(dev, "calib");
	if (IS_ERR(calcell)) {
		if (PTR_ERR(calcell) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		/*
		 * Even if the external calibration data stored in sid is
		 * not accessible, the THS hardware can still work, although
		 * the data won't be so accurate.
		 *
		 * The default value of calibration register is 0x800 for
		 * every sensor, and the calibration value is usually 0x7xx
		 * or 0x8xx, so they won't be away from the default value
		 * for a lot.
		 *
		 * So here we do not return error if the calibartion data is
		 * not available, except the probe needs deferring.
		 */
		goto out;
	}

	caldata = nvmem_cell_read(calcell, &callen);
	if (IS_ERR(caldata)) {
		ret = PTR_ERR(caldata);
		goto out;
	}

	if (!caldata[0] || callen < 2 + 2 * tmdev->chip->sensor_num) {
		ret = -EINVAL;
		goto out_free;
	}

	/*
	 * efuse layout:
	 *
	 *	0   11  16	 32
	 *	+-------+-------+-------+
	 *	|temp|  |sensor0|sensor1|
	 *	+-------+-------+-------+
	 *
	 * The calibration data on the H6 is the ambient temperature and
	 * sensor values that are filled during the factory test stage.
	 *
	 * The unit of stored FT temperature is 0.1 degreee celusis.
	 * Through the stored ambient temperature and the data read
	 * by the sensor, after a certain calculation, the calibration
	 * value to be compensated can be obtained.
	 */
	ft_temp = caldata[0] & FT_TEMP_MASK;

	for (i = 0; i < tmdev->chip->sensor_num; i++) {
		int reg = (int)caldata[i + 1];
		int sensor_temp = sun8i_ths_reg2temp(tmdev, reg);
		int delta, cdata, calib_offest;

		/*
		 * To calculate the calibration value:
		 *
		 * X(in Celsius) = Ts - ft_temp
		 * delta = X * 10000 / TEMP_TO_REG
		 * cdata = CALIBRATE_DEFAULT - delta
		 *
		 * cdata: calibration value
		 */
		delta = (sensor_temp - ft_temp * 100) * 10 / TEMP_TO_REG;
		cdata = CALIBRATE_DEFAULT - delta;
		if (cdata & ~TEMP_CALIB_MASK) {
			/*
			 * Calibration value more than 12-bit, but calibration
			 * register is 12-bit. In this case, ths hardware can
			 * still work without calibration, although the data
			 * won't be so accurate.
			 */
			dev_warn(dev, "sensor%d is not calibrated.\n", i);

			continue;
		}

		calib_offest = SUN50I_H6_THS_TEMP_CALIB + (i / 2) * 0x4;

		if (i % 2) {
			int val;

			regmap_read(tmdev->regmap, calib_offest, &val);
			val = (val & TEMP_CALIB_MASK) | (cdata << 16);
			regmap_write(tmdev->regmap, calib_offest, val);
		} else {
			regmap_write(tmdev->regmap, calib_offest, cdata);
		}
	}

out_free:
	kfree(caldata);
out:
	return ret;
}

static int sun8i_ths_resource_init(struct ths_device *tmdev)
{
	struct device *dev = tmdev->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *mem;
	void __iomem *base;
	int ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	tmdev->regmap = devm_regmap_init_mmio(dev, base, &config);
	if (IS_ERR(tmdev->regmap))
		return PTR_ERR(tmdev->regmap);

	tmdev->reset = devm_reset_control_get(dev, 0);
	if (IS_ERR(tmdev->reset))
		return PTR_ERR(tmdev->reset);

	tmdev->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(tmdev->bus_clk))
		return PTR_ERR(tmdev->bus_clk);

	if (tmdev->chip->has_mod_clk) {
		tmdev->mod_clk = devm_clk_get(&pdev->dev, "mod");
		if (IS_ERR(tmdev->mod_clk))
			return PTR_ERR(tmdev->mod_clk);
	}

	ret = reset_control_deassert(tmdev->reset);
	if (ret)
		return ret;

	ret = clk_prepare_enable(tmdev->bus_clk);
	if (ret)
		goto assert_reset;

	if (tmdev->chip->has_mod_clk) {
		ret = clk_prepare_enable(tmdev->mod_clk);
		if (ret)
			goto bus_disable;
	}

	ret = sun50i_ths_calibrate(tmdev);
	if (ret)
		goto mod_disable;

	return 0;

mod_disable:
	if (tmdev->chip->has_mod_clk)
		clk_disable_unprepare(tmdev->mod_clk);
bus_disable:
	clk_disable_unprepare(tmdev->bus_clk);
assert_reset:
	reset_control_assert(tmdev->reset);

	return ret;
}

static int sun50i_thermal_init(struct ths_device *tmdev)
{
	int val;

	/*
	 * clkin = 24MHz
	 * T acquire = clkin / (x + 1)
	 *           = 20us
	 */
	regmap_write(tmdev->regmap, SUN50I_THS_CTRL0,
		     SUN50I_THS_CTRL0_T_ACQ(479));
	/* average over 4 samples */
	regmap_write(tmdev->regmap, SUN50I_H6_THS_MFC,
		     SUN50I_THS_FILTER_EN |
		     SUN50I_THS_FILTER_TYPE(1));
	/* period = (x + 1) * 4096 / clkin; ~10ms */
	regmap_write(tmdev->regmap, SUN50I_H6_THS_PC,
		     SUN50I_H6_THS_PC_TEMP_PERIOD(58));
	/* enable sensor */
	val = GENMASK(tmdev->chip->sensor_num - 1, 0);
	regmap_write(tmdev->regmap, SUN50I_H6_THS_ENABLE, val);
	/* thermal data interrupt enable */
	val = GENMASK(tmdev->chip->sensor_num - 1, 0);
	regmap_write(tmdev->regmap, SUN50I_H6_THS_DIC, val);

	return 0;
}

static int sun8i_ths_register(struct ths_device *tmdev)
{
	struct thermal_zone_device *tzd;
	int i;

	for (i = 0; i < tmdev->chip->sensor_num; i++) {
		tmdev->sensor[i].tmdev = tmdev;
		tmdev->sensor[i].id = i;
		tzd = devm_thermal_zone_of_sensor_register(tmdev->dev,
							   i,
							   &tmdev->sensor[i],
							   &ths_ops);

		if (IS_ERR(tzd))
			tmdev->sensor[i].tzd = NULL;
		else
			tmdev->sensor[i].tzd = tzd;
	}

	return 0;
}

static int sun8i_ths_probe(struct platform_device *pdev)
{
	struct ths_device *tmdev;
	struct device *dev = &pdev->dev;
	int ret, irq;

	tmdev = devm_kzalloc(dev, sizeof(*tmdev), GFP_KERNEL);
	if (!tmdev)
		return -ENOMEM;

	tmdev->dev = dev;
	tmdev->chip = of_device_get_match_data(&pdev->dev);
	if (!tmdev->chip)
		return -EINVAL;

	platform_set_drvdata(pdev, tmdev);

	ret = sun8i_ths_resource_init(tmdev);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = tmdev->chip->init(tmdev);
	if (ret)
		return ret;

	ret = sun8i_ths_register(tmdev);
	if (ret)
		return ret;

	/*
	 * Avoid entering the interrupt handler, the thermal device is not
	 * registered yet, we deffer the registration of the interrupt to
	 * the end.
	 */
	ret = devm_request_threaded_irq(dev, irq, NULL,
					sun8i_irq_thread,
					IRQF_ONESHOT, "ths", tmdev);
	if (ret)
		return ret;

	return ret;
}

static int sun8i_ths_remove(struct platform_device *pdev)
{
	struct ths_device *tmdev = platform_get_drvdata(pdev);

	if (tmdev->chip->has_mod_clk)
		clk_disable_unprepare(tmdev->mod_clk);

	clk_disable_unprepare(tmdev->bus_clk);
	reset_control_assert(tmdev->reset);

	return 0;
}

static const struct ths_thermal_chip sun50i_h6_ths = {
	.sensor_num = 2,
	.offset = -2794,
	.scale = -67,
	.ft_deviation = SUN50I_H6_FT_DEVIATION,
	.temp_data_base = SUN50I_H6_THS_TEMP_DATA,
	.init = sun50i_thermal_init,
	.irq_ack = sun50i_h6_irq_ack,
};

static const struct of_device_id of_ths_match[] = {
	{ .compatible = "allwinner,sun50i-h6-ths", .data = &sun50i_h6_ths },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, of_ths_match);

static struct platform_driver ths_driver = {
	.probe = sun8i_ths_probe,
	.remove = sun8i_ths_remove,
	.driver = {
		.name = "sun8i-thermal",
		.of_match_table = of_ths_match,
	},
};
module_platform_driver(ths_driver);

MODULE_DESCRIPTION("Thermal sensor driver for Allwinner SOC");
MODULE_LICENSE("GPL v2");
