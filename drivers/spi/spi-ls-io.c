/*
 * Loongson SPI driver
 *
 * Copyright (C) 2017 Juxin Gao <gaojuxin@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/pci.h>
#include <linux/of.h>
#include <linux/clk.h>
// #include <asm-generic/unaligned.h>
// #include <linux/unaligned/le_byteshift.h>

/*SPI IO registers */
#define REG_CR1	0x00
#define REG_CR2	0x04
#define REG_CR3	0x08
#define REG_CR4	0x0c
#define REG_IER	0x10
#define REG_SR1	0x14
#define REG_SR2	0x18
#define REG_CFG1	0x20
#define REG_CFG2	0x24
#define REG_CFG3	0x28
#define REG_CRC1	0x30
#define REG_CRC2	0x34
#define REG_DR		0x40
#define REG_DMA_ISR	0x100
#define REG_DMA_IFCR	0x104
#define REG_DMA_CCR_RX	0x110
#define REG_DMA_CNDTR_RX	0x114
#define REG_DMA_CMAR_RX	0x11c
#define REG_DMA_CCR_TX	0x120
#define REG_DMA_CNDTR_TX	0x124
#define REG_DMA_CMAR_TX	0x12c

#define SHIFTW(_x, _s) ((_x) << (_s))
#define SHIFTR(_x, _s) ((_x) >> (_s))

/* SPI_CR1 bit fields */
#define BIT_CR1_SPE		BIT(0)
#define BIT_CR1_CSTART          BIT(1)
#define BIT_CR1_AUTOSUS		BIT(2)
#define BIT_CR1_SSREV           BIT(8)

/* SPI_CR2 bit fields */
#define BIT_CR2_RXDMAEN		BIT(7)
#define BIT_CR2_TXDMAEN		BIT(15)
#define MASK_CR2_RXFTHLV	0x3
#define MASK_CR2_TXFTHLV	0x300

/* SPI_IER bit fields */
#define BIT_IER_RXAIE		BIT(0)
#define BIT_IER_TXAIE		BIT(1)
#define BIT_IER_DXAIE		BIT(2)
#define BIT_IER_RXEIE		BIT(4)
#define BIT_IER_TXEIE		BIT(5)
#define BIT_IER_SUSPIE		BIT(7)
#define BIT_IER_OVRIE		BIT(8)
#define BIT_IER_UDRIE		BIT(9)
#define BIT_IER_CRCEIE		BIT(10)
#define BIT_IER_MODFIE		BIT(11)
#define BIT_IER_EOTIE		BIT(15)
#define BIT_IER_ALL \
	(BIT_IER_RXAIE | BIT_IER_TXAIE | BIT_IER_DXAIE | BIT_IER_RXEIE | \
	BIT_IER_TXEIE | BIT_IER_SUSPIE | BIT_IER_OVRIE | BIT_IER_UDRIE | \
	BIT_IER_CRCEIE | BIT_IER_MODFIE | BIT_IER_EOTIE)

/* SPI_SR1 bit fields */
#define BIT_SR1_RXA		BIT(0)
#define BIT_SR1_TXA		BIT(1)
#define BIT_SR1_DXA		BIT(2)
#define BIT_SR1_RXE		BIT(4)
#define BIT_SR1_TXE		BIT(5)
#define BIT_SR1_SUSP		BIT(7)
#define BIT_SR1_OVR		BIT(8)
#define BIT_SR1_UDR		BIT(9)
#define BIT_SR1_CRCE		BIT(10)
#define BIT_SR1_MODF		BIT(11)
#define BIT_SR1_EOT		BIT(15)

/* SPI_SR2 bit fields */
#define MASK_SR2_RXFLV		0x7 // << 0
#define MASK_SR2_TXFLV		0x700 // << 8

/* SPI_CFG1 bit fields */
#define BIT_CFG1_CPOL		BIT(0)
#define BIT_CFG1_CPHA		BIT(1)
#define BIT_CFG1_LSBFRST	BIT(7)
#define MASK_CFG1_DSIZE		0x1f00 // << 8

/* SPI_CFG2 bit fields */
#define SHIFT_CFG2_BRINT	8
#define MASK_CFG2_BRDEC		0xfc // << 2
#define MASK_CFG2_BRINT		0xff00 // << 8

/* SPI_CFG3 bit fields */
#define BIT_CFG3_MSTR		BIT(0)
#define BIT_CFG3_DIOSWP		BIT(1)
#define BIT_CFG3_DIE		BIT(2)
#define BIT_CFG3_DOE		BIT(3)
#define MASK_CFG3_SSMODE	0x300 // << 8

/* SPI_CRC1 bit fields */
#define BIT_CRC1_CRCEN		BIT(0)
#define MASK_CRC1_CRCPOLY	0xfffffffe // << 1

/* SPI_CRC2 bit fields */
#define BIT_CRC2_RCRCINI	BIT(0)
#define BIT_CRC2_TCRCINI	BIT(1)
#define MASK_CRC2_CRCSIZE	0x1f00 // << 8

#define ls_spiio_write_reg(spi, offset, data) \
	writel(data, spi->reg + offset)
#define ls_spiio_read_reg(spi, offset) \
	readl(spi->reg + offset)

#define ls_spiio_write_reg_byte(spi, offset, data) \
	writeb(data, spi->reg + offset)
#define ls_spiio_read_reg_byte(spi, offset) \
	readb(spi->reg + offset)


struct ls_spiio_priv {
	void __iomem *reg;
	struct spi_controller *controller;
	u32 ref_clk;
	u32 clk;
	u8 dsize;
	struct gpio_desc **cs_gpios;
	struct device *dev;
};

static int ls_spiio_idle_sr1(struct ls_spiio_priv* priv, u32 bit)
{
	int timeout = 1000;
	while(!(ls_spiio_read_reg(priv, REG_SR1) & bit)
			&& timeout--);
	return timeout;
}

static int ls_spiio_active_cs(struct ls_spiio_priv* priv, int cs)
{
	int active = 0;

	if (!priv->cs_gpios)
		ls_spiio_write_reg(priv, REG_CR1, BIT_CR1_SSREV | BIT_CR1_SPE);
	else
		gpiod_set_raw_value(priv->cs_gpios[cs], active);

	return 0;
}

static int ls_spiio_clear_cs(struct ls_spiio_priv* priv)
{
	struct spi_controller* controller = priv->controller;
	int clear = 1;
	int i;

	if (!priv->cs_gpios)
		ls_spiio_write_reg(priv, REG_CR1, BIT_CR1_SSREV);
	else
		for (i = 0; i < controller->num_chipselect; i++)
			gpiod_set_raw_value(priv->cs_gpios[i], clear);

	return 0;
}

static int ls_spiio_free_cs_gpios(struct ls_spiio_priv* priv)
{
	struct spi_controller* controller = priv->controller;
	struct device *dev = priv->dev;
	int i;

	if (!priv->cs_gpios)
		return 0;

	for (i = 0; i < controller->num_chipselect; i++)
		devm_gpiod_put(dev, priv->cs_gpios[i]);

	devm_kfree(dev, priv->cs_gpios);

	return 0;
}

static int ls_spiio_init_cs_gpios(struct ls_spiio_priv* priv)
{
	struct spi_controller* controller = priv->controller;
	struct device *dev = priv->dev;
	int clear = 1;
	int ret;
	int i;

	priv->cs_gpios = devm_kcalloc(dev, controller->num_chipselect,
					sizeof(*priv->cs_gpios),
					GFP_KERNEL);
	if (!priv->cs_gpios)
		return -ENOMEM;

	for (i = 0; i < controller->num_chipselect; i++) {
		priv->cs_gpios[i] = devm_gpiod_get_index(dev, "cs", i,
				GPIOD_OUT_HIGH);
		if (IS_ERR(priv->cs_gpios[i]))
		{
			ret = PTR_ERR(priv->cs_gpios[i]);
			dev_err(dev, "Cannot Requst cs-gpio[%d]: %d\n",
					i, ret);
			goto free_cs_gpio_array;
		}
		else
		{
			dev_info(dev, "GPIO[%d] as cs[%d]\n",
					desc_to_gpio(priv->cs_gpios[i]), i);
			gpiod_direction_output_raw(priv->cs_gpios[i], clear);
		}
	}

	return 0;

free_cs_gpio_array:
	devm_kfree(dev, priv->cs_gpios);

	return ret;
}

static int ls_spiio_xfer(struct ls_spiio_priv* priv, u8* tx, u8* rx, int len)
{
	int remain = len;

	if (len > 0)
	{
		while (remain > 0)
		{

			if (ls_spiio_idle_sr1(priv, BIT_SR1_TXA) < 0) {
				pr_debug("wait txa timeout!\n");
				return 0;
			}
			if (tx != NULL)
			{
				ls_spiio_write_reg_byte(priv, REG_DR, *tx);
				tx++;
			}
			else
			{
				ls_spiio_write_reg_byte(priv, REG_DR, 0x00);
			}

			ls_spiio_write_reg(priv, REG_CR1,
				BIT_CR1_SPE | BIT_CR1_CSTART | BIT_CR1_AUTOSUS);
			if (ls_spiio_idle_sr1(priv, BIT_SR1_EOT) < 0) {
				pr_debug("wait eot timeout!\n");
				return 0;
			}
			else
			{
				ls_spiio_write_reg(priv, REG_SR1, BIT_SR1_EOT);
			}

			if (ls_spiio_idle_sr1(priv, BIT_SR1_RXA) < 0) {
				pr_debug("wait rxa timeout!\n");
				return 0;
			}
			if (rx != NULL)
			{
				*rx = ls_spiio_read_reg_byte(priv, REG_DR);
				rx++;
			}
			else
			{
				ls_spiio_read_reg_byte(priv, REG_DR);
			}

			remain--;
		}
	}


	return len;
}

static int ls_spiio_set_clk_timing(struct ls_spiio_priv* priv,
		unsigned long clk, u32 mode)
{
	u32 val;

	val = 0;
	if (priv->clk != clk)
	{
		val= DIV_ROUND_UP(priv->ref_clk, clk);

		if (val < 2)
			val = 2;

		if (val > 255)
			val = 255;

		pr_debug("refclk = %d, need = %ld, approximate = %d\n",
				priv->ref_clk, clk, priv->ref_clk / val);

		priv->clk = clk;

		ls_spiio_write_reg(priv, REG_CFG2, (val << 8));
	}

	val = 0;
	if (mode & SPI_CPOL)
		val |= (1 << 0);
	if (mode & SPI_CPHA)
		val |= (1 << 1);
	if (mode & SPI_LSB_FIRST)
		val |= (1 << 7);
	val |= (priv->dsize << 8);

	pr_debug("clk timing : 0x%x\n", val);
	ls_spiio_write_reg(priv, REG_CFG1, val);

	return 0;
}

static int ls_spiio_init(struct ls_spiio_priv* priv)
{
	/*pin set,controller mode and full duplex*/
	//MSTR:1 DIOSWP:0 DIE:1 DOE:1 SSMODE:0
	ls_spiio_write_reg(priv, REG_CFG3,
			BIT_CFG3_MSTR | BIT_CFG3_DIE | BIT_CFG3_DOE);
	/*set clk rate */
	//BRINT:0xff BRDEC:0
	ls_spiio_write_reg(priv, REG_CFG2,0xff00);
	/*set transfer format*/
	//CPOL:0 CPHA:0 LSBFRST:0 DSIZE:7
	ls_spiio_write_reg(priv, REG_CFG1, 0x700);
	// DR Enable
	ls_spiio_write_reg(priv, REG_CR1, BIT_CR1_SPE);

	priv->dsize = 7;

	return 0;
}

static int __spi_setup(struct spi_device *spi)
{
	struct ls_spiio_priv* priv;

	priv = spi_controller_get_devdata(spi->controller);

	ls_spiio_set_clk_timing(priv, spi->max_speed_hz, spi->mode);

	return 0;
}

static int __spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct ls_spiio_priv* priv = spi_controller_get_devdata(spi->controller);
	struct spi_transfer *t = NULL;
	m->actual_length = 0;
	m->status = 0;

	ls_spiio_active_cs(priv, spi->chip_select[0]);
	list_for_each_entry(t, &m->transfers, transfer_list) {
		/*setup spi clock*/
		ls_spiio_set_clk_timing(priv, t->speed_hz, spi->mode);
		m->actual_length += ls_spiio_xfer(priv,
				(u8*)t->tx_buf, (u8*)t->rx_buf, t->len);
	}
	ls_spiio_clear_cs(priv);

	m->complete(m->context);

	return 0;
}

static int drv_ls_spiio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ls_spiio_priv* priv;
	struct spi_controller* controller;
	u32 num_chipselect;
	int ret;

	controller = spi_alloc_master(&pdev->dev, sizeof(*priv));

	if (controller == NULL) {
		dev_dbg(&pdev->dev, "controller allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	if (pdev->id != -1)
		controller->bus_num	= pdev->id;

	controller->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	controller->setup = __spi_setup;
	controller->transfer = __spi_transfer;

	if (of_property_read_u32(pdev->dev.of_node, "num-chipselects",
				&num_chipselect))
		num_chipselect = 1;
	controller->num_chipselect = num_chipselect ? num_chipselect : 1;

#ifdef CONFIG_OF
	controller->dev.of_node = of_node_get(pdev->dev.of_node);
#endif
	dev_set_drvdata(&pdev->dev, controller);

	priv = spi_controller_get_devdata(controller);
	priv->controller = controller;
	priv->dev = dev;

	if (of_find_property(pdev->dev.of_node, "cs-gpios", NULL))
	{
		ret = ls_spiio_init_cs_gpios(priv);
		if (ret)
			goto free_controller;
	}

	priv->reg = devm_platform_ioremap_resource(pdev, 0);
	if (priv->reg == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		ret = -ENXIO;
		goto try_free_cs_gpios;
	}
	if (of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				&priv->ref_clk))
	{
		dev_err(&pdev->dev, "Cannot get clock\n");
		goto try_free_cs_gpios;
	}

	ls_spiio_init(priv);

	ret = spi_register_controller(controller);
	if (ret)
	{
		dev_err(&pdev->dev, "spi register fail\n");
		goto unmap_io;
	}

	return ret;

unmap_io:
	devm_iounmap(&pdev->dev, priv->reg);
try_free_cs_gpios:
	if (of_find_property(pdev->dev.of_node, "cs-gpios", NULL))
		ls_spiio_free_cs_gpios(priv);
free_controller:
	spi_controller_put(controller);
err:
	return ret;
}

#ifdef CONFIG_OF
static struct of_device_id drvid_ls_spiio[] = {
	{ .compatible = "loongson,ls-spi-io", },
	{ },
};
MODULE_DEVICE_TABLE(of, drvid_ls_spiio);
#endif
static struct platform_driver ls_spiio_driver = {
	.probe = drv_ls_spiio_probe,
	.driver	= {
		.name	= "ls-spi-io",
		.owner	= THIS_MODULE,
		.bus = &platform_bus_type,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(drvid_ls_spiio),
#endif
	},
};

static int __init __drv_spi_init(void)
{
	int ret;

	ret =  platform_driver_register(&ls_spiio_driver);
	return ret;
}

static void __exit __drv_spi_exit(void)
{
	platform_driver_unregister(&ls_spiio_driver);
}

subsys_initcall(__drv_spi_init);
module_exit(__drv_spi_exit);

MODULE_AUTHOR("Niuyize <niuyize@loongson.cn>");
MODULE_DESCRIPTION("Loongson SPI driver");
MODULE_LICENSE("GPL");
