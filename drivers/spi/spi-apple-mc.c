// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPIMC controller driver for Apple M1 SoC
 *
 * Copyright (C) 2020-21 Corellium LLC
 *
 * Based on spi-mt7621.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

#define REG_CLKCFG			0x00
#define  REG_CLKCFG_ENABLE		0xD
#define REG_CONFIG			0x04
#define  REG_CONFIG_AUTOTX		BIT(0)
#define  REG_CONFIG_PIOEN		BIT(5)
#define  REG_CONFIG_IE_RXRDY		BIT(7)
#define  REG_CONFIG_IE_TXEMPTY		BIT(8)
#define  REG_CONFIG_IE_COMPL		BIT(21)
#define  REG_CONFIG_SET			0x00040000
#define REG_STATUS			0x08
#define  REG_STATUS_RXRDY		BIT(0)
#define  REG_STATUS_TXEMPTY		BIT(1)
#define  REG_STATUS_COMPL		BIT(22)
#define REG_PIN				0x0C
#define  REG_PIN_CS			BIT(1)
#define REG_TXDATA			0x10
#define REG_RXDATA			0x20
#define REG_CLKDIV			0x30
#define  REG_CLKDIV_MAX			2047
#define REG_RXCNT			0x34
#define REG_CLKIDLE			0x38
#define REG_TXCNT			0x4C
#define REG_AVAIL			0x10C
#define  REG_AVAIL_TXFIFO_MASK		(255 << 8)
#define   REG_AVAIL_TXFIFO_SHIFT	8
#define  REG_AVAIL_RXFIFO_MASK		(255 << 24)
#define   REG_AVAIL_RXFIFO_SHIFT	24
#define REG_HWDLY_CFG			0x150
#define  REG_HWDLY_CFG_EN		BIT(24)
#define REG_SPCON			0x154
#define  REG_SPCON_MODE			BIT(8)
#define REG_HWDLY_PRE			0x160
#define  REG_HWDLY_PRE_EN		BIT(0)
#define  REG_HWDLY_PRE_MAGIC		0x823
#define  REG_HWDLY_PRE_SHIFT		16
#define REG_HWDLY_POST			0x168
#define  REG_HWDLY_POST_EN		BIT(0)
#define  REG_HWDLY_POST_MAGIC		0x423
#define  REG_HWDLY_POST_SHIFT		16
#define REG_HWDLY_POST_2		0x194

#define TIMEOUT_MS			100

#define FIFO_SIZE			16

#define MAX_SEG_SIZE			16384

struct apple_spimc {
	struct spi_controller *master;
	struct device *dev;
	void __iomem *base;
	unsigned int clkfreq;
	unsigned int speed;
	struct clk *clk;
	struct gpio_descs *csgpio;

	spinlock_t lock;
	const unsigned char *tx_buf;
	unsigned char *rx_buf;
	unsigned int tx_compl, rx_compl, len;
	struct completion done;
};

static inline struct apple_spimc *spidev_to_apple_spimc(struct spi_device *spi)
{
	return spi_controller_get_devdata(spi->master);
}

static void apple_spimc_continue_tx(struct apple_spimc *spi, unsigned avail, unsigned prime)
{
	unsigned maxtx = (avail & REG_AVAIL_TXFIFO_MASK) >> REG_AVAIL_TXFIFO_SHIFT;
	unsigned data, maxrx, cfg;

	maxtx = FIFO_SIZE - maxtx;
	if(!prime) {
		maxrx = (avail & REG_AVAIL_RXFIFO_MASK) >> REG_AVAIL_RXFIFO_SHIFT;
		if(maxrx < maxtx)
			maxtx = maxrx;
	}

	while(spi->tx_compl < spi->len && maxtx) {
		if(spi->tx_buf)
			data = spi->tx_buf[spi->tx_compl];
		else
			data = 0x00;
		writel(data, spi->base + REG_TXDATA);
		spi->tx_compl ++;
		maxtx --;
	}

	cfg = REG_CONFIG_SET | REG_CONFIG_PIOEN | REG_CONFIG_IE_RXRDY;
	if(spi->tx_compl < spi->len)
		cfg |= REG_CONFIG_IE_TXEMPTY;
	writel(cfg, spi->base + REG_CONFIG);
}

static int apple_spimc_continue_rx(struct apple_spimc *spi, unsigned avail)
{
	unsigned maxrx = (avail & REG_AVAIL_RXFIFO_MASK) >> REG_AVAIL_RXFIFO_SHIFT;
	unsigned data;

	while(spi->rx_compl < spi->len && maxrx) {
		data = readl(spi->base + REG_RXDATA);
		if(spi->rx_buf)
			spi->rx_buf[spi->rx_compl] = data;
		spi->rx_compl ++;
		maxrx --;
	}

	if(spi->rx_compl >= spi->len) {
		writel(REG_CONFIG_SET, spi->base + REG_CONFIG);
		writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY,
			spi->base + REG_STATUS);
		return 1;
	}
	return 0;
}

static irqreturn_t apple_spimc_irq(int irq, void *dev_id)
{
	struct apple_spimc *spi = dev_id;
	unsigned long flags;
	unsigned status, avail, done;

	spin_lock_irqsave(&spi->lock, flags);

	status = readl(spi->base + REG_STATUS);
	avail = readl(spi->base + REG_AVAIL);
	writel(status, spi->base + REG_STATUS);

	done = apple_spimc_continue_rx(spi, avail);
	apple_spimc_continue_tx(spi, avail, 0);
	if(done)
		complete(&spi->done);

	spin_unlock_irqrestore(&spi->lock, flags);

	return IRQ_HANDLED;
}

static void apple_spimc_set_cs(struct spi_device *spid, int enable)
{
	struct apple_spimc *spi = spidev_to_apple_spimc(spid);
	int cs = spid->chip_select;

	if(!spi->csgpio || cs >= spi->csgpio->ndescs)
		return;

	gpiod_direction_output(spi->csgpio->desc[cs], enable);
}

static int apple_spimc_prepare(struct spi_device *spid, unsigned int speed)
{
	struct apple_spimc *spi = spidev_to_apple_spimc(spid);
	u32 rate, avail;

	rate = DIV_ROUND_UP(spi->clkfreq, speed);
	if(rate > REG_CLKDIV_MAX + 1)
		return -EINVAL;
	if(rate < 2)
		rate = 2;

	writel(0, spi->base + REG_CLKCFG);
	writel(0, spi->base + REG_PIN);

	if(readl(spi->base + REG_RXCNT))
		writel(0, spi->base + REG_RXCNT);
	if(readl(spi->base + REG_TXCNT))
		writel(0, spi->base + REG_TXCNT);
	avail = readl(spi->base + REG_AVAIL);
	avail = (avail & REG_AVAIL_RXFIFO_MASK) >> REG_AVAIL_RXFIFO_SHIFT;
	while(avail --)
		readl(spi->base + REG_RXDATA);

	writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY, spi->base + REG_STATUS);
	writel(rate, spi->base + REG_CLKDIV);
	writel(0, spi->base + REG_CLKIDLE);
	writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY, spi->base + REG_STATUS);
	writel(readl(spi->base + REG_SPCON) & ~REG_SPCON_MODE, spi->base + REG_SPCON);
	writel(REG_CONFIG_SET, spi->base + REG_CONFIG);
	writel(REG_CLKCFG_ENABLE, spi->base + REG_CLKCFG);
	writel(readl(spi->base + REG_HWDLY_CFG) & ~REG_HWDLY_CFG_EN, spi->base + REG_HWDLY_CFG);
	writel(readl(spi->base + REG_HWDLY_PRE) & ~REG_HWDLY_PRE_EN, spi->base + REG_HWDLY_PRE);
	writel(readl(spi->base + REG_HWDLY_POST) & ~REG_HWDLY_POST_EN, spi->base + REG_HWDLY_POST);
	readl(spi->base + REG_CONFIG);

	spi->speed = speed;

	return 0;
}

static int apple_spimc_transfer_one_message(struct spi_controller *master, struct spi_message *m)
{
	unsigned long timeout;
	struct apple_spimc *spi = spi_controller_get_devdata(master);
	struct spi_device *spid = m->spi;
	unsigned int speed = spid->max_speed_hz, avail;
	struct spi_transfer *t = NULL;
	int status = 0, seg, offs;
	unsigned long flags, cs_set = 0;

	list_for_each_entry(t, &m->transfers, transfer_list)
		if(t->speed_hz < speed && t->speed_hz)
			speed = t->speed_hz;

	if(apple_spimc_prepare(spid, speed)) {
		status = -EIO;
		goto msg_done;
	}

	m->actual_length = 0;
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if(!cs_set && (t->len || !t->cs_change)) {
			apple_spimc_set_cs(spid, 1);
			cs_set = 1;
		}

		for(offs=0; offs<t->len; offs+=seg) {
			seg = t->len - offs;
			if(seg > MAX_SEG_SIZE)
				seg = MAX_SEG_SIZE;

			spin_lock_irqsave(&spi->lock, flags);

			reinit_completion(&spi->done);

			spi->len = seg;
			spi->tx_compl = spi->rx_compl = 0;
			spi->tx_buf = t->tx_buf ? (t->tx_buf + offs) : NULL;
			spi->rx_buf = t->rx_buf ? (t->rx_buf + offs) : NULL;

			writel(spi->len, spi->base + REG_TXCNT);
			writel(spi->len, spi->base + REG_RXCNT);
			writel(REG_CONFIG_SET | REG_CONFIG_PIOEN, spi->base + REG_CONFIG);

			avail = readl(spi->base + REG_AVAIL);
			apple_spimc_continue_tx(spi, avail, 1);

			spin_unlock_irqrestore(&spi->lock, flags);

			timeout = msecs_to_jiffies(TIMEOUT_MS);
			timeout = wait_for_completion_timeout(&spi->done, timeout);

			spin_lock_irqsave(&spi->lock, flags);

			writel(REG_CONFIG_SET, spi->base + REG_CONFIG);
			writel(REG_STATUS_COMPL | REG_STATUS_TXEMPTY | REG_STATUS_RXRDY,
				spi->base + REG_STATUS);

			if(timeout == 0) {
				dev_err(&spid->dev, "transfer timed out with %d/%d remaining (of %d).\n",
					spi->len - spi->tx_compl, spi->len - spi->rx_compl, spi->len);
				status = -ETIMEDOUT;
			}

			m->actual_length += spi->len;

			spin_unlock_irqrestore(&spi->lock, flags);

			if(status)
				break;
		}

		if(status)
			break;

		if(t->delay_usecs)
			udelay(t->delay_usecs);
		spi_delay_exec(&t->delay, t);

		if(t->cs_change) {
			apple_spimc_set_cs(spid, 0);
			if(spi_delay_exec(&t->cs_change_delay, t) < 0)
				udelay(50);
			apple_spimc_set_cs(spid, 1);
			cs_set = 1;
		}
	}

	apple_spimc_set_cs(spid, 0);

	writel(0, spi->base + REG_CLKCFG);

msg_done:
	m->status = status;
	spi_finalize_current_message(master);

	return 0;
}

static int apple_spimc_setup(struct spi_device *spid)
{
	struct apple_spimc *spi = spidev_to_apple_spimc(spid);

	if(!spid->max_speed_hz || spid->max_speed_hz > (spi->clkfreq / 2))
		spid->max_speed_hz = spi->clkfreq / 2;

	if(spid->max_speed_hz < spi->clkfreq / (REG_CLKDIV_MAX + 1)) {
		dev_err(&spid->dev, "setup: requested speed is too low: %d Hz\n",
			spid->max_speed_hz);
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id apple_spimc_match[] = {
	{ .compatible = "apple,spi-mc-m1" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_spimc_match);

static int apple_spimc_probe(struct platform_device *pdev)
{
	struct spi_controller *master;
	struct apple_spimc *spi;
	void __iomem *base;
	struct clk *clk;
	struct gpio_descs *csgpio = NULL;
	int ret, ncs, irq;

	base = devm_platform_ioremap_resource(pdev, 0);
	if(IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get(&pdev->dev, NULL);
	if(IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get clock: %ld.\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	ncs = gpiod_count(&pdev->dev, "cs");
	if(ncs > 0) {
		csgpio = devm_gpiod_get_array(&pdev->dev, "cs", 0);
		if(IS_ERR(csgpio)) {
			if(PTR_ERR(csgpio) != -EPROBE_DEFER)
				dev_err(&pdev->dev, "failed to get chip select gpios: %ld\n",
					PTR_ERR(csgpio));
			return PTR_ERR(csgpio);
		}
	} else
		ncs = 0;

	ret = clk_prepare_enable(clk);
	if(ret)
		return ret;

	master = spi_alloc_master(&pdev->dev, sizeof(*spi));
	if(!master) {
		dev_err(&pdev->dev, "master allocation failed.\n");
		return -ENOMEM;
	}

	master->mode_bits = SPI_LSB_FIRST;
	master->flags = 0;
	master->setup = apple_spimc_setup;
	master->transfer_one_message = apple_spimc_transfer_one_message;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->dev.of_node = pdev->dev.of_node;
	master->num_chipselect = ncs ? ncs : 1;

	dev_set_drvdata(&pdev->dev, master);

	spi = spi_controller_get_devdata(master);
	spi->dev = &pdev->dev;
	spi->base = base;
	spi->clk = clk;
	spi->master = master;
	spi->clkfreq = clk_get_rate(spi->clk);
	spi->csgpio = csgpio;

	dev_err(&pdev->dev, "Apple SPI-MC at %d MHz, %d chip select GPIO%s.\n",
		spi->clkfreq / 1000000, ncs, ncs == 1 ? "" : "s");

	spin_lock_init(&spi->lock);
	init_completion(&spi->done);

	irq = platform_get_irq(pdev, 0);
	if(irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, apple_spimc_irq, 0, dev_name(&pdev->dev), spi);
	if(ret < 0)
		return ret;

	return devm_spi_register_controller(&pdev->dev, master);
}

static int apple_spimc_remove(struct platform_device *pdev)
{
	struct spi_controller *master;
	struct apple_spimc *spi;

	master = dev_get_drvdata(&pdev->dev);
	spi = spi_controller_get_devdata(master);

	clk_disable_unprepare(spi->clk);

	return 0;
}

static struct platform_driver apple_spimc_driver = {
	.driver = {
		.name = "spi-apple-mc",
		.of_match_table = apple_spimc_match,
	},
	.probe = apple_spimc_probe,
	.remove = apple_spimc_remove,
};

module_platform_driver(apple_spimc_driver);

MODULE_DESCRIPTION("Apple SoC SPI-MC driver");
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL");
