// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPMI controller driver for Apple M1 SoC
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/spmi.h>

#define REG_STAT		0x1300
#define  REG_STAT_RX_EMPTY	BIT(24)
#define  REG_STAT_TX_EMPTY	BIT(8)
#define REG_CMD			0x1304
#define  REG_CMD_ADDR_SHIFT	16
#define  REG_CMD_FINAL		BIT(15)
#define  REG_CMD_SID_SHIFT	8
#define REG_RESP		0x1308
#define REG_IRQEN(x)		(0x1320 + (x) * 4)
#define REG_IRQSTAT(x)		(0x1360 + (x) * 4)

#define IRQ_REGS		6
#define MAX_IRQS		(32 * IRQ_REGS)
#define IRQ_LOCAL_REGS		3
#define IRQ_LOCAL0		IRQ_REGS
#define IRQ_LOCAL0_DONE		BIT(0)

#define MSG_SIZE		64
#define TIMEOUT_MS		100

struct apple_m1_spmi_data {
	struct device *dev;
	struct spmi_controller *ctrl;
	struct irq_domain *domain;
	void __iomem *base;
	struct clk_bulk_data *clks;
	spinlock_t lock;
	int num_clks;
	int irq;

	struct mutex mtx;
	struct completion done;
	u32 opc, sid, addr, bytes;
	u32 tx[MSG_SIZE/4];
	unsigned txrp, txmax;
	u32 rx[MSG_SIZE/4];
	u32 ack[MSG_SIZE/32];
	unsigned ackwp;
	unsigned rxwp, rxmax;
};

static void apple_m1_spmi_continue(struct apple_m1_spmi_data *spmi)
{
	u32 cmd;
	unsigned step, i;

	step = spmi->bytes > 8 ? 8 : spmi->bytes;

	cmd = spmi->opc | (step - 1) | (spmi->sid << REG_CMD_SID_SHIFT) |
	      (spmi->addr << REG_CMD_ADDR_SHIFT);
	if(step == spmi->bytes)
		cmd |= REG_CMD_FINAL;
	spmi->addr += step;

	writel(cmd, spmi->base + REG_CMD);
	for(i=0; i<2 && spmi->txrp < spmi->txmax; i++)
		writel(spmi->tx[spmi->txrp++], spmi->base + REG_CMD);
}

static int apple_m1_spmi_cmd_int(struct spmi_controller *ctrl, u8 opc, u8 sid,
	u16 addr, u8 *rxbuf, size_t rxlen, const u8 *txbuf, size_t txlen)
{
	struct apple_m1_spmi_data *spmi = spmi_controller_get_drvdata(ctrl);
	unsigned long timeout = msecs_to_jiffies(TIMEOUT_MS);
	unsigned i, ack;
	int ret;

	if(rxlen + txlen > MSG_SIZE)
		return -EINVAL;

	mutex_lock(&spmi->mtx);
	init_completion(&spmi->done);

	spmi->opc = opc;
	spmi->sid = sid;
	spmi->addr = addr;
	spmi->bytes = txlen | rxlen;
	spmi->txmax = (txlen + 3) >> 2;
	spmi->rxmax = (rxlen + 3) >> 2;
	spmi->txrp = spmi->rxwp = 0;
	memset(spmi->tx, 0, sizeof(spmi->tx));
	for(i=0; i<txlen; i++)
		spmi->tx[i >> 2] |= (u32)txbuf[i] << (i * 8);
	memset(spmi->ack, 0, sizeof(spmi->ack));
	spmi->ackwp = 0;

	apple_m1_spmi_continue(spmi);

	timeout = wait_for_completion_timeout(&spmi->done, timeout);

	for(i=0; i<rxlen; i++)
		rxbuf[i] = spmi->rx[i >> 2] >> (i * 8);

	if(!timeout) {
		dev_warn(spmi->dev, "SPMI command [%02x,%02x,%04x,%zu/%zu] timed out\n",
			 opc, sid, addr, rxlen, txlen);
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
		for(i=0; i<rxlen; i++) {
			ack = (spmi->ack[i >> 5] >> (i & 31)) & 1;
			if(!ack) {
				ret = -EIO;
				break;
			}
		}
		if(ret)
			dev_warn(spmi->dev, "SPMI command [%02x,%02x,%04x,%zu/%zu] returned "
				 "[%08x,%08x]\n", opc, sid, addr, rxlen, txlen,
				 spmi->ack[0], spmi->ack[1]);
	}

	mutex_unlock(&spmi->mtx);

	return ret;
}

static int apple_m1_spmi_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid)
{
	return apple_m1_spmi_cmd_int(ctrl, opc, sid, 0, NULL, 0, NULL, 0);
}

static int apple_m1_spmi_read_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
				  u16 addr, u8 *buf, size_t len)
{
	return apple_m1_spmi_cmd_int(ctrl, opc, sid, addr, buf, len, NULL, 0);
}

static int apple_m1_spmi_write_cmd(struct spmi_controller *ctrl, u8 opc, u8 sid,
				   u16 addr, const u8 *buf, size_t len)
{
	return apple_m1_spmi_cmd_int(ctrl, opc, sid, addr, NULL, 0, buf, len);
}

static void apple_m1_spmi_chained_irq(struct irq_desc *desc)
{
	struct apple_m1_spmi_data *spmi = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned i, b, irq;
	u32 stat[IRQ_REGS + IRQ_LOCAL_REGS], s, val;

	chained_irq_enter(chip, desc);

	for(i=0; i<IRQ_REGS+IRQ_LOCAL_REGS; i++)
		stat[i] = readl(spmi->base + REG_IRQSTAT(i)) & readl(spmi->base + REG_IRQEN(i));
	for(i=IRQ_REGS; i<IRQ_REGS+IRQ_LOCAL_REGS; i++)
		writel(stat[i], spmi->base + REG_IRQSTAT(i));

	for(i=0; i<IRQ_REGS; i++) {
		s = stat[i];
		while(s) {
			b = ffs(s) - 1;
			s &= s - 1;
			irq = irq_find_mapping(spmi->domain, i * 32 + b);
			if(irq) {
				generic_handle_irq(irq);
				continue;
			}
			spin_lock(&spmi->lock);
			val = readl(spmi->base + REG_IRQEN(i));
			val &= ~(1 << b);
			writel(val, spmi->base + REG_IRQEN(i));
			spin_unlock(&spmi->lock);
			writel(1 << b, spmi->base + REG_IRQSTAT(i));
			dev_warn(spmi->dev, "spurious SPMI IRQ %d\n", i * 32 + b);
		}
	}

	if(stat[IRQ_LOCAL0] & IRQ_LOCAL0_DONE) {
		spin_lock(&spmi->lock);
		i = s = 0;
		while(!(readl(spmi->base + REG_STAT) & REG_STAT_RX_EMPTY)) {
			val = readl(spmi->base + REG_RESP);
			if(i) {
				if(spmi->rxwp < spmi->rxmax)
					spmi->rx[spmi->rxwp++] = val;
			} else
				s = val;
			i ++;
		}
		if(i) {
			spmi->ack[spmi->ackwp >> 5] |= ((s >> 16) & 0xFF) << (spmi->ackwp & 31);
			spmi->ackwp += 8;
			if(spmi->rxwp < spmi->rxmax || spmi->txrp < spmi->txmax)
				apple_m1_spmi_continue(spmi);
			else
				complete(&spmi->done);
		} else
			dev_warn(spmi->dev, "spurious SPMI completion IRQ\n");
		spin_unlock(&spmi->lock);
	}

	chained_irq_exit(chip, desc);
}

static void apple_m1_spmi_irq_ack(struct irq_data *d)
{
	struct apple_m1_spmi_data *spmi = irq_data_get_irq_chip_data(d);
	unsigned irq = d->hwirq;

	writel(1 << (irq & 31), spmi->base + REG_IRQSTAT(irq >> 5));
}

static void apple_m1_spmi_irq_mask(struct irq_data *d)
{
	struct apple_m1_spmi_data *spmi = irq_data_get_irq_chip_data(d);
	unsigned irq = d->hwirq;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&spmi->lock, flags);
	val = readl(spmi->base + REG_IRQEN(irq >> 5));
	val &= ~(1 << (irq & 31));
	writel(val, spmi->base + REG_IRQEN(irq >> 5));
	spin_unlock_irqrestore(&spmi->lock, flags);
}

static void apple_m1_spmi_irq_unmask(struct irq_data *d)
{
	struct apple_m1_spmi_data *spmi = irq_data_get_irq_chip_data(d);
	unsigned irq = d->hwirq;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&spmi->lock, flags);
	val = readl(spmi->base + REG_IRQEN(irq >> 5));
	val |= (1 << (irq & 31));
	writel(val, spmi->base + REG_IRQEN(irq >> 5));
	spin_unlock_irqrestore(&spmi->lock, flags);
}

static struct irq_chip apple_m1_spmi_irqchip = {
	.name			= "SPMI",
	.irq_ack		= apple_m1_spmi_irq_ack,
	.irq_mask		= apple_m1_spmi_irq_mask,
	.irq_unmask		= apple_m1_spmi_irq_unmask,
	.flags			= IRQCHIP_MASK_ON_SUSPEND,
};

static int apple_m1_spmi_irq_domain_translate(struct irq_domain *domain, struct irq_fwspec *fwspec,
					      unsigned long *out_hwirq, unsigned *out_type)
{
	if(fwspec->param[0] > MAX_IRQS)
		return -EINVAL;

	*out_hwirq = fwspec->param[0];
	*out_type  = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;
	return 0;
}

static int apple_m1_spmi_irq_domain_alloc(struct irq_domain *domain, unsigned virq,
					  unsigned nr_irqs, void *data)
{
	struct apple_m1_spmi_data *spmi = domain->host_data;
	struct irq_fwspec *fwspec = data;
	irq_flow_handler_t handler;
	irq_hw_number_t hwirq;
	unsigned type;
	int ret, i;

	ret = apple_m1_spmi_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if(ret)
		return ret;

	handler = (type & IRQ_TYPE_EDGE_BOTH) ? handle_edge_irq : handle_level_irq;

	for(i=0; i<nr_irqs; i++)
		irq_domain_set_info(domain, virq + i, hwirq + i, &apple_m1_spmi_irqchip, spmi,
				    handler, NULL, NULL);
	return 0;
}

static const struct irq_domain_ops apple_m1_spmi_irq_domain_ops = {
	.alloc = apple_m1_spmi_irq_domain_alloc,
	.free = irq_domain_free_irqs_common,
	.translate = apple_m1_spmi_irq_domain_translate,
};

static void apple_m1_spmi_setup(struct apple_m1_spmi_data *spmi)
{
	unsigned i;

	for(i=0; i<IRQ_REGS+IRQ_LOCAL_REGS; i++) {
		writel(0, spmi->base + REG_IRQEN(i));
		writel(-1, spmi->base + REG_IRQSTAT(i));
	}

	writel(IRQ_LOCAL0_DONE, spmi->base + REG_IRQEN(IRQ_LOCAL0));
}

static int apple_m1_spmi_probe(struct platform_device *pdev)
{
	struct spmi_controller *ctrl;
	struct apple_m1_spmi_data *spmi;
	struct resource *rsrc;
	int err = -EINVAL;

	ctrl = spmi_controller_alloc(&pdev->dev, sizeof(*spmi));
	if(!ctrl)
		return -ENOMEM;

	spmi = spmi_controller_get_drvdata(ctrl);
	spmi->ctrl = ctrl;
	spmi->dev = &pdev->dev;

	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!rsrc) {
		dev_err(&pdev->dev, "missing MMIO range in DT.\n");
		goto free_ctrl;
	}
	spmi->base = ioremap(rsrc->start, resource_size(rsrc));
	if(IS_ERR(spmi->base)) {
		err = PTR_ERR(spmi->base);
		dev_err(&pdev->dev, "failed to map MMIO: %d.\n", err);
		goto free_ctrl;
	}

	err = clk_bulk_get_all(spmi->dev, &spmi->clks);
	if(err < 0) {
		dev_err(&pdev->dev, "clk_bulk_get_all failed.\n");
		goto free_rsrc;
	}
	spmi->num_clks = err;
	err = clk_bulk_prepare_enable(spmi->num_clks, spmi->clks);
	if(err) {
		dev_err(&pdev->dev, "clk_bulk_prepare_enable failed.\n");
		goto free_clks;
	}

	spmi->irq = platform_get_irq(pdev, 0);
	if(spmi->irq < 0) {
		dev_err(&pdev->dev, "missing IRQ in DT.\n");
		err = -EINVAL;
		goto disable_clks;
	}

	platform_set_drvdata(pdev, ctrl);
	spin_lock_init(&spmi->lock);
	mutex_init(&spmi->mtx);
	init_completion(&spmi->done);

	apple_m1_spmi_setup(spmi);

	spmi->domain = irq_domain_add_tree(pdev->dev.of_node, &apple_m1_spmi_irq_domain_ops, spmi);
	if(!spmi->domain) {
		dev_err(&pdev->dev, "unable to create irq_domain.\n");
		err = -ENOMEM;
		goto disable_clks;
	}

	irq_set_chained_handler_and_data(spmi->irq, apple_m1_spmi_chained_irq, spmi);

	ctrl->cmd = apple_m1_spmi_cmd;
	ctrl->read_cmd = apple_m1_spmi_read_cmd;
	ctrl->write_cmd = apple_m1_spmi_write_cmd;
	err = spmi_controller_add(ctrl);
	if(err)
		goto free_domain;

	return 0;

free_domain:
	irq_set_chained_handler_and_data(spmi->irq, NULL, NULL);
	irq_domain_remove(spmi->domain);
disable_clks:
	clk_bulk_disable_unprepare(spmi->num_clks, spmi->clks);
free_clks:
	clk_bulk_put_all(spmi->num_clks, spmi->clks);
free_rsrc:
	iounmap(spmi->base);
free_ctrl:
	spmi_controller_put(ctrl);
	return err;
}

static const struct of_device_id apple_m1_spmi_match[] = {
	{ .compatible = "apple,spmi-m1" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_m1_spmi_match);

static struct platform_driver apple_m1_spmi_driver = {
	.driver = {
		.name = "spmi-apple-m1",
		.of_match_table = apple_m1_spmi_match,
	},
	.probe = apple_m1_spmi_probe,
};

module_platform_driver(apple_m1_spmi_driver);

MODULE_DESCRIPTION("Apple M1 SPMI driver");
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL");
