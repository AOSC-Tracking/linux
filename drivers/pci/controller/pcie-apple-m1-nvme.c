// SPDX-License-Identifier: GPL-2.0-only
/*
 * NVMe controller driver for Apple M1 SoC via ANS
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/apple_iop_mapper.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pci-ecam.h>
#include <linux/msi.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/nvme.h>

/*
 * Quick explanation of this driver:
 *
 * M1 contains a coprocessor, called ANS, that fronts the actual PCIe
 * bus to the NVMe device.  This coprocessor does not, unfortunately,
 * expose a PCIe like interface.  But it does have a MMIO range that is
 * a mostly normal NVMe BAR, with a few quirks (handled in NVMe code).
 *
 * So, to reduce code duplication in NVMe code (to add a non-PCI backend)
 * we add a synthetic PCI bus for this device.
 */

#define APPLE_MAX_PEND_CMDS		0x1210
#define   APPLE_MAX_PEND_CMDS_VAL	((64 << 16) | 64)
#define APPLE_BOOT_STATUS		0x1300
#define   APPLE_BOOT_STATUS_OK		0xde71ce55
#define APPLE_BASE_CMD_ID		0x1308
#define   APPLE_BASE_CMD_ID_MASK	0xffff
#define APPLE_LINEAR_SQ_CTRL		0x24908
#define   APPLE_LINEAR_SQ_CTRL_EN	BIT(0)

#define NUM_VPCI			2
#define CFGREG_SIZE			128


struct apple_m1_ans {
	struct device *dev;
	void __iomem *nvme, *sart;
	struct mbox_client mbox;
	struct mbox_chan *chan;
	struct clk_bulk_data *clks;
	int num_clks;
	int irq;
	spinlock_t lock;

	struct pci_host_bridge *bridge;
	struct irq_domain *irq_dom;
	u64 config[NUM_VPCI][CFGREG_SIZE / 8];

	u32 basecmd;
};

/* organized as value / writable mask pairs */
static const u64 apple_m1_ans_config[NUM_VPCI][CFGREG_SIZE / 4] = {
	{ /* root bridge */
		/* Status | Command | Device ID | Vendor ID */
		0x00000006ff80106b, 0x0000000600000000,
		/* BIST | Header type | Latency timer 1 | Cacheline size | Class code | Rev ID */
		0x0001000006040001, 0x0000ff0000000000,
		/* BAR 1 | BAR 0 */
		0x0000000000000000, 0x0000000000000000,
		/* 2ndary status | I/O limit | I/O base |
		   Latency timer 2 | Subordinate bus | Secondary bus | Primary bus */
		0x000000f000010100, 0x00000000ffffffff,
		/* Prefetchable limit | Prefetchable base | Memory limit | Memory base */
		0x0000fff000000000, 0x00000000fff0fff0,
		/* Prefetchable limit [hi] | Prefetchable base [hi] */
		0x00000000ffffffff, 0x0000000000000000,
		/* [reserved] | Cap pointer | I/O limit [hi] | I/O base [hi] */
		0x000000000000ffff, 0x0000000000000000,
		/* Bridge control | IRQ pin | IRQ line | Expansion ROM base */
		0x000000ff00000000, 0x0000000000000000,
	},
	{ /* NVMe device */
		/* Status | Command | Device ID | Vendor ID */
		0x00100000ff81106b, 0x0000000600000000,
		/* BIST | Header type | Latency timer 1 | Cacheline size | Class code | Rev ID */
		0x0000001001080211, 0x0000ff0000000000,
		/* BAR 1 | BAR 0 */
		0x0000000000000000, 0x00000000fffc0000,
		/* BAR 3 | BAR 2 */
		0x0000000000000000, 0x0000000000000000,
		/* BAR 5 | BAR 4 */
		0x0000000000000000, 0x0000000000000000,
		/* Subsystem ID | Subsystem vendor ID | Cardbus CIS pointer */
		0x2077106b00000000, 0x0000000000000000,
		/* Expansion ROM base */
		0x0000004000000000, 0x0000000000000000,
		/* Max_lat | Min_gnt | IRQ pin | IRQ line | [reserved] */
		0x000000ff00000000, 0x0000000000000000,
		/* Message address | Message control | Next cap | MSI cap ID */
		0xfffff00000810005, 0xffffffff00010000,
		/* [reserved] | Message data | Message address [hi] */
		0x0000000000000000, 0x0000ffffffffffff,
	}
};

static struct irq_chip apple_m1_ans_irq_chip = {
	.name = "ANS IRQ",
	.irq_ack = irq_chip_ack_parent,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
};

static void apple_m1_ans_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct apple_m1_ans *ans = irq_desc_get_handler_data(desc);
	unsigned virq;

	chained_irq_enter(chip, desc);
	virq = irq_find_mapping(ans->irq_dom, 0);
	generic_handle_irq(virq);
	chained_irq_exit(chip, desc);
}

static void apple_m1_ans_compose_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	msg->address_lo = 0xfffff000;
	msg->address_hi = 0;
	msg->data = d->hwirq;
}

static int apple_m1_ans_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void apple_m1_ans_ack_irq(struct irq_data *d)
{
	/* should be already acked at AIC level since this is a straight passthrough */
}

static void apple_m1_ans_mask_irq(struct irq_data *d)
{
	struct apple_m1_ans *ans = d->chip_data;
	disable_irq(ans->irq);
}

static void apple_m1_ans_unmask_irq(struct irq_data *d)
{
	struct apple_m1_ans *ans = d->chip_data;
	enable_irq(ans->irq);
}

static struct irq_chip apple_m1_ans_msi_chip = {
	.name = "ANS",
	.irq_ack = apple_m1_ans_ack_irq,
	.irq_mask = apple_m1_ans_mask_irq,
	.irq_unmask = apple_m1_ans_unmask_irq,
	.irq_compose_msi_msg = apple_m1_ans_compose_msi_msg,
	.irq_set_affinity = apple_m1_ans_set_affinity,
};

static struct msi_domain_info apple_m1_ans_msi_dom_info = {
	.flags = MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS,
	.chip = &apple_m1_ans_irq_chip,
};

static int apple_m1_ans_irq_domain_alloc(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs, void *args)
{
	struct apple_m1_ans *ans = dom->host_data;
	u32 busdevfn = ((u32 *)args)[2];
	unsigned bus = (busdevfn >> 19) & 255;
	if(bus < 1 || nr_irqs > 1)
		return -ENOSPC;
	irq_domain_set_info(dom, virq, 0, &apple_m1_ans_msi_chip, ans, handle_edge_irq, NULL, NULL);
	return 0;
}

static void apple_m1_ans_irq_domain_free(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs)
{
}

static const struct irq_domain_ops apple_m1_ans_irq_dom_ops = {
	.alloc = apple_m1_ans_irq_domain_alloc,
	.free = apple_m1_ans_irq_domain_free,
};

int apple_m1_ans_config_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	unsigned vpci = bus->number;
	struct apple_m1_ans *ans = bus->sysdata;
	unsigned long flags;
	u64 data;


	if(devfn != 0 || vpci >= NUM_VPCI) {
		*val = 0xFFFFFFFF;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	*val = 0;
	if((where & (size - 1)) || size > 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	if(where >= CFGREG_SIZE || size >= CFGREG_SIZE - size)
		return PCIBIOS_SUCCESSFUL;

	spin_lock_irqsave(&ans->lock, flags);

	data = ans->config[vpci][where >> 3];
	data >>= (where & 7) * 8;
	*val = data & (BIT_MASK(size * 8) - 1);

	spin_unlock_irqrestore(&ans->lock, flags);

	return 0;
}

int apple_m1_ans_config_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	unsigned vpci = bus->number;
	struct apple_m1_ans *ans = bus->sysdata;
	unsigned long flags;
	u64 data, mask;

	if(devfn != 0 || vpci >= NUM_VPCI)
		return PCIBIOS_DEVICE_NOT_FOUND;
	if((where & (size - 1)) || size > 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	if(where >= CFGREG_SIZE || size >= CFGREG_SIZE - size)
		return PCIBIOS_SUCCESSFUL;

	spin_lock_irqsave(&ans->lock, flags);

	data = val;
	mask = BIT_MASK(size * 8) - 1;
	data <<= (where & 7) * 8;
	mask <<= (where & 7) * 8;
	mask &= apple_m1_ans_config[vpci][(where >> 2) | 1];
	ans->config[vpci][where >> 3] = (ans->config[vpci][where >> 3] & ~mask) | (data & mask);

	spin_unlock_irqrestore(&ans->lock, flags);

	return 0;
}

static struct pci_ops apple_m1_ans_pci_ops = {
	.read = apple_m1_ans_config_read,
	.write = apple_m1_ans_config_write,
};

static int apple_m1_ans_prepare(struct apple_m1_ans *ans)
{
	int ret;
	u32 val;
	u64 msg[2] = { -1ull, -1ull };

	ret = mbox_send_message(ans->chan, msg);
	if(ret < 0) {
		dev_err(ans->dev, "ANS mailbox startup failed: %d.\n", ret);
		return ret;
	}

	ret = readl_poll_timeout(ans->nvme + APPLE_BOOT_STATUS, val, (val == APPLE_BOOT_STATUS_OK), 100, 500000);
	if(ret < 0) {
		dev_err(ans->dev, "ANS NVMe startup timed out (0x%x).\n", val);
		return ret;
	}

	ans->basecmd = readl(ans->nvme + APPLE_BASE_CMD_ID) & APPLE_BASE_CMD_ID_MASK;

	writel(APPLE_MAX_PEND_CMDS_VAL, ans->nvme + APPLE_MAX_PEND_CMDS);

	writel(readl(ans->nvme + 0x24004) | 0x1000, ans->nvme + 0x24004);
	writel(APPLE_LINEAR_SQ_CTRL_EN, ans->nvme + APPLE_LINEAR_SQ_CTRL);
	writel(readl(ans->nvme + 0x24008) & ~0x800, ans->nvme + 0x24008);
	/* set command permissions */
	writel(0x102, ans->nvme + 0x24118);
	writel(0x102, ans->nvme + 0x24108);
	writel(0x102, ans->nvme + 0x24420);
	writel(0x102, ans->nvme + 0x24414);
	writel(0x10002, ans->nvme + 0x2441c);
	writel(0x10002, ans->nvme + 0x24418);
	writel(0x10002, ans->nvme + 0x24144);
	writel(0x10002, ans->nvme + 0x24524);
	writel(0x102, ans->nvme + 0x24508);
	writel(0x10002, ans->nvme + 0x24504);

	dev_info(ans->dev, "ANS NVMe startup done, base command %d.\n", ans->basecmd);

	return 0;
}

/* NVMe CAP: 00000020.f0010040 VS: 00010100 CC: 00474000 CSTS: 00000000 */

static u64 apple_iop_ans_mapper_func(void *priv, u64 base, u64 size)
{
	struct apple_m1_ans *ans = priv;
	unsigned i;
	u32 val, pbase, psize;

	for(i=0; i<16; i++) {
		val = readl(ans->sart + 4 * i);
		if(!(val >> 24))
			break;
	}

	if(i >= 16) {
		dev_err(ans->dev, "out of SART ranges mapping %d bytes.\n", (int)size);
		return 0;
	}

	pbase = base >> 12;
	psize = (size + (base & 0xfff) + 0xfff) >> 12;
#if 0
	dev_info(ans->dev, "mapping %llx:%x as SART %d %08x:%08x\n", (unsigned long long)base, (unsigned)size, i, pbase, psize);
#endif
	writel(pbase, ans->sart + 4 * i + 0x40);
	writel(0xff000000 | psize, ans->sart + 4 * i);
	readl(ans->sart + 4 * i);

	return base;
}

static void apple_m1_ans_mbox_msg(struct mbox_client *cl, void *msg)
{
}

static void apple_m1_ans_free_channel(void *data)
{
	struct apple_m1_ans *ans = data;
	mbox_free_channel(ans->chan);
}

static void apple_m1_ans_clear_mapper(void *data)
{
	struct apple_m1_ans *ans = data;
	apple_iop_set_mapper_func(ans->chan, NULL, NULL);
}

static const struct of_device_id apple_m1_ans_match[] = {
	{ .compatible = "apple,nvme-m1" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_m1_ans_match);

static int apple_m1_ans_probe(struct platform_device *pdev)
{
	struct fwnode_handle *fwnode = of_node_to_fwnode(pdev->dev.of_node);
	struct apple_m1_ans *ans;
	struct resource *res;
	struct pci_host_bridge *bridge;
	struct irq_domain *msi_dom, *irq_dom;
	void __iomem *base;
	u32 bus_base;
	int err, i, j;

	bridge = devm_pci_alloc_host_bridge(&pdev->dev, sizeof(*ans));
	if(!bridge)
		return -ENOMEM;
	ans = pci_host_bridge_priv(bridge);
	ans->bridge = bridge;
	ans->dev = &pdev->dev;
	platform_set_drvdata(pdev, ans);
	spin_lock_init(&ans->lock);

	dma_set_mask_and_coherent(ans->dev, DMA_BIT_MASK(64));

	for(i=0; i<2; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if(!res) {
			dev_err(&pdev->dev, "missing MMIO range %d.\n", i);
			return -EINVAL;
		}
		base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
		if(IS_ERR(base)) {
			err = PTR_ERR(base);
			dev_err(&pdev->dev, "failed to map MMIO %d: %d.\n", i, err);
			return err;
		}
		if(i == 0) {
			ans->nvme = base;
			bus_base = res->start;
		} else
			ans->sart = base;
	}

	err = devm_clk_bulk_get_all(ans->dev, &ans->clks);
	if(err < 0) {
		dev_err(&pdev->dev, "clk_bulk_get_all failed.\n");
		return err;
	}
	ans->num_clks = err;
	err = clk_bulk_prepare_enable(ans->num_clks, ans->clks);
	if(err) {
		dev_err(&pdev->dev, "clk_bulk_prepare_enable failed.\n");
		return err;
	}

	ans->irq = platform_get_irq(pdev, 0);
	if(ans->irq < 0) {
		dev_err(&pdev->dev, "failed to map IRQ\n");
		return -EINVAL;
	}

	ans->mbox.dev = ans->dev;
	ans->mbox.rx_callback = apple_m1_ans_mbox_msg;
	ans->mbox.tx_block = true;
	ans->mbox.tx_tout = 500;
	ans->chan = mbox_request_channel(&ans->mbox, 0);
	if(IS_ERR(ans->chan)) {
		err = PTR_ERR(ans->chan);
		if(err != -EPROBE_DEFER)
			dev_err(ans->dev, "failed to attach to mailbox: %d.\n", err);
		return err;
	}
	err = devm_add_action(ans->dev, apple_m1_ans_free_channel, ans);
	if(err)
		return err;

	err = apple_iop_set_mapper_func(ans->chan, apple_iop_ans_mapper_func, ans);
	if(err)
		return err;
	err = devm_add_action(ans->dev, apple_m1_ans_clear_mapper, ans);
	if(err)
		return err;

	irq_dom = irq_domain_create_linear(fwnode, 1, &apple_m1_ans_irq_dom_ops, ans);
	if(!irq_dom) {
		dev_err(ans->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	msi_dom = pci_msi_create_irq_domain(fwnode, &apple_m1_ans_msi_dom_info, irq_dom);
	if(!msi_dom) {
		dev_err(ans->dev, "failed to create MSI domain\n");
		irq_domain_remove(irq_dom);
		return -ENOMEM;
	}

	ans->irq_dom = irq_dom;
	irq_set_chained_handler_and_data(ans->irq, apple_m1_ans_isr, ans);
	disable_irq(ans->irq);

	err = apple_m1_ans_prepare(ans);
	if(err)
		return err;

	for(i=0; i<NUM_VPCI; i++)
		for(j=0; j<CFGREG_SIZE/8; j++)
			ans->config[i][j] = apple_m1_ans_config[i][j * 2];
	/* build bridge memory base & limit */
	ans->config[0][4] |= ((bus_base >> 16) & 0xfff0) | (bus_base & 0xfff00000);
	/* build device BAR */
	ans->config[1][2] |= bus_base;

	bridge->probe_only = true;
	bridge->sysdata = ans;
	bridge->ops = &apple_m1_ans_pci_ops;

	err = pci_host_probe(bridge);
	if(err < 0)
		return err;

	return 0;
}

static int apple_m1_ans_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver apple_m1_ans_driver = {
	.driver = {
		.name = "nvme-apple-m1",
		.of_match_table = apple_m1_ans_match,
	},
	.probe = apple_m1_ans_probe,
	.remove = apple_m1_ans_remove,
};

module_platform_driver(apple_m1_ans_driver);

MODULE_DESCRIPTION("Apple M1 ANS NVMe driver");
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL");
