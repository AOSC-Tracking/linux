// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * PCIe host driver for Apple M1 SoC
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_pci.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/iopoll.h>
#include <linux/workqueue.h>
#include <asm/io.h>

#include <../pci.h>

#define NUM_MSI				32
#define MAX_PORT			3
#define NUM_RID2SID			16

#define CORE_RC_PHYIF_CTL		0x00024
#define   CORE_RC_PHYIF_CTL_RUN		BIT(0)
#define CORE_RC_PHYIF_STAT		0x00028
#define   CORE_RC_PHYIF_STAT_REFCLK	BIT(4)
#define CORE_RC_CTL			0x00050
#define   CORE_RC_CTL_RUN		BIT(0)
#define CORE_RC_STAT			0x00058
#define   CORE_RC_STAT_READY		BIT(0)
#define CORE_FABRIC_STAT		0x04000
#define   CORE_FABRIC_STAT_MASK		0x001F001F
#define CORE_PHY_CTL			0x80000
#define   CORE_PHY_CTL_CLK0REQ		BIT(0)
#define   CORE_PHY_CTL_CLK1REQ		BIT(1)
#define   CORE_PHY_CTL_CLK0ACK		BIT(2)
#define   CORE_PHY_CTL_CLK1ACK		BIT(3)
#define   CORE_PHY_CTL_RESET		BIT(7)
#define CORE_LANE_CFG(port)		(0x84000 + 0x4000 * (port))
#define   CORE_LANE_CFG_REFCLK0REQ	BIT(0)
#define   CORE_LANE_CFG_REFCLK1		BIT(1)
#define   CORE_LANE_CFG_REFCLK0ACK	BIT(2)
#define   CORE_LANE_CFG_REFCLKEN	(BIT(9) | BIT(10))
#define CORE_LANE_CTL(port)		(0x84004 + 0x4000 * (port))
#define   CORE_LANE_CTL_CFGACC		BIT(15)

#define PORT_LTSSMCTL			0x00080
#define   PORT_LTSSMCTL_START		BIT(0)
#define PORT_INTSTAT			0x00100
#define   PORT_INT_TUNNEL_ERR		BIT(31)
#define   PORT_INT_CPL_TIMEOUT		BIT(23)
#define   PORT_INT_RID2SID_MAPERR	BIT(22)
#define   PORT_INT_CPL_ABORT		BIT(21)
#define   PORT_INT_MSI_BAD_DATA		BIT(19)
#define   PORT_INT_MSI_ERR		BIT(18)
#define   PORT_INT_REQADDR_GT32		BIT(17)
#define   PORT_INT_AF_TIMEOUT		BIT(15)
#define   PORT_INT_LINK_DOWN		BIT(14)
#define   PORT_INT_LINK_UP		BIT(12)
#define   PORT_INT_LINK_BWMGMT		BIT(11)
#define   PORT_INT_AER_MASK		(15 << 4)
#define   PORT_INT_PORT_ERR		BIT(4)
#define   PORT_INT_INTx(i)		BIT(i)
#define   PORT_INT_INTxALL		15
#define PORT_INTMSK			0x00104
#define PORT_INTMSKSET			0x00108
#define PORT_INTMSKCLR			0x0010c
#define PORT_MSICFG			0x00124
#define   PORT_MSICFG_EN		BIT(0)
#define   PORT_MSICFG_L2MSINUM_SHIFT	4
#define PORT_MSIBASE			0x00128
#define   PORT_MSIBASE_1_SHIFT		16
#define PORT_MSIADDR			0x00168
#define PORT_LINKSTS			0x00208
#define   PORT_LINKSTS_UP		BIT(0)
#define   PORT_LINKSTS_BUSY		BIT(2)
#define PORT_LINKCMDSTS			0x00210
#define PORT_OUTS_NPREQS		0x00284
#define   PORT_OUTS_NPREQS_REQ		BIT(24)
#define   PORT_OUTS_NPREQS_CPL		BIT(16)
#define PORT_RXWR_FIFO			0x00288
#define   PORT_RXWR_FIFO_HDR		GENMASK(15,10)
#define   PORT_RXWR_FIFO_DATA		GENMASK(9,0)
#define PORT_RXRD_FIFO			0x0028C
#define   PORT_RXRD_FIFO_REQ		GENMASK(6,0)
#define PORT_OUTS_CPLS			0x00290
#define   PORT_OUTS_CPLS_SHRD		GENMASK(14,8)
#define   PORT_OUTS_CPLS_WAIT		GENMASK(6,0)
#define PORT_APPCLK			0x00800
#define   PORT_APPCLK_EN		BIT(0)
#define   PORT_APPCLK_CGDIS		BIT(8)
#define PORT_STATUS			0x00804
#define   PORT_STATUS_READY		BIT(0)
#define PORT_REFCLK			0x00810
#define   PORT_REFCLK_EN		BIT(0)
#define   PORT_REFCLK_CGDIS		BIT(8)
#define PORT_PERST			0x00814
#define   PORT_PERST_OFF		BIT(0)
#define PORT_RID2SID(i16)		(0x00828 + 4 * (i16))
#define   PORT_RID2SID_VALID		BIT(31)
#define   PORT_RID2SID_SID_SHIFT	16
#define   PORT_RID2SID_BUS_SHIFT	8
#define   PORT_RID2SID_DEV_SHIFT	3
#define   PORT_RID2SID_FUNC_SHIFT	0
#define PORT_OUTS_PREQS_HDR		0x00980
#define   PORT_OUTS_PREQS_HDR_MASK	GENMASK(9,0)
#define PORT_OUTS_PREQS_DATA		0x00984
#define   PORT_OUTS_PREQS_DATA_MASK	GENMASK(15,0)
#define PORT_TUNCTRL			0x00988
#define   PORT_TUNCTRL_PERST_ON		BIT(0)
#define   PORT_TUNCTRL_PERST_ACK_REQ	BIT(1)
#define PORT_TUNSTAT			0x0098c
#define   PORT_TUNSTAT_PERST_ON		BIT(0)
#define   PORT_TUNSTAT_PERST_ACK_PEND	BIT(1)
#define PORT_PREFMEM_ENABLE		0x00994

#define TYPE_PCIE			0
#define TYPE_PCIEC			1

struct pcie_apple_m1 {
	struct platform_device *pdev;
	unsigned type;
	void __iomem *base_config;
	void __iomem *base_core[2];
	void __iomem *base_port[MAX_PORT];
	struct clk *clk[3];
	struct gpio_desc *perstn[MAX_PORT];
	struct gpio_desc *clkreqn[MAX_PORT];
	struct pinctrl *pctrl;
	struct gpio_descs *devpwrio;
	struct {
		int num;
		u32 *seq;
	} devpwron[MAX_PORT];
	bool refclk_always_on[MAX_PORT];
	u32 max_speed[MAX_PORT];
	unsigned num_port, msi_per_port;

	struct pci_host_bridge *bridge;
	struct pci_config_window *cfgwin;

	uint32_t ltssm_restart[MAX_PORT];

	bool hotplug_ready;
	spinlock_t hotplug_lock;
	bool port_link[MAX_PORT];
	bool last_work_link[MAX_PORT];
	struct work_struct hotplug_work;

	spinlock_t used_rid_lock;
	uint32_t rid2sid[MAX_PORT][NUM_RID2SID];
	uint32_t used_rids[MAX_PORT];

	DECLARE_BITMAP(used_msi[MAX_PORT], NUM_MSI);
	u64 msi_doorbell;
	spinlock_t used_msi_lock;
	struct irq_domain *irq_dom, *intx_dom;
	struct pcie_apple_m1_msi {
		struct pcie_apple_m1 *pcie;
		int virq;
	} msi[NUM_MSI];
	struct pcie_apple_m1_port {
		struct pcie_apple_m1 *pcie;
	} port[MAX_PORT];
};

static inline void rmwl(u32 clr, u32 set, volatile void __iomem *addr)
{
	writel((readl(addr) & ~clr) | set, addr);
}

static inline void rmww(u16 clr, u16 set, volatile void __iomem *addr)
{
	writew((readw(addr) & ~clr) | set, addr);
}

/* this code is very similar to DWC3. would be nice to factor it out and merge, maybe
   with a programmable width for each transaction? */
static int pcie_apple_m1_tunable(struct pcie_apple_m1 *pcie, const char *name)
{
	struct device_node *np = pcie->pdev->dev.of_node;
	int cnt, idx;
	u32 addr, mask, val, range;

	cnt = of_property_count_elems_of_size(np, name, sizeof(u32));
	if(cnt < 0) {
		pr_warn("%pOFn: %s: missing tunable [%s].\n", np, __func__, name);
		return 0;
	}
	if(cnt % 3) {
		pr_warn("%pOFn: %s: tunable [%s] is not made of <addr, mask, val> triplets.\n", np, __func__, name);
		return -EINVAL;
	}

	for(idx=0; idx<cnt; idx+=3) {
		if(of_property_read_u32_index(np, name, idx, &addr) < 0 ||
		   of_property_read_u32_index(np, name, idx + 1, &mask) < 0 ||
		   of_property_read_u32_index(np, name, idx + 2, &val)) {
			pr_warn("%pOFn: %s: tunable [%s] not readable.\n", np, __func__, name);
			return -EINVAL;
		}
		range = addr >> 28;
		addr &= 0x0FFFFFFF;

		switch(range) {
		case 0:
			rmwl(mask, val, pcie->base_config + addr);
			break;
		case 1 ... 2:
			rmwl(mask, val, pcie->base_core[range - 1] + addr);
			break;
		case 3 ... 2 + MAX_PORT:
			if(pcie->base_port[range - 3]) {
				rmwl(mask, val, pcie->base_port[range - 3] + addr);
				break;
			}
			/* fallthru */
		default:
			pr_warn("%pOFn: %s: tunable [%s] refers to nonexistent range %d.\n", np, __func__, name, range);
		}
	}

	return 0;
}

static struct irq_chip pcie_apple_m1_irq_chip = {
	.name = "MSI IRQ",
	.irq_ack = irq_chip_ack_parent,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
};

static void pcie_apple_m1_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_apple_m1_msi *pciemsi = irq_desc_get_handler_data(desc);
	struct pcie_apple_m1 *pcie = pciemsi->pcie;
	unsigned idx = pciemsi - pcie->msi, virq;

	chained_irq_enter(chip, desc);

	virq = irq_find_mapping(pcie->irq_dom, idx);
	generic_handle_irq(virq);

	chained_irq_exit(chip, desc);
}

static void pcie_apple_m1_compose_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct pcie_apple_m1 *pcie = d->chip_data;
	msg->address_lo = lower_32_bits(pcie->msi_doorbell);
	msg->address_hi = upper_32_bits(pcie->msi_doorbell);
	msg->data = d->hwirq;
}

static int pcie_apple_m1_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void pcie_apple_m1_ack_irq(struct irq_data *d)
{
	/* should be already acked at AIC level since this is a straight passthrough */
}

static struct irq_chip pcie_apple_m1_msi_chip = {
	.name = "MSI",
	.irq_ack = pcie_apple_m1_ack_irq,
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
	.irq_compose_msi_msg = pcie_apple_m1_compose_msi_msg,
	.irq_set_affinity = pcie_apple_m1_set_affinity,
};

static struct msi_domain_info pcie_apple_m1_msi_dom_info = {
	.flags = MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS | MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_PCI_MSIX,
	.chip = &pcie_apple_m1_irq_chip,
};

static void pcie_apple_m1_intx_mask(struct irq_data *data)
{
	struct pcie_apple_m1 *pcie = irq_data_get_irq_chip_data(data);
	int port, line;

	if(data->hwirq >= 4 * pcie->num_port)
		return;
	port = data->hwirq >> 2;
	line = data->hwirq & 3;

	writel(1 << line, pcie->base_port[port] + PORT_INTMSKSET);
}

static void pcie_apple_m1_intx_unmask(struct irq_data *data)
{
	struct pcie_apple_m1 *pcie = irq_data_get_irq_chip_data(data);
	int port, line;

	if(data->hwirq >= 4 * pcie->num_port)
		return;
	port = data->hwirq >> 2;
	line = data->hwirq & 3;

	writel(1 << line, pcie->base_port[port] + PORT_INTMSKCLR);
}

static struct irq_chip pcie_apple_m1_intx_chip = {
	.name = "INTx",
	.irq_enable = pcie_apple_m1_intx_unmask,
	.irq_disable = pcie_apple_m1_intx_mask,
	.irq_mask = pcie_apple_m1_intx_mask,
	.irq_unmask = pcie_apple_m1_intx_unmask,
};

static int pcie_apple_m1_intx_map(struct irq_domain *domain, unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &pcie_apple_m1_intx_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);
	irq_set_status_flags(irq, IRQ_LEVEL);

	return 0;
}

static const struct irq_domain_ops pcie_apple_m1_intx_dom_ops = {
	.map = pcie_apple_m1_intx_map,
	.xlate = pci_irqd_intx_xlate,
};

static unsigned pcie_apple_m1_bus_to_port(struct pcie_apple_m1 *pcie, unsigned bus)
{
	unsigned port, bus0, bus1;
	u32 cfg;
	for(port=0; port<pcie->num_port; port++) {
		cfg = readl(pcie->base_config + (port << 15) + 0x18); /* read bus number word */
		bus0 = (cfg >> 8) & 255; /* secondary bus number */
		bus1 = (cfg >> 16) & 255; /* subordinate bus number */
		if(!bus0 || !bus1 || bus0 == 0xFF || bus1 == 0xFF)
			continue;
		if(bus >= bus0 && bus <= bus1)
			break;
	}
	return port;
}

static int pcie_apple_m1_irq_domain_alloc(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs, void *args)
{
	struct pcie_apple_m1 *pcie = dom->host_data;
	unsigned long flags;
	int pos;
	u32 busdevfn = ((u32 *)args)[2];
	unsigned bus = (busdevfn >> 19) & 255, port, idx;

	if(bus < 1) /* no MSIs on root ports */
		return -ENOSPC;

	if(nr_irqs > pcie->msi_per_port)
		return -ENOSPC;

	port = pcie_apple_m1_bus_to_port(pcie, bus);
	if(port >= pcie->num_port)
		return -ENOSPC;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	pos = bitmap_find_free_region(pcie->used_msi[port], pcie->msi_per_port, order_base_2(nr_irqs));
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);

	if(pos < 0) {
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return -ENOSPC;
	}

	for(idx=0; idx<nr_irqs; idx++)
		irq_domain_set_info(dom, virq + idx, pos + idx + pcie->msi_per_port * port,
				&pcie_apple_m1_msi_chip, pcie, handle_edge_irq, NULL, NULL);

	return 0;
}

static void pcie_apple_m1_irq_domain_free(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs)
{
	unsigned long flags;
	struct irq_data *d = irq_domain_get_irq_data(dom, virq);
	struct pcie_apple_m1 *pcie = d->chip_data;
	unsigned i, port;

	nr_irqs = __roundup_pow_of_two(nr_irqs);
	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	for(i=0; i<nr_irqs; i++) {
		port = (d->hwirq + i) / pcie->msi_per_port;
		__clear_bit(d->hwirq + i - port * pcie->msi_per_port, pcie->used_msi[port]);
	}
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static const struct irq_domain_ops pcie_apple_m1_irq_dom_ops = {
	.alloc = pcie_apple_m1_irq_domain_alloc,
	.free = pcie_apple_m1_irq_domain_free,
};

static void pcie_apple_m1_setup_rid2sid(struct device *dev, unsigned int busdevfn, unsigned sid)
{
	unsigned long flags;
	struct pcie_apple_m1 *pcie = dev_get_drvdata(dev);
	unsigned i, port, new = 0;

	busdevfn &= 0xFFFF;
	port = pcie_apple_m1_bus_to_port(pcie, busdevfn >> 8);
	if(port >= pcie->num_port)
		return;

	spin_lock_irqsave(&pcie->used_rid_lock, flags);
	for(i=0; i<NUM_RID2SID; i++)
		if((pcie->used_rids[port] & (1u << i)) && (pcie->rid2sid[port][i] & 0xFFFF) == busdevfn)
			break;
	if(i >= NUM_RID2SID) {
		for(i=0; i<NUM_RID2SID; i++)
			if(!(pcie->used_rids[port] & (1u << i)))
				break;
		if(i < NUM_RID2SID) {
			pcie->used_rids[port] |= 1u << i;
			pcie->rid2sid[port][i] = busdevfn | (sid << PORT_RID2SID_SID_SHIFT) |
						 PORT_RID2SID_VALID;
			writel(pcie->rid2sid[port][i], pcie->base_port[port] + PORT_RID2SID(i));
			readl(pcie->base_port[port] + PORT_RID2SID(i));
			new = 1;
		} else
			dev_err(dev, "out of DART stream entries on port %d.\n", port);
	}
	spin_unlock_irqrestore(&pcie->used_rid_lock, flags);

	if(new)
		dev_err(dev, "new rid2sid: %02x:%02x:%x -> %x @ %d\n", busdevfn >> 8, (busdevfn >> 3) & 31, busdevfn & 7, sid, i);
}

/* called from Thunderbolt host driver to wake PCIe */
void pcie_apple_m1_start_pcic_tunnel(struct pcie_apple_m1 *pcie)
{
	u32 stat;
	int res;

	dev_info(&pcie->pdev->dev, "start PCI tunnel.\n");

	writel(PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR | PORT_INT_LINK_UP, pcie->base_port[0] + PORT_INTMSKSET);

	res = pcie_apple_m1_tunable(pcie, "tunable-port0-config");
	if(res < 0)
		return;

	rmwl(0, PORT_APPCLK_EN, pcie->base_port[0] + PORT_APPCLK);
	readl(pcie->base_port[0] + PORT_APPCLK);
	udelay(10);

	rmwl(0, PORT_PERST_OFF, pcie->base_port[0] + PORT_PERST);
	udelay(10);

	res = pcie_apple_m1_tunable(pcie, "tunable-rc");
	if(res < 0)
		return;

	rmwl(PORT_TUNCTRL_PERST_ON, 0, pcie->base_port[0] + PORT_TUNCTRL);
	readl(pcie->base_port[0] + PORT_TUNCTRL);
	res = readl_poll_timeout(pcie->base_port[0] + PORT_TUNSTAT, stat, !(stat & PORT_TUNSTAT_PERST_ON), 100, 250000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: tunnel PERST clear timed out.\n", __func__);
		return;
	}

	writel(PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR | PORT_INT_LINK_UP, pcie->base_port[0] + PORT_INTSTAT);

	writel(PORT_LTSSMCTL_START, pcie->base_port[0] + PORT_LTSSMCTL);
	pcie->ltssm_restart[0] = 3;

	writel(PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR | PORT_INT_LINK_UP, pcie->base_port[0] + PORT_INTMSKCLR);
}
EXPORT_SYMBOL_GPL(pcie_apple_m1_start_pcic_tunnel);

static const struct {
	u32 offs, bits;
	const char *desc;
} pcie_apple_m1_status_bits[] = {
	{ PORT_OUTS_NPREQS,	PORT_OUTS_NPREQS_REQ,		"OUTS_NPREQS.REQ" },
	{ PORT_OUTS_NPREQS,	PORT_OUTS_NPREQS_CPL,		"OUTS_NPREQS.CPL" },
	{ PORT_OUTS_PREQS_HDR,	PORT_OUTS_PREQS_HDR_MASK,	"OUTS_PREQS_HDR"  },
	{ PORT_OUTS_PREQS_DATA,	PORT_OUTS_PREQS_DATA_MASK,	"OUTS_PREQS_DATA" },
	{ PORT_RXWR_FIFO,	PORT_RXWR_FIFO_DATA,		"RXWR_FIFO.DATA"  },
	{ PORT_RXWR_FIFO,	PORT_RXWR_FIFO_HDR,		"RXWR_FIFO.HDR"   },
	{ PORT_RXRD_FIFO,	PORT_RXRD_FIFO_REQ,		"RXRD_FIFO.REQ"   },
	{ PORT_OUTS_CPLS,	PORT_OUTS_CPLS_SHRD,		"OUTS_CPLS.SHRD"  },
};

static void pcie_apple_m1_stop_pcic_tunnel(struct pcie_apple_m1 *pcie)
{
	u32 stat;
	int res, i;

	rmwl(PORT_LTSSMCTL_START, 0, pcie->base_port[0] + PORT_LTSSMCTL);

	for(i=0; i<sizeof(pcie_apple_m1_status_bits)/sizeof(pcie_apple_m1_status_bits[0]); i++) {
		res = readl_poll_timeout(pcie->base_port[0] + pcie_apple_m1_status_bits[i].offs,
			stat, !(stat & pcie_apple_m1_status_bits[i].bits), 100, 50000);
		if(res < 0)
			dev_err(&pcie->pdev->dev, "%s: wait for %s timed out.\n", __func__,
				pcie_apple_m1_status_bits[i].desc);
	}

	rmwl(0, PORT_TUNCTRL_PERST_ON, pcie->base_port[0] + PORT_TUNCTRL);
	readl(pcie->base_port[0] + PORT_TUNCTRL);
	readl_poll_timeout(pcie->base_port[0] + PORT_TUNSTAT, stat, (stat & PORT_TUNSTAT_PERST_ON), 100, 50000);

	rmwl(0, PORT_TUNCTRL_PERST_ACK_REQ, pcie->base_port[0] + PORT_TUNCTRL);
	readl(pcie->base_port[0] + PORT_TUNCTRL);
	res = readl_poll_timeout(pcie->base_port[0] + PORT_TUNSTAT, stat, !(stat & PORT_TUNSTAT_PERST_ACK_PEND), 100, 250000);
	if(res < 0)
		dev_err(&pcie->pdev->dev, "%s: tunnel PERST ack set timed out.\n", __func__);
	rmwl(PORT_TUNCTRL_PERST_ACK_REQ, 0, pcie->base_port[0] + PORT_TUNCTRL);
	readl(pcie->base_port[0] + PORT_TUNCTRL);

	writel(PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR, pcie->base_port[0] + PORT_INTSTAT);
}

static int pcie_apple_m1_config_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct pcie_apple_m1 *pcie = dev_get_drvdata(cfg->parent);
	int ret;

	if(bus->number == cfg->busr.start && PCI_SLOT(devfn) >= pcie->num_port)
		return -ENODEV;
	ret = pci_generic_config_read(bus, devfn, where, size, val);

	if(!pcie->intx_dom) {
		if(where <= 0x3C && where + size > 0x3C) /* intercept reads from IRQ line */
			*val |= 0xFFu << ((0x3C - where) << 3);
		if(where <= 0x3D && where + size > 0x3D) /* intercept reads from IRQ pin */
			*val &= ~(0xFFu << ((0x3D - where) << 3));
	}

	return ret;
}

static int pcie_apple_m1_config_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct pcie_apple_m1 *pcie = dev_get_drvdata(cfg->parent);
	int ret;

	if(bus->number == cfg->busr.start && PCI_SLOT(devfn) >= pcie->num_port)
		return -ENODEV;
	if(!pcie->intx_dom && where <= 0x3C && where + size > 0x3C) /* intercept writes to IRQ line */
		val |= 0xFFu << ((0x3C - where) << 3);
	if(where == 0x04 && (val & 4)) /* writes to command register with bus master enable */
		pcie_apple_m1_setup_rid2sid(cfg->parent, ((unsigned int)bus->number << 8) | devfn, bus->number);

	ret = pci_generic_config_write(bus, devfn, where, size, val);

	if(bus->number == cfg->busr.start && where >= 0x24 && where <= 0x2f && pcie->type == TYPE_PCIEC)
		writel(1, pcie->base_port[0] + PORT_PREFMEM_ENABLE);

	return ret;
}

static struct pci_ecam_ops pcie_apple_m1_ecam_ops = {
	.bus_shift = 20,
	.pci_ops = {
		.map_bus = pci_ecam_map_bus,
		.read = pcie_apple_m1_config_read,
		.write = pcie_apple_m1_config_write,
	}
};

static void pcie_apple_m1_portirq_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_apple_m1_port *pcieport = irq_desc_get_handler_data(desc);
	struct pcie_apple_m1 *pcie = pcieport->pcie;
	unsigned idx = pcieport - pcie->port, line, virq;
	uint32_t mask;

	chained_irq_enter(chip, desc);

	mask = readl(pcie->base_port[idx] + PORT_INTSTAT);
	writel(mask, pcie->base_port[idx] + PORT_INTSTAT);

	if(pcie->intx_dom)
		while(mask & PORT_INT_INTxALL) {
			line = ffs(mask & PORT_INT_INTxALL) - 1;
			mask &= ~PORT_INT_INTx(line);

			virq = irq_find_mapping(pcie->intx_dom, line + 4 * idx);
			if(virq)
				generic_handle_irq(virq);
		}

	if(!mask)
		goto done;

	dev_info(&pcie->pdev->dev, "got port %d irq 0x%x\n", idx, mask);

	if(mask & (PORT_INT_LINK_UP | PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR)) {
		spin_lock(&pcie->hotplug_lock);
		pcie->port_link[idx] = !(mask & (PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR));
		spin_unlock(&pcie->hotplug_lock);

		if(pcie->type == TYPE_PCIEC) {
			if(mask & PORT_INT_LINK_UP)
				pcie->ltssm_restart[idx] = 0;

			if(pcie->ltssm_restart[idx] > 0) {
				pcie->ltssm_restart[idx] --;
				writel(PORT_LTSSMCTL_START, pcie->base_port[idx] + PORT_LTSSMCTL);
			} else {
				if(mask & (PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR))
					writel(PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR, pcie->base_port[idx] + PORT_INTMSKSET);
				schedule_work(&pcie->hotplug_work);
			}
		}
	}

done:
	chained_irq_exit(chip, desc);
}

static void pcie_apple_m1_plug_port(struct pcie_apple_m1 *pcie, unsigned idx)
{
	struct pci_dev *bridge, *dev;
	struct pci_bus *bus;
	int num;

	dev_info(&pcie->pdev->dev, "plugging port %d...\n", idx);

	msleep(1000);

	pci_lock_rescan_remove();

	bridge = pci_get_slot(pcie->bridge->bus, PCI_DEVFN(idx, 0));
	if(!bridge)
		goto fail;
	bus = bridge->subordinate;
	if(!bus)
		return;

	num = pci_scan_slot(bus, PCI_DEVFN(0, 0));
	if(num == 0)
		goto fail;

	for_each_pci_bridge(dev, bus)
		pci_hp_add_bridge(dev);

	pci_assign_unassigned_bridge_resources(bridge);
	pcie_bus_configure_settings(bus);
	pci_bus_add_devices(bus);

fail:
	pci_unlock_rescan_remove();
}

static void pcie_apple_m1_unplug_port(struct pcie_apple_m1 *pcie, unsigned idx)
{
	struct pci_dev *bridge, *dev, *temp;
	struct pci_bus *bus;
	unsigned long flags;

	dev_info(&pcie->pdev->dev, "unplugging port %d...\n", idx);

	bridge = pci_get_slot(pcie->bridge->bus, PCI_DEVFN(idx, 0));
	if(!bridge)
		return;
	bus = bridge->subordinate;
	if(!bus)
		return;

	pci_walk_bus(bus, pci_dev_set_disconnected, NULL);

	pci_lock_rescan_remove();

	list_for_each_entry_safe_reverse(dev, temp, &bus->devices, bus_list) {
		pci_dev_get(dev);
		pci_stop_and_remove_bus_device(dev);
		pci_dev_put(dev);
	}

	pci_unlock_rescan_remove();

	pcie_apple_m1_stop_pcic_tunnel(pcie);

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	bitmap_zero(pcie->used_msi[idx], pcie->msi_per_port);
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static void pcie_apple_m1_hotplug_work(struct work_struct *work)
{
	struct pcie_apple_m1 *pcie = container_of(work, struct pcie_apple_m1, hotplug_work);
	unsigned long flags;
	unsigned idx;
	bool link;

	if(!pcie->hotplug_ready)
		return;

	for(idx=0; idx<pcie->num_port; idx++) {
		spin_lock_irqsave(&pcie->hotplug_lock, flags);
		link = pcie->port_link[idx];
		spin_unlock_irqrestore(&pcie->hotplug_lock, flags);

		if(link == pcie->last_work_link[idx])
			goto skip;
		pcie->last_work_link[idx] = link;

		if(link)
			pcie_apple_m1_plug_port(pcie, idx);
		else
			pcie_apple_m1_unplug_port(pcie, idx);

	skip:
		writel(PORT_INT_LINK_DOWN | PORT_INT_TUNNEL_ERR | PORT_INT_LINK_UP,
			pcie->base_port[0] + PORT_INTMSKCLR);
	}
}

static int pcie_apple_m1_link_up(struct pcie_apple_m1 *pcie, unsigned idx)
{
	uint32_t linksts = readl(pcie->base_port[idx] + PORT_LINKSTS);
	return !!(linksts & PORT_LINKSTS_UP);
}

static unsigned pcie_apple_m1_find_pcicap(struct pcie_apple_m1 *pcie, unsigned busdevfn, unsigned type)
{
	unsigned ptr, next;

	ptr = readl(pcie->base_config + (busdevfn << 12) + 0x034) & 0xFF;
	while(ptr) {
		next = readl(pcie->base_config + (busdevfn << 12) + ptr);
		if((next & 0xFF) == type)
			return ptr;
		ptr = (next >> 8) & 0xFF;
	}
	return 0;
}

static void pcie_apple_m1_port_pwron(struct pcie_apple_m1 *pcie, unsigned idx)
{
	int i;
	u32 *seq = pcie->devpwron[idx].seq;

	for(i=0; i<pcie->devpwron[idx].num; i++) {
		usleep_range(2500, 5000);
		if(seq[1] < 2)
			gpiod_direction_output(pcie->devpwrio->desc[seq[0]], seq[1]);
		else if(seq[1] == 2)
			gpiod_direction_input(pcie->devpwrio->desc[seq[0]]);
		usleep_range(2500, 5000);
		usleep_range(250000, 500000);
		seq += 2;
	}
}

static void pcie_apple_m1_init_port(struct pcie_apple_m1 *pcie, unsigned idx)
{
	unsigned r2s;

	writel(0x110, pcie->base_port[idx] + 0x8c);
	writel(-1, pcie->base_port[idx] + PORT_INTSTAT);
	writel(-1, pcie->base_port[idx] + 0x148);
	writel(-1, pcie->base_port[idx] + PORT_LINKCMDSTS);
	writel(0, pcie->base_port[idx] + PORT_LTSSMCTL);
	writel(0, pcie->base_port[idx] + 0x84);
	writel(-1, pcie->base_port[idx] + PORT_INTMSK);
	writel(0, pcie->base_port[idx] + PORT_MSICFG);
	writel(0, pcie->base_port[idx] + PORT_MSIBASE);
	writel(0, pcie->base_port[idx] + PORT_MSIADDR);
	writel(0x10, pcie->base_port[idx] + 0x13c);
	writel(0x100000 | PORT_APPCLK_CGDIS, pcie->base_port[idx] + PORT_APPCLK);
	if(pcie->type == TYPE_PCIEC) {
		writel(0x100045, pcie->base_port[idx] + 0x808);
		for(r2s=0; r2s<64; r2s++)
			writel(0, pcie->base_port[idx] + PORT_RID2SID(r2s));
	} else
		writel(0x100010, pcie->base_port[idx] + 0x808);
	writel(PORT_REFCLK_CGDIS, pcie->base_port[idx] + PORT_REFCLK);
	writel(0, pcie->base_port[idx] + PORT_PERST);
	writel(0, pcie->base_port[idx] + 0x130);
	writel(0x10, pcie->base_port[idx] + 0x140);
	writel(0x00253770, pcie->base_port[idx] + 0x144);
	writel(0, pcie->base_port[idx] + 0x21c);
	writel(0, pcie->base_port[idx] + 0x81c);
	writel(0, pcie->base_port[idx] + 0x824);
}

static int pcie_apple_m1_setup_refclk(struct pcie_apple_m1 *pcie, unsigned idx)
{
	u32 stat;
	int res;

	res = readl_poll_timeout(pcie->base_core[0] + CORE_RC_PHYIF_STAT, stat, (stat & CORE_RC_PHYIF_STAT_REFCLK), 100, 50000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: core refclk wait timed out.\n", __func__);
		return res;
	}

	rmwl(0, CORE_LANE_CTL_CFGACC, pcie->base_core[0] + CORE_LANE_CTL(idx));
	rmwl(0, CORE_LANE_CFG_REFCLK0REQ, pcie->base_core[0] + CORE_LANE_CFG(idx));
	res = readl_poll_timeout(pcie->base_core[0] + CORE_LANE_CFG(idx), stat, (stat & CORE_LANE_CFG_REFCLK0ACK), 100, 50000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: lane refclk%d wait timed out (port %d).\n", __func__, 0, idx);
		return res;
	}
	rmwl(0, CORE_LANE_CFG_REFCLK1, pcie->base_core[0] + CORE_LANE_CFG(idx));
	res = readl_poll_timeout(pcie->base_core[0] + CORE_LANE_CFG(idx), stat, (stat & CORE_LANE_CFG_REFCLK1), 100, 50000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: lane refclk%d wait timed out (port %d).\n", __func__, 1, idx);
		return res;
	}
	rmwl(CORE_LANE_CTL_CFGACC, 0, pcie->base_core[0] + CORE_LANE_CTL(idx));
	udelay(1);
	rmwl(0, CORE_LANE_CFG_REFCLKEN, pcie->base_core[0] + CORE_LANE_CFG(idx));

	rmwl(0, PORT_REFCLK_EN, pcie->base_port[idx] + PORT_REFCLK);

	return 0;
}

static void pcie_apple_m1_setup_msi(struct pcie_apple_m1 *pcie, unsigned idx)
{
	unsigned nvec = order_base_2(pcie->msi_per_port);
	unsigned bvec = pcie->msi_per_port * idx;

	writel((nvec << PORT_MSICFG_L2MSINUM_SHIFT) | PORT_MSICFG_EN, pcie->base_port[idx] + PORT_MSICFG);
	writel((bvec << PORT_MSIBASE_1_SHIFT) | bvec, pcie->base_port[idx] + PORT_MSIBASE);
	writel(pcie->msi_doorbell, pcie->base_port[idx] + PORT_MSIADDR);
}

static int pcie_apple_m1_setup_port(struct pcie_apple_m1 *pcie, unsigned idx)
{
	char tunable_name[64] = { 0 };
	unsigned ptr;
	u32 stat;
	int res;

	if(pcie_apple_m1_link_up(pcie, idx))
		return 0;

	gpiod_direction_output(pcie->perstn[idx], 0);

	pcie_apple_m1_init_port(pcie, idx);

	snprintf(tunable_name, sizeof(tunable_name) - 1, "tunable-port%d-config", idx);
	res = pcie_apple_m1_tunable(pcie, tunable_name);
	if(res < 0)
		return res;

	rmwl(0, PORT_APPCLK_EN, pcie->base_port[idx] + PORT_APPCLK);

	res = pcie_apple_m1_setup_refclk(pcie, idx);
	if(res < 0)
		return res;

	pcie_apple_m1_port_pwron(pcie, idx);

	rmwl(0, PORT_PERST_OFF, pcie->base_port[idx] + PORT_PERST);
	gpiod_direction_output(pcie->perstn[idx], 1);

	res = readl_poll_timeout(pcie->base_port[idx] + PORT_STATUS, stat, (stat & PORT_STATUS_READY), 100, 250000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: status ready wait timed out (port %d).\n", __func__, idx);
		return res;
	}

	if(!pcie->refclk_always_on[idx])
		rmwl(PORT_REFCLK_CGDIS, 0, pcie->base_port[idx] + PORT_REFCLK);
	rmwl(PORT_APPCLK_CGDIS, 0, pcie->base_port[idx] + PORT_APPCLK);

	res = readl_poll_timeout(pcie->base_port[idx] + PORT_LINKSTS, stat, !(stat & PORT_LINKSTS_BUSY), 100, 250000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: link not busy wait timed out (port %d).\n", __func__, idx);
		return res;
	}

	ptr = pcie_apple_m1_find_pcicap(pcie, idx << 3, 0x10);

	rmwl(0, 1, pcie->base_config + (idx << 15) + 0x8bc);
	snprintf(tunable_name, sizeof(tunable_name) - 1, "tunable-port%d", idx);
	res = pcie_apple_m1_tunable(pcie, tunable_name);
	if(res < 0)
		return res;
	rmwl(0x3000000, 0, pcie->base_config + (idx << 15) + 0x890);
	snprintf(tunable_name, sizeof(tunable_name) - 1, "tunable-port%d-gen3-shadow", idx);
	res = pcie_apple_m1_tunable(pcie, tunable_name);
	if(res < 0)
		return res;
	rmwl(0x3000000, 0x1000000, pcie->base_config + (idx << 15) + 0x890);
	snprintf(tunable_name, sizeof(tunable_name) - 1, "tunable-port%d-gen4-shadow", idx);
	res = pcie_apple_m1_tunable(pcie, tunable_name);
	if(res < 0)
		return res;
	rmwl(1, 0, pcie->base_config + (idx << 15) + 0x8bc);

	if(ptr)
		rmww(15, pcie->max_speed[idx], pcie->base_config + (idx << 15) + (ptr + 0x30)); /* maximum speed: 2.5, 5.0, 8.0 GT/s */
	rmwl(0, 0x400, pcie->base_config + (idx << 15) + (ptr + 0x10)); /* link control: reserved field  */
	if(pcie->max_speed[idx] > 2)
		rmwl(0, 0x20000, pcie->base_config + (idx << 15) + 0x80c);

	writel(0xfb512fff, pcie->base_port[idx] + PORT_INTMSK);
	writel(0x04aed000, pcie->base_port[idx] + PORT_INTSTAT);

	pcie_apple_m1_setup_msi(pcie, idx);

	usleep_range(5000, 10000);

	rmwl(0, PORT_LTSSMCTL_START, pcie->base_port[idx] + PORT_LTSSMCTL);
	res = readl_poll_timeout(pcie->base_port[idx] + PORT_LINKSTS, stat, (stat & PORT_LINKSTS_UP), 100, 500000);
	if(res < 0)
		dev_warn(&pcie->pdev->dev, "%s: link up wait timed out (port %d).\n", __func__, idx);

	return 0;
}

static int pcie_apple_m1_setup_phy_global(struct pcie_apple_m1 *pcie)
{
	int res;
	u32 stat;

	res = pcie_apple_m1_tunable(pcie, "tunable-phy");
	if(res < 0)
		return res;

	rmwl(0, CORE_PHY_CTL_CLK0REQ, pcie->base_core[0] + CORE_PHY_CTL);
	res = readl_poll_timeout(pcie->base_core[0] + CORE_PHY_CTL, stat, (stat & CORE_PHY_CTL_CLK0ACK), 100, 50000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: global phy clk%d wait timed out.\n", __func__, 0);
		return res;
	}
	udelay(1);
	rmwl(0, CORE_PHY_CTL_CLK1REQ, pcie->base_core[0] + CORE_PHY_CTL);
	res = readl_poll_timeout(pcie->base_core[0] + CORE_PHY_CTL, stat, (stat & CORE_PHY_CTL_CLK1ACK), 100, 50000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: global phy clk%d wait timed out.\n", __func__, 1);
		return res;
	}
	rmwl(CORE_PHY_CTL_RESET, 0, pcie->base_core[0] + CORE_PHY_CTL);
	udelay(1);
	rmwl(0, CORE_RC_PHYIF_CTL_RUN, pcie->base_core[0] + CORE_RC_PHYIF_CTL);
	udelay(1);

	res = pcie_apple_m1_tunable(pcie, "tunable-fuse");
	if(res < 0)
		return res;

	res = pcie_apple_m1_tunable(pcie, "tunable-phy-ip-pll");
	if(res < 0)
		return res;

	res = pcie_apple_m1_tunable(pcie, "tunable-phy-ip-auspma");
	if(res < 0)
		return res;

	return 0;
}

static int pcie_apple_m1_setup_ports(struct pcie_apple_m1 *pcie)
{
	int res;
	unsigned port;
	u32 stat;

	res = pcie_apple_m1_tunable(pcie, "tunable-axi2af");
	if(res < 0)
		return res;

	res = pcie_apple_m1_tunable(pcie, "tunable-common");
	if(res < 0)
		return res;

	res = pcie_apple_m1_setup_phy_global(pcie);
	if(res < 0)
		return res;

	rmwl(1, 0, pcie->base_core[0] + 0x34);
	writel(0x140, pcie->base_core[0] + 0x54);
	writel(CORE_RC_CTL_RUN, pcie->base_core[0] + CORE_RC_CTL);
	res = readl_poll_timeout(pcie->base_core[0] + CORE_RC_CTL, stat, (stat & CORE_RC_STAT_READY), 100, 50000);
	if(res < 0) {
		dev_err(&pcie->pdev->dev, "%s: root complex ready wait timed out.\n", __func__);
		return res;
	}

	for(port=0; port<pcie->num_port; port++) {
		if(pcie->devpwron[port].num < 0 &&
		   pcie->type != TYPE_PCIEC)
			continue;

		pcie_apple_m1_setup_port(pcie, port);
	}

	return 0;
}

static int pcie_apple_m1_setup_pciec(struct pcie_apple_m1 *pcie)
{
	int res;

	res = pcie_apple_m1_tunable(pcie, "tunable-debug");
	if(res < 0)
		return res;

	res = pcie_apple_m1_tunable(pcie, "tunable-fabric");
	if(res < 0)
		return res;

	pcie_apple_m1_init_port(pcie, 0);

	res = pcie_apple_m1_tunable(pcie, "tunable-port0-config");
	if(res < 0)
		return res;

	rmwl(0, PORT_PERST_OFF, pcie->base_port[0] + PORT_PERST);
	rmwl(0, PORT_APPCLK_EN, pcie->base_port[0] + PORT_APPCLK);
	rmwl(PORT_APPCLK_CGDIS, 0, pcie->base_port[0] + PORT_APPCLK);

	res = pcie_apple_m1_tunable(pcie, "tunable-oe-fabric");
	if(res < 0)
		return res;

	res = pcie_apple_m1_tunable(pcie, "tunable-rc");
	if(res < 0)
		return res;

	writel(0x84aed00f, pcie->base_port[0] + PORT_INTSTAT);
	writel(0x84aed00f, pcie->base_port[0] + PORT_INTMSKCLR);

	/* allocate 15 subordinate buses */
	writel(0x200f0100, pcie->base_config + 0x18);

	pcie_apple_m1_setup_msi(pcie, 0);

	return 0;
}

static const char *const pcie_apple_m1_clock_names[] = { "core", "aux", "ref" };

static int pcie_apple_m1_pci_init(struct pcie_apple_m1 *pcie, struct resource *bus_range)
{
	struct platform_device *pdev = pcie->pdev;
	struct device *dev = &pdev->dev;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res)
		return -ENXIO;

	pcie->cfgwin = pci_ecam_create(dev, res, bus_range, &pcie_apple_m1_ecam_ops);
	if(IS_ERR(pcie->cfgwin))
		return PTR_ERR(pcie->cfgwin);

	return 0;
}

static const struct of_device_id pcie_apple_m1_ids[] = {
	{ .compatible = "apple,pcie-m1",  (void *)TYPE_PCIE },
	{ .compatible = "apple,pciec-m1", (void *)TYPE_PCIEC },
	{ },
};

static int pcie_apple_m1_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *intx_node;
	struct fwnode_handle *fwnode = of_node_to_fwnode(node);
	struct pci_host_bridge *bridge;
	struct resource_entry *bus;
	struct pcie_apple_m1 *pcie;
	void __iomem *mmio[3 + MAX_PORT];
	struct irq_domain *msi_dom, *irq_dom;
	int virq[NUM_MSI], pirq, ret;
	char name[64];
	unsigned i;

	match = of_match_device(pcie_apple_m1_ids, dev);
	if(!match)
		return -EINVAL;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if(!bridge)
		return -ENOMEM;
	pcie = pci_host_bridge_priv(bridge);
	pcie->bridge = bridge;

	INIT_WORK(&pcie->hotplug_work, pcie_apple_m1_hotplug_work);
	spin_lock_init(&pcie->hotplug_lock);

	pcie->type = (uintptr_t)match->data;
	switch(pcie->type) {
	case TYPE_PCIE:
		pcie->num_port = 3;
		pcie->msi_per_port = 8;
		break;
	case TYPE_PCIEC:
		pcie->num_port = 1;
		pcie->msi_per_port = 32;
		break;
	}

	for(i=0; i<3+pcie->num_port; i++) {
		mmio[i] = of_iomap(node, i);
		if(!mmio[i]) {
			dev_err(dev, "failed to map MMIO range %d.\n", i);
			return -EINVAL;
		}
	}
	pcie->base_config = mmio[0];
	for(i=0; i<2; i++)
		pcie->base_core[i] = mmio[i + 1];
	for(i=0; i<pcie->num_port; i++)
		pcie->base_port[i] = mmio[i + 3];

	for(i=0; i<3; i++) {
		pcie->clk[i] = devm_clk_get(dev, pcie_apple_m1_clock_names[i]);
		if(IS_ERR(pcie->clk[i]))
			return PTR_ERR(pcie->clk[i]);
	}

	if(pcie->type != TYPE_PCIEC) {
		for(i=0; i<pcie->num_port; i++) {
			pcie->perstn[i] = devm_gpiod_get_index(dev, "perst", i, 0);
			if(IS_ERR(pcie->perstn[i])) {
				if(PTR_ERR(pcie->perstn[i]) != -EPROBE_DEFER)
					dev_err(dev, "failed to get PERST#%d gpio: %ld\n", i, PTR_ERR(pcie->perstn[i]));
				return PTR_ERR(pcie->perstn[i]);
			}

			pcie->clkreqn[i] = devm_gpiod_get_index(dev, "clkreq", i, 0);
			if(IS_ERR(pcie->clkreqn[i])) {
				if(PTR_ERR(pcie->clkreqn[i]) != -EPROBE_DEFER)
				dev_err(dev, "failed to get PERST#%d gpio: %ld\n", i, PTR_ERR(pcie->clkreqn[i]));
				return PTR_ERR(pcie->clkreqn[i]);
			}
		}

		pcie->devpwrio = gpiod_get_array_optional(dev, "devpwr", 0);
		if(pcie->devpwrio && IS_ERR(pcie->devpwrio)) {
			if(PTR_ERR(pcie->devpwrio) != -EPROBE_DEFER)
				dev_err(dev, "failed to get device power gpio: %ld\n", PTR_ERR(pcie->devpwrio));
			return PTR_ERR(pcie->devpwrio);
		}

		for(i=0; i<pcie->num_port; i++) {
			sprintf(name, "devpwr-on-%d", i);
			ret = of_property_count_elems_of_size(node, name, 8);
			pcie->devpwron[i].num = ret;
			if(ret <= 0)
				continue;
			pcie->devpwron[i].seq = devm_kzalloc(dev, ret * 8, GFP_KERNEL);
			if(!pcie->devpwron[i].seq)
				return -ENOMEM;
			ret = of_property_read_variable_u32_array(node, name, pcie->devpwron[i].seq, ret * 2, ret * 2);
			if(ret < 0)
				return ret;

			sprintf(name, "refclk-always-on-%d", i);
			pcie->refclk_always_on[i] = of_property_read_bool(node, name);

			sprintf(name, "max-speed-%d", i);
			if(of_property_read_u32(node, name, &pcie->max_speed[i]) < 0)
				pcie->max_speed[i] = 2;
		}

		pcie->pctrl = devm_pinctrl_get_select_default(dev);
	}

	pcie->pdev = pdev;
	platform_set_drvdata(pdev, pcie);

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

	for(i=0; i<pcie->num_port; i++) {
		pirq = platform_get_irq(pdev, i);
		if(pirq < 0) {
			dev_err(dev, "failed to map port IRQ %d\n", i);
			return -EINVAL;
		}
		pcie->port[i].pcie = pcie;
	}

	for(i=0; i<NUM_MSI; i++) {
		virq[i] = platform_get_irq(pdev, pcie->num_port + i);
		if(virq[i] < 0) {
			dev_err(dev, "failed to map MSI IRQ %d\n", i);
			return -EINVAL;
		}
	}

	irq_dom = irq_domain_create_linear(fwnode, NUM_MSI, &pcie_apple_m1_irq_dom_ops, pcie);
	if(!irq_dom) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	msi_dom = pci_msi_create_irq_domain(fwnode, &pcie_apple_m1_msi_dom_info, irq_dom);
	if(!msi_dom) {
		dev_err(dev, "failed to create MSI domain\n");
		irq_domain_remove(irq_dom);
		return -ENOMEM;
	}

	pcie->irq_dom = irq_dom;
	spin_lock_init(&pcie->used_msi_lock);
	spin_lock_init(&pcie->used_rid_lock);

	for(i=0; i<NUM_MSI; i++) {
		pcie->msi[i].pcie = pcie;
		pcie->msi[i].virq = virq[i];
		irq_set_chained_handler_and_data(virq[i], pcie_apple_m1_msi_isr, &pcie->msi[i]);
		disable_irq(virq[i]);
	}

	intx_node = of_get_next_child(node, NULL);
	if(intx_node) {
		pcie->intx_dom = irq_domain_add_linear(intx_node, PCI_NUM_INTX * pcie->num_port, &pcie_apple_m1_intx_dom_ops, pcie);
		of_node_put(intx_node);
		if(!pcie->intx_dom) {
			dev_err(dev, "failed to create INTx domain\n");
			return -ENOMEM;
		}
	}

	for(i=0; i<pcie->num_port; i++)
		irq_set_chained_handler_and_data(pirq, pcie_apple_m1_portirq_isr, &pcie->port[i]);

	for(i=0; i<3; i++) {
		ret = clk_prepare_enable(pcie->clk[i]);
		if(ret < 0)
			return ret;
	}

	if(of_property_read_u64(node, "msi-doorbell", &pcie->msi_doorbell))
		pcie->msi_doorbell = 0xFFFFF000ul;

	bus = resource_list_first_type(&bridge->windows, IORESOURCE_BUS);
	if(!bus)
		return -ENODEV;

	ret = pcie_apple_m1_pci_init(pcie, bus->res);
	if(ret < 0)
		return ret;

	switch(pcie->type) {
	case TYPE_PCIE:
		pcie_apple_m1_setup_ports(pcie);
		break;
	case TYPE_PCIEC:
		pcie_apple_m1_setup_pciec(pcie);
		break;
	}

	bridge->sysdata = pcie->cfgwin;
	bridge->ops = (struct pci_ops *)&pcie_apple_m1_ecam_ops.pci_ops;

	ret = pci_host_probe(bridge);
	if(ret < 0)
		return ret;

	pcie->hotplug_ready = true;

	return 0;
}

static struct platform_driver pcie_apple_m1_driver = {
	.probe = pcie_apple_m1_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = pcie_apple_m1_ids,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(pcie_apple_m1_driver);
