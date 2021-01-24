// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * PCIe host driver for Apple M1 SoC
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/msi.h>
#include <linux/pci-ecam.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <asm/io.h>

#define NUM_MSI				32
#define NUM_PORT			3
#define MSI_PER_PORT			8
#define NUM_RID2SID			16

#define CORE_RC_PHYIF_CTL		0x00024
#define   CORE_RC_PHYIF_CTL_RUN		BIT(0)
#define CORE_RC_PHYIF_STAT		0x00028
#define   CORE_RC_PHYIF_STAT_REFCLK	BIT(4)
#define CORE_RC_CTL			0x00050
#define   CORE_RC_CTL_RUN		BIT(0)
#define CORE_RC_STAT			0x00058
#define   CORE_RC_STAT_READY		BIT(0)
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
#define PORT_INTMSKSET			0x00104
#define PORT_INTMSKCLR			0x00108
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

struct pcie_apple_m1 {
	struct platform_device *pdev;
	void __iomem *base_config;
	void __iomem *base_core[2];
	void __iomem *base_port[NUM_PORT];
	struct clk *clk[3];
	struct gpio_desc *perstn[NUM_PORT];
	struct gpio_desc *clkreqn[NUM_PORT];
	struct pinctrl *pctrl;
	struct gpio_descs *devpwrio;
	struct {
		int num;
		u32 *seq;
	} devpwron[NUM_PORT];
	bool refclk_always_on[NUM_PORT];
	u32 max_speed[NUM_PORT];

	struct pci_host_bridge *bridge;
	struct pci_config_window *cfgwin;

	spinlock_t used_rid_lock;
	uint32_t rid2sid[NUM_PORT][NUM_RID2SID];
	uint32_t used_rids[NUM_PORT];

	DECLARE_BITMAP(used_msi[NUM_PORT], MSI_PER_PORT);
	u64 msi_doorbell;
	spinlock_t used_msi_lock;
	struct irq_domain *irq_dom;
	struct pcie_apple_m1_msi {
		struct pcie_apple_m1 *pcie;
		int virq;
	} msi[NUM_MSI];
	struct pcie_apple_m1_port {
		struct pcie_apple_m1 *pcie;
	} port[NUM_PORT];
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
		case 3 ... 2 + NUM_PORT:
			rmwl(mask, val, pcie->base_port[range - 3] + addr);
			break;
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

static void pcie_apple_m1_mask_irq(struct irq_data *d)
{
	struct pcie_apple_m1 *pcie = d->chip_data;
	disable_irq(pcie->msi[d->hwirq].virq);
}

static void pcie_apple_m1_unmask_irq(struct irq_data *d)
{
	struct pcie_apple_m1 *pcie = d->chip_data;
	enable_irq(pcie->msi[d->hwirq].virq);
}

static struct irq_chip pcie_apple_m1_msi_chip = {
	.name = "MSI",
	.irq_ack = pcie_apple_m1_ack_irq,
	.irq_mask = pcie_apple_m1_mask_irq,
	.irq_unmask = pcie_apple_m1_unmask_irq,
	.irq_compose_msi_msg = pcie_apple_m1_compose_msi_msg,
	.irq_set_affinity = pcie_apple_m1_set_affinity,
};

static struct msi_domain_info pcie_apple_m1_msi_dom_info = {
	.flags = MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS,
	.chip = &pcie_apple_m1_irq_chip,
};

static unsigned pcie_apple_m1_bus_to_port(struct pcie_apple_m1 *pcie, unsigned bus)
{
	unsigned port, bus0, bus1;
	u32 cfg;
	for(port=0; port<NUM_PORT; port++) {
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
	unsigned bus = busdevfn >> 19, port;

	if(bus < 1) /* no MSIs on root ports */
		return -ENOSPC;

	if(nr_irqs > 1)
		return -ENOSPC;

	port = pcie_apple_m1_bus_to_port(pcie, bus);
	if(port >= NUM_PORT)
		return -ENOSPC;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);

	pos = find_first_zero_bit(pcie->used_msi[port], MSI_PER_PORT);
	if(pos >= MSI_PER_PORT) {
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return -ENOSPC;
	}
	__set_bit(pos, pcie->used_msi[port]);
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
	irq_domain_set_info(dom, virq, pos + MSI_PER_PORT * port, &pcie_apple_m1_msi_chip, pcie, handle_edge_irq, NULL, NULL);

	return 0;
}

static void pcie_apple_m1_irq_domain_free(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs)
{
	unsigned long flags;
	struct irq_data *d = irq_domain_get_irq_data(dom, virq);
	struct pcie_apple_m1 *pcie = d->chip_data;
	unsigned i, port;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	for(i=0; i<nr_irqs; i++) {
		port = (d->hwirq + i) / MSI_PER_PORT;
		__clear_bit(d->hwirq + i - port * MSI_PER_PORT, pcie->used_msi[port]);
	}
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static const struct irq_domain_ops pcie_apple_m1_irq_dom_ops = {
	.alloc = pcie_apple_m1_irq_domain_alloc,
	.free = pcie_apple_m1_irq_domain_free,
};

static void pcie_apple_m1_setup_rid2sid(struct device *dev, unsigned int busdevfn)
{
	unsigned long flags;
	struct pcie_apple_m1 *pcie = dev_get_drvdata(dev);
	unsigned i, port, new = 0;

	port = pcie_apple_m1_bus_to_port(pcie, busdevfn >> 8);
	if(port >= NUM_PORT)
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
			pcie->rid2sid[port][i] = busdevfn | PORT_RID2SID_VALID; /* use SID 0 for now - that's what DART expects */
			writel(pcie->rid2sid[port][i], pcie->base_port[port] + PORT_RID2SID(i));
			readl(pcie->base_port[port] + PORT_RID2SID(i));
			new = 1;
		} else
			dev_err(dev, "out of DART stream entries on port %d.\n", port);
	}
	spin_unlock_irqrestore(&pcie->used_rid_lock, flags);

	if(new)
		dev_err(dev, "new rid2sid: %02x:%02x:%x @ %d\n", busdevfn >> 8, (busdevfn >> 3) & 31, busdevfn & 7, i);
}

static int pcie_apple_m1_config_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
	struct pci_config_window *cfg = bus->sysdata;
	int ret;
	if(bus->number == cfg->busr.start && PCI_SLOT(devfn) >= NUM_PORT)
		return -ENODEV;
	ret = pci_generic_config_read(bus, devfn, where, size, val);
	if(where <= 0x3C && where + size > 0x3C) /* intercept reads from IRQ line */
		*val |= 0xFFu << ((0x3C - where) << 3);
	if(where <= 0x3D && where + size > 0x3D) /* intercept reads from IRQ pin */
		*val &= ~(0xFFu << ((0x3D - where) << 3));
	return ret;
}

static int pcie_apple_m1_config_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
	struct pci_config_window *cfg = bus->sysdata;
	if(bus->number == cfg->busr.start && PCI_SLOT(devfn) >= NUM_PORT)
		return -ENODEV;
	if(where <= 0x3C && where + size > 0x3C) /* intercept writes to IRQ line */
		val |= 0xFFu << ((0x3C - where) << 3);
	if(where == 0x04 && (val & 4)) /* writes to command register with bus master enable */
		pcie_apple_m1_setup_rid2sid(cfg->parent, ((unsigned int)bus->number << 8) | devfn);
	return pci_generic_config_write(bus, devfn, where, size, val);
}

static struct pci_ecam_ops pcie_apple_m1_ecam_ops = {
	.bus_shift = 20,
	.pci_ops = {
		.map_bus = pci_ecam_map_bus,
		.read = pcie_apple_m1_config_read,
		.write = pcie_apple_m1_config_write,
	}
};

static irqreturn_t pcie_apple_m1_portirq_isr(int irq, void *dev_id)
{
	struct pcie_apple_m1_port *pcieport = dev_id;
	struct pcie_apple_m1 *pcie = pcieport->pcie;
	unsigned idx = pcieport - pcie->port;
	uint32_t mask;

	mask = readl(pcie->base_port[idx] + PORT_INTSTAT);
	writel(mask, pcie->base_port[idx] + PORT_INTSTAT);
	pr_err("pcie%d: got port irq 0x%x\n", idx, mask);

	return IRQ_HANDLED;
}

static int pcie_apple_m1_link_up(struct pcie_apple_m1 *pcie, unsigned idx)
{
	uint32_t linksts = readl(pcie->base_port[idx] + PORT_LINKSTS);
	return !!(linksts & PORT_LINKSTS_UP);
}

static int pcie_apple_m1_wait_poll(struct pcie_apple_m1 *pcie, unsigned idx, volatile void __iomem *addr, uint32_t mask, uint32_t min, uint32_t max, unsigned msec, const char *msg)
{
	uint32_t val = 0;
	while(msec --) {
		val = readl(addr);
		if((val & mask) >= min && (val & mask) <= max)
			return 0;
		usleep_range(1000, 2000);
	}
	pr_err("pcie%d: %s timed out (%08x/%08x/%08x-%08x)\n", idx, msg, val, mask, min, max);
	return -ETIMEDOUT;
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
	writel(0x110, pcie->base_port[idx] + 0x8c);
	writel(-1, pcie->base_port[idx] + PORT_INTSTAT);
	writel(-1, pcie->base_port[idx] + 0x148);
	writel(-1, pcie->base_port[idx] + PORT_LINKCMDSTS);
	writel(0, pcie->base_port[idx] + PORT_LTSSMCTL);
	writel(0, pcie->base_port[idx] + 0x84);
	writel(0xfffffff, pcie->base_port[idx] + PORT_INTMSKSET);
	writel(0, pcie->base_port[idx] + PORT_MSICFG);
	writel(0, pcie->base_port[idx] + PORT_MSIBASE);
	writel(0, pcie->base_port[idx] + PORT_MSIADDR);
	writel(0x10, pcie->base_port[idx] + 0x13c);
	writel(0x100000 | PORT_APPCLK_CGDIS, pcie->base_port[idx] + PORT_APPCLK);
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
	int res;

	res = pcie_apple_m1_wait_poll(pcie, idx, pcie->base_core[0] + CORE_RC_PHYIF_STAT, CORE_RC_PHYIF_STAT_REFCLK, CORE_RC_PHYIF_STAT_REFCLK, CORE_RC_PHYIF_STAT_REFCLK, 50, "core refclk");
	if(res < 0)
		return res;

	rmwl(0, CORE_LANE_CTL_CFGACC, pcie->base_core[0] + CORE_LANE_CTL(idx));
	rmwl(0, CORE_LANE_CFG_REFCLK0REQ, pcie->base_core[0] + CORE_LANE_CFG(idx));
	res = pcie_apple_m1_wait_poll(pcie, idx, pcie->base_core[0] + CORE_LANE_CFG(idx), CORE_LANE_CFG_REFCLK0ACK, CORE_LANE_CFG_REFCLK0ACK, CORE_LANE_CFG_REFCLK0ACK, 50, "lane refclk0");
	if(res < 0)
		return res;
	rmwl(0, CORE_LANE_CFG_REFCLK1, pcie->base_core[0] + CORE_LANE_CFG(idx));
	res = pcie_apple_m1_wait_poll(pcie, idx, pcie->base_core[0] + CORE_LANE_CFG(idx), CORE_LANE_CFG_REFCLK1, CORE_LANE_CFG_REFCLK1, CORE_LANE_CFG_REFCLK1, 50, "lane refclk0");
	if(res < 0)
		return res;
	rmwl(CORE_LANE_CTL_CFGACC, 0, pcie->base_core[0] + CORE_LANE_CTL(idx));
	udelay(1);
	rmwl(0, CORE_LANE_CFG_REFCLKEN, pcie->base_core[0] + CORE_LANE_CFG(idx));

	rmwl(0, PORT_REFCLK_EN, pcie->base_port[idx] + PORT_REFCLK);

	return 0;
}

static void pcie_apple_m1_setup_msi(struct pcie_apple_m1 *pcie, unsigned idx)
{
	unsigned nvec = roundup_pow_of_two(MSI_PER_PORT);
	unsigned bvec = MSI_PER_PORT * idx;

	writel((nvec << PORT_MSICFG_L2MSINUM_SHIFT) | PORT_MSICFG_EN, pcie->base_port[idx] + PORT_MSICFG);
	writel((bvec << PORT_MSIBASE_1_SHIFT) | bvec, pcie->base_port[idx] + PORT_MSIBASE);
	writel(pcie->msi_doorbell, pcie->base_port[idx] + PORT_MSIADDR);
}

static int pcie_apple_m1_setup_port(struct pcie_apple_m1 *pcie, unsigned idx)
{
	char tunable_name[64] = { 0 };
	unsigned ptr;
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

	res = pcie_apple_m1_wait_poll(pcie, idx, pcie->base_port[idx] + PORT_STATUS, PORT_STATUS_READY, PORT_STATUS_READY, PORT_STATUS_READY, 250, "port ready");
	if(res < 0)
		return res;

	if(!pcie->refclk_always_on[idx])
		rmwl(PORT_REFCLK_CGDIS, 0, pcie->base_port[idx] + PORT_REFCLK);
	rmwl(PORT_APPCLK_CGDIS, 0, pcie->base_port[idx] + PORT_APPCLK);

	res = pcie_apple_m1_wait_poll(pcie, idx, pcie->base_port[idx] + PORT_LINKSTS, PORT_LINKSTS_BUSY, 0, 0, 250, "port link not busy");
	if(res < 0)
		return res;

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

	writel(0xfb512fff, pcie->base_port[idx] + PORT_INTMSKSET);
	writel(0x04aed000, pcie->base_port[idx] + PORT_INTSTAT);

	pcie_apple_m1_setup_msi(pcie, idx);

	usleep_range(5000, 10000);

	rmwl(0, PORT_LTSSMCTL_START, pcie->base_port[idx] + PORT_LTSSMCTL);
	pcie_apple_m1_wait_poll(pcie, idx, pcie->base_port[idx] + PORT_LINKSTS, PORT_LINKSTS_UP, PORT_LINKSTS_UP, PORT_LINKSTS_UP, 500, "link up");

	return 0;
}

static int pcie_apple_m1_setup_phy_global(struct pcie_apple_m1 *pcie)
{
	int res;

	res = pcie_apple_m1_tunable(pcie, "tunable-phy");
	if(res < 0)
		return res;

	rmwl(0, CORE_PHY_CTL_CLK0REQ, pcie->base_core[0] + CORE_PHY_CTL);
	pcie_apple_m1_wait_poll(pcie, 0, pcie->base_core[0] + CORE_PHY_CTL, CORE_PHY_CTL_CLK0ACK, CORE_PHY_CTL_CLK0ACK, CORE_PHY_CTL_CLK0ACK, 50, "global phy clk0");
	udelay(1);
	rmwl(0, CORE_PHY_CTL_CLK1REQ, pcie->base_core[0] + CORE_PHY_CTL);
	pcie_apple_m1_wait_poll(pcie, 0, pcie->base_core[0] + CORE_PHY_CTL, CORE_PHY_CTL_CLK1ACK, CORE_PHY_CTL_CLK1ACK, CORE_PHY_CTL_CLK1ACK, 50, "global phy clk1");
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
	pcie_apple_m1_wait_poll(pcie, 0, pcie->base_core[0] + CORE_RC_STAT, CORE_RC_STAT_READY, CORE_RC_STAT_READY, CORE_RC_STAT_READY, 50, "root complex ready");

	for(port=0; port<NUM_PORT; port++) {
		if(pcie->devpwron[port].num < 0)
			continue;

		pcie_apple_m1_setup_port(pcie, port);
	}

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

static int pcie_apple_m1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct fwnode_handle *fwnode = of_node_to_fwnode(node);
	struct pci_host_bridge *bridge;
	struct resource_entry *bus;
	struct pcie_apple_m1 *pcie;
	void __iomem *mmio[3 + NUM_PORT];
	struct irq_domain *msi_dom, *irq_dom;
	int virq[NUM_MSI], pirq, ret;
	char name[64];
	unsigned i;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if(!bridge)
		return -ENOMEM;
	pcie = pci_host_bridge_priv(bridge);
	pcie->bridge = bridge;

	for(i=0; i<3+NUM_PORT; i++) {
		mmio[i] = of_iomap(node, i);
		if(!mmio[i]) {
			dev_err(dev, "failed to map MMIO range %d.\n", i);
			return -EINVAL;
		}
	}
	pcie->base_config = mmio[0];
	for(i=0; i<2; i++)
		pcie->base_core[i] = mmio[i + 1];
	for(i=0; i<NUM_PORT; i++)
		pcie->base_port[i] = mmio[i + 3];

	for(i=0; i<3; i++) {
		pcie->clk[i] = devm_clk_get(dev, pcie_apple_m1_clock_names[i]);
		if(IS_ERR(pcie->clk[i]))
			return PTR_ERR(pcie->clk[i]);
	}

	for(i=0; i<NUM_PORT; i++) {
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

	for(i=0; i<NUM_PORT; i++) {
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

	pcie->pdev = pdev;
	platform_set_drvdata(pdev, pcie);

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

	for(i=0; i<NUM_PORT; i++) {
		pirq = platform_get_irq(pdev, i);
		if(pirq < 0) {
			dev_err(dev, "failed to map port IRQ %d\n", i);
			return -EINVAL;
		}
		pcie->port[i].pcie = pcie;
		if(devm_request_irq(dev, pirq, pcie_apple_m1_portirq_isr, 0, dev_name(dev), &pcie->port[i]) < 0) {
			dev_err(dev, "failed to request port IRQ %d\n", i);
			return -EINVAL;
		}
	}

	for(i=0; i<NUM_MSI; i++) {
		virq[i] = platform_get_irq(pdev, NUM_PORT + i);
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

	pcie_apple_m1_setup_ports(pcie);

	bridge->sysdata = pcie->cfgwin;
	bridge->ops = (struct pci_ops *)&pcie_apple_m1_ecam_ops.pci_ops;

	ret = pci_host_probe(bridge);
	if(ret < 0)
		return ret;

	return 0;
}

static const struct of_device_id pcie_apple_m1_ids[] = {
	{ .compatible = "apple,pcie-m1" },
	{ },
};

static struct platform_driver pcie_apple_m1_driver = {
	.probe = pcie_apple_m1_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = pcie_apple_m1_ids,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(pcie_apple_m1_driver);
