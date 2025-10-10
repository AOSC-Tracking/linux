// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technologies, Inc.
 */

#include <linux/cpuhotplug.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/irq-msi-lib.h>
#include <linux/irqdomain.h>
#include <linux/spinlock.h>
#include <linux/msi.h>

#include <asm/irq.h>
#include <asm/loongarch.h>
#include <asm/loongson.h>
#include <asm/numa.h>
#include <asm/setup.h>
#include <larchintrin.h>

#include "irq-loongson.h"

#define IRD_ENTRY_SIZE			16
#define IRD_ENTRIES			65536
#define IRD_TABLE_PAGE_ORDER		get_order(IRD_ENTRIES * IRD_ENTRY_SIZE)

#define INVALID_QUEUE_SIZE		4096
#define INV_QUEUE_PAGE_ORDER		get_order(INVALID_QUEUE_SIZE * IRD_ENTRY_SIZE)

#define GPID_ADDR_MASK			GENMASK(47, 6)
#define GPID_ADDR_SHIFT 		6

#define CQB_SIZE_MASK			0xf
#define CQB_ADDR_MASK			GENMASK(47, 12)

#define CFG_DISABLE_IDLE		2
#define INVALID_INDEX			0

#define MAX_IR_ENGINES			16

struct redirect_entry {
	struct  {
		u64	valid	: 1,
			res1	: 5,
			gpid	: 42,
			res2	: 8,
			vector	: 8;
	}	lo;
	u64	hi;
};

struct redirect_gpid {
	u64	pir[4];      // Pending interrupt requested
	u8	en	: 1, // doorbell
		res0	: 7;
	u8	irqnum;
	u16	res1;
	u32	dst;
	u32	rsvd[6];
};

struct irde_inv_cmd {
	union {
		u64	cmd_info;
		struct {
			u64	res1		: 4,
				type		: 1,
				need_notice	: 1,
				pad		: 2,
				index		: 16,
				pad2		: 40;
		}	index;
	};
	u64		notice_addr;
};

struct redirect_table {
	struct redirect_entry	*table;
	unsigned long		*bitmap;
	raw_spinlock_t		lock;
};

struct redirect_queue {
	struct irde_inv_cmd	*cmd_base;
	int			head;
	int			tail;
	raw_spinlock_t		lock;
};

struct irde_desc {
	struct	redirect_table	ird_table;
	struct	redirect_queue	inv_queue;
	int			node;
};

struct redirect_item {
	int			index;
	struct irde_desc	*irde;
	struct redirect_gpid	*gpid;
};


static struct irq_domain *redirect_domain;
static struct irde_desc irde_descs[MAX_IR_ENGINES];
static phys_addr_t msi_base_addr;
static phys_addr_t redirect_reg_base = LOONGSON_REG_BASE;

#define REDIRECT_REG(reg, node) \
	((void *)(IO_BASE | redirect_reg_base | (u64)(node) << NODE_ADDRSPACE_SHIFT | (reg)))

static inline void redirect_write_reg64(u32 node, u64 val, u32 reg)
{
	void __iomem *reg_addr = REDIRECT_REG(reg, node);

	return writeq(val, reg_addr);
}

static inline void redirect_write_reg32(int node, u32 val, u32 reg)
{
	void __iomem *reg_addr = REDIRECT_REG(reg, node);

	return writel(val, reg_addr);
}

static inline u32 redirect_read_reg32(int node, int reg)
{
	void __iomem *reg_addr = REDIRECT_REG(reg, node);

	return readl(reg_addr);
}

static inline struct redirect_entry *item_get_entry(struct redirect_item *item)
{
	return item->irde->ird_table.table + item->index;
}

static inline bool invalid_queue_is_full(int node, u32 *tail)
{
	u32 head = redirect_read_reg32(node, LOONGARCH_IOCSR_REDIRECT_CQH);

	*tail = redirect_read_reg32(node, LOONGARCH_IOCSR_REDIRECT_CQT);

	return head == ((*tail + 1) % INVALID_QUEUE_SIZE);
}

static void invalid_enqueue(struct redirect_item *item, struct irde_inv_cmd *cmd)
{
	struct redirect_queue *inv_queue = &item->irde->inv_queue;
	u32 tail;

	guard(raw_spinlock_irqsave)(&inv_queue->lock);

	while (invalid_queue_is_full(item->irde->node, &tail))
		cpu_relax();

	memcpy(&inv_queue->cmd_base[tail], cmd, sizeof(*cmd));

	redirect_write_reg32(item->irde->node, (tail + 1) % INVALID_QUEUE_SIZE,
			     LOONGARCH_IOCSR_REDIRECT_CQT);
}

static void irde_invalid_entry_node(struct redirect_item *item)
{
	struct irde_inv_cmd cmd;
	u64 raddr = 0;

	cmd.cmd_info = 0;
	cmd.index.type = INVALID_INDEX;
	cmd.index.need_notice = 1;
	cmd.index.index = item->index;
	cmd.notice_addr = (u64)(__pa(&raddr));

	invalid_enqueue(item, &cmd);

	while (!raddr)
		cpu_relax();

}

static inline struct avecintc_data *irq_data_get_avec_data(struct irq_data *data)
{
	return data->parent_data->chip_data;
}

static int redirect_table_alloc(struct redirect_item *item)
{
	struct redirect_table *ird_table = &item->irde->ird_table;
	int index;

	guard(raw_spinlock_irqsave)(&ird_table->lock);

	index = find_first_zero_bit(ird_table->bitmap, IRD_ENTRIES);
	if (index > IRD_ENTRIES) {
		pr_err("No redirect entry to use\n");
		return -ENOMEM;
	}

	__set_bit(index, ird_table->bitmap);

	item->index = index;

	return 0;
}

static void redirect_table_free(struct redirect_item *item)
{
	struct redirect_table *ird_table = &item->irde->ird_table;
	struct redirect_entry *entry = item_get_entry(item);

	memset(entry, 0, sizeof(*entry));

	scoped_guard(raw_spinlock_irq, &ird_table->lock)
		bitmap_release_region(ird_table->bitmap, item->index, 0);

	kfree(item->gpid);

	irde_invalid_entry_node(item);
}

static inline void redirect_domain_prepare_entry(struct redirect_item *item,
						 struct avecintc_data *adata)
{
	struct redirect_entry *entry = item_get_entry(item);

	item->gpid->en = 1;
	item->gpid->irqnum = adata->vec;
	item->gpid->dst = adata->cpu;

	entry->lo.valid = 1;
	entry->lo.gpid = ((u64)item->gpid & GPID_ADDR_MASK) >> GPID_ADDR_SHIFT;
	entry->lo.vector = 0xff;
}

static int redirect_set_affinity(struct irq_data *data, const struct cpumask *dest, bool force)
{
	struct redirect_item *item = data->chip_data;
	int ret;

	ret = irq_chip_set_affinity_parent(data, dest, force);
	if (ret == IRQ_SET_MASK_OK_DONE) {
		return ret;
	} else if (ret) {
		pr_err("IRDE:set_affinity error %d\n", ret);
		return ret;
	} else {
		struct avecintc_data *adata = irq_data_get_avec_data(data);

		redirect_domain_prepare_entry(item, adata);
		irde_invalid_entry_node(item);
		avecintc_sync(adata);
	}

	return IRQ_SET_MASK_OK;
}

static void redirect_compose_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct redirect_item *item = irq_data_get_irq_chip_data(d);

	msg->address_lo = (msi_base_addr | 1 << 2 | ((item->index & 0xffff) << 4));
	msg->address_hi = 0x0;
	msg->data = 0x0;
}

static struct irq_chip loongarch_redirect_chip = {
	.name			= "REDIRECT",
	.irq_ack		= irq_chip_ack_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_set_affinity	= redirect_set_affinity,
	.irq_compose_msi_msg	= redirect_compose_msi_msg,
};

static void redirect_free_resources(struct irq_domain *domain, unsigned int virq,
				    unsigned int nr_irqs)
{
	for (int i = 0; i < nr_irqs; i++) {
		struct irq_data *irq_data = irq_domain_get_irq_data(domain, virq + i);

		if (irq_data && irq_data->chip_data) {
			struct redirect_item *item = irq_data->chip_data;

			redirect_table_free(item);
			kfree(item);
		}
	}
}

static int redirect_domain_alloc(struct irq_domain *domain, unsigned int virq,
				 unsigned int nr_irqs, void *arg)
{
	msi_alloc_info_t *info = arg;
	int ret, i, node;

	node = dev_to_node(info->desc->dev);

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, arg);
	if (ret < 0)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *irq_data = irq_domain_get_irq_data(domain, virq + i);
		struct redirect_item *item;

		item = kzalloc(sizeof(*item), GFP_KERNEL);
		if (!item) {
			pr_err("Alloc redirect descriptor failed\n");
			goto out_free_resources;
		}
		item->irde = &irde_descs[node];

		ret = redirect_table_alloc(item);
		if (ret) {
			pr_err("Alloc redirect table entry failed\n");
			goto out_free_resources;
		}
		/*
		 * Only bits 47:6 of the GPID are passed to the controller,
		 * 64-byte alignment must be guaranteed.
		 */
		BUILD_BUG_ON(sizeof(*item->gpid) != 64);
		item->gpid = kzalloc_node(sizeof(*item->gpid), GFP_KERNEL, node);
		if (!item->gpid) {
			pr_err("Alloc redirect GPID failed\n");
			goto out_free_resources;
		}

		irq_data->chip_data = item;
		irq_data->chip = &loongarch_redirect_chip;

		redirect_domain_prepare_entry(item, irq_data_get_avec_data(irq_data));
	}
	return 0;

out_free_resources:
	redirect_free_resources(domain, virq, nr_irqs);
	irq_domain_free_irqs_common(domain, virq, nr_irqs);

	return -ENOMEM;
}

static void redirect_domain_free(struct irq_domain *domain, unsigned int virq, unsigned int nr_irqs)
{
	redirect_free_resources(domain, virq, nr_irqs);
	return irq_domain_free_irqs_common(domain, virq, nr_irqs);
}

static const struct irq_domain_ops redirect_domain_ops = {
	.alloc		= redirect_domain_alloc,
	.free		= redirect_domain_free,
	.select		= msi_lib_irq_domain_select,
};

static int redirect_queue_init(struct irde_desc *irde)
{
	struct redirect_queue *inv_queue = &irde->inv_queue;
	struct folio *folio;

	folio = __folio_alloc_node(GFP_KERNEL | __GFP_ZERO, INV_QUEUE_PAGE_ORDER, irde->node);
	if (!folio) {
		pr_err("Node [%d] invalid queue alloc pages failed!\n", irde->node);
		return -ENOMEM;
	}

	inv_queue->cmd_base = folio_address(folio);
	inv_queue->head = 0;
	inv_queue->tail = 0;
	raw_spin_lock_init(&inv_queue->lock);

	return 0;
}

static int redirect_table_init(struct irde_desc *irde)
{
	struct redirect_table *ird_table = &irde->ird_table;
	struct folio *folio;
	unsigned long *bitmap;
	int node = irde->node;

	folio = __folio_alloc_node(GFP_KERNEL | __GFP_ZERO, IRD_TABLE_PAGE_ORDER, node);
	if (!folio) {
		pr_err("Node [%d] redirect table alloc pages failed!\n", node);
		goto fail;
	}
	ird_table->table = folio_address(folio);

	bitmap = bitmap_zalloc(IRD_ENTRIES, GFP_KERNEL);
	if (!bitmap) {
		pr_err("Node [%d] redirect table bitmap alloc pages failed!\n", node);
		goto free_pages;
	}

	ird_table->bitmap = bitmap;

	raw_spin_lock_init(&ird_table->lock);

	return 0;

free_pages:
	folio_put(folio);
	ird_table->table = NULL;
fail:
	return -ENOMEM;
}

static void redirect_irde_cfg(struct irde_desc *irde)
{
	redirect_write_reg64(irde->node, CFG_DISABLE_IDLE, LOONGARCH_IOCSR_REDIRECT_CFG);
	redirect_write_reg64(irde->node, __pa(irde->ird_table.table), LOONGARCH_IOCSR_REDIRECT_TBR);
	redirect_write_reg32(irde->node, 0, LOONGARCH_IOCSR_REDIRECT_CQH);
	redirect_write_reg32(irde->node, 0, LOONGARCH_IOCSR_REDIRECT_CQT);
	redirect_write_reg64(irde->node, ((u64)irde->inv_queue.cmd_base & CQB_ADDR_MASK) |
			     CQB_SIZE_MASK, LOONGARCH_IOCSR_REDIRECT_CQB);
}

static void __redirect_irde_fini(struct irde_desc *irde)
{
	struct redirect_table *ird_table = &irde_descs->ird_table;
	struct redirect_queue *inv_queue = &irde_descs->inv_queue;

	if (ird_table->table) {
		folio_put(virt_to_folio(ird_table->table));
		ird_table->table = NULL;
	}

	if (ird_table->bitmap) {
		bitmap_free(ird_table->bitmap);
		ird_table->bitmap = NULL;
	}

	if (inv_queue->cmd_base) {
		folio_put(virt_to_folio(inv_queue->cmd_base));
		inv_queue->cmd_base = NULL;
	}
}

static inline void redirect_irde_fini(int node)
{
	__redirect_irde_fini(&irde_descs[node]);
}

static int redirect_irde_init(int node)
{
	struct irde_desc *irde = &irde_descs[node];
	int ret;

	irde->node = node;

	ret = redirect_table_init(irde);
	if (ret)
		goto fail;

	ret = redirect_queue_init(irde);
	if (ret)
		goto fail_clear;

	redirect_irde_cfg(irde);

	return 0;

fail_clear:
	__redirect_irde_fini(irde);
fail:
	return ret;
}

#ifdef CONFIG_ACPI
static int __init pch_msi_parse_madt(union acpi_subtable_headers *header, const unsigned long end)
{
	struct acpi_madt_msi_pic *pchmsi_entry = (struct acpi_madt_msi_pic *)header;

	msi_base_addr = pchmsi_entry->msg_address - AVEC_MSG_OFFSET;

	return pch_msi_acpi_init_avec(redirect_domain);
}

static int __init acpi_cascade_irqdomain_init(void)
{
	return acpi_table_parse_madt(ACPI_MADT_TYPE_MSI_PIC, pch_msi_parse_madt, 1);
}

int __init redirect_acpi_init(struct irq_domain *parent)
{
	struct fwnode_handle *fwnode;
	int ret, node;

	fwnode = irq_domain_alloc_named_fwnode("redirect");
	if (!fwnode) {
		pr_err("Unable to alloc redirect domain handle\n");
		goto fail;
	}

	redirect_domain = irq_domain_create_hierarchy(parent, 0, IRD_ENTRIES, fwnode,
						      &redirect_domain_ops, irde_descs);
	if (!redirect_domain) {
		pr_err("Unable to alloc redirect domain\n");
		goto out_free_fwnode;
	}


	for_each_node_mask(node, node_possible_map) {
		ret = redirect_irde_init(node);
		if (ret)
			goto out_clear_irde;
	}

	ret = acpi_cascade_irqdomain_init();
	if (ret < 0)
		pr_err("Failed to cascade IRQ domain, ret=%d\n", ret);

	pr_info("loongarch irq redirect modules init succeeded\n");
	return 0;

out_clear_irde:
	for_each_node_mask(node, node_possible_map)
		redirect_irde_fini(node);

	irq_domain_remove(redirect_domain);
out_free_fwnode:
	irq_domain_free_fwnode(fwnode);
fail:
	return ret;
}
#endif
