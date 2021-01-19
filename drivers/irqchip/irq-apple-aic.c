/*
 * Apple chip interrupt controller
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 1992, 1998 Linus Torvalds, Ingo Molnar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/exception.h>
#include <asm/irq.h>

#define REG_ID_REVISION                 0x0000
#define REG_ID_CONFIG                   0x0004
#define REG_GLOBAL_CFG                  0x0010
#define REG_TIME_LO                     0x0020
#define REG_TIME_HI                     0x0028
#define REG_ID_CPUID                    0x2000
#define REG_IRQ_ACK                     0x2004
#define  REG_IRQ_ACK_TYPE_MASK          (15 << 16)
#define   REG_IRQ_ACK_TYPE_NONE         (0 << 16)
#define   REG_IRQ_ACK_TYPE_IRQ          (1 << 16)
#define   REG_IRQ_ACK_TYPE_IPI          (4 << 16)
#define    REG_IRQ_ACK_IPI_OTHER        0x40001
#define    REG_IRQ_ACK_IPI_SELF         0x40002
#define  REG_IRQ_ACK_NUM_MASK           (4095)
#define REG_IPI_SET                     0x2008
#define   REG_IPI_FLAG_SELF             (1 << 31)
#define   REG_IPI_FLAG_OTHER            (1 << 0)
#define REG_IPI_CLEAR                   0x200C
#define REG_IPI_DISABLE                 0x2024
#define REG_IPI_ENABLE                  0x2028
#define REG_IPI_DEFER_SET               0x202C
#define REG_IPI_DEFER_CLEAR             0x2030
#define REG_TSTAMP_CTRL                 0x2040
#define REG_TSTAMP_LO                   0x2048
#define REG_TSTAMP_HI                   0x204C
#define REG_IRQ_AFFINITY(i)             (0x3000 + ((i) << 2))
#define REG_IRQ_DISABLE(i)              (0x4100 + (((i) >> 5) << 2))
#define  REG_IRQ_xABLE_MASK(i)          (1 << ((i) & 31))
#define REG_IRQ_ENABLE(i)               (0x4180 + (((i) >> 5) << 2))
#define REG_CPU_REGION                  0x5000
#define  REG_CPU_LOCAL                  0x2000
#define  REG_CPU_SHIFT                  7
#define  REG_PERCPU(r,c)                ((r)+REG_CPU_REGION-REG_CPU_LOCAL+((c)<<REG_CPU_SHIFT))

static struct aic_chip_data {
    void __iomem *base;
    struct irq_domain *domain;
    unsigned int num_irqs;
} aic;

static void apple_aic_irq_mask(struct irq_data *d)
{
    writel(REG_IRQ_xABLE_MASK(d->hwirq), aic.base + REG_IRQ_DISABLE(d->hwirq));
}

static void apple_aic_irq_unmask(struct irq_data *d)
{
    writel(REG_IRQ_xABLE_MASK(d->hwirq), aic.base + REG_IRQ_ENABLE(d->hwirq));
}

static struct irq_chip apple_aic_irq_chip = {
    .name         = "apple_aic",
    .irq_mask     = apple_aic_irq_mask,
    .irq_mask_ack = apple_aic_irq_mask,
    .irq_unmask   = apple_aic_irq_unmask,
};

static void apple_aic_fiq_mask(struct irq_data *d)
{
}

static void apple_aic_fiq_unmask(struct irq_data *d)
{
}

static struct irq_chip apple_aic_irq_chip_fiq = {
    .name        = "apple_aic_fiq",
    .irq_mask    = apple_aic_fiq_mask,
    .irq_unmask  = apple_aic_fiq_unmask,
};

static int apple_aic_irq_domain_xlate(struct irq_domain *d, struct device_node *ctrlr, const u32 *intspec, unsigned int intsize, unsigned long *out_hwirq, unsigned int *out_type)
{
    if(intspec[0]) { /* FIQ */
        if(intspec[1] >= 2)
            return -EINVAL;
        if(out_hwirq)
            *out_hwirq = aic.num_irqs + intspec[1];
    } else {
        if(intspec[1] >= aic.num_irqs)
            return -EINVAL;
        if(out_hwirq)
            *out_hwirq = intspec[1];
    }

    if(out_type)
        *out_type = intspec[2] & IRQ_TYPE_SENSE_MASK;
    return 0;
}

static int apple_aic_irq_domain_map(struct irq_domain *d, unsigned int virq, irq_hw_number_t hw)
{
    if(hw >= aic.num_irqs) {
        irq_set_percpu_devid(virq);
        irq_domain_set_info(d, virq, hw, &apple_aic_irq_chip_fiq, d->host_data, handle_percpu_devid_irq, NULL, NULL);
        irq_set_status_flags(virq, IRQ_NOAUTOEN);
    } else {
        irq_domain_set_info(d, virq, hw, &apple_aic_irq_chip, d->host_data, handle_level_irq, NULL, NULL);
        irq_set_probe(virq);
        irqd_set_single_target(irq_desc_get_irq_data(irq_to_desc(virq)));
    }
    return 0;
}

static const struct irq_domain_ops apple_aic_irq_domain_ops = {
    .xlate = apple_aic_irq_domain_xlate,
    .map = apple_aic_irq_domain_map,
};

static void __exception_irq_entry apple_aic_handle_irq(struct pt_regs *regs)
{
    uint32_t ack;
    unsigned done = 0;

    while(1) {
        ack = readl(aic.base + REG_IRQ_ACK);
        switch(ack & REG_IRQ_ACK_TYPE_MASK) {
        case REG_IRQ_ACK_TYPE_NONE:
            done = 1;
            break;
        case REG_IRQ_ACK_TYPE_IRQ:
            handle_domain_irq(aic.domain, ack & REG_IRQ_ACK_NUM_MASK, regs);
            break;
        }
        if(done)
            break;
    }
}

static void __exception_irq_entry apple_aic_handle_fiq(struct pt_regs *regs)
{
    handle_domain_irq(aic.domain, aic.num_irqs, regs);
}

void apple_aic_cpu_prepare(unsigned int cpu)
{
    unsigned i;

    for(i=0; i<aic.num_irqs; i++)
        writel(readl(aic.base + REG_IRQ_AFFINITY(i)) | (1u << cpu), aic.base + REG_IRQ_AFFINITY(i));
}

static int __init apple_aic_init(struct device_node *node, struct device_node *interrupt_parent)
{
    unsigned i;

    if(!node)
        return -ENODEV;

    aic.base = of_iomap(node, 0);
    if(WARN(!aic.base, "unable to map aic registers\n"))
        return -EINVAL;

    aic.num_irqs = readl(aic.base + REG_ID_CONFIG) & 0xFFFF;
    pr_info("Apple AIC: %d IRQs + 1 FIQ + 1 dummy\n", aic.num_irqs);

    for(i=0; i<aic.num_irqs; i++)
        writel(1, aic.base + REG_IRQ_AFFINITY(i));
    for(i=0; i<aic.num_irqs; i+=32)
        writel(-1u, aic.base + REG_IRQ_DISABLE(i));
    writel((readl(aic.base + REG_GLOBAL_CFG) & ~0xF00000) | 0x700000, aic.base + REG_GLOBAL_CFG);

    set_handle_irq(apple_aic_handle_irq);
    set_handle_fiq(apple_aic_handle_fiq);

    apple_aic_cpu_prepare(0);

    aic.domain = irq_domain_add_linear(node, aic.num_irqs + 2, &apple_aic_irq_domain_ops, &apple_aic_irq_chip);
    irq_set_default_host(aic.domain);
    return 0;
}

IRQCHIP_DECLARE(apple_aic_irq_chip, "apple,aic", apple_aic_init);
