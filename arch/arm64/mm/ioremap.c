// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on arch/arm/mm/ioremap.c
 *
 * (C) Copyright 1995 1996 Linus Torvalds
 * Hacked for ARM by Phil Blundell <philb@gnu.org>
 * Hacked to allow all architectures to build, and various cleanups
 * by Russell King
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/export.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/io.h>

#include <asm/fixmap.h>
#include <asm/tlbflush.h>

#if CONFIG_IOTYPE_RANGES

/*
 * This feature allows a device tree (see parsing in arch/arm64/mm/init.c)
 * to override device I/O type for specified ranges. It is intended for use
 * in cases where the SoC bus implementation does not work correctly with
 * the default nGnRE mode.
 */

#define MAX_IOTYPE_RANGES	8

#define IOTYPE_RANGE_nGnRE	0
#define IOTYPE_RANGE_nGnRnE	1

static struct iotype_range {
	phys_addr_t addr, size;
	u32 type;
} iotype_ranges[MAX_IOTYPE_RANGES];
static unsigned num_iotype_ranges = 0;

int ioremap_add_io_type_range(phys_addr_t addr, phys_addr_t size, u32 type)
{
	if(num_iotype_ranges >= MAX_IOTYPE_RANGES)
		return 1;

	iotype_ranges[num_iotype_ranges].addr = addr;
	iotype_ranges[num_iotype_ranges].size = size;
	iotype_ranges[num_iotype_ranges].type = type;
	num_iotype_ranges ++;

	return 0;
}

static pgprot_t apply_io_type_range_prot(phys_addr_t addr, pgprot_t prot)
{
	/* linear search... for now at least there's very few ranges */
	unsigned i;

	for (i = 0; i < num_iotype_ranges; i ++)
		if(addr >= iotype_ranges[i].addr &&
		   addr - iotype_ranges[i].addr < iotype_ranges[i].size) {
			switch(iotype_ranges[i].type) {
			case IOTYPE_RANGE_nGnRE:
				return __pgprot(PROT_DEVICE_nGnRE);
			case IOTYPE_RANGE_nGnRnE:
				return __pgprot(PROT_DEVICE_nGnRnE);
			}
		}

	return prot;
}

#else
#define apply_io_type_range_prot(addr,prot) (prot)
#endif

static void __iomem *__ioremap_caller(phys_addr_t phys_addr, size_t size,
				      pgprot_t prot, void *caller)
{
	unsigned long last_addr;
	unsigned long offset = phys_addr & ~PAGE_MASK;
	int err;
	unsigned long addr;
	struct vm_struct *area;

	/*
	 * Page align the mapping address and size, taking account of any
	 * offset.
	 */
	phys_addr &= PAGE_MASK;
	size = PAGE_ALIGN(size + offset);

	/*
	 * Allow system MMIO access specification to override mapping type.
	 */
	prot = apply_io_type_range_prot(phys_addr, prot);

	/*
	 * Don't allow wraparound, zero size or outside PHYS_MASK.
	 */
	last_addr = phys_addr + size - 1;
	if (!size || last_addr < phys_addr || (last_addr & ~PHYS_MASK))
		return NULL;

	/*
	 * Don't allow RAM to be mapped.
	 */
	if (WARN_ON(pfn_valid(__phys_to_pfn(phys_addr))))
		return NULL;

	area = get_vm_area_caller(size, VM_IOREMAP, caller);
	if (!area)
		return NULL;
	addr = (unsigned long)area->addr;
	area->phys_addr = phys_addr;

	err = ioremap_page_range(addr, addr + size, phys_addr, prot);
	if (err) {
		vunmap((void *)addr);
		return NULL;
	}

	return (void __iomem *)(offset + addr);
}

void __iomem *__ioremap(phys_addr_t phys_addr, size_t size, pgprot_t prot)
{
	return __ioremap_caller(phys_addr, size, prot,
				__builtin_return_address(0));
}
EXPORT_SYMBOL(__ioremap);

void iounmap(volatile void __iomem *io_addr)
{
	unsigned long addr = (unsigned long)io_addr & PAGE_MASK;

	/*
	 * We could get an address outside vmalloc range in case
	 * of ioremap_cache() reusing a RAM mapping.
	 */
	if (is_vmalloc_addr((void *)addr))
		vunmap((void *)addr);
}
EXPORT_SYMBOL(iounmap);

void __iomem *ioremap_cache(phys_addr_t phys_addr, size_t size)
{
	/* For normal memory we already have a cacheable mapping. */
	if (pfn_valid(__phys_to_pfn(phys_addr)))
		return (void __iomem *)__phys_to_virt(phys_addr);

	return __ioremap_caller(phys_addr, size, __pgprot(PROT_NORMAL),
				__builtin_return_address(0));
}
EXPORT_SYMBOL(ioremap_cache);

/*
 * Must be called after early_fixmap_init
 */
void __init early_ioremap_init(void)
{
	early_ioremap_setup();
}
