/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_IRQ_H
#define __ASM_IRQ_H

#ifndef __ASSEMBLER__

#include <asm-generic/irq.h>

struct pt_regs;

static inline int nr_legacy_irqs(void)
{
	return 0;
}

int set_handle_fiq(void (*handle_fiq)(struct pt_regs *));

extern void (*handle_arch_fiq)(struct pt_regs *) __ro_after_init;

#endif /* !__ASSEMBLER__ */
#endif
