/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_EXCEPTION_H
#define __ASM_EXCEPTION_H

#include <linux/kprobes.h>

struct pt_regs;
struct exception_table_entry;

extern int die (const char *str, struct pt_regs *regs, long err);
extern int die_if_kernel (char *str, struct pt_regs *regs, long err);
extern void ia64_handle_exception(struct pt_regs *regs,
				  const struct exception_table_entry *e);
extern void
__kprobes ia64_do_page_fault (unsigned long address, unsigned long isr,
			      struct pt_regs *regs);
extern void
__kprobes ia64_bad_break (unsigned long break_num, struct pt_regs *regs);

extern struct illegal_op_return
ia64_illegal_op_fault (unsigned long ec, long arg1, long arg2, long arg3,
		       long arg4, long arg5, long arg6, long arg7,
		       struct pt_regs regs);

extern void __kprobes
ia64_fault (unsigned long vector, unsigned long isr, unsigned long ifa,
	    unsigned long iim, unsigned long itir, long arg5, long arg6,
	    long arg7, struct pt_regs regs);

#define ia64_done_with_exception(regs)					  \
({									  \
	int __ex_ret = 0;						  \
	const struct exception_table_entry *e;				  \
	e = search_exception_tables((regs)->cr_iip + ia64_psr(regs)->ri); \
	if (e) {							  \
		ia64_handle_exception(regs, e);				  \
		__ex_ret = 1;						  \
	}								  \
	__ex_ret;							  \
})

#endif	/* __ASM_EXCEPTION_H */
