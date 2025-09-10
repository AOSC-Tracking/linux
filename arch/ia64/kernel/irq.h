/* SPDX-License-Identifier: GPL-2.0 */
extern void ia64_process_pending_intr(void);
extern void ia64_handle_irq (ia64_vector vector, struct pt_regs *regs);
extern void ia64_init_itm(void);
extern void register_percpu_irq(ia64_vector vec, irq_handler_t handler,
				unsigned long flags, const char *name);
extern void fixup_irqs(void);
