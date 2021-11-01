// SPDX-License-Identifier: GPL-2.0-or-later

#include "elf.h"
#include "check.h"

void arch_print_reg(struct elf *dummy_elf, struct orc_entry orc);
int arch_write_orc_entry(struct elf *elf, struct section *orc_sec,
			   struct section *ip_sec, unsigned int idx,
			   struct section *insn_sec, unsigned long insn_off,
			   struct orc_entry *o);
