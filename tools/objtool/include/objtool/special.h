/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2015 Josh Poimboeuf <jpoimboe@redhat.com>
 */

#ifndef _SPECIAL_H
#define _SPECIAL_H

#include <stdbool.h>
#include <objtool/check.h>
#include <objtool/elf.h>

#define C_JUMP_TABLE_SECTION ".rodata..c_jump_table"

struct instruction;

struct special_alt {
	struct list_head list;

	bool group;
	bool skip_orig;
	bool skip_alt;
	bool jump_or_nop;
	u8 key_addend;

	struct section *orig_sec;
	unsigned long orig_off;

	struct section *new_sec;
	unsigned long new_off;

	unsigned int orig_len, new_len; /* group only */
};

struct alternative {
	struct alternative *next;
	struct instruction *insn;
	bool skip_orig;
};

int special_get_alts(struct elf *elf, struct list_head *alts);

void arch_handle_alternative(unsigned short feature, struct special_alt *alt);

bool arch_support_alt_relocation(struct special_alt *special_alt,
				 struct instruction *insn,
				 struct reloc *reloc);
void arch_mark_func_jump_tables(struct objtool_file *file,
				struct symbol *func);

int arch_dynamic_add_jump_table_alts(struct list_head *p_orbit_list, struct objtool_file *file,
				struct symbol *func, struct instruction *insn);

bool arch_is_noreturn(struct symbol *func);
#endif /* _SPECIAL_H */
