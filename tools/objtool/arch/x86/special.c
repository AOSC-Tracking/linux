// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>

#include <objtool/special.h>
#include <objtool/builtin.h>

#define X86_FEATURE_POPCNT (4 * 32 + 23)
#define X86_FEATURE_SMAP   (9 * 32 + 20)

void arch_handle_alternative(unsigned short feature, struct special_alt *alt)
{
	switch (feature) {
	case X86_FEATURE_SMAP:
		/*
		 * If UACCESS validation is enabled; force that alternative;
		 * otherwise force it the other way.
		 *
		 * What we want to avoid is having both the original and the
		 * alternative code flow at the same time, in that case we can
		 * find paths that see the STAC but take the NOP instead of
		 * CLAC and the other way around.
		 */
		if (opts.uaccess)
			alt->skip_orig = true;
		else
			alt->skip_alt = true;
		break;
	case X86_FEATURE_POPCNT:
		/*
		 * It has been requested that we don't validate the !POPCNT
		 * feature path which is a "very very small percentage of
		 * machines".
		 */
		alt->skip_orig = true;
		break;
	default:
		break;
	}
}

bool arch_support_alt_relocation(struct special_alt *special_alt,
				 struct instruction *insn,
				 struct reloc *reloc)
{
	/*
	 * The x86 alternatives code adjusts the offsets only when it
	 * encounters a branch instruction at the very beginning of the
	 * replacement group.
	 */
	return insn->offset == special_alt->new_off &&
	       (insn->type == INSN_CALL || is_jump(insn));
}

/*
 * There are 3 basic jump table patterns:
 *
 * 1. jmpq *[rodata addr](,%reg,8)
 *
 *    This is the most common case by far.  It jumps to an address in a simple
 *    jump table which is stored in .rodata.
 *
 * 2. jmpq *[rodata addr](%rip)
 *
 *    This is caused by a rare GCC quirk, currently only seen in three driver
 *    functions in the kernel, only with certain obscure non-distro configs.
 *
 *    As part of an optimization, GCC makes a copy of an existing switch jump
 *    table, modifies it, and then hard-codes the jump (albeit with an indirect
 *    jump) to use a single entry in the table.  The rest of the jump table and
 *    some of its jump targets remain as dead code.
 *
 *    In such a case we can just crudely ignore all unreachable instruction
 *    warnings for the entire object file.  Ideally we would just ignore them
 *    for the function, but that would require redesigning the code quite a
 *    bit.  And honestly that's just not worth doing: unreachable instruction
 *    warnings are of questionable value anyway, and this is such a rare issue.
 *
 * 3. mov [rodata addr],%reg1
 *    ... some instructions ...
 *    jmpq *(%reg1,%reg2,8)
 *
 *    This is a fairly uncommon pattern which is new for GCC 6.  As of this
 *    writing, there are 11 occurrences of it in the allmodconfig kernel.
 *
 *    As of GCC 7 there are quite a few more of these and the 'in between' code
 *    is significant. Esp. with KASAN enabled some of the code between the mov
 *    and jmpq uses .rodata itself, which can confuse things.
 *
 *    TODO: Once we have DWARF CFI and smarter instruction decoding logic,
 *    ensure the same register is used in the mov and jump instructions.
 *
 *    NOTE: RETPOLINE made it harder still to decode dynamic jumps.
 */
static struct reloc *find_switch_table(struct objtool_file *file,
				    struct instruction *insn)
{
	struct reloc  *text_reloc, *rodata_reloc;
	struct section *table_sec;
	unsigned long table_offset;

	/* look for a relocation which references .rodata */
	text_reloc = find_reloc_by_dest_range(file->elf, insn->sec,
					      insn->offset, insn->len);
	if (!text_reloc || text_reloc->sym->type != STT_SECTION ||
	    !text_reloc->sym->sec->rodata)
		return NULL;

	table_offset = text_reloc->addend;
	table_sec = text_reloc->sym->sec;

	if (text_reloc->type == R_X86_64_PC32)
		table_offset += 4;

	/*
	 * Make sure the .rodata address isn't associated with a
	 * symbol.  GCC jump tables are anonymous data.
	 *
	 * Also support C jump tables which are in the same format as
	 * switch jump tables.  For objtool to recognize them, they
	 * need to be placed in the C_JUMP_TABLE_SECTION section.  They
	 * have symbols associated with them.
	 */
	if (find_symbol_containing(table_sec, table_offset) &&
	    strcmp(table_sec->name, C_JUMP_TABLE_SECTION))
		return NULL;

	/*
	 * Each table entry has a rela associated with it.  The rela
	 * should reference text in the same function as the original
	 * instruction.
	 */
	rodata_reloc = find_reloc_by_dest(file->elf, table_sec, table_offset);
	if (!rodata_reloc)
		return NULL;

	/*
	 * Use of RIP-relative switch jumps is quite rare, and
	 * indicates a rare GCC quirk/bug which can leave dead
	 * code behind.
	 */
	if (text_reloc->type == R_X86_64_PC32)
		file->ignore_unreachables = true;

	return rodata_reloc;
}

int arch_dynamic_add_jump_table_alts(struct list_head *p_orbit_list, struct objtool_file *file,
			struct symbol *func, struct instruction *insn)
{
	return 0;
}

/*
 * find_jump_table() - Given a dynamic jump, find the switch jump table
 * associated with it.
 */
static struct reloc *find_jump_table(struct objtool_file *file,
				struct symbol *func,
				struct instruction *insn)
{
	struct reloc *table_reloc;
	struct instruction *dest_insn, *orig_insn = insn;

	/*
	 * Backward search using the @first_jump_src links, these help avoid
	 * much of the 'in between' code. Which avoids us getting confused by
	 * it.
	 */
	for (;
	     insn && insn_func(insn) && insn_func(insn)->pfunc == func;
	     insn = insn->first_jump_src ?: prev_insn_same_sym(file, insn)) {

		if (insn != orig_insn && insn->type == INSN_JUMP_DYNAMIC)
			break;

		/* allow small jumps within the range */
		if (insn->type == INSN_JUMP_UNCONDITIONAL &&
		    insn->jump_dest &&
		    (insn->jump_dest->offset <= insn->offset ||
		    insn->jump_dest->offset > orig_insn->offset))
			break;

		table_reloc = find_switch_table(file, insn);
		if (!table_reloc)
			continue;
		dest_insn = find_insn(file, table_reloc->sym->sec, table_reloc->addend);
		if (!dest_insn || !insn_func(dest_insn) || insn_func(dest_insn)->pfunc != func)
			continue;

		return table_reloc;
	}

	return NULL;
}

/*
 * First pass: Mark the head of each jump table so that in the next pass,
 * we know when a given jump table ends and the next one starts.
 */
void arch_mark_func_jump_tables(struct objtool_file *file,
				struct symbol *func)
{
	struct instruction *insn, *last = NULL;
	struct reloc *reloc;

	func_for_each_insn(file, func, insn) {
		if (!last)
			last = insn;

		/*
		 * Store back-pointers for unconditional forward jumps such
		 * that find_jump_table() can back-track using those and
		 * avoid some potentially confusing code.
		 */
		if (insn->type == INSN_JUMP_UNCONDITIONAL && insn->jump_dest &&
				insn->offset > last->offset &&
				insn->jump_dest->offset > insn->offset &&
				!insn->jump_dest->first_jump_src) {

			insn->jump_dest->first_jump_src = insn;
			last = insn->jump_dest;
		}

		if (insn->type != INSN_JUMP_DYNAMIC)
			continue;

		reloc = find_jump_table(file, func, insn);
		if (reloc) {
			reloc->jump_table_start = true;
			insn->_jump_table = reloc;
		}
	}
}

/*
 * Unfortunately these have to be hard coded because the noreturn
 * attribute isn't provided in ELF data. Keep 'em sorted.
 */
bool arch_is_noreturn(struct symbol *func)
{
	int i;

	static const char * const arch_noreturns[] = {
		"__invalid_creds",
		"__module_put_and_kthread_exit",
		"__reiserfs_panic",
		"__stack_chk_fail",
		"__ubsan_handle_builtin_unreachable",
		"arch_call_rest_init",
		"arch_cpu_idle_dead",
		"btrfs_assertfail",
		"cpu_bringup_and_idle",
		"cpu_startup_entry",
		"do_exit",
		"do_group_exit",
		"do_task_dead",
		"ex_handler_msr_mce",
		"fortify_panic",
		"hlt_play_dead",
		"hv_ghcb_terminate",
		"kthread_complete_and_exit",
		"kthread_exit",
		"kunit_try_catch_throw",
		"lbug_with_loc",
		"machine_real_restart",
		"make_task_dead",
		"mpt_halt_firmware",
		"nmi_panic_self_stop",
		"panic",
		"panic_smp_self_stop",
		"rest_init",
		"resume_play_dead",
		"rewind_stack_and_make_dead",
		"sev_es_terminate",
		"snp_abort",
		"start_kernel",
		"stop_this_cpu",
		"usercopy_abort",
		"x86_64_start_kernel",
		"x86_64_start_reservations",
		"xen_cpu_bringup_again",
		"xen_start_kernel",
	};

	for (i = 0; i < ARRAY_SIZE(arch_noreturns); i++)
		if (!strcmp(func->name, arch_noreturns[i]))
			return true;

	return false;
}
