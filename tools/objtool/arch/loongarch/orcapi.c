// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>
#include <stdlib.h>

#include <asm/orc_types.h>
#include <linux/objtool_types.h>
#include <objtool/orc.h>
#include <objtool/warn.h>
#include <objtool/endianness.h>

#ifndef R_LARCH_SOP_PUSH_PCREL
#define R_LARCH_SOP_PUSH_PCREL 22
#endif

#ifndef R_LARCH_SOP_PUSH_DUP
#define R_LARCH_SOP_PUSH_DUP 24
#endif

#ifndef R_LARCH_SOP_PUSH_ABSOLUTE
#define R_LARCH_SOP_PUSH_ABSOLUTE 23
#endif

#ifndef R_LARCH_SOP_SR
#define R_LARCH_SOP_SR 34
#endif

#ifndef R_LARCH_SOP_SL
#define R_LARCH_SOP_SL 33
#endif

#ifndef R_LARCH_SOP_SUB
#define R_LARCH_SOP_SUB 32
#endif

#ifndef R_LARCH_SOP_POP_32_U
#define R_LARCH_SOP_POP_32_U 46
#endif

struct orc_entry arch_null = { .type = ORC_TYPE_UNDEFINED };

int arch_init_orc_entry(struct orc_entry *orc, struct cfi_state *cfi,
			  struct instruction *insn)
{
	struct cfi_reg *fp = &cfi->regs[CFI_FP];
	struct cfi_reg *ra = &cfi->regs[CFI_RA];

	memset(orc, 0, sizeof(*orc));

	if (!cfi) {
		/*
		 * This is usually either unreachable nops/traps (which don't
		 * trigger unreachable instruction warnings), or
		 * STACK_FRAME_NON_STANDARD functions.
		 */
		orc->type = ORC_TYPE_UNDEFINED;
		return 0;
	}

	switch (cfi->type) {
	case UNWIND_HINT_TYPE_UNDEFINED:
		orc->type = ORC_TYPE_UNDEFINED;
		return 0;
	case UNWIND_HINT_TYPE_END_OF_STACK:
		orc->type = ORC_TYPE_END_OF_STACK;
		return 0;
	case UNWIND_HINT_TYPE_CALL:
		orc->type = ORC_TYPE_CALL;
		break;
	case UNWIND_HINT_TYPE_REGS:
		orc->type = ORC_TYPE_REGS;
		break;
	default:
		WARN_INSN(insn, "unknown unwind hint type %d", cfi->type);
		return -1;
	}

	orc->signal = cfi->signal;

	switch (cfi->cfa.base) {
	case CFI_SP:
		orc->sp_reg = ORC_REG_SP;
		break;
	case CFI_FP:
		orc->sp_reg = ORC_REG_FP;
		break;
	default:
		WARN_INSN(insn, "unknown CFA base reg %d", cfi->cfa.base);
		return -1;
	}

	switch (fp->base) {
	case CFI_UNDEFINED:
		orc->fp_reg = ORC_REG_UNDEFINED;
		orc->fp_offset = 0;
		break;
	case CFI_CFA:
		orc->fp_reg = ORC_REG_PREV_SP;
		orc->fp_offset = fp->offset;
		break;
	default:
		WARN_INSN(insn, "unknown FP base reg %d", fp->base);
	}

	switch (ra->base) {
	case CFI_UNDEFINED:
		orc->ra_reg = ORC_REG_UNDEFINED;
		orc->ra_offset = 0;
		break;
	case CFI_CFA:
		orc->ra_reg = ORC_REG_PREV_SP;
		orc->ra_offset = ra->offset;
		break;
	default:
		WARN_INSN(insn, "unknown RA base reg %d", ra->base);
	}

	orc->sp_offset = cfi->cfa.offset;

	return 0;
}

int arch_write_orc_entry(struct elf *elf, struct section *orc_sec,
			   struct section *ip_sec, unsigned int idx,
			   struct section *insn_sec, unsigned long insn_off,
			   struct orc_entry *o)
{
	struct reloc *reloc;
	struct orc_entry *orc;

	/* populate ORC data */
	orc = (struct orc_entry *)orc_sec->data->d_buf + idx;
	memcpy(orc, o, sizeof(*orc));
	orc->sp_offset = bswap_if_needed(elf, orc->sp_offset);

	/*
	 * R_LARCH_ADD32 ip
	 * R_LARCH_SUB32 orc
	 */
	if ((reloc = elf_add_reloc_to_insn(elf, ip_sec, idx * sizeof(int),
					   R_LARCH_ADD32,
					   insn_sec, insn_off, NULL)) == NULL)
		return -1;


	if ((reloc = elf_add_reloc_to_insn(elf, ip_sec, idx * sizeof(int),
					   R_LARCH_SUB32,
					   insn_sec, idx * sizeof(int), reloc)) == NULL)
		return -1;

	return 0;
}

static const char *reg_name(unsigned int reg)
{
	switch (reg) {
	case ORC_REG_SP:
		return "sp";
	case ORC_REG_FP:
		return "fp";
	case ORC_REG_PREV_SP:
		return "prevsp";
	default:
		return "?";
	}
}

static const char *orc_type_name(unsigned int type)
{
	switch (type) {
	case ORC_TYPE_UNDEFINED:
		return "(und)";
	case ORC_TYPE_END_OF_STACK:
		return "end";
	case ORC_TYPE_CALL:
		return "call";
	case ORC_TYPE_REGS:
		return "regs";
	default:
		return "?";
	}
}

static void print_reg(unsigned int reg, int offset)
{
	if (reg == ORC_REG_UNDEFINED)
		printf(" (und) ");
	else
		printf("%s + %3d", reg_name(reg), offset);
}

void arch_print_reg(struct elf *dummy_elf, struct orc_entry orc)
{
	printf("type:%s", orc_type_name(orc.type));

	printf(" sp:");

	print_reg(orc.sp_reg, bswap_if_needed(dummy_elf, orc.sp_offset));

	printf(" fp:");

	print_reg(orc.fp_reg, bswap_if_needed(dummy_elf, orc.fp_offset));

	printf(" ra:");

	print_reg(orc.ra_reg, bswap_if_needed(dummy_elf, orc.ra_offset));

	printf(" signal:%d\n", orc.signal);
}
