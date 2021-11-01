/*
 * Copyright (C) 2015 Josh Poimboeuf <jpoimboe@redhat.com>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */

#include <stdio.h>
#include <stdlib.h>

#define unlikely(cond) (cond)
#include <asm/inst.h>
#include <asm/orc_types.h>

#include <linux/objtool_types.h>
#include <objtool/check.h>
#include <objtool/elf.h>
#include <objtool/arch.h>
#include <objtool/warn.h>

#ifndef LOONGARCH_INSN_SIZE
#define LOONGARCH_INSN_SIZE sizeof(union loongarch_instruction)
#endif

#ifndef EM_LOONGARCH
#define EM_LOONGARCH	258
#endif

#define to_cfi_reg(reg) (reg)

#define signex(x, symbol_idx)					\
({								\
	unsigned long ___u64;					\
	___u64 = ((x) & (1UL << symbol_idx)) ?			\
		~((1UL << (symbol_idx + 1)) - 1) | (x) :	\
		((1UL << (symbol_idx + 1)) - 1) & (x);		\
	___u64;											\
})

const unsigned int code_nop = 0x03400000;
const unsigned int code_ret = 0x4c000020;

static int is_loongarch(const struct elf *elf)
{
	if (elf->ehdr.e_machine == EM_LOONGARCH)
		return 1;

	WARN("unexpected ELF machine type %x\n", elf->ehdr.e_machine);
	return 0;
}

bool arch_callee_saved_reg(unsigned char reg)
{
	switch (reg) {
	case CFI_S0 ... CFI_S8:
	case CFI_FP:
	case CFI_RA:
		return true;
	default:
		return false;
	}
}

unsigned long arch_dest_reloc_offset(int addend)
{
	return addend;
}

unsigned long arch_jump_destination(struct instruction *insn)
{
	return insn->offset + insn->immediate * 4;
}

bool arch_pc_relative_reloc(struct reloc *reloc)
{
	return true;
}

#define ADD_OP(op) \
	if (!(op = calloc(1, sizeof(*op)))) \
		return -1; \
	else for (*ops_list = op, ops_list = &op->next; op; op = NULL)

int arch_decode_instruction(struct objtool_file *file, const struct section *sec,
			    unsigned long offset, unsigned int maxlen,
			    struct instruction *insn)
{
	struct stack_op **ops_list = &insn->stack_ops;
	const struct elf *elf = file->elf;
	union loongarch_instruction code;
	struct stack_op *op = NULL;

	if (!is_loongarch(elf))
		return -1;

	if (maxlen < LOONGARCH_INSN_SIZE)
		return 0;

	insn->len = LOONGARCH_INSN_SIZE;
	insn->type = INSN_OTHER;
	insn->immediate = 0;

	code = *(union loongarch_instruction *)(sec->data->d_buf + offset);

	/* For some where we .fill 0 and we cannot execute it. */
	if (code.word == 0)
		insn->type = INSN_NOP;

	switch (code.reg2i12_format.opcode) {
	case addid_op:
		if ((code.reg2i12_format.rj == CFI_SP) || (code.reg2i12_format.rd == CFI_SP)) {
			/* addi.d reg1,reg2,imm */
			insn->immediate = signex(code.reg2i12_format.simmediate, 11);
			ADD_OP(op) {
				op->src.type = OP_SRC_ADD;
				op->src.reg = to_cfi_reg(code.reg2i12_format.rj);
				op->src.offset = insn->immediate;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = to_cfi_reg(code.reg2i12_format.rd);
			}
		}
		break;
	case std_op:
		if ((code.reg2i12_format.rj == CFI_SP)) {
			/* st.d reg,sp,imm */
			insn->immediate = signex(code.reg2i12_format.simmediate, 11);
			ADD_OP(op) {
				op->src.type = OP_SRC_REG;
				op->src.reg = to_cfi_reg(code.reg2i12_format.rd);
				op->dest.type = OP_DEST_REG_INDIRECT;
				op->dest.reg = CFI_SP;
				op->dest.offset = insn->immediate;
			}
		}
		break;
	case ldd_op:
		if ((code.reg2i12_format.rj == CFI_SP)) {
			/* ld.d reg,sp,imm */
			insn->immediate = signex(code.reg2i12_format.simmediate, 11);
			ADD_OP(op) {
				op->src.type = OP_SRC_REG_INDIRECT;
				op->src.reg = CFI_SP;
				op->src.offset = insn->immediate;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = to_cfi_reg(code.reg2i12_format.rd);
			}
		}
		break;
	default:
		switch (code.reg2i16_format.opcode) {
		case jirl_op:
			if (code.reg2i16_format.simmediate) {
				WARN("unexpected insn type 0x%lx\n", offset);
				return -1;
			}
			if (code.reg2i16_format.rj == CFI_RA &&
			     code.reg2i16_format.rd == 0)
				/* jr ra */
				insn->type = INSN_RETURN;
			else if (code.reg2i16_format.rd == CFI_RA)
				/* jalr reg */
				insn->type = INSN_CALL_DYNAMIC;
			else if (code.reg2i16_format.rd == 0)
				/* jr reg */
				insn->type = INSN_JUMP_DYNAMIC;
			break;
		case beq_op:
		case bne_op:
		case blt_op:
		case bge_op:
		case bltu_op:
		case bgeu_op:
			insn->immediate = signex(code.reg2i16_format.simmediate, 15);
			insn->type = INSN_JUMP_CONDITIONAL;
			break;
		case beqz_op:
		case bnez_op:
			insn->immediate = signex(code.reg1i21_format.simmediate_h << 16 |
					     code.reg1i21_format.simmediate_l, 20);
			insn->type = INSN_JUMP_CONDITIONAL;
			break;
		case bl_op:
			insn->type = INSN_CALL;
			break;
		case b_op:
			insn->type = INSN_JUMP_UNCONDITIONAL;
			break;
		default:
			if (code.reg2i14_format.opcode == stptrd_op &&
				code.reg2i14_format.rj == CFI_SP) {
				/* stptr.d reg,sp,imm */
				insn->immediate = signex(code.reg2i14_format.simmediate, 13);
				ADD_OP(op) {
					op->src.type = OP_SRC_REG;
					op->src.reg = to_cfi_reg(code.reg2i14_format.rd);
					op->dest.type = OP_DEST_REG_INDIRECT;
					op->dest.reg = CFI_SP;
					op->dest.offset = insn->immediate;
				}
			} else if (code.reg2i14_format.opcode == ldptrd_op &&
				code.reg2i14_format.rj == CFI_SP) {
				/* ldptr.d reg,sp,imm */
				insn->immediate = signex(code.reg2i14_format.simmediate, 13);
				ADD_OP(op) {
					op->src.type = OP_SRC_REG_INDIRECT;
					op->src.reg = CFI_SP;
					op->src.offset = insn->immediate;
					op->dest.type = OP_DEST_REG;
					op->dest.reg = to_cfi_reg(code.reg2i14_format.rd);
				}
			} else if (code.reg0i15_format.opcode == break_op) {
				/* break */
				insn->type = INSN_BUG;
			} else if (code.reg2_format.opcode == ertn_op) {
				/* ertn */
				insn->type = INSN_RETURN;
			} else if (code.reg2ui12_format.opcode == andi_op &&
				   code.reg2ui12_format.simmediate == 0 &&
				   code.reg2ui12_format.rj == 0 &&
				   code.reg2ui12_format.rd == 0) {
				/* nop */
				insn->type = INSN_NOP;
			}
			break;
		}
		break;
	}

	return 0;
}

void arch_initial_func_cfi_state(struct cfi_init_state *state)
{
	int i;

	for (i = 0; i < CFI_NUM_REGS; i++) {
		state->regs[i].base = CFI_UNDEFINED;
		state->regs[i].offset = 0;
	}

	/* initial CFA (call frame address) */
	state->cfa.base = CFI_SP;
	state->cfa.offset = 0;
}

int arch_decode_hint_reg(u8 sp_reg, int *base)
{
	switch (sp_reg) {
	case ORC_REG_UNDEFINED:
		*base = CFI_UNDEFINED;
		break;
	case ORC_REG_SP:
		*base = CFI_SP;
		break;
	case ORC_REG_FP:
		*base = CFI_FP;
		break;
	default:
		return -1;
	}

	return 0;
}

bool arch_has_valid_stack_frame(struct insn_state *state)
{
	return true;
}

static int update_cfi_state_regs(struct instruction *insn,
				 struct cfi_state *cfi,
				 struct stack_op *op)
{
	struct cfi_reg *cfa = &cfi->cfa;

	if (cfa->base != CFI_SP && cfa->base != CFI_SP_INDIRECT)
		return 0;

	/* addi.d sp, sp, imm */
	if (op->dest.type == OP_DEST_REG && op->src.type == OP_SRC_ADD &&
	    op->dest.reg == CFI_SP && op->src.reg == CFI_SP)
		cfa->offset -= op->src.offset;

	return 0;
}

static int update_cfi_state(struct instruction *insn,
			    struct instruction *next_insn,
			    struct cfi_state *cfi, struct stack_op *op)
{
	struct cfi_reg *cfa = &cfi->cfa;
	struct cfi_reg *regs = cfi->regs;

	if (cfa->base == CFI_UNDEFINED) {
		if (insn_func(insn)) {
			WARN_INSN(insn, "undefined stack state");
			return -1;
		}
		return 0;
	}

	if (cfi->type == UNWIND_HINT_TYPE_REGS)
		return update_cfi_state_regs(insn, cfi, op);

	switch (op->dest.type) {
	case OP_DEST_REG:
		switch (op->src.type) {
		case OP_SRC_ADD:
			if (op->dest.reg == CFI_SP && op->src.reg == CFI_SP) {
				/* addi.d sp, sp, imm */
				cfi->stack_size -= op->src.offset;
				if (cfa->base == CFI_SP)
					cfa->offset -= op->src.offset;
			} else if (op->dest.reg == CFI_FP && op->src.reg == CFI_SP) {
				/* addi.d fp, sp, imm */
				if (cfa->base == CFI_SP && cfa->offset == op->src.offset) {
					cfa->base = CFI_FP;
					cfa->offset = 0;
				}
			} else if (op->dest.reg == CFI_SP && op->src.reg == CFI_FP) {
				/* addi.d sp, fp, imm */
				if (cfa->base == CFI_FP && cfa->offset == 0) {
					cfa->base = CFI_SP;
					cfa->offset = -op->src.offset;
				}
			}
			break;
		case OP_SRC_REG_INDIRECT:
			/* ld.d _reg, sp, imm */
			if (op->src.reg == CFI_SP &&
				op->src.offset == (regs[op->dest.reg].offset + cfi->stack_size)) {
				restore_reg(cfi, op->dest.reg);
				/* Gcc may not restore sp, we adjust it directly. */
				if (cfa->base == CFI_FP && cfa->offset == 0) {
					cfa->base = CFI_SP;
					cfa->offset = cfi->stack_size;
				}
			}
			break;
		default:
			break;
		}
		break;
	case OP_DEST_REG_INDIRECT:
		if (op->src.type == OP_SRC_REG) {
			/* st.d _reg, sp, imm */
			save_reg(cfi, op->src.reg, CFI_CFA, op->dest.offset - cfi->stack_size);
		}
		break;
	default:
		WARN_INSN(insn, "unknown stack-related instruction");
		return -1;
	}

	return 0;
}

int arch_handle_insn_ops(struct instruction *insn,
			   struct instruction *next_insn,
			   struct insn_state *state)
{
	struct stack_op *op;

	for (op = insn->stack_ops; op; op = op->next) {
		int res;

		res = update_cfi_state(insn, next_insn, &state->cfi, op);
		if (res)
			return res;
	}

	return 0;
}

#ifndef R_LARCH_MARK_LA
#define R_LARCH_MARK_LA 20
#endif

void arch_try_find_call(struct list_head *p_orbit_list, struct objtool_file *file,
			struct symbol *func, struct instruction *insn)
{
	int count = 0, reg;
	struct instruction *orbit;
	struct reloc *reloc;
	union loongarch_instruction code;

	if (list_empty(p_orbit_list)) {
		WARN_FUNC("BUG: why do I have no insn track?", insn->sec, insn->offset);
		return;
	}

	if (func_last_orbit(p_orbit_list) != insn) {
		WARN_FUNC("BUG: insn is not expected.", insn->sec, insn->offset);
		return;
	}

	code = *(union loongarch_instruction *)(insn->sec->data->d_buf + insn->offset);
	if (code.reg2i16_format.opcode != jirl_op || code.reg2i16_format.rd != CFI_RA) {
		WARN_FUNC("BUG: first insn track is not expected.", insn->sec, insn->offset);
		return;
	}

	reg = code.reg2i16_format.rj;
        list_for_each_entry(orbit, p_orbit_list, orbit_node) {
		count++;
		/* jirl, la.abs (== lu12i.w, ori, lu32i.d, lu52i.d) */
		if (count == 5)
			break;
	}

	if (count != 5)
		return;

	code = *(union loongarch_instruction *)(orbit->sec->data->d_buf + orbit->offset);
	if (code.reg1i20_format.opcode != lu12iw_op || code.reg1i20_format.rd != reg)
		return;

	reloc = find_reloc_by_dest(file->elf, orbit->sec, orbit->offset);
	if (!reloc)
		return;

	if (reloc->type != R_LARCH_MARK_LA)
		return;

	insn->type = INSN_CALL;
	insn->_call_dest = reloc->sym;
}

const char *arch_nop_insn(int len)
{
	if (len != LOONGARCH_INSN_SIZE)
		WARN("invalid NOP size: %d\n", len);

	return (const char *)&code_nop;
}

const char *arch_ret_insn(int len)
{
	if (len != LOONGARCH_INSN_SIZE)
		WARN("invalid RET size: %d\n", len);

	return (const char *)&code_ret;
}
