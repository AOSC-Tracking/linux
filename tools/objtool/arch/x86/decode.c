// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2015 Josh Poimboeuf <jpoimboe@redhat.com>
 */

#include <stdio.h>
#include <stdlib.h>

#define unlikely(cond) (cond)
#include <asm/insn.h>
#include <linux/static_call_types.h>

#include "../../../arch/x86/lib/inat.c"
#include "../../../arch/x86/lib/insn.c"

#define CONFIG_64BIT 1
#include <asm/nops.h>

#include <asm/orc_types.h>
#include <linux/objtool_types.h>
#include <objtool/builtin.h>
#include <objtool/check.h>
#include <objtool/elf.h>
#include <objtool/arch.h>
#include <objtool/warn.h>
#include <objtool/endianness.h>
#include <objtool/builtin.h>
#include <arch/elf.h>

int arch_ftrace_match(char *name)
{
	return !strcmp(name, "__fentry__");
}

static int is_x86_64(const struct elf *elf)
{
	switch (elf->ehdr.e_machine) {
	case EM_X86_64:
		return 1;
	case EM_386:
		return 0;
	default:
		WARN("unexpected ELF machine type %d", elf->ehdr.e_machine);
		return -1;
	}
}

static bool check_reg_frame_pos(const struct cfi_reg *reg,
				int expected_offset)
{
	return reg->base == CFI_CFA &&
	       reg->offset == expected_offset;
}

static int update_cfi_state_regs(struct instruction *insn,
				  struct cfi_state *cfi,
				  struct stack_op *op)
{
	struct cfi_reg *cfa = &cfi->cfa;

	if (cfa->base != CFI_SP && cfa->base != CFI_SP_INDIRECT)
		return 0;

	/* push */
	if (op->dest.type == OP_DEST_PUSH || op->dest.type == OP_DEST_PUSHF)
		cfa->offset += 8;

	/* pop */
	if (op->src.type == OP_SRC_POP || op->src.type == OP_SRC_POPF)
		cfa->offset -= 8;

	/* add immediate to sp */
	if (op->dest.type == OP_DEST_REG && op->src.type == OP_SRC_ADD &&
	    op->dest.reg == CFI_SP && op->src.reg == CFI_SP)
		cfa->offset -= op->src.offset;

	return 0;
}

/*
 * A note about DRAP stack alignment:
 *
 * GCC has the concept of a DRAP register, which is used to help keep track of
 * the stack pointer when aligning the stack.  r10 or r13 is used as the DRAP
 * register.  The typical DRAP pattern is:
 *
 *   4c 8d 54 24 08		lea    0x8(%rsp),%r10
 *   48 83 e4 c0		and    $0xffffffffffffffc0,%rsp
 *   41 ff 72 f8		pushq  -0x8(%r10)
 *   55				push   %rbp
 *   48 89 e5			mov    %rsp,%rbp
 *				(more pushes)
 *   41 52			push   %r10
 *				...
 *   41 5a			pop    %r10
 *				(more pops)
 *   5d				pop    %rbp
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * There are some variations in the epilogues, like:
 *
 *   5b				pop    %rbx
 *   41 5a			pop    %r10
 *   41 5c			pop    %r12
 *   41 5d			pop    %r13
 *   41 5e			pop    %r14
 *   c9				leaveq
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * and:
 *
 *   4c 8b 55 e8		mov    -0x18(%rbp),%r10
 *   48 8b 5d e0		mov    -0x20(%rbp),%rbx
 *   4c 8b 65 f0		mov    -0x10(%rbp),%r12
 *   4c 8b 6d f8		mov    -0x8(%rbp),%r13
 *   c9				leaveq
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * Sometimes r13 is used as the DRAP register, in which case it's saved and
 * restored beforehand:
 *
 *   41 55			push   %r13
 *   4c 8d 6c 24 10		lea    0x10(%rsp),%r13
 *   48 83 e4 f0		and    $0xfffffffffffffff0,%rsp
 *				...
 *   49 8d 65 f0		lea    -0x10(%r13),%rsp
 *   41 5d			pop    %r13
 *   c3				retq
 */
static int update_cfi_state(struct instruction *insn,
			    struct instruction *next_insn,
			    struct cfi_state *cfi, struct stack_op *op)
{
	struct cfi_reg *cfa = &cfi->cfa;
	struct cfi_reg *regs = cfi->regs;

	/* stack operations don't make sense with an undefined CFA */
	if (cfa->base == CFI_UNDEFINED) {
		if (insn_func(insn)) {
			WARN_INSN(insn, "undefined stack state");
			return -1;
		}
		return 0;
	}

	if (cfi->type == UNWIND_HINT_TYPE_REGS ||
	    cfi->type == UNWIND_HINT_TYPE_REGS_PARTIAL)
		return update_cfi_state_regs(insn, cfi, op);

	switch (op->dest.type) {

	case OP_DEST_REG:
		switch (op->src.type) {

		case OP_SRC_REG:
			if (op->src.reg == CFI_SP && op->dest.reg == CFI_BP &&
			    cfa->base == CFI_SP &&
			    check_reg_frame_pos(&regs[CFI_BP], -cfa->offset)) {

				/* mov %rsp, %rbp */
				cfa->base = op->dest.reg;
				cfi->bp_scratch = false;
			}

			else if (op->src.reg == CFI_SP &&
				 op->dest.reg == CFI_BP && cfi->drap) {

				/* drap: mov %rsp, %rbp */
				regs[CFI_BP].base = CFI_BP;
				regs[CFI_BP].offset = -cfi->stack_size;
				cfi->bp_scratch = false;
			}

			else if (op->src.reg == CFI_SP && cfa->base == CFI_SP) {

				/*
				 * mov %rsp, %reg
				 *
				 * This is needed for the rare case where GCC
				 * does:
				 *
				 *   mov    %rsp, %rax
				 *   ...
				 *   mov    %rax, %rsp
				 */
				cfi->vals[op->dest.reg].base = CFI_CFA;
				cfi->vals[op->dest.reg].offset = -cfi->stack_size;
			}

			else if (op->src.reg == CFI_BP && op->dest.reg == CFI_SP &&
				 (cfa->base == CFI_BP || cfa->base == cfi->drap_reg)) {

				/*
				 * mov %rbp, %rsp
				 *
				 * Restore the original stack pointer (Clang).
				 */
				cfi->stack_size = -cfi->regs[CFI_BP].offset;
			}

			else if (op->dest.reg == cfa->base) {

				/* mov %reg, %rsp */
				if (cfa->base == CFI_SP &&
				    cfi->vals[op->src.reg].base == CFI_CFA) {

					/*
					 * This is needed for the rare case
					 * where GCC does something dumb like:
					 *
					 *   lea    0x8(%rsp), %rcx
					 *   ...
					 *   mov    %rcx, %rsp
					 */
					cfa->offset = -cfi->vals[op->src.reg].offset;
					cfi->stack_size = cfa->offset;

				} else if (cfa->base == CFI_SP &&
					   cfi->vals[op->src.reg].base == CFI_SP_INDIRECT &&
					   cfi->vals[op->src.reg].offset == cfa->offset) {

					/*
					 * Stack swizzle:
					 *
					 * 1: mov %rsp, (%[tos])
					 * 2: mov %[tos], %rsp
					 *    ...
					 * 3: pop %rsp
					 *
					 * Where:
					 *
					 * 1 - places a pointer to the previous
					 *     stack at the Top-of-Stack of the
					 *     new stack.
					 *
					 * 2 - switches to the new stack.
					 *
					 * 3 - pops the Top-of-Stack to restore
					 *     the original stack.
					 *
					 * Note: we set base to SP_INDIRECT
					 * here and preserve offset. Therefore
					 * when the unwinder reaches ToS it
					 * will dereference SP and then add the
					 * offset to find the next frame, IOW:
					 * (%rsp) + offset.
					 */
					cfa->base = CFI_SP_INDIRECT;

				} else {
					cfa->base = CFI_UNDEFINED;
					cfa->offset = 0;
				}
			}

			else if (op->dest.reg == CFI_SP &&
				 cfi->vals[op->src.reg].base == CFI_SP_INDIRECT &&
				 cfi->vals[op->src.reg].offset == cfa->offset) {

				/*
				 * The same stack swizzle case 2) as above. But
				 * because we can't change cfa->base, case 3)
				 * will become a regular POP. Pretend we're a
				 * PUSH so things don't go unbalanced.
				 */
				cfi->stack_size += 8;
			}


			break;

		case OP_SRC_ADD:
			if (op->dest.reg == CFI_SP && op->src.reg == CFI_SP) {

				/* add imm, %rsp */
				cfi->stack_size -= op->src.offset;
				if (cfa->base == CFI_SP)
					cfa->offset -= op->src.offset;
				break;
			}

			if (op->dest.reg == CFI_SP && op->src.reg == CFI_BP) {

				/* lea disp(%rbp), %rsp */
				cfi->stack_size = -(op->src.offset + regs[CFI_BP].offset);
				break;
			}

			if (op->src.reg == CFI_SP && cfa->base == CFI_SP) {

				/* drap: lea disp(%rsp), %drap */
				cfi->drap_reg = op->dest.reg;

				/*
				 * lea disp(%rsp), %reg
				 *
				 * This is needed for the rare case where GCC
				 * does something dumb like:
				 *
				 *   lea    0x8(%rsp), %rcx
				 *   ...
				 *   mov    %rcx, %rsp
				 */
				cfi->vals[op->dest.reg].base = CFI_CFA;
				cfi->vals[op->dest.reg].offset = \
					-cfi->stack_size + op->src.offset;

				break;
			}

			if (cfi->drap && op->dest.reg == CFI_SP &&
			    op->src.reg == cfi->drap_reg) {

				 /* drap: lea disp(%drap), %rsp */
				cfa->base = CFI_SP;
				cfa->offset = cfi->stack_size = -op->src.offset;
				cfi->drap_reg = CFI_UNDEFINED;
				cfi->drap = false;
				break;
			}

			if (op->dest.reg == cfi->cfa.base && !(next_insn && next_insn->hint)) {
				WARN_INSN(insn, "unsupported stack register modification");
				return -1;
			}

			break;

		case OP_SRC_AND:
			if (op->dest.reg != CFI_SP ||
			    (cfi->drap_reg != CFI_UNDEFINED && cfa->base != CFI_SP) ||
			    (cfi->drap_reg == CFI_UNDEFINED && cfa->base != CFI_BP)) {
				WARN_INSN(insn, "unsupported stack pointer realignment");
				return -1;
			}

			if (cfi->drap_reg != CFI_UNDEFINED) {
				/* drap: and imm, %rsp */
				cfa->base = cfi->drap_reg;
				cfa->offset = cfi->stack_size = 0;
				cfi->drap = true;
			}

			/*
			 * Older versions of GCC (4.8ish) realign the stack
			 * without DRAP, with a frame pointer.
			 */

			break;

		case OP_SRC_POP:
		case OP_SRC_POPF:
			if (op->dest.reg == CFI_SP && cfa->base == CFI_SP_INDIRECT) {

				/* pop %rsp; # restore from a stack swizzle */
				cfa->base = CFI_SP;
				break;
			}

			if (!cfi->drap && op->dest.reg == cfa->base) {

				/* pop %rbp */
				cfa->base = CFI_SP;
			}

			if (cfi->drap && cfa->base == CFI_BP_INDIRECT &&
			    op->dest.reg == cfi->drap_reg &&
			    cfi->drap_offset == -cfi->stack_size) {

				/* drap: pop %drap */
				cfa->base = cfi->drap_reg;
				cfa->offset = 0;
				cfi->drap_offset = -1;

			} else if (cfi->stack_size == -regs[op->dest.reg].offset) {

				/* pop %reg */
				restore_reg(cfi, op->dest.reg);
			}

			cfi->stack_size -= 8;
			if (cfa->base == CFI_SP)
				cfa->offset -= 8;

			break;

		case OP_SRC_REG_INDIRECT:
			if (!cfi->drap && op->dest.reg == cfa->base &&
			    op->dest.reg == CFI_BP) {

				/* mov disp(%rsp), %rbp */
				cfa->base = CFI_SP;
				cfa->offset = cfi->stack_size;
			}

			if (cfi->drap && op->src.reg == CFI_BP &&
			    op->src.offset == cfi->drap_offset) {

				/* drap: mov disp(%rbp), %drap */
				cfa->base = cfi->drap_reg;
				cfa->offset = 0;
				cfi->drap_offset = -1;
			}

			if (cfi->drap && op->src.reg == CFI_BP &&
			    op->src.offset == regs[op->dest.reg].offset) {

				/* drap: mov disp(%rbp), %reg */
				restore_reg(cfi, op->dest.reg);

			} else if (op->src.reg == cfa->base &&
			    op->src.offset == regs[op->dest.reg].offset + cfa->offset) {

				/* mov disp(%rbp), %reg */
				/* mov disp(%rsp), %reg */
				restore_reg(cfi, op->dest.reg);

			} else if (op->src.reg == CFI_SP &&
				   op->src.offset == regs[op->dest.reg].offset + cfi->stack_size) {

				/* mov disp(%rsp), %reg */
				restore_reg(cfi, op->dest.reg);
			}

			break;

		default:
			WARN_INSN(insn, "unknown stack-related instruction");
			return -1;
		}

		break;

	case OP_DEST_PUSH:
	case OP_DEST_PUSHF:
		cfi->stack_size += 8;
		if (cfa->base == CFI_SP)
			cfa->offset += 8;

		if (op->src.type != OP_SRC_REG)
			break;

		if (cfi->drap) {
			if (op->src.reg == cfa->base && op->src.reg == cfi->drap_reg) {

				/* drap: push %drap */
				cfa->base = CFI_BP_INDIRECT;
				cfa->offset = -cfi->stack_size;

				/* save drap so we know when to restore it */
				cfi->drap_offset = -cfi->stack_size;

			} else if (op->src.reg == CFI_BP && cfa->base == cfi->drap_reg) {

				/* drap: push %rbp */
				cfi->stack_size = 0;

			} else {

				/* drap: push %reg */
				save_reg(cfi, op->src.reg, CFI_BP, -cfi->stack_size);
			}

		} else {

			/* push %reg */
			save_reg(cfi, op->src.reg, CFI_CFA, -cfi->stack_size);
		}

		/* detect when asm code uses rbp as a scratch register */
		if (opts.stackval && insn_func(insn) && op->src.reg == CFI_BP &&
		    cfa->base != CFI_BP)
			cfi->bp_scratch = true;
		break;

	case OP_DEST_REG_INDIRECT:

		if (cfi->drap) {
			if (op->src.reg == cfa->base && op->src.reg == cfi->drap_reg) {

				/* drap: mov %drap, disp(%rbp) */
				cfa->base = CFI_BP_INDIRECT;
				cfa->offset = op->dest.offset;

				/* save drap offset so we know when to restore it */
				cfi->drap_offset = op->dest.offset;
			} else {

				/* drap: mov reg, disp(%rbp) */
				save_reg(cfi, op->src.reg, CFI_BP, op->dest.offset);
			}

		} else if (op->dest.reg == cfa->base) {

			/* mov reg, disp(%rbp) */
			/* mov reg, disp(%rsp) */
			save_reg(cfi, op->src.reg, CFI_CFA,
				 op->dest.offset - cfi->cfa.offset);

		} else if (op->dest.reg == CFI_SP) {

			/* mov reg, disp(%rsp) */
			save_reg(cfi, op->src.reg, CFI_CFA,
				 op->dest.offset - cfi->stack_size);

		} else if (op->src.reg == CFI_SP && op->dest.offset == 0) {

			/* mov %rsp, (%reg); # setup a stack swizzle. */
			cfi->vals[op->dest.reg].base = CFI_SP_INDIRECT;
			cfi->vals[op->dest.reg].offset = cfa->offset;
		}

		break;

	case OP_DEST_MEM:
		if (op->src.type != OP_SRC_POP && op->src.type != OP_SRC_POPF) {
			WARN_INSN(insn, "unknown stack-related memory operation");
			return -1;
		}

		/* pop mem */
		cfi->stack_size -= 8;
		if (cfa->base == CFI_SP)
			cfa->offset -= 8;

		break;

	default:
		WARN_INSN(insn, "unknown stack-related instruction");
		return -1;
	}

	return 0;
}

bool arch_callee_saved_reg(unsigned char reg)
{
	switch (reg) {
	case CFI_BP:
	case CFI_BX:
	case CFI_R12:
	case CFI_R13:
	case CFI_R14:
	case CFI_R15:
		return true;

	case CFI_AX:
	case CFI_CX:
	case CFI_DX:
	case CFI_SI:
	case CFI_DI:
	case CFI_SP:
	case CFI_R8:
	case CFI_R9:
	case CFI_R10:
	case CFI_R11:
	case CFI_RA:
	default:
		return false;
	}
}

unsigned long arch_dest_reloc_offset(int addend)
{
	return addend + 4;
}

unsigned long arch_jump_destination(struct instruction *insn)
{
	return insn->offset + insn->len + insn->immediate;
}

bool arch_pc_relative_reloc(struct reloc *reloc)
{
	/*
	 * All relocation types where P (the address of the target)
	 * is included in the computation.
	 */
	switch (reloc->type) {
	case R_X86_64_PC8:
	case R_X86_64_PC16:
	case R_X86_64_PC32:
	case R_X86_64_PC64:

	case R_X86_64_PLT32:
	case R_X86_64_GOTPC32:
	case R_X86_64_GOTPCREL:
		return true;

	default:
		break;
	}

	return false;
}

#define ADD_OP(op) \
	if (!(op = calloc(1, sizeof(*op)))) \
		return -1; \
	else for (*ops_list = op, ops_list = &op->next; op; op = NULL)

/*
 * Helpers to decode ModRM/SIB:
 *
 * r/m| AX  CX  DX  BX |  SP |  BP |  SI  DI |
 *    | R8  R9 R10 R11 | R12 | R13 | R14 R15 |
 * Mod+----------------+-----+-----+---------+
 * 00 |    [r/m]       |[SIB]|[IP+]|  [r/m]  |
 * 01 |  [r/m + d8]    |[S+d]|   [r/m + d8]  |
 * 10 |  [r/m + d32]   |[S+D]|   [r/m + d32] |
 * 11 |                   r/ m               |
 */

#define mod_is_mem()	(modrm_mod != 3)
#define mod_is_reg()	(modrm_mod == 3)

#define is_RIP()   ((modrm_rm & 7) == CFI_BP && modrm_mod == 0)
#define have_SIB() ((modrm_rm & 7) == CFI_SP && mod_is_mem())

#define rm_is(reg) (have_SIB() ? \
		    sib_base == (reg) && sib_index == CFI_SP : \
		    modrm_rm == (reg))

#define rm_is_mem(reg)	(mod_is_mem() && !is_RIP() && rm_is(reg))
#define rm_is_reg(reg)	(mod_is_reg() && modrm_rm == (reg))

static bool has_notrack_prefix(struct insn *insn)
{
	int i;

	for (i = 0; i < insn->prefixes.nbytes; i++) {
		if (insn->prefixes.bytes[i] == 0x3e)
			return true;
	}

	return false;
}

int arch_decode_instruction(struct objtool_file *file, const struct section *sec,
			    unsigned long offset, unsigned int maxlen,
			    struct instruction *insn)
{
	struct stack_op **ops_list = &insn->stack_ops;
	const struct elf *elf = file->elf;
	struct insn ins;
	int x86_64, ret;
	unsigned char op1, op2, op3, prefix,
		      rex = 0, rex_b = 0, rex_r = 0, rex_w = 0, rex_x = 0,
		      modrm = 0, modrm_mod = 0, modrm_rm = 0, modrm_reg = 0,
		      sib = 0, /* sib_scale = 0, */ sib_index = 0, sib_base = 0;
	struct stack_op *op = NULL;
	struct symbol *sym;
	u64 imm;

	x86_64 = is_x86_64(elf);
	if (x86_64 == -1)
		return -1;

	ret = insn_decode(&ins, sec->data->d_buf + offset, maxlen,
			  x86_64 ? INSN_MODE_64 : INSN_MODE_32);
	if (ret < 0) {
		WARN("can't decode instruction at %s:0x%lx", sec->name, offset);
		return -1;
	}

	insn->len = ins.length;
	insn->type = INSN_OTHER;

	if (ins.vex_prefix.nbytes)
		return 0;

	prefix = ins.prefixes.bytes[0];

	op1 = ins.opcode.bytes[0];
	op2 = ins.opcode.bytes[1];
	op3 = ins.opcode.bytes[2];

	if (ins.rex_prefix.nbytes) {
		rex = ins.rex_prefix.bytes[0];
		rex_w = X86_REX_W(rex) >> 3;
		rex_r = X86_REX_R(rex) >> 2;
		rex_x = X86_REX_X(rex) >> 1;
		rex_b = X86_REX_B(rex);
	}

	if (ins.modrm.nbytes) {
		modrm = ins.modrm.bytes[0];
		modrm_mod = X86_MODRM_MOD(modrm);
		modrm_reg = X86_MODRM_REG(modrm) + 8*rex_r;
		modrm_rm  = X86_MODRM_RM(modrm)  + 8*rex_b;
	}

	if (ins.sib.nbytes) {
		sib = ins.sib.bytes[0];
		/* sib_scale = X86_SIB_SCALE(sib); */
		sib_index = X86_SIB_INDEX(sib) + 8*rex_x;
		sib_base  = X86_SIB_BASE(sib)  + 8*rex_b;
	}

	switch (op1) {

	case 0x1:
	case 0x29:
		if (rex_w && rm_is_reg(CFI_SP)) {

			/* add/sub reg, %rsp */
			ADD_OP(op) {
				op->src.type = OP_SRC_ADD;
				op->src.reg = modrm_reg;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = CFI_SP;
			}
		}
		break;

	case 0x50 ... 0x57:

		/* push reg */
		ADD_OP(op) {
			op->src.type = OP_SRC_REG;
			op->src.reg = (op1 & 0x7) + 8*rex_b;
			op->dest.type = OP_DEST_PUSH;
		}

		break;

	case 0x58 ... 0x5f:

		/* pop reg */
		ADD_OP(op) {
			op->src.type = OP_SRC_POP;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = (op1 & 0x7) + 8*rex_b;
		}

		break;

	case 0x68:
	case 0x6a:
		/* push immediate */
		ADD_OP(op) {
			op->src.type = OP_SRC_CONST;
			op->dest.type = OP_DEST_PUSH;
		}
		break;

	case 0x70 ... 0x7f:
		insn->type = INSN_JUMP_CONDITIONAL;
		break;

	case 0x80 ... 0x83:
		/*
		 * 1000 00sw : mod OP r/m : immediate
		 *
		 * s - sign extend immediate
		 * w - imm8 / imm32
		 *
		 * OP: 000 ADD    100 AND
		 *     001 OR     101 SUB
		 *     010 ADC    110 XOR
		 *     011 SBB    111 CMP
		 */

		/* 64bit only */
		if (!rex_w)
			break;

		/* %rsp target only */
		if (!rm_is_reg(CFI_SP))
			break;

		imm = ins.immediate.value;
		if (op1 & 2) { /* sign extend */
			if (op1 & 1) { /* imm32 */
				imm <<= 32;
				imm = (s64)imm >> 32;
			} else { /* imm8 */
				imm <<= 56;
				imm = (s64)imm >> 56;
			}
		}

		switch (modrm_reg & 7) {
		case 5:
			imm = -imm;
			/* fallthrough */
		case 0:
			/* add/sub imm, %rsp */
			ADD_OP(op) {
				op->src.type = OP_SRC_ADD;
				op->src.reg = CFI_SP;
				op->src.offset = imm;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = CFI_SP;
			}
			break;

		case 4:
			/* and imm, %rsp */
			ADD_OP(op) {
				op->src.type = OP_SRC_AND;
				op->src.reg = CFI_SP;
				op->src.offset = ins.immediate.value;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = CFI_SP;
			}
			break;

		default:
			/* WARN ? */
			break;
		}

		break;

	case 0x89:
		if (!rex_w)
			break;

		if (modrm_reg == CFI_SP) {

			if (mod_is_reg()) {
				/* mov %rsp, reg */
				ADD_OP(op) {
					op->src.type = OP_SRC_REG;
					op->src.reg = CFI_SP;
					op->dest.type = OP_DEST_REG;
					op->dest.reg = modrm_rm;
				}
				break;

			} else {
				/* skip RIP relative displacement */
				if (is_RIP())
					break;

				/* skip nontrivial SIB */
				if (have_SIB()) {
					modrm_rm = sib_base;
					if (sib_index != CFI_SP)
						break;
				}

				/* mov %rsp, disp(%reg) */
				ADD_OP(op) {
					op->src.type = OP_SRC_REG;
					op->src.reg = CFI_SP;
					op->dest.type = OP_DEST_REG_INDIRECT;
					op->dest.reg = modrm_rm;
					op->dest.offset = ins.displacement.value;
				}
				break;
			}

			break;
		}

		if (rm_is_reg(CFI_SP)) {

			/* mov reg, %rsp */
			ADD_OP(op) {
				op->src.type = OP_SRC_REG;
				op->src.reg = modrm_reg;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = CFI_SP;
			}
			break;
		}

		/* fallthrough */
	case 0x88:
		if (!rex_w)
			break;

		if (rm_is_mem(CFI_BP)) {

			/* mov reg, disp(%rbp) */
			ADD_OP(op) {
				op->src.type = OP_SRC_REG;
				op->src.reg = modrm_reg;
				op->dest.type = OP_DEST_REG_INDIRECT;
				op->dest.reg = CFI_BP;
				op->dest.offset = ins.displacement.value;
			}
			break;
		}

		if (rm_is_mem(CFI_SP)) {

			/* mov reg, disp(%rsp) */
			ADD_OP(op) {
				op->src.type = OP_SRC_REG;
				op->src.reg = modrm_reg;
				op->dest.type = OP_DEST_REG_INDIRECT;
				op->dest.reg = CFI_SP;
				op->dest.offset = ins.displacement.value;
			}
			break;
		}

		break;

	case 0x8b:
		if (!rex_w)
			break;

		if (rm_is_mem(CFI_BP)) {

			/* mov disp(%rbp), reg */
			ADD_OP(op) {
				op->src.type = OP_SRC_REG_INDIRECT;
				op->src.reg = CFI_BP;
				op->src.offset = ins.displacement.value;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = modrm_reg;
			}
			break;
		}

		if (rm_is_mem(CFI_SP)) {

			/* mov disp(%rsp), reg */
			ADD_OP(op) {
				op->src.type = OP_SRC_REG_INDIRECT;
				op->src.reg = CFI_SP;
				op->src.offset = ins.displacement.value;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = modrm_reg;
			}
			break;
		}

		break;

	case 0x8d:
		if (mod_is_reg()) {
			WARN("invalid LEA encoding at %s:0x%lx", sec->name, offset);
			break;
		}

		/* skip non 64bit ops */
		if (!rex_w)
			break;

		/* skip RIP relative displacement */
		if (is_RIP())
			break;

		/* skip nontrivial SIB */
		if (have_SIB()) {
			modrm_rm = sib_base;
			if (sib_index != CFI_SP)
				break;
		}

		/* lea disp(%src), %dst */
		ADD_OP(op) {
			op->src.offset = ins.displacement.value;
			if (!op->src.offset) {
				/* lea (%src), %dst */
				op->src.type = OP_SRC_REG;
			} else {
				/* lea disp(%src), %dst */
				op->src.type = OP_SRC_ADD;
			}
			op->src.reg = modrm_rm;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = modrm_reg;
		}
		break;

	case 0x8f:
		/* pop to mem */
		ADD_OP(op) {
			op->src.type = OP_SRC_POP;
			op->dest.type = OP_DEST_MEM;
		}
		break;

	case 0x90:
		insn->type = INSN_NOP;
		break;

	case 0x9c:
		/* pushf */
		ADD_OP(op) {
			op->src.type = OP_SRC_CONST;
			op->dest.type = OP_DEST_PUSHF;
		}
		break;

	case 0x9d:
		/* popf */
		ADD_OP(op) {
			op->src.type = OP_SRC_POPF;
			op->dest.type = OP_DEST_MEM;
		}
		break;

	case 0x0f:

		if (op2 == 0x01) {

			if (modrm == 0xca)
				insn->type = INSN_CLAC;
			else if (modrm == 0xcb)
				insn->type = INSN_STAC;

		} else if (op2 >= 0x80 && op2 <= 0x8f) {

			insn->type = INSN_JUMP_CONDITIONAL;

		} else if (op2 == 0x05 || op2 == 0x07 || op2 == 0x34 ||
			   op2 == 0x35) {

			/* sysenter, sysret */
			insn->type = INSN_CONTEXT_SWITCH;

		} else if (op2 == 0x0b || op2 == 0xb9) {

			/* ud2 */
			insn->type = INSN_BUG;

		} else if (op2 == 0x0d || op2 == 0x1f) {

			/* nopl/nopw */
			insn->type = INSN_NOP;

		} else if (op2 == 0x1e) {

			if (prefix == 0xf3 && (modrm == 0xfa || modrm == 0xfb))
				insn->type = INSN_ENDBR;


		} else if (op2 == 0x38 && op3 == 0xf8) {
			if (ins.prefixes.nbytes == 1 &&
			    ins.prefixes.bytes[0] == 0xf2) {
				/* ENQCMD cannot be used in the kernel. */
				WARN("ENQCMD instruction at %s:%lx", sec->name,
				     offset);
			}

		} else if (op2 == 0xa0 || op2 == 0xa8) {

			/* push fs/gs */
			ADD_OP(op) {
				op->src.type = OP_SRC_CONST;
				op->dest.type = OP_DEST_PUSH;
			}

		} else if (op2 == 0xa1 || op2 == 0xa9) {

			/* pop fs/gs */
			ADD_OP(op) {
				op->src.type = OP_SRC_POP;
				op->dest.type = OP_DEST_MEM;
			}
		}

		break;

	case 0xc9:
		/*
		 * leave
		 *
		 * equivalent to:
		 * mov bp, sp
		 * pop bp
		 */
		ADD_OP(op) {
			op->src.type = OP_SRC_REG;
			op->src.reg = CFI_BP;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;
		}
		ADD_OP(op) {
			op->src.type = OP_SRC_POP;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_BP;
		}
		break;

	case 0xcc:
		/* int3 */
		insn->type = INSN_TRAP;
		break;

	case 0xe3:
		/* jecxz/jrcxz */
		insn->type = INSN_JUMP_CONDITIONAL;
		break;

	case 0xe9:
	case 0xeb:
		insn->type = INSN_JUMP_UNCONDITIONAL;
		break;

	case 0xc2:
	case 0xc3:
		insn->type = INSN_RETURN;
		break;

	case 0xc7: /* mov imm, r/m */
		if (!opts.noinstr)
			break;

		if (ins.length == 3+4+4 && !strncmp(sec->name, ".init.text", 10)) {
			struct reloc *immr, *disp;
			struct symbol *func;
			int idx;

			immr = find_reloc_by_dest(elf, (void *)sec, offset+3);
			disp = find_reloc_by_dest(elf, (void *)sec, offset+7);

			if (!immr || strcmp(immr->sym->name, "pv_ops"))
				break;

			idx = (immr->addend + 8) / sizeof(void *);

			func = disp->sym;
			if (disp->sym->type == STT_SECTION)
				func = find_symbol_by_offset(disp->sym->sec, disp->addend);
			if (!func) {
				WARN("no func for pv_ops[]");
				return -1;
			}

			objtool_pv_add(file, idx, func);
		}

		break;

	case 0xcf: /* iret */
		/*
		 * Handle sync_core(), which has an IRET to self.
		 * All other IRET are in STT_NONE entry code.
		 */
		sym = find_symbol_containing(sec, offset);
		if (sym && sym->type == STT_FUNC) {
			ADD_OP(op) {
				/* add $40, %rsp */
				op->src.type = OP_SRC_ADD;
				op->src.reg = CFI_SP;
				op->src.offset = 5*8;
				op->dest.type = OP_DEST_REG;
				op->dest.reg = CFI_SP;
			}
			break;
		}

		/* fallthrough */

	case 0xca: /* retf */
	case 0xcb: /* retf */
		insn->type = INSN_CONTEXT_SWITCH;
		break;

	case 0xe0: /* loopne */
	case 0xe1: /* loope */
	case 0xe2: /* loop */
		insn->type = INSN_JUMP_CONDITIONAL;
		break;

	case 0xe8:
		insn->type = INSN_CALL;
		/*
		 * For the impact on the stack, a CALL behaves like
		 * a PUSH of an immediate value (the return address).
		 */
		ADD_OP(op) {
			op->src.type = OP_SRC_CONST;
			op->dest.type = OP_DEST_PUSH;
		}
		break;

	case 0xfc:
		insn->type = INSN_CLD;
		break;

	case 0xfd:
		insn->type = INSN_STD;
		break;

	case 0xff:
		if (modrm_reg == 2 || modrm_reg == 3) {

			insn->type = INSN_CALL_DYNAMIC;
			if (has_notrack_prefix(&ins))
				WARN("notrack prefix found at %s:0x%lx", sec->name, offset);

		} else if (modrm_reg == 4) {

			insn->type = INSN_JUMP_DYNAMIC;
			if (has_notrack_prefix(&ins))
				WARN("notrack prefix found at %s:0x%lx", sec->name, offset);

		} else if (modrm_reg == 5) {

			/* jmpf */
			insn->type = INSN_CONTEXT_SWITCH;

		} else if (modrm_reg == 6) {

			/* push from mem */
			ADD_OP(op) {
				op->src.type = OP_SRC_CONST;
				op->dest.type = OP_DEST_PUSH;
			}
		}

		break;

	default:
		break;
	}

	insn->immediate = ins.immediate.nbytes ? ins.immediate.value : 0;

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
	state->cfa.offset = 8;

	/* initial RA (return address) */
	state->regs[CFI_RA].base = CFI_CFA;
	state->regs[CFI_RA].offset = -8;
}

bool arch_has_valid_stack_frame(struct insn_state *state)
{
	struct cfi_state *cfi = &state->cfi;

	if (cfi->cfa.base == CFI_BP &&
	    check_reg_frame_pos(&cfi->regs[CFI_BP], -cfi->cfa.offset) &&
	    check_reg_frame_pos(&cfi->regs[CFI_RA], -cfi->cfa.offset + 8))
		return true;

	if (cfi->drap && cfi->regs[CFI_BP].base == CFI_BP)
		return true;

	return false;
}

int arch_handle_insn_ops(struct instruction *insn,
			   struct instruction *next_insn,
			   struct insn_state *state)
{
	struct stack_op *op;

	for (op = insn->stack_ops; op; op = op->next) {

		if (update_cfi_state(insn, next_insn, &state->cfi, op))
			return 1;

		if (!insn->alt_group)
			continue;

		if (op->dest.type == OP_DEST_PUSHF) {
			if (!state->uaccess_stack) {
				state->uaccess_stack = 1;
			} else if (state->uaccess_stack >> 31) {
				WARN_FUNC("PUSHF stack exhausted",
					  insn->sec, insn->offset);
				return 1;
			}
			state->uaccess_stack <<= 1;
			state->uaccess_stack  |= state->uaccess;
		}

		if (op->src.type == OP_SRC_POPF) {
			if (state->uaccess_stack) {
				state->uaccess = state->uaccess_stack & 1;
				state->uaccess_stack >>= 1;
				if (state->uaccess_stack == 1)
					state->uaccess_stack = 0;
			}
		}
	}

	return 0;
}

int arch_create_static_call_sections(struct objtool_file *file)
{
	struct section *sec;
	struct static_call_site *site;
	struct instruction *insn;
	struct symbol *key_sym;
	char *key_name, *tmp;
	int idx;

	sec = find_section_by_name(file->elf, ".static_call_sites");
	if (sec) {
		INIT_LIST_HEAD(&file->static_call_list);
		WARN("file already has .static_call_sites section, skipping");
		return 0;
	}

	if (list_empty(&file->static_call_list))
		return 0;

	idx = 0;
	list_for_each_entry(insn, &file->static_call_list, call_node)
		idx++;

	sec = elf_create_section(file->elf, ".static_call_sites", SHF_WRITE,
				 sizeof(struct static_call_site), idx);
	if (!sec)
		return -1;

	idx = 0;
	list_for_each_entry(insn, &file->static_call_list, call_node) {

		site = (struct static_call_site *)sec->data->d_buf + idx;
		memset(site, 0, sizeof(struct static_call_site));

		/* populate reloc for 'addr' */
		if (!elf_add_reloc_to_insn(file->elf, sec,
					   idx * sizeof(struct static_call_site),
					   R_X86_64_PC32,
					   insn->sec, insn->offset, NULL))
			return -1;

		/* find key symbol */
		key_name = strdup(insn->_call_dest->name);
		if (!key_name) {
			perror("strdup");
			return -1;
		}
		if (strncmp(key_name, STATIC_CALL_TRAMP_PREFIX_STR,
			    STATIC_CALL_TRAMP_PREFIX_LEN)) {
			WARN("static_call: trampoline name malformed: %s", key_name);
			free(key_name);
			return -1;
		}
		tmp = key_name + STATIC_CALL_TRAMP_PREFIX_LEN - STATIC_CALL_KEY_PREFIX_LEN;
		memcpy(tmp, STATIC_CALL_KEY_PREFIX_STR, STATIC_CALL_KEY_PREFIX_LEN);

		key_sym = find_symbol_by_name(file->elf, tmp);
		if (!key_sym) {
			if (!opts.module) {
				WARN("static_call: can't find static_call_key symbol: %s", tmp);
				free(key_name);
				return -1;
			}

			/*
			 * For modules(), the key might not be exported, which
			 * means the module can make static calls but isn't
			 * allowed to change them.
			 *
			 * In that case we temporarily set the key to be the
			 * trampoline address.  This is fixed up in
			 * static_call_add_module().
			 */
			key_sym = insn->_call_dest;
		}
		free(key_name);

		/* populate reloc for 'key' */
		if (!elf_add_reloc(file->elf, sec,
				   idx * sizeof(struct static_call_site) + 4,
				   R_X86_64_PC32, key_sym,
				   is_sibling_call(insn) * STATIC_CALL_SITE_TAIL, NULL))
			return -1;

		idx++;
	}

	return 0;
}

/*
 * Return true if name matches an instrumentation function, where calls to that
 * function from noinstr code can safely be removed, but compilers won't do so.
 */
static bool is_profiling_func(const char *name)
{
	/*
	 * Many compilers cannot disable KCOV with a function attribute.
	 */
	if (!strncmp(name, "__sanitizer_cov_", 16))
		return true;

	/*
	 * Some compilers currently do not remove __tsan_func_entry/exit nor
	 * __tsan_atomic_signal_fence (used for barrier instrumentation) with
	 * the __no_sanitize_thread attribute, remove them. Once the kernel's
	 * minimum Clang version is 14.0, this can be removed.
	 */
	if (!strncmp(name, "__tsan_func_", 12) ||
	    !strcmp(name, "__tsan_atomic_signal_fence"))
		return true;

	return false;
}

int arch_classify_symbols(struct objtool_file *file)
{
	struct symbol *func;

	for_each_sym(file, func) {
		if (func->bind != STB_GLOBAL)
			continue;

		if (!strncmp(func->name, STATIC_CALL_TRAMP_PREFIX_STR,
			     strlen(STATIC_CALL_TRAMP_PREFIX_STR)))
			func->static_call_tramp = true;

		if (arch_is_retpoline(func))
			func->retpoline_thunk = true;

		if (arch_is_rethunk(func))
			func->return_thunk = true;

		if (arch_ftrace_match(func->name))
			func->fentry = true;

		if (is_profiling_func(func->name))
			func->profiling_func = true;
	}

	return 0;
}

const char *arch_nop_insn(int len)
{
	static const char nops[5][5] = {
		{ BYTES_NOP1 },
		{ BYTES_NOP2 },
		{ BYTES_NOP3 },
		{ BYTES_NOP4 },
		{ BYTES_NOP5 },
	};

	if (len < 1 || len > 5) {
		WARN("invalid NOP size: %d\n", len);
		return NULL;
	}

	return nops[len-1];
}

#define BYTE_RET	0xC3

const char *arch_ret_insn(int len)
{
	static const char ret[5][5] = {
		{ BYTE_RET },
		{ BYTE_RET, 0xcc },
		{ BYTE_RET, 0xcc, BYTES_NOP1 },
		{ BYTE_RET, 0xcc, BYTES_NOP2 },
		{ BYTE_RET, 0xcc, BYTES_NOP3 },
	};

	if (len < 1 || len > 5) {
		WARN("invalid RET size: %d\n", len);
		return NULL;
	}

	return ret[len-1];
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
	case ORC_REG_BP:
		*base = CFI_BP;
		break;
	case ORC_REG_SP_INDIRECT:
		*base = CFI_SP_INDIRECT;
		break;
	case ORC_REG_R10:
		*base = CFI_R10;
		break;
	case ORC_REG_R13:
		*base = CFI_R13;
		break;
	case ORC_REG_DI:
		*base = CFI_DI;
		break;
	case ORC_REG_DX:
		*base = CFI_DX;
		break;
	default:
		return -1;
	}

	return 0;
}

bool arch_is_retpoline(struct symbol *sym)
{
	return !strncmp(sym->name, "__x86_indirect_", 15);
}

bool arch_is_rethunk(struct symbol *sym)
{
	return !strcmp(sym->name, "__x86_return_thunk") ||
	       !strcmp(sym->name, "srso_untrain_ret") ||
	       !strcmp(sym->name, "srso_safe_ret") ||
	       !strcmp(sym->name, "__ret");
}

void arch_try_find_call(struct list_head *p_orbit_list, struct objtool_file *file,
			struct symbol *func, struct instruction *insn)
{
}
