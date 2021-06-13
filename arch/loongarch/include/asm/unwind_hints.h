/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Most of this ideas comes from x86.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_UNWIND_HINTS_H
#define _ASM_UNWIND_HINTS_H

#include <linux/objtool.h>
#include "orc_types.h"

#ifdef __ASSEMBLY__

.macro UNWIND_HINT_UNDEFINED
	UNWIND_HINT sp_reg=ORC_REG_UNDEFINED type=UNWIND_HINT_TYPE_UNDEFINED
.endm

.macro UNWIND_HINT_END_OF_STACK
	UNWIND_HINT sp_reg=ORC_REG_UNDEFINED type=UNWIND_HINT_TYPE_END_OF_STACK
.endm

.macro UNWIND_HINT_REGS base=ORC_REG_SP offset=0 signal=1
	UNWIND_HINT sp_reg=\base sp_offset=\offset type=UNWIND_HINT_TYPE_REGS
.endm

.macro UNWIND_HINT_FUNC offset=0
	UNWIND_HINT sp_reg=ORC_REG_SP sp_offset=\offset type=UNWIND_HINT_TYPE_FUNC
.endm

.macro NOT_SIBLING_CALL_HINT
876:	.pushsection .discard.not_sibling_call
	.long 876b - .
	.popsection
.endm

#else /* !__ASSEMBLY__ */

#define UNWIND_HINT_SAVE UNWIND_HINT(0, 0, UNWIND_HINT_TYPE_SAVE, 0)

#define UNWIND_HINT_RESTORE UNWIND_HINT(0, 0, UNWIND_HINT_TYPE_RESTORE, 0)

#define NOT_SIBLING_CALL_HINT					\
	"876:\n\t"						\
	".pushsection .discard.not_sibling_call\n\t"		\
	".long 876b - .\n\t"					\
	".popsection\n\t"

#endif /* __ASSEMBLY__ */

#endif /* _ASM_UNWIND_HINTS_H */
