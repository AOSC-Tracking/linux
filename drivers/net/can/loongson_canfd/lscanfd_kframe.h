/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LOONGSON LSCANFD controller
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __LSCANFD_KFRAME_H__
#define __LSCANFD_KFRAME_H__

#include <linux/bits.h>

/* CAN_Frame_format memory map */
enum lscanfd_can_frame_format {
	LSCANFD_META0               = 0x0,
	LSCANFD_META1               = 0x4,
	LSCANFD_DATA_1_4_W          = 0x8,
	LSCANFD_DATA_5_8_W          = 0xc,
	LSCANFD_DATA_9_12_W         = 0x10,
	LSCANFD_DATA_13_16_W        = 0x14,
	LSCANFD_DATA_17_20_W        = 0x18,
	LSCANFD_DATA_21_24_W        = 0x1c,
	LSCANFD_DATA_25_28_W        = 0x20,
	LSCANFD_DATA_29_32_W        = 0x24,
	LSCANFD_DATA_33_36_W        = 0x28,
	LSCANFD_DATA_37_40_W        = 0x2c,
	LSCANFD_DATA_41_44_W        = 0x30,
	LSCANFD_DATA_45_48_W        = 0x34,
	LSCANFD_DATA_49_52_W        = 0x38,
	LSCANFD_DATA_53_56_W        = 0x3c,
	LSCANFD_DATA_57_60_W        = 0x40,
	LSCANFD_DATA_61_64_W        = 0x44
};
/* CAN_FD_Frame_format memory region */

/*  FRAME_META0 registers */
#define REG_FRAME_FORMAT_W_ESI_RSV BIT(31)
#define REG_FRAME_FORMAT_W_IDE BIT(30)
#define REG_FRAME_FORMAT_W_RTR BIT(29)
#define REG_IDENTIFIER_W_IDENTIFIER_BASE GENMASK(28, 18)
#define REG_IDENTIFIER_W_IDENTIFIER_EXT GENMASK(17, 0)

/*  FRAME_META1 registers */
#define REG_FRAME_FORMAT_W_RWCNT GENMASK(28, 24)
#define REG_FRAME_FORMAT_W_FDF BIT(21)
#define REG_FRAME_FORMAT_W_BRS BIT(20)
#define REG_FRAME_FORMAT_W_DLC GENMASK(19, 16)
#define REG_META1_W_TIMESTAMP  GENMASK(15, 0)

/*  DATA_1_4_W registers */
#define REG_DATA_1_4_W_DATA_1 GENMASK(7, 0)
#define REG_DATA_1_4_W_DATA_2 GENMASK(15, 8)
#define REG_DATA_1_4_W_DATA_3 GENMASK(23, 16)
#define REG_DATA_1_4_W_DATA_4 GENMASK(31, 24)

/*  DATA_5_8_W registers */
#define REG_DATA_5_8_W_DATA_5 GENMASK(7, 0)
#define REG_DATA_5_8_W_DATA_6 GENMASK(15, 8)
#define REG_DATA_5_8_W_DATA_7 GENMASK(23, 16)
#define REG_DATA_5_8_W_DATA_8 GENMASK(31, 24)

/*  DATA_9_12_W registers */
#define REG_DATA_9_12_W_DATA_9 GENMASK(7, 0)
#define REG_DATA_9_12_W_DATA_10 GENMASK(15, 8)
#define REG_DATA_9_12_W_DATA_11 GENMASK(23, 16)
#define REG_DATA_9_12_W_DATA_12 GENMASK(31, 24)

/*  DATA_13_16_W registers */
#define REG_DATA_13_16_W_DATA_13 GENMASK(7, 0)
#define REG_DATA_13_16_W_DATA_14 GENMASK(15, 8)
#define REG_DATA_13_16_W_DATA_15 GENMASK(23, 16)
#define REG_DATA_13_16_W_DATA_16 GENMASK(31, 24)

/*  DATA_17_20_W registers */
#define REG_DATA_17_20_W_DATA_17 GENMASK(7, 0)
#define REG_DATA_17_20_W_DATA_18 GENMASK(15, 8)
#define REG_DATA_17_20_W_DATA_19 GENMASK(23, 16)
#define REG_DATA_17_20_W_DATA_20 GENMASK(31, 24)

/*  DATA_21_24_W registers */
#define REG_DATA_21_24_W_DATA_21 GENMASK(7, 0)
#define REG_DATA_21_24_W_DATA_22 GENMASK(15, 8)
#define REG_DATA_21_24_W_DATA_23 GENMASK(23, 16)
#define REG_DATA_21_24_W_DATA_24 GENMASK(31, 24)

/*  DATA_25_28_W registers */
#define REG_DATA_25_28_W_DATA_25 GENMASK(7, 0)
#define REG_DATA_25_28_W_DATA_26 GENMASK(15, 8)
#define REG_DATA_25_28_W_DATA_27 GENMASK(23, 16)
#define REG_DATA_25_28_W_DATA_28 GENMASK(31, 24)

/*  DATA_29_32_W registers */
#define REG_DATA_29_32_W_DATA_29 GENMASK(7, 0)
#define REG_DATA_29_32_W_DATA_30 GENMASK(15, 8)
#define REG_DATA_29_32_W_DATA_31 GENMASK(23, 16)
#define REG_DATA_29_32_W_DATA_32 GENMASK(31, 24)

/*  DATA_33_36_W registers */
#define REG_DATA_33_36_W_DATA_33 GENMASK(7, 0)
#define REG_DATA_33_36_W_DATA_34 GENMASK(15, 8)
#define REG_DATA_33_36_W_DATA_35 GENMASK(23, 16)
#define REG_DATA_33_36_W_DATA_36 GENMASK(31, 24)

/*  DATA_37_40_W registers */
#define REG_DATA_37_40_W_DATA_37 GENMASK(7, 0)
#define REG_DATA_37_40_W_DATA_38 GENMASK(15, 8)
#define REG_DATA_37_40_W_DATA_39 GENMASK(23, 16)
#define REG_DATA_37_40_W_DATA_40 GENMASK(31, 24)

/*  DATA_41_44_W registers */
#define REG_DATA_41_44_W_DATA_41 GENMASK(7, 0)
#define REG_DATA_41_44_W_DATA_42 GENMASK(15, 8)
#define REG_DATA_41_44_W_DATA_43 GENMASK(23, 16)
#define REG_DATA_41_44_W_DATA_44 GENMASK(31, 24)

/*  DATA_45_48_W registers */
#define REG_DATA_45_48_W_DATA_45 GENMASK(7, 0)
#define REG_DATA_45_48_W_DATA_46 GENMASK(15, 8)
#define REG_DATA_45_48_W_DATA_47 GENMASK(23, 16)
#define REG_DATA_45_48_W_DATA_48 GENMASK(31, 24)

/*  DATA_49_52_W registers */
#define REG_DATA_49_52_W_DATA_49 GENMASK(7, 0)
#define REG_DATA_49_52_W_DATA_50 GENMASK(15, 8)
#define REG_DATA_49_52_W_DATA_51 GENMASK(23, 16)
#define REG_DATA_49_52_W_DATA_52 GENMASK(31, 24)

/*  DATA_53_56_W registers */
#define REG_DATA_53_56_W_DATA_53 GENMASK(7, 0)
#define REG_DATA_53_56_W_DATA_56 GENMASK(15, 8)
#define REG_DATA_53_56_W_DATA_55 GENMASK(23, 16)
#define REG_DATA_53_56_W_DATA_54 GENMASK(31, 24)

/*  DATA_57_60_W registers */
#define REG_DATA_57_60_W_DATA_57 GENMASK(7, 0)
#define REG_DATA_57_60_W_DATA_58 GENMASK(15, 8)
#define REG_DATA_57_60_W_DATA_59 GENMASK(23, 16)
#define REG_DATA_57_60_W_DATA_60 GENMASK(31, 24)

/*  DATA_61_64_W registers */
#define REG_DATA_61_64_W_DATA_61 GENMASK(7, 0)
#define REG_DATA_61_64_W_DATA_62 GENMASK(15, 8)
#define REG_DATA_61_64_W_DATA_63 GENMASK(23, 16)
#define REG_DATA_61_64_W_DATA_64 GENMASK(31, 24)

/*  FRAME_TEST_W registers */
#define REG_FRAME_TEST_W_FSTC BIT(0)
#define REG_FRAME_TEST_W_FCRC BIT(1)
#define REG_FRAME_TEST_W_SDLC BIT(2)
#define REG_FRAME_TEST_W_TPRM GENMASK(12, 8)

#endif
