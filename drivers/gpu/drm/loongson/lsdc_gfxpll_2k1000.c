// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Loongson Technology Corporation Limited
 */

#include <linux/delay.h>

#include <drm/drm_file.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "lsdc_drv.h"

union ls2k1000_ddr_pll_reg_bitmap {
	struct ls2k1000_ddr_pll_bitmap {
		/* Byte 0 ~ Byte 3 */
		unsigned bypass       :  1;  /*  0: Bypass ddr pll entirely */
		unsigned oe_gpu       :  1;  /*  1: GPU output clk enable   */
		unsigned write_en     :  1;  /*  2: Allow software update   */
		unsigned _reserved_1_ : 23;  /*  3 : 25  Don't care         */
		unsigned div_ref      :  6;  /* 31 : 26  Input clk divider  */
		/* Byte 4 ~ Byte 7 */
		unsigned loopc        : 10;   /* 41 : 32 Clock multiplier   */
		unsigned _reserved_2_ : 22;   /* 42 : 63 Useless            */

		/* Byte 8 ~ Byte 15 */
		unsigned div_out_ddr  :  6;  /*  5 : 0  DDR output divider */
		unsigned _reserved_3_ : 16;  /* 21 : 6  */
		unsigned div_out_gpu  :  6;  /* 27 : 22 GPU output divider */
		unsigned _reserved_4_ : 16;  /* 43 : 28 */
		unsigned div_out_hda  :  7;  /* 50 : 44 HDA output divider */
		unsigned _reserved_5_ : 13;  /* 63 : 51 */
	} bitmap;
	u32 w[4];
	u64 d[2];
};

union ls2k1000_dc_pll_reg_bitmap {
	struct ls2k1000_dc_pll_bitmap {
		/* Byte 0 ~ Byte 3 */
		unsigned _reserved_1_ : 26;  /*  0 : 25  Don't care         */
		unsigned div_ref      :  6;  /* 31 : 26  Input clk divider  */
		/* Byte 4 ~ Byte 7 */
		unsigned loopc        : 10;   /* 41 : 32 Clock multiplier   */
		unsigned _reserved_2_ : 22;   /* 42 : 63 Useless            */

		/* Byte 8 ~ Byte 15 */
		unsigned div_out_dc   :  6;  /*  5 : 0  DC output divider */
		unsigned _reserved_3_ : 16;  /* 21 : 6  */
		unsigned div_out_gmac :  6;  /* 27 : 22 GMAC output divider */
		unsigned _reserved_4_ :  4;  /* 31 : 28 */
		unsigned _reserved_5_ : 32;  /* 63 : 32 */
	} bitmap;
	u32 w[4];
	u64 d[2];
};

static int ls2k1000_gfxpll_init(struct loongson_gfxpll * const this)
{
	struct drm_printer printer = drm_info_printer(this->ddev->dev);

	this->mmio = ioremap(this->reg_base, this->reg_size);
	if (IS_ERR_OR_NULL(this->mmio))
		return -ENOMEM;

	this->funcs->print(this, &printer, false);

	return 0;
}

static inline void __rreg_u128(void __iomem *mmio, u32 w[], u64 d[])
{
#if defined(CONFIG_64BIT)
	d[0] = readq(mmio);
	d[1] = readq(mmio + 8);
#else
	w[0] = readl(mmio);
	w[1] = readl(mmio + 4);
	w[2] = readl(mmio + 8);
	w[3] = readl(mmio + 12);
#endif
}

static void ls2k1000_gfxpll_get_rates(struct loongson_gfxpll * const this,
				      unsigned int *dc,
				      unsigned int *ddr,
				      unsigned int *gpu)
{
	unsigned int ref_clock_mhz = LSDC_PLL_REF_CLK_KHZ / 1000;
	union ls2k1000_ddr_pll_reg_bitmap ddr_pll_reg_bitmap;
	union ls2k1000_dc_pll_reg_bitmap dc_pll_reg_bitmap;
	unsigned int div_ref;
	unsigned int loopc;
	unsigned int div_out_dc;
	unsigned int div_out_ddr;
	unsigned int div_out_gpu;
	unsigned int dc_mhz;
	unsigned int ddr_mhz;
	unsigned int gpu_mhz;

	__rreg_u128(this->mmio, ddr_pll_reg_bitmap.w, ddr_pll_reg_bitmap.d);
	div_ref = ddr_pll_reg_bitmap.bitmap.div_ref;
	loopc = ddr_pll_reg_bitmap.bitmap.loopc;
	div_out_ddr = ddr_pll_reg_bitmap.bitmap.div_out_ddr;
	div_out_gpu = ddr_pll_reg_bitmap.bitmap.div_out_gpu;
	ddr_mhz = ref_clock_mhz / div_ref * loopc / div_out_ddr;
	gpu_mhz = ref_clock_mhz / div_ref * loopc / div_out_gpu;

	__rreg_u128(this->mmio + 16, dc_pll_reg_bitmap.w, dc_pll_reg_bitmap.d);
	div_out_dc = dc_pll_reg_bitmap.bitmap.div_out_dc;
	div_ref = dc_pll_reg_bitmap.bitmap.div_ref;
	loopc = dc_pll_reg_bitmap.bitmap.loopc;
	dc_mhz = ref_clock_mhz / div_ref * loopc / div_out_dc;

	if (dc)
		*dc = dc_mhz;

	if (ddr)
		*ddr = ddr_mhz;

	if (gpu)
		*gpu = gpu_mhz;
}

static void ls2k1000_gfxpll_print(struct loongson_gfxpll * const this,
				  struct drm_printer *p,
				  bool verbose)
{
	unsigned int dc, ddr, gpu;

	this->funcs->get_rates(this, &dc, &ddr, &gpu);

	drm_printf(p, "dc: %uMHz, ddr: %uMHz, gpu: %uMHz\n", dc, ddr, gpu);
}

const struct loongson_gfxpll_funcs ls2k1000_gfx_pll_funcs = {
	.init = ls2k1000_gfxpll_init,
	.get_rates = ls2k1000_gfxpll_get_rates,
	.print = ls2k1000_gfxpll_print,
};
