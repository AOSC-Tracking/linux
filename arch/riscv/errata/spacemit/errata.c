// SPDX-License-Identifier: GPL-2.0-only
/*
 * Errata workarounds for SpacemiT CPU cores
 */

#include <linux/bug.h>
#include <linux/cleanup.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/mutex.h>
#include <asm/alternative.h>
#include <asm/errata_list.h>
#include <asm/text-patching.h>
#include <asm/vendorid_list.h>
#include <asm/vendor_extensions.h>

#define SPACEMIT_X100_ARCHID	0x8000000058000002UL
#define SPACEMIT_A100_ARCHID	0x8000000041000002UL

static bool spacemit_errata_probe_wrs_nto(unsigned long archid)
{
	if (!IS_ENABLED(CONFIG_ERRATA_SPACEMIT_WRS_NTO))
		return false;

	/*
	 * The K3 A100 cores may not wake up from WRS.NTO. Errata are probed on
	 * the boot hart only and the kernel text is shared by all harts, so
	 * match the X100 cores of K3 as well.
	 */
	return archid == SPACEMIT_X100_ARCHID || archid == SPACEMIT_A100_ARCHID;
}

static u32 spacemit_errata_probe(unsigned long archid, unsigned long impid)
{
	u32 cpu_req_errata = 0;

	if (spacemit_errata_probe_wrs_nto(archid))
		cpu_req_errata |= BIT(ERRATA_SPACEMIT_WRS_NTO);

	return cpu_req_errata;
}

void spacemit_errata_patch_func(struct alt_entry *begin, struct alt_entry *end,
				unsigned long archid, unsigned long impid,
				unsigned int stage)
{
	u32 cpu_req_errata;
	struct alt_entry *alt;
	void *oldptr, *altptr;

	BUILD_BUG_ON(ERRATA_SPACEMIT_NUMBER >= RISCV_VENDOR_EXT_ALTERNATIVES_BASE);

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT)
		return;

	cpu_req_errata = spacemit_errata_probe(archid, impid);

	for (alt = begin; alt < end; alt++) {
		if (alt->vendor_id != SPACEMIT_VENDOR_ID ||
		    alt->patch_id >= ERRATA_SPACEMIT_NUMBER)
			continue;

		if (cpu_req_errata & BIT(alt->patch_id)) {
			oldptr = ALT_OLD_PTR(alt);
			altptr = ALT_ALT_PTR(alt);

			guard(mutex)(&text_mutex);
			patch_text_nosync(oldptr, altptr, alt->alt_len);
		}
	}
}
