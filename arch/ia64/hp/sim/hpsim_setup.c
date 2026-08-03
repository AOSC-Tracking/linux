// SPDX-License-Identifier: GPL-2.0
/*
 * Platform dependent support for HP simulator.
 *
 * Copyright (C) 1998, 1999, 2002 Hewlett-Packard Co
 *	David Mosberger-Tang <davidm@hpl.hp.com>
 * Copyright (C) 1999 Vijay Chander <vijay@engr.sgi.com>
 */
#include <linux/console.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/param.h>
#include <linux/root_dev.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/pal.h>
/*
#include <asm/machvec.h>
*/
#include <asm/pgtable.h>
#include <asm/sal.h>
#include <asm/hpsim.h>

#include "hpsim_ssc.h"

void
ia64_ctl_trace (long on)
{
	ia64_ssc(on, 0, 0, 0, SSC_CTL_TRACE);
}

/* workaround for a warning with -Wmissing-prototypes */
void __init hpsim_setup (char **cmdline_p);

void __init
hpsim_setup (char **cmdline_p)
{
	ROOT_DEV = MKDEV(SCSI_DISK0_MAJOR, 1);		/* default to first SCSI drive */

	simcons_register();
}
