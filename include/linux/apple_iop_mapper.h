/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Notifier used to let devices know that memory is being mapped.
 * This is used to configure address filters in hardware.
 */

#ifndef _LINUX_APPLE_IOP_MAPPER_H_
#define _LINUX_APPLE_IOP_MAPPER_H_

#include <linux/types.h>

typedef u64 (*apple_iop_mapper_func_t)(void *, u64, u64);
int apple_iop_set_mapper_func(void *iop_mbox_chan, apple_iop_mapper_func_t func, void *priv);

#endif
