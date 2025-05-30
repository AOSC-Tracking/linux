/* SPDX-License-Identifier: GPL-2.0+ */
/* LOONGSON LSCANFD controller
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __LSCANFD_H__
#define __LSCANFD_H__

#include <linux/netdevice.h>
#include <linux/can/dev.h>
#include <linux/list.h>

enum lscanfd_can_registers;

struct lscanfd_priv {
	struct can_priv can; /* must be first member! */
	void __iomem *mem_base;
	u8 read_idx;
	u16 last_res;
	u8 txtb_flags;
	u8 canfd_dmarx;
	u32 ntxbufs;
	spinlock_t tx_lock;/*protect the sending queue from being interrupted*/
	struct napi_struct napi;
	struct device *dev;
	struct clk *can_clk;
	int irq_flags;
	struct dma_chan *rx_ch;  /* dma rx channel            */
	dma_addr_t rx_dma_buf;   /* dma rx buffer bus address */
	unsigned int *rx_buf;   /* dma rx buffer cpu address */
	resource_size_t  mapbase;  /* for ioremap */
	resource_size_t  mapsize;
};

int lscanfd_probe_common(struct device *dev, void __iomem *addr,
		resource_size_t mapbase,
		int irq, unsigned int ntxbufs,
		unsigned long can_clk_rate, bool canfd_dmarx,
		void (*set_drvdata_fnc)(struct device *dev,
			struct net_device *ndev));
#endif /*__LSCANFD__*/
