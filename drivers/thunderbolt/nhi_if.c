// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thunderbolt driver - NHI interface layer
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/module.h>
#include <linux/thunderbolt.h>
#include "nhi.h"

int nhi_mailbox_cmd(struct tb_nhi *nhi, enum nhi_mailbox_cmd cmd, u32 data)
{
	if(!nhi->ops->mailbox_cmd)
		return -ENOTSUPP;
	return nhi->ops->mailbox_cmd(nhi, cmd, data);
}

enum nhi_fw_mode nhi_mailbox_mode(struct tb_nhi *nhi)
{
	if(!nhi->ops->mailbox_mode)
		return -ENOTSUPP;
	return nhi->ops->mailbox_mode(nhi);
}

struct tb_ring *tb_ring_alloc_tx(struct tb_nhi *nhi, int hop, int size,
				 unsigned int flags)
{
	return nhi->ops->ring_alloc_tx(nhi, hop, size, flags);
}
EXPORT_SYMBOL_GPL(tb_ring_alloc_rx);

struct tb_ring *tb_ring_alloc_rx(struct tb_nhi *nhi, int hop, int size,
				 unsigned int flags, int e2e_tx_hop,
				 u16 sof_mask, u16 eof_mask,
				 void (*start_poll)(void *), void *poll_data)
{
	return nhi->ops->ring_alloc_rx(nhi, hop, size, flags, e2e_tx_hop,
				sof_mask, eof_mask, start_poll, poll_data);
}
EXPORT_SYMBOL_GPL(tb_ring_alloc_tx);

void tb_ring_start(struct tb_ring *ring)
{
	ring->nhi->ops->ring_start(ring);
}
EXPORT_SYMBOL_GPL(tb_ring_start);

void tb_ring_stop(struct tb_ring *ring)
{
	ring->nhi->ops->ring_stop(ring);
}
EXPORT_SYMBOL_GPL(tb_ring_stop);

void tb_ring_free(struct tb_ring *ring)
{
	ring->nhi->ops->ring_free(ring);
}
EXPORT_SYMBOL_GPL(tb_ring_free);

int __tb_ring_enqueue(struct tb_ring *ring, struct ring_frame *frame)
{
	return ring->nhi->ops->ring_enqueue(ring, frame);
}
EXPORT_SYMBOL_GPL(__tb_ring_enqueue);

struct ring_frame *tb_ring_poll(struct tb_ring *ring)
{
	return ring->nhi->ops->ring_poll(ring);
}
EXPORT_SYMBOL_GPL(tb_ring_poll);

void tb_ring_poll_complete(struct tb_ring *ring)
{
	ring->nhi->ops->ring_poll_complete(ring);
}
EXPORT_SYMBOL_GPL(tb_ring_poll_complete);

void nhi_notify_pci_tunnel(struct tb_nhi *nhi, unsigned down_adapter)
{
	if(!nhi->ops->notify_pci_tunnel)
		return;
	nhi->ops->notify_pci_tunnel(nhi, down_adapter);
}

int nhi_read_drom(struct tb_nhi *nhi, unsigned offs, void *buf, unsigned size)
{
	if(!nhi->ops->read_drom)
		return -ENOTSUPP;
	return nhi->ops->read_drom(nhi, offs, buf, size);
}
