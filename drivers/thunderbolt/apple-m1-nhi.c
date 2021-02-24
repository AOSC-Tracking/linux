// SPDX-License-Identifier: GPL-2.0
/*
 * Apple M1 SoC Converged-IO NHI driver
 *
 * Copyright (c) 2021 Corellium LLC
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/thunderbolt.h>
#include <linux/apple_atc_phy.h>

#include "nhi.h"
#include "nhi_regs.h"
#include "tb.h"
#include "ctl.h"

#define DEBUG_CTL	0

#define MAX_RS		4
#define RS_NHI		0
#define RS_M3		1

struct apple_m1_nhi {
	struct device *dev;
	void __iomem *reg[MAX_RS];
	int num_reg;
	struct clk_bulk_data *clks;
	int num_clks;
	u32 num_rings;
	struct pinctrl *pctrl;
	u8 *drom;
	unsigned drom_len;
	struct apple_atc_phy_conn phy_conn;

	struct apple_m1_nhi_ring {
		struct apple_m1_nhi *nhi;
		struct tb_ring tbring;
	} *rings;

	struct tb_nhi tbnhi;
	struct tb *tb;
	struct tb_switch *hostsw;
	struct mutex hostsw_mutex;
};
#define M1_RING(tbr) container_of((tbr), struct apple_m1_nhi_ring, tbring)
#define M1_NHI(tbn) container_of((tbn), struct apple_m1_nhi, tbnhi)

#define RINGHOP_FREE			-1
#define RINGHOP_BUSY			-2

#define M3_CONTROL			0x0000C
#define   M3_CONTROL_RUN		BIT(1)
#define M3_STATUS			0x000A8
#define   M3_STATUS_FWSTATE_MASK	(0x7F << 24)

#define NHI_HOP_COUNT			0x00000
#define   NHI_HOP_COUNT_MASK		0x7FF

#define NHI_RING(r)			((r) * 0x20)

#define NHI_TXRING_BASE_LO		0x10000
#define NHI_TXRING_BASE_HI		0x10004
#define NHI_TXRING_PTRS			0x10008
#define   NHI_TXRING_PTRS_WPTR_MASK	(0xFFFF << 16)
#define     NHI_TXRING_PTRS_WPTR_SHIFT	16
#define   NHI_TXRING_PTRS_RPTR_MASK	(0xFFFF << 0)
#define     NHI_TXRING_PTRS_RPTR_SHIFT	0
#define NHI_TXRING_NUM			0x1000C
#define NHI_TXRING_FLAGS		0x10010
#define NHI_TXRING_CONFIG		0x10014
#define   NHI_TXRING_CONFIG_VAL		2

#define NHI_RXRING_BASE_LO		0x80000
#define NHI_RXRING_BASE_HI		0x80004
#define NHI_RXRING_PTRS			0x80008
#define   NHI_RXRING_PTRS_RPTR_MASK	(0xFFFF << 16)
#define     NHI_RXRING_PTRS_RPTR_SHIFT	16
#define   NHI_RXRING_PTRS_WPTR_MASK	(0xFFFF << 0)
#define     NHI_RXRING_PTRS_WPTR_SHIFT	0
#define NHI_RXRING_SIZES		0x8000C
#define   NHI_RXRING_SIZES_BUF_MASK	(0xFFFF << 16)
#define     NHI_RXRING_SIZES_BUF_SHIFT	16
#define   NHI_RXRING_SIZES_NUM_MASK	(0xFFFF << 0)
#define     NHI_RXRING_SIZES_NUM_SHIFT	0
#define NHI_RXRING_FLAGS		0x80010
#define NHI_RXRING_PDFFILT		0x80014
#define   NHI_RXRING_PDFFILT_SOF_MASK	(0xFFFF << 16)
#define   NHI_RXRING_PDFFILT_SOF_SHIFT	16
#define   NHI_RXRING_PDFFILT_EOF_MASK	(0xFFFF << 0)
#define   NHI_RXRING_PDFFILT_EOF_SHIFT	0

#define NHI_IRQ_STAT_0			0xD0000
#define NHI_IRQ_STAT_1			0xD0004
#define NHI_IRQ_ENABLE			0xD0010
#define NHI_IRQ_THROTTLE(r)		(0xD004C + 4 * (r))

#define HOSTSW_CABLE_PRESENT		BIT(0)
#define HOSTSW_CABLE_ORIENTATION	BIT(1)
#define HOSTSW_CABLE_ACTIVE		BIT(2)
#define HOSTSW_CABLE_LINK_TRAINING	BIT(3)
#define HOSTSW_CABLE_20_GBPS		BIT(4)
#define HOSTSW_CABLE_LEGACY_ADAPTER	BIT(9)
#define HOSTSW_CABLE_TBT2_3		BIT(10)

#define m3_writel(val,offs) writel(val, nhi->reg[RS_M3] + (offs))
#define nhi_writel(val,offs) writel(val, nhi->reg[RS_NHI] + (offs))
#define nhi_readl(offs) readl(nhi->reg[RS_NHI] + (offs))

static void apple_m1_nhi_irq_enable(struct apple_m1_nhi *nhi, unsigned txring, unsigned ringidx)
{
	if(!txring)
		ringidx += nhi->num_rings;
	nhi_writel(nhi_readl(NHI_IRQ_ENABLE) | BIT(ringidx), NHI_IRQ_ENABLE);
}

static void apple_m1_nhi_irq_disable(struct apple_m1_nhi *nhi, unsigned txring, unsigned ringidx)
{
	if(!txring)
		ringidx += nhi->num_rings;
	nhi_writel(nhi_readl(NHI_IRQ_ENABLE) & ~BIT(ringidx), NHI_IRQ_ENABLE);
}

static void apple_m1_nhi_ring_start(struct tb_ring *tbring)
{
	struct apple_m1_nhi *nhi = M1_RING(tbring)->nhi;
	u16 frame_size;
	u32 flags;
	unsigned hop;

	spin_lock_irq(&tbring->nhi->lock);
	spin_lock(&tbring->lock);
	if(tbring->nhi->going_away)
		goto err;
	if(tbring->hop < 0) {
		dev_warn(nhi->dev, "invalid %s ring for start\n", tbring->is_tx ? "tx" : "rx");
		goto err;
	}
	hop = tbring->hop;
	if(tbring->running) {
		dev_warn(nhi->dev, "%s ring %d already started\n", tbring->is_tx ? "tx" : "rx",
			hop);
		goto err;
	}
	dev_dbg(&tbring->nhi->pdev->dev, "starting %s ring %d\n", tbring->is_tx ? "tx" : "rx",
		hop);

	if(tbring->flags & RING_FLAG_FRAME) {
		/* Means 4096 */
		frame_size = 0;
		flags = RING_FLAG_ENABLE;
	} else {
		frame_size = TB_FRAME_SIZE;
		flags = RING_FLAG_ENABLE | RING_FLAG_RAW;
	}

	if(tbring->is_tx) {
		nhi_writel(tbring->descriptors_dma,		NHI_RING(hop) + NHI_TXRING_BASE_LO);
		nhi_writel(tbring->descriptors_dma >> 32,	NHI_RING(hop) + NHI_TXRING_BASE_HI);
		nhi_writel(0,					NHI_RING(hop) + NHI_TXRING_PTRS);
		nhi_writel(tbring->size,			NHI_RING(hop) + NHI_TXRING_NUM);
		nhi_writel(2,					NHI_RING(hop) + NHI_TXRING_CONFIG);
		nhi_writel(flags,				NHI_RING(hop) + NHI_TXRING_FLAGS);
	} else {
		u32 pdffilt = (tbring->sof_mask << NHI_RXRING_PDFFILT_SOF_SHIFT) |
			      (tbring->eof_mask << NHI_RXRING_PDFFILT_EOF_SHIFT);
		u32 sizes = (tbring->size << NHI_RXRING_SIZES_NUM_SHIFT) |
			    (frame_size << NHI_RXRING_SIZES_BUF_SHIFT);

		nhi_writel(tbring->descriptors_dma,		NHI_RING(hop) + NHI_RXRING_BASE_LO);
		nhi_writel(tbring->descriptors_dma >> 32,	NHI_RING(hop) + NHI_RXRING_BASE_HI);
		nhi_writel(0,					NHI_RING(hop) + NHI_RXRING_PTRS);
		nhi_writel(sizes,				NHI_RING(hop) + NHI_RXRING_SIZES);
		nhi_writel(pdffilt,				NHI_RING(hop) + NHI_RXRING_PDFFILT);
		nhi_writel(flags,				NHI_RING(hop) + NHI_RXRING_FLAGS);
	}

	/*
	 * Now that the ring valid bit is set we can configure E2E if
	 * enabled for the ring.
	 */
	if(tbring->flags & RING_FLAG_E2E) {
		if(!tbring->is_tx) {
			u32 hop;

			hop = tbring->e2e_tx_hop << REG_RX_OPTIONS_E2E_HOP_SHIFT;
			hop &= REG_RX_OPTIONS_E2E_HOP_MASK;
			flags |= hop;
			flags |= RING_FLAG_E2E_FLOW_CONTROL;

			dev_dbg(nhi->dev, "enabling E2E for rx ring %d with tx hop ID %d\n", hop,
				tbring->e2e_tx_hop);
			nhi_writel(flags, NHI_RING(hop) + NHI_RXRING_FLAGS);
		} else {
			flags |= RING_FLAG_E2E_FLOW_CONTROL;

			dev_dbg(nhi->dev, "enabling E2E for tx ring %d\n", hop);
			nhi_writel(flags, NHI_RING(hop) + NHI_TXRING_FLAGS);
		}
	}

	apple_m1_nhi_irq_enable(nhi, tbring->is_tx, hop);
	tbring->running = true;
err:
	spin_unlock(&tbring->lock);
	spin_unlock_irq(&tbring->nhi->lock);
}

static void apple_m1_nhi_ring_stop(struct tb_ring *tbring)
{
	struct apple_m1_nhi *nhi = M1_RING(tbring)->nhi;
	unsigned hop;

	spin_lock_irq(&tbring->nhi->lock);
	spin_lock(&tbring->lock);

	if(tbring->hop < 0) {
		dev_warn(nhi->dev, "invalid %s ring for stop\n", tbring->is_tx ? "tx" : "rx");
		goto err;
	}
	hop = tbring->hop;

	dev_dbg(nhi->dev, "stopping %s ring %d\n", tbring->is_tx ? "tx" : "rx", hop);
	if(tbring->nhi->going_away)
		goto err;
	if(!tbring->running) {
		dev_warn(nhi->dev, "%s ring %d already stopped\n", tbring->is_tx ? "tx" : "rx",
			hop);
		goto err;
	}

	apple_m1_nhi_irq_disable(nhi, tbring->is_tx, hop);

	if(tbring->is_tx) {
		nhi_writel(0, NHI_RING(hop) + NHI_TXRING_FLAGS);
		nhi_writel(0, NHI_RING(hop) + NHI_TXRING_BASE_LO);
		nhi_writel(0, NHI_RING(hop) + NHI_TXRING_BASE_HI);
		nhi_writel(0, NHI_RING(hop) + NHI_TXRING_PTRS);
		nhi_writel(0, NHI_RING(hop) + NHI_TXRING_NUM);
		nhi_writel(0, NHI_RING(hop) + NHI_TXRING_CONFIG);
	} else {
		nhi_writel(0, NHI_RING(hop) + NHI_RXRING_FLAGS);
		nhi_writel(0, NHI_RING(hop) + NHI_RXRING_BASE_LO);
		nhi_writel(0, NHI_RING(hop) + NHI_RXRING_BASE_HI);
		nhi_writel(0, NHI_RING(hop) + NHI_RXRING_PTRS);
		nhi_writel(0, NHI_RING(hop) + NHI_RXRING_SIZES);
		nhi_writel(0, NHI_RING(hop) + NHI_RXRING_PDFFILT);
	}

	tbring->head = 0;
	tbring->tail = 0;
	tbring->running = false;

err:
	spin_unlock(&tbring->lock);
	spin_unlock_irq(&tbring->nhi->lock);

	/*
	 * schedule ring->work to invoke callbacks on all remaining frames.
	 */
	schedule_work(&tbring->work);
	flush_work(&tbring->work);
}

static void apple_m1_nhi_ring_work(struct work_struct *work);

static struct tb_ring *apple_m1_nhi_ring_alloc(struct tb_nhi *tbnhi, u32 hop, int size,
				bool transmit, unsigned int flags, int e2e_tx_hop,
				u16 sof_mask, u16 eof_mask, void (*start_poll)(void *),
				void *poll_data)
{
	struct tb_ring *tbring = NULL;
	dma_addr_t descdma;
	void *descp;

	dev_dbg(tbnhi->dev, "allocating %s ring %d of size %d\n",
		transmit ? "TX" : "RX", hop, size);

	descp = dma_alloc_coherent(tbnhi->dev, size * sizeof(struct ring_desc),
			&descdma, GFP_KERNEL | __GFP_ZERO);
	if(!descp)
		return NULL;

	/* allocate hop ID if required */

	spin_lock_irq(&tbnhi->lock);
	if((int)hop < 0) {
		for(hop=1; hop<tbnhi->hop_count; hop++) {
			tbring = transmit ? tbnhi->tx_rings[hop] : tbnhi->rx_rings[hop];
			if(tbring->hop == RINGHOP_FREE)
				break;
		}
		if(hop >= tbnhi->hop_count) {
			dev_err(tbnhi->dev, "out of %s rings.\n",
				transmit ? "tx" : "rx");
			goto fail;
		}
		tbring->hop = hop;
	} else {
		tbring = transmit ? tbnhi->tx_rings[hop] : tbnhi->rx_rings[hop];
		if(tbring->hop >= 0) {
			dev_err(tbnhi->dev, "%s ring %d already in use.\n",
				transmit ? "tx" : "rx", hop);
			goto fail;
		}
		tbring->hop = hop;
	}

	INIT_LIST_HEAD(&tbring->queue);
	INIT_LIST_HEAD(&tbring->in_flight);
	INIT_WORK(&tbring->work, apple_m1_nhi_ring_work);

	tbring->nhi = tbnhi;
	tbring->is_tx = transmit;
	tbring->size = size;
	tbring->flags = flags;
	tbring->e2e_tx_hop = e2e_tx_hop;
	tbring->sof_mask = sof_mask;
	tbring->eof_mask = eof_mask;
	tbring->head = 0;
	tbring->tail = 0;
	tbring->running = false;
	tbring->start_poll = start_poll;
	tbring->poll_data = poll_data;

	tbring->descriptors = descp;
	tbring->descriptors_dma = descdma;

	spin_unlock_irq(&tbnhi->lock);
	return tbring;

fail:
	spin_unlock_irq(&tbnhi->lock);
	dma_free_coherent(tbnhi->dev, size * sizeof(struct ring_desc), descp, descdma);
	return NULL;
}

static struct tb_ring *apple_m1_nhi_ring_alloc_tx(struct tb_nhi *tbnhi, int hop, int size,
				 unsigned int flags)
{
	return apple_m1_nhi_ring_alloc(tbnhi, hop, size, true, flags, 0, 0, 0, NULL, NULL);
}

static struct tb_ring *apple_m1_nhi_ring_alloc_rx(struct tb_nhi *tbnhi, int hop, int size,
				 unsigned int flags, int e2e_tx_hop,
				 u16 sof_mask, u16 eof_mask,
				 void (*start_poll)(void *), void *poll_data)
{
	return apple_m1_nhi_ring_alloc(tbnhi, hop, size, false, flags, e2e_tx_hop, sof_mask,
				eof_mask, start_poll, poll_data);
}

static void apple_m1_nhi_ring_free(struct tb_ring *tbring)
{
	spin_lock_irq(&tbring->nhi->lock);
	if(tbring->running) {
		dev_warn(tbring->nhi->dev, "%s ring %d still running\n",
			 tbring->is_tx ? "tx" : "rx", tbring->hop);
	}
	tbring->hop = RINGHOP_BUSY;
	spin_unlock_irq(&tbring->nhi->lock);

	dev_dbg(tbring->nhi->dev, "freeing %s ring %d\n", tbring->is_tx ? "tx" : "rx", tbring->hop);

	dma_free_coherent(tbring->nhi->dev,
			  tbring->size * sizeof(*tbring->descriptors),
			  tbring->descriptors, tbring->descriptors_dma);
	tbring->descriptors = NULL;
	tbring->descriptors_dma = 0;

	flush_work(&tbring->work);

	spin_lock_irq(&tbring->nhi->lock);
	tbring->hop = RINGHOP_FREE;
	spin_unlock_irq(&tbring->nhi->lock);
}

static void apple_m1_nhi_ring_write_descriptors(struct tb_ring *tbring)
{
	struct apple_m1_nhi *nhi = M1_RING(tbring)->nhi;
	struct ring_frame *frame, *n;
	struct ring_desc *descriptor;
	int nhead;

	list_for_each_entry_safe(frame, n, &tbring->queue, list) {
		nhead = (tbring->head + 1) % tbring->size;
		if(nhead == tbring->tail)
			break;
		list_move_tail(&frame->list, &tbring->in_flight);
		descriptor = &tbring->descriptors[tbring->head];
#if DEBUG_CTL
		if(tbring->is_tx) {
			dev_info(nhi->dev, "tx%d %d-%d:\n", tbring->hop, frame->sof, frame->eof);
			if(!tbring->hop) {
				struct ctl_pkg *pkg = container_of(frame, struct ctl_pkg, frame);
				print_hex_dump(KERN_INFO, " > ", DUMP_PREFIX_OFFSET, 32, 4, pkg->buffer, frame->size, false);
			}
		}
#endif
		descriptor->phys = frame->buffer_phy;
		descriptor->time = 0;
		descriptor->flags = RING_DESC_POSTED | RING_DESC_INTERRUPT;
		if(tbring->is_tx) {
			descriptor->length = frame->size;
			descriptor->eof = frame->eof;
			descriptor->sof = frame->sof;
		}
		tbring->head = nhead;
		if(tbring->is_tx)
			nhi_writel(nhead << NHI_TXRING_PTRS_WPTR_SHIFT,
				NHI_RING(tbring->hop) + NHI_TXRING_PTRS);
		else
			nhi_writel(nhead << NHI_RXRING_PTRS_WPTR_SHIFT,
				NHI_RING(tbring->hop) + NHI_RXRING_PTRS);
	}
}

static void apple_m1_nhi_ring_work(struct work_struct *work)
{
	struct tb_ring *tbring = container_of(work, typeof(*tbring), work);
	struct ring_frame *frame;
	bool canceled = false;
	unsigned long flags;
	LIST_HEAD(done);

	spin_lock_irqsave(&tbring->lock, flags);

	if(!tbring->running) {
		/*  Move all frames to done and mark them as canceled. */
		list_splice_tail_init(&tbring->in_flight, &done);
		list_splice_tail_init(&tbring->queue, &done);
		canceled = true;
		goto invoke_callback;
	}

	while(tbring->head != tbring->tail) {
		if(!(tbring->descriptors[tbring->tail].flags & RING_DESC_COMPLETED))
			break;
		frame = list_first_entry(&tbring->in_flight, typeof(*frame), list);
		list_move_tail(&frame->list, &done);
		if(!tbring->is_tx) {
			frame->size = tbring->descriptors[tbring->tail].length;
			frame->eof = tbring->descriptors[tbring->tail].eof;
			frame->sof = tbring->descriptors[tbring->tail].sof;
			frame->flags = tbring->descriptors[tbring->tail].flags;
#if DEBUG_CTL
			dev_info(tbring->nhi->dev, "rx%d %d-%d:\n", tbring->hop, frame->sof, frame->eof);
			if(!tbring->hop) {
				struct ctl_pkg *pkg = container_of(frame, struct ctl_pkg, frame);
				print_hex_dump(KERN_INFO, " > ", DUMP_PREFIX_OFFSET, 32, 4, pkg->buffer, frame->size, false);
			}
#endif
		}
		tbring->tail = (tbring->tail + 1) % tbring->size;
	}
	apple_m1_nhi_ring_write_descriptors(tbring);

invoke_callback:
	/* allow callbacks to schedule new work */
	spin_unlock_irqrestore(&tbring->lock, flags);
	while(!list_empty(&done)) {
		frame = list_first_entry(&done, typeof(*frame), list);
		/*
		 * The callback may reenqueue or delete frame.
		 * Do not hold on to it.
		 */
		list_del_init(&frame->list);
		if(frame->callback)
			frame->callback(tbring, frame, canceled);
	}
}

static int apple_m1_nhi_ring_enqueue(struct tb_ring *tbring, struct ring_frame *frame)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&tbring->lock, flags);
	if(tbring->running && tbring->hop >= 0) {
		list_add_tail(&frame->list, &tbring->queue);
		apple_m1_nhi_ring_write_descriptors(tbring);
	} else {
		ret = -ESHUTDOWN;
	}
	spin_unlock_irqrestore(&tbring->lock, flags);
	return ret;
}

struct ring_frame *apple_m1_nhi_ring_poll(struct tb_ring *tbring)
{
	struct ring_frame *frame = NULL;
	unsigned long flags;

	spin_lock_irqsave(&tbring->lock, flags);
	if(!tbring->running)
		goto unlock;
	if(tbring->head == tbring->tail)
		goto unlock;

	if(tbring->descriptors[tbring->tail].flags & RING_DESC_COMPLETED) {
		frame = list_first_entry(&tbring->in_flight, typeof(*frame), list);
		list_del_init(&frame->list);

		if(!tbring->is_tx) {
			frame->size = tbring->descriptors[tbring->tail].length;
			frame->eof = tbring->descriptors[tbring->tail].eof;
			frame->sof = tbring->descriptors[tbring->tail].sof;
			frame->flags = tbring->descriptors[tbring->tail].flags;
#if DEBUG_CTL
			dev_info(tbring->nhi->dev, "rx%d %d-%d:\n", tbring->hop, frame->sof, frame->eof);
			if(!tbring->hop) {
				struct ctl_pkg *pkg = container_of(frame, struct ctl_pkg, frame);
				print_hex_dump(KERN_INFO, " > ", DUMP_PREFIX_OFFSET, 32, 4, pkg->buffer, frame->size, false);
			}
#endif
		}

		tbring->tail = (tbring->tail + 1) % tbring->size;
	}

unlock:
	spin_unlock_irqrestore(&tbring->lock, flags);
	return frame;
}

static void apple_m1_nhi_ring_poll_complete(struct tb_ring *tbring)
{
	struct apple_m1_nhi *nhi = M1_RING(tbring)->nhi;
	unsigned long flags;

	spin_lock_irqsave(&tbring->nhi->lock, flags);
	spin_lock(&tbring->lock);
	if(tbring->start_poll)
		apple_m1_nhi_irq_enable(nhi, tbring->is_tx, tbring->hop);
	spin_unlock(&tbring->lock);
	spin_unlock_irqrestore(&tbring->nhi->lock, flags);
}

/* in pcie-apple-m1.c */
void pcie_apple_m1_start_pcic_tunnel(void *pcie);

static void apple_m1_nhi_notify_pci_tunnel(struct tb_nhi *tbnhi, unsigned adapter)
{
	struct apple_m1_nhi *nhi = M1_NHI(tbnhi);
	struct device_node *pcinp = of_parse_phandle(nhi->dev->of_node, "pcie", 0);
	struct platform_device *pcipd;

	if(!pcinp) {
		dev_warn(nhi->dev, "Missing 'pcie' property, can't notify PCIe bus.\n");
		return;
	}
	pcipd = of_find_device_by_node(pcinp);

	if(!pcipd) {
		dev_warn(nhi->dev, "PCIe bus not available, can't notify.\n");
		return;
	}

	pcie_apple_m1_start_pcic_tunnel(platform_get_drvdata(pcipd));
}

int apple_m1_nhi_read_drom(struct tb_nhi *tbnhi, unsigned offs, void *buf, unsigned size)
{
	struct apple_m1_nhi *nhi = M1_NHI(tbnhi);

	if(!nhi->drom_len)
		return -ENOTSUPP;

	if(!buf)
		return nhi->drom_len;

	if(offs > nhi->drom_len)
		return -EINVAL;
	if(size > nhi->drom_len - offs)
		size = nhi->drom_len - offs;

	memcpy(buf, nhi->drom + offs, size);
	return size;
}

static void apple_m1_nhi_phy_notify(struct apple_atc_phy_conn *conn)
{
	struct apple_m1_nhi *nhi = conn->client;
	struct tb_switch *sw;
	u32 cable_info = 0;
	int ret;

	mutex_lock(&nhi->hostsw_mutex);
	sw = nhi->hostsw;

	if (sw && sw->cap_vsc0) {
		if(conn->mode == TYPEC_MODE_USB4) {
			cable_info = HOSTSW_CABLE_PRESENT;
			if(conn->orientation == TYPEC_ORIENTATION_REVERSE)
				cable_info |= HOSTSW_CABLE_ORIENTATION;
			if(conn->cable.speed >= TYPEC_CABLE_SPEED_20_GBPS)
				cable_info |= HOSTSW_CABLE_20_GBPS;
			if(conn->cable.is_active) {
				cable_info |= HOSTSW_CABLE_ACTIVE;
				if(conn->cable.link_training)
					cable_info |= HOSTSW_CABLE_LINK_TRAINING;
			}
			if(conn->cable.is_legacy_adapter)
				cable_info |= HOSTSW_CABLE_LEGACY_ADAPTER;
			cable_info |= HOSTSW_CABLE_TBT2_3;
		}

		if(cable_info)
			msleep(250);

		tb_sw_info(sw, "setting Apple cable info to 0x%x\n", cable_info);
		ret = tb_sw_write(sw, &cable_info, TB_CFG_SWITCH, sw->cap_vsc0 + 1, 1);
		if(ret)
			tb_sw_warn(sw, "setting Apple cable info failed: %d\n", ret);

		if(!cable_info)
			msleep(250);
	} else if(sw)
		tb_sw_warn(sw, "cannot find Apple TB_VSE_CAP_VSC0\n");

	mutex_unlock(&nhi->hostsw_mutex);
}

static void apple_m1_nhi_attach_host_switch(struct tb_switch *sw, int attach)
{
	struct apple_m1_nhi *nhi = M1_NHI(sw->tb->nhi);

	mutex_lock(&nhi->hostsw_mutex);
	if(attach) {
		nhi->hostsw = sw;
		mutex_unlock(&nhi->hostsw_mutex);

		apple_atc_phy_conn_register(NULL, &nhi->phy_conn);
	} else {
		nhi->hostsw = NULL;
		mutex_unlock(&nhi->hostsw_mutex);
	}
}

static const struct tb_nhi_ops apple_m1_nhi_ops = {
	.ring_start = apple_m1_nhi_ring_start,
	.ring_stop = apple_m1_nhi_ring_stop,
	.ring_alloc_tx = apple_m1_nhi_ring_alloc_tx,
	.ring_alloc_rx = apple_m1_nhi_ring_alloc_rx,
	.ring_free = apple_m1_nhi_ring_free,
	.ring_enqueue = apple_m1_nhi_ring_enqueue,

	.ring_poll = apple_m1_nhi_ring_poll,
	.ring_poll_complete = apple_m1_nhi_ring_poll_complete,

	.notify_pci_tunnel = apple_m1_nhi_notify_pci_tunnel,
	.read_drom = apple_m1_nhi_read_drom,

	.attach_host_switch = apple_m1_nhi_attach_host_switch,
};

static irqreturn_t apple_m1_nhi_irq(int irq, void *dev_id)
{
	struct apple_m1_nhi_ring *ring = dev_id;
	struct apple_m1_nhi *nhi = ring->nhi;
	struct tb_ring *tbring = &ring->tbring;
	unsigned ringidx = ring - nhi->rings;
	unsigned txring = ringidx < nhi->num_rings;

	nhi_writel(BIT(ringidx), NHI_IRQ_STAT_0);

	if(!txring)
		ringidx -= nhi->num_rings;

	spin_lock(&tbring->nhi->lock);
	spin_lock(&tbring->lock);

	if(tbring->running) {
		if(tbring->start_poll) {
			apple_m1_nhi_irq_disable(nhi, txring, ringidx);
			tbring->start_poll(tbring->poll_data);
		} else
			schedule_work(&tbring->work);
	}

	spin_unlock(&tbring->lock);
	spin_unlock(&tbring->nhi->lock);

	return IRQ_HANDLED;
}

static void apple_m1_nhi_shutdown(struct apple_m1_nhi *nhi)
{
	int i;

	dev_dbg(nhi->dev, "shutdown\n");

	for(i=0; i<nhi->tbnhi.hop_count; i++) {
		if(nhi->tbnhi.tx_rings[i]->hop >= 0)
			dev_warn(nhi->dev, "tx ring %d is still active\n", i);
		if(nhi->tbnhi.rx_rings[i]->hop >= 0)
			dev_warn(nhi->dev, "rx ring %d is still active\n", i);
	}

	nhi_writel(0, NHI_IRQ_ENABLE);
	nhi_writel(-1, NHI_IRQ_STAT_0);
	nhi_writel(-1, NHI_IRQ_STAT_1);
}

/* this plays back tunables passed from bootloader. those are somewhat hardware specific,
   and ultimately provided by the first-stage platform bootloader.

   i kind of wish i could factor this out as library function since we use this tunable
   format all over the place. */
static int apple_m1_nhi_tunable(struct apple_m1_nhi *nhi, const char *name)
{
	struct device_node *np = nhi->dev->of_node;
	int cnt, idx;
	u32 addr, mask, val, range, tmp;

	cnt = of_property_count_elems_of_size(np, name, sizeof(u32));
	if(cnt < 0) {
		pr_warn("%pOFn: %s: missing tunable [%s].\n", np, __func__, name);
		return 0;
	}
	if(cnt % 3) {
		pr_warn("%pOFn: %s: tunable [%s] is not made of <addr, mask, val> triplets.\n",
			np, __func__, name);
		return -EINVAL;
	}

	for(idx=0; idx<cnt; idx+=3) {
		if(of_property_read_u32_index(np, name, idx, &addr) < 0 ||
		   of_property_read_u32_index(np, name, idx + 1, &mask) < 0 ||
		   of_property_read_u32_index(np, name, idx + 2, &val)) {
			pr_warn("%pOFn: %s: tunable [%s] not readable.\n", np, __func__, name);
			return -EINVAL;
		}
		range = addr >> 28;
		addr &= 0x0FFFFFFF;

		if(range < nhi->num_reg) {
			tmp = readl(nhi->reg[range] + addr);
			tmp = (tmp & ~mask) | (val & mask);
			writel(tmp, nhi->reg[range] + addr);
		} else
			pr_warn("%pOFn: %s: tunable [%s] refers to nonexistent range %d.\n",
				np, __func__, name, range);
	}

	return 0;
}

static const char *const apple_m1_nhi_tunables[] = {
	"tunable-top",
	"tunable-hbw_fabric",
	"tunable-lbw_fabric",
	"tunable-hi_up_tx_desc_fabric",
	"tunable-hi_up_tx_data_fabric",
	"tunable-hi_up_rx_desc_fabric",
	"tunable-hi_up_wr_fabric",
	"tunable-hi_up_merge_fabric",
	"tunable-hi_dn_merge_fabric",
	"tunable-fw_int_ctl_management",
	"tunable-pcie_adapter_regs",
	NULL
};

static int apple_m1_nhi_start(struct apple_m1_nhi *nhi)
{
	int ret, i;
	u32 rdv;

	m3_writel(M3_CONTROL_RUN, M3_CONTROL);
	ret = readl_poll_timeout(nhi->reg[RS_M3] + M3_STATUS, rdv,
		(rdv & M3_STATUS_FWSTATE_MASK), 100, 500000);
	if(ret < 0) {
		dev_err(nhi->dev, "M3 firmware did not boot, state: 0x%08x.\n", rdv);
		return ret;
	}

	for(i=0; apple_m1_nhi_tunables[i]; i++) {
		ret = apple_m1_nhi_tunable(nhi, apple_m1_nhi_tunables[i]);
		if(ret < 0)
			return ret;
	}

	nhi->tbnhi.hop_count = nhi_readl(NHI_HOP_COUNT) & NHI_HOP_COUNT_MASK;

	return 0;
}

/* this lets us check that functions aren't grossly incompatible, should the arguments change */
static void wrap_apple_atc_phy_conn_deregister(void *p) { apple_atc_phy_conn_deregister(p); }

static int apple_m1_nhi_probe(struct platform_device *pdev)
{
	struct apple_m1_nhi *nhi;
	struct resource *rsrc;
	struct property *prop;
	struct device_node *phynode;
	struct platform_device *phypdev;
	int err, i;

	nhi = devm_kzalloc(&pdev->dev, sizeof(*nhi), GFP_KERNEL);
	if(!nhi)
		return -ENOMEM;
	nhi->dev = &pdev->dev;

	err = of_property_read_u32(nhi->dev->of_node, "num-rings", &nhi->num_rings);
	if(err < 0) {
		dev_err(&pdev->dev, "property 'num-rings' is required.\n");
		return err;
	}

	prop = of_find_property(nhi->dev->of_node, "thunderbolt-drom", &err);
	if(prop) {
		nhi->drom = devm_kzalloc(&pdev->dev, err, GFP_KERNEL);
		if(!nhi->drom)
			return -ENOMEM;
		nhi->drom_len = err;
		memcpy(nhi->drom, prop->value, nhi->drom_len);
	} else
		dev_warn(&pdev->dev, "property 'thunderbolt-drom' may be needed for operation.\n");

	nhi->rings = devm_kzalloc(nhi->dev,
		sizeof(struct apple_m1_nhi_ring) * 2 * nhi->num_rings, GFP_KERNEL);
	if(!nhi->rings)
		return -ENOMEM;
	for(i=0; i<2*nhi->num_rings; i++) {
		nhi->rings[i].nhi = nhi;
		nhi->rings[i].tbring.irq = platform_get_irq(pdev, i);
		if(nhi->rings[i].tbring.irq < 0) {
			dev_err(&pdev->dev, "property 'interrupts' needs %d entries.\n",
				2 * nhi->num_rings);
			return nhi->rings[i].tbring.irq;
		}
		spin_lock_init(&nhi->rings[i].tbring.lock);
		nhi->rings[i].tbring.hop = RINGHOP_FREE;
	}

	spin_lock_init(&nhi->tbnhi.lock);
	mutex_init(&nhi->hostsw_mutex);
	nhi->tbnhi.dev = nhi->dev;
	nhi->tbnhi.ops = &apple_m1_nhi_ops;

	nhi->tbnhi.tx_rings = devm_kzalloc(nhi->dev,
		sizeof(struct tb_ring *) * 2 * nhi->num_rings, GFP_KERNEL);
	if(!nhi->tbnhi.tx_rings)
		return -ENOMEM;
	nhi->tbnhi.rx_rings = nhi->tbnhi.tx_rings + nhi->num_rings;
	for(i=0; i<2*nhi->num_rings; i++)
		nhi->tbnhi.tx_rings[i] = &nhi->rings[i].tbring;

	nhi->tbnhi.quirks = NHI_QUIRK_NO_HOST_ADAPTER_0 |
			    NHI_QUIRK_NO_HOST_ROUTE_UP_BIT;

	err = clk_bulk_get_all(nhi->dev, &nhi->clks);
	if(err < 0) {
		dev_err(&pdev->dev, "clk_bulk_get_all failed.\n");
		return err;
	}
	nhi->num_clks = err;

	for(i=0; i<pdev->num_resources; i++) {
		rsrc = &pdev->resource[i];
		if(resource_type(rsrc) != IORESOURCE_MEM)
			continue;

		if(nhi->num_reg >= MAX_RS) {
			dev_warn(nhi->dev, "too many 'reg' entries, expected up to %d.\n", MAX_RS);
			break;
		}

		nhi->reg[nhi->num_reg] = devm_ioremap(nhi->dev, rsrc->start, resource_size(rsrc));
		if(IS_ERR(nhi->reg[nhi->num_reg])) {
			err = PTR_ERR(nhi->reg[nhi->num_reg]);
			dev_err(nhi->dev, "failed to map MMIO %d: %d.\n", nhi->num_reg, err);
			return err;
		}
		nhi->num_reg ++;
	}

	err = clk_bulk_prepare_enable(nhi->num_clks, nhi->clks);
	if(err) {
		dev_err(&pdev->dev, "clk_bulk_prepare_enable failed.\n");
		return err;
	}

	platform_set_drvdata(pdev, nhi);

	phynode = of_parse_phandle(pdev->dev.of_node, "phys", 0);
	if(!phynode) {
		dev_err(&pdev->dev, "missing 'phys' property.\n");
		return -EINVAL;
	}
	phypdev = of_find_device_by_node(phynode);
	if(!phypdev)
		return -EPROBE_DEFER;

	nhi->phy_conn.client = nhi;
	nhi->phy_conn.notify = apple_m1_nhi_phy_notify;
	err = apple_atc_phy_conn_register(&phypdev->dev, &nhi->phy_conn);
	if(err < 0)
		return err;
	err = devm_add_action(nhi->dev, wrap_apple_atc_phy_conn_deregister, &nhi->phy_conn);
	if(err)
		return err;

	nhi->pctrl = devm_pinctrl_get_select_default(nhi->dev);

	nhi_writel(0, NHI_IRQ_ENABLE);
	nhi_writel(-1, NHI_IRQ_STAT_0);
	nhi_writel(-1, NHI_IRQ_STAT_1);

	for(i=0; i<2*nhi->num_rings; i++) {
		err = devm_request_irq(nhi->dev, nhi->rings[i].tbring.irq, apple_m1_nhi_irq, 0,
				       dev_name(nhi->dev), &nhi->rings[i]);
		if(err < 0)
			return err;
	}

	err = apple_m1_nhi_start(nhi);
	if(err < 0)
		return err;

	nhi->tb = tb_probe(&nhi->tbnhi);
	if(!nhi->tb) {
		dev_err(nhi->dev, "failed to probe connection manager.\n");
		return -ENODEV;
	}

	dev_info(nhi->dev, "NHI initialized, starting Thunderbolt.\n");

	err = tb_domain_add(nhi->tb);
	if(err) {
		/*
		 * At this point the RX/TX rings might already have been
		 * activated. Do a proper shutdown.
		 */
		tb_domain_put(nhi->tb);
		apple_m1_nhi_shutdown(nhi);
		return err;
	}

	return 0;
}

static int apple_m1_nhi_remove(struct platform_device *pdev)
{
	struct apple_m1_nhi *nhi = platform_get_drvdata(pdev);

	apple_atc_phy_conn_deregister(&nhi->phy_conn);

	if(nhi->tb)
		tb_domain_remove(nhi->tb);
	apple_m1_nhi_shutdown(nhi);
	return 0;
}

static void apple_m1_nhi_shutdown_drv(struct platform_device *pdev)
{
	apple_m1_nhi_remove(pdev);
}

static const struct of_device_id apple_m1_nhi_match[] = {
	{ .compatible = "apple,cio-nhi-m1" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_m1_nhi_match);

static struct platform_driver apple_m1_nhi_driver = {
	.driver = {
		.name = "apple-m1-nhi",
		.of_match_table = apple_m1_nhi_match,
	},
	.probe = apple_m1_nhi_probe,
	.remove = apple_m1_nhi_remove,
	.shutdown = apple_m1_nhi_shutdown_drv,
};
static struct platform_driver *const apple_m1_nhi_drivers[] = {
	&apple_m1_nhi_driver,
};

static int __init apple_m1_nhi_init(void)
{
	int ret;

	ret = tb_domain_init();
	if(ret)
		return ret;
	ret = platform_register_drivers(apple_m1_nhi_drivers, 1);
	if(ret)
		tb_domain_exit();
	return ret;
}

static void __exit apple_m1_nhi_unload(void)
{
	platform_unregister_drivers(apple_m1_nhi_drivers, 1);
	tb_domain_exit();
}

rootfs_initcall(apple_m1_nhi_init);
module_exit(apple_m1_nhi_unload);

MODULE_AUTHOR("Corellium LLC");
MODULE_DESCRIPTION("Apple M1 SoC Converged-IO NHI driver");
MODULE_LICENSE("GPL");
