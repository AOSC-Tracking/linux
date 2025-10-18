// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Looongson-2 Multi-Channel DMA Controller driver
 *
 * Copyright (C) 2024-2025 Loongson Technology Corporation Limited
 *
 */

#include <linux/acpi.h>
#include <linux/acpi_dma.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "dmaengine.h"
#include "virt-dma.h"

#define LOONGSON2_MDMA_ISR		0x0 /* DMA Int Status Reg */
#define LOONGSON2_MDMA_IFCR		0x4 /* DMA Int Flag Clear Reg */
#define LOONGSON2_MDMA_CCR(x)		(0x8 + 0x18 * (x)) /* DMA channel x Configuration Register */
#define LOONGSON2_MDMA_CNDTR(x)		(0xc + 0x18 * (x))
#define LOONGSON2_MDMA_CPAR(x)		(0x10 + 0x18 * (x))
#define LOONGSON2_MDMA_CMAR(x)		(0x14 + 0x18 * (x))

/* Bitfields of DMA interrupt status register */
#define LOONGSON2_MDMA_TCI		BIT(1) /* Transfer Complete Interrupt */
#define LOONGSON2_MDMA_HTI		BIT(2) /* Half Transfer Interrupt */
#define LOONGSON2_MDMA_TEI		BIT(3) /* Transfer Error Interrupt */
#define LOONGSON2_MDMA_MASKI		(LOONGSON2_MDMA_TCI | \
					 LOONGSON2_MDMA_HTI | \
					 LOONGSON2_MDMA_TEI)

/* Bitfields of DMA channel x Configuration Register */
#define LOONGSON2_MDMA_CCR_EN		BIT(0) /* Stream Enable */
#define LOONGSON2_MDMA_CCR_TCIE		BIT(1) /* Transfer Complete Int Enable */
#define LOONGSON2_MDMA_CCR_HTIE		BIT(2) /* Half Transfer Complete Int Enable */
#define LOONGSON2_MDMA_CCR_TEIE		BIT(3) /* Transfer Error Int Enable */
#define LOONGSON2_MDMA_CCR_DIR		BIT(4)
#define LOONGSON2_MDMA_CCR_CIRC		BIT(5) /* Circular mode */
#define LOONGSON2_MDMA_CCR_PINC		BIT(6) /* Peripheral increment mode */
#define LOONGSON2_MDMA_CCR_MINC		BIT(7) /* Memory increment mode */
#define LOONGSON2_MDMA_CCR_PSIZE_MASK	GENMASK(9, 8)
#define LOONGSON2_MDMA_CCR_MSIZE_MASK	GENMASK(11, 10)
#define LOONGSON2_MDMA_CCR_PL_MASK	GENMASK(13, 12)
#define LOONGSON2_MDMA_CCR_M2M		BIT(14)

#define LOONGSON2_MDMA_CCR_CFG_MASK	(LOONGSON2_MDMA_CCR_PINC | \
					 LOONGSON2_MDMA_CCR_MINC | \
					 LOONGSON2_MDMA_CCR_PL_MASK)

#define LOONGSON2_MDMA_CCR_IRQ_MASK	(LOONGSON2_MDMA_CCR_TCIE | \
					 LOONGSON2_MDMA_CCR_HTIE | \
					 LOONGSON2_MDMA_CCR_TEIE)

#define LOONGSON2_MDMA_STREAM_MASK	(LOONGSON2_MDMA_CCR_CFG_MASK | LOONGSON2_MDMA_CCR_IRQ_MASK)

#define LOONGSON2_MDMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
					 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
					 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

enum loongson2_mdma_width {
	LOONGSON2_MDMA_BYTE,
	LOONGSON2_MDMA_HALF_WORD,
	LOONGSON2_MDMA_WORD,
};

struct loongson2_mdma_cfg {
	u32 channel_id;
	u32 stream_config;
};

struct loongson2_mdma_chan_reg {
	u32 dma_ccr;
	u32 dma_cndtr;
	u32 dma_cpar;
	u32 dma_cmar;
};

struct loongson2_mdma_sg_req {
	u32 len;
	struct loongson2_mdma_chan_reg chan_reg;
};

struct loongson2_mdma_desc {
	struct virt_dma_desc vdesc;
	bool cyclic;
	u32 num_sgs;
	struct loongson2_mdma_sg_req sg_req[] __counted_by(num_sgs);
};

struct loongson2_mdma_chan {
	struct virt_dma_chan vchan;
	u32 id;
	u32 irq;
	u32 next_sg;
	struct loongson2_mdma_desc *desc;
	struct dma_slave_config	dma_sconfig;
	struct loongson2_mdma_chan_reg chan_reg;
};

struct loongson2_mdma_dev {
	struct dma_device ddev;
	struct clk *dma_clk;
	void __iomem *base;
	int nr_channels;
	struct loongson2_mdma_chan chan[] __counted_by(nr_channels);
};

static struct loongson2_mdma_dev *lmdma_get_dev(struct loongson2_mdma_chan *lchan)
{
	return container_of(lchan->vchan.chan.device, struct loongson2_mdma_dev, ddev);
}

static struct loongson2_mdma_chan *to_lmdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct loongson2_mdma_chan, vchan.chan);
}

static struct loongson2_mdma_desc *to_lmdma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct loongson2_mdma_desc, vdesc);
}

static struct device *chan2dev(struct loongson2_mdma_chan *lchan)
{
	return &lchan->vchan.chan.dev->device;
}

static u32 loongson2_mdma_read(struct loongson2_mdma_dev *lddev, u32 reg)
{
	return readl(lddev->base + reg);
}

static void loongson2_mdma_write(struct loongson2_mdma_dev *lddev, u32 reg, u32 val)
{
	writel(val, lddev->base + reg);
}

static int loongson2_mdma_get_width(struct loongson2_mdma_chan *lchan,
				    enum dma_slave_buswidth width)
{
	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return LOONGSON2_MDMA_BYTE;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return LOONGSON2_MDMA_HALF_WORD;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		return LOONGSON2_MDMA_WORD;
	default:
		dev_err(chan2dev(lchan), "Dma bus width not supported\n");
		return -EINVAL;
	}
}

static struct loongson2_mdma_desc *loongson2_mdma_alloc_desc(u32 num_sgs)
{
	return kzalloc(sizeof(struct loongson2_mdma_desc) +
		       sizeof(struct loongson2_mdma_sg_req) * num_sgs, GFP_NOWAIT);
}

static int loongson2_mdma_slave_config(struct dma_chan *chan, struct dma_slave_config *config)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);

	memcpy(&lchan->dma_sconfig, config, sizeof(*config));

	return 0;
}

static void loongson2_mdma_irq_clear(struct loongson2_mdma_chan *lchan, u32 flags)
{
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	u32 dma_ifcr;

	dma_ifcr = flags << (4 * lchan->id);
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_IFCR, dma_ifcr);
}

static void loongson2_mdma_stop(struct loongson2_mdma_chan *lchan)
{
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	u32 dma_ccr;

	dma_ccr = loongson2_mdma_read(lddev, LOONGSON2_MDMA_CCR(lchan->id));
	dma_ccr &= ~(LOONGSON2_MDMA_CCR_IRQ_MASK | LOONGSON2_MDMA_CCR_EN);
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CCR(lchan->id), dma_ccr);

	loongson2_mdma_irq_clear(lchan, LOONGSON2_MDMA_MASKI);
}

static int loongson2_mdma_terminate_all(struct dma_chan *chan)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);
	unsigned long flags;

	LIST_HEAD(head);

	spin_lock_irqsave(&lchan->vchan.lock, flags);
	if (lchan->desc) {
		vchan_terminate_vdesc(&lchan->desc->vdesc);
		loongson2_mdma_stop(lchan);
		lchan->desc = NULL;
	}
	vchan_get_all_descriptors(&lchan->vchan, &head);
	spin_unlock_irqrestore(&lchan->vchan.lock, flags);

	vchan_dma_desc_free_list(&lchan->vchan, &head);

	return 0;
}

static void loongson2_mdma_synchronize(struct dma_chan *chan)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);

	vchan_synchronize(&lchan->vchan);
}

static void loongson2_mdma_start_transfer(struct loongson2_mdma_chan *lchan)
{
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	struct loongson2_mdma_sg_req *sg_req;
	struct loongson2_mdma_chan_reg *reg;
	struct virt_dma_desc *vdesc;

	loongson2_mdma_stop(lchan);

	if (!lchan->desc) {
		vdesc = vchan_next_desc(&lchan->vchan);
		if (!vdesc)
			return;

		list_del(&vdesc->node);
		lchan->desc = to_lmdma_desc(vdesc);
		lchan->next_sg = 0;
	}

	if (lchan->next_sg == lchan->desc->num_sgs)
		lchan->next_sg = 0;

	sg_req = &lchan->desc->sg_req[lchan->next_sg];
	reg = &sg_req->chan_reg;

	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CCR(lchan->id), reg->dma_ccr);
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CNDTR(lchan->id), reg->dma_cndtr);
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CPAR(lchan->id), reg->dma_cpar);
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CMAR(lchan->id), reg->dma_cmar);

	lchan->next_sg++;

	/* Start DMA */
	reg->dma_ccr |= LOONGSON2_MDMA_CCR_EN;
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CCR(lchan->id), reg->dma_ccr);
}

static void loongson2_mdma_configure_next_sg(struct loongson2_mdma_chan *lchan)
{
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	struct loongson2_mdma_sg_req *sg_req;
	u32 dma_cmar, dma_ccr;
	u32 id = lchan->id;

	if (lchan->next_sg == lchan->desc->num_sgs)
		lchan->next_sg = 0;

	/* stop to update mem addr */
	dma_ccr = loongson2_mdma_read(lddev, LOONGSON2_MDMA_CCR(id));
	dma_ccr &= ~LOONGSON2_MDMA_CCR_EN;
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CCR(id), dma_ccr);

	sg_req = &lchan->desc->sg_req[lchan->next_sg];
	dma_cmar = sg_req->chan_reg.dma_cmar;
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CMAR(id), dma_cmar);

	/* start transition */
	dma_ccr |= LOONGSON2_MDMA_CCR_EN;
	loongson2_mdma_write(lddev, LOONGSON2_MDMA_CCR(id), dma_ccr);
}

static void loongson2_mdma_handle_chan_done(struct loongson2_mdma_chan *lchan)
{
	if (!lchan->desc)
		return;

	if (lchan->desc->cyclic) {
		vchan_cyclic_callback(&lchan->desc->vdesc);
		/* LOONGSON2_MDMA_CCR_CIRC mode don't need update register */
		if (lchan->desc->num_sgs == 1)
			return;
		loongson2_mdma_configure_next_sg(lchan);
		lchan->next_sg++;
	} else {
		if (lchan->next_sg == lchan->desc->num_sgs) {
			vchan_cookie_complete(&lchan->desc->vdesc);
			lchan->desc = NULL;
		}
		loongson2_mdma_start_transfer(lchan);
	}
}

static irqreturn_t loongson2_mdma_chan_irq(int irq, void *devid)
{
	struct loongson2_mdma_chan *lchan = devid;
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	u32 ists, status, scr;

	spin_lock(&lchan->vchan.lock);

	ists = loongson2_mdma_read(lddev, LOONGSON2_MDMA_ISR);
	scr = loongson2_mdma_read(lddev, LOONGSON2_MDMA_CCR(lchan->id));

	status = (ists >> (4 * lchan->id)) & LOONGSON2_MDMA_MASKI;
	status &= scr;

	if (status & LOONGSON2_MDMA_TCI)
		loongson2_mdma_handle_chan_done(lchan);

	if (status & LOONGSON2_MDMA_HTI)
		loongson2_mdma_irq_clear(lchan, LOONGSON2_MDMA_HTI);

	if (status & LOONGSON2_MDMA_TEI)
		dev_err(chan2dev(lchan), "DMA Transform Error\n");

	loongson2_mdma_irq_clear(lchan, status);

	spin_unlock(&lchan->vchan.lock);

	return IRQ_HANDLED;
}

static void loongson2_mdma_issue_pending(struct dma_chan *chan)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&lchan->vchan.lock, flags);
	if (vchan_issue_pending(&lchan->vchan) && !lchan->desc) {
		dev_dbg(chan2dev(lchan), "vchan %pK: issued\n", &lchan->vchan);
		loongson2_mdma_start_transfer(lchan);

	}
	spin_unlock_irqrestore(&lchan->vchan.lock, flags);
}

static int loongson2_mdma_set_xfer_param(struct loongson2_mdma_chan *lchan,
					 enum dma_transfer_direction direction,
					 enum dma_slave_buswidth *buswidth, u32 buf_len)
{
	struct dma_slave_config	sconfig = lchan->dma_sconfig;
	int dev_width;
	u32 ccr;

	switch (direction) {
	case DMA_MEM_TO_DEV:
		dev_width = loongson2_mdma_get_width(lchan, sconfig.dst_addr_width);
		if (dev_width < 0)
			return dev_width;
		lchan->chan_reg.dma_cpar = sconfig.dst_addr;
		ccr = LOONGSON2_MDMA_CCR_DIR;
		*buswidth = sconfig.dst_addr_width;
		break;
	case DMA_DEV_TO_MEM:
		dev_width = loongson2_mdma_get_width(lchan, sconfig.src_addr_width);
		if (dev_width < 0)
			return dev_width;
		lchan->chan_reg.dma_cpar = sconfig.src_addr;
		ccr = LOONGSON2_MDMA_CCR_MINC;
		*buswidth = sconfig.src_addr_width;
		break;
	default:
		return -EINVAL;
	}

	ccr |= FIELD_PREP(LOONGSON2_MDMA_CCR_PSIZE_MASK, dev_width) |
	       FIELD_PREP(LOONGSON2_MDMA_CCR_MSIZE_MASK, dev_width);

	/* Set DMA control register */
	lchan->chan_reg.dma_ccr &= ~(LOONGSON2_MDMA_CCR_PSIZE_MASK | LOONGSON2_MDMA_CCR_MSIZE_MASK);
	lchan->chan_reg.dma_ccr |= ccr;

	return 0;
}

static struct dma_async_tx_descriptor *
loongson2_mdma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl, u32 sg_len,
			     enum dma_transfer_direction direction,
			     unsigned long flags, void *context)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);
	enum dma_slave_buswidth buswidth;
	struct loongson2_mdma_desc *desc;
	struct scatterlist *sg;
	u32 num_items;
	int i, ret;

	desc = loongson2_mdma_alloc_desc(sg_len);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		ret = loongson2_mdma_set_xfer_param(lchan, direction, &buswidth, sg_dma_len(sg));
		if (ret)
			return NULL;

		desc->sg_req[i].len = sg_dma_len(sg);

		num_items = desc->sg_req[i].len / buswidth;
		if (num_items >= SZ_64K) {
			dev_err(chan2dev(lchan), "Number of items not supported\n");
			kfree(desc);
			return NULL;
		}
		desc->sg_req[i].chan_reg.dma_ccr = lchan->chan_reg.dma_ccr;
		desc->sg_req[i].chan_reg.dma_cpar = lchan->chan_reg.dma_cpar;
		desc->sg_req[i].chan_reg.dma_cmar = sg_dma_address(sg);
		desc->sg_req[i].chan_reg.dma_cndtr = num_items;
	}

	desc->num_sgs = sg_len;
	desc->cyclic = false;

	return vchan_tx_prep(&lchan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *
loongson2_mdma_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
			       size_t period_len, enum dma_transfer_direction direction,
			       unsigned long flags)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);
	struct loongson2_mdma_desc *desc;
	enum dma_slave_buswidth buswidth;
	u32 num_periods, num_items;
	int i, ret;

	if (unlikely(buf_len % period_len))
		return NULL;

	ret = loongson2_mdma_set_xfer_param(lchan, direction, &buswidth, period_len);
	if (ret)
		return NULL;

	num_items = period_len / buswidth;
	if (num_items >= SZ_64K) {
		dev_err(chan2dev(lchan), "Number of items not supported\n");
		return NULL;
	}

	/* Enable Circular mode */
	if (buf_len == period_len)
		lchan->chan_reg.dma_ccr |= LOONGSON2_MDMA_CCR_CIRC;

	num_periods = buf_len / period_len;
	desc = loongson2_mdma_alloc_desc(num_periods);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		desc->sg_req[i].len = period_len;
		desc->sg_req[i].chan_reg.dma_ccr = lchan->chan_reg.dma_ccr;
		desc->sg_req[i].chan_reg.dma_cpar = lchan->chan_reg.dma_cpar;
		desc->sg_req[i].chan_reg.dma_cmar = buf_addr;
		desc->sg_req[i].chan_reg.dma_cndtr = num_items;
		buf_addr += period_len;
	}

	desc->num_sgs = num_periods;
	desc->cyclic = true;

	return vchan_tx_prep(&lchan->vchan, &desc->vdesc, flags);
}

static size_t loongson2_mdma_desc_residue(struct loongson2_mdma_chan *lchan,
					  struct loongson2_mdma_desc *desc, u32 next_sg)
{
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	u32 residue, width, ndtr, ccr;
	int i;

	ccr = loongson2_mdma_read(lddev, LOONGSON2_MDMA_CCR(lchan->id));
	width = FIELD_GET(LOONGSON2_MDMA_CCR_PSIZE_MASK, ccr);

	ndtr = loongson2_mdma_read(lddev, LOONGSON2_MDMA_CNDTR(lchan->id));
	residue = ndtr << width;

	if (lchan->desc->cyclic && next_sg == 0)
		return residue;

	for (i = next_sg; i < desc->num_sgs; i++)
		residue += desc->sg_req[i].len;

	return residue;
}

static enum dma_status loongson2_mdma_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
						struct dma_tx_state *state)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(chan, cookie, state);
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&lchan->vchan.lock, flags);
	vdesc = vchan_find_desc(&lchan->vchan, cookie);
	if (lchan->desc && cookie == lchan->desc->vdesc.tx.cookie)
		state->residue = loongson2_mdma_desc_residue(lchan, lchan->desc, lchan->next_sg);
	else if (vdesc)
		state->residue = loongson2_mdma_desc_residue(lchan, to_lmdma_desc(vdesc), 0);

	spin_unlock_irqrestore(&lchan->vchan.lock, flags);

	return status;
}

static int loongson2_mdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);

	loongson2_mdma_stop(lchan);
	return 0;
}

static void loongson2_mdma_free_chan_resources(struct dma_chan *chan)
{
	vchan_free_chan_resources(to_virt_chan(chan));
}

static void loongson2_mdma_desc_free(struct virt_dma_desc *vdesc)
{
	struct loongson2_mdma_desc *desc = container_of(vdesc, struct loongson2_mdma_desc, vdesc);

	kfree(desc);
}

static void loongson2_mdma_set_config(struct loongson2_mdma_chan *lchan,
				      struct loongson2_mdma_cfg *cfg)
{
	memset(&lchan->chan_reg, 0, sizeof(struct loongson2_mdma_chan_reg));

	lchan->chan_reg.dma_ccr = cfg->stream_config & LOONGSON2_MDMA_STREAM_MASK;
}

static bool loongson2_mdma_acpi_filter(struct dma_chan *chan, void *param)
{
	struct loongson2_mdma_chan *lchan = to_lmdma_chan(chan);
	struct loongson2_mdma_dev *lddev = lmdma_get_dev(lchan);
	struct loongson2_mdma_cfg *cfg = param;

	if (cfg->channel_id >= lddev->nr_channels) {
		dev_err(chan2dev(lchan), "Invalid channel id\n");
		return false;
	}

	loongson2_mdma_set_config(lchan, cfg);

	return true;
}

static struct dma_chan *loongson2_mdma_apci_xlate(struct acpi_dma_spec *dma_spec,
						  struct acpi_dma *adma)
{
	struct loongson2_mdma_dev *lddev = adma->data;
	struct loongson2_mdma_cfg cfg;

	cfg.channel_id = dma_spec->slave_id;
	cfg.stream_config = dma_spec->chan_id;

	return dma_request_channel(lddev->ddev.cap_mask, loongson2_mdma_acpi_filter, &cfg);
}

static struct dma_chan *loongson2_mdma_of_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	struct loongson2_mdma_dev *lddev = ofdma->of_dma_data;
	struct device *dev = lddev->ddev.dev;
	struct loongson2_mdma_cfg cfg;
	struct loongson2_mdma_chan *lchan;
	struct dma_chan *chan;

	if (dma_spec->args_count < 2)
		return NULL;

	cfg.channel_id = dma_spec->args[0];
	cfg.stream_config = dma_spec->args[1];

	if (cfg.channel_id >= lddev->nr_channels) {
		dev_err(dev, "Invalid channel id\n");
		return NULL;
	}

	lchan = &lddev->chan[cfg.channel_id];
	chan = dma_get_slave_channel(&lchan->vchan.chan);
	if (!chan) {
		dev_err(dev, "No more channels available\n");
		return NULL;
	}

	loongson2_mdma_set_config(lchan, &cfg);

	return chan;
}

static int loongson2_mdma_probe(struct platform_device *pdev)
{
	struct loongson2_mdma_chan *lchan;
	struct loongson2_mdma_dev *lddev;
	struct device *dev = &pdev->dev;
	struct dma_device *ddev;
	int nr_chans, i, ret;

	ret = device_property_read_u32(dev, "dma-channels", &nr_chans);
	if (ret)
		//return dev_err_probe(dev, -EINVAL, "missing or invalid dma-channels property\n");
		dev_warn(dev, "missing or invalid dma-channels property\n");

	nr_chans = 4;

	lddev = devm_kzalloc(dev, struct_size(lddev, chan, nr_chans), GFP_KERNEL);
	if (!lddev)
		return -ENOMEM;

	lddev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lddev->base))
		return PTR_ERR(lddev->base);

	platform_set_drvdata(pdev, lddev);
	lddev->nr_channels = nr_chans;

	lddev->dma_clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(lddev->dma_clk))
		return dev_err_probe(dev, PTR_ERR(lddev->dma_clk), "Failed to get dma clock\n");

	ddev = &lddev->ddev;
	ddev->dev = dev;

	dma_cap_zero(ddev->cap_mask);
	dma_cap_set(DMA_SLAVE, ddev->cap_mask);
	dma_cap_set(DMA_PRIVATE, ddev->cap_mask);
	dma_cap_set(DMA_CYCLIC, ddev->cap_mask);

	ddev->device_alloc_chan_resources = loongson2_mdma_alloc_chan_resources;
	ddev->device_config = loongson2_mdma_slave_config;
	ddev->device_prep_slave_sg = loongson2_mdma_prep_slave_sg;
	ddev->device_prep_dma_cyclic = loongson2_mdma_prep_dma_cyclic;
	ddev->device_issue_pending = loongson2_mdma_issue_pending;
	ddev->device_synchronize = loongson2_mdma_synchronize;
	ddev->device_tx_status = loongson2_mdma_tx_status;
	ddev->device_terminate_all = loongson2_mdma_terminate_all;
	ddev->device_free_chan_resources = loongson2_mdma_free_chan_resources;

	ddev->src_addr_widths = LOONGSON2_MDMA_BUSWIDTHS;
	ddev->dst_addr_widths = LOONGSON2_MDMA_BUSWIDTHS;
	ddev->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	INIT_LIST_HEAD(&ddev->channels);

	for (i = 0; i < nr_chans; i++) {
		lchan = &lddev->chan[i];

		lchan->id = i;
		lchan->vchan.desc_free = loongson2_mdma_desc_free;
		vchan_init(&lchan->vchan, ddev);
	}

	ret = dma_async_device_register(ddev);
	if (ret)
		return ret;

	for (i = 0; i < nr_chans; i++) {
		lchan = &lddev->chan[i];

		lchan->irq = platform_get_irq(pdev, i);
		if (lchan->irq < 0) {
			ret = -EINVAL;
			goto unregister_dmac;
		}

		ret = devm_request_irq(dev, lchan->irq, loongson2_mdma_chan_irq, IRQF_SHARED,
				       dev_name(chan2dev(lchan)), lchan);
		if (ret)
			goto unregister_dmac;
	}

	if (has_acpi_companion(dev))
		ret = devm_acpi_dma_controller_register(dev, loongson2_mdma_apci_xlate, lddev);
	else
		ret = of_dma_controller_register(dev->of_node, loongson2_mdma_of_xlate, lddev);
	if (ret)
		goto unregister_dmac;

	dev_info(dev, "Loongson-2 Multi-Channel DMA Controller driver registered successfully.\n");
	return 0;

unregister_dmac:
	dma_async_device_unregister(ddev);

	return ret;
}

static void loongson2_mdma_remove(struct platform_device *pdev)
{
	struct loongson2_mdma_dev *lddev = platform_get_drvdata(pdev);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&lddev->ddev);
}

static const struct of_device_id loongson2_mdma_of_match[] = {
	{ .compatible = "loongson,ls2k0300-dma" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, loongson2_mdma_of_match);

static const struct acpi_device_id loongson2_mdma_acpi_match[] = {
	{ "LOON0013" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, loongson2_mdma_acpi_match);

static struct platform_driver loongson2_mdma_driver = {
	.driver = {
		.name = "loongson2-mdma",
		.of_match_table = loongson2_mdma_of_match,
		.acpi_match_table = loongson2_mdma_acpi_match,
	},
	.probe = loongson2_mdma_probe,
	.remove = loongson2_mdma_remove,
};

module_platform_driver(loongson2_mdma_driver);

MODULE_DESCRIPTION("Looongson-2 Multi-Channel DMA Controller driver");
MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_LICENSE("GPL");
