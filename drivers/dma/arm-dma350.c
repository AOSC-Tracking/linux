// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2024-2025 Arm Limited
// Arm DMA-350 driver

#include <linux/acpi.h>
#include <linux/acpi_dma.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_reserved_mem.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "dmaengine.h"
#include "virt-dma.h"

#define DMAINFO			0x0f00

#define DMA_BUILDCFG0		0xb0
#define DMA_CFG_DATA_WIDTH	GENMASK(18, 16)
#define DMA_CFG_ADDR_WIDTH	GENMASK(15, 10)
#define DMA_CFG_NUM_CHANNELS	GENMASK(9, 4)

#define DMA_BUILDCFG1		0xb4
#define DMA_CFG_NUM_TRIGGER_IN	GENMASK(8, 0)

#define IIDR			0xc8
#define IIDR_PRODUCTID		GENMASK(31, 20)
#define IIDR_VARIANT		GENMASK(19, 16)
#define IIDR_REVISION		GENMASK(15, 12)
#define IIDR_IMPLEMENTER	GENMASK(11, 0)

#define PRODUCTID_DMA350	0x3a0
#define IMPLEMENTER_ARM		0x43b

#define DMACH(n)		(0x1000 + 0x0100 * (n))

#define CH_CMD			0x00
#define CH_CMD_RESUME		BIT(5)
#define CH_CMD_PAUSE		BIT(4)
#define CH_CMD_STOP		BIT(3)
#define CH_CMD_DISABLE		BIT(2)
#define CH_CMD_CLEAR		BIT(1)
#define CH_CMD_ENABLE		BIT(0)

#define CH_STATUS		0x04
#define CH_STAT_RESUMEWAIT	BIT(21)
#define CH_STAT_PAUSED		BIT(20)
#define CH_STAT_STOPPED		BIT(19)
#define CH_STAT_DISABLED	BIT(18)
#define CH_STAT_ERR		BIT(17)
#define CH_STAT_DONE		BIT(16)
#define CH_STAT_INTR_ERR	BIT(1)
#define CH_STAT_INTR_DONE	BIT(0)
#define CH_STAT_INTR_MASK	GENMASK(10, 0)	/* All interrupt flag bits */

#define CH_INTREN		0x08
#define CH_INTREN_ERR		BIT(1)
#define CH_INTREN_DONE		BIT(0)

#define CH_CTRL			0x0c
#define CH_CTRL_USEDESTRIGIN	BIT(26)
#define CH_CTRL_USESRCTRIGIN	BIT(26)
#define CH_CTRL_DONETYPE	GENMASK(23, 21)
#define CH_CTRL_REGRELOADTYPE	GENMASK(20, 18)
#define CH_CTRL_XTYPE		GENMASK(11, 9)
#define CH_CTRL_TRANSIZE	GENMASK(2, 0)

#define CH_SRCADDR		0x10
#define CH_SRCADDRHI		0x14
#define CH_DESADDR		0x18
#define CH_DESADDRHI		0x1c
#define CH_XSIZE		0x20
#define CH_XSIZEHI		0x24
#define CH_SRCTRANSCFG		0x28
#define CH_DESTRANSCFG		0x2c
#define CH_CFG_MAXBURSTLEN	GENMASK(19, 16)
#define CH_CFG_PRIVATTR		BIT(11)
#define CH_CFG_SHAREATTR	GENMASK(9, 8)
#define CH_CFG_MEMATTR		GENMASK(7, 0)

#define TRANSCFG_DEVICE					\
	FIELD_PREP(CH_CFG_MAXBURSTLEN, 0xf) |		\
	FIELD_PREP(CH_CFG_SHAREATTR, SHAREATTR_OSH) |	\
	FIELD_PREP(CH_CFG_MEMATTR, MEMATTR_DEVICE)
#define TRANSCFG_NC					\
	FIELD_PREP(CH_CFG_MAXBURSTLEN, 0xf) |		\
	FIELD_PREP(CH_CFG_SHAREATTR, SHAREATTR_OSH) |	\
	FIELD_PREP(CH_CFG_MEMATTR, MEMATTR_NC)
#define TRANSCFG_WB					\
	FIELD_PREP(CH_CFG_MAXBURSTLEN, 0xf) |		\
	FIELD_PREP(CH_CFG_SHAREATTR, SHAREATTR_ISH) |	\
	FIELD_PREP(CH_CFG_MEMATTR, MEMATTR_WB)

#define CH_XADDRINC		0x30
#define CH_XY_DES		GENMASK(31, 16)
#define CH_XY_SRC		GENMASK(15, 0)

#define CH_FILLVAL		0x38
#define CH_SRCTRIGINCFG		0x4c
#define CH_DESTRIGINCFG		0x50
#define CH_LINKATTR		0x70
#define CH_LINK_SHAREATTR	GENMASK(9, 8)
#define CH_LINK_MEMATTR		GENMASK(7, 0)

#define CH_AUTOCFG		0x74
#define CH_LINKADDR		0x78
#define CH_LINKADDR_EN		BIT(0)

#define CH_LINKADDRHI		0x7c
#define CH_ERRINFO		0x90

/*
 * Trigger configuration register fields (for peripheral DMA)
 * Same layout for both SRCTRIGINCFG and DESTRIGINCFG
 */
#define CH_TRIGINCFG_SEL	GENMASK(7, 0)	/* Request line select */
#define CH_TRIGINCFG_TYPE	GENMASK(9, 8)	/* Trigger type */
#define CH_TRIGINCFG_MODE	GENMASK(11, 10)	/* Trigger mode */
#define CH_TRIGINCFG_BLKSIZE	GENMASK(23, 16)	/* Block size */

/* Trigger type values */
#define TRIGIN_TYPE_SW		0	/* Software only */
#define TRIGIN_TYPE_HW		2	/* Hardware trigger */
#define TRIGIN_TYPE_INTERNAL	3	/* Internal trigger */

/* Trigger mode values */
#define TRIGIN_MODE_CMD			0	/* CMD mode */
#define TRIGIN_MODE_DMA_FLOW		2	/* DMA flow control */
#define TRIGIN_MODE_PERIPH_FLOW		3	/* Peripheral flow control */
#define CH_ERRINFO_AXIRDPOISERR BIT(18)
#define CH_ERRINFO_AXIWRRESPERR BIT(17)
#define CH_ERRINFO_AXIRDRESPERR BIT(16)

#define CH_BUILDCFG0		0xf8
#define CH_CFG_INC_WIDTH	GENMASK(29, 26)
#define CH_CFG_DATA_WIDTH	GENMASK(24, 22)
#define CH_CFG_DATA_BUF_SIZE	GENMASK(7, 0)

#define CH_BUILDCFG1		0xfc
#define CH_CFG_HAS_CMDLINK	BIT(8)
#define CH_CFG_HAS_TRIGSEL	BIT(7)
#define CH_CFG_HAS_TRIGIN	BIT(5)
#define CH_CFG_HAS_WRAP		BIT(1)

/* Global interrupt control registers */
#define DMA_CHINTRSTATUS0	0x200	/* Collated channel interrupt flags */
#define DMA_NSEC_INTREN		0x20c	/* Non-secure interrupt enable */
#define DMA_NSEC_INTREN_ANY	BIT(0)	/* Enable any channel interrupt */

/* AUDSS CRU register for interrupt routing to AP (Sky1 specific) */
#define AUDSS_CRU_AP_IRQ	0x54	/* Bit N enables channel N IRQ to AP */

#define LINK_REGCLEAR		BIT(0)
#define LINK_INTREN		BIT(2)
#define LINK_CTRL		BIT(3)
#define LINK_SRCADDR		BIT(4)
#define LINK_SRCADDRHI		BIT(5)
#define LINK_DESADDR		BIT(6)
#define LINK_DESADDRHI		BIT(7)
#define LINK_XSIZE		BIT(8)
#define LINK_XSIZEHI		BIT(9)
#define LINK_SRCTRANSCFG	BIT(10)
#define LINK_DESTRANSCFG	BIT(11)
#define LINK_XADDRINC		BIT(12)
#define LINK_FILLVAL		BIT(14)
#define LINK_SRCTRIGINCFG	BIT(19)
#define LINK_DESTRIGINCFG	BIT(20)
#define LINK_AUTOCFG		BIT(29)
#define LINK_LINKADDR		BIT(30)
#define LINK_LINKADDRHI		BIT(31)


enum ch_ctrl_donetype {
	CH_CTRL_DONETYPE_NONE = 0,
	CH_CTRL_DONETYPE_CMD = 1,
	CH_CTRL_DONETYPE_CYCLE = 3
};

enum ch_ctrl_xtype {
	CH_CTRL_XTYPE_DISABLE = 0,
	CH_CTRL_XTYPE_CONTINUE = 1,
	CH_CTRL_XTYPE_WRAP = 2,
	CH_CTRL_XTYPE_FILL = 3
};

enum ch_cfg_shareattr {
	SHAREATTR_NSH = 0,
	SHAREATTR_OSH = 2,
	SHAREATTR_ISH = 3
};

enum ch_cfg_memattr {
	MEMATTR_DEVICE = 0x00,
	MEMATTR_NC = 0x44,
	MEMATTR_WB = 0xff
};

struct d350_desc {
	struct virt_dma_desc vd;
	u32 command[16];
	u16 xsize;
	u16 xsizehi;
	u8 tsz;

	/* Transfer type flags */
	bool cyclic;
	bool scatter_gather;

	/* Cyclic transfer state */
	size_t num_periods;
	size_t period_len;

	/* Scatter-gather state */
	unsigned int sg_len;
	size_t total_len;

	/* Common fields */
	enum dma_transfer_direction dir;

	/* DMA-coherent command link array (cyclic and scatter-gather) */
	u32 *cmdlink_virt;
	dma_addr_t cmdlink_phys;
	size_t cmdlink_size;
};

struct d350_chan {
	struct virt_dma_chan vc;
	struct d350_desc *desc;
	void __iomem *base;
	int irq;
	enum dma_status status;
	dma_cookie_t cookie;
	u32 residue;
	u8 tsz;
	bool has_trig;
	bool has_wrap;
	bool coherent;
	/* Slave DMA configuration */
	struct dma_slave_config slave_cfg;
	u32 req_line;		/* DMA request/trigger line */
	dma_addr_t dev_addr;	/* Peripheral address */
	enum dma_transfer_direction dir;
	/* Cyclic state tracking */
	size_t period_idx;	/* Current period for residue calc */
};

struct d350 {
	struct dma_device dma;
	struct clk *clk;
	struct reset_control *rst;
	void __iomem *base;

	/* Address translation (optional, parsed from DT) */
	struct {
		u64 reg_cpu;	/* CPU address of peripheral registers */
		u64 reg_dma;	/* DMA view of peripheral registers */
		u64 ram_cpu;	/* CPU address of RAM base */
		u64 ram_dma;	/* DMA view of RAM base */
	} addr_xlat;

	/* Remote control regmap (e.g., AUDSS CRU for interrupt routing) */
	struct regmap *remote_ctrl;

	int nchan;
	int nreq;
	struct d350_chan channels[] __counted_by(nchan);
};

static inline struct d350_chan *to_d350_chan(struct dma_chan *chan)
{
	return container_of(chan, struct d350_chan, vc.chan);
}

static inline struct d350_desc *to_d350_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct d350_desc, vd);
}

static inline struct d350 *to_d350(struct dma_device *dma)
{
	return container_of(dma, struct d350, dma);
}

/* Translate CPU address to DMA-350 view */
static inline dma_addr_t d350_translate_addr(struct d350 *dmac, dma_addr_t addr)
{
	/* Translate RAM addresses (e.g., 0xc0000000+ -> 0x30000000+) */
	if (dmac->addr_xlat.ram_cpu && addr >= dmac->addr_xlat.ram_cpu)
		return addr - dmac->addr_xlat.ram_cpu + dmac->addr_xlat.ram_dma;

	/* Translate register/peripheral addresses using base offset */
	if (dmac->addr_xlat.reg_cpu && dmac->addr_xlat.reg_dma)
		return addr - dmac->addr_xlat.reg_cpu + dmac->addr_xlat.reg_dma;

	return addr;
}

/* Build trigger configuration word for peripheral DMA */
static inline u32 d350_build_trigincfg(u32 req_line)
{
	return FIELD_PREP(CH_TRIGINCFG_SEL, req_line) |
	       FIELD_PREP(CH_TRIGINCFG_TYPE, TRIGIN_TYPE_HW) |
	       FIELD_PREP(CH_TRIGINCFG_MODE, TRIGIN_MODE_PERIPH_FLOW);
}

/*
 * Command link word count per transfer entry.
 * Fields: header, intren, ctrl, srcaddr, srcaddrhi, desaddr, desaddrhi,
 *         xsize, xsizehi, srctranscfg, destranscfg, xaddrinc, trigincfg,
 *         linkaddr, linkaddrhi, padding
 */
#define CMDLINK_WORDS_PER_ENTRY		16

/**
 * d350_build_slave_command - Build a slave DMA command link entry
 * @dmac: DMA controller
 * @dch: DMA channel
 * @cmd: Command buffer (CMDLINK_WORDS_PER_ENTRY words)
 * @mem_addr: Memory address (CPU view, will be translated)
 * @dev_addr: Device address (already translated)
 * @len: Transfer length in bytes
 * @transize: Transfer size (0=byte, 1=halfword, 2=word, 3=dword)
 * @dir: Transfer direction
 * @enable_irq: Enable completion interrupt for this command
 * @next_cmd_phys: Physical address of next command (0 = no link, stop after)
 * @first_cmd: True for first command in chain (clears registers)
 */
static void d350_build_slave_command(struct d350 *dmac, struct d350_chan *dch,
				     u32 *cmd, dma_addr_t mem_addr,
				     dma_addr_t dev_addr, size_t len,
				     u8 transize, enum dma_transfer_direction dir,
				     bool enable_irq, dma_addr_t next_cmd_phys,
				     bool first_cmd)
{
	dma_addr_t src_addr, dst_addr;
	u32 header, ctrl, xaddrinc, trigincfg;
	u32 srctranscfg, destranscfg;
	u32 xsize;
	int idx;

	/* Set addresses based on direction */
	if (dir == DMA_MEM_TO_DEV) {
		src_addr = d350_translate_addr(dmac, mem_addr);
		dst_addr = dev_addr;
	} else {
		src_addr = dev_addr;
		dst_addr = d350_translate_addr(dmac, mem_addr);
	}

	/* Transfer count in units of transize */
	xsize = len >> transize;

	/*
	 * First command: Full configuration with REGCLEAR to initialize all
	 * DMA registers. Includes CTRL, INTREN, TRANSCFG, XADDRINC, TRIGINCFG.
	 *
	 * Subsequent commands: Minimal format with only addresses, size, and
	 * link. Other registers are inherited from the first command, ensuring
	 * seamless transitions without glitches at command boundaries.
	 */
	if (first_cmd) {
		/* Set direction-specific attributes */
		if (dir == DMA_MEM_TO_DEV) {
			xaddrinc = FIELD_PREP(CH_XY_SRC, 1) | FIELD_PREP(CH_XY_DES, 0);
			srctranscfg = TRANSCFG_NC;
			destranscfg = TRANSCFG_DEVICE;
		} else {
			xaddrinc = FIELD_PREP(CH_XY_SRC, 0) | FIELD_PREP(CH_XY_DES, 1);
			srctranscfg = TRANSCFG_DEVICE;
			destranscfg = TRANSCFG_NC;
		}

		/* Full header with all configuration fields */
		header = LINK_REGCLEAR | LINK_INTREN | LINK_CTRL |
			 LINK_SRCADDR | LINK_SRCADDRHI |
			 LINK_DESADDR | LINK_DESADDRHI |
			 LINK_XSIZE | LINK_XSIZEHI |
			 LINK_SRCTRANSCFG | LINK_DESTRANSCFG |
			 LINK_XADDRINC |
			 LINK_LINKADDR | LINK_LINKADDRHI;

		/* Add trigger config based on direction */
		if (dir == DMA_MEM_TO_DEV)
			header |= LINK_DESTRIGINCFG;
		else
			header |= LINK_SRCTRIGINCFG;

		/* Build control word */
		ctrl = FIELD_PREP(CH_CTRL_TRANSIZE, transize) |
		       FIELD_PREP(CH_CTRL_XTYPE, CH_CTRL_XTYPE_CONTINUE) |
		       FIELD_PREP(CH_CTRL_DONETYPE,
				  next_cmd_phys ? CH_CTRL_DONETYPE_CMD : CH_CTRL_DONETYPE_NONE);

		/* Enable trigger in control */
		if (dir == DMA_MEM_TO_DEV)
			ctrl |= CH_CTRL_USEDESTRIGIN;
		else
			ctrl |= CH_CTRL_USESRCTRIGIN;

		/* Build trigger configuration */
		trigincfg = d350_build_trigincfg(dch->req_line);

		/* Write full command - fields in order of LINK_* bit positions */
		idx = 0;
		cmd[idx++] = header;
		cmd[idx++] = enable_irq ? (CH_INTREN_DONE | CH_INTREN_ERR) : 0;
		cmd[idx++] = ctrl;
		cmd[idx++] = lower_32_bits(src_addr);
		cmd[idx++] = upper_32_bits(src_addr);
		cmd[idx++] = lower_32_bits(dst_addr);
		cmd[idx++] = upper_32_bits(dst_addr);
		cmd[idx++] = FIELD_PREP(CH_XY_SRC, xsize) | FIELD_PREP(CH_XY_DES, xsize);
		cmd[idx++] = 0;  /* XSIZEHI - for transfers > 4GB */
		cmd[idx++] = srctranscfg;
		cmd[idx++] = destranscfg;
		cmd[idx++] = xaddrinc;
		cmd[idx++] = trigincfg;
		if (next_cmd_phys) {
			cmd[idx++] = lower_32_bits(next_cmd_phys) | CH_LINKADDR_EN;
			cmd[idx++] = upper_32_bits(next_cmd_phys);
		} else {
			cmd[idx++] = 0;
			cmd[idx++] = 0;
		}
	} else {
		/*
		 * Minimal continuation command - only addresses, size, and link.
		 * All other configuration inherited from first command.
		 */
		header = LINK_SRCADDR | LINK_SRCADDRHI |
			 LINK_DESADDR | LINK_DESADDRHI |
			 LINK_XSIZE | LINK_XSIZEHI |
			 LINK_LINKADDR | LINK_LINKADDRHI;

		/* Write minimal command - packed format */
		idx = 0;
		cmd[idx++] = header;
		cmd[idx++] = lower_32_bits(src_addr);
		cmd[idx++] = upper_32_bits(src_addr);
		cmd[idx++] = lower_32_bits(dst_addr);
		cmd[idx++] = upper_32_bits(dst_addr);
		cmd[idx++] = FIELD_PREP(CH_XY_SRC, xsize) | FIELD_PREP(CH_XY_DES, xsize);
		cmd[idx++] = 0;  /* XSIZEHI */
		if (next_cmd_phys) {
			cmd[idx++] = lower_32_bits(next_cmd_phys) | CH_LINKADDR_EN;
			cmd[idx++] = upper_32_bits(next_cmd_phys);
		} else {
			cmd[idx++] = 0;
			cmd[idx++] = 0;
		}
	}
}

static void d350_desc_free(struct virt_dma_desc *vd)
{
	struct d350_desc *desc = to_d350_desc(vd);
	struct dma_chan *chan = vd->tx.chan;
	struct device *dev = chan->device->dev;

	if (desc->cmdlink_virt) {
		dma_free_coherent(dev, desc->cmdlink_size,
				  desc->cmdlink_virt, desc->cmdlink_phys);
	}
	kfree(desc);
}

static struct dma_async_tx_descriptor *d350_prep_memcpy(struct dma_chan *chan,
		dma_addr_t dest, dma_addr_t src, size_t len, unsigned long flags)
{
	struct d350_chan *dch = to_d350_chan(chan);
	struct d350_desc *desc;
	u32 *cmd;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->tsz = __ffs(len | dest | src | (1 << dch->tsz));
	desc->xsize = lower_16_bits(len >> desc->tsz);
	desc->xsizehi = upper_16_bits(len >> desc->tsz);

	cmd = desc->command;
	cmd[0] = LINK_CTRL | LINK_SRCADDR | LINK_SRCADDRHI | LINK_DESADDR |
		 LINK_DESADDRHI | LINK_XSIZE | LINK_XSIZEHI | LINK_SRCTRANSCFG |
		 LINK_DESTRANSCFG | LINK_XADDRINC | LINK_LINKADDR;

	cmd[1] = FIELD_PREP(CH_CTRL_TRANSIZE, desc->tsz) |
		 FIELD_PREP(CH_CTRL_XTYPE, CH_CTRL_XTYPE_CONTINUE) |
		 FIELD_PREP(CH_CTRL_DONETYPE, CH_CTRL_DONETYPE_CMD);

	cmd[2] = lower_32_bits(src);
	cmd[3] = upper_32_bits(src);
	cmd[4] = lower_32_bits(dest);
	cmd[5] = upper_32_bits(dest);
	cmd[6] = FIELD_PREP(CH_XY_SRC, desc->xsize) | FIELD_PREP(CH_XY_DES, desc->xsize);
	cmd[7] = FIELD_PREP(CH_XY_SRC, desc->xsizehi) | FIELD_PREP(CH_XY_DES, desc->xsizehi);
	cmd[8] = dch->coherent ? TRANSCFG_WB : TRANSCFG_NC;
	cmd[9] = dch->coherent ? TRANSCFG_WB : TRANSCFG_NC;
	cmd[10] = FIELD_PREP(CH_XY_SRC, 1) | FIELD_PREP(CH_XY_DES, 1);
	cmd[11] = 0;

	return vchan_tx_prep(&dch->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *d350_prep_memset(struct dma_chan *chan,
		dma_addr_t dest, int value, size_t len, unsigned long flags)
{
	struct d350_chan *dch = to_d350_chan(chan);
	struct d350_desc *desc;
	u32 *cmd;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->tsz = __ffs(len | dest | (1 << dch->tsz));
	desc->xsize = lower_16_bits(len >> desc->tsz);
	desc->xsizehi = upper_16_bits(len >> desc->tsz);

	cmd = desc->command;
	cmd[0] = LINK_CTRL | LINK_DESADDR | LINK_DESADDRHI |
		 LINK_XSIZE | LINK_XSIZEHI | LINK_DESTRANSCFG |
		 LINK_XADDRINC | LINK_FILLVAL | LINK_LINKADDR;

	cmd[1] = FIELD_PREP(CH_CTRL_TRANSIZE, desc->tsz) |
		 FIELD_PREP(CH_CTRL_XTYPE, CH_CTRL_XTYPE_FILL) |
		 FIELD_PREP(CH_CTRL_DONETYPE, CH_CTRL_DONETYPE_CMD);

	cmd[2] = lower_32_bits(dest);
	cmd[3] = upper_32_bits(dest);
	cmd[4] = FIELD_PREP(CH_XY_DES, desc->xsize);
	cmd[5] = FIELD_PREP(CH_XY_DES, desc->xsizehi);
	cmd[6] = dch->coherent ? TRANSCFG_WB : TRANSCFG_NC;
	cmd[7] = FIELD_PREP(CH_XY_DES, 1);
	cmd[8] = (u8)value * 0x01010101;
	cmd[9] = 0;

	return vchan_tx_prep(&dch->vc, &desc->vd, flags);
}

static int d350_pause(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&dch->vc.lock, flags);
	if (dch->status == DMA_IN_PROGRESS) {
		writel_relaxed(CH_CMD_PAUSE, dch->base + CH_CMD);
		dch->status = DMA_PAUSED;
	}
	spin_unlock_irqrestore(&dch->vc.lock, flags);

	return 0;
}

static int d350_resume(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&dch->vc.lock, flags);
	if (dch->status == DMA_PAUSED) {
		writel_relaxed(CH_CMD_RESUME, dch->base + CH_CMD);
		dch->status = DMA_IN_PROGRESS;
	}
	spin_unlock_irqrestore(&dch->vc.lock, flags);

	return 0;
}

static u32 d350_get_residue(struct d350_chan *dch)
{
	u32 res, xsize, xsizehi, hi_new;
	int retries = 3; /* 1st time unlucky, 2nd improbable, 3rd just broken */

	hi_new = readl_relaxed(dch->base + CH_XSIZEHI);
	do {
		xsizehi = hi_new;
		xsize = readl_relaxed(dch->base + CH_XSIZE);
		hi_new = readl_relaxed(dch->base + CH_XSIZEHI);
	} while (xsizehi != hi_new && --retries);

	res = FIELD_GET(CH_XY_DES, xsize);
	res |= FIELD_GET(CH_XY_DES, xsizehi) << 16;

	return res << dch->desc->tsz;
}

static int d350_terminate_all(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);
	unsigned long flags;
	LIST_HEAD(list);

	spin_lock_irqsave(&dch->vc.lock, flags);

	/* Stop the channel */
	writel_relaxed(CH_CMD_STOP, dch->base + CH_CMD);

	/* Disable command linking for cyclic transfers */
	writel_relaxed(0, dch->base + CH_LINKADDR);
	writel_relaxed(0, dch->base + CH_LINKADDRHI);

	if (dch->desc) {
		if (dch->status != DMA_ERROR)
			vchan_terminate_vdesc(&dch->desc->vd);
		dch->desc = NULL;
		dch->status = DMA_COMPLETE;
	}
	dch->period_idx = 0;

	vchan_get_all_descriptors(&dch->vc, &list);
	list_splice_tail(&list, &dch->vc.desc_terminated);
	spin_unlock_irqrestore(&dch->vc.lock, flags);

	return 0;
}

static void d350_synchronize(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);

	vchan_synchronize(&dch->vc);
}

static u32 d350_desc_bytes(struct d350_desc *desc)
{
	return ((u32)desc->xsizehi << 16 | desc->xsize) << desc->tsz;
}

static enum dma_status d350_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				      struct dma_tx_state *state)
{
	struct d350_chan *dch = to_d350_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status status;
	unsigned long flags;
	u32 residue = 0;

	status = dma_cookie_status(chan, cookie, state);

	spin_lock_irqsave(&dch->vc.lock, flags);
	if (cookie == dch->cookie) {
		status = dch->status;
		if (dch->desc && dch->desc->cyclic) {
			/*
			 * For cyclic DMA, residue is the position within
			 * the buffer. We track this via period_idx updated
			 * in the IRQ handler.
			 */
			size_t total = dch->desc->num_periods * dch->desc->period_len;
			size_t completed = dch->period_idx * dch->desc->period_len;

			residue = total - completed;
			/* Cyclic is always in progress unless error */
			if (status != DMA_ERROR)
				status = DMA_IN_PROGRESS;
		} else {
			if (status == DMA_IN_PROGRESS || status == DMA_PAUSED)
				dch->residue = d350_get_residue(dch);
			residue = dch->residue;
		}
	} else if ((vd = vchan_find_desc(&dch->vc, cookie))) {
		struct d350_desc *desc = to_d350_desc(vd);

		if (desc->cyclic)
			residue = desc->num_periods * desc->period_len;
		else
			residue = d350_desc_bytes(desc);
	} else if (status == DMA_IN_PROGRESS) {
		/* Somebody else terminated it? */
		status = DMA_ERROR;
	}
	spin_unlock_irqrestore(&dch->vc.lock, flags);

	dma_set_residue(state, residue);
	return status;
}

static void d350_start_cyclic(struct d350_chan *dch)
{
	struct d350 *dmac = to_d350(dch->vc.chan.device);
	dma_addr_t cmdlink_dma;

	/*
	 * For cyclic DMA, we use hardware command linking.
	 * The DMA controller fetches commands from memory and
	 * automatically cycles through them.
	 */
	dch->period_idx = 0;

	/* Calculate initial residue as full buffer */
	dch->residue = dch->desc->num_periods * dch->desc->period_len;

	/* Clear channel state */
	writel_relaxed(CH_CMD_CLEAR, dch->base + CH_CMD);

	/* Set XSIZE to 0 - command link will provide the actual size */
	writel_relaxed(0, dch->base + CH_XSIZE);
	writel_relaxed(0, dch->base + CH_XSIZEHI);

	/*
	 * For cyclic DMA, disable register-based INTREN.
	 * Let the command link entries control interrupts.
	 * This matches vendor behavior and avoids potential issues
	 * with both register and command link enabling interrupts.
	 */
	writel_relaxed(0, dch->base + CH_INTREN);

	/* Enable channel first */
	writel(CH_CMD_ENABLE, dch->base + CH_CMD);

	/* Set up and enable hardware command linking */
	cmdlink_dma = d350_translate_addr(dmac, dch->desc->cmdlink_phys);

	writel_relaxed(lower_32_bits(cmdlink_dma) | CH_LINKADDR_EN,
		       dch->base + CH_LINKADDR);
	writel_relaxed(upper_32_bits(cmdlink_dma), dch->base + CH_LINKADDRHI);

	/* Re-enable to start command fetch */
	writel(CH_CMD_ENABLE, dch->base + CH_CMD);
}

static void d350_start_sg(struct d350_chan *dch)
{
	struct d350 *dmac = to_d350(dch->vc.chan.device);
	dma_addr_t cmdlink_dma;

	/*
	 * For scatter-gather DMA, we use hardware command linking
	 * similar to cyclic, but with linear linking (no loop back).
	 * The last command has no link address and stops automatically.
	 */
	dch->residue = dch->desc->total_len;

	/* Clear channel state */
	writel_relaxed(CH_CMD_CLEAR, dch->base + CH_CMD);

	/* Set XSIZE to 0 - command link will provide the actual size */
	writel_relaxed(0, dch->base + CH_XSIZE);
	writel_relaxed(0, dch->base + CH_XSIZEHI);

	/* Enable interrupts */
	writel_relaxed(CH_INTREN_DONE | CH_INTREN_ERR, dch->base + CH_INTREN);

	/* Enable channel first */
	writel(CH_CMD_ENABLE, dch->base + CH_CMD);

	/* Set up and enable hardware command linking */
	cmdlink_dma = d350_translate_addr(dmac, dch->desc->cmdlink_phys);

	writel_relaxed(lower_32_bits(cmdlink_dma) | CH_LINKADDR_EN,
		       dch->base + CH_LINKADDR);
	writel_relaxed(upper_32_bits(cmdlink_dma), dch->base + CH_LINKADDRHI);

	/* Re-enable to start command fetch */
	writel(CH_CMD_ENABLE, dch->base + CH_CMD);
}

static void d350_start_next(struct d350_chan *dch)
{
	u32 hdr, *reg;

	dch->desc = to_d350_desc(vchan_next_desc(&dch->vc));
	if (!dch->desc)
		return;

	dch->status = DMA_IN_PROGRESS;
	dch->cookie = dch->desc->vd.tx.cookie;

	/* Handle cyclic transfers specially */
	if (dch->desc->cyclic) {
		/* Don't remove from list - we need it for callbacks */
		d350_start_cyclic(dch);
		return;
	}

	/* Handle scatter-gather transfers with command linking */
	if (dch->desc->scatter_gather) {
		/* Don't remove from list until complete */
		d350_start_sg(dch);
		return;
	}

	/* Simple transfer: remove from list and program directly */
	list_del(&dch->desc->vd.node);
	dch->residue = d350_desc_bytes(dch->desc);

	hdr = dch->desc->command[0];
	reg = &dch->desc->command[1];

	if (hdr & LINK_INTREN)
		writel_relaxed(*reg++, dch->base + CH_INTREN);
	if (hdr & LINK_CTRL)
		writel_relaxed(*reg++, dch->base + CH_CTRL);
	if (hdr & LINK_SRCADDR)
		writel_relaxed(*reg++, dch->base + CH_SRCADDR);
	if (hdr & LINK_SRCADDRHI)
		writel_relaxed(*reg++, dch->base + CH_SRCADDRHI);
	if (hdr & LINK_DESADDR)
		writel_relaxed(*reg++, dch->base + CH_DESADDR);
	if (hdr & LINK_DESADDRHI)
		writel_relaxed(*reg++, dch->base + CH_DESADDRHI);
	if (hdr & LINK_XSIZE)
		writel_relaxed(*reg++, dch->base + CH_XSIZE);
	if (hdr & LINK_XSIZEHI)
		writel_relaxed(*reg++, dch->base + CH_XSIZEHI);
	if (hdr & LINK_SRCTRANSCFG)
		writel_relaxed(*reg++, dch->base + CH_SRCTRANSCFG);
	if (hdr & LINK_DESTRANSCFG)
		writel_relaxed(*reg++, dch->base + CH_DESTRANSCFG);
	if (hdr & LINK_XADDRINC)
		writel_relaxed(*reg++, dch->base + CH_XADDRINC);
	if (hdr & LINK_FILLVAL)
		writel_relaxed(*reg++, dch->base + CH_FILLVAL);
	if (hdr & LINK_SRCTRIGINCFG)
		writel_relaxed(*reg++, dch->base + CH_SRCTRIGINCFG);
	if (hdr & LINK_DESTRIGINCFG)
		writel_relaxed(*reg++, dch->base + CH_DESTRIGINCFG);
	if (hdr & LINK_AUTOCFG)
		writel_relaxed(*reg++, dch->base + CH_AUTOCFG);
	if (hdr & LINK_LINKADDR)
		writel_relaxed(*reg++, dch->base + CH_LINKADDR);
	if (hdr & LINK_LINKADDRHI)
		writel_relaxed(*reg++, dch->base + CH_LINKADDRHI);

	writel(CH_CMD_ENABLE, dch->base + CH_CMD);
}

static void d350_issue_pending(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&dch->vc.lock, flags);
	if (vchan_issue_pending(&dch->vc) && !dch->desc)
		d350_start_next(dch);
	spin_unlock_irqrestore(&dch->vc.lock, flags);
}

static irqreturn_t d350_irq(int irq, void *data)
{
	struct d350_chan *dch = data;
	struct virt_dma_desc *vd;
	u32 ch_status, intr_flags;

	ch_status = readl(dch->base + CH_STATUS);

	/*
	 * CH_STATUS contains both interrupt flags (bits 0-10) and status
	 * flags (bits 16-26). The INTR bits are read-only indicators.
	 * To clear an interrupt, write-1 to the corresponding STAT bit.
	 * STAT bits are at (INTR bit position + 16).
	 */
	intr_flags = ch_status & CH_STAT_INTR_MASK;
	if (!intr_flags)
		return IRQ_NONE;

	/* Clear status flags to acknowledge interrupts (STAT = INTR << 16) */
	writel_relaxed(intr_flags << 16, dch->base + CH_STATUS);

	spin_lock(&dch->vc.lock);

	if (!dch->desc) {
		spin_unlock(&dch->vc.lock);
		return IRQ_HANDLED;
	}

	vd = &dch->desc->vd;

	if (intr_flags & CH_STAT_INTR_ERR) {
		u32 errinfo = readl_relaxed(dch->base + CH_ERRINFO);

		dev_err(dch->vc.chan.device->dev,
			"DMA error on channel, errinfo=0x%08x\n", errinfo);

		if (errinfo & (CH_ERRINFO_AXIRDPOISERR | CH_ERRINFO_AXIRDRESPERR))
			vd->tx_result.result = DMA_TRANS_READ_FAILED;
		else if (errinfo & CH_ERRINFO_AXIWRRESPERR)
			vd->tx_result.result = DMA_TRANS_WRITE_FAILED;
		else
			vd->tx_result.result = DMA_TRANS_ABORTED;

		vd->tx_result.residue = d350_get_residue(dch);
		dch->status = DMA_ERROR;
		dch->residue = vd->tx_result.residue;

		/* Stop the channel on error */
		writel_relaxed(CH_CMD_STOP, dch->base + CH_CMD);
	} else if (intr_flags & CH_STAT_INTR_DONE) {
		if (dch->desc->cyclic) {
			static ktime_t last_irq;
			ktime_t now = ktime_get();
			s64 delta_us = ktime_to_us(ktime_sub(now, last_irq));

			if (delta_us > 1000) /* Only log if > 1ms gap */
				pr_debug("dma350: period %zu, delta %lld us\n",
					 dch->period_idx, delta_us);
			last_irq = now;

			/*
			 * Cyclic DMA: notify each period completion.
			 * The DMA continues to run automatically.
			 */
			dch->period_idx++;
			if (dch->period_idx >= dch->desc->num_periods)
				dch->period_idx = 0;

			/* Update residue for position tracking */
			dch->residue = (dch->desc->num_periods - dch->period_idx) *
				       dch->desc->period_len;

			vchan_cyclic_callback(vd);
		} else {
			/* Non-cyclic: mark complete and start next */
			vchan_cookie_complete(vd);
			dch->status = DMA_COMPLETE;
			dch->residue = 0;
			d350_start_next(dch);
		}
	} else {
		dev_warn(dch->vc.chan.device->dev,
			 "Unexpected IRQ flags: 0x%03x (status: 0x%08x)\n",
			 intr_flags, ch_status);
	}

	spin_unlock(&dch->vc.lock);

	return IRQ_HANDLED;
}

static int d350_alloc_chan_resources(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);
	struct d350 *dmac = to_d350(chan->device);
	int chan_id = dch->vc.chan.chan_id;
	int ret;

	ret = request_irq(dch->irq, d350_irq, IRQF_SHARED,
			  dev_name(&dch->vc.chan.dev->device), dch);
	if (ret)
		return ret;

	writel_relaxed(CH_INTREN_DONE | CH_INTREN_ERR, dch->base + CH_INTREN);

	/*
	 * Enable interrupt routing to AP via remote control regmap.
	 * On Sky1, this sets bit N in AUDSS_CRU_AP_IRQ to route channel N
	 * interrupts to the application processor.
	 */
	if (dmac->remote_ctrl) {
		regmap_update_bits(dmac->remote_ctrl, AUDSS_CRU_AP_IRQ,
				   BIT(chan_id), BIT(chan_id));
		dev_dbg(chan->device->dev, "Enabled IRQ routing for channel %d\n",
			chan_id);
	}

	return 0;
}

static void d350_free_chan_resources(struct dma_chan *chan)
{
	struct d350_chan *dch = to_d350_chan(chan);

	writel_relaxed(0, dch->base + CH_INTREN);
	free_irq(dch->irq, dch);
	vchan_free_chan_resources(&dch->vc);
}

/*
 * Slave DMA support
 */
static int d350_device_config(struct dma_chan *chan,
			      struct dma_slave_config *config)
{
	struct d350_chan *dch = to_d350_chan(chan);

	memcpy(&dch->slave_cfg, config, sizeof(*config));

	if (config->direction == DMA_MEM_TO_DEV) {
		dch->dev_addr = config->dst_addr;
		dch->dir = DMA_MEM_TO_DEV;
	} else if (config->direction == DMA_DEV_TO_MEM) {
		dch->dev_addr = config->src_addr;
		dch->dir = DMA_DEV_TO_MEM;
	}

	return 0;
}

static struct dma_chan *d350_of_xlate(struct of_phandle_args *dma_spec,
				      struct of_dma *ofdma)
{
	struct d350 *dmac = ofdma->of_dma_data;
	struct dma_chan *chan;
	struct d350_chan *dch;
	u32 request = 0;
	u32 channel_id = 0;

	if (dma_spec->args_count >= 1)
		request = dma_spec->args[0];
	if (dma_spec->args_count >= 2)
		channel_id = dma_spec->args[1];

	/* Channel 0xFF means dynamic allocation */
	if (channel_id == 0xFF) {
		chan = dma_get_any_slave_channel(&dmac->dma);
	} else {
		if (channel_id >= dmac->nchan)
			return NULL;
		chan = dma_get_slave_channel(&dmac->channels[channel_id].vc.chan);
	}

	if (!chan)
		return NULL;

	dch = to_d350_chan(chan);
	dch->req_line = request;

	return chan;
}

static struct dma_chan *d350_acpi_xlate(struct acpi_dma_spec *dma_spec,
				       struct acpi_dma *adma)
{
	struct d350 *dmac = adma->data;
	struct dma_chan *chan;
	struct d350_chan *dch;
	u32 request = dma_spec->slave_id;
	u32 channel_id = dma_spec->chan_id;

	if (channel_id == 0xFF)
		chan = dma_get_any_slave_channel(&dmac->dma);
	else if (channel_id < dmac->nchan)
		chan = dma_get_slave_channel(&dmac->channels[channel_id].vc.chan);
	else
		return NULL;

	if (!chan)
		return NULL;

	dch = to_d350_chan(chan);
	dch->req_line = request;

	return chan;
}

static struct dma_async_tx_descriptor *d350_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction dir, unsigned long flags, void *context)
{
	struct d350_chan *dch = to_d350_chan(chan);
	struct d350 *dmac = to_d350(chan->device);
	struct device *dev = chan->device->dev;
	struct d350_desc *desc;
	struct scatterlist *sg;
	dma_addr_t dev_addr;
	size_t cmdlink_size, total_len = 0;
	u8 transize;
	u32 *cmd;
	unsigned int i;

	/* Validate inputs */
	if (!sgl || !sg_len)
		return NULL;

	if (dir != DMA_MEM_TO_DEV && dir != DMA_DEV_TO_MEM)
		return NULL;

	dev_dbg(dev, "prep_slave_sg: chan=%d dir=%d sg_len=%u\n",
		dch->vc.chan.chan_id, dir, sg_len);

	/* Get device address and transfer size from slave config */
	if (dir == DMA_MEM_TO_DEV) {
		dev_addr = d350_translate_addr(dmac, dch->slave_cfg.dst_addr);
		transize = ffs(dch->slave_cfg.dst_addr_width) - 1;
	} else {
		dev_addr = d350_translate_addr(dmac, dch->slave_cfg.src_addr);
		transize = ffs(dch->slave_cfg.src_addr_width) - 1;
	}

	/* Allocate descriptor */
	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->scatter_gather = true;
	desc->sg_len = sg_len;
	desc->dir = dir;
	desc->tsz = transize;

	/* Allocate DMA coherent memory for command link array */
	cmdlink_size = sg_len * CMDLINK_WORDS_PER_ENTRY * sizeof(u32);
	desc->cmdlink_virt = dma_alloc_coherent(dev, cmdlink_size,
						&desc->cmdlink_phys, GFP_NOWAIT);
	if (!desc->cmdlink_virt) {
		dev_err(dev, "slave_sg: failed to alloc cmdlink buffer\n");
		kfree(desc);
		return NULL;
	}
	desc->cmdlink_size = cmdlink_size;

	/* Build command for each sg entry */
	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t mem_addr = sg_dma_address(sg);
		size_t len = sg_dma_len(sg);
		dma_addr_t next_cmd_phys;
		bool last_entry = (i == sg_len - 1);
		bool enable_irq = last_entry;  /* IRQ only on last entry */

		total_len += len;

		/* Get command buffer pointer */
		cmd = &desc->cmdlink_virt[i * CMDLINK_WORDS_PER_ENTRY];

		/* Calculate next command address (0 for last = stop) */
		if (!last_entry) {
			next_cmd_phys = desc->cmdlink_phys +
					(i + 1) * CMDLINK_WORDS_PER_ENTRY * sizeof(u32);
			next_cmd_phys = d350_translate_addr(dmac, next_cmd_phys);
		} else {
			next_cmd_phys = 0;
		}

		/* Build the command using shared helper */
		d350_build_slave_command(dmac, dch, cmd, mem_addr, dev_addr,
					 len, transize, dir, enable_irq,
					 next_cmd_phys, i == 0);
	}

	desc->total_len = total_len;

	dev_dbg(dev, "slave_sg: prepared %u entries, total %zu bytes\n",
		sg_len, total_len);

	return vchan_tx_prep(&dch->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *d350_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction dir,
	unsigned long flags)
{
	struct d350_chan *dch = to_d350_chan(chan);
	struct d350 *dmac = to_d350(chan->device);
	struct device *dev = chan->device->dev;
	struct d350_desc *desc;
	size_t num_periods, cmdlink_size;
	dma_addr_t dev_addr;
	u8 transize;
	u32 *cmd;
	size_t i;

	/* Validate direction */
	if (dir != DMA_MEM_TO_DEV && dir != DMA_DEV_TO_MEM) {
		dev_err(dev, "cyclic: invalid direction %d\n", dir);
		return NULL;
	}

	/* Validate lengths */
	if (!buf_len || !period_len || buf_len % period_len) {
		dev_err(dev, "cyclic: invalid lengths buf=%zu period=%zu\n",
			buf_len, period_len);
		return NULL;
	}

	num_periods = buf_len / period_len;

	dev_dbg(dev, "prep_dma_cyclic: chan=%d dir=%d buf=%pad len=%zu period=%zu periods=%zu\n",
		dch->vc.chan.chan_id, dir, &buf_addr, buf_len, period_len, num_periods);

	/* Get device address and transfer size from slave config */
	if (dir == DMA_MEM_TO_DEV) {
		dev_addr = d350_translate_addr(dmac, dch->slave_cfg.dst_addr);
		transize = ffs(dch->slave_cfg.dst_addr_width) - 1;
	} else {
		dev_addr = d350_translate_addr(dmac, dch->slave_cfg.src_addr);
		transize = ffs(dch->slave_cfg.src_addr_width) - 1;
	}

	/* Allocate descriptor */
	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->cyclic = true;
	desc->num_periods = num_periods;
	desc->period_len = period_len;
	desc->dir = dir;
	desc->tsz = transize;

	/* Allocate DMA coherent memory for command link array */
	cmdlink_size = num_periods * CMDLINK_WORDS_PER_ENTRY * sizeof(u32);
	desc->cmdlink_virt = dma_alloc_coherent(dev, cmdlink_size,
						&desc->cmdlink_phys, GFP_NOWAIT);
	if (!desc->cmdlink_virt) {
		dev_err(dev, "cyclic: failed to alloc cmdlink buffer\n");
		kfree(desc);
		return NULL;
	}
	desc->cmdlink_size = cmdlink_size;

	dev_dbg(dev, "cyclic: cmdlink alloc %zu bytes at phys=%pad virt=%px\n",
		cmdlink_size, &desc->cmdlink_phys, desc->cmdlink_virt);

	/* Generate command link for each period using shared helper */
	for (i = 0; i < num_periods; i++) {
		dma_addr_t mem_addr = buf_addr + i * period_len;
		dma_addr_t next_cmd_phys;

		cmd = &desc->cmdlink_virt[i * CMDLINK_WORDS_PER_ENTRY];

		/* Calculate next command address (circular: last links to first) */
		if (i == num_periods - 1)
			next_cmd_phys = desc->cmdlink_phys;
		else
			next_cmd_phys = desc->cmdlink_phys +
					(i + 1) * CMDLINK_WORDS_PER_ENTRY * sizeof(u32);
		next_cmd_phys = d350_translate_addr(dmac, next_cmd_phys);

		/* Build command using shared helper - IRQ on every period */
		d350_build_slave_command(dmac, dch, cmd, mem_addr, dev_addr,
					 period_len, transize, dir,
					 true, next_cmd_phys, i == 0);
	}

	return vchan_tx_prep(&dch->vc, &desc->vd, flags);
}

static int d350_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct d350 *dmac;
	struct clk *clk;
	void __iomem *base;
	u32 reg;
	int ret, nchan, dw, aw, r, p;
	bool coherent, memset;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* Get clock (optional - some platforms may not have one) */
	clk = devm_clk_get_optional(dev, "axiclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to get clock\n");

	/* Fallback to unnamed clock if axiclk not found */
	if (!clk) {
		clk = devm_clk_get_optional(dev, NULL);
		if (IS_ERR(clk))
			return dev_err_probe(dev, PTR_ERR(clk), "Failed to get clock\n");
	}

	/*
	 * Under ACPI, the audio clock controller (CIXH6061) provides clocks
	 * via clkdev and also activates the audio power domain during its
	 * probe.  If we got no clock, the clock controller hasn't probed
	 * yet — defer until it registers its clkdev entries.
	 */
	if (!clk && has_acpi_companion(dev))
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for audio clock controller\n");

	/*
	 * Enable runtime PM to activate power domain. The runtime_resume
	 * callback handles NULL dmac gracefully during this initial call.
	 * We then manually enable the clock since dmac isn't set up yet.
	 */
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "Failed to power on device\n");
	}

	/* Manually enable clock for initial probe (runtime_resume skipped it) */
	ret = clk_prepare_enable(clk);
	if (ret) {
		pm_runtime_put(dev);
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "Failed to enable clock\n");
	}

	/* Get and deassert reset (optional - some platforms may not have one) */
	{
		struct reset_control *rst;

		rst = devm_reset_control_get_optional_exclusive(dev, "dma_reset");
		if (IS_ERR(rst)) {
			ret = PTR_ERR(rst);
			dev_err(dev, "Failed to get reset: %d\n", ret);
			goto err_pm_put;
		}
		/*
		 * Under ACPI, the audio reset controller (CIXH6062)
		 * registers lookup entries during its probe.  If we
		 * got NULL, it hasn't probed yet — defer.
		 */
		if (!rst && has_acpi_companion(dev)) {
			dev_info(dev, "waiting for audio reset controller\n");
			ret = -EPROBE_DEFER;
			goto err_pm_put;
		}
		if (rst) {
			dev_dbg(dev, "DMA-350 probe: deasserting reset\n");
			ret = reset_control_deassert(rst);
			if (ret) {
				dev_err(dev, "Failed to deassert reset: %d\n", ret);
				goto err_pm_put;
			}
			/* Small delay after reset deassert */
			usleep_range(10, 20);
		}
	}

	reg = readl_relaxed(base + DMAINFO + IIDR);
	r = FIELD_GET(IIDR_VARIANT, reg);
	p = FIELD_GET(IIDR_REVISION, reg);
	if (FIELD_GET(IIDR_IMPLEMENTER, reg) != IMPLEMENTER_ARM ||
	    FIELD_GET(IIDR_PRODUCTID, reg) != PRODUCTID_DMA350) {
		ret = -ENODEV;
		dev_err(dev, "Not a DMA-350!");
		goto err_pm_put;
	}

	reg = readl_relaxed(base + DMAINFO + DMA_BUILDCFG0);
	nchan = FIELD_GET(DMA_CFG_NUM_CHANNELS, reg) + 1;
	dw = 1 << FIELD_GET(DMA_CFG_DATA_WIDTH, reg);
	aw = FIELD_GET(DMA_CFG_ADDR_WIDTH, reg) + 1;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(aw));
	coherent = device_get_dma_attr(dev) == DEV_DMA_COHERENT;

	dmac = devm_kzalloc(dev, struct_size(dmac, channels, nchan), GFP_KERNEL);
	if (!dmac) {
		ret = -ENOMEM;
		goto err_pm_put;
	}

	dmac->nchan = nchan;
	dmac->clk = clk;
	dmac->base = base;
	platform_set_drvdata(pdev, dmac);

	/* Initialize reserved memory region if specified in DT */
	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV && ret != -EINVAL) {
		dev_err(dev, "Failed to initialize reserved memory: %d\n", ret);
		goto err_pm_put;
	}

	reg = readl_relaxed(base + DMAINFO + DMA_BUILDCFG1);
	dmac->nreq = FIELD_GET(DMA_CFG_NUM_TRIGGER_IN, reg);

	/*
	 * Parse optional address translation for DMA-350 controllers
	 * that have a different memory map view than the CPU.
	 * Format: <cpu_addr dma_addr>
	 */
	{
		u32 reg_map[2], ram_map[2];

		if (device_property_read_u32_array(dev, "arm,reg-map",
						   reg_map, 2) == 0) {
			dmac->addr_xlat.reg_cpu = reg_map[0];
			dmac->addr_xlat.reg_dma = reg_map[1];
			dev_info(dev, "Address translation: reg 0x%llx -> 0x%llx\n",
				 dmac->addr_xlat.reg_cpu, dmac->addr_xlat.reg_dma);
		}

		if (device_property_read_u32_array(dev, "arm,ram-map",
						   ram_map, 2) == 0) {
			dmac->addr_xlat.ram_cpu = ram_map[0];
			dmac->addr_xlat.ram_dma = ram_map[1];
			dev_info(dev, "Address translation: ram 0x%llx -> 0x%llx\n",
				 dmac->addr_xlat.ram_cpu, dmac->addr_xlat.ram_dma);
		}
	}

	/*
	 * Parse optional remote control regmap (e.g., AUDSS CRU).
	 * On Sky1, this is used to route DMA interrupts to the AP.
	 */
	dmac->remote_ctrl = syscon_regmap_lookup_by_phandle(dev->of_node,
							    "arm,remote-ctrl");
	if (IS_ERR(dmac->remote_ctrl) && has_acpi_companion(dev)) {
		struct fwnode_handle *fw;
		struct device *syscon_dev;

		fw = fwnode_find_reference(dev_fwnode(dev),
					  "arm,remote-ctrl", 0);
		if (!IS_ERR(fw)) {
			syscon_dev = bus_find_device_by_fwnode(
					&platform_bus_type, fw);
			fwnode_handle_put(fw);
			if (syscon_dev) {
				dmac->remote_ctrl = dev_get_regmap(
							syscon_dev, NULL);
				put_device(syscon_dev);
				if (!dmac->remote_ctrl)
					dmac->remote_ctrl = ERR_PTR(-EPROBE_DEFER);
			}
		}
	}
	if (IS_ERR(dmac->remote_ctrl)) {
		if (PTR_ERR(dmac->remote_ctrl) == -ENODEV)
			dmac->remote_ctrl = NULL;  /* Optional */
		else
			dev_dbg(dev, "No remote control regmap: %ld\n",
				PTR_ERR(dmac->remote_ctrl));
		dmac->remote_ctrl = NULL;
	}
	if (dmac->remote_ctrl)
		dev_info(dev, "Using remote control regmap for interrupt routing\n");

	dev_dbg(dev, "DMA-350 r%dp%d with %d channels, %d requests\n", r, p, dmac->nchan, dmac->nreq);

	dmac->dma.dev = dev;
	for (int i = min(dw, 16); i > 0; i /= 2) {
		dmac->dma.src_addr_widths |= BIT(i);
		dmac->dma.dst_addr_widths |= BIT(i);
	}
	dmac->dma.directions = BIT(DMA_MEM_TO_MEM) | BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	dmac->dma.descriptor_reuse = true;
	dmac->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dmac->dma.device_alloc_chan_resources = d350_alloc_chan_resources;
	dmac->dma.device_free_chan_resources = d350_free_chan_resources;
	dmac->dma.device_config = d350_device_config;
	dma_cap_set(DMA_MEMCPY, dmac->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, dmac->dma.cap_mask);
	dma_cap_set(DMA_CYCLIC, dmac->dma.cap_mask);
	dma_cap_set(DMA_PRIVATE, dmac->dma.cap_mask);
	dmac->dma.device_prep_dma_memcpy = d350_prep_memcpy;
	dmac->dma.device_prep_slave_sg = d350_prep_slave_sg;
	dmac->dma.device_prep_dma_cyclic = d350_prep_dma_cyclic;
	dmac->dma.device_pause = d350_pause;
	dmac->dma.device_resume = d350_resume;
	dmac->dma.device_terminate_all = d350_terminate_all;
	dmac->dma.device_synchronize = d350_synchronize;
	dmac->dma.device_tx_status = d350_tx_status;
	dmac->dma.device_issue_pending = d350_issue_pending;
	INIT_LIST_HEAD(&dmac->dma.channels);

	/* Would be nice to have per-channel caps for this... */
	memset = true;
	for (int i = 0; i < nchan; i++) {
		struct d350_chan *dch = &dmac->channels[i];

		dch->base = base + DMACH(i);
		writel_relaxed(CH_CMD_CLEAR, dch->base + CH_CMD);

		reg = readl_relaxed(dch->base + CH_BUILDCFG1);
		if (!(FIELD_GET(CH_CFG_HAS_CMDLINK, reg))) {
			dev_warn(dev, "No command link support on channel %d\n", i);
			continue;
		}
		dch->irq = platform_get_irq_optional(pdev, i);
		if (dch->irq < 0) {
			/*
			 * Some platforms (e.g., Sky1 AUDSS) provide only a single
			 * shared interrupt for all channels. Fall back to IRQ 0.
			 */
			if (i > 0) {
				dch->irq = dmac->channels[0].irq;
				dev_dbg(dev, "Channel %d using shared IRQ %d\n",
					i, dch->irq);
			} else {
				return dev_err_probe(dev, dch->irq,
						     "Failed to get IRQ for channel %d\n", i);
			}
		}

		dch->has_wrap = FIELD_GET(CH_CFG_HAS_WRAP, reg);
		dch->has_trig = FIELD_GET(CH_CFG_HAS_TRIGIN, reg) &
				FIELD_GET(CH_CFG_HAS_TRIGSEL, reg);

		/* Fill is a special case of Wrap */
		memset &= dch->has_wrap;

		reg = readl_relaxed(dch->base + CH_BUILDCFG0);
		dch->tsz = FIELD_GET(CH_CFG_DATA_WIDTH, reg);

		reg = FIELD_PREP(CH_LINK_SHAREATTR, coherent ? SHAREATTR_ISH : SHAREATTR_OSH);
		reg |= FIELD_PREP(CH_LINK_MEMATTR, coherent ? MEMATTR_WB : MEMATTR_NC);
		writel_relaxed(reg, dch->base + CH_LINKATTR);

		dch->vc.desc_free = d350_desc_free;
		vchan_init(&dch->vc, &dmac->dma);
	}

	if (memset) {
		dma_cap_set(DMA_MEMSET, dmac->dma.cap_mask);
		dmac->dma.device_prep_dma_memset = d350_prep_memset;
	}

	/*
	 * Enable global non-secure interrupt propagation.
	 * Without this, per-channel INTREN settings have no effect.
	 */
	writel_relaxed(DMA_NSEC_INTREN_ANY, base + DMA_NSEC_INTREN);

	ret = dma_async_device_register(&dmac->dma);
	if (ret) {
		dev_err(dev, "Failed to register DMA device: %d\n", ret);
		goto err_pm_put;
	}

	/* Register for device tree DMA channel requests */
	if (dev->of_node) {
		ret = of_dma_controller_register(dev->of_node, d350_of_xlate, dmac);
		if (ret) {
			dev_err(dev, "Failed to register OF DMA controller: %d\n", ret);
			goto err_dma_unregister;
		}
	}

	/* Register for ACPI DMA channel requests */
	if (has_acpi_companion(dev)) {
		ret = acpi_dma_controller_register(dev, d350_acpi_xlate, dmac);
		if (ret) {
			dev_err(dev, "Failed to register ACPI DMA controller: %d\n", ret);
			goto err_dma_unregister;
		}
		acpi_dev_clear_dependencies(ACPI_COMPANION(dev));
	}

	dev_info(dev, "DMA-350 r%dp%d initialized with %d channels\n",
		 r, p, dmac->nchan);
	return 0;

err_dma_unregister:
	dma_async_device_unregister(&dmac->dma);

err_pm_put:
	clk_disable_unprepare(clk);
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	return ret;
}

static void d350_remove(struct platform_device *pdev)
{
	struct d350 *dmac = platform_get_drvdata(pdev);

	if (has_acpi_companion(&pdev->dev))
		acpi_dma_controller_free(&pdev->dev);
	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&dmac->dma);
	of_reserved_mem_device_release(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static int __maybe_unused d350_runtime_suspend(struct device *dev)
{
	struct d350 *dmac = dev_get_drvdata(dev);

	if (dmac && dmac->clk)
		clk_disable_unprepare(dmac->clk);
	return 0;
}

static int __maybe_unused d350_runtime_resume(struct device *dev)
{
	struct d350 *dmac = dev_get_drvdata(dev);

	if (dmac && dmac->clk)
		return clk_prepare_enable(dmac->clk);
	return 0;
}

static const struct dev_pm_ops d350_pm_ops = {
	SET_RUNTIME_PM_OPS(d350_runtime_suspend, d350_runtime_resume, NULL)
};

static const struct of_device_id d350_of_match[] __maybe_unused = {
	{ .compatible = "arm,dma-350" },
	{ .compatible = "arm,dma350-no-pause" },
	{}
};
MODULE_DEVICE_TABLE(of, d350_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id d350_acpi_match[] = {
	{ "CIXH1006", 0 },
	{ "CIXHA014", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, d350_acpi_match);
#endif

static struct platform_driver d350_driver = {
	.driver = {
		.name = "arm-dma350",
		.of_match_table = of_match_ptr(d350_of_match),
		.acpi_match_table = ACPI_PTR(d350_acpi_match),
		.pm = &d350_pm_ops,
	},
	.probe = d350_probe,
	.remove = d350_remove,
};
module_platform_driver(d350_driver);

MODULE_AUTHOR("Robin Murphy <robin.murphy@arm.com>");
MODULE_DESCRIPTION("Arm DMA-350 driver");
MODULE_LICENSE("GPL v2");
