// SPDX-License-Identifier: GPL-2.0+
/*
 * IOP mailbox driver for Apple M1 SoC
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/mailbox_controller.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/apple_iop_mapper.h>

#define NUM_IRQ			2
#define IRQ_A2I_EMPTY		0
#define IRQ_I2A_FULL		1

#define EP_MGMT			0
#define EP_CRASHLOG		1
#define EP_SYSLOG		2
#define EP_DEBUG		3
#define EP_IORPT		4
#define EP_INVALID		(-1u)

/* EP_CRASHLOG to EP_IORPT */
#define AUTO_EP_MASK		0x1E

#define MSG_INVALID(msg)	(((msg)[1] | 0xFF) == -1ull)

enum apple_iop_mailbox_ep0_state {
	EP0_IDLE,
	EP0_WAIT_HELLO,
	EP0_SEND_HELLO,
	EP0_WAIT_EPMAP,
	EP0_SEND_EPACK,
	EP0_SEND_EPSTART,
	EP0_WAIT_PWROK,
	EP0_SEND_PWRACK,
};

struct apple_iop_mailbox_data;

struct apple_iop_mailbox_hwops {
	void (*a2i_msg)(struct apple_iop_mailbox_data *, u64 *);
	void (*i2a_msg)(struct apple_iop_mailbox_data *, u64 *);
	u32 (*a2i_stat)(struct apple_iop_mailbox_data *);
	u32 (*i2a_stat)(struct apple_iop_mailbox_data *);
	bool (*a2i_full)(u32);
	bool (*i2a_empty)(u32);
	void (*a2i_ack)(struct apple_iop_mailbox_data *);
	void (*i2a_ack)(struct apple_iop_mailbox_data *);
	void (*try_boot)(struct apple_iop_mailbox_data *);
	void (*mask_a2i_empty)(struct apple_iop_mailbox_data *);
	void (*unmask_a2i_empty)(struct apple_iop_mailbox_data *);
};

struct apple_iop_mailbox_data {
	struct device *dev;
	void __iomem *base;
	struct mbox_controller mbctrl;
	struct clk_bulk_data *clks;
	struct work_struct builtin_work;
	u32 builtin_mask;
	int num_clks;
	spinlock_t lock;
	int irq[NUM_IRQ];
	const struct apple_iop_mailbox_hwops *hwops;
	bool a2i_empty_masked;
	/* memory mapper used by built-in endpoints for allocation */
	apple_iop_mapper_func_t mapper_func;
	void *mapper_priv;
	/* wait with setup for first message; this is used by IOPs that
	   need custom initialization before the mailbox is live */
	bool wait_init;
	/* show debug message trace */
	bool trace;
	/* table of expected endpoints (index in ep to EP number) */
	struct apple_iop_mailbox_ep {
		struct apple_iop_mailbox_data *am;
		struct list_head a2i_list;
		u64 a2i_msg[2], i2a_msg[2];
		u32 epnum;
		bool a2i_busy, a2i_wait, i2a_live, i2a_work;
		bool builtin, discovered;
	} *ep;
	struct mbox_chan *mbchans;
	unsigned num_ep;
	/* reverse map of endpoints (EP number to index in ep) */
	u32 *rev_ep;
	unsigned num_rev_ep;
	/* list of endpoints with A2I messages waiting */
	struct list_head a2i_list;
	struct list_head a2i_list_system;
	/* channel currently in A2I buffer */
	unsigned a2i_cur_chan;
	/* EP0 discovery state */
	enum apple_iop_mailbox_ep0_state ep0_state;
	u64 ep0_sub;
	/* builtin EP state */
	struct apple_iop_mailbox_builtin_ep {
		dma_addr_t dmah;
		size_t size;
		void *ptr;
	} builtin_ep[EP_IORPT-EP_CRASHLOG+1];
};

static unsigned apple_iop_mailbox_ep0_next_discovered_ep(struct apple_iop_mailbox_data *am,
						     unsigned ep)
{
	while(ep < am->num_ep) {
		if(am->ep[ep].discovered)
			return ep;
		ep ++;
	}
	return EP_INVALID;
}

static int apple_iop_mailbox_ep0_next_a2i(struct apple_iop_mailbox_data *am, u64 *msg)
{
	switch(am->ep0_state) {
	case EP0_IDLE:
	case EP0_WAIT_HELLO:
	case EP0_WAIT_EPMAP:
	case EP0_WAIT_PWROK:
	default:
		return 0;
	case EP0_SEND_HELLO:
		msg[0] = 0x0020000100000000 | am->ep0_sub;
		msg[1] = 0;
		am->ep0_state = EP0_WAIT_EPMAP;
		return 1;
	case EP0_SEND_EPACK:
		msg[0] = 0x0080000000000000 | (am->ep0_sub << 32) | (!(am->ep0_sub & 7));
		msg[1] = 0;
		if(am->ep0_sub & 0x80000) {
			am->ep0_sub = apple_iop_mailbox_ep0_next_discovered_ep(am, 0);
			if(am->ep0_sub == EP_INVALID)
				am->ep0_state = EP0_WAIT_PWROK;
			else
				am->ep0_state = EP0_SEND_EPSTART;
		} else
			am->ep0_state = EP0_WAIT_EPMAP;
		return 1;
	case EP0_SEND_EPSTART:
		msg[0] = 0x0050000000000002 | ((u64)am->ep[am->ep0_sub].epnum << 32);
		msg[1] = 0;
		am->ep0_sub = apple_iop_mailbox_ep0_next_discovered_ep(am, am->ep0_sub + 1);
		if(am->ep0_sub == EP_INVALID)
			am->ep0_state = EP0_WAIT_PWROK;
		return 1;
	case EP0_SEND_PWRACK:
		msg[0] = 0x00b0000000000000 | am->ep0_sub;
		msg[1] = 0;
		am->ep0_state = EP0_IDLE;
		dev_info(am->dev, "completed startup.\n");
		return 1;
	}
}

static void apple_iop_mailbox_ep0_push_i2a(struct apple_iop_mailbox_data *am, u64 *msg)
{
	unsigned msgtype = (msg[0] >> 52) & 15;
	unsigned idx, ep;

	switch(am->ep0_state) {
	case EP0_IDLE:
		if(msgtype == 11)
			return;
		/* fallthru */
	case EP0_WAIT_HELLO:
		if(msgtype == 1) {
			am->ep0_sub = msg[0] & 0xFFFFFFFFul;
			am->ep0_state = EP0_SEND_HELLO;
			return;
		}
		break;
	case EP0_WAIT_EPMAP:
		if(msgtype == 8) {
			for(idx=0; idx<32; idx++)
				if(msg[0] & (1ul << idx)) {
					ep = idx + 32 * ((msg[0] >> 32) & 7);
					if(ep < am->num_rev_ep) {
						ep = am->rev_ep[ep];
						if(ep != EP_INVALID)
							am->ep[ep].discovered = true;
					}
				}
			am->ep0_sub = (msg[0] >> 32) & 0x80007; /* last msg bit + ep block index */
			am->ep0_state = EP0_SEND_EPACK;
			return;
		}
		break;
	case EP0_WAIT_PWROK:
		if(msgtype == 7) {
			am->ep0_state = EP0_SEND_PWRACK;
			am->ep0_sub = msg[0] & 0xFFFFFFFFul;
			return;
		}
		break;
	default:
		;
	}

	dev_warn(am->dev, "received unexpected message %016llx:%llx in state %d\n",
		 msg[0], msg[1], am->ep0_state);
}

static void apple_iop_mailbox_mask_a2i_empty(struct apple_iop_mailbox_data *am)
{
	if(am->hwops->mask_a2i_empty) {
		am->hwops->mask_a2i_empty(am);
		return;
	}

	if(am->a2i_empty_masked)
		return;
	disable_irq_nosync(am->irq[IRQ_A2I_EMPTY]);
	am->a2i_empty_masked = true;
}

static void apple_iop_mailbox_unmask_a2i_empty(struct apple_iop_mailbox_data *am)
{
	if(am->hwops->unmask_a2i_empty) {
		am->hwops->unmask_a2i_empty(am);
		return;
	}

	if(!am->a2i_empty_masked)
		return;
	enable_irq(am->irq[IRQ_A2I_EMPTY]);
	am->a2i_empty_masked = false;
}

static int apple_iop_mailbox_a2i_ok(struct apple_iop_mailbox_data *am, unsigned epnum)
{
	if(am->ep0_state == EP0_IDLE)
		return 1;
	if(am->ep0_state == EP0_WAIT_PWROK && epnum < 32)
		return 1;
	return (epnum == 0);
}

static irqreturn_t apple_iop_mailbox_a2i_empty_isr(int irq, void *dev_id)
{
	struct apple_iop_mailbox_data *am = dev_id;
	struct apple_iop_mailbox_ep *ep;
	unsigned long flags;
	u64 msg[2];
	unsigned chan = EP_INVALID;
	u32 stat;

	spin_lock_irqsave(&am->lock, flags);

	if(am->hwops->a2i_ack)
		am->hwops->a2i_ack(am);

	stat = am->hwops->a2i_stat(am);
	while(!am->hwops->a2i_full(stat)) {
		chan = am->a2i_cur_chan;
		am->a2i_cur_chan = EP_INVALID;

		apple_iop_mailbox_mask_a2i_empty(am);
		if(apple_iop_mailbox_ep0_next_a2i(am, msg)) {
			if(am->trace)
				dev_info(am->dev, "tx msg %016llx %16llx\n", msg[0], msg[1]);
			am->hwops->a2i_msg(am, msg);
			apple_iop_mailbox_unmask_a2i_empty(am);

		} else if(!list_empty(&am->a2i_list_system) || !list_empty(&am->a2i_list)) {
			if(!list_empty(&am->a2i_list_system))
				ep = list_first_entry(&am->a2i_list_system,
					struct apple_iop_mailbox_ep, a2i_list);
			else
				ep = list_first_entry(&am->a2i_list,
					struct apple_iop_mailbox_ep, a2i_list);

			if(apple_iop_mailbox_a2i_ok(am, ep->epnum)) {
				list_del(&ep->a2i_list);
				if(am->trace)
					dev_info(am->dev, "tx msg %016llx %16llx [%d]\n",
						 ep->a2i_msg[0], ep->a2i_msg[1], ep->epnum);
				if(!MSG_INVALID(ep->a2i_msg))
					am->hwops->a2i_msg(am, ep->a2i_msg);
				apple_iop_mailbox_unmask_a2i_empty(am);

				am->a2i_cur_chan = ep - am->ep;
			}
			break;
		} else
			break;
		stat = am->hwops->a2i_stat(am);
	}

	if(chan != EP_INVALID && !am->ep[chan].i2a_live)
		chan = EP_INVALID;

	spin_unlock_irqrestore(&am->lock, flags);

	if(chan != EP_INVALID) {
		if(am->trace)
			dev_info(am->dev, "tx complete [%d/%d].\n", am->ep[chan].epnum, chan);
		am->ep[chan].a2i_busy = false;

		if(am->ep[chan].builtin)
			schedule_work(&am->builtin_work);
		else
			mbox_chan_txdone(&am->mbchans[chan], 0);
	}

	return IRQ_HANDLED;
}

static irqreturn_t apple_iop_mailbox_i2a_full_isr(int irq, void *dev_id)
{
	struct apple_iop_mailbox_data *am = dev_id;
	unsigned long flags;
	u64 msg[2];
	unsigned epnum, chan = EP_INVALID;
	u32 stat;

	spin_lock_irqsave(&am->lock, flags);

	stat = am->hwops->i2a_stat(am);
	if(am->hwops->i2a_empty(stat)) {
		dev_err(am->dev, "full_isr called but i2a_stat is 0x%x - empty.\n", stat);
		spin_unlock_irqrestore(&am->lock, flags);
		return IRQ_HANDLED;
	}

	am->hwops->i2a_msg(am, msg);
	epnum = msg[1] & 0xFF;
	if(am->trace)
		dev_info(am->dev, "rx msg %016llx %16llx\n", msg[0], msg[1]);

	if(epnum < am->num_rev_ep)
		chan = am->rev_ep[epnum];

	if(chan == EP_INVALID && !epnum) {
		apple_iop_mailbox_ep0_push_i2a(am, msg);

		/* if empty, push the notification state machine */
		stat = am->hwops->a2i_stat(am);
		while(!am->hwops->a2i_full(stat) && apple_iop_mailbox_ep0_next_a2i(am, msg)) {
			if(am->trace)
				dev_info(am->dev, "tx msg %016llx %16llx\n", msg[0], msg[1]);
			am->hwops->a2i_msg(am, msg);
			apple_iop_mailbox_unmask_a2i_empty(am);
			stat = am->hwops->a2i_stat(am);
		}
	}

	if(chan != EP_INVALID && am->ep[chan].builtin) {
		am->ep[chan].i2a_work = true;
		am->ep[chan].i2a_msg[0] = msg[0];
		am->ep[chan].i2a_msg[1] = msg[1];
	} else if(chan != EP_INVALID && !am->ep[chan].i2a_live)
		chan = EP_INVALID;

	if(am->hwops->i2a_ack)
		am->hwops->i2a_ack(am);

	spin_unlock_irqrestore(&am->lock, flags);

	if(chan != EP_INVALID) {
		if(am->ep[chan].builtin)
			schedule_work(&am->builtin_work);
		else
			mbox_chan_received_data(&am->mbchans[chan], msg);
	}

	return IRQ_HANDLED;
}

static int apple_iop_mailbox_send_data(struct mbox_chan *chan, void *_msg)
{
	struct apple_iop_mailbox_ep *ep = chan->con_priv;
	struct apple_iop_mailbox_data *am = ep->am;
	unsigned long flags;
	u64 *msg = _msg, msg1[2];
	u32 stat;

	spin_lock_irqsave(&am->lock, flags);

	if(am->wait_init) {
		am->wait_init = false;
		if(am->ep0_state == EP0_IDLE)
			am->ep0_state = EP0_WAIT_HELLO;
		if(am->hwops->try_boot)
			am->hwops->try_boot(am);
	}

	if(ep->a2i_busy) {
		spin_unlock_irqrestore(&am->lock, flags);
		return -EBUSY;
	}

	msg1[0] = msg[0];
	msg1[1] = (msg[1] & ~0xFFul) | ep->epnum;

	if(am->a2i_cur_chan != EP_INVALID || !apple_iop_mailbox_a2i_ok(am, ep->epnum) ||
	   MSG_INVALID(msg)) {
		ep->a2i_msg[0] = msg1[0];
		ep->a2i_msg[1] = msg1[1];
		ep->a2i_busy = true;
		if(ep->epnum < 32)
			list_add(&ep->a2i_list, &am->a2i_list_system);
		else
			list_add(&ep->a2i_list, &am->a2i_list);
		if(MSG_INVALID(msg))
			apple_iop_mailbox_unmask_a2i_empty(am);
	} else {
		stat = am->hwops->a2i_stat(am);
		if(!am->hwops->a2i_full(stat)) {
			if(am->trace)
				dev_info(am->dev, "tx msg %016llx %16llx [%d]\n",
					 msg[0], msg[1], ep->epnum);
			am->hwops->a2i_msg(am, msg1);
			apple_iop_mailbox_unmask_a2i_empty(am);

			am->a2i_cur_chan = ep - am->ep;
			ep->a2i_busy = true;
		} else
			dev_err(am->dev, "no entry in transmit buffer "
				"but a2i_stat is 0x%x - not empty.\n", stat);
	}

	spin_unlock_irqrestore(&am->lock, flags);

	return 0;
}

static const char * const apple_iop_builtin_epname[] = {
	"crash log",
	"syslog",
	"debug",
	"I/O report",
};

static void apple_iop_builtin_work_ep(struct apple_iop_mailbox_data *am, unsigned epnum,
	struct mbox_chan *chan, u64 *msg)
{
	struct apple_iop_mailbox_builtin_ep *bep = &am->builtin_ep[epnum - 1];
	unsigned long flags;
	u64 size, addr;

	if(!msg)
		return;

	if(bep->ptr) {
		/* process message - print? */
		apple_iop_mailbox_send_data(chan, msg);
		if(epnum == EP_CRASHLOG)
			dev_err(am->dev, "received crash: 0x%llx (buffer at 0x%llx+0x%llx)\n",
				(long long)msg[0], (long long)bep->dmah, (long long)bep->size);
	} else {
		/* allocate buffer for this built-in EP */
		size = (msg[0] >> 44) & 255;
		switch((msg[0] >> 52) & 3) {
		case 0:
			size = 0;
			break;
		case 1:
		case 3:
			size <<= 12;
			break;
		case 2:
			size <<= 20;
			break;
		}

		dev_info(am->dev, "allocating %d kB buffer for %s endpoint.\n",
			 (int)(size >> 10), apple_iop_builtin_epname[epnum - 1]);
		bep->ptr = dma_alloc_coherent(am->dev, size, &bep->dmah, GFP_KERNEL);
		bep->size = size;
		if(bep->ptr) {
			addr = bep->dmah;
			spin_lock_irqsave(&am->lock, flags);
			if(am->mapper_func)
				addr = am->mapper_func(am->mapper_priv, addr, size);
			spin_unlock_irqrestore(&am->lock, flags);
		} else
			addr = 0;
		msg[0] &= ~0xFFFFFFFFFFF;
		msg[0] |= bep->dmah;
		apple_iop_mailbox_send_data(chan, msg);
	}
}

static void apple_iop_builtin_work_func(struct work_struct *work)
{
	struct apple_iop_mailbox_data *am =
		container_of(work, struct apple_iop_mailbox_data, builtin_work);
	unsigned long flags;
	uint64_t msg[2];
	unsigned epnum, vld, chan;
	struct apple_iop_mailbox_ep *ep;

	spin_lock_irqsave(&am->lock, flags);

	for(epnum=EP_CRASHLOG; epnum<=EP_IORPT; epnum++) {
		chan = am->rev_ep[epnum];
		if(chan == EP_INVALID)
			continue;
		ep = &am->ep[chan];
		if(!ep->builtin || !(ep->a2i_wait || ep->i2a_work))
			continue;

		vld = ep->i2a_work;
		if(vld) {
			msg[0] = ep->i2a_msg[0];
			msg[1] = ep->i2a_msg[1];
		}
		ep->a2i_wait = false;
		ep->i2a_work = false;

		spin_unlock_irqrestore(&am->lock, flags);
		apple_iop_builtin_work_ep(am, epnum, &am->mbchans[chan], vld ? msg : NULL);
		spin_lock_irqsave(&am->lock, flags);
	}

	spin_unlock_irqrestore(&am->lock, flags);
}

static int apple_iop_mailbox_startup(struct mbox_chan *chan)
{
	struct apple_iop_mailbox_ep *ep = chan->con_priv;
	struct apple_iop_mailbox_data *am = ep->am;
	unsigned long flags;

	spin_lock_irqsave(&am->lock, flags);
	ep->i2a_live = true;
	spin_unlock_irqrestore(&am->lock, flags);

	return 0;
}

static void apple_iop_mailbox_shutdown(struct mbox_chan *chan)
{
	struct apple_iop_mailbox_ep *ep = chan->con_priv;
	struct apple_iop_mailbox_data *am = ep->am;
	unsigned long flags;

	spin_lock_irqsave(&am->lock, flags);
	ep->i2a_live = false;
	spin_unlock_irqrestore(&am->lock, flags);
}

static const struct mbox_chan_ops apple_iop_mailbox_ops = {
	.startup = apple_iop_mailbox_startup,
	.send_data = apple_iop_mailbox_send_data,
	.shutdown = apple_iop_mailbox_shutdown,
};

static const irq_handler_t apple_iop_mailbox_isr_table[NUM_IRQ] = {
	apple_iop_mailbox_a2i_empty_isr,
	apple_iop_mailbox_i2a_full_isr,
};

static struct mbox_chan *apple_iop_mailbox_of_xlate(struct mbox_controller *mbox,
						    const struct of_phandle_args *sp)
{
	struct apple_iop_mailbox_data *am = dev_get_drvdata(mbox->dev);
	unsigned epnum, chan;

	if(sp->args_count != 1) {
		dev_err(mbox->dev, "invalid argument count %d.\n", sp->args_count);
		return ERR_PTR(-EINVAL);
	}
	epnum = sp->args[0];

	if(epnum >= am->num_rev_ep || am->rev_ep[epnum] == EP_INVALID) {
		dev_err(mbox->dev, "endpoint %d not mapped.\n", epnum);
		return ERR_PTR(-ENODEV);
	}

	chan = am->rev_ep[epnum];
	return &mbox->chans[chan];
}

int apple_iop_set_mapper_func(void *iop_mbox_chan, apple_iop_mapper_func_t func, void *priv)
{
	struct mbox_chan *chan = iop_mbox_chan;
	struct apple_iop_mailbox_ep *ep = chan->con_priv;
	struct apple_iop_mailbox_data *am = ep->am;
	unsigned long flags;

	spin_lock_irqsave(&am->lock, flags);
	am->mapper_func = func;
	am->mapper_priv = priv;
	spin_unlock_irqrestore(&am->lock, flags);
	return 0;
}

/* A2I -> AP to IOP, I2A -> IOP to AP */
#define A7V4_CPU_CTRL		0x0044
#define   A7V4_CPU_CTRL_RUN	BIT(4)
#define A7V4_A2I_STAT		0x8110
#define   A7V4_A2I_STAT_EMPTY	BIT(17)
#define   A7V4_A2I_STAT_FULL	BIT(16)
#define A7V4_I2A_STAT		0x8114
#define   A7V4_I2A_STAT_EMPTY	BIT(17)
#define   A7V4_I2A_STAT_FULL	BIT(16)
#define A7V4_A2I_MSG0		0x8800
#define A7V4_A2I_MSG1		0x8808
#define A7V4_I2A_MSG0		0x8830
#define A7V4_I2A_MSG1		0x8838

static void apple_iop_mailbox_a7v4_a2i_msg(struct apple_iop_mailbox_data *am, u64 *msg)
{
	writeq(msg[0], am->base + A7V4_A2I_MSG0);
	writeq(msg[1], am->base + A7V4_A2I_MSG1);
}

static void apple_iop_mailbox_a7v4_i2a_msg(struct apple_iop_mailbox_data *am, u64 *msg)
{
	msg[0] = readq(am->base + A7V4_I2A_MSG0);
	msg[1] = readq(am->base + A7V4_I2A_MSG1);
}

static u32 apple_iop_mailbox_a7v4_a2i_stat(struct apple_iop_mailbox_data *am)
{
	return readl(am->base + A7V4_A2I_STAT);
}

static u32 apple_iop_mailbox_a7v4_i2a_stat(struct apple_iop_mailbox_data *am)
{
	return readl(am->base + A7V4_I2A_STAT);
}

static bool apple_iop_mailbox_a7v4_a2i_full(u32 stat)
{
	return !!(stat & A7V4_A2I_STAT_FULL);
}

static bool apple_iop_mailbox_a7v4_i2a_empty(u32 stat)
{
	return !!(stat & A7V4_I2A_STAT_EMPTY);
}

static void apple_iop_mailbox_a7v4_try_boot(struct apple_iop_mailbox_data *am)
{
	u32 ctrl;

	if(!am->wait_init && am->ep0_state == EP0_WAIT_HELLO) {
		ctrl = readl(am->base + A7V4_CPU_CTRL);
		if(ctrl & A7V4_CPU_CTRL_RUN) {
			dev_info(am->dev, "waking coprocessor.\n");
			writeq(0x0060000000000220, am->base + A7V4_A2I_MSG0);
			writeq(0, am->base + A7V4_A2I_MSG1);
			apple_iop_mailbox_unmask_a2i_empty(am);
		} else {
			dev_info(am->dev, "booting coprocessor.\n");
			writel(ctrl | A7V4_CPU_CTRL_RUN, am->base + A7V4_CPU_CTRL);
		}
	}
}

const struct apple_iop_mailbox_hwops apple_iop_mailbox_a7v4_hwops = {
	.a2i_msg = apple_iop_mailbox_a7v4_a2i_msg,
	.i2a_msg = apple_iop_mailbox_a7v4_i2a_msg,
	.a2i_stat = apple_iop_mailbox_a7v4_a2i_stat,
	.i2a_stat = apple_iop_mailbox_a7v4_i2a_stat,
	.a2i_full = apple_iop_mailbox_a7v4_a2i_full,
	.i2a_empty = apple_iop_mailbox_a7v4_i2a_empty,
	.try_boot = apple_iop_mailbox_a7v4_try_boot,
};

#define M3V1_IRQEN		0x0048
#define   M3V1_IRQEN_A2I	BIT(0)
#define   M3V1_IRQEN_I2A	BIT(3)
#define M3V1_IRQACK		0x004c
#define   M3V1_IRQACK_A2I	BIT(0)
#define   M3V1_IRQACK_I2A	BIT(3)
#define M3V1_A2I_STAT		0x0050
#define   M3V1_A2I_STAT_EMPTY	BIT(17)
#define   M3V1_A2I_STAT_FULL	BIT(16)
#define M3V1_A2I_MSG0		0x0060
#define M3V1_A2I_MSG1		0x0068
#define M3V1_I2A_STAT		0x0080
#define   M3V1_I2A_STAT_EMPTY	BIT(17)
#define   M3V1_I2A_STAT_FULL	BIT(16)
#define M3V1_I2A_MSG0		0x00A0
#define M3V1_I2A_MSG1		0x00A8

static void apple_iop_mailbox_m3v1_a2i_msg(struct apple_iop_mailbox_data *am, u64 *msg)
{
	writeq(msg[0], am->base + M3V1_A2I_MSG0);
	writeq(msg[1], am->base + M3V1_A2I_MSG1);
}

static void apple_iop_mailbox_m3v1_i2a_msg(struct apple_iop_mailbox_data *am, u64 *msg)
{
	msg[0] = readq(am->base + M3V1_I2A_MSG0);
	msg[1] = readq(am->base + M3V1_I2A_MSG1);
}

static u32 apple_iop_mailbox_m3v1_a2i_stat(struct apple_iop_mailbox_data *am)
{
	return readl(am->base + M3V1_A2I_STAT);
}

static u32 apple_iop_mailbox_m3v1_i2a_stat(struct apple_iop_mailbox_data *am)
{
	return readl(am->base + M3V1_I2A_STAT);
}

static bool apple_iop_mailbox_m3v1_a2i_full(u32 stat)
{
	return !!(stat & M3V1_A2I_STAT_FULL);
}

static bool apple_iop_mailbox_m3v1_i2a_empty(u32 stat)
{
	return !!(stat & M3V1_I2A_STAT_EMPTY);
}

static void apple_iop_mailbox_m3v1_a2i_ack(struct apple_iop_mailbox_data *am)
{
	writel(M3V1_IRQACK_A2I, am->base + M3V1_IRQACK);
}

static void apple_iop_mailbox_m3v1_i2a_ack(struct apple_iop_mailbox_data *am)
{
	writel(M3V1_IRQACK_I2A, am->base + M3V1_IRQACK);
}

static void apple_iop_mailbox_m3v1_mask_a2i_empty(struct apple_iop_mailbox_data *am)
{
	writel(readl(am->base + M3V1_IRQEN) & ~M3V1_IRQEN_A2I, am->base + M3V1_IRQEN);
}

static void apple_iop_mailbox_m3v1_unmask_a2i_empty(struct apple_iop_mailbox_data *am)
{
	writel(readl(am->base + M3V1_IRQEN) | M3V1_IRQEN_A2I, am->base + M3V1_IRQEN);
}

const struct apple_iop_mailbox_hwops apple_iop_mailbox_m3v1_hwops = {
	.a2i_msg = apple_iop_mailbox_m3v1_a2i_msg,
	.i2a_msg = apple_iop_mailbox_m3v1_i2a_msg,
	.a2i_stat = apple_iop_mailbox_m3v1_a2i_stat,
	.i2a_stat = apple_iop_mailbox_m3v1_i2a_stat,
	.a2i_full = apple_iop_mailbox_m3v1_a2i_full,
	.i2a_empty = apple_iop_mailbox_m3v1_i2a_empty,
	.a2i_ack = apple_iop_mailbox_m3v1_a2i_ack,
	.i2a_ack = apple_iop_mailbox_m3v1_i2a_ack,
	.mask_a2i_empty = apple_iop_mailbox_m3v1_mask_a2i_empty,
	.unmask_a2i_empty = apple_iop_mailbox_m3v1_unmask_a2i_empty,
};

static const struct of_device_id apple_iop_mailbox_of_match[] = {
	{ .compatible = "apple,iop-mailbox-m1",		&apple_iop_mailbox_a7v4_hwops },
	{ .compatible = "apple,iop-mailbox-micro-m1",	&apple_iop_mailbox_m3v1_hwops },
	{ }
};

static int apple_iop_mailbox_probe(struct platform_device *pdev)
{
	struct apple_iop_mailbox_data *am;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	struct resource *res;
	unsigned int i, irq, max_ep = 0, system_ep_mask = 0, num_system_ep;
	unsigned long flags;
	u32 epnum;
	int err;

	match = of_match_node(apple_iop_mailbox_of_match, pdev->dev.of_node);
	if(!match)
		return -EINVAL;

	am = devm_kzalloc(&pdev->dev, sizeof(*am), GFP_KERNEL);
	if(!am)
		return -ENOMEM;
	am->dev = &pdev->dev;
	am->hwops = match->data;
	spin_lock_init(&am->lock);
	INIT_LIST_HEAD(&am->a2i_list);
	INIT_LIST_HEAD(&am->a2i_list_system);
	am->a2i_cur_chan = EP_INVALID;

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(&pdev->dev, "missing MMIO range.\n");
		return -EINVAL;
	}

	am->base = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(am->base)) {
		err = PTR_ERR(am->base);
		dev_err(&pdev->dev, "failed to map MMIO: %d.\n", err);
		return err;
	}

	am->wait_init = of_property_read_bool(np, "wait-init");
	am->trace = of_property_read_bool(np, "debug-trace");

	err = of_property_count_elems_of_size(np, "endpoints", sizeof(u32));
	if(err <= 0) {
		dev_err(&pdev->dev, "endpoints list not provided or empty.\n");
		return err;
	}
	am->num_ep = err;

	/* figure out which system endpoints will be built-in */
	for(i=0; i<am->num_ep; i++) {
		err = of_property_read_u32_index(np, "endpoints", i, &epnum);
		if(err < 0)
			return err;
		if(epnum < 32)
			system_ep_mask |= 1 << epnum;
	}

	system_ep_mask = (~system_ep_mask) & AUTO_EP_MASK;
	am->builtin_mask = system_ep_mask;
	num_system_ep = hweight_long(system_ep_mask);

	am->ep = devm_kzalloc(&pdev->dev, sizeof(am->ep[0]) * (am->num_ep + num_system_ep), GFP_KERNEL);
	if(!am->ep)
		return -ENOMEM;

	am->mbchans = devm_kzalloc(&pdev->dev, sizeof(am->mbchans[0]) * (am->num_ep + num_system_ep), GFP_KERNEL);
	if(!am->mbchans)
		return -ENOMEM;

	/* build table of explicitly specified endpoints */
	for(i=0; i<am->num_ep; i++) {
		am->ep[i].am = am;
		err = of_property_read_u32_index(np, "endpoints", i, &am->ep[i].epnum);
		if(err < 0)
			return err;
		if(am->ep[i].epnum > max_ep)
			max_ep = am->ep[i].epnum;

		am->mbchans[i].con_priv = &am->ep[i];
	}

	/* build table of built-in endpoints */
	for(epnum=EP_CRASHLOG; epnum<=EP_IORPT; epnum++) {
		if(!(system_ep_mask & (1 << epnum)))
			continue;
		i = (am->num_ep ++);

		am->ep[i].am = am;
		am->ep[i].epnum = epnum;
		am->ep[i].builtin = true;
		am->ep[i].i2a_live = true;
		if(epnum > max_ep)
			max_ep = epnum;

		am->mbchans[i].con_priv = &am->ep[i];
	}

	max_ep ++;

	am->rev_ep = devm_kzalloc(&pdev->dev, sizeof(u32) * max_ep, GFP_KERNEL);
	if(!am->rev_ep)
		return -ENOMEM;
	am->num_rev_ep = max_ep;
	for(i=0; i<max_ep; i++)
		am->rev_ep[i] = EP_INVALID;
	for(i=0; i<am->num_ep; i++)
		am->rev_ep[am->ep[i].epnum] = i;

	/* if EP0 is called out as a channel, we do not do discovery */
	if(am->rev_ep[0] != EP_INVALID)
		am->ep0_state = EP0_IDLE;
	else
		am->ep0_state = EP0_WAIT_HELLO;

	INIT_WORK(&am->builtin_work, apple_iop_builtin_work_func);

	err = devm_clk_bulk_get_all(am->dev, &am->clks);
	if(err < 0) {
		dev_err(&pdev->dev, "clk_bulk_get_all failed.\n");
		return err;
	}
	am->num_clks = err;

	err = clk_bulk_prepare_enable(am->num_clks, am->clks);
	if(err) {
		dev_err(&pdev->dev, "clk_bulk_prepare_enable failed.\n");
		return err;
	}

	for(i=0; i<NUM_IRQ; i++) {
		irq = platform_get_irq(pdev, i);
		if(irq < 0) {
			dev_err(&pdev->dev, "failed to map IRQ %d\n", i);
			return -EINVAL;
		}
		am->irq[i] = irq;
		if(devm_request_irq(&pdev->dev, irq, apple_iop_mailbox_isr_table[i], 0,
				    dev_name(&pdev->dev), am) < 0) {
			dev_err(&pdev->dev, "failed to request IRQ %d\n", i);
			return -EINVAL;
		}
	}

	am->mbctrl.dev = &pdev->dev;
	am->mbctrl.chans = am->mbchans;
	am->mbctrl.txdone_irq = true;
	am->mbctrl.ops = &apple_iop_mailbox_ops;
	am->mbctrl.num_chans = am->num_ep;
	am->mbctrl.of_xlate = apple_iop_mailbox_of_xlate;

	platform_set_drvdata(pdev, am);

	err = devm_mbox_controller_register(&pdev->dev, &am->mbctrl);
	if(err < 0) {
		platform_set_drvdata(pdev, NULL);
		dev_err(&pdev->dev, "mailbox register failed: %d.\n", err);
		return err;
	}

	spin_lock_irqsave(&am->lock, flags);
	if(am->hwops->try_boot)
		am->hwops->try_boot(am);
	spin_unlock_irqrestore(&am->lock, flags);

	return 0;
}

MODULE_DEVICE_TABLE(of, apple_iop_mailbox_of_match);

static struct platform_driver apple_iop_mailbox_platform_driver = {
	.driver = {
		.name = "apple-iop-mailbox",
		.of_match_table = apple_iop_mailbox_of_match,
	},
	.probe = apple_iop_mailbox_probe,
};
module_platform_driver(apple_iop_mailbox_platform_driver);

MODULE_DESCRIPTION("Apple SoC ASC mailbox driver");
MODULE_LICENSE("GPL v2");
