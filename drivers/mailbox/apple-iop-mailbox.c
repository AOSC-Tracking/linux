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
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>

#define DEBUG_MSG		0

#define NUM_IRQ			2
#define IRQ_A2I_EMPTY		0
#define IRQ_I2A_FULL		1

#define EP_INVALID		(-1u)

enum apple_iop_mailbox_ep0_state {
	EP0_IDLE,
	EP0_WAIT_HELLO,
	EP0_SEND_HELLO,
	EP0_WAIT_EPMAP,
	EP0_SEND_EPACK,
	EP0_SEND_EPSTART,
	EP0_SEND_PWRON,
};

struct apple_iop_mailbox_data {
	struct device *dev;
	void __iomem *base;
	struct mbox_controller mbctrl;
	struct clk_bulk_data *clks;
	int num_clks;
	spinlock_t lock;
	int irq[NUM_IRQ];
	bool a2i_empty_masked;
	/* wait with setup for first message; this is used by IOPs that
	   need custom initialization before the mailbox is live */
	bool wait_init;
	/* table of expected endpoints (index in ep to EP number) */
	struct apple_iop_mailbox_ep {
		struct apple_iop_mailbox_data *am;
		struct list_head a2i_list;
		u64 a2i_msg[2], i2a_msg[2];
		u32 epnum;
		bool a2i_busy;
		bool discovered;
	} *ep;
	struct mbox_chan *mbchans;
	unsigned num_ep;
	/* reverse map of endpoints (EP number to index in ep) */
	u32 *rev_ep;
	unsigned num_rev_ep;
	/* list of endpoints with A2I messages waiting */
	struct list_head a2i_list;
	/* channel currently in A2I buffer */
	unsigned a2i_cur_chan;
	/* EP0 discovery state */
	enum apple_iop_mailbox_ep0_state ep0_state;
	u64 ep0_sub;
};

/* A2I -> AP to IOP, I2A -> IOP to AP */
#define CPU_CTRL		0x0044
#define   CPU_CTRL_RUN		BIT(4)
#define A2I_STAT		0x8110
#define   A2I_STAT_EMPTY	BIT(17)
#define   A2I_STAT_FULL		BIT(16)
#define I2A_STAT		0x8114
#define   I2A_STAT_EMPTY	BIT(17)
#define   I2A_STAT_FULL		BIT(16)
#define   I2A_STAT_ENABLE	BIT(0)
#define A2I_MSG0		0x8800
#define A2I_MSG1		0x8808
#define I2A_MSG0		0x8830
#define I2A_MSG1		0x8838

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
				am->ep0_state = EP0_SEND_PWRON;
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
			am->ep0_state = EP0_SEND_PWRON;
		return 1;
	case EP0_SEND_PWRON:
		msg[0] = 0x00b0000000000020;
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
		if(msgtype == 7) {
			am->ep0_sub = apple_iop_mailbox_ep0_next_discovered_ep(am, 0);
			if(am->ep0_sub == EP_INVALID)
				am->ep0_state = EP0_SEND_PWRON;
			else
				am->ep0_state = EP0_SEND_EPSTART;
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
	if(am->a2i_empty_masked)
		return;
	disable_irq_nosync(am->irq[IRQ_A2I_EMPTY]);
	am->a2i_empty_masked = true;
}

static void apple_iop_mailbox_unmask_a2i_empty(struct apple_iop_mailbox_data *am)
{
	if(!am->a2i_empty_masked)
		return;
	enable_irq(am->irq[IRQ_A2I_EMPTY]);
	am->a2i_empty_masked = false;
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

	stat = readl(am->base + A2I_STAT);
	if(!(stat & A2I_STAT_FULL)) {
		chan = am->a2i_cur_chan;
		am->a2i_cur_chan = EP_INVALID;

		apple_iop_mailbox_mask_a2i_empty(am);
		if(apple_iop_mailbox_ep0_next_a2i(am, msg)) {
#if DEBUG_MSG
			dev_info(am->dev, "tx msg %016llx %16llx\n", msg[0], msg[1]);
#endif
			writeq(msg[0], am->base + A2I_MSG0);
			writeq(msg[1], am->base + A2I_MSG1);
			apple_iop_mailbox_unmask_a2i_empty(am);
		} else if(!list_empty(&am->a2i_list)) {
			ep = list_first_entry(&am->a2i_list, struct apple_iop_mailbox_ep, a2i_list);
			list_del(&ep->a2i_list);

#if DEBUG_MSG
			dev_info(am->dev, "tx msg %016llx %16llx [%d]\n",
				 ep->a2i_msg[0], ep->a2i_msg[1], ep->epnum);
#endif
			writeq(ep->a2i_msg[0], am->base + A2I_MSG0);
			writeq(ep->a2i_msg[1], am->base + A2I_MSG1);
			apple_iop_mailbox_unmask_a2i_empty(am);

			am->a2i_cur_chan = ep - am->ep;
		}
	} else
		dev_err(am->dev, "empty_isr called but a2i_stat is 0x%x - not empty.\n", stat);

	spin_unlock_irqrestore(&am->lock, flags);

	if(chan != EP_INVALID) {
#if DEBUG_MSG
		dev_info(am->dev, "tx complete [%d/%d].\n", am->ep[chan].epnum, chan);
#endif
		am->ep[chan].a2i_busy = false;
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

	stat = readl(am->base + I2A_STAT);
	if(stat & I2A_STAT_EMPTY) {
		dev_err(am->dev, "full_isr called but i2a_stat is 0x%x - not full.\n", stat);
		spin_unlock_irqrestore(&am->lock, flags);
		return IRQ_HANDLED;
	}

	msg[0] = readq(am->base + I2A_MSG0);
	msg[1] = readq(am->base + I2A_MSG1);
	epnum = msg[1] & 0xFF;
#if DEBUG_MSG
	dev_info(am->dev, "rx msg %016llx %16llx\n", msg[0], msg[1]);
#endif

	if(epnum < am->num_rev_ep)
		chan = am->rev_ep[epnum];

	if(chan == EP_INVALID && !epnum) {
		apple_iop_mailbox_ep0_push_i2a(am, msg);

		/* if empty, push the notification state machine */
		stat = readl(am->base + A2I_STAT);
		if(!(stat & A2I_STAT_FULL) && apple_iop_mailbox_ep0_next_a2i(am, msg)) {
#if DEBUG_MSG
			dev_info(am->dev, "tx msg %016llx %16llx\n", msg[0], msg[1]);
#endif
			writeq(msg[0], am->base + A2I_MSG0);
			writeq(msg[1], am->base + A2I_MSG1);
			apple_iop_mailbox_unmask_a2i_empty(am);
		}
	}

	spin_unlock_irqrestore(&am->lock, flags);

	if(chan != EP_INVALID)
		mbox_chan_received_data(&am->mbchans[chan], msg);

	return IRQ_HANDLED;
}

static void apple_iop_mailbox_try_boot(struct apple_iop_mailbox_data *am)
{
	u32 ctrl;

	if(!am->wait_init && am->ep0_state == EP0_WAIT_HELLO) {
		ctrl = readl(am->base + CPU_CTRL);
		if(ctrl & CPU_CTRL_RUN) {
			dev_info(am->dev, "waking coprocessor.\n");
			writeq(0x0060000000000220, am->base + A2I_MSG0);
			writeq(0, am->base + A2I_MSG1);
			apple_iop_mailbox_unmask_a2i_empty(am);
		} else {
			dev_info(am->dev, "booting coprocessor.\n");
			writel(ctrl | CPU_CTRL_RUN, am->base + CPU_CTRL);
		}
	}
}

static int apple_iop_mailbox_send_data(struct mbox_chan *chan, void *_msg)
{
	struct apple_iop_mailbox_ep *ep = chan->con_priv;
	struct apple_iop_mailbox_data *am = ep->am;
	unsigned long flags;
	u64 *msg = _msg, msg1;
	u32 stat;

	spin_lock_irqsave(&am->lock, flags);

	if(am->wait_init) {
		am->wait_init = false;
		am->ep0_state = EP0_WAIT_HELLO;
		apple_iop_mailbox_try_boot(am);
	}

	if(ep->a2i_busy) {
		spin_unlock_irqrestore(&am->lock, flags);
		return -EBUSY;
	}

	msg1 = (msg[1] & ~0xFFul) | ep->epnum;

	if(am->a2i_cur_chan != EP_INVALID || am->ep0_state != EP0_IDLE) {
		ep->a2i_msg[0] = msg[0];
		ep->a2i_msg[1] = msg1;
		ep->a2i_busy = true;
		list_add(&ep->a2i_list, &am->a2i_list);
	} else {
		stat = readl(am->base + A2I_STAT);
		if(!(stat & A2I_STAT_FULL)) {
#if DEBUG_MSG
			dev_info(am->dev, "tx msg %016llx %16llx [%d]\n",
				 msg[0], msg[1], ep->epnum);
#endif
			writeq(msg[0], am->base + A2I_MSG0);
			writeq(msg1, am->base + A2I_MSG1);
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

static const struct mbox_chan_ops apple_iop_mailbox_ops = {
	.send_data = apple_iop_mailbox_send_data,
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

static int apple_iop_mailbox_probe(struct platform_device *pdev)
{
	struct apple_iop_mailbox_data *am;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	unsigned int i, irq, max_ep = 0;
	unsigned long flags;
	int err;

	am = devm_kzalloc(&pdev->dev, sizeof(*am), GFP_KERNEL);
	if(!am)
		return -ENOMEM;
	am->dev = &pdev->dev;
	spin_lock_init(&am->lock);
	INIT_LIST_HEAD(&am->a2i_list);
	am->a2i_cur_chan = EP_INVALID;

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

	err = of_property_count_elems_of_size(np, "endpoints", sizeof(u32));
	if(err <= 0) {
		dev_err(&pdev->dev, "endpoints list not provided or empty.\n");
		return err;
	}
	am->num_ep = err;

	am->ep = devm_kzalloc(&pdev->dev, sizeof(am->ep[0]) * am->num_ep, GFP_KERNEL);
	if(!am->ep)
		return -ENOMEM;

	am->mbchans = devm_kzalloc(&pdev->dev, sizeof(am->mbchans[0]) * am->num_ep, GFP_KERNEL);
	if(!am->mbchans)
		return -ENOMEM;

	for(i=0; i<am->num_ep; i++) {
		am->ep[i].am = am;
		err = of_property_read_u32_index(np, "endpoints", i, &am->ep[i].epnum);
		if(err < 0)
			return err;
		if(am->ep[i].epnum > max_ep)
			max_ep = am->ep[i].epnum;

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
	apple_iop_mailbox_try_boot(am);
	spin_unlock_irqrestore(&am->lock, flags);

	return 0;
}

static const struct of_device_id apple_iop_mailbox_of_match[] = {
	{ .compatible = "apple,iop-mailbox-m1" },
	{ }
};
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
