// SPDX-License-Identifier: GPL-2.0+
/*
 * CIX DSP IPC interface driver (host side)
 *
 * This driver provides mailbox-based IPC communication helpers for
 * the Sound Open Firmware (SOF) driver to communicate with the DSP.
 *
 * Copyright 2024 Cix Technology Group Co., Ltd.
 */

#include <linux/firmware/cix/dsp.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>

static const char * const dsp_mbox_ch_names[CIX_DSP_MBOX_NUM] = {
	"txdb",
	"rxdb"
};

/**
 * cix_dsp_ipc_send - Send an IPC message to the DSP
 * @ipc: DSP IPC context
 * @idx: Mailbox channel index (CIX_DSP_IPC_REQ or CIX_DSP_IPC_REP)
 * @msg: Message to send
 *
 * Returns 0 on success, negative error code on failure.
 */
int cix_dsp_ipc_send(struct cix_dsp_ipc *ipc, unsigned int idx, u32 msg)
{
	struct cix_dsp_chan *dsp_chan;
	int ret;

	if (idx >= CIX_DSP_MBOX_NUM)
		return -EINVAL;

	dsp_chan = &ipc->chans[idx];
	ret = mbox_send_message(dsp_chan->ch, &msg);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(cix_dsp_ipc_send);

static void cix_dsp_rx_callback(struct mbox_client *cl, void *msg)
{
	struct cix_dsp_chan *chan = container_of(cl, struct cix_dsp_chan, cl);
	struct device *dev = cl->dev;

	switch (chan->idx) {
	case CIX_DSP_MBOX_REPLY:
		chan->ipc->ops->handle_reply(chan->ipc);
		mbox_client_txdone(chan->ch, 0);
		break;
	case CIX_DSP_MBOX_REQUEST:
		chan->ipc->ops->handle_request(chan->ipc);
		break;
	default:
		dev_err(dev, "wrong mbox chan %d\n", chan->idx);
		break;
	}
}

/**
 * cix_dsp_request_mbox - Request mailbox channels for DSP IPC
 * @dsp_ipc: DSP IPC context
 *
 * Sets up the TX (request) and RX (response) mailbox channels for
 * bidirectional communication with the DSP.
 *
 * Returns 0 on success, negative error code on failure.
 */
int cix_dsp_request_mbox(struct cix_dsp_ipc *dsp_ipc)
{
	struct device *dev = dsp_ipc->dev;
	struct cix_dsp_chan *dsp_chan;
	struct mbox_client *cl;
	int i, j;
	int ret;

	/*
	 * Communication model:
	 * AP req -- txdb --> DSP
	 *    AP <-- txdb --  DSP rsp
	 *    AP <-- rxdb --  DSP req
	 * AP rsp -- rxdb --> DSP
	 */
	for (i = 0; i < CIX_DSP_MBOX_NUM; i++) {
		dsp_chan = &dsp_ipc->chans[i];
		cl = &dsp_chan->cl;
		cl->dev = dev;
		cl->tx_block = false;
		cl->knows_txdone = false;
		cl->tx_prepare = NULL;
		cl->rx_callback = cix_dsp_rx_callback;

		dsp_chan->ipc = dsp_ipc;
		dsp_chan->idx = i;
		dsp_chan->ch = mbox_request_channel_byname(cl,
							   dsp_mbox_ch_names[i]);
		if (IS_ERR(dsp_chan->ch)) {
			ret = PTR_ERR(dsp_chan->ch);
			if (ret != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to request mbox chan %s ret %d\n",
					dsp_mbox_ch_names[i], ret);

			for (j = 0; j < i; j++) {
				dsp_chan = &dsp_ipc->chans[j];
				mbox_free_channel(dsp_chan->ch);
			}

			return ret;
		}
	}

	dsp_ipc->dev = dev;

	dev_dbg(dev, "CIX DSP IPC mailbox channels requested\n");

	return 0;
}
EXPORT_SYMBOL(cix_dsp_request_mbox);

/**
 * cix_dsp_free_mbox - Free mailbox channels
 * @dsp_ipc: DSP IPC context
 */
void cix_dsp_free_mbox(struct cix_dsp_ipc *dsp_ipc)
{
	struct cix_dsp_chan *dsp_chan;
	int i;

	for (i = 0; i < CIX_DSP_MBOX_NUM; i++) {
		dsp_chan = &dsp_ipc->chans[i];
		mbox_free_channel(dsp_chan->ch);
	}

	dev_dbg(dsp_ipc->dev, "CIX DSP IPC mailbox channels freed\n");
}
EXPORT_SYMBOL(cix_dsp_free_mbox);

static int cix_dsp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cix_dsp_ipc *dsp_ipc;

	device_set_node(&pdev->dev, pdev->dev.parent->fwnode);

	dsp_ipc = devm_kzalloc(dev, sizeof(*dsp_ipc), GFP_KERNEL);
	if (!dsp_ipc)
		return -ENOMEM;

	dsp_ipc->dev = dev;
	dev_set_drvdata(dev, dsp_ipc);

	dev_dbg(dev, "CIX DSP IPC initialized\n");

	return 0;
}

static struct platform_driver cix_dsp_driver = {
	.driver = {
		.name = "cix-dsp",
	},
	.probe = cix_dsp_probe,
};
builtin_platform_driver(cix_dsp_driver);

MODULE_AUTHOR("Joakim Zhang <joakim.zhang@cixtech.com>");
MODULE_DESCRIPTION("CIX DSP IPC Driver");
MODULE_LICENSE("GPL");
