/*
 * Freescale MPC5200 PSC in I2S mode
 * ALSA SoC Digital Audio Interface (DAI) driver
 *
 * Copyright (C) 2008 Secret Lab Technologies Ltd.
 * Copyright (C) 2009 Jon Smirl, Digispeaker
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "ls-lib.h"
#include <uapi/linux/pci.h>

#include <linux/pci_ids.h>
#include <linux/pci.h>
#include <linux/acpi.h>

#include "ls-i2s.h"
#include "ls-pcm.h"

#define PCM_STEREO_OUT_NAME "I2S PCM Stereo out"
#define PCM_STEREO_IN_NAME "I2S PCM Stereo in"

#define IISRXADDR       0x0c
#define IISTXADDR       0x10

static struct ls_pcm_dma_params ls_i2s_pcm_stereo_out;
static struct ls_pcm_dma_params ls_i2s_pcm_stereo_in;
static unsigned int revision_id;

struct ls_i2s {
	bool slave_mode;
	bool mclk_external;
	bool mclk_fixed;
	unsigned int refclk_freq;
	unsigned int mclk_freq;
	unsigned int xfs;
};

static struct ls_i2s g_ls_i2s;

bool ls_i2s_is_slave_mode(void)
{
	return g_ls_i2s.slave_mode;
}

static int ls_i2s_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	return 0;
}

static int ls_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct ls_pcm_dma_params *dma_data;
	void *i2sbase;
	unsigned int mclk_ratio;
	unsigned int mclk_ratio_frac;
	unsigned int bclk_ratio;
	unsigned int wlen = params_width(params);
	unsigned int depth = params_width(params);
	unsigned int fs = params->rate_num;
	unsigned int xfs = g_ls_i2s.xfs;
	unsigned int mclk_freq = g_ls_i2s.mclk_freq;
	unsigned int bclk_freq = 0;
	unsigned int refclk_freq = g_ls_i2s.refclk_freq;
	unsigned int val = (wlen<<24) | (depth<<16);

	i2sbase = dai->dev->platform_data;

	if (!g_ls_i2s.mclk_external) {
		if (!g_ls_i2s.mclk_fixed) {
			mclk_freq = xfs * fs;
		}

		if (revision_id == 0)
		{
			mclk_ratio = DIV_ROUND_CLOSEST(refclk_freq, mclk_freq * 2) - 1;
			val |= (mclk_ratio<<0);
		}
		else
		{
			mclk_ratio = refclk_freq / mclk_freq;
			mclk_ratio_frac = (u64)(refclk_freq - mclk_ratio * mclk_freq)
				* 65536 / mclk_freq;
		}
	}

	if (!g_ls_i2s.slave_mode) {
		bclk_freq = depth * 2 * fs;
		if (revision_id == 0)
			bclk_ratio = DIV_ROUND_CLOSEST(refclk_freq, bclk_freq * 2) - 1;
		else
			bclk_ratio = DIV_ROUND_CLOSEST(mclk_freq, bclk_freq * 2) - 1;

		val |= (bclk_ratio<<8);
	}

	if (revision_id == 1)
		val |= depth;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &ls_i2s_pcm_stereo_out;
	else
		dma_data = &ls_i2s_pcm_stereo_in;

	snd_soc_dai_set_dma_data(dai, substream, dma_data);

	writel(val, i2sbase + 0x4);
	if (revision_id == 1)
		writel((mclk_ratio_frac << 16) | mclk_ratio, i2sbase + 0x14);

	return 0;
}

static int ls_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
			      int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int ls_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int format)
{
	return 0;
}

static int ls_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	struct ls_runtime_data *prtd = substream->runtime->private_data;
	uint32_t data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			data = readl((void*)prtd->i2s_ctl_base);
			data &= ~(0x1 << 12);
			writel(data, (void*)prtd->i2s_ctl_base);
			data = readl((void*)prtd->i2s_ctl_base);
			if (ls_i2s_is_slave_mode())
				data |= 0x4010 | (1 << 7) | (1 << 12);
			else
				data |= 0xc010 | (1 << 7) | (1 << 12);
			writel(data, (void*)prtd->i2s_ctl_base);
		} else {
			data = readl((void*)prtd->i2s_ctl_base);
			if (ls_i2s_is_slave_mode())
				data |= 0x4010 | (1 << 11) | (1 << 13);
			else
				data |= 0xc010 | (1 << 11) | (1 << 13);
			writel(data, (void*)prtd->i2s_ctl_base);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			data = readl((void*)prtd->i2s_ctl_base);
			data &= ~((1 << 7) | (1 << 12));
			writel(data, (void*)prtd->i2s_ctl_base);
		} else {
			data = readl((void*)prtd->i2s_ctl_base);
			data &= ~((1 << 13) | (1 << 11));
			writel(data, (void*)prtd->i2s_ctl_base);
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void ls_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{

}

static int ls_i2s_probe(struct snd_soc_dai *dai)
{
	return 0;
}

static int  ls_i2s_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops psc_i2s_dai_ops = {
	.probe		= ls_i2s_probe,
	.remove		= ls_i2s_remove,
	.startup	= ls_i2s_startup,
	.shutdown	= ls_i2s_shutdown,
	.trigger	= ls_i2s_trigger,
	.hw_params	= ls_i2s_hw_params,
	.set_fmt	= ls_i2s_set_dai_fmt,
	.set_sysclk	= ls_i2s_set_dai_sysclk,
};

static struct snd_soc_dai_driver psc_i2s_dai[] = {{
	.name = "loongson-i2s-dai",
	.playback = {
		.stream_name = "I2S Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PSC_I2S_RATES,
		.formats = PSC_I2S_FORMATS,
	}, .capture = {
		.stream_name = "I2S Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PSC_I2S_RATES,
		.formats = PSC_I2S_FORMATS,
	},
	.ops = &psc_i2s_dai_ops,
	.symmetric_rate = 1,
} };

static const struct snd_soc_component_driver psc_i2s_component = {
	.name		= "loongson-i2s-dai",
};

static int ls_i2s_drv_probe(struct platform_device *pdev)
{
	uint64_t IISRxData, IISTxData;
	struct clk* refclk;

#if 0 // 4.19 与 5.10 不兼容代码
	if (ACPI_COMPANION(&pdev->dev)) {
		resource_size_t mmio_base, mmio_size;
		struct pci_dev *ppdev;
		static void __iomem *pci_i2s_reg;
		int ret;

		ppdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_I2S, NULL);

		if (ppdev) {
			ret = pci_enable_device(ppdev);

			mmio_base = pci_resource_start(ppdev, 0);
			mmio_size = pci_resource_len(ppdev, 0);
			pci_i2s_reg = ioremap(mmio_base, mmio_size);
			revision_id = ppdev->revision;
		}

		pdev->dev.platform_data = pci_i2s_reg;
		IISRxData = (uint64_t)pci_i2s_reg | IISRXADDR;
		IISTxData = (uint64_t)pci_i2s_reg | IISTXADDR;
	} else {
#endif
		struct device_node *np = pdev->dev.of_node;
		struct resource *r;
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

		if (r == NULL) {
			dev_err(&pdev->dev, "no IO memory resource defined\n");
			return -ENODEV;
		}

		pdev->dev.platform_data = ioremap(r->start, r->end - r->start + 1);;
		IISRxData = r->start | IISRXADDR;
		IISTxData = r->start | IISTXADDR;

		if (of_device_is_compatible(np, "loongson,ls2k0300-i2s"))
			revision_id = 1;
#if 0
	}
#endif

	refclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(refclk))
	{
		if (device_property_read_u32(&pdev->dev, "clock-frequency",
				&g_ls_i2s.refclk_freq))
		{
			dev_err(&pdev->dev, "Clock Get Fail!\n");
			return -ENODEV;
		}
	}
	else
	{
		g_ls_i2s.refclk_freq = clk_get_rate(refclk);
	}

	pdev->dev.kobj.name = "loongson-i2s-dai";

	if (device_property_present(&pdev->dev, "slave-mode")) {
		g_ls_i2s.slave_mode = true;
		dev_info(&pdev->dev, "Choose SLAVE-MODE...\n");
	}
	else {
		g_ls_i2s.slave_mode = false;
	}

	if (device_property_present(&pdev->dev, "mclk-external")) {
		g_ls_i2s.mclk_external = true;
		dev_info(&pdev->dev, "Use EXTERNAL-MCLK...\n");
	}
	else {
		g_ls_i2s.mclk_external = false;
	}

	if (device_property_present(&pdev->dev, "mclk-fixed")) {
		g_ls_i2s.mclk_fixed = true;
		device_property_read_u32(&pdev->dev, "mclk-fixed",
				&g_ls_i2s.mclk_freq);
		dev_info(&pdev->dev, "Set MCLK-FIXED <%d> ...\n",
				g_ls_i2s.mclk_freq);
	}
	else {
		g_ls_i2s.mclk_fixed = false;
	}

	if(device_property_read_u32(&pdev->dev, "xfs", &g_ls_i2s.xfs))
		g_ls_i2s.xfs = 128;
	dev_info(&pdev->dev, "Set xfs <%d> ...\n", g_ls_i2s.xfs);

	ls_i2s_pcm_stereo_out.name = PCM_STEREO_OUT_NAME;
	ls_i2s_pcm_stereo_out.dev_addr = IISTxData;

	ls_i2s_pcm_stereo_in.name = PCM_STEREO_IN_NAME;
	ls_i2s_pcm_stereo_in.dev_addr = IISRxData;

	return snd_soc_register_component(&pdev->dev, &psc_i2s_component,
	psc_i2s_dai, 1);
}

static void ls_i2s_drv_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
}

static const struct of_device_id snd_ls_i2s_dt_match[] = {
    { .compatible = "loongson,ls-i2s", },
    { .compatible = "loongson,ls2k0300-i2s", },
    { .compatible = "loongson,loongson2-i2s", },
    { .compatible = "loongson,ls7a-i2s", },
    {},
};
MODULE_DEVICE_TABLE(of, snd_ls_i2s_dt_match);

static struct platform_driver ls_i2s_driver = {
	.probe = ls_i2s_drv_probe,
	.remove = ls_i2s_drv_remove,
	.driver = {
		.name = "loongson-i2sp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(snd_ls_i2s_dt_match),
	},
};

static int __init loongson_i2s_init(void)
{
	return platform_driver_register(&ls_i2s_driver);
}

static void __exit loongson_i2s_exit(void)
{
	platform_driver_unregister(&ls_i2s_driver);
}

module_init(loongson_i2s_init)
module_exit(loongson_i2s_exit)

MODULE_AUTHOR("loongson");
MODULE_DESCRIPTION("Loongson I2S mode ASoC Driver");
MODULE_LICENSE("GPL");

