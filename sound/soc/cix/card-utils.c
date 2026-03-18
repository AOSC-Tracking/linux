// SPDX-License-Identifier: GPL-2.0
// Copyright 2024 Cix Technology Group Co., Ltd.

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include "card-utils.h"
#include <sound/jack.h>

#define SKY1_AUDSS_CRU_INFO_MCLK		0x70
#define SKY1_AUDSS_CRU_INFO_MCLK_DIV_OFF(x)	(10 + (3 * (x)))
#define SKY1_AUDSS_CRU_INFO_MCLK_DIV_MASK(x)	GENMASK((12 + (3 * (x))), (10 + (3 * (x))))

static const char *mclk_pll_names[AUDIO_CLK_NUM] = {
	[AUDIO_CLK0] = "audio_clk0",
	[AUDIO_CLK2] = "audio_clk2",
};

static int cix_jack_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct dai_link_info *link_info =
		(struct dai_link_info *)((priv->link_info + rtd->id));
	struct device *dev = rtd->card->dev;
	int ret, jack[JACK_CNT], cnt, i;

	if (!link_info->jack_det_mask)
		return 0;

	cnt = 0;
	if (link_info->jack_det_mask & JACK_MASK_DPIN) {
		jack[cnt] = JACK_DPIN;
		cnt++;
	}

	if (link_info->jack_det_mask & JACK_MASK_DPOUT) {
		jack[cnt] = JACK_DPOUT;
		cnt++;
	}

	if (link_info->jack_det_mask & JACK_MASK_HP) {
		jack[cnt] = JACK_HP;
		cnt++;
	}

	for (i = 0; i < cnt; i++) {
		ret = snd_soc_card_jack_new(rtd->card,
				link_info->jack_pin[jack[i]].pin,
				link_info->jack_pin[jack[i]].mask,
				&link_info->jack[jack[i]]);
		if (ret) {
			dev_err(dev, "can't new jack:%d, %d\n", i, ret);
			return ret;
		}
		dev_info(dev, "codec component %s\n", codec_dai->component->name);

		snd_soc_component_set_jack(codec_dai->component,
						&link_info->jack[jack[i]], NULL);
	}

	return 0;
}

static int cix_dailink_parsing_fmt(struct device_node *np,
				   struct device_node *codec_np,
				   unsigned int *fmt)
{
	struct device_node *bitclkmaster = NULL;
	struct device_node *framemaster = NULL;
	unsigned int dai_fmt;

	dai_fmt = snd_soc_daifmt_parse_format(np, NULL);

	snd_soc_daifmt_parse_clock_provider_as_phandle(np, NULL,
						       &bitclkmaster,
						       &framemaster);
	if (bitclkmaster != framemaster) {
		pr_info("Must be the same bitclock and frame master\n");
		of_node_put(bitclkmaster);
		of_node_put(framemaster);
		return -EINVAL;
	}
	if (bitclkmaster) {
		dai_fmt &= ~SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK;
		if (codec_np == bitclkmaster)
			dai_fmt |= SND_SOC_DAIFMT_CBP_CFP;
		else
			dai_fmt |= SND_SOC_DAIFMT_CBC_CFC;
	} else {
		/*
		 * No clock master specified - default to codec as consumer
		 * (CPU/I2S provides clocks). This matches vendor kernel
		 * behavior from snd_soc_daifmt_clock_provider_from_bitmap(0).
		 */
		dai_fmt |= SND_SOC_DAIFMT_CBC_CFC;
	}
	of_node_put(bitclkmaster);
	of_node_put(framemaster);
	*fmt = dai_fmt;

	return 0;
}

/* only disabled dai-link status, not continue to parse */
static bool cix_dailink_status_check(struct device_node *np)
{
	const char *status;
	int ret;

	ret = of_property_read_string(np, "status", &status);

	if (status) {
		if (!strcmp(status, "okay") || !strcmp(status, "ok"))
			return true;
		else
			return false;
	}

	return true;
}

static int dai_set_sysclk(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  unsigned int mclk_fs)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct dai_link_info *link_info =
		(struct dai_link_info *)((priv->link_info + rtd->id));
	struct snd_soc_dai *codec_dai;
	unsigned int mclk, mclk_div, mclk_parent_rate, sample_rate;
	struct clk *mclk_parent;
	int ret, i;
	u32 val;

	if (!mclk_fs || !priv->cru_regmap)
		return 0;

	sample_rate = params_rate(params);

	dev_dbg(rtd->dev, "sample rate:%d, mclk-fs ratio:%d\n", sample_rate, mclk_fs);

	/* Adjust mclk_fs due to mclk design limitation */
	if (sample_rate % 8000 == 0) {
		if (sample_rate < 32000 || sample_rate > 192000) {
			dev_err(rtd->dev,
				"8khz_pll cannot satisfy mclk less than 32khz or large than 192khz");
			return -EINVAL;
		}

		if (sample_rate == 32000) {
			mclk_fs = 512;
			mclk_div = 3;
		} else if (sample_rate == 48000) {
			mclk_fs = 512;
			mclk_div = 2;
		} else if (sample_rate == 64000) {
			mclk_fs = 256;
			mclk_div = 3;
		} else if (sample_rate == 96000) {
			mclk_fs = 256;
			mclk_div = 2;
		} else if (sample_rate == 192000) {
			mclk_fs = 256;
			mclk_div = 0;
		}

		mclk_parent = link_info->clks[AUDIO_CLK0];
	} else if (sample_rate % 11025 == 0) {
		if (sample_rate < 44100 || sample_rate > 176400) {
			dev_err(rtd->dev,
				"11.025khz_pll cannot satisfy mclk less than 44.1khz or large than 176.4khz");
			return -EINVAL;
		}

		if (sample_rate == 44100) {
			mclk_fs = 512;
			mclk_div = 2;
		} else if (sample_rate == 88200) {
			mclk_fs = 256;
			mclk_div = 2;
		} else if (sample_rate == 176400) {
			mclk_fs = 256;
			mclk_div = 0;
		}

		mclk_parent = link_info->clks[AUDIO_CLK2];
	} else {
		dev_err(rtd->dev, "invalid sample rate");
		return -EINVAL;
	}

	mclk_parent_rate = clk_get_rate(mclk_parent);
	dev_dbg(rtd->dev, "mclk parent rate = %d\n", mclk_parent_rate);

	mclk = sample_rate * mclk_fs;

	dev_dbg(rtd->dev, "mclk-fs ratio:%d, mclk-div:%d, mclk freq:%d\n",
		 mclk_fs, mclk_div, mclk);

	/* for cpu dai */
	regmap_read(priv->cru_regmap, SKY1_AUDSS_CRU_INFO_MCLK, &val);
	val &= ~SKY1_AUDSS_CRU_INFO_MCLK_DIV_MASK(link_info->mclk_idx);
	val |= (mclk_div << SKY1_AUDSS_CRU_INFO_MCLK_DIV_OFF(link_info->mclk_idx));
	regmap_write(priv->cru_regmap, SKY1_AUDSS_CRU_INFO_MCLK, val);

	ret = clk_set_parent(link_info->clk_mclk, mclk_parent);
	if (ret) {
		dev_err(rtd->dev, "failed to set mclk parent\n");
		return ret;
	}

	/* for codec dai */
	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
				SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

static int dai_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct dai_link_info *link_info =
		(struct dai_link_info *)((priv->link_info + rtd->id));
	int ret;

	if (link_info->mclk_fs) {
		ret = clk_prepare_enable(link_info->clk_mclk);
		if (ret) {
			dev_err(rtd->dev, "failed to enable mclk\n");
			return ret;
		}

		link_info->mclk_enabled = true;
	}

	return 0;
}

static void dai_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct dai_link_info *link_info =
		(struct dai_link_info *)((priv->link_info + rtd->id));

	if (link_info->mclk_fs) {
		link_info->mclk_enabled = false;

		clk_disable_unprepare(link_info->clk_mclk);
	}
}

static int dai_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct dai_link_info *link_info =
		(struct dai_link_info *)((priv->link_info + rtd->id));

	if (link_info->mclk_fs)
		return dai_set_sysclk(substream, params, link_info->mclk_fs);

	return 0;
}

static struct snd_soc_ops cix_dailink_ops = {
	.startup = dai_startup,
	.hw_params = dai_hw_params,
	.shutdown = dai_shutdown,
};

static int cix_dailink_init(struct snd_soc_pcm_runtime *rtd)
{
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct device *dev = rtd->card->dev;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct dai_link_info *link_info =
		(struct dai_link_info *)((priv->link_info + rtd->id));
	struct snd_soc_dai_link *dai_link =
		(struct snd_soc_dai_link *)((priv)->card->dai_link + rtd->id);
	int ret, fmt;

	dev_dbg(dev, "%s, dai_fmt:0x%x\n", __func__, dai_link->dai_fmt);

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_link->dai_fmt);
	if (ret && ret != -ENOTSUPP)
	      return ret;

	fmt = dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	if (fmt == SND_SOC_DAIFMT_DSP_A || fmt == SND_SOC_DAIFMT_DSP_B) {
		dev_dbg(dev,
			"\ttdm tx mask:0x%x, rx mask:0x%x, slots:%d, slot width:%d\n",
			link_info->tx_mask, link_info->rx_mask,
			link_info->slots, link_info->slot_width);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, link_info->tx_mask,
				link_info->rx_mask, link_info->slots, link_info->slot_width);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	cix_jack_init(rtd);

	return 0;
}

static int cix_gpio_init(struct cix_asoc_card *priv)
{
	struct snd_soc_card *card = priv->card;
	struct device *dev = card->dev;
	int ret;

	priv->pdb0_gpiod = devm_gpiod_get_optional(dev, "pdb0", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pdb0_gpiod)) {
		ret = PTR_ERR(priv->pdb0_gpiod);
		dev_err(dev, "failed to pdb gpio, ret: %d\n", ret);
		return ret;
	}

	priv->pdb1_gpiod = devm_gpiod_get_optional(dev, "pdb1", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pdb1_gpiod)) {
		ret = PTR_ERR(priv->pdb1_gpiod);
		dev_err(dev, "failed to get amplifier gpio: %d\n", ret);
		return ret;
	}

	priv->pdb2_gpiod = devm_gpiod_get_optional(dev, "pdb2", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pdb2_gpiod)) {
		ret = PTR_ERR(priv->pdb2_gpiod);
		dev_err(dev, "failed to get amplifier gpio: %d\n", ret);
		return ret;
	}

	priv->pdb3_gpiod = devm_gpiod_get_optional(dev, "pdb3", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pdb3_gpiod)) {
		ret = PTR_ERR(priv->pdb3_gpiod);
		dev_err(dev, "failed to get amplifier gpio: %d\n", ret);
		return ret;
	}

	priv->beep_gpiod = devm_gpiod_get_optional(dev, "beep", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->beep_gpiod)) {
		ret = PTR_ERR(priv->beep_gpiod);
		dev_err(dev, "failed to beep gpio, ret: %d\n", ret);
		return ret;
	}

	priv->codec_gpiod = devm_gpiod_get_optional(dev, "codec", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->codec_gpiod)) {
		ret = PTR_ERR(priv->codec_gpiod);
		dev_err(dev, "failed to codec gpio, ret: %d\n", ret);
		return ret;
	}

	priv->i2sint_gpiod = devm_gpiod_get_optional(dev, "i2sint", GPIOD_IN);
	if (IS_ERR(priv->i2sint_gpiod)) {
		ret = PTR_ERR(priv->i2sint_gpiod);
		dev_err(dev, "failed to i2s int gpio, ret: %d\n", ret);
		return ret;
	}

	priv->mclk_gpiod = devm_gpiod_get_optional(dev, "mclkext", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->mclk_gpiod)) {
		ret = PTR_ERR(priv->mclk_gpiod);
		dev_err(dev, "failed to get mclk gpio, ret: %d\n", ret);
		return ret;
	}

	priv->hpmicdet_gpiod = devm_gpiod_get_optional(dev, "hpmicdet", GPIOD_IN);
	if (IS_ERR(priv->hpmicdet_gpiod)) {
		ret = PTR_ERR(priv->hpmicdet_gpiod);
		dev_err(dev, "failed to get hp mic detect gpio, ret: %d\n", ret);
		return ret;
	}

	return 0;
}

static int cix_card_suspend_post(struct snd_soc_card *card)
{
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct dai_link_info *link_info;

	for_each_card_rtds(card, rtd) {
		link_info = (struct dai_link_info *)((priv->link_info + rtd->id));

		if (link_info->mclk_enabled)
			clk_disable_unprepare(link_info->clk_mclk);
	}

	return 0;
}

static int cix_card_resume_pre(struct snd_soc_card *card)
{
	struct cix_asoc_card *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct dai_link_info *link_info;
	int ret;

	for_each_card_rtds(card, rtd) {
		link_info = (struct dai_link_info *)((priv->link_info + rtd->id));

		if (link_info->mclk_enabled) {
			ret = clk_prepare_enable(link_info->clk_mclk);
			if (ret) {
				dev_err(rtd->dev, "failed to enable mclk\n");
				return ret;
			}
		}
	}

	return 0;
}

int cix_card_parse_of(struct cix_asoc_card *priv)
{
	struct snd_soc_card *card = priv->card;
	struct device *dev = card->dev;
	struct device_node *np;
	struct device_node *codec;
	struct device_node *cpu;
	struct snd_soc_dai_link *link;
	struct dai_link_info *link_info;
	struct of_phandle_args args;
	struct snd_soc_dai_link_component *comp_cpu;
	int i, ret, num_links, current_np;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret) {
		dev_err(dev, "error parsing card name: %d\n", ret);
		return ret;
	}

	ret = cix_gpio_init(priv);
	if (ret) {
		dev_err(dev, "failed to init gpio: %d\n", ret);
		return ret;
	}

	priv->cru_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
							   "cru-ctrl");
	if (PTR_ERR(priv->cru_regmap) == -ENODEV) {
		priv->cru_regmap = NULL;
	} else if (IS_ERR(priv->cru_regmap)) {
		return PTR_ERR(priv->cru_regmap);
	}

	num_links = of_get_child_count(dev->of_node);

	link = devm_kcalloc(dev, num_links, sizeof(*link), GFP_KERNEL);
	if (!link)
		return -ENOMEM;
	link_info = devm_kcalloc(dev, num_links, sizeof(*link_info), GFP_KERNEL);
	if (!link_info)
		return -ENOMEM;

	/* DMA trigger ordering: start before I2S, stop after I2S */
	link->trigger_start = SND_SOC_TRIGGER_ORDER_DEFAULT;
	link->trigger_stop = SND_SOC_TRIGGER_ORDER_LDC;

	card->num_links = num_links;
	card->dai_link = link;
	card->suspend_post = cix_card_suspend_post;
	card->resume_pre = cix_card_resume_pre;
	priv->link_info = link_info;

	current_np = 0;
	for_each_child_of_node(dev->of_node, np) {
		if (!cix_dailink_status_check(np)) {
			num_links--;
			card->num_links = num_links;
			continue;
		}

		comp_cpu = devm_kzalloc(dev, 2 * sizeof(*comp_cpu), GFP_KERNEL);
		if (!comp_cpu) {
			ret = -ENOMEM;
			goto err_put_np;
		}

		cpu = of_get_child_by_name(np, "cpu");
		if (!cpu) {
			dev_err(dev, "can't find cpu device node\n");
			ret = -EINVAL;
			goto err_put_cpu;
		}

		ret = of_parse_phandle_with_args(cpu, "sound-dai",
						 "#sound-dai-cells", 0, &args);
		if (ret) {
			dev_err(dev, "%s: error getting cpu phandle\n", cpu->name);
			goto err_put_cpu;
		}

		link->cpus = &comp_cpu[0];
		link->num_cpus = 1;
		link->cpus->of_node = args.np;
		link->id = args.args[0];

		link->platforms	= &comp_cpu[1];
		link->num_platforms = 1;
		link->platforms->of_node = link->cpus->of_node;

		dev_info(dev, "dai-link name:%s\n", np->name);

		ret = snd_soc_of_get_dai_name(cpu, &link->cpus->dai_name, 0);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(card->dev, "%s: error getting cpu dai name: %d\n",
					link->name, ret);
			goto err_put_cpu;
		}
		dev_info(dev, "\tcpu dai name:%s\n", link->cpus->dai_name);

		codec = of_get_child_by_name(np, "codec");
		if (codec) {
			ret = snd_soc_of_get_dai_link_codecs(dev, codec, link);
			if (ret < 0) {
				if (ret != -EPROBE_DEFER)
					dev_err(card->dev, "%s: codec dai not found: %d\n",
						link->name, ret);
				goto err_put_codec;
			}
		} else {
			struct snd_soc_dai_link_component *comp_codec;

			comp_codec = devm_kzalloc(dev, sizeof(*comp_codec), GFP_KERNEL);
			if (!comp_codec) {
				ret = -ENOMEM;
				goto err_put_codec;
			}

			link->num_codecs = 1;
			link->codecs = comp_codec;
			link->codecs->dai_name = "snd-soc-dummy-dai";
			link->codecs->name = "snd-soc-dummy";
		}
		dev_info(dev, "\tcodec dai name:%s\n", link->codecs->dai_name );

		cix_dailink_parsing_fmt(np, codec, &link->dai_fmt);

		of_property_read_u32(np, "mclk-fs", &link_info->mclk_fs);
		if (link_info->mclk_fs) {
			link_info->clk_mclk = devm_get_clk_from_child(dev, np, "mclk");
			if (IS_ERR(link_info->clk_mclk)) {
				dev_err(dev, "failed to get clk_mclk clock\n");
				return PTR_ERR(link_info->clk_mclk);
			}

			for (i = 0; i < AUDIO_CLK_NUM; i++) {
				link_info->clks[i] = devm_get_clk_from_child(dev, np, mclk_pll_names[i]);
				if (IS_ERR(link_info->clks[i])) {
					dev_err(dev, "failed to get clock %s\n", mclk_pll_names[i]);
					return PTR_ERR(link_info->clks[i]);
				}
			}

			ret = of_property_read_u8(np, "mclk-idx", &link_info->mclk_idx);
			if (ret) {
				dev_err(dev, "failed to get mclk-idx: %d", ret);
				return ret;
			}
		}

		snd_soc_of_parse_tdm_slot(np,
					  &link_info->tx_mask,
					  &link_info->rx_mask,
					  &link_info->slots,
					  &link_info->slot_width);

		if (of_property_read_bool(np, "jack-det,dpin")) {
			link_info->jack_pin[JACK_DPIN].pin = np->name ? np->name : "jack-dpin";
			link_info->jack_pin[JACK_DPIN].mask = SND_JACK_LINEIN;
			link_info->jack_det_mask |= JACK_MASK_DPIN;
		}
		if (of_property_read_bool(np, "jack-det,dpout")) {
			char dp_str[32];

			snprintf(dp_str, sizeof(dp_str),
				"HDMI/DP,pcm=%d", current_np);

			link_info->jack_pin[JACK_DPOUT].pin = kstrdup(dp_str, GFP_KERNEL);
			link_info->jack_pin[JACK_DPOUT].mask = SND_JACK_LINEOUT;
			link_info->jack_det_mask |= JACK_MASK_DPOUT;
		}
		if (of_property_read_bool(np, "jack-det,hp")) {
			link_info->jack_pin[JACK_HP].pin = "Headset";
			link_info->jack_pin[JACK_HP].mask = SND_JACK_HEADSET;
			link_info->jack_det_mask |= JACK_MASK_HP;
		}

		dev_info(dev, "\t\tdai_fmt:0x%x\n", link->dai_fmt);
		dev_info(dev, "\t\tmclk_fs:%d\n", link_info->mclk_fs);
		dev_info(dev,
			"\t\ttdm tx mask:0x%x, rx mask:0x%x, slots:%d, slot width:%d\n",
			link_info->tx_mask, link_info->rx_mask,
			link_info->slots, link_info->slot_width);
		dev_info(dev, "\t\tjack_det_mask:0x%x\n",
			link_info->jack_det_mask);

		link->stream_name = np->name ? np->name : link->cpus->dai_name;
		link->name = np->name ? np->name : link->cpus->dai_name;
		link->ops = &cix_dailink_ops;
		link->init = cix_dailink_init;

		link++;
		link_info++;
		current_np++;

		of_node_put(cpu);
		of_node_put(codec);
	}

	return 0;

err_put_codec:
	of_node_put(codec);
err_put_cpu:
	of_node_put(cpu);
err_put_np:
	of_node_put(np);

	return ret;
}
EXPORT_SYMBOL(cix_card_parse_of);

/*
 * ACPI HDMI audio topology for Sky1 Orion O6.
 *
 * Under ACPI, the DAI link topology is fixed:
 *   I2S5 (CIXH6011:02) → hdmi-audio-codec.0 (DP0)
 *   I2S6 (CIXH6011:03) → hdmi-audio-codec.1 (DP1)
 *   I2S7 (CIXH6011:04) → hdmi-audio-codec.2 (DP2/eDP)
 *   I2S8 (CIXH6011:05) → hdmi-audio-codec.3 (DP3)
 *   I2S9 (CIXH6011:06) → hdmi-audio-codec.4 (DP4)
 */
#define SKY1_HDMI_LINKS	5

static const struct {
	const char *i2s_name;	/* ACPI platform device name */
	int codec_id;		/* hdmi-audio-codec.N */
	const char *stream;	/* ALSA stream name */
} sky1_hdmi_topology[SKY1_HDMI_LINKS] = {
	{ "CIXH6011:02", 0, "HDMI/DP 0" },
	{ "CIXH6011:03", 1, "HDMI/DP 1" },
	{ "CIXH6011:04", 2, "HDMI/DP 2" },
	{ "CIXH6011:05", 3, "HDMI/DP 3" },
	{ "CIXH6011:06", 4, "HDMI/DP 4" },
};

int cix_card_parse_acpi(struct cix_asoc_card *priv)
{
	struct snd_soc_card *card = priv->card;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *links;
	struct snd_soc_dai_link_component *comps;
	struct dai_link_info *link_info;
	int i, num_links = 0;

	card->name = "cix_sky1";

	links = devm_kcalloc(dev, SKY1_HDMI_LINKS, sizeof(*links), GFP_KERNEL);
	if (!links)
		return -ENOMEM;

	/* 3 components per link: cpu, codec, platform */
	comps = devm_kcalloc(dev, SKY1_HDMI_LINKS * 3, sizeof(*comps),
			     GFP_KERNEL);
	if (!comps)
		return -ENOMEM;

	link_info = devm_kcalloc(dev, SKY1_HDMI_LINKS, sizeof(*link_info),
				 GFP_KERNEL);
	if (!link_info)
		return -ENOMEM;

	for (i = 0; i < SKY1_HDMI_LINKS; i++) {
		const char *i2s_name = sky1_hdmi_topology[i].i2s_name;
		struct snd_soc_dai_link_component *cpu, *codec, *plat;
		struct snd_soc_dai_link *link;
		struct device *i2s_dev;

		/* Skip links whose I2S platform device doesn't exist */
		i2s_dev = bus_find_device_by_name(&platform_bus_type, NULL,
						  i2s_name);
		if (!i2s_dev) {
			dev_info(dev, "I2S device %s not present, skipping\n",
				 i2s_name);
			continue;
		}
		put_device(i2s_dev);

		cpu = &comps[num_links * 3];
		codec = &comps[num_links * 3 + 1];
		plat = &comps[num_links * 3 + 2];
		link = &links[num_links];

		/* CPU DAI: I2S MC playback */
		cpu->name = i2s_name;
		cpu->dai_name = "i2s-mc-aif1";

		/* Codec DAI: hdmi-audio-codec */
		codec->name = devm_kasprintf(dev, GFP_KERNEL,
					     "hdmi-audio-codec.%d",
					     sky1_hdmi_topology[i].codec_id);
		if (!codec->name)
			return -ENOMEM;
		codec->dai_name = "i2s-hifi";

		/* Platform: DMA is co-located with CPU */
		plat->name = cpu->name;

		link->cpus = cpu;
		link->num_cpus = 1;
		link->codecs = codec;
		link->num_codecs = 1;
		link->platforms = plat;
		link->num_platforms = 1;

		link->name = sky1_hdmi_topology[i].stream;
		link->stream_name = sky1_hdmi_topology[i].stream;
		link->id = i;

		link->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_IB_NF |
				SND_SOC_DAIFMT_CBC_CFC;
		link->ops = &cix_dailink_ops;
		link->init = cix_dailink_init;
		link->trigger_start = SND_SOC_TRIGGER_ORDER_DEFAULT;
		link->trigger_stop = SND_SOC_TRIGGER_ORDER_LDC;

		/* HDMI/DP output jack detection */
		link_info[num_links].jack_pin[JACK_DPOUT].pin =
			devm_kasprintf(dev, GFP_KERNEL, "HDMI/DP,pcm=%d",
				       num_links);
		if (!link_info[num_links].jack_pin[JACK_DPOUT].pin)
			return -ENOMEM;
		link_info[num_links].jack_pin[JACK_DPOUT].mask = SND_JACK_LINEOUT;
		link_info[num_links].jack_det_mask = JACK_MASK_DPOUT;

		num_links++;
	}

	card->dai_link = links;
	card->num_links = num_links;
	card->suspend_post = cix_card_suspend_post;
	card->resume_pre = cix_card_resume_pre;
	priv->link_info = link_info;

	return 0;
}
EXPORT_SYMBOL(cix_card_parse_acpi);

MODULE_DESCRIPTION("Sound Card Utils for Cix Technology");
MODULE_AUTHOR("Xing.Wang <xing.wang@cixtech.com>");
MODULE_LICENSE("GPL v2");
