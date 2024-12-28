#include <linux/module.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/pci_ids.h>
#include <linux/pci.h>

static int loongson_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;

}

static struct snd_soc_ops loongson_dai_link_ops = {
	.hw_params = loongson_hw_params,
};

enum {
	PRI_PLAYBACK = 0,
	PRI_CAPTURE,
};
#define LOONGSON_DAI_FMT (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | \
	SND_SOC_DAIFMT_CBS_CFS)
//	SND_SOC_DAIFMT_CBM_CFM)

SND_SOC_DAILINK_DEF(ls_dai_cpus, COMP_CPU("loongson-i2s-dai"));
SND_SOC_DAILINK_DEF(ls_dai_codecs, COMP_CODEC("", ""));
SND_SOC_DAILINK_DEF(ls_dai_platforms, COMP_PLATFORM("loongson-i2s"));

static struct snd_soc_dai_link loongson_dai_link[] = {
	[PRI_PLAYBACK] = {
		/* Primary Playback i/f */
		.name = "Loongson Dai link RX",
		.stream_name = "Playback",
		.dai_fmt = LOONGSON_DAI_FMT,
		.ops = &loongson_dai_link_ops,
		SND_SOC_DAILINK_REG(ls_dai),
	},
	[PRI_CAPTURE] = {
		/* Primary Capture i/f */
		.name = "Loongson Dai link TX",
		.stream_name = "Capture",
		.dai_fmt = LOONGSON_DAI_FMT,
		.ops = &loongson_dai_link_ops,
		SND_SOC_DAILINK_REG(ls_dai),
	},
};

static struct snd_soc_card ls_sound_card = {
	.owner = THIS_MODULE,
	.dai_link = loongson_dai_link,
	.num_links = ARRAY_SIZE(loongson_dai_link),

};

static void put_of_nodes(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *dai_link;
	int i;

	for_each_card_prelinks(card, i, dai_link) {
		of_node_put(dai_link->cpus->of_node);
		of_node_put(dai_link->codecs->of_node);
	}
}

static int ls_sound_init_codec_name(struct platform_device* pdev,
		struct i2c_client *codec_i2c)
{
	char codec_name[32];
	const char* codec_dai_name;
	struct snd_soc_component *codec_component;
	struct snd_soc_dai *codec_dai;

	codec_component = snd_soc_lookup_component(&codec_i2c->dev, NULL);
	if (!codec_component)
	{
		dev_err(&pdev->dev,
			"I2C codec Not Found\n");
		return -EAGAIN;
	}

	// codec-name = <codec-i2c-driver-name>.<i2cx>-<i2c-addr>
	sprintf(codec_name, "%s.%d-%04x",
			codec_i2c->dev.driver->name,
			codec_i2c->adapter->nr, codec_i2c->addr);

	// codec-dai-name = depends on codec driver
	for_each_component_dais(codec_component, codec_dai){
		codec_dai_name = codec_dai->name;
		break;
	}

	ls_dai_codecs[0].name = codec_name;
	ls_dai_codecs[0].dai_name = codec_dai_name;

	dev_info(&pdev->dev, "name,dai-name: %s,%s\n", codec_name, codec_dai_name);

	return 0;
}

static int ls_sound_drv_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card;
	struct snd_soc_dai_link *dai_link;
	int ret;
	struct device_node *codec_np;
	struct i2c_client *codec_i2c;
	const char* card_name;

	card = (struct snd_soc_card *)of_device_get_match_data(&pdev->dev);
	card->dev = &pdev->dev;
	dai_link = card->dai_link;

	dai_link->cpus->of_node = of_parse_phandle(np, "loongson,i2s-controller", 0);
	if (!dai_link->cpus->of_node) {
		dev_err(&pdev->dev,
			"Property 'loongson,i2s-controller' missing or invalid\n");
		return -EINVAL;
	}

	if (!dai_link->platforms->name)
		dai_link->platforms->of_node = dai_link->cpus->of_node;

	codec_np = of_parse_phandle(np, "loongson,audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev,
			"Property 'loongson,audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto err_put_of_nodes;
	}

	codec_i2c = of_find_i2c_device_by_node(codec_np);
	if (!codec_i2c)
	{
		dev_err(&pdev->dev,
			"Property 'loongson,audio-codec' is not a i2c device\n");
		ret = -EINVAL;
		goto err_put_of_nodes;
	}

	of_property_read_string(np, "loongson,sound-card-name", &card_name);

	card->name = card_name;

	ls_sound_init_codec_name(pdev, codec_i2c);

	ret = devm_snd_soc_register_card(card->dev, card);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"snd_soc_register_card() failed: %d\n", ret);
		goto err_put_of_nodes;
	}
	return 0;

err_put_of_nodes:
	put_of_nodes(card);
	return ret;
}

static void ls_sound_drv_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	put_of_nodes(card);
}

static const struct of_device_id snd_ls_sound_dt_match[] = {
	{ .compatible = "loongson,ls-sound", .data = &ls_sound_card},
	{},
};
MODULE_DEVICE_TABLE(of, snd_ls_sound_dt_match);

static struct platform_driver ls_sound_driver = {
	.probe = ls_sound_drv_probe,
	.remove = ls_sound_drv_remove,
	.driver = {
		.name = "ls-sound",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(snd_ls_sound_dt_match),
	},
};

static int __init loongson_audio_init(void)
{
	return platform_driver_register(&ls_sound_driver);
}
module_init(loongson_audio_init);

static void __exit loongson_audio_exit(void)
{
	platform_driver_unregister(&ls_sound_driver);
}
module_exit(loongson_audio_exit);

MODULE_AUTHOR("loongson");
MODULE_DESCRIPTION("ALSA SoC loongson");
MODULE_LICENSE("GPL");
