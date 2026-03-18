// SPDX-License-Identifier: GPL-2.0
/* Copyright 2024 Cix Technology Group Co., Ltd. */
#include <linux/kernel.h>
#include <linux/hdmi.h>
#include <drm/drm_eld.h>
#include "trilin_dptx_reg.h"
#include "trilin_dptx.h"
#include "dptx_infoframe.h"

/* dp audio */
void dptx_audio_handle_plugged_change(struct dptx_audio *dp_audio, bool plugged)
{
	if (dp_audio->codec_dev && dp_audio->plugged_cb)
		dp_audio->plugged_cb(dp_audio->codec_dev, plugged);
}

static void dptx_audio_setup(struct trilin_dp *dp, int source, int freq,
			     int sample_len, int channel_count)
{
	unsigned int offset;
	unsigned int cs_length_orig_freq;
	unsigned int cs_freq_clock_accuracy;

	if (!dp)
		return;

	offset = (TRILIN_DPTX_SEC1_AUDIO_ENABLE -
		  TRILIN_DPTX_SEC0_AUDIO_ENABLE) *
		 source;

	/* Bit 7:4 source number; Bit 3, '0' for linear PCM samples; */
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_CS_SOURCE_FORMAT + offset,
			(source << DPTX_CS_SOURCE_NUMBER_SHIFT));
	/* Categroy code: 0, General. */
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_CS_CATEGORY_CODE + offset,
			DPTX_CS_CATEGORY_CODE);
	switch (sample_len) {
	case 16:
		cs_length_orig_freq = DPTX_CS_SAMPLE_WORD_LENGTH_16BITS;
		break;
	case 18:
		cs_length_orig_freq = DPTX_CS_SAMPLE_WORD_LENGTH_18BITS;
		break;
	case 20:
		cs_length_orig_freq = DPTX_CS_SAMPLE_WORD_LENGTH_20BITS;
		break;
	case 24:
	default:
		cs_length_orig_freq = DPTX_CS_SAMPLE_WORD_LENGTH_24BITS;
		break;
	}
	/* clock accuracy: 00, level II, default; 10, level I; 01, level III */
	switch (freq) {
	case 32000:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_32000HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_32000Hz;
		break;
	case 44100:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_44100HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_44100Hz;
		break;
	case 88200:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_88200HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_88200Hz;
		break;
	case 96000:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_96000HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_96000Hz;
		break;
	case 176400:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_176400HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_176400Hz;
		break;
	case 192000:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_192000HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_192000Hz;
		break;
	case 48000:
	default:
		cs_length_orig_freq |= DPTX_CS_SAMPLING_ORIG_FREQ_48000HZ;
		cs_freq_clock_accuracy = DPTX_CS_SAMPLING_FREQ_48000Hz;
		break;
	}

	trilin_dp_write(dp, TRILIN_DPTX_SEC0_CS_LENGTH_ORIG_FREQ + offset,
			cs_length_orig_freq);
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_CS_FREQ_CLOCK_ACCURACY + offset,
			cs_freq_clock_accuracy);
	/* copyright*/
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_CS_COPYRIGHT + offset,
			DPTX_CS_COPYRIGHT);
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_TIMESTAMP_INTERVAL,
			DPTX_CS_TIMESTAMP_INTERVAL);
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_AUDIO_CHANNEL_MAP + offset,
			DPTX_CS_AUDIO_CHANNEL_MAP_DEFAULT);
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_CHANNEL_COUNT + offset,
			channel_count);
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_INPUT_SELECT + offset, source);
}

/*
 * Send an Audio InfoFrame SDP so the sink knows the channel layout.
 * Without this, the sink defaults to stereo regardless of how many
 * channels are being transmitted.
 */
static void dptx_audio_send_infoframe(struct trilin_dp *dp, int source,
				       const struct hdmi_audio_infoframe *cea)
{
	struct dp_sdp sdp;
	ssize_t len;

	len = hdmi_audio_infoframe_pack_for_dp(cea, &sdp, dp->dpcd[DP_DPCD_REV]);
	if (len < 0) {
		dev_warn(dp->dev, "failed to pack audio infoframe: %zd\n", len);
		return;
	}

	cix_infoframe_write_packet(dp, source, CIX_AUDIO_INFO_SDP, &sdp, 1);
}

static int dptx_audio_get_eld(struct device *dev, void *data,
			      uint8_t *buf, size_t len)
{
	struct trilin_dp *dp = data;

	/* Only copy the ELD if the sink is connected AND the ELD has
	 * been populated from EDID (baseline_eld_len > 0).  At boot
	 * the plugged callback fires before EDID is read, so the
	 * connector ELD is still all-zeros ��� returning that causes
	 * noisy "Unknown ELD version 0" warnings from snd_parse_eld.
	 */
	if (!dp->plugin || !dp->connector.base.eld[DRM_ELD_BASELINE_ELD_LEN])
		return -ENODEV;

	memcpy(buf, dp->connector.base.eld,
	       min(sizeof(dp->connector.base.eld), len));

	return 0;
}

static int dptx_audio_startup(struct device *dev, void *data)
{
	struct trilin_dp *dp = (struct trilin_dp *)data;
	struct dptx_audio *dp_audio = &dp->dp_audio;

	if (!dp->plugin)
		return 0;

	trilin_dp_write(dp, TRILIN_DPTX_SEC0_AUDIO_ENABLE, 1);

	dp_audio->running = true;

	return 0;
}

static void dptx_audio_shutdown(struct device *dev, void *data)
{
	struct trilin_dp *dp = (struct trilin_dp *)data;
	struct dptx_audio *dp_audio = &dp->dp_audio;

	dp_audio->running = false;

	/* Disable audio packets in hardware.  If the sink was already
	 * unplugged, core_off has powered down the PHY but this write
	 * is still safe (MMIO register block remains accessible).
	 */
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_AUDIO_ENABLE, 0);
}

static int dptx_audio_hw_params(struct device *dev, void *data,
				struct hdmi_codec_daifmt *daifmt,
				struct hdmi_codec_params *params)
{
	struct trilin_dp *dp = (struct trilin_dp *)data;
	struct dptx_audio *dp_audio = &dp->dp_audio;

	if (!dp->plugin)
		return 0;

	dp_audio->params.sample_width = params->sample_width;
	dp_audio->params.sample_rate = params->sample_rate;
	dp_audio->params.channels = params->channels;
	dp_audio->params.cea = params->cea;

	dev_dbg(dev,
		"%s, daifmt fmt:%d, bit_clk_inv:%d, frame_clk_inv:%d, bit_clk_master:%d, frame_clk_master:%d, params sample_rate:%d, sample_width:%d, channels:%d, ca:%d\n",
		__func__, daifmt->fmt, daifmt->bit_clk_inv,
		daifmt->frame_clk_inv, daifmt->bit_clk_provider,
		daifmt->frame_clk_provider, params->sample_rate,
		params->sample_width, params->channels,
		params->cea.channel_allocation);

	dptx_audio_setup(dp, 0, params->sample_rate, params->sample_width,
			 params->channels);
	dptx_audio_send_infoframe(dp, 0, &params->cea);

	return 0;
}

static int dptx_audio_get_dai_id(struct snd_soc_component *comment,
				 struct device_node *endpoint,
				 void *data)
{
	return 0;
}

static int dptx_audio_hook_plugged_cb(struct device *dev, void *data,
				      hdmi_codec_plugged_cb fn,
				      struct device *codec_dev)
{
	struct trilin_dp *dp = (struct trilin_dp *)data;
	struct dptx_audio *dp_audio = &dp->dp_audio;

	dp_audio->plugged_cb = fn;
	dp_audio->codec_dev = codec_dev;

	/* Only fire the initial plugged notification if the ELD has
	 * already been populated (i.e. EDID was read before hdmi-codec
	 * registered).  Otherwise, trilin_connector_update_modes()
	 * will fire it after the ELD is ready.  This avoids the
	 * "Unknown ELD version 0" warning from snd_parse_eld.
	 */
	if (dp->plugin && dp->connector.base.eld[DRM_ELD_BASELINE_ELD_LEN])
		dptx_audio_handle_plugged_change(dp_audio, true);

	return 0;
}

void dptx_audio_reconfig_and_enable(void *data)
{
	struct trilin_dp *dp = (struct trilin_dp *)data;
	struct dptx_audio *dp_audio = &dp->dp_audio;

	dptx_audio_setup(dp, 0, dp_audio->params.sample_rate,
			 dp_audio->params.sample_width,
			 dp_audio->params.channels);
	dptx_audio_send_infoframe(dp, 0, &dp_audio->params.cea);

	/* enable dptx audio */
	trilin_dp_write(dp, TRILIN_DPTX_SEC0_AUDIO_ENABLE, 1);
}

const struct hdmi_codec_ops dptx_audio_codec_ops = {
	.hw_params = dptx_audio_hw_params,
	.audio_startup = dptx_audio_startup,
	.audio_shutdown = dptx_audio_shutdown,
	.get_eld = dptx_audio_get_eld,
	.get_dai_id = dptx_audio_get_dai_id,
	.hook_plugged_cb = dptx_audio_hook_plugged_cb
};
