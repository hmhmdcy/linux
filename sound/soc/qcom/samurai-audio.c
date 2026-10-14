// SPDX-License-Identifier: GPL-2.0-only
/* Realme X2 Pro: native Q6ASM/ADM/AFE, WCD934x SLIM and MAX98937 I2S.
 * SLIM channel setup follows the upstream SDM845 machine driver. */
#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "qdsp6/q6afe.h"
#include "common.h"

static int samurai_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *ch = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	rate->min = rate->max = 48000;
	ch->min = ch->max = rtd->dai_link->id == SLIMBUS_0_TX ? 1 : 2;
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int samurai_init(struct snd_soc_pcm_runtime *rtd)
{
	unsigned int rx[] = {144,145,146,147,148,149,150,151,152,153,154,155,156};
	unsigned int tx[] = {128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143};
	struct snd_soc_dai *codec;
	int i, ret;
	if (rtd->dai_link->id != SLIMBUS_0_RX && rtd->dai_link->id != SLIMBUS_0_TX)
		return 0;
	/* Set up every SLIM runtime: never gate capture on a card-wide flag. */
	for_each_rtd_codec_dais(rtd, i, codec) {
		ret = snd_soc_dai_set_channel_map(codec, ARRAY_SIZE(tx), tx,
						ARRAY_SIZE(rx), rx);
		if (ret)
			return ret;
	}
	return 0;
}

static int samurai_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0), *codec;
	int i, ret;
	if (cpu->id != QUATERNARY_MI2S_RX)
		return 0;
	ret = snd_soc_dai_set_fmt(cpu, SND_SOC_DAIFMT_BP_FP | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
	if (ret)
		return ret;
	for_each_rtd_codec_dais(rtd, i, codec) {
		ret = snd_soc_dai_set_fmt(codec, SND_SOC_DAIFMT_BC_FC | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
		if (ret)
			return ret;
	}
	return snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
				     48000 * 2 * 16, SNDRV_PCM_STREAM_PLAYBACK);
}

static void samurai_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;
	if (cpu->id != QUATERNARY_MI2S_RX)
		return;
	ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT,
				    0, SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		dev_err(rtd->dev, "quaternary clock shutdown: %d\n", ret);
}

static int samurai_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec = snd_soc_rtd_to_codec(rtd, 0);
	unsigned int rx[13], tx[16], nrx = 0, ntx = 0;
	int ret;
	if (cpu->id == QUATERNARY_MI2S_RX)
		return 0;
	ret = snd_soc_dai_get_channel_map(codec, &ntx, tx, &nrx, rx);
	if (ret)
		return ret;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!nrx || nrx > ARRAY_SIZE(rx))
			return -EINVAL;
		return snd_soc_dai_set_channel_map(cpu, 0, NULL, nrx, rx);
	}
	if (!ntx || ntx > ARRAY_SIZE(tx))
		return -EINVAL;
	return snd_soc_dai_set_channel_map(cpu, ntx, tx, 0, NULL);
}

static const struct snd_soc_ops samurai_ops = {
	.startup = samurai_startup, .shutdown = samurai_shutdown,
	.hw_params = samurai_params,
};

static int samurai_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct snd_soc_dai_link *link;
	int i, ret;
	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;
	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm) {
			link->init = samurai_init;
			link->ops = &samurai_ops;
			link->be_hw_params_fixup = samurai_fixup;
		}
	}
	return devm_snd_soc_register_card(&pdev->dev, card);
}
static const struct of_device_id samurai_audio_of[] = {
	{ .compatible = "realme,samurai-audio" }, { }
};
MODULE_DEVICE_TABLE(of, samurai_audio_of);
static struct platform_driver samurai_audio_driver = {
	.probe = samurai_probe,
	.driver = { .name = "samurai-audio", .of_match_table = samurai_audio_of },
};
module_platform_driver(samurai_audio_driver);
MODULE_DESCRIPTION("Realme X2 Pro QDSP6 audio machine");
MODULE_LICENSE("GPL");
