/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "../jz4740/jz4740-pcm.h"
#include "jzcodec.h"

#define AUDIO_NAME "jzcodec"
#define JZCODEC_VERSION "1.0"

/*
 * Debug
 */

#define JZCODEC_DEBUG 0

#ifdef JZCODEC_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

struct snd_soc_codec_device soc_codec_dev_jzcodec;

/* codec private data */
struct jzcodec_priv {
	unsigned int sysclk;
};

/*
 * jzcodec register cache
 */
static u32 jzcodec_reg[JZCODEC_CACHEREGNUM / 2];

/*
 * codec register is 16 bits width in ALSA, so we define array to store 16 bits configure paras
 */
static u16 jzcodec_reg_LH[JZCODEC_CACHEREGNUM];

/*
 * read jzcodec register cache
 */
static inline unsigned int jzcodec_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	if (reg >= JZCODEC_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write jzcodec register cache
 */
static inline void jzcodec_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, u16 value)
{
	u16 *cache = codec->reg_cache;
	u32 reg_val;

	if (reg >= JZCODEC_CACHEREGNUM) {
		return;
	}

	cache[reg] = value;
	/* update internal codec register value */
	switch (reg) {
	case 0:
	case 1:
		reg_val = cache[0] & 0xffff;
		reg_val = reg_val | (cache[1] << 16); 
		jzcodec_reg[0] = reg_val;
		break;
	case 2:
	case 3:
		reg_val = cache[2] & 0xffff;
		reg_val = reg_val | (cache[3] << 16); 
		jzcodec_reg[1] = reg_val;
		break;
	}
}

/*
 * write to the jzcodec register space
 */
static int jzcodec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
//	dump_stack();
	jzcodec_write_reg_cache(codec, reg, value);
	if(codec->hw_write)
		codec->hw_write(&value, NULL, reg);	
	return 0;
}

static int jzcodec_reset(struct snd_soc_codec *codec)
{
	u16 val;

	val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
	val = val | 0x1;
	jzcodec_write(codec, ICODEC_1_LOW, val);
	mdelay(1);
	
	val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
	val = val & ~0x1;
	jzcodec_write(codec, ICODEC_1_LOW, val);
	mdelay(1);

	return 0;
}

static const struct snd_kcontrol_new jzcodec_snd_controls[] = {

	//SOC_DOUBLE_R("Master Playback Volume", 1, 1, 0, 3, 0),
	SOC_SINGLE("Master Playback Volume", ICODEC_2_LOW, 0, 3, 0),
	//SOC_DOUBLE_R("MICBG", ICODEC_2_LOW, ICODEC_2_LOW, 4, 3, 0),
	//SOC_DOUBLE_R("Line", 2, 2, 0, 31, 0),
//	SOC_DOUBLE_R("Line", ICODEC_2_HIGH, ICODEC_2_HIGH, 0, 31, 0),
	SOC_SINGLE("HP Mute Switch", ICODEC_1_LOW, 14, 1, 0),
	SOC_SINGLE("MIC Boost Gain", ICODEC_2_LOW, 4, 3, 0),
	SOC_SINGLE("Capture Volume", ICODEC_2_HIGH, 0, 31, 0),
};

static const struct snd_kcontrol_new jzcodec_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("Bypass Switch", ICODEC_1_HIGH, 11, 1, 0),
	SOC_DAPM_SINGLE("Playback Switch", ICODEC_1_HIGH, 9, 1, 0),
};

static const struct snd_kcontrol_new jzcodec_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("Line Capture Switch", ICODEC_1_HIGH, 13, 1, 0),
	SOC_DAPM_SINGLE("MIC Capture Switch", ICODEC_1_HIGH, 12, 1, 0),
};

/* add non dapm controls */
static int jzcodec_add_controls(struct snd_soc_codec *codec)
{
	int err, i;
	
	for (i = 0; i < ARRAY_SIZE(jzcodec_snd_controls); i++) {
		if ((err = snd_ctl_add(codec->card,
				       snd_soc_cnew(&jzcodec_snd_controls[i], codec, NULL))) < 0)
			return err;
	}
	
	return 0;
}

static const struct snd_soc_dapm_widget jzcodec_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),
	SND_SOC_DAPM_INPUT("MICIN"),
	SND_SOC_DAPM_INPUT("RLINEIN"),
	SND_SOC_DAPM_INPUT("LLINEIN"),

	SND_SOC_DAPM_MIXER("Output Mixer", SND_SOC_NOPM, 0, 0,
			jzcodec_output_mixer_controls, ARRAY_SIZE(jzcodec_output_mixer_controls)),
	SND_SOC_DAPM_MIXER("Input Mixer", SND_SOC_NOPM, 0, 0,
			jzcodec_input_mixer_controls, ARRAY_SIZE(jzcodec_input_mixer_controls)),
	SND_SOC_DAPM_ADC("ADC", "Capture", ICODEC_1_HIGH, 10, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", ICODEC_1_HIGH, 8, 0),

	SND_SOC_DAPM_PGA("Line Input", SND_SOC_NOPM, 0, 0, NULL, 0),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* output mixer */
	{"Output Mixer", "Bypass Switch", "Input Mixer"},
	{"Output Mixer", "Playback Switch", "DAC"},

	/* outputs */
	{"RHPOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},
	{"LHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},

	/* input mixer */
	{"Input Mixer", "Line Capture Switch", "Line Input"},
	{"Input Mixer", "MIC Capture Switch", "MICIN"},
	{"ADC", NULL, "Input Mixer"},

	/* inputs */
	{"Line Input", NULL, "LLINEIN"},
	{"Line Input", NULL, "RLINEIN"},
};

static int jzcodec_add_widgets(struct snd_soc_codec *codec)
{
	int i,cnt;
	
	cnt = ARRAY_SIZE(jzcodec_dapm_widgets);
	for(i = 0; i < ARRAY_SIZE(jzcodec_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &jzcodec_dapm_widgets[i]);
	}

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int jzcodec_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 reg_val = jzcodec_read_reg_cache(codec, ICODEC_2_LOW);

	/* bit size. codec side */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	}
	/* sample rate */
	reg_val = reg_val & ~(0xf << 8);
	switch (params_rate(params)) {
	case 8000:
		reg_val |= (0x0 << 8);
		break;
	case 11025:
		reg_val |= (0x1 << 8);
		break;
	case 16000:
		reg_val |= (0x3 << 8);
		break;
	case 22050:
		reg_val |= (0x4 << 8);
		break;
	case 32000:
		reg_val |= (0x6 << 8);
		break;
	case 44100:
		reg_val |= (0x7 << 8);
		break;
	case 48000:
		reg_val |= (0x8 << 8);
		break;
	default:
		printk(" invalid rate :0x%08x\n",params_rate(params));
	}

        jzcodec_write(codec, ICODEC_2_LOW, reg_val);
	return 0;
}

static int jzcodec_pcm_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
	u16 val;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		//case SNDRV_PCM_TRIGGER_RESUME:
		//case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			val = 0x7302;
			jzcodec_write(codec, ICODEC_1_LOW, val);
			val = 0x0003;
			jzcodec_write(codec, ICODEC_1_HIGH, val);
			mdelay(2);
			val = 0x6000;
			jzcodec_write(codec, ICODEC_1_LOW, val);
			val = 0x0300;
			jzcodec_write(codec, ICODEC_1_HIGH, val);
			mdelay(2);
			val = 0x2000;
			jzcodec_write(codec, ICODEC_1_LOW, val);
			val = 0x0300;
			jzcodec_write(codec, ICODEC_1_HIGH, val);
		} else {
			val = 0x4300;
			jzcodec_write(codec, ICODEC_1_LOW, val);
			val = 0x1402;
			jzcodec_write(codec, ICODEC_1_HIGH, val);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		//case SNDRV_PCM_TRIGGER_SUSPEND:
		//case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			val = 0x3300;
			jzcodec_write(codec, ICODEC_1_LOW, val);
			val = 0x0003;
			jzcodec_write(codec, ICODEC_1_HIGH, val);
		} else {
			val = 0x3300;
			jzcodec_write(codec, ICODEC_1_LOW, val);
			val = 0x0003;
			jzcodec_write(codec, ICODEC_1_HIGH, val);
		}
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int jzcodec_pcm_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	/*struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec; */

	return 0;
}

static void jzcodec_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
	}
}

static int jzcodec_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);

	if (mute != 0) 
		mute = 1;
	if (mute)
		reg_val = reg_val | (0x1 << 14);
	else
		reg_val = reg_val & ~(0x1 << 14);
	
	jzcodec_write(codec, ICODEC_1_LOW, reg_val);
	return 0;
}

static int jzcodec_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct jzcodec_priv *jzcodec = codec->private_data;

	jzcodec->sysclk = freq;
	return 0;
}
/*
 * Set's ADC and Voice DAC format. called by pavo_hw_params() in pavo.c
 */
static int jzcodec_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */

	/* set master/slave audio interface. codec side */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
                /* set master mode for codec */
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* set slave mode for codec */
		break;
	default:
		return -EINVAL;
	}

	/* interface format . set some parameter for codec side */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S: 
		/* set I2S mode for codec */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		/* set right J mode */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* set left J mode */
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* set dsp A mode */
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* set dsp B mode */
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion. codec side */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:		
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	default:
		return -EINVAL;
		}

	/* jzcodec_write(codec, 0, val); */
	return 0;
}

static int jzcodec_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	u16 reg_val;
	int hpmute;

	switch (level) {
	case SND_SOC_BIAS_ON: /* full On */
		/* vref/mid, osc on, dac unmute */
		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
		reg_val &= ~(ICODEC_1_HIGH_PDVR | ICODEC_1_HIGH_PDVRA);
		reg_val &= ~(ICODEC_1_HIGH_VRCGL | ICODEC_1_HIGH_VRCGH);
		jzcodec_write_reg_cache(codec, ICODEC_1_HIGH, reg_val);

		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
		hpmute = reg_val & ICODEC_1_LOW_HPMUTE;
		reg_val |= ICODEC_1_LOW_HPOV0 | ICODEC_1_LOW_HPCG | ICODEC_1_LOW_PDHP | ICODEC_1_LOW_PDHPM | ICODEC_1_LOW_SUSP;
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);
		mdelay(1);

		reg_val &= ~(ICODEC_1_LOW_HPOV0 | ICODEC_1_LOW_PDHP | ICODEC_1_LOW_PDHPM | ICODEC_1_LOW_SUSP);
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);
		mdelay(1);

		reg_val &= ~(ICODEC_1_LOW_HPMUTE | ICODEC_1_LOW_HPCG);
		reg_val |= hpmute | ICODEC_1_LOW_HPOV0;
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);
		break;

	case SND_SOC_BIAS_PREPARE: /* partial On */
		break;
	case SND_SOC_BIAS_STANDBY: /* Off, with power */
		/* everything off except vref/vmid, */
		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
		reg_val &= ~ICODEC_1_LOW_HPOV0;
		reg_val |= ICODEC_1_LOW_PDHPM | ICODEC_1_LOW_PDHP;
		jzcodec_write_reg_cache(codec, ICODEC_1_LOW, reg_val);

		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
		reg_val |= ICODEC_1_HIGH_VRCGL | ICODEC_1_HIGH_VRCGH;
		jzcodec_write(codec, ICODEC_1_HIGH, reg_val);

		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
		reg_val &= ~(ICODEC_1_LOW_PDHPM);
		reg_val |= (ICODEC_1_LOW_SUSP
				| ICODEC_1_LOW_PDHP
				| ICODEC_1_LOW_PDHPM
				);
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);
		break;

	case SND_SOC_BIAS_OFF: /* Off, without power */
		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
		reg_val |= ICODEC_1_HIGH_VRCGL | ICODEC_1_HIGH_VRCGH;
		jzcodec_write_reg_cache(codec, ICODEC_1_HIGH, reg_val);

		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
		reg_val |= (ICODEC_1_LOW_SUSP | ICODEC_1_LOW_PDHP | ICODEC_1_LOW_PDHPM |
				ICODEC_1_LOW_HPPLDM | ICODEC_1_LOW_HPPLDR);
		jzcodec_write(codec, ICODEC_1_LOW, reg_val);

		reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
		reg_val |= (ICODEC_1_HIGH_PDVR | ICODEC_1_HIGH_PDVRA);
		jzcodec_write(codec, ICODEC_1_HIGH, reg_val);
		break;

	}
	codec->bias_level = level;
	return 0;
}

#define JZCODEC_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000)

#define JZCODEC_FORMATS (SNDRV_PCM_FORMAT_S8 | SNDRV_PCM_FMTBIT_S16_LE)

struct snd_soc_dai jzcodec_dai = {
	.name = "JZCODEC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZCODEC_RATES,
		.formats = JZCODEC_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = JZCODEC_RATES,
		.formats = JZCODEC_FORMATS,},
	.ops = {
		.prepare = jzcodec_pcm_prepare,
		.hw_params = jzcodec_hw_params,
		.shutdown = jzcodec_shutdown,
		.digital_mute = jzcodec_mute,
		.set_sysclk = jzcodec_set_dai_sysclk,
		.set_fmt = jzcodec_set_dai_fmt,
	}
};
EXPORT_SYMBOL_GPL(jzcodec_dai);

#ifdef CONFIG_PM
static u16 jzcodec_reg_pm[JZCODEC_CACHEREGNUM];
static int jzcodec_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	jzcodec_reg_pm[ICODEC_1_LOW] = jzcodec_read_reg_cache(codec, ICODEC_1_LOW);
	jzcodec_reg_pm[ICODEC_1_HIGH] = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
	jzcodec_reg_pm[ICODEC_2_LOW] = jzcodec_read_reg_cache(codec, ICODEC_2_LOW);
	jzcodec_reg_pm[ICODEC_2_HIGH] = jzcodec_read_reg_cache(codec, ICODEC_2_HIGH);

	jzcodec_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int jzcodec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	u16 reg_val;

	jzcodec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	reg_val = jzcodec_reg_pm[ICODEC_1_LOW];
	jzcodec_write(codec, ICODEC_1_LOW, reg_val);
	reg_val = jzcodec_reg_pm[ICODEC_1_HIGH];
	jzcodec_write(codec, ICODEC_1_HIGH, reg_val);
	reg_val = jzcodec_reg_pm[ICODEC_2_LOW];
	jzcodec_write(codec, ICODEC_2_LOW, reg_val);
	reg_val = jzcodec_reg_pm[ICODEC_2_HIGH];
	jzcodec_write(codec, ICODEC_2_HIGH, reg_val);

	jzcodec_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}
#else
#define jzcodec_suspend	NULL
#define jzcodec_resume	NULL
#endif
/*
 * initialise the JZCODEC driver
 * register the mixer and dsp interfaces with the kernel
 */
static int jzcodec_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int reg, ret = 0;
	u16 reg_val;

	/* Default values at reset */
	REG_ICDC_CDCCR1 = 0x001b2302;
	REG_ICDC_CDCCR2 = 0x00017803;
	for (reg = 0; reg < JZCODEC_CACHEREGNUM / 2; reg++) {
		switch (reg) {
		case 0:
			jzcodec_reg[reg] = REG_ICDC_CDCCR1;
			jzcodec_reg_LH[ICODEC_1_LOW] = jzcodec_reg[reg] & 0xffff;
			jzcodec_reg_LH[ICODEC_1_HIGH] = (jzcodec_reg[reg] & 0xffff0000) >> 16;
			break;
		case 1:
			jzcodec_reg[reg] = REG_ICDC_CDCCR2;
			jzcodec_reg_LH[ICODEC_2_LOW] = jzcodec_reg[reg] & 0xffff;
			jzcodec_reg_LH[ICODEC_2_HIGH] = (jzcodec_reg[reg] & 0xffff0000) >> 16;
			break;
		}
	}

	codec->name = "JZCODEC";
	codec->owner = THIS_MODULE;
	codec->read = jzcodec_read_reg_cache;
	codec->write = jzcodec_write;
	codec->set_bias_level = jzcodec_set_bias_level;
	codec->dai = &jzcodec_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(jzcodec_reg_LH);
	codec->reg_cache = kmemdup(jzcodec_reg_LH, sizeof(jzcodec_reg_LH), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	jzcodec_reset(codec);
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "jzcodec: failed to create pcms\n");
		goto pcm_err;
	}

	/* Set HPVOL to 0 */
	reg_val = jzcodec_read_reg_cache(codec, ICODEC_2_LOW);
	reg_val &= ~0x00000003ul;
	jzcodec_write(codec, ICODEC_2_LOW, reg_val);

	/* Enable playback path by default */
	reg_val = jzcodec_read_reg_cache(codec, ICODEC_1_HIGH);
	reg_val |= ICODEC_1_HIGH_SW2ON;
	jzcodec_write(codec, ICODEC_1_HIGH, reg_val);

	/* power on device */
	jzcodec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	jzcodec_add_controls(codec);
	jzcodec_add_widgets(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "jzcodec: failed to register card\n");
		goto card_err;
	}
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

static struct snd_soc_device *jzcodec_socdev;

static int write_codec_reg(u16 * add, char * name, int reg)
{
	switch (reg) {
	case 0:
	case 1:
		REG_ICDC_CDCCR1 = jzcodec_reg[0];
		break;
	case 2:
	case 3:
		REG_ICDC_CDCCR2 = jzcodec_reg[1];
		break;
	}
	return 0;
}

static int jzcodec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct jzcodec_priv *jzcodec;
	int ret = 0;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	jzcodec = kzalloc(sizeof(struct jzcodec_priv), GFP_KERNEL);
	if (jzcodec == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = jzcodec;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	jzcodec_socdev = socdev;

	/* Add other interfaces here ,no I2C connection */
	codec->hw_write = (hw_write_t)write_codec_reg;
	ret = jzcodec_init(jzcodec_socdev);

	if (ret < 0) {
		codec = jzcodec_socdev->codec;
		err("failed to initialise jzcodec\n");
		kfree(codec);
	}

	return ret;
}

/* power down chip */
static int jzcodec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		jzcodec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_jzcodec = {
	.probe = 	jzcodec_probe,
	.remove = 	jzcodec_remove,
	.suspend = 	jzcodec_suspend,
	.resume =	jzcodec_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_jzcodec);

static int __init jzcodec_modinit(void)
{
	return snd_soc_register_dai(&jzcodec_dai);
}
module_init(jzcodec_modinit);

static void __exit jzcodec_modexit(void)
{
	snd_soc_unregister_dai(&jzcodec_dai);
}
module_exit(jzcodec_modexit);

MODULE_DESCRIPTION("ASoC JZCODEC driver");
MODULE_AUTHOR("Richard");
MODULE_LICENSE("GPL");
