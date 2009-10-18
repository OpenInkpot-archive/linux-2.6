/*
 * n516.c  --  SoC audio for n516
 *
 * Copyright (C) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *	OpenInkpot project
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/jzcodec.h"
#include "jz4740-pcm.h"
#include "jz4740-i2s.h"

#define N516_SPK_OFF	0
#define N516_SPK_ON	1
#define N516_SPK_AUTO	2

 /* audio clock in Hz - rounded from 12.235MHz */
#define N516_AUDIO_CLOCK 12288000

static int n516_headphone_present;
static int n516_spk_func;

/* in ms */
#define HPLUG_DETECT_DELAY 50

static struct timer_list n516_hplug_timer;

static void n516_ext_control(struct snd_soc_codec *codec)
{
	switch (n516_spk_func) {
	case N516_SPK_ON:
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
		break;
	case N516_SPK_OFF:
		snd_soc_dapm_disable_pin(codec, "Ext Spk");
		break;
	case N516_SPK_AUTO:
		if (n516_headphone_present)
			snd_soc_dapm_disable_pin(codec, "Ext Spk");
		else
			snd_soc_dapm_enable_pin(codec, "Ext Spk");
		break;
	}

	if (n516_headphone_present)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
}

static int n516_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	/* check the jack status at stream startup */
	n516_ext_control(codec);
	return 0;
}

static void n516_shutdown(struct snd_pcm_substream *substream)
{
	/*struct snd_soc_pcm_runtime *rtd = substream->private_data;
	  struct snd_soc_codec *codec = rtd->socdev->codec;*/

	return;
}

static int n516_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;

	/* set codec DAI configuration */
	ret = codec_dai->ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops.set_sysclk(codec_dai, JZCODEC_SYSCLK, 111,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = cpu_dai->ops.set_sysclk(cpu_dai, JZ4740_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops n516_ops = {
	.startup = n516_startup,
	.hw_params = n516_hw_params,
	.shutdown = n516_shutdown,
};

static int n516_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = n516_spk_func;
	return 0;
}

static int n516_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (n516_spk_func == ucontrol->value.integer.value[0])
		return 0;

	n516_spk_func = ucontrol->value.integer.value[0];
	n516_ext_control(codec);
	return 1;
}

static int n516_amp_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		__gpio_set_pin(GPIO_SPK_SHUD);
	else
		__gpio_clear_pin(GPIO_SPK_SHUD);

	return 0;
}

static void n516_hplug_timer_func(unsigned long data)
{
	struct snd_soc_codec *codec = (struct snd_soc_codec *)data;

	n516_headphone_present = __gpio_get_pin(GPIO_HPHONE_PLUG);

	if (n516_headphone_present)
		__gpio_as_irq_fall_edge(GPIO_HPHONE_PLUG);
	else
		__gpio_as_irq_rise_edge(GPIO_HPHONE_PLUG);

	n516_ext_control(codec);
}

static irqreturn_t n516_hplug_irq(int irq, void *dev)
{
	printk("%s!\n", __FUNCTION__);
	mod_timer(&n516_hplug_timer, jiffies + msecs_to_jiffies(HPLUG_DETECT_DELAY));

	return IRQ_HANDLED;
}

/* n516 machine dapm widgets */
static const struct snd_soc_dapm_widget jzcodec_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_HP("Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", n516_amp_event),
};

/* n516 machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MICIN", NULL, "Mic"},
};

static const char *spk_function[] = {"Off", "On", "Auto"};
static const struct soc_enum n516_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, spk_function),
};

static const struct snd_kcontrol_new jzcodec_n516_controls[] = {
	SOC_ENUM_EXT("Speaker Function", n516_enum[0], n516_get_spk,
		n516_set_spk),
};

/*
 * n516 for a jzcodec as connected on jz4740 Device
 */
static int n516_jzcodec_init(struct snd_soc_codec *codec)
{
	int i, err;

	n516_headphone_present = __gpio_get_pin(GPIO_HPHONE_PLUG);
	setup_timer(&n516_hplug_timer, n516_hplug_timer_func, (unsigned long)codec);

	if (n516_headphone_present)
		__gpio_as_irq_fall_edge(GPIO_HPHONE_PLUG);
	else
		__gpio_as_irq_rise_edge(GPIO_HPHONE_PLUG);

	snd_soc_dapm_disable_pin(codec, "LLINEIN");
	snd_soc_dapm_disable_pin(codec, "RLINEIN");

	/* Add n516 specific controls */
	for (i = 0; i < ARRAY_SIZE(jzcodec_n516_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&jzcodec_n516_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}

	/* Add n516 specific widgets */
	for(i = 0; i < ARRAY_SIZE(jzcodec_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &jzcodec_dapm_widgets[i]);
	}

	/* Set up n516 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
	return 0;
}

/* n516 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link n516_dai = {
	.name = "JZCODEC",
	.stream_name = "JZCODEC",
	.cpu_dai = &jz4740_i2s_dai,
	.codec_dai = &jzcodec_dai,
	.init = n516_jzcodec_init,
	.ops = &n516_ops,
};

/* n516 audio machine driver */
static struct snd_soc_card snd_soc_n516 = {
	.name = "n516",
	.platform = &jz4740_soc_platform,
	.dai_link = &n516_dai,
	.num_links = 1,
};

/* n516 audio subsystem */
static struct snd_soc_device n516_snd_devdata = {
	.card = &snd_soc_n516,
	.codec_dev = &soc_codec_dev_jzcodec,
};

static struct platform_device *n516_snd_device;

static int __init n516_init(void)
{
	int ret;

	n516_snd_device = platform_device_alloc("soc-audio", -1);

	if (!n516_snd_device)
		return -ENOMEM;

	platform_set_drvdata(n516_snd_device, &n516_snd_devdata);
	n516_snd_devdata.dev = &n516_snd_device->dev;
	ret = platform_device_add(n516_snd_device);

	if (ret)
		platform_device_put(n516_snd_device);

	__gpio_enable_pull(GPIO_HPHONE_PLUG);
	__gpio_as_input(GPIO_HPHONE_PLUG);
	ret = request_irq(gpio_to_irq(GPIO_HPHONE_PLUG),
			n516_hplug_irq, IRQF_SHARED,
			"Headphone detect", (void *)123123);
	if (ret) {
		dev_err(&n516_snd_device->dev, "Unable to request headphone detection IRQ\n");
		return ret;
	}


	return ret;
}

static void __exit n516_exit(void)
{
	free_irq(gpio_to_irq(GPIO_HPHONE_PLUG), (void *)123123);
	__gpio_as_input(GPIO_HPHONE_PLUG);
	platform_device_unregister(n516_snd_device);
}

module_init(n516_init);
module_exit(n516_exit);

/* Module information */
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_DESCRIPTION("ALSA SoC n516");
MODULE_LICENSE("GPL");
