/*
 * prs505-display.c -- Platform device for Sony PRS-505 display driver
 *
 * Copyright (C) 2008, Yauhen Kharuzhy
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * This driver is written to be used with the Metronome display controller.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/console.h>

#include <mach/imxfb.h>
#include <mach/irqs.h>
#include <mach/iomux.h>

#include <video/metronomefb.h>

#include "devices.h"

static struct platform_device *prs505_device;

struct prs505_metronome_info {
	uint8_t *metromem;
	size_t wfm_size;
	struct fb_info *host_fbinfo; /* the host LCD controller's fbi */
	unsigned int fw;
	unsigned int fh;
	unsigned int gap;
};

static struct work_struct prs505_init_work;
static struct completion prs505_init_completed;
static struct prs505_metronome_info prs505_metronome_info;

struct imx_fb_videomode prs505_fb_modes[] = {
	{
		.mode = {
			.name		= "Metronome-800x600 dual",
			.refresh	= 50,
			.xres		= 864,
			.yres		= 312,
			.pixclock	= 16000000,
			.hsync_len	= 10,
			.left_margin	= 13,
			.right_margin	= 9,
			.vsync_len	= 15,
			.upper_margin	= 15,
			.lower_margin	= 15,
			.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		},
		.bpp		= 16,
		.pcr		= PCR_TFT | PCR_CLKPOL | PCR_SCLKIDLE |
				PCR_SCLK_SEL,
	},
};

struct imx_fb_platform_data prs505_fb_data = {
	.mode		= prs505_fb_modes,
	.num_modes	= ARRAY_SIZE(prs505_fb_modes),
	.dmacr		= DMACR_BURST | DMACR_HM(8) | DMACR_TM(4),
};

#define STDBY_GPIO_PIN		(GPIO_PORTD | 10)
#define RST_GPIO_PIN		(GPIO_PORTD | 11)
#define RDY_GPIO_PIN		(GPIO_PORTD | 9)
#define ERR_GPIO_PIN		(GPIO_PORTD | 7)
#define PWR_VIO_GPIO_PIN	(GPIO_PORTA | 2)
#define PWR_VCORE_GPIO_PIN	(GPIO_PORTA | 4)

static int gpios[] = {  STDBY_GPIO_PIN , RST_GPIO_PIN,
			RDY_GPIO_PIN, ERR_GPIO_PIN };

static char *gpio_names[] = { "STDBY" , "RST", "RDY", "ERR",
	"VCORE", "VIO" };

static int prs505_init_gpio_regs(struct metronomefb_par *par)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		err = gpio_request(gpios[i], gpio_names[i]);
		if (err) {
			dev_err(&prs505_device->dev, "failed requesting "
				"gpio %s, err=%d\n", gpio_names[i], err);
			goto err_req_gpio;
		}
	}

	gpio_direction_output(STDBY_GPIO_PIN, 0);
	gpio_direction_output(RST_GPIO_PIN, 0);

	gpio_direction_input(RDY_GPIO_PIN);
	gpio_direction_input(ERR_GPIO_PIN);

	gpio_set_value(PWR_VCORE_GPIO_PIN, 1);
	gpio_set_value(PWR_VIO_GPIO_PIN, 1);

	mdelay(10);

	return 0;

err_req_gpio:
	while (i > 0)
		gpio_free(gpios[--i]);

	return err;
}

static void prs505_cleanup(struct metronomefb_par *par)
{
	int i;

	free_irq(gpio_to_irq(RDY_GPIO_PIN), par);

	for (i = 0; i < ARRAY_SIZE(gpios); i++)
		gpio_free(gpios[i]);
}

static void prs505_enable_hostfb(bool enable)
{
	int blank;

	acquire_console_sem();

	blank = enable ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
	fb_blank(prs505_metronome_info.host_fbinfo, blank);

	release_console_sem();
}

static void prs505_init_worker(struct work_struct *work)
{
	int ret;

	dev_dbg(&prs505_device->dev, "ENTER %s\n", __FUNCTION__);
	/* Disable host fb until we need it */
	prs505_enable_hostfb(0);

	/* try to refcount host drv since we are the consumer after this */
	if (!try_module_get(prs505_metronome_info.host_fbinfo->fbops->owner)) {
		dev_err(&prs505_device->dev, "Failed to get module\n");
		return;
	}

	/* this _add binds metronomefb to prs505. metronomefb refcounts prs505 */
	ret = platform_device_add(prs505_device);

	if (ret) {
		platform_device_put(prs505_device);
		dev_err(&prs505_device->dev,
				"platform_device_add() has failed\n");
		return;
	}

	/* request our platform independent driver */
	request_module("metronomefb");
	complete(&prs505_init_completed);
	dev_dbg(&prs505_device->dev, "EXIT %s\n", __FUNCTION__);
}


static int prs505_share_video_mem(struct fb_info *info)
{
	dev_dbg(&prs505_device->dev, "ENTER %s\n", __func__);
	/* rough check if this is our desired fb and not something else */
	if ((info->var.xres != prs505_fb_modes[0].mode.xres)
		|| (info->var.yres != prs505_fb_modes[0].mode.yres))
		return 0;

	/* we've now been notified that we have our new fb */
	prs505_metronome_info.metromem = info->screen_base;
	prs505_metronome_info.host_fbinfo = info;

	schedule_work(&prs505_init_work);

	return 0;
}

static int prs505_unshare_video_mem(struct fb_info *info)
{
	dev_dbg(&prs505_device->dev, "ENTER %s\n", __func__);

	if (info != prs505_metronome_info.host_fbinfo)
		return 0;

	module_put(prs505_metronome_info.host_fbinfo->fbops->owner);
	return 0;
}

static int prs505_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct fb_info *info = evdata->info;

	switch (event) {
	case FB_EVENT_FB_REGISTERED:
		return prs505_share_video_mem(info);
	case FB_EVENT_FB_UNREGISTERED:
		return prs505_unshare_video_mem(info);
	default:
		break;
	}

	return 0;
}

static struct notifier_block prs505_fb_notif = {
	.notifier_call = prs505_fb_notifier_callback,
};

/* this gets called as part of our init. these steps must be done now so
 * that we can use set_pxa_fb_info */
static void __init prs505_presetup_fb(void)
{
	/* the frame buffer is divided as follows:
	command | CRC | padding
	16kb waveform data | CRC | padding
	image data | CRC
	*/

	/* Double-width framefuffer:
	 *
	 * Metronome virtual framebuffer:
	 * 400px   64px  400px
	 * line1 | GAP | line2
	 * line3 | GAP | line4
	 * ...
	 *
	 *
	 * Framebuffer layout:
	 * 400px  64px  400px
	 * CMD	| GAP | WF		| 1
	 * WF	| GAP | WF		| 2
	 * ...
	 * WF	| GAP | WF + padding	| 11
	 * IMG	| GAP | IMG		| 12
	 * ...
	 * IMG	| GAP | IMG		| 311
	 * CRC	| GAP | padding		| 312
	 *
	 */

	prs505_metronome_info.fw = 800;
	prs505_metronome_info.fh = 600;
	prs505_metronome_info.gap = 128;


	/* waveform must be 16k + 2 for checksum */
	/* We have double-width framebuffer, so calculate WFM size as
	 * 10 double-lines * 2 bytes per pixel + one half-line
	 */
	prs505_metronome_info.wfm_size = 10 * 864 * 2 + 800;

	imx_fb_device.dev.platform_data = &prs505_fb_data;
	platform_device_register(&imx_fb_device);
}

/* this gets called by metronomefb as part of its init, in our case, we
 * have already completed initial framebuffer init in presetup_fb so we
 * can just setup the fb access pointers */
static int prs505_setup_fb(struct metronomefb_par *par)
{
	/* metromem was set up by the notifier in share_video_mem so now
	 * we can use its value to calculate the other entries */
	par->metromem_cmd = (struct metromem_cmd *) prs505_metronome_info.metromem;

	par->metromem_wfm = prs505_metronome_info.metromem +
		prs505_metronome_info.fw + prs505_metronome_info.gap;
	par->metromem_wfm_csum = ((u16 *)(par->metromem_cmd)) + 18592 / 2;

	par->metromem_img = par->metromem_wfm + prs505_metronome_info.wfm_size;

	par->metromem_img_csum = (u16 *) (par->metromem_img +
		((prs505_metronome_info.fw * 2 + prs505_metronome_info.gap) *
		prs505_metronome_info.fh / 2));

	par->metromem_dma = prs505_metronome_info.host_fbinfo->fix.smem_start;

	return 0;
}

static int prs505_get_panel_type(void)
{
	return 6;
}

static int prs505_get_rdy(struct metronomefb_par *par)
{
	return gpio_get_value(RDY_GPIO_PIN);
}

static int prs505_get_err(struct metronomefb_par *par)
{
	return gpio_get_value(ERR_GPIO_PIN);
}

static irqreturn_t prs505_handle_irq(int irq, void *dev_id)
{
	struct metronomefb_par *par = dev_id;

	wake_up_all(&par->waitq);
	dev_dbg(&prs505_device->dev, "IRQ, ERR=%d\n", prs505_get_err(par));

	return IRQ_HANDLED;
}

static void prs505_power_ctl(struct metronomefb_par *par, int cmd)
{
	dev_dbg(&prs505_device->dev, "ENTER %s, cmd=%d\n", __FUNCTION__, cmd);

	switch (cmd) {
	case METRONOME_POWER_OFF:
		fb_blank(prs505_metronome_info.host_fbinfo, FB_BLANK_NORMAL);
		gpio_set_value(PWR_VCORE_GPIO_PIN, 0);
		gpio_set_value(PWR_VIO_GPIO_PIN, 0);
		break;
	case METRONOME_POWER_ON:
		gpio_set_value(PWR_VCORE_GPIO_PIN, 1);
		gpio_set_value(PWR_VIO_GPIO_PIN, 1);
		fb_blank(prs505_metronome_info.host_fbinfo, FB_BLANK_UNBLANK);
		break;
	}
}

static int prs505_setup_irq(struct fb_info *info)
{
	int ret;

	ret = request_irq(gpio_to_irq(RDY_GPIO_PIN), prs505_handle_irq,
				IRQF_TRIGGER_RISING,
				"PRS505-display", info->par);
	if (ret)
		dev_err(&prs505_device->dev, "request_irq failed: %d\n", ret);

	return ret;
}

static void prs505_set_rst(struct metronomefb_par *par, int state)
{
	dev_dbg(&prs505_device->dev, "ENTER %s, RDY=%d\n",
		__FUNCTION__, prs505_get_rdy(par));

	gpio_set_value(RST_GPIO_PIN, !!state);
}

static void prs505_set_stdby(struct metronomefb_par *par, int state)
{
	dev_dbg(&prs505_device->dev, "ENTER %s, RDY=%d\n",
			__FUNCTION__, prs505_get_rdy(par));

	gpio_set_value(STDBY_GPIO_PIN, !!state);
}

static int prs505_wait_event(struct metronomefb_par *par)
{
	unsigned long timeout = jiffies + HZ / 20;

	dev_dbg(&prs505_device->dev, "ENTER1 %s, RDY=%d\n",
			__FUNCTION__, prs505_get_rdy(par));
	while (prs505_get_rdy(par) && time_before(jiffies, timeout))
		schedule();

	dev_dbg(&prs505_device->dev, "ENTER2 %s, RDY=%d\n",
			__FUNCTION__, prs505_get_rdy(par));
	return wait_event_timeout(par->waitq,
			prs505_get_rdy(par), HZ * 2) ? 0 : -EIO;
}

static int prs505_wait_event_intr(struct metronomefb_par *par)
{
	unsigned long timeout = jiffies + HZ / 20;

	dev_dbg(&prs505_device->dev, "ENTER1 %s, RDY=%d\n",
			__FUNCTION__, prs505_get_rdy(par));
	while (prs505_get_rdy(par) && time_before(jiffies, timeout))
		schedule();

	dev_dbg(&prs505_device->dev, "ENTER2 %s, RDY=%d\n",
			__FUNCTION__, prs505_get_rdy(par));
	return wait_event_interruptible_timeout(par->waitq,
			prs505_get_rdy(par), HZ * 2) ? 0 : -EIO;
}

static struct metronome_board prs505_board = {
	.owner			= THIS_MODULE,
	.power_ctl		= prs505_power_ctl,
	.setup_irq		= prs505_setup_irq,
	.setup_io		= prs505_init_gpio_regs,
	.setup_fb		= prs505_setup_fb,
	.set_rst		= prs505_set_rst,
	.set_stdby		= prs505_set_stdby,
	.get_err		= prs505_get_err,
	.get_rdy		= prs505_get_rdy,
	.met_wait_event		= prs505_wait_event,
	.met_wait_event_intr	= prs505_wait_event_intr,
	.get_panel_type		= prs505_get_panel_type,
	.cleanup		= prs505_cleanup,
	.panel_rotation		= FB_ROTATE_CW,
	/* values for next fields were picked from original PRS-505 kernel sources */
	.double_width_data	= {
		.ddw		= 399,
		.dhw		= 23,
		.dbw		= 19,
		.dew		= 19,
	},
	.pwr_timings		= { 322, 322, 100 },
};

static int __init prs505_init(void)
{
	int ret;

	/* Keep the metronome off, until its driver is loaded */
	ret = gpio_request(PWR_VIO_GPIO_PIN, "Metronome VIO");
	if (ret) {
		printk(KERN_ERR "prs505-display: Cannot request GPIO\n");
		goto err_gpio_metronome_pwr_vio;
	}

	ret = gpio_request(PWR_VCORE_GPIO_PIN, "Metronome VCORE");
	if (ret) {
		printk(KERN_ERR "prs505-display: Cannot request GPIO\n");
		goto err_gpio_metronome_pwr_vcore;
	}

	gpio_direction_output(PWR_VCORE_GPIO_PIN, 0);
	gpio_direction_output(PWR_VIO_GPIO_PIN, 0);

	INIT_WORK(&prs505_init_work, prs505_init_worker);
	init_completion(&prs505_init_completed);

	/* before anything else, we request notification for any fb
	 * creation events */
	fb_register_client(&prs505_fb_notif);

	prs505_device = platform_device_alloc("metronomefb", -1);
	if (!prs505_device) {
		ret = -ENOMEM;
		goto err_pdev_alloc;
	}

	/* the prs505_board that will be seen by metronomefb is a copy */
	platform_device_add_data(prs505_device, &prs505_board,
					sizeof(prs505_board));

	prs505_presetup_fb();

	ret = wait_for_completion_timeout(&prs505_init_completed, HZ * 5);
	if (ret < 0) {
		dev_err(&prs505_device->dev, "Initialization was interrupted\n");
		goto err_wait_for_completion;
	} else {
		if (ret == 0) {
			dev_err(&prs505_device->dev, "Initialization timed out\n");
			ret = -ETIMEDOUT;
			goto err_wait_for_completion;
		}
	}


	return 0;

err_wait_for_completion:
	platform_device_put(prs505_device);
err_pdev_alloc:
	gpio_free(PWR_VCORE_GPIO_PIN);
err_gpio_metronome_pwr_vcore:
	gpio_free(PWR_VIO_GPIO_PIN);
err_gpio_metronome_pwr_vio:

	return ret;
}

static void __exit prs505_exit(void)
{
	gpio_set_value(PWR_VCORE_GPIO_PIN, 0);
	gpio_set_value(PWR_VIO_GPIO_PIN, 0);
	fb_unregister_client(&prs505_fb_notif);
	platform_device_unregister(prs505_device);
}


module_init(prs505_init);
module_exit(prs505_exit);

MODULE_DESCRIPTION("board driver for Sony PRS-505 book reader display controller");
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL");


