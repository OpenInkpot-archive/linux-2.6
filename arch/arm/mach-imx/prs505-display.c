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

#define DEBUG

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

#include <mach/imxfb.h>
#include <mach/irqs.h>
#include <video/metronomefb.h>

#include "generic.h"

static struct platform_device *prs505_display_device;
static struct metronome_board prs505_board;

struct imxfb_mach_info prs505_fb_info = {
	.pixclock	= 40189,
	.xres		= 832,
	.yres		= 622 / 2,
	.bpp		= 16,
	.hsync_len	= 28,
	.left_margin	= 34,
	.right_margin	= 34,
	.vsync_len	= 25,
	.upper_margin	= 0,
	.lower_margin	= 2,
	.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.pcr		= PCR_TFT | PCR_BPIX_16 | PCR_CLKPOL | PCR_SCLKIDLE |
				PCR_SCLK_SEL | PCR_PCD(2),
	.dmacr		= DMACR_HM(3) | DMACR_TM(10), 
};

#define STDBY_GPIO_PIN		(GPIO_PORTD | 10)
#define RST_GPIO_PIN		(GPIO_PORTD | 11)
#define RDY_GPIO_PIN		(GPIO_PORTD | 9)
#define ERR_GPIO_PIN		(GPIO_PORTD | 7)
#define PWR_VIO_GPIO_PIN	(GPIO_PORTA | 2)
#define PWR_VCORE_GPIO_PIN	(GPIO_PORTA | 4)

static int gpios[] = {  STDBY_GPIO_PIN , RST_GPIO_PIN,
			RDY_GPIO_PIN, ERR_GPIO_PIN, PWR_VCORE_GPIO_PIN,
			PWR_VIO_GPIO_PIN };

static char *gpio_names[] = { "STDBY" , "RST", "RDY", "ERR",
	"VCORE", "VIO" };

static int prs505_init_gpio_regs(struct metronomefb_par *par)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		err = gpio_request(gpios[i], gpio_names[i]);
		if (err) {
			dev_err(&prs505_display_device->dev, "failed requesting "
				"gpio %s, err=%d\n", gpio_names[i], err);
			goto err_req_gpio;
		}
	}

	gpio_direction_output(STDBY_GPIO_PIN, 0);
	gpio_direction_output(RST_GPIO_PIN, 0);

	gpio_direction_input(RDY_GPIO_PIN);
	gpio_direction_input(ERR_GPIO_PIN);

	gpio_direction_output(PWR_VCORE_GPIO_PIN, 1);
	gpio_direction_output(PWR_VIO_GPIO_PIN, 1);

	mdelay(10);

	return 0;

err_req_gpio:
	while (i > 0)
		gpio_free(gpios[i--]);

	return err;
}

static void prs505_cleanup(struct metronomefb_par *par)
{
	int i;

	free_irq(gpio_to_irq(RDY_GPIO_PIN), par);

	for (i = 0; i < ARRAY_SIZE(gpios); i++)
		gpio_free(gpios[i]);
}

static int prs505_share_video_mem(struct fb_info *info)
{
	/* rough check if this is our desired fb and not something else */
	if ((info->var.xres != prs505_fb_info.xres)
		|| (info->var.yres != prs505_fb_info.yres))
		return 0;

	/* we've now been notified that we have our new fb */
	prs505_board.metromem = info->screen_base;
	prs505_board.host_fbinfo = info;

	/* try to refcount host drv since we are the consumer after this */
	if (!try_module_get(info->fbops->owner))
		return -ENODEV;

	return 0;
}

static int prs505_unshare_video_mem(struct fb_info *info)
{
	dev_dbg(&prs505_display_device->dev, "ENTER %s\n", __func__);

	if (info != prs505_board.host_fbinfo)
		return 0;

	module_put(prs505_board.host_fbinfo->fbops->owner);
	return 0;
}

static int prs505_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct fb_info *info = evdata->info;

//	dev_dbg(&prs505_display_device->dev, "ENTER %s\n", __func__);

	if (event == FB_EVENT_FB_REGISTERED)
		return prs505_share_video_mem(info);
	else if (event == FB_EVENT_FB_UNREGISTERED)
		return prs505_unshare_video_mem(info);

	return 0;
}

static struct notifier_block prs505_fb_notif = {
	.notifier_call = prs505_fb_notifier_callback,
};

/* this gets called as part of our init. these steps must be done now so
 * that we can use set_pxa_fb_info */
static int __init prs505_presetup_fb(void)
{
	int fw;
	int fh;
	int padding_size;
	int totalsize;
	int ret;

	/* the frame buffer is divided as follows:
	command | CRC | padding
	16kb waveform data | CRC | padding
	image data | CRC
	*/

	fw = prs505_fb_info.xres;
	fh = prs505_fb_info.yres;

	/* waveform must be 16k + 2 for checksum */
	prs505_board.wfm_size = roundup(16*1024 + 2, fw);

	padding_size = PAGE_SIZE + (4 * fw);

	/* total is 1 cmd , 1 wfm, padding and image */
	totalsize = fw + prs505_board.wfm_size + padding_size + (fw*fh);

	/* save this off because we're manipulating fw after this and
	 * we'll need it when we're ready to setup the framebuffer */
	prs505_board.fw = fw;
	prs505_board.fh = fh;

	/* the reason we do this adjustment is because we want to acquire
	 * more framebuffer memory without imposing custom awareness on the
	 * underlying pxafb driver */
	prs505_fb_info.yres = DIV_ROUND_UP(totalsize, fw);

	/* we divide since we told the LCD controller we're 16bpp */
	prs505_fb_info.xres /= 2;

	set_imx_fb_info(&prs505_fb_info);

	ret = platform_device_register(&imxfb_device);
	if (ret)
		dev_err(&prs505_display_device->dev, "Failed to register imxfb platform device\n");

	return ret;
}

/* this gets called by metronomefb as part of its init, in our case, we
 * have already completed initial framebuffer init in presetup_fb so we
 * can just setup the fb access pointers */
static int prs505_setup_fb(struct metronomefb_par *par)
{
	int fw;
	int fh;

	fw = prs505_board.fw;
	fh = prs505_board.fh;

	/* metromem was set up by the notifier in share_video_mem so now
	 * we can use its value to calculate the other entries */
	par->metromem_cmd = (struct metromem_cmd *) prs505_board.metromem;
	par->metromem_wfm = prs505_board.metromem + fw;
	par->metromem_img = par->metromem_wfm + prs505_board.wfm_size;
	par->metromem_img_csum = (u16 *) (par->metromem_img + (fw * fh));
	par->metromem_dma = prs505_board.host_fbinfo->fix.smem_start;

	return 0;
}

static int prs505_get_panel_type(void)
{
	return 6;
}

static irqreturn_t prs505_handle_irq(int irq, void *dev_id)
{
	struct metronomefb_par *par = dev_id;

	wake_up_interruptible(&par->waitq);
	printk("metronome IRQ\n");
	return IRQ_HANDLED;
}

static int prs505_setup_irq(struct fb_info *info)
{
	int ret;

	ret = request_irq(gpio_to_irq(RDY_GPIO_PIN), prs505_handle_irq,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"PRS505-display", info->par);
	if (ret)
		dev_err(&prs505_display_device->dev, "request_irq failed: %d\n", ret);

	return ret;
}

static void prs505_set_rst(struct metronomefb_par *par, int state)
{
	printk("%s: will set RST to %d, RDY = %d\n", __FUNCTION__, state, gpio_get_value(RDY_GPIO_PIN));
	if (state && gpio_get_value(RDY_GPIO_PIN))
		dev_err(&prs505_display_device->dev, "RDY != 0 before set_rst\n");
	gpio_set_value(RST_GPIO_PIN, state);
}

static void prs505_set_stdby(struct metronomefb_par *par, int state)
{
	printk("%s: will set STDBY to %d, RDY = %d\n", __FUNCTION__, state, gpio_get_value(RDY_GPIO_PIN));
	gpio_set_value(STDBY_GPIO_PIN, state);
}

static int prs505_wait_event(struct metronomefb_par *par)
{
	int ret;
	ret = wait_event_timeout(par->waitq, gpio_get_value(RDY_GPIO_PIN), HZ);
	dev_dbg(&prs505_display_device->dev, "%s: ret = %d\n", __FUNCTION__, ret);
	return ret ? 0 : -EIO;
}

static int prs505_wait_event_intr(struct metronomefb_par *par)
{
	return wait_event_interruptible_timeout(par->waitq,
			gpio_get_value(RDY_GPIO_PIN), HZ) ? 0 : -EIO;
}

static struct metronome_board prs505_board = {
	.owner			= THIS_MODULE,
	.setup_irq		= prs505_setup_irq,
	.setup_io		= prs505_init_gpio_regs,
	.setup_fb		= prs505_setup_fb,
	.set_rst		= prs505_set_rst,
	.set_stdby		= prs505_set_stdby,
	.met_wait_event		= prs505_wait_event,
	.met_wait_event_intr	= prs505_wait_event_intr,
	.get_panel_type		= prs505_get_panel_type,
	.cleanup		= prs505_cleanup,
};

static int __init prs505_init(void)
{
	int ret;

	/* before anything else, we request notification for any fb
	 * creation events */
	fb_register_client(&prs505_fb_notif);

	prs505_presetup_fb();

	if (!prs505_board.host_fbinfo) {
		printk(KERN_ERR "prs505_board.host_fbinfo is NULL. Probably you don't have the imxfb driver enabled\n");
		platform_device_unregister(&imxfb_device);
		return -ENODEV;
	}
	/* request our platform independent driver */
	request_module("metronomefb");

	prs505_display_device = platform_device_alloc("metronomefb", -1);
	if (!prs505_display_device) {
		platform_device_unregister(&imxfb_device);
		return -ENOMEM;
	}

	/* the prs505_board that will be seen by metronomefb is a copy */
	platform_device_add_data(prs505_display_device, &prs505_board,
					sizeof(prs505_board));

	/* this _add binds metronomefb to prs505. metronomefb refcounts prs505 */
	ret = platform_device_add(prs505_display_device);

	if (ret) {
		platform_device_put(prs505_display_device);
		fb_unregister_client(&prs505_fb_notif);
		platform_device_unregister(&imxfb_device);
		return ret;
	}

	return 0;
}

static void __exit prs505_exit(void)
{
		platform_device_put(prs505_display_device);
		fb_unregister_client(&prs505_fb_notif);
		platform_device_unregister(&imxfb_device);
}


module_init(prs505_init);
module_exit(prs505_exit);

MODULE_DESCRIPTION("board driver for Sony PRS-505 book reader display controller");
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL");


