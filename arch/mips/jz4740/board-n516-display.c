/*
 * board-n516-display.c -- Platform device for N516 display
 *
 * Copyright (C) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

/* #define DEBUG */

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

#include <asm/jzlcd.h>
#include <asm/jzsoc.h>

#include <video/metronomefb.h>

#define STDBY_GPIO_PIN	GPIO_DISPLAY_STBY
#define RST_GPIO_PIN	GPIO_DISPLAY_RST_L
#define RDY_GPIO_PIN	GPIO_DISPLAY_RDY
#define ERR_GPIO_PIN	GPIO_DISPLAY_ERR
#define RDY_GPIO_IRQ	(IRQ_GPIO_0 + RDY_GPIO_PIN)

extern struct platform_device jz_lcd_device;

static struct jzfb_info n516_fbinfo = {
	.fclk		= 50,
	.w		= 800,
	.h		= 624,
	.bpp		= 16,
	.hsw		= 31,
	.vsw		= 23,
	.elw		= 31,
	.blw		= 5,
	.efw		= 2,
	.bfw		= 1,
	.cfg		= MODE_TFT_GEN | HSYNC_P | VSYNC_P | PSM_DISABLE | CLSM_DISABLE | SPLM_DISABLE | REVM_DISABLE,
};


static struct platform_device *n516_device;
static struct metronome_board n516_board;

static int n516_init_gpio_regs(struct metronomefb_par *par)
{
	__gpio_as_output(STDBY_GPIO_PIN);
	__gpio_as_output(RST_GPIO_PIN);
	__gpio_clear_pin(STDBY_GPIO_PIN);
	__gpio_clear_pin(RST_GPIO_PIN);

	__gpio_as_input(RDY_GPIO_PIN);
	__gpio_as_input(ERR_GPIO_PIN);

	__gpio_as_irq_rise_edge(RDY_GPIO_PIN);

	__gpio_as_output(GPIO_DISP_OFF_N);
	__gpio_clear_pin(GPIO_DISP_OFF_N);

	return 0;
}

static int n516_share_video_mem(struct fb_info *info)
{
	dev_dbg(&n516_device->dev, "ENTER %s\n", __func__);
	/* rough check if this is our desired fb and not something else */
	if ((info->var.xres != n516_fbinfo.w)
		|| (info->var.yres != n516_fbinfo.h))
		return 0;

	/* we've now been notified that we have our new fb */
	n516_board.metromem = info->screen_base;
	n516_board.host_fbinfo = info;

	/* try to refcount host drv since we are the consumer after this */
	if (!try_module_get(info->fbops->owner))
		return -ENODEV;

	return 0;
}

static int n516_unshare_video_mem(struct fb_info *info)
{
	dev_dbg(&n516_device->dev, "ENTER %s\n", __func__);

	if (info != n516_board.host_fbinfo)
		return 0;

	module_put(n516_board.host_fbinfo->fbops->owner);
	return 0;
}

static int n516_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct fb_info *info = evdata->info;

	dev_dbg(&n516_device->dev, "ENTER %s\n", __func__);

	if (event == FB_EVENT_FB_REGISTERED)
		return n516_share_video_mem(info);
	else if (event == FB_EVENT_FB_UNREGISTERED)
		return n516_unshare_video_mem(info);

	return 0;
}

static struct notifier_block n516_fb_notif = {
	.notifier_call = n516_fb_notifier_callback,
};

/* this gets called as part of our init. these steps must be done now so
 * that we can use set_pxa_fb_info */
static void __init n516_presetup_fb(void)
{
	int fw;
	int fh;
	int padding_size;
	int totalsize;

	/* the frame buffer is divided as follows:
	command | CRC | padding
	16kb waveform data | CRC | padding
	image data | CRC
	*/

	fw = n516_fbinfo.w;
	fh = n516_fbinfo.h;

	/* waveform must be 16k + 2 for checksum */
	n516_board.wfm_size = roundup(16*1024 + 2, fw);

	padding_size = PAGE_SIZE + (4 * fw);

	/* total is 1 cmd , 1 wfm, padding and image */
	totalsize = fw + n516_board.wfm_size + padding_size + (fw*fh);

	/* save this off because we're manipulating fw after this and
	 * we'll need it when we're ready to setup the framebuffer */
	n516_board.fw = fw;
	n516_board.fh = fh;

	/* the reason we do this adjustment is because we want to acquire
	 * more framebuffer memory without imposing custom awareness on the
	 * underlying driver */
	n516_fbinfo.h = DIV_ROUND_UP(totalsize, fw);

	/* we divide since we told the LCD controller we're 16bpp */
	n516_fbinfo.w /= 2;

	jz_lcd_device.dev.platform_data = &n516_fbinfo;
	platform_device_register(&jz_lcd_device);
}

/* this gets called by metronomefb as part of its init, in our case, we
 * have already completed initial framebuffer init in presetup_fb so we
 * can just setup the fb access pointers */
static int n516_setup_fb(struct metronomefb_par *par)
{
	int fw;
	int fh;

	fw = n516_board.fw;
	fh = n516_board.fh;

	/* metromem was set up by the notifier in share_video_mem so now
	 * we can use its value to calculate the other entries */
	par->metromem_cmd = (struct metromem_cmd *) n516_board.metromem;
	par->metromem_wfm = n516_board.metromem + fw;
	par->metromem_img = par->metromem_wfm + n516_board.wfm_size;
	par->metromem_img_csum = (u16 *) (par->metromem_img + (fw * fh));
	par->metromem_dma = n516_board.host_fbinfo->fix.smem_start;

	return 0;
}

static int n516_get_panel_type(void)
{
	return 5;
}

static irqreturn_t n516_handle_irq(int irq, void *dev_id)
{
	struct metronomefb_par *par = dev_id;

	dev_dbg(&par->pdev->dev, "Metronome IRQ! RDY=%d\n", __gpio_get_pin(RDY_GPIO_PIN));
	wake_up_all(&par->waitq);

	return IRQ_HANDLED;
}

static void n516_power_ctl(struct metronomefb_par *par, int cmd)
{
	switch (cmd) {
	case METRONOME_POWER_OFF:
		__gpio_set_pin(GPIO_DISP_OFF_N);
		break;
	case METRONOME_POWER_ON:
		__gpio_clear_pin(GPIO_DISP_OFF_N);
		break;
	}
}

static inline int get_rdy(void)
{
	return __gpio_get_pin(RDY_GPIO_PIN);
}

static int n516_get_err(struct metronomefb_par *par)
{
	return __gpio_get_pin(ERR_GPIO_PIN);
}

static int n516_setup_irq(struct fb_info *info)
{
	int ret;

	dev_dbg(&n516_device->dev, "ENTER %s\n", __func__);
	__gpio_as_irq_rise_edge(RDY_GPIO_PIN);
	ret = request_irq(RDY_GPIO_IRQ, n516_handle_irq,
				IRQF_DISABLED,
				"n516", info->par);
	__gpio_unmask_irq(RDY_GPIO_PIN);
	if (ret)
		dev_err(&n516_device->dev, "request_irq failed: %d\n", ret);

	return ret;
}

static void n516_set_rst(struct metronomefb_par *par, int state)
{
	dev_dbg(&n516_device->dev, "ENTER %s, RDY=%d\n", __func__, __gpio_get_pin(RDY_GPIO_PIN));
	if (state)
		__gpio_set_pin(RST_GPIO_PIN);
	else
		__gpio_clear_pin(RST_GPIO_PIN);
}

static void n516_set_stdby(struct metronomefb_par *par, int state)
{
	dev_dbg(&n516_device->dev, "ENTER %s, RDY=%d\n", __func__, __gpio_get_pin(RDY_GPIO_PIN));
	if (state)
		__gpio_set_pin(STDBY_GPIO_PIN);
	else
		__gpio_clear_pin(STDBY_GPIO_PIN);
}

static int n516_wait_event(struct metronomefb_par *par)
{
	unsigned long timeout = jiffies + HZ/20;

	while (get_rdy() && time_before(jiffies, timeout))
		schedule();

	dev_dbg(&n516_device->dev, "ENTER %s, RDY=%d\n", __func__, __gpio_get_pin(RDY_GPIO_PIN));
	return wait_event_timeout(par->waitq, get_rdy(), HZ * 3) ? 0 : -EIO;
}

static int n516_wait_event_intr(struct metronomefb_par *par)
{
	unsigned long timeout = jiffies + HZ/20;

	while (get_rdy() && time_before(jiffies, timeout))
		schedule();

	dev_dbg(&n516_device->dev, "ENTER %s, RDY=%d\n", __func__, __gpio_get_pin(RDY_GPIO_PIN));
	return wait_event_interruptible_timeout(par->waitq,
					get_rdy(), HZ * 3) ? 0 : -EIO;
}

static void n516_cleanup(struct metronomefb_par *par)
{
	__gpio_as_input(RDY_GPIO_PIN);
	free_irq(RDY_GPIO_IRQ, par);
}

static struct metronome_board n516_board = {
	.owner			= THIS_MODULE,
	.power_ctl		= n516_power_ctl,
	.setup_irq		= n516_setup_irq,
	.setup_io		= n516_init_gpio_regs,
	.setup_fb		= n516_setup_fb,
	.set_rst		= n516_set_rst,
	.get_err		= n516_get_err,
	.set_stdby		= n516_set_stdby,
	.met_wait_event		= n516_wait_event,
	.met_wait_event_intr	= n516_wait_event_intr,
	.get_panel_type		= n516_get_panel_type,
	.cleanup		= n516_cleanup,
};

static int __init n516_init(void)
{
	int ret;

	/* before anything else, we request notification for any fb
	 * creation events */
	fb_register_client(&n516_fb_notif);

	n516_device = platform_device_alloc("metronomefb", -1);
	if (!n516_device)
		return -ENOMEM;

	/* the n516_board that will be seen by metronomefb is a copy */
	platform_device_add_data(n516_device, &n516_board,
					sizeof(n516_board));

	n516_presetup_fb();
	/* this _add binds metronomefb to n516. metronomefb refcounts n516 */
	ret = platform_device_add(n516_device);

	if (ret) {
		platform_device_put(n516_device);
		fb_unregister_client(&n516_fb_notif);
		return ret;
	}

	/* request our platform independent driver */
	request_module("metronomefb");


	return 0;
}

module_init(n516_init);

MODULE_DESCRIPTION("board driver for n516 display");
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL");
