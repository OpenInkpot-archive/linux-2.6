/*
 * e60-display.c -- Platform device for Samsung E60
 *
 * Copyright (C) 2011, Alexander Kerner <lunohod@openinkpot.org>
 * Copyright (C) 2008, Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
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

#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

#include <video/broadsheetfb.h>

#define S5PC100_GPF0_BASE	(S5P_VA_GPIO + 0x00E0)
#define S5PC100_GPF1_BASE	(S5P_VA_GPIO + 0x0100)

#define S5PC100_GPF0DAT	(S5PC100_GPF0_BASE + 0x04)
#define S5PC100_GPF1DAT	(S5PC100_GPF1_BASE + 0x04)

static unsigned int panel_type = 6;
static struct platform_device *e60_device;
static struct broadsheet_board e60_board;

/* register offsets for gpio control */
#define RDY_GPIO_PIN	S5PC100_GPF2(7)
#define DC_GPIO_PIN	S5PC100_GPF2(1)
#define RST_GPIO_PIN	S5PC100_GPF2(6)
#define RD_GPIO_PIN	S5PC100_GPF2(3)
#define WR_GPIO_PIN	S5PC100_GPF2(2)
#define CS_GPIO_PIN	S5PC100_GPF2(0)
#define IRQ_GPIO_PIN	S5PC100_GPF2(4)

static int gpios[] = { RDY_GPIO_PIN, DC_GPIO_PIN, RST_GPIO_PIN, RD_GPIO_PIN,
	WR_GPIO_PIN, CS_GPIO_PIN, IRQ_GPIO_PIN, };
static char *gpio_names[] = { "RDY", "DC", "RST", "RD", "WR", "CS", "IRQ" };

static int e60_wait_event(struct broadsheetfb_par *par)
{
	unsigned long timeout = jiffies + HZ / 20;

	while (!gpio_get_value(RDY_GPIO_PIN) && time_before(jiffies, timeout))
		yield();

	timeout = jiffies + 2 * HZ;

	while (!gpio_get_value(RDY_GPIO_PIN) && time_before(jiffies, timeout))
		;

	if(!gpio_get_value(RD_GPIO_PIN)) {
		printk(KERN_ERR "%s: Wait for READY, timeout\n", __func__);
		return 1;
	}

	return 0;
}

static int e60_init_gpio_regs(struct broadsheetfb_par *par)
{
	int i;
	int err;
	char dbname[8];
	unsigned int pin;

	/* hdb bus */
	for (i = 0; i < 16; i++) {
		sprintf(dbname, "DB%d", i);

		if (i < 8)
			pin = S5PC100_GPF0(i);
		else
			pin = S5PC100_GPF1(i%8);

		err = gpio_request(pin, dbname);

		if (err) {
			dev_err(&e60_device->dev, "failed requesting "
				"gpio %d, err=%d\n", i, err);
			goto err_req_gpio2;
		}

		s3c_gpio_cfgpin(pin, S3C_GPIO_SFN(1));
		gpio_direction_output(pin, 0);
	}

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		err = gpio_request(gpios[i], gpio_names[i]);
		if (err) {
			dev_err(&e60_device->dev, "failed requesting "
				"gpio %s, err=%d\n", gpio_names[i], err);
			goto err_req_gpio;
		}
	}

	for (i = 0; i < 7; i++)
		s3c_gpio_cfgpin(S5PC100_GPF2(i), S3C_GPIO_SFN(1));

	s3c_gpio_cfgpin(S5PC100_GPF2(i), S3C_GPIO_SFN(0));

	/* setup the outputs and init values */
	gpio_direction_output(DC_GPIO_PIN, 0);
	gpio_direction_output(RD_GPIO_PIN, 1);
	gpio_direction_output(WR_GPIO_PIN, 1);
	gpio_direction_output(CS_GPIO_PIN, 1);
	gpio_direction_output(RST_GPIO_PIN, 0);

	/* setup the inputs */
	gpio_direction_input(RDY_GPIO_PIN);
	gpio_direction_input(IRQ_GPIO_PIN);

	/* go into command mode */
	gpio_set_value(RST_GPIO_PIN, 0);
	msleep(10);
	gpio_set_value(RST_GPIO_PIN, 1);
	msleep(10);

	e60_wait_event(par);

	return 0;

err_req_gpio2:
	while (--i >= 0) {
		if (i < 8)
			pin = S5PC100_GPF0(i);
		else
			pin = S5PC100_GPF1(i%8);
		gpio_free(pin);
	}
	i = ARRAY_SIZE(gpios);
err_req_gpio:
	while (--i >= 0)
		gpio_free(gpios[i]);

	return err;
}

static int e60_init_board(struct broadsheetfb_par *par)
{
	return e60_init_gpio_regs(par);
}

static void e60_cleanup(struct broadsheetfb_par *par)
{
	int i;
	unsigned int pin;

	for (i = 0; i < ARRAY_SIZE(gpios); i++)
		gpio_free(gpios[i]);

	for (i = 0; i < 16; i++) {
		if (i < 8)
			pin = S5PC100_GPF0(i);
		else
			pin = S5PC100_GPF1(i%8);

		gpio_free(pin);
	}
}

static u16 e60_get_hdb(struct broadsheetfb_par *par)
{
	u16 res;

	res = readl(S5PC100_GPF0DAT) & 0xff;
	res |= (readl(S5PC100_GPF1DAT) & 0xff) << 8;

	return res;
}

static void e60_set_hdb(struct broadsheetfb_par *par, u16 data)
{
	writel(data & 0xff, S5PC100_GPF0DAT);
	writel((data >> 8) & 0xff, S5PC100_GPF1DAT);
}

static void e60_set_ctl(struct broadsheetfb_par *par, unsigned char bit,
				u8 state)
{
	switch (bit) {
	case BS_CS:
		gpio_set_value(CS_GPIO_PIN, state);
		break;
	case BS_DC:
		gpio_set_value(DC_GPIO_PIN, state);
		break;
	case BS_WR:
		gpio_set_value(WR_GPIO_PIN, state);
		break;
	}
}

static int e60_get_panel_type(void)
{
	return panel_type;
}

static int e60_setup_irq(struct fb_info *info)
{
	return 0;
}

static struct broadsheet_board e60_board = {
	.owner			= THIS_MODULE,
	.init			= e60_init_board,
	.cleanup		= e60_cleanup,
	.set_hdb		= e60_set_hdb,
	.get_hdb		= e60_get_hdb,
	.set_ctl		= e60_set_ctl,
	.wait_for_rdy		= e60_wait_event,
	.get_panel_type		= e60_get_panel_type,
	.setup_irq		= e60_setup_irq,
	.panel_rotation		= FB_ROTATE_CCW,
};

int __init e60_init(void)
{
	int ret;

	/* request our platform independent driver */
	request_module("broadsheetfb");

	e60_device = platform_device_alloc("broadsheetfb", -1);
	if (!e60_device)
		return -ENOMEM;

	/* the e60_board that will be seen by broadsheetfb is a copy */
	platform_device_add_data(e60_device, &e60_board,
					sizeof(e60_board));

	ret = platform_device_add(e60_device);

	if (ret) {
		platform_device_put(e60_device);
		return ret;
	}

	return 0;
}

module_param(panel_type, uint, 0);
MODULE_PARM_DESC(panel_type, "Select the panel type: 37, 6, 97");

MODULE_DESCRIPTION("board driver for samsung e60");
MODULE_AUTHOR("Alexander Kerner");
MODULE_LICENSE("GPL");
