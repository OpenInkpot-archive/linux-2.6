/*
 * drivers/input/touchscreen/e60-ts.c
 *
 * Driver for Samsung E60 touchscreen.
 *
 * Copyright 2011 Alexander Kerner <lunohod@openinkpot.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define DRIVER_DESC "Driver for Samsung E60 touchscreen"

/* Sampling rate */
#define SR_50_PPS (1 << 6)
#define SR_100_PPS (1 << 5)
#define SR_200_PPS (1 << 4)

struct e60_ts_data {
	struct input_dev *dev;
	struct serio *serio;
	unsigned char buf[8];
	int idx;
	char phys[32];
};

static irqreturn_t e60_ts_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct e60_ts_data *ts_data = serio_get_drvdata(serio);
	unsigned char *buf = ts_data->buf;

	if (ts_data->idx == 0 && !(data & 0x80))
		return IRQ_HANDLED;

	buf[ts_data->idx++] = data;

	if (ts_data->idx >= 7) {
		int pr = (buf[5] & 0x7f) | (buf[6] & 0x07) << 7;

		int x = (buf[1] & 0x7f) << 9 | (buf[2] & 0x7f) << 2
			| (buf[6] & 0x60) >> 5;

		int y = (buf[3] & 0x7f) << 9 | (buf[4] & 0x7f) << 2
			| (buf[6] & 0x10) >> 3 | (buf[6] & 0x08) >> 3;

		input_report_abs(ts_data->dev, ABS_X, x);
		input_report_abs(ts_data->dev, ABS_Y, y);
		input_report_abs(ts_data->dev, ABS_PRESSURE, pr);
		input_report_key(ts_data->dev, BTN_TOUCH, pr);
		input_sync(ts_data->dev);

		ts_data->idx = 0;
	}

	return IRQ_HANDLED;
}

static int e60_ts_connect(struct serio *serio, struct serio_driver *drv)
{
	struct e60_ts_data *ts_data;
	struct input_dev *ts_input_dev;
	int ret;

	ts_data = kzalloc(sizeof(struct e60_ts_data), GFP_KERNEL);
	ts_input_dev = input_allocate_device();
	if (!ts_data || !ts_input_dev) {
		ret = -ENOMEM;
		goto err_no_mem;
	}

	ts_data->dev = ts_input_dev;
	ts_data->serio = serio;

	snprintf(ts_data->phys, sizeof(ts_data->phys), "%s/input0", serio->phys);

	ts_input_dev->name = "Samsung E60 touchscreen";
	ts_input_dev->phys = ts_data->phys;
	ts_input_dev->id.bustype = BUS_RS232;
	ts_input_dev->id.vendor = 0xC1;
	ts_input_dev->id.product = 0x0051;
	ts_input_dev->id.version = 0x0100;
	ts_input_dev->dev.parent = &serio->dev;
	ts_input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts_input_dev->absbit[0] = ( BIT_MASK(ABS_X)|BIT_MASK(ABS_Y)|BIT_MASK(ABS_PRESSURE) );
	ts_input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(ts_input_dev, ABS_X, 0, 6144, 0, 0);
	input_set_abs_params(ts_input_dev, ABS_Y, 0, 8192, 0, 0);
	input_set_abs_params(ts_input_dev, ABS_PRESSURE, 0, 1023, 0, 0);

	serio_set_drvdata(serio, ts_data);

	ret = serio_open(serio, drv);
	if (ret)
		goto err_serio_open;

	ret = input_register_device(ts_input_dev);
	if (ret)
		goto err_input_dev_reg;

	serio_write(serio, SR_200_PPS);
	msleep(20);

	return 0;

err_input_dev_reg:
	serio_close(serio);
err_serio_open:
	serio_set_drvdata(serio, NULL);
err_no_mem:
	input_free_device(ts_input_dev);
	kfree(ts_data);

	return ret;
}

static void e60_ts_disconnect(struct serio *serio)
{
	struct e60_ts_data *data = serio_get_drvdata(serio);

	input_get_device(data->dev);
	input_unregister_device(data->dev);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_put_device(data->dev);
	kfree(data);
}

static struct serio_device_id e60_ts_ids[] = {
	{
		.type   = SERIO_RS232,
		.proto  = SERIO_E60_TS,
		.id     = SERIO_ANY,
		.extra  = SERIO_ANY,
	},
	{ 0 }
};

static struct serio_driver e60_ts_drv = {
	.driver = {
		.name	= "e60-ts",
	},
	.description	= DRIVER_DESC,
	.id_table	= e60_ts_ids,
	.interrupt	= e60_ts_interrupt,
	.connect	= e60_ts_connect,
	.disconnect	= e60_ts_disconnect,
};

static int __init e60_ts_init(void)
{
	return serio_register_driver(&e60_ts_drv);
}

static void __exit e60_ts_exit(void)
{
	serio_unregister_driver(&e60_ts_drv);
}

module_init(e60_ts_init);
module_exit(e60_ts_exit);

MODULE_AUTHOR("Alexander Kerner <lunohod@openinkpot.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
