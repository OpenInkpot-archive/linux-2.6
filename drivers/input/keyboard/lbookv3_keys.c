/*
 * Driver for lBook/Jinke eReader V3 keys
 *
 * Copyright 2008 Eugene Konev <ejka@imfi.kspu.ru>
 * Copyright 2008 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>

#define KEYB_DELAY		(50 * HZ / 1000)
#define LONGPRESS_TIME		(HZ * 6 / 10) /* 0.6 seconds */

static unsigned long poll_interval = KEYB_DELAY;
static unsigned long longpress_time = LONGPRESS_TIME;

static unsigned long int column_pins[] = {
	S3C2410_GPG(6), S3C2410_GPG(7), S3C2410_GPG(8), S3C2410_GPG(9), S3C2410_GPG(10),
	S3C2410_GPG(11), S3C2410_GPG(4),
};

static unsigned long int row_pins[] = {
	S3C2410_GPF(0), S3C2410_GPF(1), S3C2410_GPF(2),
};

static unsigned long keypad_state[ARRAY_SIZE(row_pins)][ARRAY_SIZE(column_pins)];


static unsigned char keypad_codes[ARRAY_SIZE(row_pins)][ARRAY_SIZE(column_pins)] = {
	{ KEY_1, KEY_7, KEY_4, KEY_0, KEY_KPPLUS, KEY_ENTER, KEY_RIGHT },
	{ KEY_6, KEY_3, KEY_9, KEY_RESERVED, KEY_KPMINUS, KEY_ESC, KEY_LEFT },
	{ KEY_2, KEY_8, KEY_5, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED }
};

static struct timer_list kb_timer;
static struct timer_list power_timer;

static inline void set_col(int col, int to)
{
	s3c2410_gpio_setpin(column_pins[col], to);
}

static void set_columns_to(int to)
{
	int j;

	for (j = 0; j < ARRAY_SIZE(column_pins); j++) {
		while ((!!s3c2410_gpio_getpin(column_pins[j]) != to))
			s3c2410_gpio_setpin(column_pins[j], to);
	}
}

static int wait_for_rows_high(void)
{
	int timeout, i;

	for (timeout = 0xffff; timeout; timeout--) {
		int is_low = 0;

		for (i = 0; i < ARRAY_SIZE(row_pins); i++)
			if (!s3c2410_gpio_getpin(row_pins[i]))
				is_low = 1;
		if (!is_low)
			break;
		udelay(10);
	}
	return timeout;
}

static void cfg_rows_to(unsigned int to)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(row_pins); i++)
		s3c2410_gpio_cfgpin(row_pins[i], to);
}


static void generate_longpress_event(struct input_dev *input, unsigned char key)
{
	input_event(input, EV_KEY, KEY_LEFTALT, 1);
	input_event(input, EV_KEY, key, 1);
	input_event(input, EV_KEY, key, 0);
	input_event(input, EV_KEY, KEY_LEFTALT, 0);
	input_sync(input);
}

static void lbookv3_keys_kb_timer(unsigned long data)
{
	int row, col;
	int pressed = 0;
	struct input_dev *input = (struct input_dev *)data;

	cfg_rows_to(S3C2410_GPIO_INPUT);
	for (col = 0; col < ARRAY_SIZE(column_pins); col++) {
		set_columns_to(1);
		wait_for_rows_high();
		set_col(col, 0);
		udelay(30);

		for (row = 0; row < ARRAY_SIZE(row_pins); row++) {
			if (!s3c2410_gpio_getpin(row_pins[row])) {
				if (!keypad_state[row][col]) {
					keypad_state[row][col] = jiffies + longpress_time;
				} else {
					if (time_after(jiffies, keypad_state[row][col]) &&
							(keypad_state[row][col] > 1)) {
						generate_longpress_event(input, keypad_codes[row][col]);
						keypad_state[row][col] = 1;
					}
				}
				pressed = 1;
			} else {
				if (keypad_state[row][col]) {
					unsigned char key = keypad_codes[row][col];

					if (keypad_state[row][col] > 1) {
						if (time_after(jiffies, keypad_state[row][col])) {
							generate_longpress_event(input, key);
						} else {
							input_event(input, EV_KEY, key, 1);
							input_event(input, EV_KEY, key, 0);
							input_sync(input);
						}
					}

					keypad_state[row][col] = 0;
				}
			}
		}
	}

	set_columns_to(0);
	udelay(10);
	cfg_rows_to(S3C2410_GPIO_IRQ);
	if (pressed & !timer_pending(&kb_timer)) {
		kb_timer.expires = jiffies + poll_interval;
		add_timer(&kb_timer);
	}
}

static unsigned long int power_longpress_timeout;

static void lbookv3_keys_power_timer(unsigned long data)
{
	struct input_dev *input = (struct input_dev *)data;
	static int prev_state = 0;
	int pressed;

	pressed = !s3c2410_gpio_getpin(S3C2410_GPF(6));

	if (pressed) {

		if (time_before(jiffies, power_longpress_timeout)) {
			power_timer.expires = jiffies + poll_interval;
			add_timer(&power_timer);
			prev_state = pressed;
			return;
		} else {
			generate_longpress_event(input, KEY_POWER);
		}

	} else if ((prev_state && !pressed) &&
			time_before(jiffies, power_longpress_timeout)) {
		input_event(input, EV_KEY, KEY_POWER, 1);
		input_event(input, EV_KEY, KEY_POWER, 0);
		input_sync(input);
	}
	prev_state = pressed;

	s3c2410_gpio_cfgpin(S3C2410_GPF(6), S3C2410_GPF6_EINT6);
}

static irqreturn_t lbookv3_powerkey_isr(int irq, void *dev_id)
{
	if (s3c2410_gpio_getpin(S3C2410_GPF(6)))
		return IRQ_HANDLED;

	s3c2410_gpio_cfgpin(S3C2410_GPF(6), S3C2410_GPIO_INPUT);

	power_longpress_timeout = jiffies + longpress_time;
	power_timer.expires = jiffies + poll_interval;
	add_timer(&power_timer);

	return IRQ_HANDLED;
}

static irqreturn_t lbookv3_keys_isr(int irq, void *dev_id)
{
	cfg_rows_to(S3C2410_GPIO_INPUT);

	set_columns_to(1);

	lbookv3_keys_kb_timer((unsigned long)(dev_id));

	return IRQ_HANDLED;
}

static int lbookv3_keys_poll_interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", poll_interval * 1000 / HZ);
}

static int lbookv3_keys_poll_interval_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	poll_interval = simple_strtoul(buf, NULL, 10) * HZ / 1000;

	return size;
}

static int lbookv3_keys_longpress_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", longpress_time * 1000 / HZ);
}

static int lbookv3_keys_longpress_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	longpress_time = simple_strtoul(buf, NULL, 10) * HZ / 1000;

	return size;
}

DEVICE_ATTR(poll_interval, 0644, lbookv3_keys_poll_interval_show,
		lbookv3_keys_poll_interval_store);
DEVICE_ATTR(longpress_time, 0644, lbookv3_keys_longpress_time_show,
		lbookv3_keys_longpress_time_store);

static struct input_dev *input;
static int __init lbookv3_keys_init(void)
{
	int i, j, error;

	for (i = 0; i < ARRAY_SIZE(column_pins); i++) {
		s3c2410_gpio_cfgpin(column_pins[i], S3C2410_GPIO_OUTPUT);
		s3c2410_gpio_setpin(column_pins[i], 0);
	}

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input->evbit[0] = BIT(EV_KEY);

	input->name = "lbookv3-keys";
	input->phys = "lbookv3-keys/input0";

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	setup_timer(&kb_timer, lbookv3_keys_kb_timer, (unsigned long)input);
	setup_timer(&power_timer, lbookv3_keys_power_timer, (unsigned long)input);
	memset(keypad_state, 0, sizeof(keypad_state));

	for (i = 0; i < ARRAY_SIZE(row_pins); i++) {
		for (j = 0; j < 7; j++) {
			if (keypad_codes[i][j] != KEY_RESERVED)
				input_set_capability(input, EV_KEY,
						keypad_codes[i][j]);
				input_set_capability(input, EV_KEY,
						KEY_LEFTALT);
		}
	}

	input_set_capability(input, EV_KEY, KEY_POWER);

	error = input_register_device(input);
	if (error) {
		printk(KERN_ERR "Unable to register lbookv3-keys input device\n");
		input_free_device(input);
		return -ENOMEM;
	}

	error = device_create_file(&input->dev, &dev_attr_poll_interval);
	if (error)
		goto err_add_poll_interval;

	error = device_create_file(&input->dev, &dev_attr_longpress_time);
	if (error)
		goto err_add_longpress_time;


	lbookv3_powerkey_isr(0, input);
	lbookv3_keys_isr(0, input);

	for (i = S3C2410_GPF(0); i <= S3C2410_GPF(2); i++) {
		int irq = s3c2410_gpio_getirq(i);

		s3c2410_gpio_cfgpin(i, S3C2410_GPIO_SFN2);
		s3c2410_gpio_pullup(i, 1);

		set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
		error = request_irq(irq, lbookv3_keys_isr, IRQF_SAMPLE_RANDOM,
				    "lbookv3_keys", input);
		if (error) {
			printk(KERN_ERR "lbookv3-keys: unable to claim irq %d; error %d\n",
				irq, error);
			goto fail_reg_irqs;
		}
		enable_irq_wake(irq);
	}

	set_irq_type(IRQ_EINT6, IRQ_TYPE_EDGE_FALLING);
	error = request_irq(IRQ_EINT6, lbookv3_powerkey_isr, IRQF_SAMPLE_RANDOM,
			"lbookv3_keys", input);
	if (error) {
		printk(KERN_ERR "lbookv3-keys: unable to claim irq %d; error %d\n",
				IRQ_EINT6, error);
		goto fail_reg_eint6;
	}
	enable_irq_wake(IRQ_EINT6);

	s3c2410_gpio_cfgpin(S3C2410_GPF(6), S3C2410_GPF6_EINT6);
	s3c2410_gpio_pullup(S3C2410_GPF(6), 1);

	return 0;

	free_irq(IRQ_EINT6, input);
fail_reg_irqs:
fail_reg_eint6:
	for (i = i - 1; i >= S3C2410_GPF(0); i--)
		free_irq(s3c2410_gpio_getirq(i), input);

	device_remove_file(&input->dev, &dev_attr_poll_interval);
err_add_poll_interval:
	device_remove_file(&input->dev, &dev_attr_longpress_time);
err_add_longpress_time:

	return error;
}

static void __exit lbookv3_keys_exit(void)
{
	int i;

	disable_irq_wake(IRQ_EINT6);
	free_irq(IRQ_EINT6, input);

	for (i = S3C2410_GPF(0); i <= S3C2410_GPF(2); i++) {
		int irq = s3c2410_gpio_getirq(i);

		disable_irq_wake(irq);
		free_irq(irq, input);
	}

	del_timer_sync(&kb_timer);
	del_timer_sync(&power_timer);
	input_unregister_device(input);
}

module_init(lbookv3_keys_init);
module_exit(lbookv3_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eugene Konev <ejka@imfi.kspu.ru>");
MODULE_DESCRIPTION("Keyboard driver for Lbook V3 GPIOs");
