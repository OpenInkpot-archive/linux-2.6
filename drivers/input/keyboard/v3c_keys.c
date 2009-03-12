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
	S3C2410_GPC8, S3C2410_GPC9,
/*	S3C2410_GPC10, S3C2410_GPC11, */
};

static unsigned long int row_pins[] = {
	S3C2410_GPF0, S3C2410_GPF1, S3C2410_GPF2,
};

static unsigned long keypad_state[ARRAY_SIZE(row_pins)][ARRAY_SIZE(column_pins)];


static unsigned char keypad_codes[ARRAY_SIZE(row_pins)][ARRAY_SIZE(column_pins)] = {
	{ KEY_RIGHT,	KEY_DOWN },
	{ KEY_ENTER,	KEY_LEFT },
	{ KEY_UP,	KEY_RESERVED },
};

static struct timer_list kb_timer;

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
	input_sync(input);
	input_event(input, EV_KEY, key, 0);
	input_event(input, EV_KEY, KEY_LEFTALT, 0);
	input_sync(input);
}

static void v3c_keys_kb_timer(unsigned long data)
{
	int row, col;
	int pressed = 0;
	struct input_dev *input = (struct input_dev *)data;

	cfg_rows_to(S3C2410_GPIO_INPUT);
	for (col = 0; col < ARRAY_SIZE(column_pins); col++) {
		set_columns_to(1);
		if (!wait_for_rows_high())
			printk(KERN_ERR "%s: wait_for_rows_high() timeout\n", __func__);
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

static irqreturn_t v3c_powerkey_isr(int irq, void *dev_id)
{
	struct input_dev *input = dev_id;

	s3c2410_gpio_cfgpin(S3C2410_GPF7, S3C2410_GPF7_INP);

	if (s3c2410_gpio_getpin(S3C2410_GPF7))
		input_event(input, EV_KEY, KEY_POWER, 0);
	else
		input_event(input, EV_KEY, KEY_POWER, 1);

	input_sync(input);

	s3c2410_gpio_cfgpin(S3C2410_GPF7, S3C2410_GPF7_EINT7);

	return IRQ_HANDLED;
}


static irqreturn_t v3c_keys_isr(int irq, void *dev_id)
{
	cfg_rows_to(S3C2410_GPIO_INPUT);

	set_columns_to(1);

	v3c_keys_kb_timer((unsigned long)(dev_id));

	return IRQ_HANDLED;
}

static int v3c_keys_poll_interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", poll_interval * 1000 / HZ);
}

static int v3c_keys_poll_interval_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	poll_interval = simple_strtoul(buf, NULL, 10) * HZ / 1000;

	return size;
}

static int v3c_keys_longpress_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", longpress_time * 1000 / HZ);
}

static int v3c_keys_longpress_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	longpress_time = simple_strtoul(buf, NULL, 10) * HZ / 1000;

	return size;
}

DEVICE_ATTR(poll_interval, 0644, v3c_keys_poll_interval_show,
		v3c_keys_poll_interval_store);
DEVICE_ATTR(longpress_time, 0644, v3c_keys_longpress_time_show,
		v3c_keys_longpress_time_store);

static struct input_dev *input;
static int __init v3c_keys_init(void)
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

	input->name = "v3c-keys";
	input->phys = "v3c-keys/input0";

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	setup_timer(&kb_timer, v3c_keys_kb_timer, (unsigned long)input);
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
		printk(KERN_ERR "Unable to register v3c-keys input device\n");
		input_free_device(input);
		return -ENOMEM;
	}

	error = device_create_file(&input->dev, &dev_attr_poll_interval);
	if (error)
		goto err_add_poll_interval;

	error = device_create_file(&input->dev, &dev_attr_longpress_time);
	if (error)
		goto err_add_longpress_time;


	v3c_powerkey_isr(0, input);
	v3c_keys_isr(0, input);

	for (i = S3C2410_GPF0; i <= S3C2410_GPF2; i++) {
		int irq = s3c2410_gpio_getirq(i);

		s3c24xx_gpio_pullupdown(i, S3C2416_GPIO_PUP_ENABLE);
		s3c2410_gpio_cfgpin(i, S3C2410_GPIO_SFN2);

		error = request_irq(irq, v3c_keys_isr, IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				    "v3c_keys", input);
		if (error) {
			printk(KERN_ERR "v3c-keys: unable to claim irq %d; error %d\n",
				irq, error);
			goto fail_reg_irqs;
		}
	}

	/* FIXME: false powerkey keypress/keyrelease events */
	s3c24xx_gpio_pullupdown(S3C2410_GPF7, S3C2416_GPIO_PUP_ENABLE);
	s3c2410_gpio_cfgpin(S3C2410_GPF7, S3C2410_GPF7_EINT7);
	error = request_irq(IRQ_EINT7, v3c_powerkey_isr, IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"v3c_keys", input);
	if (error) {
		printk(KERN_ERR "v3c-keys: unable to claim irq %d; error %d\n",
				IRQ_EINT7, error);
		goto fail_reg_eint7;
	}

	return 0;

	free_irq(IRQ_EINT7, input);
fail_reg_irqs:
fail_reg_eint7:
	for (i = i - 1; i >= S3C2410_GPF0; i--)
		free_irq(s3c2410_gpio_getirq(i), input);

	device_remove_file(&input->dev, &dev_attr_poll_interval);
err_add_poll_interval:
	device_remove_file(&input->dev, &dev_attr_longpress_time);
err_add_longpress_time:

	return error;
}

static void __exit v3c_keys_exit(void)
{
	int i;

	free_irq(IRQ_EINT7, input);

	for (i = S3C2410_GPF0; i <= S3C2410_GPF2; i++) {
		int irq = s3c2410_gpio_getirq(i);

		free_irq(irq, input);
	}

	del_timer_sync(&kb_timer);
	input_unregister_device(input);

	input_free_device(input);
}
/*
#ifdef CONFIG_PM
static v3c_keys_resume_early(struct platform_device *pdev)
{
	struct input_dev *input = platform_get_drvdata(pdev);


}
#endif

static struct platform_driver v3c_keys_driver = {
	.probe = v3c_keys_probe,
	.remove = v3c_keys_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "v3c-keys",
	},
#ifdef CONFIG_PM
	.resume_early	= v3c_keys_resume_early,
#endif
};
*/
module_init(v3c_keys_init);
module_exit(v3c_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eugene Konev <ejka@imfi.kspu.ru>");
MODULE_DESCRIPTION("Keyboard driver for V3C GPIOs");
