/*
 * Driver for Netronix EB-600 keys
 *
 * Copyright 2009 Alexandr Tsidaev <a.tsidaev@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 * Based on lbookv3-keys by
 *  Eugene Konev <ejka@imfi.kspu.ru>
 *  Yauhen Kharuzhy <jekhor@gmail.com>
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
#include <linux/gpio.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>

#define KEYB_DELAY		(50 * HZ / 1000)
#define LONGPRESS_TIME		(HZ * 6 / 10) /* 0.6 seconds */

static unsigned long poll_interval = KEYB_DELAY;
static unsigned long longpress_time = LONGPRESS_TIME;

struct eb600_key_info
{
	unsigned long pin;
	unsigned int pin_config;
	unsigned char key_code;
};

static struct eb600_key_info eb600_keys[] = {
	{S3C2410_GPG(2), S3C2410_GPG2_EINT10, KEY_LEFT},
	{S3C2410_GPG(3), S3C2410_GPG3_EINT11, KEY_RIGHT},
	{S3C2410_GPG(0), S3C2410_GPG0_EINT8, KEY_UP},
	{S3C2410_GPG(1), S3C2410_GPG1_EINT9, KEY_DOWN},
	{S3C2410_GPF(7), S3C2410_GPF7_EINT7, KEY_ENTER},

	{S3C2410_GPF(6), S3C2410_GPF6_EINT6, KEY_DIRECTION},
	{S3C2410_GPF(5), S3C2410_GPF5_EINT5, KEY_ESC},
	{S3C2410_GPF(4), S3C2410_GPF4_EINT4, KEY_PLAYPAUSE},
	{S3C2410_GPF(3), S3C2410_GPF3_EINT3, KEY_MENU},

	{S3C2410_GPG(4), S3C2410_GPG4_EINT12, KEY_KPPLUS},
	{S3C2410_GPG(5), S3C2410_GPG5_EINT13, KEY_KPMINUS},

	{S3C2410_GPF(0), S3C2410_GPF0_EINT0, KEY_POWER},
};

	
static unsigned long key_state[ARRAY_SIZE(eb600_keys)];	

static struct timer_list kb_timer;	

static void generate_longpress_event(struct input_dev *input, unsigned char key)
{
	input_event(input, EV_KEY, KEY_LEFTALT, 1);
	input_event(input, EV_KEY, key, 1);
	input_event(input, EV_KEY, key, 0);
	input_event(input, EV_KEY, KEY_LEFTALT, 0);
	input_sync(input);
}

static void eb600_keys_kb_timer(unsigned long data)
{
	int i;
	int pressed = 0;
	struct input_dev *input = (struct input_dev *)data;

	for (i = 0; i < ARRAY_SIZE(eb600_keys); i++) {
		if (!gpio_get_value(eb600_keys[i].pin)) {
			if (!key_state[i]) {
				key_state[i] = jiffies + longpress_time;
			} else {
				if (time_after(jiffies, key_state[i]) &&
						(key_state[i] > 1)) {
					generate_longpress_event(input, eb600_keys[i].key_code);
					key_state[i] = 1;
				}
			}
			pressed = 1;
		} else {
			if (key_state[i]) {
				unsigned char key = eb600_keys[i].key_code;
				if (key_state[i] > 1) {
					if (time_after(jiffies, key_state[i])) {
						generate_longpress_event(input, key);
					} else {
						input_event(input, EV_KEY, key, 1);
						input_event(input, EV_KEY, key, 0);
						input_sync(input);
					}
				}
				key_state[i] = 0;
			}
		}
	}

	if(pressed && !timer_pending(&kb_timer)) {
		kb_timer.expires = jiffies + poll_interval;
		add_timer(&kb_timer);
	}
}

static irqreturn_t eb600_keys_isr(int irq, void *dev_id)
{
	eb600_keys_kb_timer((unsigned long)(dev_id));
	return IRQ_HANDLED;
}

static int eb600_keys_poll_interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", poll_interval * 1000 / HZ);
}

static int eb600_keys_poll_interval_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	poll_interval = simple_strtoul(buf, NULL, 10) * HZ / 1000;

	return size;
}

static int eb600_keys_longpress_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", longpress_time * 1000 / HZ);
}

static int eb600_keys_longpress_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	longpress_time = simple_strtoul(buf, NULL, 10) * HZ / 1000;

	return size;
}

DEVICE_ATTR(poll_interval, 0644, eb600_keys_poll_interval_show,
		eb600_keys_poll_interval_store);
DEVICE_ATTR(longpress_time, 0644, eb600_keys_longpress_time_show,
		eb600_keys_longpress_time_store);

static struct input_dev *input;
static int __init eb600_keys_init(void)
{
	int i, error;
	for (i=0;i<ARRAY_SIZE(eb600_keys);i++)
		s3c_gpio_cfgpin(eb600_keys[i].pin, S3C2410_GPIO_INPUT);

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input->evbit[0] = BIT(EV_KEY);

	input->name = "eb600-keys";
	input->phys = "eb600-keys/input0";

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	setup_timer(&kb_timer, eb600_keys_kb_timer, (unsigned long)input);

	for (i=0;i<ARRAY_SIZE(eb600_keys);i++)
		input_set_capability(input, EV_KEY, eb600_keys[i].key_code);

	input_set_capability(input, EV_KEY, KEY_LEFTALT);

	error = input_register_device(input);
	if (error) {
		printk(KERN_ERR "Unable to register eb600-keys input device\n");
		input_free_device(input);
		return -ENOMEM;
	}

	error = device_create_file(&input->dev, &dev_attr_poll_interval);
	if (error)
		goto err_add_poll_interval;

	error = device_create_file(&input->dev, &dev_attr_longpress_time);
	if (error)
		goto err_add_longpress_time;

	eb600_keys_isr(0, input);

	for (i = 0; i < ARRAY_SIZE(eb600_keys); i++) {
		int irq = gpio_to_irq(eb600_keys[i].pin);

		s3c_gpio_cfgpin(eb600_keys[i].pin, eb600_keys[i].pin_config);
		s3c_gpio_setpull(eb600_keys[i].pin, S3C_GPIO_PULL_NONE);

		error = request_irq(irq, eb600_keys_isr, IRQF_SAMPLE_RANDOM | 
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				    "eb600_keys", input);
		if (error) {
			printk(KERN_ERR "eb600-keys: unable to claim irq %d; error %d\n",
				irq, error);
			goto fail_reg_irqs;
		}
		enable_irq_wake(irq);
	}

	return 0;

fail_reg_irqs:
	for (i = i - 1; i >= 0; i--)
		free_irq(gpio_to_irq(eb600_keys[i].pin), input);
	device_remove_file(&input->dev, &dev_attr_poll_interval);

err_add_poll_interval:
	device_remove_file(&input->dev, &dev_attr_longpress_time);
err_add_longpress_time:

	return error;
}

static void __exit eb600_keys_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eb600_keys); i++) {
		int irq = gpio_to_irq(eb600_keys[i].pin);

		disable_irq_wake(irq);
		free_irq(irq, input);
	}

	del_timer_sync(&kb_timer);
	input_unregister_device(input);
}

module_init(eb600_keys_init);
module_exit(eb600_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexandr Tsidaev <a.tsidaev@gmail.com>");
MODULE_DESCRIPTION("Keyboard driver for Netronix EB-600 GPIOs");
