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

#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

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
	{S3C2410_GPG8, S3C2410_GPG8_EINT16, KEY_LEFT},
	{S3C2410_GPG3, S3C2410_GPG3_EINT11, KEY_RIGHT},
	{S3C2410_GPG0, S3C2410_GPG0_EINT8, KEY_UP},
	{S3C2410_GPG1, S3C2410_GPG1_EINT9, KEY_DOWN},
	{S3C2410_GPF7, S3C2410_GPF7_EINT7, KEY_ENTER},

	{S3C2410_GPF6, S3C2410_GPF6_EINT6, KEY_DIRECTION},
	{S3C2410_GPF5, S3C2410_GPF5_EINT5, KEY_ESC},
	{S3C2410_GPF4, S3C2410_GPF4_EINT4, KEY_PLAYPAUSE},
	{S3C2410_GPF3, S3C2410_GPF3_EINT3, KEY_MENU},

	{S3C2410_GPG9, S3C2410_GPG9_EINT17, KEY_KPPLUS},
	{S3C2410_GPG12, S3C2410_GPG12_EINT20, KEY_KPMINUS},


	{S3C2410_GPD13, S3C2410_GPIO_INPUT, KEY_5},
	{S3C2410_GPD14, S3C2410_GPIO_INPUT, KEY_6},
	{S3C2410_GPD15, S3C2410_GPIO_INPUT, KEY_9},

	{S3C2440_GPJ0, S3C2410_GPIO_INPUT, KEY_0},
	{S3C2440_GPJ1, S3C2410_GPIO_INPUT, KEY_1},
	{S3C2440_GPJ2, S3C2410_GPIO_INPUT, KEY_2},
	{S3C2440_GPJ3, S3C2410_GPIO_INPUT, KEY_3},
	{S3C2440_GPJ4, S3C2410_GPIO_INPUT, KEY_4},

	{S3C2440_GPJ7, S3C2410_GPIO_INPUT, KEY_7},
	{S3C2440_GPJ8, S3C2410_GPIO_INPUT, KEY_8},

	{S3C2440_GPJ10, S3C2410_GPIO_INPUT, KEY_PAGEUP},
	{S3C2440_GPJ11, S3C2410_GPIO_INPUT, KEY_KPENTER},
	{S3C2440_GPJ12, S3C2410_GPIO_INPUT, KEY_PAGEDOWN},
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
		if (!s3c2410_gpio_getpin(eb600_keys[i].pin)) {
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
	printk("EB600_KEYS_ISR\n");
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
		s3c2410_gpio_cfgpin(eb600_keys[i].pin, S3C2410_GPIO_INPUT);
		
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
		int irq;
		if(eb600_keys[i].pin_config == S3C2410_GPIO_INPUT) {
			s3c2410_gpio_cfgpin(eb600_keys[i].pin, S3C2410_GPIO_INPUT);
			s3c2410_gpio_pullup(eb600_keys[i].pin, 1);
			continue;
		}

		irq = s3c2410_gpio_getirq(eb600_keys[i].pin);

		s3c2410_gpio_cfgpin(eb600_keys[i].pin, eb600_keys[i].pin_config);
		s3c2410_gpio_pullup(eb600_keys[i].pin, 1);

		set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
		error = request_irq(irq, eb600_keys_isr, IRQF_SAMPLE_RANDOM,
				    "eb600_keys", input);
		if (error) {
			printk(KERN_ERR "eb600-keys: unable to claim irq %d; error %d\n",
				irq, error);
			goto fail_reg_irqs;
		}
		enable_irq_wake(irq);
	}

	{
		int irq = s3c2410_gpio_getirq(S3C2410_GPF1);

		s3c2410_gpio_cfgpin(S3C2410_GPF1, S3C2410_GPF1_EINT1);
		s3c2410_gpio_pullup(S3C2410_GPF1, 1);

		set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
		error = request_irq(irq, eb600_keys_isr, IRQF_SAMPLE_RANDOM,
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
		free_irq(s3c2410_gpio_getirq(eb600_keys[i].pin), input);
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
		int irq = s3c2410_gpio_getirq(eb600_keys[i].pin);

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
