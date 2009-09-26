/*
 * Driver for Netronix EB-600 keys
 *
 * Copyright 2009 Alexandr Tsidaev <a.tsidaev@gmail.com>
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

static unsigned long int key_pins[] = { 
	S3C2410_GPG2, S3C2410_GPG3, S3C2410_GPG0, S3C2410_GPG1, S3C2410_GPF7, 
	S3C2410_GPF6, S3C2410_GPG5, S3C2410_GPG4, S3C2410_GPG3,
	S3C2410_GPG4, S3C2410_GPG5 };
	
static unsigned long key_state[ARRAY_SIZE(key_pins)];	
	
static unsigned char key_codes[] = {
	KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN, KEY_ENTER,
	KEY_DELETE, KEY_ESC, KEY_MEDIA, KEY_MENU,
	KEY_VOLUMEUP, KEY_VOLUMEDOWN };

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
	
}

static irqreturn_t eb600_keys_isr(int irq, void *dev_id)
{
	printk("pressed %d\n", irq);
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
	for (i=0;i<ARRAY_SIZE(key_pins);i++)
		s3c2410_gpio_cfgpin(key_pins[i], S3C2410_GPIO_OUTPUT);
		
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
	
	for (i=0;i<ARRAY_SIZE(key_codes);i++)
		input_set_capability(input, EV_KEY, key_codes[i]);
		
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

	for (i = 0; i < ARRAY_SIZE(key_pins); i++) {
		int irq = s3c2410_gpio_getirq(key_pins[i]);

		s3c2410_gpio_cfgpin(key_pins[i], S3C2410_GPIO_SFN2);
		s3c2410_gpio_pullup(key_pins[i], 1);

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
		free_irq(s3c2410_gpio_getirq(key_pins[i]), input);
	device_remove_file(&input->dev, &dev_attr_poll_interval);
	
err_add_poll_interval:
	device_remove_file(&input->dev, &dev_attr_longpress_time);
err_add_longpress_time:

	return error;
}

static void __exit eb600_keys_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(key_pins); i++) {
		int irq = s3c2410_gpio_getirq(key_pins[i]);

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
