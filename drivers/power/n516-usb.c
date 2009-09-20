/*
 * drivers/power/n516-usb.c
 *
 * Copyright (C) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * USB power supply driver for N516
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/platform_device.h>

#include <asm/jzsoc.h>

/* in ms */
#define USB_STATUS_DELAY 50

static struct timer_list n516_usb_timer;

static inline int n516_usb_connected(void)
{
	return !__gpio_get_pin(GPIO_USB_DETECT);
}

static int n516_usb_pwr_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = n516_usb_connected();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void n516_usb_pwr_change_timer_func(unsigned long data)
{
	struct power_supply *psy = (struct power_supply *)data;

	if (!n516_usb_connected())
		__gpio_as_irq_fall_edge(GPIO_USB_DETECT);
	else
		__gpio_as_irq_rise_edge(GPIO_USB_DETECT);

	power_supply_changed(psy);
}

static irqreturn_t n516_usb_pwr_change_irq(int irq, void *dev)
{
	struct power_supply *psy = dev;

	dev_dbg(psy->dev, "USB change IRQ\n");

	mod_timer(&n516_usb_timer, jiffies + msecs_to_jiffies(USB_STATUS_DELAY));

	return IRQ_HANDLED;
}

static char* n516_usb_power_supplied_to[] = {
	"n516-battery",
};

static enum power_supply_property n516_usb_pwr_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply n516_usb_psy = {
	.name		= "usb",
	.type           = POWER_SUPPLY_TYPE_USB,
	.supplied_to    = n516_usb_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(n516_usb_power_supplied_to),
	.properties     = n516_usb_pwr_props,
	.num_properties = ARRAY_SIZE(n516_usb_pwr_props),
	.get_property   = n516_usb_pwr_get_property,

};

static int __devinit n516_usb_pwr_probe(struct platform_device *pdev)
{
	int ret;


	setup_timer(&n516_usb_timer, n516_usb_pwr_change_timer_func, (unsigned long)&n516_usb_psy);

	ret = power_supply_register(NULL, &n516_usb_psy);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register USB power supply\n");
		goto err_psy_reg;
	}

	if (!n516_usb_connected())
		__gpio_as_irq_fall_edge(GPIO_USB_DETECT);
	else
		__gpio_as_irq_rise_edge(GPIO_USB_DETECT);

	ret = request_irq(gpio_to_irq(GPIO_USB_DETECT),
			n516_usb_pwr_change_irq, IRQF_SHARED,
			"usb-power", &n516_usb_psy);
	if (ret) {
		dev_err(&pdev->dev, "Unable to claim IRQ\n");
		goto err_claim_irq;
	}


	return 0;

	power_supply_unregister(&n516_usb_psy);
err_psy_reg:
	free_irq(gpio_to_irq(GPIO_USB_DETECT), &n516_usb_psy);
err_claim_irq:

	return ret;
}

static int __devexit n516_usb_pwr_remove(struct platform_device *pdev)
{
	free_irq(gpio_to_irq(GPIO_USB_DETECT), &n516_usb_psy);
	del_timer_sync(&n516_usb_timer);
	power_supply_unregister(&n516_usb_psy);

	return 0;
}

static struct platform_driver n516_usb_power_driver = {
	.driver		= {
		.name	= "n516-usb-power",
		.owner	= THIS_MODULE,
	},
	.probe		= n516_usb_pwr_probe,
	.remove		= __devexit_p(n516_usb_pwr_remove),
};

static int __init n516_usb_pwr_init(void)
{
	return platform_driver_register(&n516_usb_power_driver);
}

static void __exit n516_usb_pwr_exit(void)
{
	platform_driver_unregister(&n516_usb_power_driver);
}

module_init(n516_usb_pwr_init);
module_exit(n516_usb_pwr_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_DESCRIPTION("USB power supply driver for N516");

