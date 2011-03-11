/************************************************************************
 * drivers/power/eb600_battery.c										*
 * Battery driver for Netronix EB-600									*
 *																		*
 * Author: Alexandr Tsidaev <a.tsidaev@gmail.com> 						*
 *																		*
 * Based on code for lbook v3 by										*
 *																		*
 * Authors: Piter Konstantinov <pit.here@gmail.com>						*
 *  																	*
 ************************************************************************/

 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <mach/regs-gpio.h>
#include <plat/regs-adc.h>
#include <mach/io.h>
#include <asm/mach/map.h>
#include <asm/io.h>

#define EB600_USBCK_PIN	S3C2410_GPF(2)
#define EB600_USBCK_IRQ	S3C2410_GPF2_EINT2

static void __iomem *adc_base;
static struct clk* adc_clk;

#define ADC_BATTERY_CH 0
#define EB600_MAX_VOLT 4120
#define EB600_MIN_VOLT 3562
#define EB600_5PERC_VOLT 3613
#define MAGIC_NUMBER 6317

static unsigned int adc_get_val (unsigned int ch)
{
	int wait = 0xffff;
	ch &= 0x07;
	ch <<= 3;
	__raw_writel(0x4c41 | ch, adc_base);
	while (((__raw_readl(adc_base) & 0x8000) == 0) && (--wait != 0));
	if (wait == 0)
		return 0;
	else
		return (__raw_readl(adc_base+0xc) & 0x3ff);
}

static int eb600_battery_get_voltage(struct power_supply *b)
{
	unsigned int adc_data;

	adc_data = adc_get_val(ADC_BATTERY_CH);
	if (adc_data == 0) {
		printk(KERN_DEBUG "eb600_battery: cannot get voltage -> ADC timeout\n");
		return 0;
	}

	return (adc_data * MAGIC_NUMBER) / 1000;
}

static int eb600_battery_get_capacity(struct power_supply *b)
{
	unsigned int voltage;

	voltage = eb600_battery_get_voltage(b);

	if (voltage < EB600_MIN_VOLT)
		voltage = EB600_MIN_VOLT;

	if (voltage > EB600_MAX_VOLT)
		voltage = EB600_MAX_VOLT;

	return ((voltage - EB600_MIN_VOLT) * 100 / (EB600_MAX_VOLT - EB600_MIN_VOLT));
}

static int eb600_usb_connected (void)
{
	return gpio_get_value(EB600_USBCK_PIN) ? 0 : 1;
}

static int eb600_battery_charging (void)
{
	return eb600_usb_connected();
}

static int eb600_battery_get_status(struct power_supply *b)
{
	// TODO: should check BUTT_FUNC bits of MISCCR
	if (eb600_battery_charging())
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_DISCHARGING;
}

static enum power_supply_property eb600_battery_props[] = 
{
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static int eb600_battery_get_property	(struct power_supply *b,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = EB600_MAX_VOLT;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = EB600_MIN_VOLT;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = eb600_battery_get_capacity(b);
		if (val->intval > 100)
			val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = eb600_battery_get_voltage(b);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = eb600_battery_get_status(b);
		break;
	default:
		break;
	}
	return 0;
}

static int eb600_usb_get_property (struct power_supply *b,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = eb600_usb_connected();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

void eb600_battery_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

struct power_supply eb600_battery =
{
	.name		= "eb600_battery",
	.get_property   = eb600_battery_get_property,
	.properties     = eb600_battery_props,
	.num_properties = ARRAY_SIZE(eb600_battery_props),
	.external_power_changed = eb600_battery_external_power_changed,
};

static char *eb600_power_supplied_to[] = {
	"eb600_battery",
};

static enum power_supply_property eb600_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct power_supply eb600_usb = {
	.name		= "usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.supplied_to	= eb600_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(eb600_power_supplied_to),
	.properties	= eb600_usb_props,
	.num_properties = ARRAY_SIZE(eb600_usb_props),
	.get_property	= eb600_usb_get_property,

};

static irqreturn_t eb600_usb_change_irq(int irq, void *dev)
{
	printk(KERN_DEBUG "USB change irq: usb cable has been %s\n",
			eb600_usb_connected() ? "connected" : "disconnected");

	power_supply_changed(&eb600_usb);

	return IRQ_HANDLED;
}

static int s3c2410_adc_init(void)
{
	int err = 0;
	adc_base = __arm_ioremap(0x58000000, 0x00100000, MT_DEVICE);
	adc_clk = clk_get(NULL, "adc");
	if(IS_ERR(adc_clk))
	{
		printk(KERN_ALERT "eb600_battery: adc clock get failed\n");
		goto err1;
	}

	err = clk_enable(adc_clk);
	if(err != 0)
	{
		printk(KERN_ALERT "eb600_battery: adc clock enable failed\n");
		goto err2;
	}

	__raw_writel(0x00, adc_base + 0x04);
	__raw_writel(0x00, adc_base + 0x08);
	return 0;
err2:
	clk_put(adc_clk);
err1:
	return err;

}

static int eb600_battery_probe(struct platform_device *dev)
{
	int ret;
	int irq;

	ret = s3c2410_adc_init();
	if(ret != 0)
		goto err1;

	s3c_gpio_cfgpin(EB600_USBCK_PIN, S3C2410_GPIO_INPUT);	//USB_pin
	s3c_gpio_cfgpin(EB600_USBCK_PIN, EB600_USBCK_IRQ);

	ret = power_supply_register(NULL, &eb600_battery);
	if(ret != 0)
	{
		printk(KERN_ERR "eb600_battery: could not register battery class\n");
		goto err2;
	}

	ret = power_supply_register(NULL, &eb600_usb);
	if(ret) {
		printk(KERN_ERR "eb600_battery: could not register USB power supply\n");
		goto err_reg_usb;
	}

	irq = gpio_to_irq(EB600_USBCK_PIN);
	ret = request_irq(irq, eb600_usb_change_irq,
			IRQF_DISABLED | IRQF_TRIGGER_RISING
			| IRQF_TRIGGER_FALLING | IRQF_SHARED,
			"usb power", &eb600_usb);

	if (ret) {
		printk(KERN_ERR "eb600_battery: could not request USB VBUS irq\n");
		goto err_usb_irq;
	}

	enable_irq_wake(irq);

	return ret;

	free_irq(gpio_to_irq(EB600_USBCK_PIN), &eb600_usb);
err_usb_irq:
	power_supply_unregister(&eb600_usb);
err_reg_usb:
	power_supply_unregister(&eb600_battery);
err2:
	clk_disable(adc_clk);
	clk_put(adc_clk);
err1:
	return 0;
}

static int eb600_battery_remove(struct platform_device *dev)
{
	disable_irq_wake(gpio_to_irq(EB600_USBCK_PIN));
	free_irq(gpio_to_irq(EB600_USBCK_PIN), &eb600_usb);

	power_supply_unregister(&eb600_usb);
	power_supply_unregister(&eb600_battery);
	clk_disable(adc_clk);
	clk_put(adc_clk);

	return 0;
}

#ifdef CONFIG_PM
static int eb600_battery_suspend(struct platform_device *pdev, pm_message_t message)
{
	__raw_writel(__raw_readl(adc_base + S3C2410_ADCCON) |
			S3C2410_ADCCON_STDBM, adc_base + S3C2410_ADCCON);
	return 0;
}

static int eb600_battery_resume(struct platform_device *pdev)
{
	__raw_writel(__raw_readl(adc_base + S3C2410_ADCCON) ^
			S3C2410_ADCCON_STDBM, adc_base + S3C2410_ADCCON);
	return 0;
}


#else
#define eb600_battery_suspend NULL
#define eb600_battery_resume NULL
#endif

static struct platform_driver eb600_battery_driver = {
	.driver = {
		.name = "eb600-battery",
	},
	.probe = eb600_battery_probe,
	.remove = eb600_battery_remove,
	.suspend = eb600_battery_suspend,
	.resume = eb600_battery_resume,
};

static int __init eb600_battery_init(void)
{
	return platform_driver_register(&eb600_battery_driver);
}

static void __exit eb600_battery_exit(void)
{
	platform_driver_unregister(&eb600_battery_driver);
}

module_init(eb600_battery_init);
module_exit(eb600_battery_exit);

/* Module information */
MODULE_AUTHOR("Alexandr Tsidaev <a.tsidaev@gmail.com>");
MODULE_AUTHOR("Piter Konstantinov <pit.here@gmail.com>");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_DESCRIPTION("Battery driver for eb-600");
MODULE_LICENSE("GPL");
