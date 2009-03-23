/* drivers/leds/leds-jz47xxxx.c
 *
 * (c) Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * JZ47XX - LEDs GPIO driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <asm/jzsoc.h>
#include <asm/jz47xx-leds.h>

/* our context */

struct jz47xx_gpio_led {
	struct led_classdev		 cdev;
	struct jz47xx_led_platdata	*pdata;
};

static inline struct jz47xx_gpio_led *pdev_to_gpio(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static inline struct jz47xx_gpio_led *to_gpio(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct jz47xx_gpio_led, cdev);
}

static void jz47xx_led_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct jz47xx_gpio_led *led = to_gpio(led_cdev);
	struct jz47xx_led_platdata *pd = led->pdata;

	/* there will be a short delay between setting the output and
	 * going from output to input when using tristate. */

	if ((value ? 1 : 0) ^ (pd->flags & JZ47XX_LEDF_ACTLOW))
		__gpio_set_pin(pd->gpio);
	else
		__gpio_clear_pin(pd->gpio);
}

static int jz47xx_led_remove(struct platform_device *dev)
{
	struct jz47xx_gpio_led *led = pdev_to_gpio(dev);

	led_classdev_unregister(&led->cdev);
	kfree(led);

	return 0;
}

static int jz47xx_led_probe(struct platform_device *dev)
{
	struct jz47xx_led_platdata *pdata = dev->dev.platform_data;
	struct jz47xx_gpio_led *led;
	int ret;

	led = kzalloc(sizeof(struct jz47xx_gpio_led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&dev->dev, "No memory for device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(dev, led);

	led->cdev.brightness_set = jz47xx_led_set;
	led->cdev.default_trigger = pdata->def_trigger;
	led->cdev.name = pdata->name;

	led->pdata = pdata;

	/* no point in having a pull-up if we are always driving */

	__gpio_as_output(pdata->gpio);
	__gpio_clear_pin(pdata->gpio);

	/* register our new led device */

	ret = led_classdev_register(&dev->dev, &led->cdev);
	if (ret < 0) {
		dev_err(&dev->dev, "led_classdev_register failed\n");
		goto exit_err1;
	}

	return 0;

 exit_err1:
	kfree(led);
	return ret;
}


#ifdef CONFIG_PM
static int jz47xx_led_suspend(struct platform_device *dev, pm_message_t state)
{
	struct jz47xx_gpio_led *led = pdev_to_gpio(dev);

	led_classdev_suspend(&led->cdev);
	return 0;
}

static int jz47xx_led_resume(struct platform_device *dev)
{
	struct jz47xx_gpio_led *led = pdev_to_gpio(dev);

	led_classdev_resume(&led->cdev);
	return 0;
}
#else
#define jz47xx_led_suspend NULL
#define jz47xx_led_resume NULL
#endif

static struct platform_driver jz47xx_led_driver = {
	.probe		= jz47xx_led_probe,
	.remove		= jz47xx_led_remove,
	.suspend	= jz47xx_led_suspend,
	.resume		= jz47xx_led_resume,
	.driver		= {
		.name		= "jz47xx_led",
		.owner		= THIS_MODULE,
	},
};

static int __init jz47xx_led_init(void)
{
	return platform_driver_register(&jz47xx_led_driver);
}

static void __exit jz47xx_led_exit(void)
{
	platform_driver_unregister(&jz47xx_led_driver);
}

module_init(jz47xx_led_init);
module_exit(jz47xx_led_exit);

MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_DESCRIPTION("JZ47XX LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz47xx_led");
