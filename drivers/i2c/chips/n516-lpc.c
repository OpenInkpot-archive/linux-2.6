
#define DEBUG

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
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/suspend.h>

#include <linux/i2c.h>

#include <asm/jzsoc.h>


#define IRQ_LPC_INT	(IRQ_GPIO_0 + GPIO_LPC_INT)

static int batt_level=0;
module_param(batt_level, int, 0);

struct n516_lpc_chip {
	struct i2c_client	*i2c_client;
	struct work_struct	work;
	struct input_dev	*input;
	unsigned int		battery_level;
	unsigned int		suspending:1, can_sleep:1;
};

static struct n516_lpc_chip *the_lpc;

struct i2c_device_id n516_lpc_i2c_ids[] = {
	{"LPC524", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, n516_lpc_i2c_ids);

static const unsigned short normal_i2c[] = {0x54, I2C_CLIENT_END};

static const unsigned int keymap[][2] = {
	{0x05, KEY_0},
	{0x04, KEY_1},
	{0x03, KEY_2},
	{0x02, KEY_3},
	{0x01, KEY_4},
	{0x0b, KEY_5},
	{0x0a, KEY_6},
	{0x09, KEY_7},
	{0x08, KEY_8},
	{0x07, KEY_9},
	{0x1a, KEY_PAGEUP},
	{0x19, KEY_PAGEDOWN},
	{0x17, KEY_LEFT},
	{0x16, KEY_RIGHT},
	{0x14, KEY_UP},
	{0x15, KEY_DOWN},
	{0x13, KEY_ENTER},
	{0x11, KEY_SPACE},
	{0x0e, KEY_MENU},
	{0x10, KEY_DIRECTION},
	{0x0f, KEY_SEARCH},
	{0x0d, KEY_PLAYPAUSE},
	{0x1d, KEY_ESC},
	{0x1c, KEY_POWER},
	{0x1e, KEY_SLEEP},
	{0x1f, KEY_WAKEUP},
};

static const unsigned int batt_charge[] = {0, 7, 20, 45, 65, 80, 100};

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(n516_lpc);

static inline int n516_bat_usb_connected(void)
{
	return !__gpio_get_pin(GPIO_USB_DETECT);
}

static inline int n516_bat_charging(void)
{
	return !__gpio_get_pin(GPIO_CHARG_STAT_N);
}

static int n516_bat_get_status(struct power_supply *b)
{
	if (n516_bat_usb_connected()) {
		if (n516_bat_charging())
			return POWER_SUPPLY_STATUS_CHARGING;
		else
			return POWER_SUPPLY_STATUS_FULL;
	} else {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

static int n516_bat_get_charge(struct power_supply *b)
{
	return batt_charge[the_lpc->battery_level];
}

static int n516_bat_get_property(struct power_supply *b,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = n516_bat_get_status(b);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = n516_bat_get_charge(b);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void n516_bat_power_changed(struct power_supply *p)
{
	power_supply_changed(p);
}

static enum power_supply_property n516_bat_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static struct power_supply n516_battery = {
	.name		= "n516-battery",
	.get_property	= n516_bat_get_property,
	.properties	= n516_bat_properties,
	.num_properties	= ARRAY_SIZE(n516_bat_properties),
	.external_power_changed = n516_bat_power_changed,
};

static irqreturn_t n516_bat_charge_irq(int irq, void *dev)
{
	struct power_supply *psy = dev;

	dev_dbg(psy->dev, "Battery charging IRQ\n");

	power_supply_changed(psy);

	if (n516_bat_charging())
		__gpio_as_irq_rise_edge(GPIO_CHARG_STAT_N);
	else
		__gpio_as_irq_fall_edge(GPIO_CHARG_STAT_N);

	return IRQ_HANDLED;
}

static void n516_lpc_read_keys(struct work_struct *work)
{
	struct n516_lpc_chip *chip = container_of(work, struct n516_lpc_chip, work);
	unsigned char raw_msg;
	struct i2c_client *client = chip->i2c_client;
	struct i2c_msg msg = {client->addr, client->flags | I2C_M_RD, 1, &raw_msg}; 
	int ret, i;

	__gpio_as_input(GPIO_LPC_INT);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_dbg(&client->dev, "I2C error\n");
	} else {
		dev_dbg(&client->dev, "msg: 0x%02x\n", raw_msg);
		for (i = 0; i < ARRAY_SIZE(keymap); i++)
			if (raw_msg == keymap[i][0]) {
				input_report_key(chip->input, keymap[i][1], 1);
				input_sync(chip->input);
				input_report_key(chip->input, keymap[i][1], 0);
				input_sync(chip->input);
			}
		if ((raw_msg >= 0x81) && (raw_msg <= 0x87)) {
			unsigned int old_level = chip->battery_level;

			chip->battery_level = raw_msg - 0x81;
			if (old_level != chip->battery_level)
				power_supply_changed(&n516_battery);
		}
		if (the_lpc->suspending)
			the_lpc->can_sleep = 0;
	}
	__gpio_as_irq_fall_edge(GPIO_LPC_INT);
}

static irqreturn_t n516_lpc_irq(int irq, void *dev_id)
{
	struct n516_lpc_chip *chip = dev_id;
	struct device *dev = &chip->i2c_client->dev;

	if (dev->power.status != DPM_ON)
		return IRQ_HANDLED;

	if (!work_pending(&chip->work))
		schedule_work(&chip->work);

	return IRQ_HANDLED;
}

static int n516_lpc_set_normal_mode(struct n516_lpc_chip *chip)
{
	struct i2c_client *client = chip->i2c_client;
	unsigned char val = 0x02;
	struct i2c_msg msg = {client->addr, client->flags, 1, &val};
	int ret = 0;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret > 0 ? 0 : ret;
}

static void n516_lpc_power_off(void)
{
	struct i2c_client *client = the_lpc->i2c_client;
	unsigned char val = 0x01;
	struct i2c_msg msg = {client->addr, client->flags, 1, &val};

	__gpio_set_pin(GPIO_LED_EN);
	printk("Issue LPC POWEROFF command...\n");
	while (1)
		i2c_transfer(client->adapter, &msg, 1);
}

static int n516_lpc_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	return 0;
}


static int n516_lpc_suspend_notifier(struct notifier_block *nb,
		                                unsigned long event,
						void *dummy)
{
	switch(event) {
	case PM_SUSPEND_PREPARE:
		the_lpc->suspending = 1;
		the_lpc->can_sleep = 1;
		break;
	case PM_POST_SUSPEND:
		the_lpc->suspending = 0;
		the_lpc->can_sleep = 1;
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block n516_lpc_notif_block = {
	.notifier_call = n516_lpc_suspend_notifier,
};


static int n516_lpc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct n516_lpc_chip *chip;
	struct input_dev *input;
	int ret = 0;
	int i;

	printk("%s\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	the_lpc = chip;
	chip->i2c_client = client;
	if ((batt_level > 0) && (batt_level < ARRAY_SIZE(batt_charge)))
		chip->battery_level = batt_level;
	INIT_WORK(&chip->work, n516_lpc_read_keys);
	i2c_set_clientdata(client, chip);

	n516_lpc_set_normal_mode(chip);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "Unable to allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	chip->input = input;

	__set_bit(EV_KEY, input->evbit);

	for (i = 0; i < ARRAY_SIZE(keymap); i++)
		__set_bit(keymap[i][1], input->keybit);

	input->name = "n516-keys";
	input->phys = "n516-keys/input0";
	input->dev.parent = &client->dev;
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register input device\n");
		goto err_input_register;
	}

	ret = power_supply_register(NULL, &n516_battery);
	if (ret) {
		dev_err(&client->dev, "Unable to register N516 battery\n");
		goto err_bat_reg;
	}

	__gpio_as_irq_fall_edge(GPIO_LPC_INT);
	ret = request_irq(IRQ_LPC_INT, n516_lpc_irq, 0, "lpc", chip);
	if (ret) {
		dev_err(&client->dev, "request_irq failed: %d\n", ret);
		goto err_request_lpc_irq;
	}

	if (n516_bat_charging())
		__gpio_as_irq_rise_edge(GPIO_CHARG_STAT_N);
	else
		__gpio_as_irq_fall_edge(GPIO_CHARG_STAT_N);

	ret = request_irq(gpio_to_irq(GPIO_CHARG_STAT_N), n516_bat_charge_irq,
				IRQF_SHARED,
				"battery charging", &n516_battery);
	if (ret) {
		dev_err(&client->dev, "Unable to claim battery charging IRQ\n");
		goto err_request_chrg_irq;
	}

	pm_power_off = n516_lpc_power_off;
	ret = register_pm_notifier(&n516_lpc_notif_block);
	if (ret) {
		dev_err(&client->dev, "Unable to register PM notify block\n");
		goto err_reg_pm_notifier;
	}

	return 0;

	unregister_pm_notifier(&n516_lpc_notif_block);
err_reg_pm_notifier:
	free_irq(gpio_to_irq(GPIO_CHARG_STAT_N), &n516_battery);
err_request_chrg_irq:
	free_irq(IRQ_LPC_INT, chip);
err_request_lpc_irq:
	power_supply_unregister(&n516_battery);
err_bat_reg:
	input_unregister_device(input);
err_input_register:
	input_free_device(input);
err_input_alloc:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int n516_lpc_remove(struct i2c_client *client)
{
	struct n516_lpc_chip *chip = i2c_get_clientdata(client);

	unregister_pm_notifier(&n516_lpc_notif_block);
	pm_power_off = NULL;
	free_irq(gpio_to_irq(GPIO_CHARG_STAT_N), &n516_battery);
	__gpio_as_input(GPIO_LPC_INT);
	free_irq(IRQ_LPC_INT, chip);
	flush_scheduled_work();
	power_supply_unregister(&n516_battery);
	input_unregister_device(chip->input);
	kfree(chip);

	return 0;
}

#if CONFIG_PM
static int n516_lpc_suspend(struct i2c_client *client, pm_message_t msg)
{
	if (!the_lpc->can_sleep)
		return -EBUSY;

	disable_irq(IRQ_LPC_INT);
	return 0;
}

static int n516_lpc_resume(struct i2c_client *client)
{
	enable_irq(IRQ_LPC_INT);
	return 0;
}
#else
#define n516_lpc_suspend NULL
#define n516_lpc_resume NULL
#endif


static struct i2c_driver n516_lpc_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name	= "n516-keys",
		.owner	= THIS_MODULE,
	},
	.probe		= n516_lpc_probe,
	.remove		= __devexit_p(n516_lpc_remove),
	.detect		= n516_lpc_detect,
	.id_table	= n516_lpc_i2c_ids,
	.address_data	= &addr_data,
	.suspend	= n516_lpc_suspend,
	.resume		= n516_lpc_resume,
};

static int n516_lpc_init(void)
{
	return i2c_add_driver(&n516_lpc_driver);
}

static void n516_lpc_exit(void)
{
	i2c_del_driver(&n516_lpc_driver);
}


module_init(n516_lpc_init);
module_exit(n516_lpc_exit);

MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Keys and power controller driver for N516");
MODULE_ALIAS("platform:n516-keys");

