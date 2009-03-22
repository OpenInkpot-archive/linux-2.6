
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

#include <linux/i2c.h>

#include <asm/jzsoc.h>


#define IRQ_LPC_INT	(IRQ_GPIO_0 + GPIO_LPC_INT)

struct n516_keys_chip {
	struct i2c_client	*i2c_client;
	struct work_struct	work;
	struct input_dev	*input; 
};

struct i2c_device_id n516_keys_i2c_ids[] = {
	{"LPC524", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, n516_keys_i2c_ids);

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
	{0x11, KEY_ENTER},
	{0x0e, KEY_FIND},
	{0x10, KEY_MENU},
//	{0x0f, KEY_},
	{0x0d, KEY_PLAYPAUSE},
	{0x1d, KEY_ESC},
	{0x1c, KEY_POWER},
//	{0x1e, KEY_PAGEDOWN},
//	{0x1f, KEY_PAGEDOWN},
};

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(n516_keys);


static void n516_keys_read_keys(struct work_struct *work)
{
	struct n516_keys_chip *chip = container_of(work, struct n516_keys_chip, work);
	unsigned char raw_msg;
	unsigned int key = KEY_RESERVED;
	struct i2c_client *client = chip->i2c_client;
	struct i2c_msg msg = {client->addr, client->flags | I2C_M_RD, 1, &raw_msg}; 
	int ret, i;


//	while (__gpio_get_pin(GPIO_LPC_INT)

	__gpio_as_input(GPIO_LPC_INT);
	do {
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
		}

	} while (raw_msg);
	__gpio_as_irq_fall_edge(GPIO_LPC_INT);
}

static irqreturn_t n516_keys_irq(int irq, void *dev_id)
{
	struct n516_keys_chip *chip = dev_id;

	printk("keys IRQ!\n");

	if (!work_pending(&chip->work))
		schedule_work(&chip->work);

	return IRQ_HANDLED;
}

static int n516_keys_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	return 0;
}

static int n516_keys_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct n516_keys_chip *chip;
	struct input_dev *input;
	int ret = 0;
	int i;

	printk("%s\n", __func__);

	chip = kmalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->i2c_client = client;
	INIT_WORK(&chip->work, n516_keys_read_keys);

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

	i2c_set_clientdata(client, chip);

	__gpio_as_irq_fall_edge(GPIO_LPC_INT);
	ret = request_irq(IRQ_LPC_INT, n516_keys_irq, 0, "lpc", chip);
	if (ret) {
		dev_err(&client->dev, "request_irq failed: %d\n", ret);
		goto err_request_irq;
	}


	printk("LPC INT = %d\n", __gpio_get_pin(GPIO_LPC_INT));

	return 0;

	free_irq(IRQ_LPC_INT, chip);
err_request_irq:
	input_unregister_device(input);
err_input_register:
	input_free_device(input);
err_input_alloc:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int n516_keys_remove(struct i2c_client *client)
{
	struct n516_keys_chip *chip = i2c_get_clientdata(client);

	__gpio_as_input(GPIO_LPC_INT);
	free_irq(IRQ_LPC_INT, chip);
	input_unregister_device(chip->input);
	kfree(chip);

	return 0;
}


static struct i2c_driver n516_keys_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name	= "n516-keys",
		.owner	= THIS_MODULE,
	},
	.probe		= n516_keys_probe,
	.remove		= __devexit_p(n516_keys_remove),
	.detect		= n516_keys_detect,
	.id_table	= n516_keys_i2c_ids,
	.address_data	= &addr_data,
};

static int n516_keys_init(void)
{
	return i2c_add_driver(&n516_keys_driver);
}

static void n516_keys_exit(void)
{
	i2c_del_driver(&n516_keys_driver);
}


module_init(n516_keys_init);
module_exit(n516_keys_exit);

MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Keys controller driver for N516");
MODULE_ALIAS("platform:n516-keys");

