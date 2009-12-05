#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>

#include <linux/spi/spi.h>

#include <mach/regs-gpio.h>
#include <mach/hardware.h>

#define IC2201_SOUND_PIN	S3C2410_GPC(11)
#define IC2201_RESET_PIN	S3C2410_GPC(10)
#define IC2201_CCS_PIN		S3C2410_GPD(8)
#define IC2201_DCS_PIN		S3C2410_GPD(9)

struct ic2201_data {
	struct spi_device	*spi_sci;
	struct spi_device	*spi_sdi;
};

enum ic2201_tests {
	IC2201_TEST_MEM,
	IC2201_TEST_SCI,
	IC2201_TEST_SINE,
	IC2201_TEST_SINE_STOP,
};

static inline void ic2201_setpin(int pin, int to)
{
	s3c2410_gpio_setpin(pin, to);
}

#define ic2201_disable() ic2201_setpin(IC2201_CCS_PIN, 1)
#define ic2201_enable() ic2201_setpin(IC2201_CCS_PIN, 0)


static int ic2201_write_register(struct ic2201_data *ic2201, unsigned char addr, unsigned short int val)
{
	unsigned char txbuf[4];

	txbuf[0] = 0x02;
	txbuf[1] = addr;
	txbuf[2] = (val >> 8) & 0xff;
	txbuf[3] = val & 0xff;

	return spi_write(ic2201->spi_sci, txbuf, 4);
}

static int ic2201_read_register(struct ic2201_data *ic2201, unsigned char addr, unsigned short int *res)
{
	int ret;
	unsigned char txbuf[4];
	unsigned char rxbuf[4];
	struct spi_transfer t = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = 4,
	};
	struct spi_message m;

	txbuf[0] = 0x03;
	txbuf[1] = addr;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ic2201_enable();
	ret = spi_sync(ic2201->spi_sci, &m);

	ic2201_disable();
	*res = (rxbuf[2] << 8) | rxbuf[3];

	return ret;
}

static void ic2201_init_ios(struct ic2201_data *ic2201)
{
	s3c2410_gpio_cfgpin(IC2201_SOUND_PIN, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(IC2201_SOUND_PIN, 0);

	s3c2410_gpio_cfgpin(S3C2410_GPB(8),S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPB(8),1);

	s3c2410_gpio_cfgpin(S3C2410_GPB(7),S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPB(7),1);

	mdelay(1);

	s3c2410_gpio_cfgpin(IC2201_RESET_PIN, S3C2410_GPIO_OUTPUT);
	ic2201_setpin(IC2201_RESET_PIN, 0);

	mdelay(1);

	s3c2410_gpio_cfgpin(IC2201_CCS_PIN, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(IC2201_DCS_PIN, S3C2410_GPIO_OUTPUT);

	s3c2410_gpio_cfgpin(S3C2410_GPG(3),S3C2410_GPIO_INPUT);

	s3c2410_gpio_cfgpin(S3C2410_GPE(11), S3C2410_GPE11_SPIMISO0);
	s3c2410_gpio_cfgpin(S3C2410_GPE(12), S3C2410_GPE12_SPIMOSI0);
	s3c2410_gpio_cfgpin(S3C2410_GPE(13), S3C2410_GPE13_SPICLK0);

	s3c2410_gpio_pullup(S3C2410_GPE(11), 0);
	s3c2410_gpio_pullup(S3C2410_GPE(12), 0);
	s3c2410_gpio_pullup(S3C2410_GPE(13), 1);

	ic2201_setpin(IC2201_DCS_PIN, 1);
	ic2201_setpin(IC2201_CCS_PIN, 0);

	ic2201_setpin(IC2201_RESET_PIN, 1);
	mdelay(2);
//	ic2201_setpin(IC2201_RESET_PIN, 0);
//	mdelay(5);
}

static inline void ic2201_reset(struct ic2201_data *ic2201)
{
	ic2201_setpin(IC2201_RESET_PIN, 0);
	mdelay(5);
	ic2201_setpin(IC2201_RESET_PIN, 1);
	mdelay(1);
}

static void ic2201_test(struct ic2201_data *ic2201, enum ic2201_tests test, unsigned char arg)
{
	unsigned char txbuf[8];

	memset(txbuf, 0, sizeof(txbuf));

	ic2201_reset(ic2201);

	switch (test) {
	case IC2201_TEST_MEM:
		txbuf[0] = 0x4d;
		txbuf[1] = 0xea;
		txbuf[2] = 0x6d;
		txbuf[3] = 0x54;
		break;
	case IC2201_TEST_SCI:
		txbuf[0] = 0x53;
		txbuf[1] = 0x70;
		txbuf[2] = 0xee;
		txbuf[3] = arg;
		break;
	case IC2201_TEST_SINE:
		txbuf[0] = 0x53;
		txbuf[1] = 0xef;
		txbuf[2] = 0x6e;
		txbuf[3] = arg;
		break;
	case IC2201_TEST_SINE_STOP:
		txbuf[0] = 0x45;
		txbuf[1] = 0x78;
		txbuf[2] = 0x69;
		txbuf[3] = 0x74;
		break;
	}

	spi_write(ic2201->spi_sdi, txbuf, 8);
}


static struct spi_board_info ic2201_sdi_board_info = {
	.modalias = "ic2201-sdi",
	.max_speed_hz	= 3000000,
	.bus_num	= 0,
	.chip_select	= 1,
	.platform_data	= 0,
};

static int ic2201_probe(struct spi_device *spi)
{
	struct ic2201_data	*ic2201;
	struct spi_device	*sdi;
	int ret = 0;
	int i;

	ic2201 = kzalloc(sizeof(*ic2201), GFP_KERNEL);
	if(!ic2201) {
		ret = -ENOMEM;
		goto err_ic2201_alloc;
	}

	ic2201->spi_sci = spi_dev_get(spi);

	ic2201_sdi_board_info.platform_data = ic2201;
	sdi = spi_new_device(spi->master, &ic2201_sdi_board_info);
	if (!sdi) {
		dev_err(&spi->dev, "Allocation of SPI device for SDI failed\n");
		goto err_sdi_alloc;
	}
	ic2201->spi_sdi = sdi;

	dev_set_drvdata(&spi->dev, ic2201);

	dev_info(&spi->dev, "IC2201 MP3 decoder\n");

	ic2201_init_ios(ic2201);

	ic2201_write_register(ic2201, 0, 0x0804);
	mdelay(1);
	ic2201_write_register(ic2201, 11, 0x1010);

	ic2201_test(ic2201, IC2201_TEST_SINE, 0xa8);
	for (i = 0; i < 15; i++) {
		unsigned short val = 0;

		if (!(ret = ic2201_read_register(ic2201, i, &val)))
			printk("ic2201: reg 0x%02x = 0x%04x\n", i, val);
		else
			printk("ic2201: SPI error: %d\n", ret);
	}

	return 0;

	spi_unregister_device(sdi);
err_sdi_alloc:
	kfree(ic2201);
err_ic2201_alloc:

	return ret;
}

static int __devexit ic2201_remove(struct spi_device *spi)
{
	struct ic2201_data	*ic2201;

	ic2201 = dev_get_drvdata(&spi->dev);
	spi_unregister_device(ic2201->spi_sdi);
	kfree(ic2201);

	return 0;
}

static struct spi_driver ic2201_driver = {
	.driver = {
		.name	= "ic2201",
		.owner	= THIS_MODULE,
	},
	.probe		= ic2201_probe,
	.remove		= __devexit_p(ic2201_remove),
};

static int ic2201_sdi_probe(struct spi_device *spi)
{
	dev_info(&spi->dev, "Initilized\n");
	return 0;
}

static int __devexit ic2201_sdi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ic2201_sdi_driver = {
	.driver = {
		.name	= "ic2201-sdi",
		.owner	= THIS_MODULE,
	},
	.probe		= ic2201_sdi_probe,
	.remove		= __devexit_p(ic2201_sdi_remove),
};

static int __init ic2201_init(void)
{
	int ret;

	ret = spi_register_driver(&ic2201_driver);
	if (ret)
		return ret;
	return spi_register_driver(&ic2201_sdi_driver);
}

static void __exit ic2201_exit(void)
{
	spi_unregister_driver(&ic2201_driver);
	spi_unregister_driver(&ic2201_sdi_driver);
}

module_init(ic2201_init);
module_exit(ic2201_exit);

MODULE_DESCRIPTION("IC2201 driver");
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL");
