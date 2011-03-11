/* linux/arch/arm/mach-s3c2440/mach-eb600.c
 *
 * Copyright (c) 2009 
 * 	Alexandr Tsidaev <a.tsidaev@gmail.com>
 * 
 *  Based on linux/arch/arm/mach-s3c2440/mach-eb600.c
 *    by Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/host.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/regs-clock.h>

#include <mach/idle.h>
#include <mach/fb.h>
#include <plat/iic.h>

#include <linux/eink_apollofb.h>
#include <linux/delay.h>

#include <mach/leds-gpio.h>

#include <plat/s3c2410.h>
#include <plat/s3c244x.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/udc.h>
#include <plat/cpu.h>
#include <plat/nand.h>
#include <plat/mci.h>
#include <plat/pm.h>

#include <plat/common-smdk.h>

#include <sound/s3c24xx_uda134x.h>
#include <linux/clk.h>

#define EB600_GREEN_LED_PIN S3C2410_GPB(8)

static struct map_desc eb600_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

/* UART configuration */

static struct s3c2410_uartcfg eb600_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

/* LEDS */

static struct s3c24xx_led_platdata eb600_pdata_led_green = {
	.gpio		= EB600_GREEN_LED_PIN,
	.flags		= S3C24XX_LEDF_ACTLOW,
	.name		= "green",
	.def_trigger	= "nand-disk",
};

static struct platform_device eb600_led_green = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data = &eb600_pdata_led_green,
	},
};

/* NAND driver info */

static struct mtd_partition eb600_nand_part[] = {
	[0] = {
		.name	= "uboot",
		.offset = 0x00000000,
		.size	= 0x00040000,
	},
	[1] = {
		.name	= "kernel",
		.offset = 0x00040000,
		.size	= 0x00200000 - 0x00040000,
	},
	[2] = {
		.name	= "rootfs",
		.offset = 0x00200000,
		.size	= 0x01000000,
	},
	[3] = {
		.name	= "app",
		.offset	= 0x01200000,
		.size	= 0x01000000,
	},
	[4] = {
		.name	= "update",
		.offset	= 0x02200000,
		.size	= 0x01200000,
	},
	[5] = {
		.name	= "other",
		.offset	= 0x03400000,
		.size	= 0x07e00000 - 0x03400000,
	},
	[6] = {
		.name	= "startbmp",
		.offset	= 0x07e00000,
		.size	= 0x00100000,
	},
};

static struct s3c2410_nand_set eb600_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(eb600_nand_part),
		.partitions	= eb600_nand_part
	},
};

static struct s3c2410_platform_nand eb600_nand_info = {
	.tacls		= 15,
	.twrph0		= 45,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(eb600_nand_sets),
	.sets		= eb600_nand_sets,
};

/* Keyboard */

static struct platform_device eb600_keys = {
	.name		= "eb600-keys",
	.id		= -1,
};

/* Battery */

static struct platform_device eb600_battery = {
	.name		= "eb600-battery",
	.id		= -1,
};


/* Apollo eInk driver info */

const unsigned int apollo_pins[] = {S3C2410_GPC(10), S3C2410_GPC(11),
	S3C2410_GPC(9), S3C2410_GPC(12), S3C2410_GPC(8), S3C2410_GPC(13),
	S3C2410_GPC(14),
	S3C2410_GPC(0), S3C2410_GPC(1), S3C2410_GPC(2), S3C2410_GPC(3),
	S3C2410_GPC(4), S3C2410_GPC(5), S3C2410_GPC(6), S3C2410_GPC(7), };

static int apollo_get_ctl_pin(unsigned int pin)
{
	return gpio_get_value(apollo_pins[pin]) ? 1 : 0;
}

static void apollo_set_ctl_pin(unsigned int pin, unsigned char val)
{
	gpio_set_value(apollo_pins[pin], val);
}

static void apollo_set_data_pins_as_output(void)
{
	int i;
	for(i = 0; i <= 7; i++)
		gpio_direction_output(S3C2410_GPC(i), 0);
}

static void apollo_set_data_pins_as_input(void)
{
	int i;
	for(i = 0; i <= 7; i++)
		gpio_direction_input(S3C2410_GPC(i));
}

static void apollo_write_value(unsigned char val)
{
	unsigned long int gpc;
	unsigned long flags;

	local_irq_save(flags);

	gpc = __raw_readl(S3C2410_GPCDAT);
	gpc &= ~0xff;
	gpc |= val;
	__raw_writel(gpc, S3C2410_GPCDAT);

	local_irq_restore(flags);
}

static unsigned char apollo_read_value(void)
{
	unsigned char res = 0;
	unsigned long flags;

	apollo_set_data_pins_as_input();

	local_irq_save(flags);
	res = __raw_readl(S3C2410_GPCDAT);
	local_irq_restore(flags);

	apollo_set_data_pins_as_output();

	return res;
}

int apollo_send_data_fast(void *foo, unsigned char data)
{
	int res = 0;
	unsigned long int gpc;
	unsigned long flags;
	unsigned long timeout = jiffies + 2 * HZ;

	local_irq_save(flags);

	gpc = __raw_readl(S3C2410_GPCDAT);
	gpc &= ~0xff;
	gpc |= data;
	__raw_writel(gpc, S3C2410_GPCDAT);

	gpc &= ~0x0200;
	__raw_writel(gpc, S3C2410_GPCDAT);

	while (__raw_readl(S3C2410_GPCDAT) & 0x1000)
		if (time_before(jiffies, timeout)) {
		} else {
			printk(KERN_ERR "%s: Wait for H_ACK == %u, timeout\n",
					__func__, 0);
			res = 1;
		}

	gpc = __raw_readl(S3C2410_GPCDAT);
	gpc |= 0x0200;
	__raw_writel(gpc, S3C2410_GPCDAT);

	local_irq_restore(flags);

	return res;
}

static int apollo_setuphw(void)
{
	unsigned int i;
	for(i = 0; i < sizeof(apollo_pins) / sizeof(unsigned int); i++)
		gpio_request(apollo_pins[i], "apollo pin");

	apollo_set_data_pins_as_output();
	gpio_direction_input(apollo_pins[H_ACK]);
	gpio_direction_output(apollo_pins[H_PWR], 1);
	udelay(30);
	gpio_direction_output(apollo_pins[H_CD], 0);
	gpio_direction_output(apollo_pins[H_DS], 1);
	gpio_direction_output(apollo_pins[H_RW], 0);
	gpio_direction_output(apollo_pins[H_WUP], 0);
	gpio_direction_output(apollo_pins[H_NRST], 1);

	udelay(20);
	apollo_set_ctl_pin(H_NRST, 0);
	udelay(20);
	apollo_set_ctl_pin(H_NRST, 1);
	udelay(20);

	apollo_set_ctl_pin(H_CD, 0);
	return 0;
}

static void apollo_initialize(void)
{
	apollo_set_ctl_pin(H_RW, 0);
}

static int apollo_init(void)
{
	apollo_setuphw();
	apollo_initialize();

	return 0;
}

static struct eink_apollofb_platdata eb600_apollofb_platdata = {
	.ops = {
		.set_ctl_pin	= apollo_set_ctl_pin,
		.get_ctl_pin	= apollo_get_ctl_pin,
		.read_value	= apollo_read_value,
		.write_value	= apollo_write_value,
		.send_data_fast = apollo_send_data_fast,
		.initialize = apollo_init,
	},
	.defio_delay = HZ / 20,
};

static struct platform_device eb600_apollo = {
	.name		= "eink-apollo",
	.id		= -1,
	.dev		= {
		.platform_data = &eb600_apollofb_platdata,
	},
};

static void set_mmc_power(unsigned char power_mode, unsigned short vdd)
{
	switch (power_mode) {
		case MMC_POWER_OFF:
			gpio_set_value(S3C2410_GPB(6), 0);
			break;
		case MMC_POWER_UP:
		case MMC_POWER_ON:
			gpio_set_value(S3C2410_GPB(6), 1);
			break;
		default:
			break;
	}
}

/* MMC driver info */

static struct s3c24xx_mci_pdata eb600_mmc_cfg = {
	.gpio_detect    = S3C2410_GPG(7),
	.set_power      = set_mmc_power,
	.ocr_avail      = MMC_VDD_32_33,
};

/* Audio */

static struct s3c24xx_uda134x_platform_data eb600_audio_pins = {
	.l3_clk = S3C2410_GPB(4),
	.l3_mode = S3C2410_GPB(2),
	.l3_data = S3C2410_GPB(3),
	.model = UDA134X_UDA1345
};

static struct platform_device eb600_audio = {
	.name           = "s3c24xx_uda134x",
	.id             = 0,
	.dev            = {
		.platform_data  = &eb600_audio_pins,
	},
};

/* UDC */

static void eb600_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	s3c_gpio_cfgpin(S3C2410_GPG(6), S3C2410_GPIO_OUTPUT);
	switch(cmd) {
		case S3C2410_UDC_P_DISABLE:
			gpio_set_value(S3C2410_GPG(6), 0);
			break;
		case S3C2410_UDC_P_ENABLE:
			gpio_set_value(S3C2410_GPG(6), 1);
			break;
		case S3C2410_UDC_P_RESET:
			gpio_set_value(S3C2410_GPG(6), 0);
			udelay(50);
			gpio_set_value(S3C2410_GPG(6), 1);
			break;
	}
}

static struct s3c2410_udc_mach_info eb600_udc_platform_data = {
	.udc_command	= eb600_udc_command,
};

static struct platform_device *eb600_devices[] __initdata = {
	//&s3c_device_nand,
	&s3c_device_usbgadget,
	&s3c_device_wdt,
	&s3c_device_sdi,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&s3c_device_rtc,
	&eb600_led_green,
	&eb600_apollo,
	&eb600_keys,
	&eb600_battery,
	&eb600_audio,
};

static void eb600_power_off(void)
{
	int pin_state = 1;
	while (pin_state)
	{
		s3c_gpio_cfgpin(S3C2410_GPB(5), S3C2410_GPIO_OUTPUT);
		gpio_set_value(S3C2410_GPB(5), 0);
		pin_state = gpio_get_value(S3C2410_GPB(5));
	}
}

static long eb600_panic_blink(int state)
{
	gpio_set_value(EB600_GREEN_LED_PIN, 1);
	mdelay(200);
	gpio_set_value(EB600_GREEN_LED_PIN, 0);
	mdelay(200);

	return 400;
}

static void __init eb600_map_io(void)
{
	s3c24xx_init_io(eb600_iodesc, ARRAY_SIZE(eb600_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(eb600_uartcfgs, ARRAY_SIZE(eb600_uartcfgs));
}

static void __init eb600_init_gpio(void)
{
	int i;

	// Green led
	s3c_gpio_cfgpin(eb600_pdata_led_green.gpio, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(eb600_pdata_led_green.gpio, 1);
	
	//USB:
	// Switching FSUSB20L to s3c onboard usb:
	s3c_gpio_cfgpin(S3C2410_GPH(8), S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPH(8), 1);
	// Pulling up DP
	eb600_udc_command(S3C2410_UDC_P_ENABLE);

	// set GPF1 to 0
	s3c_gpio_cfgpin(S3C2410_GPF(1), S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(S3C2410_GPF(1), 1);

	__raw_writel(0xfff, S3C2410_GPBUP);
	__raw_writel(0x155555, S3C2410_GPBCON);
	__raw_writel(0x1a0, S3C2410_GPBDAT);

	__raw_writel(0xEFFF, S3C2410_GPCUP);

	__raw_writel(0xFFFF, S3C2410_GPDUP);
	__raw_writel(0xD5555555, S3C2410_GPDCON);
	__raw_writel(0x0, S3C2410_GPDDAT);

	for(i = 0; i < 13; i++) {
		s3c_gpio_cfgpin(S3C2410_GPJ(i), S3C2410_GPIO_OUTPUT);
		s3c_gpio_setpull(S3C2410_GPJ(i), S3C_GPIO_PULL_NONE);
	}
}

static void __init eb600_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);

	//s3c_device_nand.dev.platform_data = &eb600_nand_info;
	s3c_device_sdi.dev.platform_data = &eb600_mmc_cfg;
	s3c_device_usbgadget.dev.platform_data = &eb600_udc_platform_data;

	eb600_init_gpio();
	
	platform_add_devices(eb600_devices, ARRAY_SIZE(eb600_devices));
	
	pm_power_off = &eb600_power_off;
	panic_blink = eb600_panic_blink;
	s3c_pm_init();

	clk_disable(clk_get(NULL, "usb-host"));
	clk_disable(clk_get(NULL, "lcd"));

	/* doesn't work
	   clk_disable(clk_get(NULL, "camif"));
	   clk_disable(clk_get(NULL, "ac97"));
	*/

	// disable unneeded clocks manually
	__raw_writel(0x00E780, S3C2410_CLKCON);
}

MACHINE_START(EB600, "Netronix EB-600")
	/* Maintainer: Alexandr Tsidaev <a.tsidaev@gmail.com> */
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= eb600_map_io,
	.init_machine	= eb600_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
