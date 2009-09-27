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

#include <mach/idle.h>
#include <mach/fb.h>
#include <plat/iic.h>

#include <linux/eink_apollofb.h>
#include <linux/delay.h>
#include <linux/spi/spi.h> 

#include <mach/leds-gpio.h>

#include <plat/s3c2410.h>
#include <plat/s3c2440.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/nand.h>
#include <plat/mci.h>

#include <plat/common-smdk.h>

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
	.gpio		= S3C2410_GPB8,
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


/* Apollo eInk driver info */

const unsigned int apollo_pins[] = {S3C2410_GPC10, S3C2410_GPC11,
	S3C2410_GPC9, S3C2410_GPC12, S3C2410_GPC8, S3C2410_GPC13, 
	S3C2410_GPC14,
	S3C2410_GPC0, S3C2410_GPC1, S3C2410_GPC2, S3C2410_GPC3,
	S3C2410_GPC4, S3C2410_GPC5, S3C2410_GPC6, S3C2410_GPC7, };

static int apollo_get_ctl_pin(unsigned int pin)
{
	return s3c2410_gpio_getpin(apollo_pins[pin]) ? 1 : 0;
}

static void apollo_set_ctl_pin(unsigned int pin, unsigned char val)
{
	s3c2410_gpio_setpin(apollo_pins[pin], val);
}

static void apollo_set_data_pins_as_output(void)
{
	s3c2410_gpio_cfgpin(S3C2410_GPC0, S3C2410_GPC0_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC1, S3C2410_GPC1_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC2, S3C2410_GPC2_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC3, S3C2410_GPC3_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC4, S3C2410_GPC4_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC5, S3C2410_GPC5_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC6, S3C2410_GPC6_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC7, S3C2410_GPC7_OUTP);
}

static void apollo_set_data_pins_as_input(void)
{
	s3c2410_gpio_cfgpin(S3C2410_GPC0, S3C2410_GPC0_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC1, S3C2410_GPC1_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC2, S3C2410_GPC2_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC3, S3C2410_GPC3_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC4, S3C2410_GPC4_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC5, S3C2410_GPC5_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC6, S3C2410_GPC6_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC7, S3C2410_GPC7_INP);
}

static void apollo_write_value(unsigned char val)
{
	unsigned char pin;
	unsigned char mask;
	
	mask = 0x1;
	for (pin=H_D0; pin<=H_D7; pin++)
	{
		apollo_set_ctl_pin(pin, (val & mask)==mask ? 1 : 0 );
		mask <<= 1;
	}
}

static unsigned char apollo_read_value(void)
{
	unsigned char res = 0;
	unsigned char mask;
	unsigned char pin;

	apollo_set_data_pins_as_input();

	mask = 0x1;
	for (pin=H_D0; pin<=H_D7; pin++)
	{
		if (apollo_get_ctl_pin(pin) == 1)
			res |= mask;
		mask <<= 1;
	}
	
	apollo_set_data_pins_as_output();

	return res;
}

static int apollo_setuphw(void)
{
	s3c2410_gpio_cfgpin(S3C2410_GPC10, S3C2410_GPC10_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC9,  S3C2410_GPC9_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC11, S3C2410_GPC11_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC12, S3C2410_GPC12_INP);
	s3c2410_gpio_cfgpin(S3C2410_GPC8,  S3C2410_GPC8_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC13, S3C2410_GPC13_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPC14, S3C2410_GPC14_OUTP);

	apollo_set_data_pins_as_output();

	apollo_set_ctl_pin(H_PWR, 1);
	udelay(30);

	apollo_set_ctl_pin(H_CD, 0);
	apollo_set_ctl_pin(H_DS, 1);
	apollo_set_ctl_pin(H_RW, 0);
	apollo_set_ctl_pin(H_WUP, 0);
	
	apollo_set_ctl_pin(H_NRST, 1);
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
		.initialize = apollo_init,
	},
	.defio_delay = HZ / 2,
};

static struct platform_device eb600_apollo = {
	.name		= "eink-apollo",
	.id		= -1,
	.dev		= {
		.platform_data = &eb600_apollofb_platdata,
	},
};


/* MMC driver info */

static struct s3c24xx_mci_pdata eb600_mmc_cfg = {
        .gpio_detect    = S3C2410_GPG7,
        .set_power      = NULL,
        .ocr_avail      = MMC_VDD_32_33,
};

static struct platform_device *eb600_devices[] __initdata = {
	&s3c_device_nand,
	&s3c_device_usb,
	&s3c_device_wdt,
	&s3c_device_sdi,
	&s3c_device_i2c0,
	&s3c_device_iis,
	&s3c_device_adc,
	&s3c_device_rtc,
	&eb600_led_green,
	&eb600_apollo,
	&eb600_keys,
};

static void __init eb600_map_io(void)
{
	s3c24xx_init_io(eb600_iodesc, ARRAY_SIZE(eb600_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(eb600_uartcfgs, ARRAY_SIZE(eb600_uartcfgs));
}

static void __init eb600_init_gpio(void)
{
	s3c2410_gpio_cfgpin(eb600_pdata_led_green.gpio, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(eb600_pdata_led_green.gpio, 1);
}

static void __init eb600_machine_init(void)
{
	eb600_init_gpio();
	
	s3c_i2c0_set_platdata(NULL);

	s3c_device_nand.dev.platform_data = &eb600_nand_info;
	s3c_device_sdi.dev.platform_data = &eb600_mmc_cfg;

	platform_add_devices(eb600_devices, ARRAY_SIZE(eb600_devices));
}

MACHINE_START(EB600, "Netronix EB-600")
	/* Maintainer: Alexandr Tsidaev <a.tsidaev@gmail.com> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= eb600_map_io,
	.init_machine	= eb600_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
