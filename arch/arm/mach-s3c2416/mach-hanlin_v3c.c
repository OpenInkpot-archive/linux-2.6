/* linux/arch/arm/mach-s3c2416/mach-hanlin_v3c.c
 *
 * Copyright (c) 2009 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
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
#include <linux/mtd/partitions.h>

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
#include <mach/leds-gpio.h>
#include <plat/iic.h>

#include <plat/s3c2416.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/nand.h>

#include <plat/common-smdk.h>

static struct map_desc v3c_iodesc[] __initdata = {
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

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg v3c_uartcfgs[] __initdata = {
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
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

static struct resource s3c_usb2gadget_resource[] = {
	[0] = {
		.start = S3C24XX_PA_USB2DEV,
		.end   = S3C24XX_PA_USB2DEV + S3C24XX_SZ_USB2DEV - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBD,
		.end   = IRQ_USBD,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_usb2gadget = {
	.name		  = "s3c24xx-usb2gadget",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_usb2gadget_resource),
	.resource	  = s3c_usb2gadget_resource,
};

static struct mtd_partition v3c_nand_part[] = {
	[0] = {
		.name		= "all",
		.offset		= 0,
		.size		= MTDPART_SIZ_FULL,
	},
};


static struct s3c2410_nand_set v3c_nand_sets[] = {
	[0] = {
		.name           = "flash",
		.nr_chips       = 1,
		.nr_partitions	= 0,
		.nr_partitions  = ARRAY_SIZE(v3c_nand_part),
		.partitions     = v3c_nand_part,
	},
};


static struct s3c2410_platform_nand v3c_nand_info = {
	/* FIXME: set real timings */
	.tacls		= 25,
	.twrph0		= 25,
	.twrph1		= 25,
	.sets		= v3c_nand_sets,
	.nr_sets	= ARRAY_SIZE(v3c_nand_sets),
};

static struct s3c24xx_led_platdata v3c_pdata_led1_red = {
	.gpio		= S3C2410_GPD1,
	.flags		= 0,
	.name		= "led1:red",
	.def_trigger	= "v3c_battery-charging",
};

static struct s3c24xx_led_platdata v3c_pdata_led1_green = {
	.gpio		= S3C2410_GPD2,
	.flags		= 0,
	.name		= "led1:green",
	.def_trigger	= "nand-disk",
};

static struct s3c24xx_led_platdata v3c_pdata_led2_red = {
	.gpio		= S3C2410_GPD3,
	.flags		= 0,
	.name		= "led2:red",
	.def_trigger	= "v3c_battery-charging",
};

static struct s3c24xx_led_platdata v3c_pdata_led2_green = {
	.gpio		= S3C2410_GPD4,
	.flags		= 0,
	.name		= "led2:green",
	.def_trigger	= "v3c_battery-charging",
};

static struct platform_device v3c_led1_red = {
	.name		= "s3c24xx_led",
	.id		= 0,
	.dev		= {
		.platform_data = &v3c_pdata_led1_red,
	},
};

static struct platform_device v3c_led1_green = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data = &v3c_pdata_led1_green,
	},
};

static struct platform_device v3c_led2_red = {
	.name		= "s3c24xx_led",
	.id		= 2,
	.dev		= {
		.platform_data = &v3c_pdata_led2_red,
	},
};

static struct platform_device v3c_led2_green = {
	.name		= "s3c24xx_led",
	.id		= 3,
	.dev		= {
		.platform_data = &v3c_pdata_led2_green,
	},
};


static struct platform_device *v3c_devices[] __initdata = {
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_hsmmc0,
	&s3c_device_usb2gadget,
	&s3c_device_nand,
	&v3c_led1_red,
	&v3c_led1_green,
	&v3c_led2_red,
	&v3c_led2_green,
};

extern void printascii(const char *);
static void __init v3c_map_io(void)
{

	s3c24xx_init_io(v3c_iodesc, ARRAY_SIZE(v3c_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(v3c_uartcfgs, ARRAY_SIZE(v3c_uartcfgs));

}

static void __init v3c_machine_init(void)
{
	printascii("v3c_machine_init\n");

	s3c_i2c0_set_platdata(NULL);
	s3c_device_nand.dev.platform_data = &v3c_nand_info;

	platform_add_devices(v3c_devices, ARRAY_SIZE(v3c_devices));
	printascii("v3c_machine_init end\n");
}

MACHINE_START(SMDK2416, "HANLIN V3C")
	/* Maintainer: Yauhen Kharuzhy <jekhor@gmail.com> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= v3c_map_io,
	.init_machine	= v3c_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
