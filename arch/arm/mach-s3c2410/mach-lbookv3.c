/* linux/arch/arm/mach-s3c2410/mach-smdk2410.c
 *
 * linux/arch/arm/mach-s3c2410/mach-smdk2410.c
 *
 * Copyright (C) 2004 by FS Forth-Systeme GmbH
 * All rights reserved.
 *
 * $Id: mach-smdk2410.c,v 1.1 2004/05/11 14:15:38 mpietrek Exp $
 * @Author: Jonas Dietsche
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
 *
 * @History:
 * derived from linux/arch/arm/mach-s3c2410/mach-bast.c, written by
 * Ben Dooks <ben@simtec.co.uk>
 *
 ***********************************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/memblock.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/host.h>
#include <linux/eink_apollofb.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/delay.h>

#include <mach/regs-clock.h>
#include <mach/leds-gpio.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <mach/spi.h>

#include <plat/pll.h>
#include <plat/regs-serial.h>
#include <plat/nand.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/udc.h>
#include <plat/pm.h>
#include <plat/mci.h>

#include <plat/common-smdk.h>

static struct map_desc lbookv3_iodesc[] __initdata = {
	{0xe8000000, __phys_to_pfn(S3C2410_CS5), 0x100000, MT_DEVICE}
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg lbookv3_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

static struct s3c24xx_led_platdata lbookv3_pdata_led_red = {
	.gpio		= S3C2410_GPC(5),
	.flags		= 0,
	.name		= "led:red",
	.def_trigger	= "lbookv3_battery-charging",
};

static struct s3c24xx_led_platdata lbookv3_pdata_led_green = {
	.gpio		= S3C2410_GPC(6),
	.flags		= 0,
	.name		= "led:green",
	.def_trigger	= "nand-disk",
};

static struct platform_device lbookv3_led_red = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data = &lbookv3_pdata_led_red,
	},
};

static struct platform_device lbookv3_led_green = {
	.name		= "s3c24xx_led",
	.id		= 2,
	.dev		= {
		.platform_data = &lbookv3_pdata_led_green,
	},
};

#ifdef CONFIG_ARCH_LBOOK_V3_EXT

static struct mtd_partition lbookv3_nand_part[] = {
	[0] = {
		.name	= "KERNEL",
		.offset = SZ_1M * 8,
		.size	= SZ_1M,
	},
	[1] = {
		.name	= "ROOTFS",
		.offset = SZ_1M * 9,
		.size	= SZ_1M * 50,
	},
	[2] = {
		.name	= "LOGO",
		.offset = SZ_1M * 59,
		.size	= SZ_1M,
	},
	[3] = {
		.name	= "USERDATA",
		.offset	= SZ_1M * 0x3C,
		.size	= SZ_2M,
	},
	[4] = {
		.name	= "STORAGE",
		.offset	= SZ_1M * 0x3E,
		.size	= SZ_1M * 450,
	},
	[5] = {
		.name	= "SPARE",
		.size	= SZ_1M * 8,
		.offset	= 0,
	},
};

#else

static struct mtd_partition lbookv3_nand_part[] = {
	[0] = {
		.name	= "KERNEL",
		.offset = SZ_1M,
		.size	= SZ_1M,
	},
	[1] = {
		.name	= "ROOTFS",
		.offset = SZ_2M,
		.size	= SZ_1M*50,
	},
	[2] = {
		.name	= "LOGO",
		.offset = SZ_1M * 52,
		.size	= SZ_1M,
	},
	[3] = {
		.name	= "USERDATA",
		.offset	= SZ_1M * 0x35,
		.size	= SZ_2M,
	},
	[4] = {
		.name	= "STORAGE",
		.offset	= SZ_1M * 0x37,
		.size	= SZ_1M * 9,
	},
	[5] = {
		.name	= "SPARE",
		.size	= SZ_1M,
		.offset	= 0,
	},
};
#endif


static struct s3c2410_nand_set lbookv3_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(lbookv3_nand_part),
		.partitions	= lbookv3_nand_part
	},
};

/* choose a set of timings which should suit most 512Mbit
 * chips and beyond.
*/


static struct s3c2410_platform_nand lbookv3_nand_info = {
//#ifdef CONFIG_ARCH_LBOOK_V3_EXT
#if 0
	.tacls		= 12,
	.twrph0		= 12,
	.twrph1		= 10,
#else
	.tacls		= 30,
	.twrph0		= 60,
	.twrph1		= 30,
#endif
	.nr_sets	= ARRAY_SIZE(lbookv3_nand_sets),
	.sets		= lbookv3_nand_sets,
};

static struct resource lbookv3_nor_resource[] = {
	[0] = {
		.start = S3C2410_CS0,
		.end   = S3C2410_CS0 + SZ_2M - 1,
		.flags = IORESOURCE_MEM,
	}
};

static struct mtd_partition lbookv3_nor_part[] = {
	{
		.name		= "Bootloader",
		.size		= SZ_2M,
		.offset		= 0,
		.mask_flags	= MTD_WRITEABLE,
	}
};

static struct physmap_flash_data lbookv3_nor_flash_data = {
	.width		= 2,
	.parts		= lbookv3_nor_part,
	.nr_parts	= ARRAY_SIZE(lbookv3_nor_part),
};

static struct platform_device lbookv3_device_nor = {
	.name		= "physmap-flash",
	.id		= -1,
	.dev = {
		.platform_data = &lbookv3_nor_flash_data,
	},
	.num_resources	= ARRAY_SIZE(lbookv3_nor_resource),
	.resource	= lbookv3_nor_resource,
};

static void lbookv3_mmc_set_power(unsigned char power_mode, unsigned short vdd)
{
	switch(power_mode) {
		case MMC_POWER_UP:
		case MMC_POWER_ON:
			s3c_gpio_cfgpin(S3C2410_GPB(6), S3C2410_GPIO_OUTPUT);
			gpio_set_value(S3C2410_GPB(6), 1);
			break;
		case MMC_POWER_OFF:
		default:
			s3c_gpio_cfgpin(S3C2410_GPB(6), S3C2410_GPIO_OUTPUT);
			s3c_gpio_setpull(S3C2410_GPB(6), S3C_GPIO_PULL_NONE);
			gpio_set_value(S3C2410_GPB(6), 0);
	}
}

static struct s3c24xx_mci_pdata lbookv3_mmc_cfg = {
//	.gpio_wprotect	= S3C2410_GPH(8),
	.gpio_detect	= S3C2410_GPF(5),
	.set_power	= &lbookv3_mmc_set_power,
	.ocr_avail	= MMC_VDD_32_33,
	.use_dma	= 1,
};

static void lbookv3_udc_command(enum s3c2410_udc_cmd_e cmd)
{
	switch(cmd) {
		case S3C2410_UDC_P_DISABLE:
			gpio_set_value(S3C2410_GPC(12), 0);
			break;
		case S3C2410_UDC_P_ENABLE:
			gpio_set_value(S3C2410_GPC(12), 1);
			break;
		case S3C2410_UDC_P_RESET:
			break;
	}
}

static struct s3c2410_udc_mach_info lbookv3_udc_platform_data = {
	.udc_command	= lbookv3_udc_command,
	.vbus_pin	= S3C2410_GPF(4),
};

const unsigned int apollo_pins[] = {S3C2410_GPD(10), S3C2410_GPD(12),
	S3C2410_GPD(13), S3C2410_GPD(11), S3C2410_GPD(14), S3C2410_GPD(15)};

static int apollo_get_ctl_pin(unsigned int pin)
{
	return s3c2410_gpio_getpin(apollo_pins[pin]) ? 1 : 0;
}

static void apollo_set_gpa_14_15(int val)
{
	if (val != 1) {
		s3c_gpio_cfgpin(S3C2410_GPA(15), S3C2410_GPIO_OUTPUT);
		gpio_set_value(S3C2410_GPA(15), 0);
		s3c_gpio_cfgpin(S3C2410_GPA(14), 0);
		gpio_set_value(S3C2410_GPA(14), 1);

	} else {
		s3c_gpio_cfgpin(S3C2410_GPA(15), S3C2410_GPIO_OUTPUT);
		gpio_set_value(S3C2410_GPA(15), 1);
		s3c_gpio_cfgpin(S3C2410_GPA(14), S3C2410_GPIO_OUTPUT);
		gpio_set_value(S3C2410_GPA(14), 0);
	}
}

static void apollo_set_ctl_pin(unsigned int pin, unsigned char val)
{
	gpio_set_value(apollo_pins[pin], val);
}


static void apollo_write_value(unsigned char val)
{
	writeb(val, 0xE8000000);
}

static unsigned char apollo_read_value(void)
{
	unsigned char res;

	apollo_set_gpa_14_15(1);
	res = readb(0xE8000000);
	apollo_set_gpa_14_15(0);

	return res;
}

static int apollo_setuphw(void)
{
	apollo_set_gpa_14_15(0);
	s3c_gpio_cfgpin(S3C2410_GPD(10), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPD(13), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPD(12), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPD(11), S3C2410_GPIO_INPUT);
	s3c_gpio_cfgpin(S3C2410_GPD(14), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPD(15), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPE(1), S3C2410_GPIO_OUTPUT);

	gpio_set_value(S3C2410_GPD(10), 0);
	gpio_set_value(S3C2410_GPD(13), 1);
	gpio_set_value(S3C2410_GPD(12), 0);
	gpio_set_value(S3C2410_GPD(14), 0);
	gpio_set_value(S3C2410_GPD(15), 1);
	gpio_set_value(S3C2410_GPE(1), 0);

	s3c_gpio_cfgpin(S3C2410_GPD(2), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPD(2), 1);

	s3c_gpio_cfgpin(S3C2410_GPC(13), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPC(13), 1);

	gpio_set_value(S3C2410_GPD(15), 0);
	udelay(20);
	gpio_set_value(S3C2410_GPD(15), 1);
	udelay(20);

	apollo_set_ctl_pin(H_CD, 0);
	return 0;
}

static void apollo_initialize(void)
{
	apollo_set_ctl_pin(H_RW, 0);
	gpio_set_value(S3C2410_GPA(14), 1);
	gpio_set_value(S3C2410_GPA(15), 0);
	s3c_gpio_cfgpin(S3C2410_GPE(1), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPE(1), 0);
	s3c_gpio_cfgpin(S3C2410_GPC(13), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPC(13), 1);
}

static int apollo_init(void)
{
	apollo_setuphw();
	apollo_initialize();

	return 0;
}

static struct eink_apollofb_platdata lbookv3_apollofb_platdata = {
	.ops = {
		.set_ctl_pin	= apollo_set_ctl_pin,
		.get_ctl_pin	= apollo_get_ctl_pin,
		.read_value	= apollo_read_value,
		.write_value	= apollo_write_value,
		.initialize = apollo_init,
	},
};

static struct platform_device lbookv3_apollo = {
	.name		= "eink-apollo",
	.id		= -1,
	.dev		= {
		.platform_data = &lbookv3_apollofb_platdata,
	},
};

static struct platform_device lbookv3_keys = {
	.name		= "lbookv3-keys",
	.id		= -1,
};

static struct platform_device lbookv3_battery = {
	.name		= "lbookv3-battery",
	.id		= -1,
};

static struct platform_device lbookv3_speaker = {
	.name		= "lbookv3-speaker",
	.id		= -1,
};

static unsigned long lbookv3_spi0_cs_pins[2] = {S3C2410_GPD(8), S3C2410_GPD(9)};

static void lbookv3_spi0_set_cs(struct s3c2410_spi_info *spi, int cs, int pol)
{
	gpio_set_value(lbookv3_spi0_cs_pins[cs], pol);
}

static struct s3c2410_spi_info lbookv3_spi_info = {
	.num_cs		= 2,
	.set_cs		= lbookv3_spi0_set_cs,
};

static struct platform_device *lbookv3_devices[] __initdata = {
	&s3c_device_wdt,
	&s3c_device_iis,
	&s3c_device_usbgadget,
	&s3c_device_ohci,
	&s3c_device_nand,
	&s3c_device_rtc,
	&s3c_device_adc,
	&s3c_device_spi0,
	&s3c_device_sdi,
	&s3c_device_timer[0],
	&lbookv3_led_red,
	&lbookv3_led_green,
	&lbookv3_device_nor,
	&lbookv3_apollo,
	&lbookv3_keys,
	&lbookv3_battery,
	&lbookv3_speaker,
};

struct spi_board_info lbookv3_mp3_info = {
	.modalias	= "ic2201",
	.max_speed_hz	= 3000000,
	.bus_num	= 0,
	.chip_select	= 0,
	.platform_data	= 0,
};

static void lbookv3_power_off(void)
{
	/* Voodoo from original kernel */
	gpio_set_value(S3C2410_GPB(8), 0);
	udelay(1000);
	gpio_set_value(S3C2410_GPB(5), 0);
}

static long lbookv3_panic_blink(int state)
{
	gpio_set_value(S3C2410_GPC(5), 1);
	mdelay(200);
	gpio_set_value(S3C2410_GPC(5), 0);
	mdelay(200);

	return 400;
}

void dbg_led_on(void)
{
	gpio_set_value(S3C2410_GPC(6), 1);
}

static void __init lbookv3_init_gpio(void)
{
	/* LEDs */
	s3c_gpio_cfgpin(S3C2410_GPC(5), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPC(6), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPC(5), 0);
	gpio_set_value(S3C2410_GPC(6), 0);

	/* Voodoo from original kernel */
	s3c_gpio_cfgpin(S3C2410_GPB(0), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(1), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(5), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(6), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(7), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(8), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(9), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPC(12), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPC(13), S3C2410_GPIO_OUTPUT);

	gpio_set_value(S3C2410_GPB(0), 0);
	gpio_set_value(S3C2410_GPB(1), 0);
	gpio_set_value(S3C2410_GPB(5), 1);
	gpio_set_value(S3C2410_GPB(6), 1);
	gpio_set_value(S3C2410_GPB(7), 0);
	gpio_set_value(S3C2410_GPB(8), 1);
	gpio_set_value(S3C2410_GPB(9), 0);
	gpio_set_value(S3C2410_GPC(13), 0);
	gpio_set_value(S3C2410_GPC(12), 0);

	s3c_gpio_cfgpin(S3C2410_GPF(4), S3C2410_GPF4_EINT4);

	/* SIM card  interface */
/*	s3c_gpio_cfgpin(S3C2410_GPB(8), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPB(9), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPC(1), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPC(2), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPH(8), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPH(9), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPH(10), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPB(8), 1);
	gpio_set_value(S3C2410_GPB(9), 0);
	gpio_set_value(S3C2410_GPC(2), 0);
	gpio_set_value(S3C2410_GPC(1), 0);
	gpio_set_value(S3C2410_GPH(10), 0);
*/

	/* MP3 decoder interface */
/*	s3c_gpio_cfgpin(S3C2410_GPC(11), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPD(9), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPC(10), S3C2410_GPIO_OUTPUT);
	gpio_set_value(S3C2410_GPC(11), 0);
*/

	s3c_gpio_cfgpin(S3C2410_GPG(12), S3C2410_GPG12_XMON);
	s3c_gpio_cfgpin(S3C2410_GPG(13), S3C2410_GPG13_nXPON);
	s3c_gpio_cfgpin(S3C2410_GPG(14), S3C2410_GPG14_YMON);
	s3c_gpio_cfgpin(S3C2410_GPG(15), S3C2410_GPG15_nYPON);

	s3c_gpio_setpull(S3C2410_GPG(12), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(13), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S3C2410_GPG(14), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPG(15), S3C_GPIO_PULL_NONE);

	enable_irq_wake(gpio_to_irq(S3C2410_GPF(5)));
}

static void __init lbookv3_map_io(void)
{
	s3c24xx_init_io(lbookv3_iodesc, ARRAY_SIZE(lbookv3_iodesc));
	s3c24xx_init_clocks(0);
	s3c24xx_init_uarts(lbookv3_uartcfgs, ARRAY_SIZE(lbookv3_uartcfgs));
}

static void __init lbookv3_init(void)
{
	unsigned int tmp;

	lbookv3_init_gpio();


	tmp =	(0x78 << S3C24XX_PLLCON_MDIVSHIFT)
		| (0x02 << S3C24XX_PLLCON_PDIVSHIFT)
		| (0x03 << S3C24XX_PLLCON_SDIVSHIFT);
	writel(tmp, S3C2410_UPLLCON);

	s3c_device_nand.dev.platform_data = &lbookv3_nand_info;
	s3c_device_sdi.dev.platform_data = &lbookv3_mmc_cfg;
	s3c_device_usbgadget.dev.platform_data = &lbookv3_udc_platform_data;
	s3c_device_spi0.dev.platform_data = &lbookv3_spi_info;

	platform_add_devices(lbookv3_devices, ARRAY_SIZE(lbookv3_devices));

	spi_register_board_info(&lbookv3_mp3_info, 1);

	pm_power_off = &lbookv3_power_off;
	panic_blink = lbookv3_panic_blink;
	s3c_pm_init();
}

static void __init lbookv3_reserve(void)
{
	memblock_reserve(0x3026f000, 0x1400);
}

MACHINE_START(LBOOK_V3, "LBOOK_V3") /* @TODO: request a new identifier and switch
				    * to LBOOK_V3 */
	/* Maintainer: Yauhen Kharuzhy */
	.boot_params	= S3C2410_SDRAM_PA + 0x100,
	.map_io		= lbookv3_map_io,
	.reserve	= lbookv3_reserve,
	.init_irq	= s3c24xx_init_irq,
	.init_machine	= lbookv3_init,
	.timer		= &s3c24xx_timer,
MACHINE_END


