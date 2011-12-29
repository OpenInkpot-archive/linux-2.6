/* linux/arch/arm/mach-s5pc100/mach-e60.c
 *
 * Copyright 2011 Alexander Kerner <lunohod@openinkpot.org>
 *
 * based on mach-smdkc100.c
 * Copyright 2009 Samsung Electronics Co.
 * Author: Byungho Min <bhmin@samsung.com>
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
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>

#include <video/platform_lcd.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>

#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/s5pc100.h>
#include <plat/iic.h>
#include <plat/ata.h>
#include <plat/adc.h>
#include <plat/keypad.h>
#include <plat/ts.h>
#include <plat/audio.h>
#include <plat/udc-hs.h>

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKC100_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKC100_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKC100_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S3C2440_UFCON_RXTRIG8 |	\
				 S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdkc100_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
};

/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
	{I2C_BOARD_INFO("wm8580", 0x1b),},
};

/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
};

static struct s3c_ide_platdata smdkc100_ide_pdata __initdata = {
	.setup_gpio	= s5pc100_ide_setup_gpio,
};

static uint32_t smdkc100_keymap[] /*__initdata*/ = {
	/* KEY(row, col, keycode) */
	KEY(2, 1, KEY_PAGEDOWN),
	KEY(2, 2, KEY_PAGEUP),
	KEY(1, 1, KEY_ENTER),
	KEY(1, 0, KEY_UP),
	KEY(1, 4, KEY_DOWN),
	KEY(1, 3, KEY_RIGHT),
	KEY(1, 2, KEY_LEFT),
	KEY(0, 3, KEY_MENU),
	KEY(2, 0, KEY_ESC),
	KEY(0, 4, KEY_SPACE),
	KEY(0, 2, KEY_DIRECTION),
	KEY(0, 0, KEY_KPPLUS),
	KEY(0, 1, KEY_KPMINUS),
};

static struct matrix_keymap_data smdkc100_keymap_data /*__initdata*/ = {
	.keymap		= smdkc100_keymap,
	.keymap_size	= ARRAY_SIZE(smdkc100_keymap),
};

static struct samsung_keypad_platdata smdkc100_keypad_data /*__initdata*/ = {
	.keymap_data	= &smdkc100_keymap_data,
	.rows		= 3,
	.cols		= 5,
};

static struct platform_device *smdkc100_devices[] __initdata = {
	&s3c_device_adc,
	&s3c_device_cfcon,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_ts,
	&s3c_device_wdt,
	&samsung_asoc_dma,
	&s5pc100_device_iis0,
	&samsung_device_keypad,
	&s5pc100_device_ac97,
	&s3c_device_rtc,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5pc100_device_spdif,
	&s3c_device_timer[0],
	&s3c_device_usb_hsotg,
};

static struct s3c2410_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
};

static void __init smdkc100_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdkc100_uartcfgs, ARRAY_SIZE(smdkc100_uartcfgs));
}

static struct s3c_hsotg_plat s3c_hsotg_pdata __initdata = {
    .is_osc = 1,
};

int __attribute__((weak)) e60_init(void)
{
	return 0;
}

static void __init smdkc100_machine_init(void)
{
	s3c24xx_ts_set_platdata(&s3c_ts_platform);

	/* I2C */
	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	s3c_ide_set_platdata(&smdkc100_ide_pdata);

	samsung_keypad_set_platdata(&smdkc100_keypad_data);

	s5pc100_spdif_setup_gpio(S5PC100_SPDIF_GPD);

	s3c_device_usb_hsotg.dev.platform_data = &s3c_hsotg_pdata;
	clk_xusbxti.rate = 12000000;

	platform_add_devices(smdkc100_devices, ARRAY_SIZE(smdkc100_devices));

	e60_init();
}

MACHINE_START(SAMSUNG_E60, "SAMSUNG_E60")
	/* Maintainer: Alexander Kerner <lunohod@openinkpot.org> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= s5pc100_init_irq,
	.map_io		= smdkc100_map_io,
	.init_machine	= smdkc100_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
