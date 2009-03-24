/*
 * linux/arch/mips/jz4740/board-516.c
 *
 * JZ4740 n516 board setup routines.
 *
 * Copyright (c) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>
#include <asm/mach-jz4740/gpio-pins.h>
#include <asm/jz47xx-leds.h>
#include <asm/mach-jz4740/jz4740-nand.h>

/*
extern void (*jz_timer_callback)(void);

static void dancing(void)
{
	static unsigned int count = 0;

	count ++;
	count &= 1;
	if (count)
		__gpio_set_pin(GPIO_LED_EN);
	else
		__gpio_clear_pin(GPIO_LED_EN);
}

static void pavo_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}
*/

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4740/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Most of the GPIO pins should have been initialized by the boot-loader
	 */

	/*
	 * Initialize MSC pins
	 */
	__gpio_as_msc();

	/*
	 * Initialize LCD pins
	 */
	__gpio_as_lcd_18bit();

	/*
	 * Initialize SSI pins
	 */
	__gpio_as_ssi();

	/*
	 * Initialize I2C pins
	 */
	__gpio_as_i2c();

	/*
	 * Initialize Other pins
	 */
	__gpio_as_output(GPIO_SD_VCC_EN_N);
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);

	__gpio_as_input(GPIO_SD_CD_N);
	__gpio_disable_pull(GPIO_SD_CD_N);

	__gpio_as_input(GPIO_SD_WP);
	__gpio_disable_pull(GPIO_SD_WP);

//	__gpio_as_input(GPIO_DC_DETE_N);
	__gpio_as_input(GPIO_CHARG_STAT_N);
	__gpio_as_input(GPIO_USB_DETE);

	__gpio_as_output(GPIO_DISP_OFF_N);

	__gpio_as_output(GPIO_LED_EN);
}

void __init jz_board_setup(void)
{
	printk("JZ4740 N516 board setup\n");

	board_cpm_setup();
	board_gpio_setup();

//	jz_timer_callback = pavo_timer_callback;
}

static const struct i2c_board_info n516_keys_board_info = {
	.type		= "LPC524",
	.addr		= 0x54,
};

static const struct i2c_board_info n516_lm75a_board_info = {
	.type		= "lm75a",
	.addr		= 0x48,
};

static struct jz47xx_led_platdata n516_led_data = {
	.gpio		= GPIO_LED_EN,
	.name		= "led:blue",
	.def_trigger	= "nand-disk",
};

static struct platform_device n516_led = {
	.name		= "jz47xx_led",
	.id		= 0,
	.dev		= {
		.platform_data	= &n516_led_data,
	},
};

struct mtd_partition n516_nand_parts[] = {
	{
		.name	= "ALL",
		.size	= MTDPART_SIZ_FULL,
		.offset	= 0,
	},
};


static struct jz4740_pdata n516_nand_pdata = {
	.partitions	= n516_nand_parts,
	.nr_partitions	= ARRAY_SIZE(n516_nand_parts),
};

static struct platform_device n516_nand_dev = {
	.name		= "jz4740-nand",
	.id		= -1,
	.dev		= {
		.platform_data = &n516_nand_pdata,
	},
};

static int n516_setup_platform(void)
{
	i2c_register_board_info(0, &n516_keys_board_info, 1);
	i2c_register_board_info(0, &n516_lm75a_board_info, 1);

	platform_device_register(&n516_led);
	platform_device_register(&n516_nand_dev);

	return 0;
}
arch_initcall(n516_setup_platform);
