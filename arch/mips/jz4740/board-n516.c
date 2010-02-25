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
#include <linux/mtd/mtd.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/sizes.h>

#include <asm/jzsoc.h>
#include <asm/mach-jz4740/gpio-pins.h>
#include <asm/jz47xx-leds.h>
#include <asm/mach-jz4740/jz4740-nand.h>
#include <asm/mach-jz4740/pm.h>

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

/* 
 * __gpio_as_sleep set all pins to pull-disable, and set all pins as input
 * except sdram, nand flash pins and the pins which can be used as CS1_N  
 * to CS4_N for chip select. 
 */
#define __gpio_as_sleep()	              \
do {	                                      \
	REG_GPIO_PXFUNC(1) = ~0x9fffffff;     \
	REG_GPIO_PXSELC(1) = ~0x9fffffff;     \
	REG_GPIO_PXDIRC(1) = ~0x9fffffff;     \
	REG_GPIO_PXPES(1)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(2) =  0xc8ffffff;     \
	REG_GPIO_PXSELC(2) =  0xc8ffffff;     \
	REG_GPIO_PXDIRC(2) =  0xc8ffffff;     \
	REG_GPIO_PXPES(2)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(3) =  0xefc57f7f;     \
	REG_GPIO_PXSELC(3) =  0xefc57f7f;     \
	REG_GPIO_PXDIRC(3) =  0xefc57f7f;     \
	REG_GPIO_PXPES(3)  =  0xfbffffff;     \
	REG_GPIO_PXPEC(3)  =  0x04000000;     \
} while (0)

static long n516_panic_blink(long time)
{
	__gpio_set_pin(GPIO_LED_EN);
	mdelay(200);
	__gpio_clear_pin(GPIO_LED_EN);
	mdelay(200);

	return 400;
}

void n516_do_sleep(unsigned long *ptr)
{
	unsigned char i;

        /* Print messages of GPIO registers for debug */
	for(i=0;i<4;i++) {
		pr_debug("run dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n",        \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}

        /* Save GPIO registers */
	for(i = 1; i < 4; i++) {
		*ptr++ = REG_GPIO_PXFUN(i);
		*ptr++ = REG_GPIO_PXSEL(i);
		*ptr++ = REG_GPIO_PXDIR(i);
		*ptr++ = REG_GPIO_PXPE(i);
		*ptr++ = REG_GPIO_PXIM(i);
		*ptr++ = REG_GPIO_PXDAT(i);
		*ptr++ = REG_GPIO_PXTRG(i);
	}
	__gpio_set_pin(GPIO_LED_EN);
	mdelay(50);
	__gpio_clear_pin(GPIO_LED_EN);

	__gpio_as_sleep();
	__gpio_as_input(JZ4740_GPB27);
	__gpio_enable_pull(JZ4740_GPB28);
	__gpio_enable_pull(JZ4740_GPB29);
	__gpio_enable_pull(JZ4740_GPB30);
	__gpio_enable_pull(JZ4740_GPB31);
	__gpio_enable_pull(JZ4740_GPC27);
}

static void n516_do_resume(unsigned long *ptr)
{
	unsigned char i;

	__gpio_set_pin(GPIO_LED_EN);
	mdelay(50);

	/* Restore GPIO registers */
	for(i = 1; i < 4; i++) {
		 REG_GPIO_PXFUNS(i) = *ptr;
		 REG_GPIO_PXFUNC(i) = ~(*ptr++);

		 REG_GPIO_PXSELS(i) = *ptr;
		 REG_GPIO_PXSELC(i) = ~(*ptr++);

		 REG_GPIO_PXDIRS(i) = *ptr;
		 REG_GPIO_PXDIRC(i) = ~(*ptr++);

		 REG_GPIO_PXPES(i) = *ptr;
		 REG_GPIO_PXPEC(i) = ~(*ptr++);

		 REG_GPIO_PXIMS(i)=*ptr;
		 REG_GPIO_PXIMC(i)=~(*ptr++);
	
		 REG_GPIO_PXDATS(i)=*ptr;
		 REG_GPIO_PXDATC(i)=~(*ptr++);
	
		 REG_GPIO_PXTRGS(i)=*ptr;
		 REG_GPIO_PXTRGC(i)=~(*ptr++);
	}

}

void n516_setup_wakeup_ints(void)
{
	__gpio_as_irq_fall_edge(GPIO_LPC_INT);
	__gpio_unmask_irq(GPIO_LPC_INT);
	__gpio_unmask_irq(MSC_HOTPLUG_PIN);
	__gpio_unmask_irq(GPIO_USB_DETECT);
	__gpio_unmask_irq(GPIO_CHARG_STAT_N);
}

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4740/setup.c.
	 */

//	__cpm_stop_uart1();
	__cpm_stop_uhc();
	__cpm_stop_ipu();
	__cpm_stop_udc();
	__cpm_stop_cim();
	__cpm_stop_sadc();
//	__cpm_stop_aic1();
//	__cpm_stop_aic2();
	__cpm_stop_ssi();
//	__cpm_stop_tcu();
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
	__gpio_as_lcd_16bit();

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
	__gpio_as_input(GPIO_USB_DETECT);
	__gpio_as_input(GPIO_HPHONE_PLUG);

	__gpio_as_output(GPIO_DISP_OFF_N);
	__gpio_as_output(GPIO_SPK_SHUD);

	__gpio_as_output(GPIO_LED_EN);

	/* Disable display controller on booting */
	__gpio_as_output(GPIO_DISPLAY_STBY);
	__gpio_as_output(GPIO_DISPLAY_RST_L);
	__gpio_clear_pin(GPIO_DISPLAY_STBY);
	__gpio_clear_pin(GPIO_DISPLAY_RST_L);

	__gpio_enable_pull(JZ4740_GPD26);
}

void __init jz_board_setup(void)
{
	printk("JZ4740 N516 board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	panic_blink = n516_panic_blink;
	jz_board_do_sleep = n516_do_sleep;
	jz_board_setup_wakeup_ints = n516_setup_wakeup_ints;
	jz_board_do_resume = n516_do_resume;
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

static struct jz4740_pdata n516_nand_pdata = {
	.part_probe_types = (const char *[]) {"cmdlinepart", NULL}
};

static struct platform_device n516_nand_dev = {
	.name		= "jz4740-nand",
	.id		= -1,
	.dev		= {
		.platform_data = &n516_nand_pdata,
	},
};

static struct platform_device n516_rtc_dev = {
	.name		= "jz-rtc",
	.id		= -1,
};

static int n516_setup_platform(void)
{
	i2c_register_board_info(0, &n516_keys_board_info, 1);
	i2c_register_board_info(0, &n516_lm75a_board_info, 1);

	platform_device_register(&n516_led);
	platform_device_register(&n516_nand_dev);
	platform_device_register(&n516_rtc_dev);

	return 0;
}
arch_initcall(n516_setup_platform);
