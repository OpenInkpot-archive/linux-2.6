/*
 *  linux/include/asm-mips/mach-jz4740/board-n516.h
 *
 *  JZ4730-based N516 board definition.
 *
 *  Copyright (C) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_JZ4740_N516_H__
#define __ASM_JZ4740_N516_H__

#include <asm/mach-jz4740/gpio-pins.h>

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */


/*====================================================================== 
 * GPIO
 */
#define GPIO_SD_VCC_EN_N	JZ4740_GPD17
#define GPIO_SD_CD_N		JZ4740_GPD7
#define GPIO_SD_WP		JZ4740_GPD15
#define GPIO_USB_DETECT		JZ4740_GPD19
//#define GPIO_DC_DETE_N		103 /* GPD7 */
#define GPIO_CHARG_STAT_N	JZ4740_GPD16
#define GPIO_DISP_OFF_N		JZ4740_GPD1
#define GPIO_LED_EN       	JZ4740_GPD28
#define GPIO_LPC_INT		JZ4740_GPD14
#define GPIO_HPHONE_PLUG	JZ4740_GPD20
#define GPIO_SPK_SHUD		JZ4740_GPD21

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETECT

/* Display */
#define GPIO_DISPLAY_RST_L	JZ4740_GPB18
#define GPIO_DISPLAY_RDY	JZ4740_GPB17
#define GPIO_DISPLAY_STBY	JZ4740_GPC22
#define GPIO_DISPLAY_ERR	JZ4740_GPC23



/*====================================================================== 
 * MMC/SD
 */

#define MSC_WP_PIN		GPIO_SD_WP
#define MSC_HOTPLUG_PIN		GPIO_SD_CD_N
#define MSC_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD_CD_N)

#define __msc_init_io()				\
do {						\
	__gpio_as_output(GPIO_SD_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD_CD_N);		\
} while (0)

#define __msc_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD_CD_N))	\
		detected = 0;			\
	detected;				\
})

#endif /* __ASM_JZ4740_N516_H__ */
