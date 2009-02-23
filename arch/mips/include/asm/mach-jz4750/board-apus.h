/*
 *  linux/include/asm-mips/mach-jz4750/board-apus.h
 *
 *  JZ4750-based APUS board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750_APUS_H__
#define __ASM_JZ4750_APUS_H__

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */


/*====================================================================== 
 * GPIO
 */
#define GPIO_SD_VCC_EN_N	(32*2+10) /* GPC10 */
#define GPIO_SD_CD_N		(32*2+11) /* GPC11 */
#define GPIO_SD_WP		(32*2+12) /* GPC12 */
#define GPIO_SD1_VCC_EN_N	(32*2+13) /* GPC13 */
#define GPIO_SD1_CD_N		(32*2+14) /* GPC14 */
#define GPIO_USB_DETE		(32*2+11) /* GPC15 */
#define GPIO_DC_DETE_N		(32*2+8)  /* GPC8 */
#define GPIO_CHARG_STAT_N	(32*2+9)  /* GPC9 */
#define GPIO_DISP_OFF_N		(32*4+25) /* GPE25, PWM5 */
//#define GPIO_LED_EN       	124 /* GPD28 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE
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

#endif /* __ASM_JZ4750_APUS_H__ */
