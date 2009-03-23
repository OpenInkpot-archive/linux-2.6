/* arch/mips/include/asm/jz47xx-leds.h
 *
 * Copyright (c) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * JZ47XX --- GPIO LED platform device data
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#ifndef __ASM_JZ47XX_LEDS_H
#define __ASM_JZ47XX_LEDS_H "leds-gpio.h"

#define JZ47XX_LEDF_ACTLOW     (1<<0)          /* LED is on when GPIO low */
#define JZ47XX_LEDF_TRISTATE   (1<<1)          /* tristate to turn off */

struct jz47xx_led_platdata {
	unsigned int             gpio;
	unsigned int             flags;

	char                    *name;
	char                    *def_trigger;
};

#endif /* __ASM_JZ47XX_LEDS_H */

