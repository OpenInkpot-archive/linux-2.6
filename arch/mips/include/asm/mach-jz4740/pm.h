/*
 * arch/mips/include/mach-jz4740/pm.h
 *
 * JZ4740 power management
 *
 * Copyright (c) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _ASM_MACH_JZ4740_PM_H_
#define _ASM_MACH_JZ4740_PM_H_

extern void (*jz_board_do_sleep)(unsigned long *);
extern void (*jz_board_do_resume)(unsigned long *);
extern void (*jz_board_setup_wakeup_ints)(void);

#endif /* _ASM_MACH_JZ4740_PM_H_ */
