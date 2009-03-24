/* 
 * arch/mips/include/asm/mach-jz4740/jz4740-nand.h
 *
 * Copyright (c) 2009, Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * JZ4740 NAND driver platform data
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _ASM_MACH_JZ4740_JZ4740_NAND_H_
#define _ASM_MACH_JZ4740_JZ4740_NAND_H_

#include <linux/mtd/partitions.h>

struct jz4740_pdata {
	struct mtd_partition *partitions;
	unsigned int nr_partitions;
};

#endif /* _ASM_MACH_JZ4740_JZ4740_NAND_H_ */
