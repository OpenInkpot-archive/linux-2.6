/* arch/arm/mach-s5p100/include/mach/regs-sys.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PC100 - System registers definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define S5PC100_USB_PHY_CON	(S3C_VA_SYS + 0x8200)
#define S5PC100_USB_PHY0_EN	(1 << 16)

/* compatibility defines for s3c-hsotg driver */
#define S3C64XX_OTHERS		S5PC100_USB_PHY_CON
#define S3C64XX_OTHERS_USBMASK	S5PC100_USB_PHY0_EN
