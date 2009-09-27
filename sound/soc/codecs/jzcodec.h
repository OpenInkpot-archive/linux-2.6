/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ICODEC_H
#define _ICODEC_H

/* jzcodec register space */
#define ICODEC_1_LOW   0x00  /* bit0 -- bit15 in CDCCR1 */
#define ICODEC_1_HIGH  0x01  /* bit16 -- bit31 in CDCCR1 */
#define ICODEC_2_LOW   0x02  /* bit0 -- bit16 in CDCCR2 */
#define ICODEC_2_HIGH  0x03  /* bit16 -- bit31 in CDCCR2 */

#define ICODEC_1_HIGH_ELININ	(1 << 13)
#define ICODEC_1_HIGH_EMIC	(1 << 12)
#define ICODEC_1_HIGH_SW1ON	(1 << 11)
#define ICODEC_1_HIGH_EADC	(1 << 10)
#define ICODEC_1_HIGH_SW2ON	(1 << 9)
#define ICODEC_1_HIGH_EDAC	(1 << 8)
#define ICODEC_1_HIGH_PDVR	(1 << 4)
#define ICODEC_1_HIGH_PDVRA	(1 << 3)
#define ICODEC_1_HIGH_VRPLD	(1 << 2)
#define ICODEC_1_HIGH_VRCGL	(1 << 1)
#define ICODEC_1_HIGH_VRCGH	(1 << 0)

#define ICODEC_1_LOW_HPMUTE	(1 << 14)
#define ICODEC_1_LOW_HPOV0	(1 << 13)
#define ICODEC_1_LOW_HPCG	(1 << 12)
#define ICODEC_1_LOW_HPPLDM	(1 << 11)
#define ICODEC_1_LOW_HPPLDR	(1 << 10)
#define ICODEC_1_LOW_PDHPM	(1 << 9)
#define ICODEC_1_LOW_PDHP	(1 << 8)
#define ICODEC_1_LOW_SUSP	(1 << 1)
#define ICODEC_1_LOW_RST	(1 << 0)

#define JZCODEC_CACHEREGNUM  4
#define JZCODEC_SYSCLK	0

extern struct snd_soc_dai jzcodec_dai;
extern struct snd_soc_codec_device soc_codec_dev_jzcodec;

#endif
