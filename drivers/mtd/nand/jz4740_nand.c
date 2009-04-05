/*
 * linux/drivers/mtd/nand/jz4740_nand.c
 *
 * Copyright (c) 2005 - 2007 Ingenic Semiconductor Inc.
 *
 * Ingenic JZ4740 NAND driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/jzsoc.h>
#include <asm/mach-jz4740/jz4740-nand.h>

#define NAND_DATA_PORT	       0xB8000000  /* read-write area */

#define PAR_SIZE 9

#define __nand_enable()	       (REG_EMC_NFCSR |= EMC_NFCSR_NFE1 | EMC_NFCSR_NFCE1)
#define __nand_disable()       (REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1)

#define __nand_ecc_enable()    (REG_EMC_NFECR = EMC_NFECR_ECCE | EMC_NFECR_ERST )
#define __nand_ecc_disable()   (REG_EMC_NFECR &= ~EMC_NFECR_ECCE)

#define __nand_select_hm_ecc() (REG_EMC_NFECR &= ~EMC_NFECR_RS )
#define __nand_select_rs_ecc() (REG_EMC_NFECR |= EMC_NFECR_RS)

#define __nand_read_hm_ecc()   (REG_EMC_NFECC & 0x00ffffff)

#define __nand_rs_ecc_encoding()	(REG_EMC_NFECR |= EMC_NFECR_RS_ENCODING)
#define __nand_rs_ecc_decoding()	(REG_EMC_NFECR &= ~EMC_NFECR_RS_ENCODING)
#define __nand_ecc_encode_sync() while (!(REG_EMC_NFINTS & EMC_NFINTS_ENCF))
#define __nand_ecc_decode_sync() while (!(REG_EMC_NFINTS & EMC_NFINTS_DECF))


#ifdef CONFIG_MTD_HW_RS_ECC
/* ECC layout compatible with bootloader */

static struct nand_ecclayout nand_oob_rs64 = {
	.eccbytes = 36,
	.eccpos = {
		6,  7,  8,  9,  10, 11, 12, 13,
		14, 15, 16, 17, 18, 19, 20, 21,
		22, 23, 24, 25, 26, 27, 28, 29,
		30, 31, 32, 33, 34, 35, 36, 37,
		38, 39, 40, 41},
	.oobfree = { {2, 4}, {42, 22} }
};


#endif


static void jz_hwcontrol(struct mtd_info *mtd, int dat,
			 unsigned int ctrl)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	unsigned int nandaddr = (unsigned int)this->IO_ADDR_W;

	if (ctrl & NAND_CTRL_CHANGE) {
		if ( ctrl & NAND_ALE )
			nandaddr = (unsigned int)((unsigned long)(this->IO_ADDR_W) | 0x00010000);
		else
			nandaddr = (unsigned int)((unsigned long)(this->IO_ADDR_W) & ~0x00010000);

		if ( ctrl & NAND_CLE )
			nandaddr = nandaddr | 0x00008000;
		else
			nandaddr = nandaddr & ~0x00008000;
		if ( ctrl & NAND_NCE )
			REG_EMC_NFCSR |= EMC_NFCSR_NFCE1;
		else
			REG_EMC_NFCSR &= ~EMC_NFCSR_NFCE1;
	}

	this->IO_ADDR_W = (void __iomem *)nandaddr;
	if (dat != NAND_CMD_NONE)
		writeb(dat, this->IO_ADDR_W);
}

static int jz_device_ready(struct mtd_info *mtd)
{
	int ready, wait = 10;
	while (wait--);
	ready = __gpio_get_pin(94);
	return ready;
}

/*
 * EMC setup
 */
static void jz_device_setup(void)
{
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE1;

	/* FIXME: Use real timings */
	/* Read/Write timings */
	REG_EMC_SMCR1 = 0x09221200;
}

#ifdef CONFIG_MTD_HW_HM_ECC

static int jzsoc_nand_calculate_hm_ecc(struct mtd_info* mtd,
				       const u_char* dat, u_char* ecc_code)
{
	unsigned int calc_ecc;
	unsigned char *tmp;

	__nand_ecc_disable();

	calc_ecc = ~(__nand_read_hm_ecc()) | 0x00030000;

	tmp = (unsigned char *)&calc_ecc;
	//adjust eccbytes order for compatible with software ecc
	ecc_code[0] = tmp[1];
	ecc_code[1] = tmp[0];
	ecc_code[2] = tmp[2];

	return 0;
}

static void jzsoc_nand_enable_hm_hwecc(struct mtd_info* mtd, int mode)
{
	__nand_ecc_enable();
	__nand_select_hm_ecc();
}

static int jzsoc_nand_hm_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	u_char a, b, c, d1, d2, d3, add, bit, i;

	/* Do error detection */
	d1 = calc_ecc[0] ^ read_ecc[0];
	d2 = calc_ecc[1] ^ read_ecc[1];
	d3 = calc_ecc[2] ^ read_ecc[2];

	if ((d1 | d2 | d3) == 0) {
		/* No errors */
		return 0;
	}
	else {
		a = (d1 ^ (d1 >> 1)) & 0x55;
		b = (d2 ^ (d2 >> 1)) & 0x55;
		c = (d3 ^ (d3 >> 1)) & 0x54;

		/* Found and will correct single bit error in the data */
		if ((a == 0x55) && (b == 0x55) && (c == 0x54)) {
			c = 0x80;
			add = 0;
			a = 0x80;
			for (i=0; i<4; i++) {
				if (d1 & c)
					add |= a;
				c >>= 2;
				a >>= 1;
			}
			c = 0x80;
			for (i=0; i<4; i++) {
				if (d2 & c)
					add |= a;
				c >>= 2;
				a >>= 1;
			}
			bit = 0;
			b = 0x04;
			c = 0x80;
			for (i=0; i<3; i++) {
				if (d3 & c)
					bit |= b;
				c >>= 2;
				b >>= 1;
			}
			b = 0x01;
			a = dat[add];
			a ^= (b << bit);
			dat[add] = a;
			return 1;
		}
		else {
			i = 0;
			while (d1) {
				if (d1 & 0x01)
					++i;
				d1 >>= 1;
			}
			while (d2) {
				if (d2 & 0x01)
					++i;
				d2 >>= 1;
			}
			while (d3) {
				if (d3 & 0x01)
					++i;
				d3 >>= 1;
			}
			if (i == 1) {
				/* ECC Code Error Correction */
				read_ecc[0] = calc_ecc[0];
				read_ecc[1] = calc_ecc[1];
				read_ecc[2] = calc_ecc[2];
				return 1;
			}
			else {
				/* Uncorrectable Error */
				printk("NAND: uncorrectable ECC error\n");
				return -1;
			}
		}
	}

	/* Should never happen */
	return -1;
}

#endif /* CONFIG_MTD_HW_HM_ECC */

#ifdef CONFIG_MTD_HW_RS_ECC

static void jzsoc_nand_enable_rs_hwecc(struct mtd_info* mtd, int mode)
{
	REG_EMC_NFINTS = 0x0;
	__nand_ecc_enable();
	__nand_select_rs_ecc();

	if (mode == NAND_ECC_READ)
		__nand_rs_ecc_decoding();

	if (mode == NAND_ECC_WRITE)
		__nand_rs_ecc_encoding();
}

static void jzsoc_rs_correct(unsigned char *dat, int idx, int mask)
{
	int i;

	idx--;

	i = idx + (idx >> 3);
	if (i >= 512)
		return;

	mask <<= (idx & 0x7);

	dat[i] ^= mask & 0xff;
	if (i < 511)
		dat[i+1] ^= (mask >> 8) & 0xff;
}

/*
 * calc_ecc points to oob_buf for us
 */
static int jzsoc_nand_rs_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	volatile u8 *paraddr = (volatile u8 *)EMC_NFPAR0;
	short k;
	u32 stat;

	/* Set PAR values */
	for (k = 0; k < PAR_SIZE; k++) {
		*paraddr++ = read_ecc[k];
	}

	/* Set PRDY */
	REG_EMC_NFECR |= EMC_NFECR_PRDY;

	/* Wait for completion */
	__nand_ecc_decode_sync();
	__nand_ecc_disable();

	/* Check decoding */
	stat = REG_EMC_NFINTS;

	if (stat & EMC_NFINTS_ERR) {
		/* Error occurred */
		if (stat & EMC_NFINTS_UNCOR) {
			printk("NAND: Uncorrectable ECC error\n");
			return -1;
		}
		else {
			u32 errcnt = (stat & EMC_NFINTS_ERRCNT_MASK) >> EMC_NFINTS_ERRCNT_BIT;
			switch (errcnt) {
			case 4:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR3 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR3 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				/* FALL-THROUGH */
			case 3:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR2 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR2 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				/* FALL-THROUGH */
			case 2:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR1 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR1 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				/* FALL-THROUGH */
			case 1:
				jzsoc_rs_correct(dat, (REG_EMC_NFERR0 & EMC_NFERR_INDEX_MASK) >> EMC_NFERR_INDEX_BIT, (REG_EMC_NFERR0 & EMC_NFERR_MASK_MASK) >> EMC_NFERR_MASK_BIT);
				return errcnt;
			default:
				break;
			}
		}
	}

	return 0;
}

static int jzsoc_nand_calculate_rs_ecc(struct mtd_info* mtd, const u_char* dat,
				u_char* ecc_code)
{
	volatile u8 *paraddr = (volatile u8 *)EMC_NFPAR0;
	short i;

	__nand_ecc_encode_sync();
	__nand_ecc_disable();

	for(i = 0; i < PAR_SIZE; i++) {
		ecc_code[i] = *paraddr++;
	}

	return 0;
}

/**
 * nand_read_page_hwecc_rs - hardware rs ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * Not for syndrome calculating ecc controllers which need a special oob layout
 */
static int nand_read_page_hwecc_rs(struct mtd_info *mtd, struct nand_chip *chip,
				   uint8_t *buf)
{
	int i, j;
	int eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	uint32_t page;
	uint8_t flag = 0;

	page = (buf[3]<<24) + (buf[2]<<16) + (buf[1]<<8) + buf[0];

	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	for (i = 0; i < chip->ecc.total; i++) {
		ecc_code[i] = chip->oob_poi[eccpos[i]];
//		if (ecc_code[i] != 0xff) flag = 1;
	}

	eccsteps = chip->ecc.steps;
	p = buf;

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0x00, -1);
	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		flag = 0;
		for (j = 0; j < eccbytes; j++)
			if (ecc_code[i + j] != 0xff) {
				flag = 1;
				break;
			}

		if (flag) {
			chip->ecc.hwctl(mtd, NAND_ECC_READ);
			chip->read_buf(mtd, p, eccsize);
			stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
			if (stat < 0)
				mtd->ecc_stats.failed++;
			else
				mtd->ecc_stats.corrected += stat;
		}
		else {
			chip->ecc.hwctl(mtd, NAND_ECC_READ);
			chip->read_buf(mtd, p, eccsize);
		}
	}
	return 0;
}

const uint8_t empty_ecc[] = {0xcd, 0x9d, 0x90, 0x58, 0xf4, 0x8b, 0xff, 0xb7, 0x6f};
static void nand_write_page_hwecc_rs(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, j, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int page_maybe_empty;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_WRITE);
		chip->write_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);

		page_maybe_empty = 1;
		for (j = 0; j < eccbytes; j++)
			if (ecc_calc[i + j] != empty_ecc[j]) {
				page_maybe_empty = 0;
				break;
			}

		if (page_maybe_empty)
			for (j = 0; j < eccsize; j++)
				if (p[j] != 0xff) {
					page_maybe_empty = 0;
					break;
				}
		if (page_maybe_empty)
			memset(&ecc_calc[i], 0xff, eccbytes);
	}

	for (i = 0; i < chip->ecc.total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}
#endif /* CONFIG_MTD_HW_RS_ECC */

/*
 * Main initialization routine
 */
static int __devinit jz4740_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct mtd_info *jz_mtd;
	struct jz4740_pdata *pdata = pdev->dev.platform_data;


	/* Allocate memory for MTD device structure and private data */
	jz_mtd = kzalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!jz_mtd) {
		printk ("Unable to allocate JzSOC NAND MTD device structure.\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, jz_mtd);

	/* Get pointer to private data */
	this = (struct nand_chip *) (&jz_mtd[1]);

	/* Link the private data with the MTD structure */
	jz_mtd->priv = this;
	jz_mtd->owner = THIS_MODULE;

	/* Set & initialize NAND Flash controller */
	jz_device_setup();

        /* Set address of NAND IO lines */
        this->IO_ADDR_R = (void __iomem *) NAND_DATA_PORT;
        this->IO_ADDR_W = (void __iomem *) NAND_DATA_PORT;
        this->cmd_ctrl = jz_hwcontrol;
        this->dev_ready = jz_device_ready;

#ifdef CONFIG_MTD_HW_HM_ECC
	this->ecc.calculate = jzsoc_nand_calculate_hm_ecc;
	this->ecc.correct   = jzsoc_nand_hm_correct_data;
	this->ecc.hwctl     = jzsoc_nand_enable_hm_hwecc;
	this->ecc.mode      = NAND_ECC_HW;
	this->ecc.size      = 256;
	this->ecc.bytes     = 3;

#endif

#ifdef CONFIG_MTD_HW_RS_ECC
	this->ecc.calculate	= jzsoc_nand_calculate_rs_ecc;
	this->ecc.correct	= jzsoc_nand_rs_correct_data;
	this->ecc.hwctl		= jzsoc_nand_enable_rs_hwecc;
	this->ecc.read_page	= nand_read_page_hwecc_rs;
	this->ecc.write_page	= nand_write_page_hwecc_rs;
	/* FIXME: Add suport of various oob sizes */
	this->ecc.layout	= &nand_oob_rs64;
	this->ecc.mode		= NAND_ECC_HW;
	this->ecc.size		= 512;
	this->ecc.bytes		= 9;
#endif

#ifdef  CONFIG_MTD_SW_HM_ECC
	this->ecc.mode      = NAND_ECC_SOFT;
#endif
        /* 20 us command delay time */
        this->chip_delay = 20;

	/* Scan to find existance of the device */
	if (nand_scan(jz_mtd, 1)) {
		kfree (jz_mtd);
		return -ENXIO;
	}

	add_mtd_partitions(jz_mtd, pdata->partitions, pdata->nr_partitions);

	return 0;
}

/*
 * Clean up routine
 */
static int __devexit jz4740_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *jz_mtd = platform_get_drvdata(pdev);

	nand_release(jz_mtd);

	platform_set_drvdata(pdev, NULL);

	/* Free the MTD device structure */
	kfree (jz_mtd);

	return 0;
}

#ifdef CONFIG_PM
static int jz4740_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	return 0;
}

static int jz4740_nand_resume(struct platform_device *dev)
{
	return 0;
}
#else
#define jz4740_nand_suspend NULL
#define jz4740_nand_resume NULL
#endif

static struct platform_driver jz4740_nand_driver = {
	.probe		= jz4740_nand_probe,
	.remove		= __devexit_p(jz4740_nand_remove),
	.suspend	= jz4740_nand_suspend,
	.resume		= jz4740_nand_resume,
	.driver		= {
		.name	= "jz4740-nand",
		.owner	= THIS_MODULE,
	},
};


static int __init jz4740_nand_init(void)
{
	return platform_driver_register(&jz4740_nand_driver);
}

static void __exit jz4740_nand_exit(void)
{
	platform_driver_unregister(&jz4740_nand_driver);
}

module_init(jz4740_nand_init);
module_exit(jz4740_nand_exit);
MODULE_AUTHOR("Yauhen Kharuzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL");
