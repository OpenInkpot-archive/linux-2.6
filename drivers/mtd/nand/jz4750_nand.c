/*
 * linux/drivers/mtd/nand/jz4750_nand.c
 * 
 * JZ4750 NAND driver
 *
 * Copyright (c) 2005 - 2007 Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
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

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/jzsoc.h>

#define DEBUG1        0
#if DEBUG1
#define	dprintk(n,x...) printk(n,##x)
#else
#define	dprintk(n,x...)
#endif

#if defined(CONFIG_MTD_NAND_DMA)
#define USE_DMABUF    0  /* 1: use dma buffers for nand reading and writing in this file 
                               instead of upper buffer, just for testing
			    0: use upper buffer for dma, it's faster */

#define  USE_IRQ      1
enum {
	NAND_NONE,
	NAND_PROG,
	NAND_READ
};

static u32 addrport;
static u32 cmdport;

static volatile u8 nand_status;
static volatile int dma_ack = 0;
static volatile int dma_ack1 = 0;
static char nand_dma_chan;	/* automatically select a free channel */
static char bch_dma_chan = 0;	/* fixed to channel 0 */

static u32 *errs;
static jz_dma_desc_8word *dma_desc_enc, *dma_desc_enc1, *dma_desc_dec, *dma_desc_dec1, *dma_desc_dec2,
	*dma_desc_nand_prog, *dma_desc_nand_read;

DECLARE_WAIT_QUEUE_HEAD(nand_prog_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(nand_read_wait_queue);
#endif

#if defined(CONFIG_MTD_HW_BCH_8BIT)
#define __ECC_ENCODING __ecc_encoding_8bit
#define __ECC_DECODING __ecc_decoding_8bit
#define ERRS_SIZE       5	/* 5 words */
#else
#define __ECC_ENCODING __ecc_encoding_4bit
#define __ECC_DECODING __ecc_decoding_4bit
#define ERRS_SIZE       3	/* 3 words */
#endif

#define NAND_DATA_PORT	       0xB8000000	/* read-write area */

/*
 * MTD structure for JzSOC board
 */
static struct mtd_info *jz_mtd = NULL;

/* it indicates share mode between nand and SDRAM Bus */
static int share_mode = 1;

/* 
 * Define partitions for flash devices
 */
#ifdef CONFIG_JZ4750_FUWA
static struct mtd_partition partition_info[] = {
      {name:"NAND BOOT partition",
	      offset:0 * 0x100000,
      size:4 * 0x100000},
      {name:"NAND KERNEL partition",
	      offset:4 * 0x100000,
      size:4 * 0x100000},
      {name:"NAND ROOTFS partition",
	      offset:8 * 0x100000,
      size:120 * 0x100000},
      {name:"NAND DATA1 partition",
	      offset:128 * 0x100000,
      size:128 * 0x100000},
      {name:"NAND DATA2 partition",
	      offset:256 * 0x100000,
      size:256 * 0x100000},
      {name:"NAND VFAT partition",
	      offset:512 * 0x100000,
      size:512 * 0x100000},
};


/* Define max reserved bad blocks for each partition.
 * This is used by the mtdblock-jz.c NAND FTL driver only.
 *
 * The NAND FTL driver reserves some good blocks which can't be
 * seen by the upper layer. When the bad block number of a partition
 * exceeds the max reserved blocks, then there is no more reserved
 * good blocks to be used by the NAND FTL driver when another bad
 * block generated.
 */
static int partition_reserved_badblocks[] = {
	2,			/* reserved blocks of mtd0 */
	2,			/* reserved blocks of mtd1 */
	10,			/* reserved blocks of mtd2 */
	10,			/* reserved blocks of mtd3 */
	20,			/* reserved blocks of mtd4 */
	20
};				/* reserved blocks of mtd5 */
#endif				/* CONFIG_JZ4750_FUWA */

/*-------------------------------------------------------------------------
 * Following three functions are exported and used by the mtdblock-jz.c
 * NAND FTL driver only.
 */

unsigned short get_mtdblock_write_verify_enable(void)
{
#ifdef CONFIG_MTD_MTDBLOCK_WRITE_VERIFY_ENABLE
	return 1;
#endif
	return 0;
}

EXPORT_SYMBOL(get_mtdblock_write_verify_enable);

unsigned short get_mtdblock_oob_copies(void)
{
	return CONFIG_MTD_OOB_COPIES;
}

EXPORT_SYMBOL(get_mtdblock_oob_copies);

int *get_jz_badblock_table(void)
{
	return partition_reserved_badblocks;
}

EXPORT_SYMBOL(get_jz_badblock_table);

/*-------------------------------------------------------------------------*/

static void jz_hwcontrol(struct mtd_info *mtd, int dat, u32 ctrl)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	u32 nandaddr = (u32)this->IO_ADDR_W;

	if (ctrl & NAND_CTRL_CHANGE) {
		if (share_mode) {
			if (ctrl & NAND_ALE)
				nandaddr = (u32)((u32)(this->IO_ADDR_W) | 0x00010000);
			else
				nandaddr = (u32)((u32)(this->IO_ADDR_W) & ~0x00010000);

			if (ctrl & NAND_CLE)
				nandaddr = nandaddr | 0x00008000;
			else
				nandaddr = nandaddr & ~0x00008000;
		} else {
			if (ctrl & NAND_ALE)
				nandaddr = (u32)((u32)(this->IO_ADDR_W) | 0x00000010);
			else
				nandaddr = (u32)((u32)(this->IO_ADDR_W) & ~0x00000010);

			if (ctrl & NAND_CLE)
				nandaddr = nandaddr | 0x00000008;
			else
				nandaddr = nandaddr & ~0x00000008;
		}

		if (ctrl & NAND_NCE)
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
	ready = __gpio_get_pin(91);
	return ready;
}

/*
 * EMC setup
 */
static void jz_device_setup(void)
{
	/* Set NFE bit */
	REG_EMC_NFCSR |= EMC_NFCSR_NFE1;

	/* Read/Write timings */
	REG_EMC_SMCR1 = 0x04444400;
}

#ifdef CONFIG_MTD_HW_BCH_ECC

static void jzsoc_nand_enable_bch_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccbytes = this->ecc.bytes;
#if defined(CONFIG_MTD_NAND_DMA)
	int eccsteps = this->ecc.steps;
	int oob_per_eccsize = this->ecc.layout->eccpos[0] / eccsteps;
#endif

	REG_BCH_INTS = 0xffffffff;
	if (mode == NAND_ECC_READ) {
		__ECC_DECODING();

#if defined(CONFIG_MTD_NAND_DMA)
		__ecc_cnt_dec(eccsize + oob_per_eccsize + eccbytes);
		__ecc_dma_enable();
#else
		__ecc_cnt_dec(eccsize + eccbytes);
#endif
	}

	if (mode == NAND_ECC_WRITE) {
		__ECC_ENCODING();

#if defined(CONFIG_MTD_NAND_DMA)
		__ecc_cnt_enc(eccsize + oob_per_eccsize);
		__ecc_dma_enable();
#else
		__ecc_cnt_enc(eccsize);
#endif
	}

}

/**
 * bch_correct
 * @dat:        data to be corrected
 * @idx:        the index of error bit in an eccsize
 */
static void bch_correct(u8 * dat, int idx)
{
	int i, bit;		/* the 'bit' of i byte is error */
	i = (idx - 1) >> 3;
	bit = (idx - 1) & 0x7;
	dat[i] ^= (1 << bit);
}

#if defined(CONFIG_MTD_NAND_DMA)
/**
 * jzsoc_nand_bch_correct_data
 * @mtd:	mtd info structure
 * @dat:        data to be corrected
 * @errs0:      pointer to the dma target buffer of bch decoding which stores BHINTS and 
 *              BHERR0~3(8-bit BCH) or BHERR0~1(4-bit BCH)
 * @calc_ecc:   no used
 */
static int jzsoc_nand_bch_correct_data(struct mtd_info *mtd, u_char * dat, u_char * errs0, u_char * calc_ecc)
{
	u32 stat;
	u32 *errs = (u32 *)errs0;

	if (REG_DMAC_DCCSR(0) & DMAC_DCCSR_BERR) {
		stat = errs[0];
		dprintk("stat=%x err0:%x err1:%x \n", stat, errs[1], errs[2]);

		if (stat & BCH_INTS_ERR) {
			if (stat & BCH_INTS_UNCOR) {
				printk("NAND: Uncorrectable ECC error\n");
				return -1;
			} else {
				u32 errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
				switch (errcnt) {
#if defined(CONFIG_MTD_HW_BCH_8BIT)
				case 8:
					bch_correct(dat, (errs[4] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 7:
					bch_correct(dat, (errs[4] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				case 6:
					bch_correct(dat, (errs[3] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 5:
					bch_correct(dat, (errs[3] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
#endif
				case 4:
					bch_correct(dat, (errs[2] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 3:
					bch_correct(dat, (errs[2] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				case 2:
					bch_correct(dat, (errs[1] & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				case 1:
					bch_correct(dat, (errs[1] & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				default:
					break;
				}
			}
		}
	}

	return 0;
}

#else				/* cpu mode */
/**
 * jzsoc_nand_bch_correct_data
 * @mtd:	mtd info structure
 * @dat:        data to be corrected
 * @read_ecc:   pointer to ecc buffer calculated when nand writing
 * @calc_ecc:   no used
 */
static int jzsoc_nand_bch_correct_data(struct mtd_info *mtd, u_char * dat, u_char * read_ecc, u_char * calc_ecc)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccsize = this->ecc.size;
	int eccbytes = this->ecc.bytes;
	short k;
	u32 stat;

	/* Write data to REG_BCH_DR */
	for (k = 0; k < eccsize; k++) {
		REG_BCH_DR = dat[k];
	}

	/* Write parities to REG_BCH_DR */
	for (k = 0; k < eccbytes; k++) {
		REG_BCH_DR = read_ecc[k];
	}

	/* Wait for completion */
	__ecc_decode_sync();
	__ecc_disable();

	/* Check decoding */
	stat = REG_BCH_INTS;

	if (stat & BCH_INTS_ERR) {
		/* Error occurred */
		if (stat & BCH_INTS_UNCOR) {
			printk("NAND: Uncorrectable ECC error--\n");
			return -1;
		} else {
			u32 errcnt = (stat & BCH_INTS_ERRC_MASK) >> BCH_INTS_ERRC_BIT;
			switch (errcnt) {
#if defined(CONFIG_MTD_HW_BCH_8BIT)
			case 8:
				bch_correct(dat, (REG_BCH_ERR3 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 7:
				bch_correct(dat, (REG_BCH_ERR3 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				/* FALL-THROUGH */
			case 6:
				bch_correct(dat, (REG_BCH_ERR2 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 5:
				bch_correct(dat, (REG_BCH_ERR2 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				/* FALL-THROUGH */
#endif
			case 4:
				bch_correct(dat, (REG_BCH_ERR1 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 3:
				bch_correct(dat, (REG_BCH_ERR1 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				/* FALL-THROUGH */
			case 2:
				bch_correct(dat, (REG_BCH_ERR0 & BCH_ERR_INDEX_ODD_MASK) >> BCH_ERR_INDEX_ODD_BIT);
				/* FALL-THROUGH */
			case 1:
				bch_correct(dat, (REG_BCH_ERR0 & BCH_ERR_INDEX_EVEN_MASK) >> BCH_ERR_INDEX_EVEN_BIT);
				return 0;
			default:
				break;
			}
		}
	}

	return 0;
}
#endif				/* CONFIG_MTD_NAND_DMA */

static int jzsoc_nand_calculate_bch_ecc(struct mtd_info *mtd, const u_char * dat, u_char * ecc_code)
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int eccbytes = this->ecc.bytes;
	volatile u8 *paraddr = (volatile u8 *)BCH_PAR0;
	short i;

	/* Write data to REG_BCH_DR */
	for (i = 0; i < this->ecc.size; i++) {
		REG_BCH_DR = dat[i];
	}

	__ecc_encode_sync();
	__ecc_disable();

	for (i = 0; i < eccbytes; i++) {
		ecc_code[i] = *paraddr++;
	}

	return 0;
}

#if defined(CONFIG_MTD_NAND_DMA)

/**
 * nand_write_page_hwecc_bch_dma - [REPLACABLE] hardware ecc based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void nand_write_page_hwecc_bch_dma(struct mtd_info *mtd, struct nand_chip *chip, const uint8_t * buf)
{
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int pagesize = mtd->writesize;
	int oobsize = mtd->oobsize;
	int i, err;
	const u8 *databuf;
	u8 *oobbuf;
	jz_dma_desc_8word *desc;

#if USE_DMABUF
	memcpy(prog_buf, buf, pagesize);
	memcpy(prog_buf + pagesize, chip->oob_poi, oobsize);
	dma_cache_wback_inv((u32)prog_buf, pagesize + oobsize);
#else
	databuf = buf;
	oobbuf = chip->oob_poi;

	/* descriptors for encoding data blocks */
	desc = dma_desc_enc;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)databuf) + i * eccsize;	/* DMA source address */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
		dprintk("dma_desc_enc:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for encoding oob blocks */
	desc = dma_desc_enc1;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address, 28/4 = 7bytes */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
		dprintk("dma_desc_enc1:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptor for nand programing data block */
	desc = dma_desc_nand_prog;
	desc->dsadr = CPHYSADDR((u32)databuf);	/* DMA source address */
	dprintk("dma_desc_nand_prog:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptor for nand programing oob block */
	desc++;
	desc->dsadr = CPHYSADDR((u32)oobbuf);	/* DMA source address */
	dprintk("dma_desc_oob_prog:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	dma_cache_wback_inv((u32)dma_desc_enc, (eccsteps * 2 + 2) * (sizeof(jz_dma_desc_8word)));
	dma_cache_wback_inv((u32)databuf, pagesize);
	dma_cache_wback_inv((u32)oobbuf, oobsize);
#endif

	REG_DMAC_DCCSR(bch_dma_chan) = 0;
	REG_DMAC_DCCSR(nand_dma_chan) = 0;

	/* Setup DMA descriptor address */
	REG_DMAC_DDA(bch_dma_chan) = CPHYSADDR((u32)dma_desc_enc);
	REG_DMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_nand_prog);

	/* Setup request source */
	REG_DMAC_DRSR(bch_dma_chan) = DMAC_DRSR_RS_BCH_ENC;
	REG_DMAC_DRSR(nand_dma_chan) = DMAC_DRSR_RS_AUTO;

	/* Setup DMA channel control/status register */
	REG_DMAC_DCCSR(bch_dma_chan) = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;	/* descriptor transfer, clear status, start channel */

	/* Enable DMA */
	REG_DMAC_DMACR(0) |= DMAC_DMACR_DMAE;
	REG_DMAC_DMACR(nand_dma_chan/HALF_DMA_NUM) |= DMAC_DMACR_DMAE;

	/* Enable BCH encoding */
	chip->ecc.hwctl(mtd, NAND_ECC_WRITE);

	dma_ack1 = 0;
	nand_status = NAND_PROG;

	/* DMA doorbell set -- start DMA now ... */
	__dmac_channel_set_doorbell(bch_dma_chan);

#if USE_IRQ
	dprintk("nand prog before wake up\n");
	err = wait_event_interruptible_timeout(nand_prog_wait_queue, dma_ack1, 3 * HZ);
	nand_status = NAND_NONE;
	dprintk("nand prog after wake up\n");
	if (!err) {
		dump_jz_dma_channel(0);
		dump_jz_dma_channel(nand_dma_chan);
		printk("use irq, %s Warning, wait event 3s timeout reached\n", __FILE__);
	}
	dprintk("timeout remain = %d\n", err);
#else
	while ((!__dmac_channel_transmit_end_detected(nand_dma_chan)) && (timeout--));
	if (timeout <= 0)
		printk("not use irq, prog timeout!!!!!!\n");
#endif
}
#endif				/* CONFIG_MTD_NAND_DMA */

/**
 * nand_read_page_hwecc_bch - [REPLACABLE] hardware ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * Not for syndrome calculating ecc controllers which need a special oob layout
 */
#if defined(CONFIG_MTD_NAND_DMA)
static int nand_read_page_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	int i, eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int pagesize = mtd->writesize;
	int oobsize = mtd->oobsize;
	u8 *tmpbuf, *databuf, *oobbuf;
	jz_dma_desc_8word *desc;
	u32 page = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
	int err;

#if USE_DMABUF
	dma_cache_inv((u32)read_buf, pagesize + oobsize);	// databuf should be invalidated.
	memset(errs, 0, eccsteps * ERRS_SIZE * 4);
	dma_cache_wback_inv((u32)errs, eccsteps * ERRS_SIZE * 4);
#else

	databuf = buf;
	oobbuf = chip->oob_poi;

	/* descriptor for nand reading data block */
	desc = dma_desc_nand_read;
	desc->dtadr = CPHYSADDR((u32)databuf);	/* DMA target address */

	dprintk("desc_nand_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptor for nand reading oob block */
	desc++;
	desc->dtadr = CPHYSADDR((u32)oobbuf);	/* DMA target address */
	dprintk("desc_oob_read:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
	       desc->ddadr);

	/* descriptors for data to be written to bch */
	desc = dma_desc_dec;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)databuf) + i * eccsize;	/* DMA source address */
		dprintk("dma_desc_dec:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for oob to be written to bch */
	desc = dma_desc_dec1;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address */
		dprintk("dma_desc_dec1:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for parities to be written to bch */
	desc = dma_desc_dec2;
	for (i = 0; i < eccsteps; i++) {
		desc->dsadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA source address */
		dprintk("dma_desc_dec2:desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	dma_cache_wback_inv((u32)dma_desc_nand_read, (2 + eccsteps * 3) * (sizeof(jz_dma_desc_8word)));

	memset(errs, 0, eccsteps * ERRS_SIZE * 4);
	dma_cache_inv((u32)databuf, pagesize);	// databuf should be invalidated.
	dma_cache_inv((u32)oobbuf, oobsize);	// oobbuf should be invalidated too
	dma_cache_wback_inv((u32)errs, eccsteps * ERRS_SIZE * 4);
#endif
	REG_DMAC_DCCSR(bch_dma_chan) = 0;
	REG_DMAC_DCCSR(nand_dma_chan) = 0;

	/* Setup DMA descriptor address */
	REG_DMAC_DDA(nand_dma_chan) = CPHYSADDR((u32)dma_desc_nand_read);
	REG_DMAC_DDA(bch_dma_chan) = CPHYSADDR((u32)dma_desc_dec);

	/* Setup request source */
	REG_DMAC_DRSR(nand_dma_chan) = DMAC_DRSR_RS_NAND;
	REG_DMAC_DRSR(bch_dma_chan) = DMAC_DRSR_RS_BCH_DEC;

	/* Enable DMA */
	REG_DMAC_DMACR(0) |= DMAC_DMACR_DMAE;
	REG_DMAC_DMACR(nand_dma_chan/HALF_DMA_NUM) |= DMAC_DMACR_DMAE;

	/* Enable BCH decoding */
	chip->ecc.hwctl(mtd, NAND_ECC_READ);

	dma_ack = 0;
	nand_status = NAND_READ;
	/* DMA doorbell set -- start nand DMA now ... */
	__dmac_channel_set_doorbell(nand_dma_chan);

	/* Setup DMA channel control/status register */
	REG_DMAC_DCCSR(nand_dma_chan) = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;

#define __nand_cmd(n)		(REG8(cmdport) = (n))
#define __nand_addr(n)		(REG8(addrport) = (n))

	__nand_cmd(NAND_CMD_READ0);

	__nand_addr(0);
	if (pagesize != 512)
		__nand_addr(0);

	__nand_addr(page & 0xff);
	__nand_addr((page >> 8) & 0xff);

	/* One more address cycle for the devices whose number of page address bits > 16  */
	if (((chip->chipsize >> chip->page_shift) >> 16) - 1 > 0)
		__nand_addr((page >> 16) & 0xff);

	if (pagesize != 512)
		__nand_cmd(NAND_CMD_READSTART);

#if USE_IRQ
	err = wait_event_interruptible_timeout(nand_read_wait_queue, dma_ack, 3 * HZ);
	nand_status = NAND_NONE;

	if (!err) {
		printk("*** NAND READ, Warning, wait event 3s timeout reached\n");
		dump_jz_dma_channel(0);
		printk("REG_BCH_CR=%x REG_BCH_CNT=0x%x REG_BCH_INTS=%x\n", REG_BCH_CR, REG_BCH_CNT, REG_BCH_INTS);
		printk("databuf[0]=%x\n", databuf[0]);
	}
	dprintk("timeout remain = %d\n", err);
#else
	int timeout;
	timeout = 100000;
	while ((!__dmac_channel_transmit_end_detected(bch_dma_chan)) && (timeout--));
	if (timeout <= 0) {
		printk("not use irq, NAND READ timeout!!!!!!\n");
	}
#endif

#if USE_DMABUF
	tmpbuf = read_buf;
#else
	tmpbuf = buf;
#endif
	for (i = 0; i < eccsteps; i++, tmpbuf += eccsize) {
		int stat;

		stat = chip->ecc.correct(mtd, tmpbuf, (u8 *)&errs[i * ERRS_SIZE], NULL);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}

#if USE_DMABUF
	memcpy(buf, read_buf, pagesize);
	memcpy(chip->oob_poi, read_buf + pagesize, oobsize);
#endif
	return 0;
}
#else				/* nand read in cpu mode */
static int nand_read_page_hwecc_bch(struct mtd_info *mtd, struct nand_chip *chip, uint8_t * buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int pagesize = mtd->writesize;
	int oobsize = mtd->oobsize;

	dprintk("cpu nand_read_page_hwecc_bch--\n");
	chip->read_buf(mtd, buf, pagesize);
	chip->read_buf(mtd, chip->oob_poi, oobsize);

	for (i = 0; i < chip->ecc.total; i++) {
		ecc_code[i] = chip->oob_poi[eccpos[i]];
	}

	eccsteps = chip->ecc.steps;
	p = buf;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}

	return 0;
}
#endif				/* CONFIG_MTD_NAND_DMA */

#endif				/* CONFIG_MTD_HW_BCH_ECC */


#if defined(CONFIG_MTD_NAND_DMA)

#if USE_IRQ
static irqreturn_t nand_dma_irq(int irq, void *dev_id)
{
	u8 dma_chan;
	volatile int wakeup = 0;

	dma_chan = irq - IRQ_DMA_0;

	dprintk("jz4750_dma_irq %d, channel %d\n", irq, dma_chan);

	if (__dmac_channel_transmit_halt_detected(dma_chan)) {
		printk("DMA HALT\n");
		__dmac_channel_clear_transmit_halt(dma_chan);
		wakeup = 1;
	}

	if (__dmac_channel_address_error_detected(dma_chan)) {

		printk("REG_DMAC_DMADBR(0)=0x%08x\n", REG_DMAC_DMADBR(0));
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;	/* disable DMA */
		__dmac_channel_clear_address_error(dma_chan);

		REG_DMAC_DSAR(dma_chan) = 0;	/* reset source address register */
		REG_DMAC_DTAR(dma_chan) = 0;	/* reset destination address register */
		printk("REG_DMAC_DMADBR(0)=0x%08x\n", REG_DMAC_DMADBR(0));

		/* clear address error in DMACR */
		REG_DMAC_DMACR((dma_chan / HALF_DMA_NUM)) &= ~(1 << 2);
		wakeup = 1;
	}

	if (__dmac_channel_descriptor_invalid_detected(dma_chan)) {
		printk("DMA DESC INVALID\n");
		__dmac_channel_clear_descriptor_invalid(dma_chan);
		wakeup = 1;
	}
#if 1

	while (!__dmac_channel_transmit_end_detected(dma_chan));

	if (__dmac_channel_count_terminated_detected(dma_chan)) {
		dprintk("DMA CT\n");
		__dmac_channel_clear_count_terminated(dma_chan);
		wakeup = 0;
	}
#endif

	if (__dmac_channel_transmit_end_detected(dma_chan)) {
		dprintk("DMA TT\n");
		REG_DMAC_DCCSR(dma_chan) &= ~DMAC_DCCSR_EN;	/* disable DMA */
		__dmac_channel_clear_transmit_end(dma_chan);
		wakeup = 1;
	}

	if (wakeup) {
		dprintk("ack %d irq , wake up dma_chan %d nand_status %d\n", dma_ack, dma_chan, nand_status);
		/* wakeup wait event */
		if ((dma_chan == nand_dma_chan) && (nand_status == NAND_PROG)) {
			dprintk("nand prog dma irq, wake up----\n");
			dma_ack1 = 1;
			wake_up_interruptible(&nand_prog_wait_queue);
		}

		if ((dma_chan == bch_dma_chan) && (nand_status == NAND_READ)) {
			dprintk("nand read irq, wake up----\n");
			dma_ack = 1;
			wake_up_interruptible(&nand_read_wait_queue);
		}
		wakeup = 0;
	}

	return IRQ_HANDLED;
}
#endif				/* USE_IRQ */

static int jz4750_nand_dma_init(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int ecc_pos = chip->ecc.layout->eccpos[0];
	int oob_per_eccsize = ecc_pos / eccsteps;
	int pagesize = mtd->writesize;
	int oobsize = mtd->oobsize;
	int i, err;
	jz_dma_desc_8word *desc, *dma_desc_bch_ddr, *dma_desc_nand_ddr;
	u32 *pval_nand_ddr, *pval_nand_dcs, *pval_bch_ddr, *pval_bch_dcs;
	u32 next;
#if USE_DMABUF
	u8 *oobbuf;
#endif

	if (share_mode) {
		addrport = 0xb8010000;
		cmdport = 0xb8008000;
	} else {
		addrport = 0xb8000010;
		cmdport = 0xb8000008;
	}

#if USE_IRQ
	if ((nand_dma_chan = jz_request_dma(DMA_ID_NAND, "nand read or write", nand_dma_irq, IRQF_DISABLED, NULL)) < 0) {
		printk("can't reqeust DMA nand channel.\n");
		return 0;
	}
	dprintk("nand dma channel:%d----\n", nand_dma_chan);

	if ((err = request_irq(IRQ_DMA_0 + bch_dma_chan, nand_dma_irq, IRQF_DISABLED, "bch_dma", NULL))) {
		printk("bch_dma irq request err\n");
		return 0;
	}
#else
	if ((nand_dma_chan = jz_request_dma(DMA_ID_NAND, "nand read or write", NULL, IRQF_DISABLED, NULL)) < 0) {
		printk("can't reqeust DMA nand channel.\n");
		return 0;
	}
	dprintk("nand dma channel:%d----\n", nand_dma_chan);
#endif

	__dmac_channel_enable_clk(nand_dma_chan);
	__dmac_channel_enable_clk(bch_dma_chan);

#if USE_DMABUF
	if (pagesize < 4096) {
		read_buf = prog_buf = (u8 *) __get_free_page(GFP_KERNEL);
	} else {
		read_buf = prog_buf = (u8 *) __get_free_pages(GFP_KERNEL, 1);
	}
#endif
	/* space for the error reports of bch decoding((4 * 5 * eccsteps) bytes), and the space for the value 
         * of ddr and dcs of channel 0 and channel nand_dma_chan (4 * (2 + 2) bytes) */
	errs = (u32 *)kmalloc(4 * (2 + 2 + 5 * eccsteps), GFP_KERNEL);
	pval_nand_ddr = errs + 5 * eccsteps;
	pval_nand_dcs = pval_nand_ddr + 1;
	pval_bch_ddr = pval_nand_dcs + 1;
	pval_bch_dcs = pval_bch_ddr + 1;

	/* desc can't across 4KB boundary, as desc base address is fixed */
	/* space of descriptors for nand reading data and oob blocks */
	dma_desc_nand_read = (jz_dma_desc_8word *) __get_free_page(GFP_KERNEL);

	/* space of descriptors for bch decoding */
	dma_desc_dec = dma_desc_nand_read + 2;
	dma_desc_dec1 = dma_desc_dec + eccsteps;
	dma_desc_dec2 = dma_desc_dec + eccsteps * 2;

	/* space of descriptors for notifying bch channel */
	dma_desc_bch_ddr = dma_desc_dec2 + eccsteps;

	/* space of descriptors for bch encoding */
	dma_desc_enc = dma_desc_bch_ddr + 2;
	dma_desc_enc1 = dma_desc_enc + eccsteps;

	/* space of descriptors for nand programing data and oob blocks*/
	dma_desc_nand_prog = dma_desc_enc1 + eccsteps;

	/* space of descriptors for notifying nand channel */
	dma_desc_nand_ddr = dma_desc_nand_prog + 2;

/*************************************
 * Setup of nand programing descriptors
 *************************************/
#if USE_DMABUF
	oobbuf = prog_buf + pagesize;
#endif
	/* set descriptor for encoding data blocks */
	desc = dma_desc_enc;
	for (i = 0; i < eccsteps; i++) {
		next = (CPHYSADDR((u32)dma_desc_enc1) + i * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
		    DMAC_DCMD_DS_16BYTE | DMAC_DCMD_LINK;
#if USE_DMABUF
		desc->dsadr = CPHYSADDR((u32)prog_buf) + i * eccsize;	/* DMA source address */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
#endif
		desc->ddadr = (next << 24) + eccsize / 16;	/* size: eccsize bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_ENC;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);
		desc++;
	}

	/* set descriptor for encoding oob blocks */
	desc = dma_desc_enc1;
	for (i = 0; i < eccsteps; i++) {
		next = (CPHYSADDR((u32)dma_desc_enc) + (i + 1) * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_BLAST | DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 |
		    DMAC_DCMD_DWDH_8 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_LINK;
#if USE_DMABUF
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address, 28/4 = 7bytes */
		desc->dtadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA target address */
#endif
		desc->ddadr = (next << 24) + (oob_per_eccsize + 3) / 4;	/* size: 7 bytes -> 2 words */
		desc->dreqt = DMAC_DRSR_RS_BCH_ENC;
		dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);
		desc++;
	}

	next = (CPHYSADDR((u32)dma_desc_nand_ddr)) >> 4;
	desc--;
	desc->ddadr = (next << 24) + (oob_per_eccsize + 3) / 4;

	/* set the descriptor to set door bell of nand_dma_chan for programing nand */
	desc = dma_desc_nand_ddr;
	*pval_nand_ddr = 1 << (nand_dma_chan - nand_dma_chan / 6 * 6);
	next = (CPHYSADDR((u32)dma_desc_nand_ddr) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_nand_ddr);	/* DMA source address */
	desc->dtadr = CPHYSADDR(DMAC_DMADBSR(nand_dma_chan / 6));	/* nand_dma_chan's descriptor addres register */
	desc->ddadr = (next << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("*pval_nand_ddr=0x%x\n", *pval_nand_ddr);

	/* set the descriptor to write dccsr of nand_dma_chan for programing nand, dccsr should be set at last */
	desc++;
	*pval_nand_dcs = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;	/* set value for writing ddr to enable channel nand_dma_chan */
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT;
	desc->dsadr = CPHYSADDR((u32)pval_nand_dcs);	/* DMA source address */
	desc->dtadr = CPHYSADDR(DMAC_DCCSR(nand_dma_chan));	/* address of dma door bell set register */
	desc->ddadr = (0 << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("*pval_nand_dcs=0x%x\n", *pval_nand_dcs);

	/* set descriptor for nand programing data block */
	desc = dma_desc_nand_prog;
	next = (CPHYSADDR((u32)dma_desc_nand_prog) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
	    DMAC_DCMD_DS_16BYTE | DMAC_DCMD_LINK;
#if USE_DMABUF
	desc->dsadr = CPHYSADDR((u32)prog_buf);	/* DMA source address */
#endif
	desc->dtadr = CPHYSADDR(NAND_DATA_PORT);	/* DMA target address */
	desc->ddadr = (next << 24) + pagesize / 16;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set descriptor for nand programing oob block */
	desc++;
#if USE_IRQ
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
	    DMAC_DCMD_DS_16BYTE | DMAC_DCMD_TIE;
#else
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_8 |
	    DMAC_DCMD_DS_16BYTE;
#endif
#if USE_DMABUF
	desc->dsadr = CPHYSADDR((u32)oobbuf);	/* DMA source address */
#endif
	desc->dtadr = CPHYSADDR(NAND_DATA_PORT);	/* DMA target address */
	desc->ddadr = (0 << 24) + oobsize / 16;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	dma_cache_wback_inv((u32)dma_desc_enc, (eccsteps * 2 + 2 + 2) * (sizeof(jz_dma_desc_8word)));
	dma_cache_wback_inv((u32)pval_nand_ddr, 4 * 2); /* two words */

/*************************************
 * Setup of nand reading descriptors
 *************************************/
#if USE_DMABUF
	oobbuf = read_buf + pagesize;
#endif
	/* set descriptor for nand reading data block */
	desc = dma_desc_nand_read;
	next = (CPHYSADDR((u32)dma_desc_nand_read) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 |
	    DMAC_DCMD_DS_16BYTE | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR(NAND_DATA_PORT);	/* DMA source address */
#if USE_DMABUF
	desc->dtadr = CPHYSADDR((u32)read_buf);	/* DMA target address */
#endif
	desc->ddadr = (next << 24) + pagesize / 16;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_NAND;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set descriptor for nand reading oob block */
	desc++;
	next = (CPHYSADDR((u32)dma_desc_bch_ddr)) >> 4;
	desc->dcmd =
	    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 |
	    DMAC_DCMD_DS_16BYTE | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR(NAND_DATA_PORT);	/* DMA source address */
#if USE_DMABUF
	desc->dtadr = CPHYSADDR((u32)oobbuf);	/* DMA target address */
#endif
	desc->ddadr = (next << 24) + oobsize / 16;	/* size: eccsize bytes */
	desc->dreqt = DMAC_DRSR_RS_AUTO;
	dprintk("cmd:%x sadr:%x tadr:%x dadr:%x\n", desc->dcmd, desc->dsadr, desc->dtadr, desc->ddadr);

	/* set the descriptor to set door bell for bch */
	desc = dma_desc_bch_ddr;
	*pval_bch_ddr = DMAC_DMADBSR_DBS0;	// set value for writing ddr to enable channel 0
	next = (CPHYSADDR((u32)dma_desc_bch_ddr) + sizeof(jz_dma_desc_8word)) >> 4;
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT | DMAC_DCMD_LINK;
	desc->dsadr = CPHYSADDR((u32)pval_bch_ddr);	/* DMA source address */
	desc->dtadr = CPHYSADDR(DMAC_DMADBSR(0));	/* channel 1's descriptor addres register */
	desc->ddadr = (next << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;

	/* set descriptor for writing dcsr */
	desc++;
	*pval_bch_dcs = DMAC_DCCSR_DES8 | DMAC_DCCSR_EN;	// set value for writing ddr to enable channel 1
	desc->dcmd = DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_32BIT;
	desc->dsadr = CPHYSADDR((u32)pval_bch_dcs);	/* DMA source address */
	desc->dtadr = CPHYSADDR(DMAC_DCCSR(bch_dma_chan));	/* address of dma door bell set register */
	desc->ddadr = (0 << 24) + 1;	/* size: 1 word */
	desc->dreqt = DMAC_DRSR_RS_AUTO;

	/* descriptors for data to be written to bch */
	desc = dma_desc_dec;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_dec1 + i * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_32 | DMAC_DCMD_DWDH_32 |
		    DMAC_DCMD_DS_16BYTE | DMAC_DCMD_LINK;
#if USE_DMABUF
		desc->dsadr = CPHYSADDR((u32)read_buf) + i * eccsize;	/* DMA source address */
#endif
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->ddadr = (next << 24) + eccsize / 16;	/* size: eccsize bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_DEC;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for oob to be written to bch */
	desc = dma_desc_dec1;
	for (i = 0; i < eccsteps; i++) {
		next = CPHYSADDR((u32)dma_desc_dec2 + i * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 | DMAC_DCMD_DWDH_32 |
		    DMAC_DCMD_DS_8BIT | DMAC_DCMD_LINK;
#if USE_DMABUF
		desc->dsadr = CPHYSADDR((u32)oobbuf) + oob_per_eccsize * i;	/* DMA source address */
#endif
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->ddadr = (next << 24) + oob_per_eccsize;	/* size: 7 bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_DEC;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}

	/* descriptors for parities to be written to bch */
	desc = dma_desc_dec2;
	for (i = 0; i < eccsteps; i++) {
		next = (CPHYSADDR((u32)dma_desc_dec) + (i + 1) * (sizeof(jz_dma_desc_8word))) >> 4;

		desc->dcmd =
		    DMAC_DCMD_BLAST | DMAC_DCMD_SAI | DMAC_DCMD_DAI | DMAC_DCMD_RDIL_IGN | DMAC_DCMD_SWDH_8 |
		    DMAC_DCMD_DWDH_32 | DMAC_DCMD_DS_16BYTE | DMAC_DCMD_LINK;
#if USE_DMABUF
		desc->dsadr = CPHYSADDR((u32)oobbuf) + ecc_pos + i * eccbytes;	/* DMA source address */
#endif
		desc->dtadr = CPHYSADDR((u32)errs) + i * 4 * ERRS_SIZE;	/* DMA target address */
		desc->ddadr = (next << 24) + (eccbytes + 15) / 16;	/* size: eccbytes bytes */
		desc->dreqt = DMAC_DRSR_RS_BCH_DEC;
		dprintk("desc:%x cmd:%x sadr:%x tadr:%x dadr:%x\n", (u32)desc, desc->dcmd, desc->dsadr, desc->dtadr,
			desc->ddadr);
		desc++;
	}
	desc--;
	desc->dcmd &= ~DMAC_DCMD_LINK;
#if USE_IRQ
	desc->dcmd |= DMAC_DCMD_TIE;
#endif

	dma_cache_wback_inv((u32)dma_desc_nand_read, (2 + 2 + eccsteps * 3) * (sizeof(jz_dma_desc_8word)));
	dma_cache_wback_inv((u32)pval_bch_ddr, 4 * 2); /* two words */

	return 0;
}

#endif				/* CONFIG_MTD_NAND_DMA */
/*
 * Main initialization routine
 */
int __init jznand_init(void)
{
	struct nand_chip *this;
	int nr_partitions;

	dprintk("--use buf?%d--jznand_init----\n", USE_DMABUF);

	__gpio_as_nand_8bit(1);

	/* Allocate memory for MTD device structure and private data */
	jz_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	if (!jz_mtd) {
		printk("Unable to allocate JzSOC NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&jz_mtd[1]);

	/* Initialize structures */
	memset((char *)jz_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	jz_mtd->priv = this;

	if (is_share_mode())
		share_mode = 1;
	else
		share_mode = 0;

	/* Set & initialize NAND Flash controller */
	jz_device_setup();

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (void __iomem *)NAND_DATA_PORT;
	this->IO_ADDR_W = (void __iomem *)NAND_DATA_PORT;
	this->cmd_ctrl = jz_hwcontrol;
	this->dev_ready = jz_device_ready;

#ifdef CONFIG_MTD_HW_BCH_ECC
	this->ecc.calculate = jzsoc_nand_calculate_bch_ecc;
	this->ecc.correct = jzsoc_nand_bch_correct_data;
	this->ecc.hwctl = jzsoc_nand_enable_bch_hwecc;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = 512;
	this->ecc.read_page = nand_read_page_hwecc_bch;
#if defined(CONFIG_MTD_NAND_DMA)
	this->ecc.write_page = nand_write_page_hwecc_bch_dma;
#endif
#if defined(CONFIG_MTD_HW_BCH_8BIT)
	this->ecc.bytes = 13;
#else
	this->ecc.bytes = 7;
#endif
#endif

#ifdef  CONFIG_MTD_SW_HM_ECC
	this->ecc.mode = NAND_ECC_SOFT;
#endif
	/* 20 us command delay time */
	this->chip_delay = 20;

	/* Scan to find existance of the device */
	if (nand_scan(jz_mtd, 1)) {
		kfree(jz_mtd);
		return -ENXIO;
	}
#if defined(CONFIG_MTD_NAND_DMA)
	jz4750_nand_dma_init(jz_mtd);
#endif

	/* Register the partitions */
	nr_partitions = sizeof(partition_info) / sizeof(struct mtd_partition);
	add_mtd_partitions(jz_mtd, partition_info, nr_partitions);

	return 0;
}

module_init(jznand_init);

/*
 * Clean up routine
 */
#ifdef MODULE

#if defined(CONFIG_MTD_NAND_DMA)
static int jz4750_nand_dma_exit(struct mtd_info *mtd)
{
	int pagesize = mtd->writesize;

#if USE_IRQ
	free_irq(IRQ_DMA_0 + nand_dma_chan, NULL);
	free_irq(IRQ_DMA_0 + bch_dma_chan, NULL);
#endif

	/* space for the error reports of bch decoding((4 * 5 * eccsteps) bytes),
         * and the space for the value of ddr and dcs of channel 0 and channel 
         * nand_dma_chan (4 * (2 + 2) bytes) */
	kfree(errs);

	/* space for dma_desc_nand_read contains dma_desc_nand_prog, 
	 * dma_desc_enc and dma_desc_dec */
	free_page((u32)dma_desc_nand_read);

#if USE_DMABUF
	if (pagesize < 4096) {
		free_page((u32)prog_buf);
	} else {
		free_pages((u32)prog_buf, 1);
	}
#endif

	__dmac_channel_disable_clk(nand_dma_chan);
	__dmac_channel_disable_clk(bch_dma_chan);

	return 0;
}
#endif

static void __exit jznand_cleanup(void)
{
	struct nand_chip *this = (struct nand_chip *)&jz_mtd[1];

#if defined(CONFIG_MTD_NAND_DMA)
	jz4750_nand_dma_exit(jz_mtd);
#endif

	/* Unregister partitions */
	del_mtd_partitions(jz_mtd);

	/* Unregister the device */
	del_mtd_device(jz_mtd);

	/* Free internal data buffers */
	kfree(this->data_buf);

	/* Free the MTD device structure */
	kfree(jz_mtd);
}

module_exit(jznand_cleanup);
#endif
