/*
 *  sdmparse.c : sdm partition parser
 *
 *  Copyright 2002 Sony Corporation.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  version 2 of the  License.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * $Id: sdmparse.c,v 1.7.14.3 2003/05/08 05:33:57 satoru-i Exp $
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/crc32.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/sdmparse.h>

/* use CRC checking or not */
#define SDM_CRC_CHECK

/* support zlib decompression */
#define SDM_ZLIB_DECOMP
#define SDM_ZALLOC               0      /* use default func. */
#define SDM_ZFREE                0      /* use default func. */

/* adjust crc value to SDM */
#define sdm_crc32(buf, len)      (crc32(0x0, buf, len))
#define gz_crc32(crc, buf, len)  (crc32(crc, buf, len))


/* Configuration */
#define SDM_NOR_DEVICES           1
#define SDM_NAND_DEVICES          2
#define SDM_MASTER_DEV            (SDM_NOR_DEVICES)  /* first nand device */
#define SDM_DEFAULT_MAXDESC      16  /* Maximum number of sdm descriptor */
#define SDM_TABLE_STARTSECT       0  /* sdm table starts from sector 0 */
#define SDM_RESERVE_TABLE_SECTS   4  /* number of sectors reserved for sdm table wear leveling */

#define MAX_SDM_DEV (SDM_NOR_DEVICES + SDM_NAND_DEVICES)

#define SDM_DEV_PREFIX "sdm device"
#define MAX_NUM_RETRY  1

#define COLOR_NOT_SET  0
#define COLOR_RED      0x00ff
#define COLOR_BLUE     0xff00

static struct mtd_info *devices[MAX_SDM_DEV];


static int
check_sdm_table(struct sdm_table *ptable, u_int32_t sdm_table_size)
{
        u_int32_t crc;

        if (ptable->magic != SDM_TABLE_MAGIC) {
                SDM_INFO("SDM table invalid magic number 0x%x(0x%x)\n",
			 ptable->magic, SDM_TABLE_MAGIC);
                return SDM_ERR;
        }
        if (ptable->ver_major < SDM_VERSION2) {
                SDM_INFO("SDM table old version\n");
                return SDM_ERR;
        }

        if (ptable->max_descs != SDM_DEFAULT_MAXDESC) {
                SDM_INFO("SDM table has different number of "
                         "max descriptors %d(%d)\n",
                         ptable->max_descs, SDM_DEFAULT_MAXDESC);
                return SDM_ERR;
        }

	/* check crc over the table if the table version is grater than 2.4 */
	if (ptable->ver_major > 2 ||
	    (ptable->ver_major == 2 && ptable->ver_minor >= 4)) {
		u_int32_t cksum_bak = ptable->table_cksum;
		ptable->table_cksum = SDM_TABLE_CKSUM_SEED;
		crc = sdm_crc32((u_int8_t *)ptable, SDM_TABLE_DESC_OFFSET);
		ptable->table_cksum = cksum_bak;
		if (crc != cksum_bak) {
			SDM_INFO("SDM table invalid cksum 0x%x(0x%x)\n",
				 cksum_bak, crc);
			return SDM_ERR;
		}
	}

        crc = sdm_crc32((u_int8_t *)((u_int32_t)ptable + SDM_TABLE_DESC_OFFSET),
                        sdm_table_size - SDM_TABLE_DESC_OFFSET);
        if (ptable->cksum != crc) {
                SDM_INFO("SDM table invalid cksum 0x%x(0x%x)\n",
                         ptable->cksum, crc);
                return SDM_ERR;
        }
        return SDM_OK;
}

static int is_badblock(struct mtd_info *mtd, u_int32_t block)
{
	if (mtd->type != MTD_NANDFLASH)
		return 0;

	if (mtd->block_isbad &&
			mtd->block_isbad(mtd, (block * mtd->erasesize)))
		return 1;
	else
		return 0;
}

static int is_badblock_inside(struct mtd_info *mtd, u_int32_t start, u_int32_t sectors)
{
	int i, rval;
	int ret = 0;

	for (i = 0; i < sectors; i++) {

		rval = is_badblock(mtd, start + i);

		if (rval < 0)
			return rval;

		ret |= rval;
	}

	return ret;
}

static u_int16_t decode_color(u_int16_t color_code)
{
	int i;
	int color_blue = 0, color_red = 0;

	for (i = 0; i < 8; i++, color_code >>= 1) {
		if (color_code & 1)
			color_red++;
	}
	for (i = 0; i < 8; i++, color_code >>= 1) {
		if (color_code & 1)
			color_blue++;
	}
	if (color_red == color_blue)
		return COLOR_NOT_SET;

	return (color_red > color_blue) ? COLOR_RED : COLOR_BLUE;
}

static struct sdm_table *load_sdm_table(struct mtd_info *mtd, u_int32_t start_sector,
	       u_int32_t table_size, u_int32_t reserved_sectors)
{
#define OOB_BUF_LEN         8

	struct sdm_table *ptable[2] = { NULL, NULL };
        u_int32_t table_sects;
	u_int16_t current_color = COLOR_NOT_SET;
	u_int16_t tmp_color;
	int tmp_table = 0, latest_table = -1;
	int i, num_retry, rval;
	size_t retlen;

        table_sects = (table_size + mtd->erasesize - 1) / mtd->erasesize;

	ptable[0] = (struct sdm_table *)
		kmalloc(mtd->erasesize * table_sects, GFP_KERNEL);
	ptable[1] = (struct sdm_table *)
		kmalloc(mtd->erasesize * table_sects, GFP_KERNEL);

	if (ptable[0] == NULL|| ptable[1] == NULL) {
                SDM_INFO("cannot allocate sdm table\n");
                goto load_cleanup_exit;
        }

	for (i = start_sector;
	     i + table_sects <= reserved_sectors; i += table_sects) {

		num_retry = 0;
retry:
		if (num_retry++ > MAX_NUM_RETRY) {
			/* give up for this sector */
			continue;
		}

		rval = is_badblock_inside(mtd, i, table_sects);
		if (rval < 0) {
			printk(KERN_WARNING
			       "read error occured while "
			       "check for badblock\n");
		} else if (rval) {
			/* bad block exists inside the region */
			continue;
		}

		rval = mtd->read(mtd, i * mtd->erasesize,
				 table_sects * mtd->erasesize,
				 &retlen, (u_char *)ptable[tmp_table]);

		if (rval < 0) {
			printk(KERN_WARNING
			       "# read error occured while "
			       "reading sdm table\n");
			goto retry;
		} else if (retlen != table_sects * mtd->erasesize) {
			printk(KERN_WARNING 
			       "# partial read error occured while "
			       "reading sdm table\n");
		}

		/* mtd->read success */
		if (ptable[tmp_table]->magic == 0xffffffff) {
			/* apparently not a sdm table */
			continue;
		}
		if (check_sdm_table(ptable[tmp_table], table_size) != SDM_OK) {
			goto retry;
		}

		/* valid sdm table found at current location */

		/* check if the table support table multiplexing */
		if (!(~ptable[tmp_table]->cap_flags & SDM_CAP_TABLE_MULTIPLEX)) {
			/* if i == start_sector (i.e. 1st SDM table),
			 * we use it as only one (not multiplexed) table. */
			if (i == start_sector) {
				latest_table = tmp_table;
				tmp_table = 1 - tmp_table;
				break;
			}

			/* valid sdm table found at elsewhere,
			 * we simply ignore the table. */
			continue;
		}

		tmp_color = decode_color(ptable[tmp_table]->color);

		if ( current_color != COLOR_NOT_SET &&
		     tmp_color != current_color ) {
			/* table color changed! */
			break;
		}

		/* found candidate for latest table */

		current_color = tmp_color;
		latest_table = tmp_table;
		tmp_table = 1 - tmp_table;
	}

load_cleanup_exit:

	if (latest_table != 0 && ptable[0])
		kfree(ptable[0]);
	if (latest_table != 1 && ptable[1])
		kfree(ptable[1]);

        return (latest_table < 0) ? NULL : ptable[latest_table];
}

/*
 * Main function. It was adopted from original Sony driver
 *
 * notice: only ONE (master) device is supported
 */
static int parse_sdm_partitions(struct mtd_info *master,
		struct mtd_partition **pparts,
		unsigned long origin)
{
	struct sdm_table *ptable = NULL;
	struct sdm_image_desc *img = NULL;
	struct mtd_partition *ppart = NULL;
	struct mtd_info *mtd = NULL;

	char *pname = NULL;
	int num_part = 0, namelen_total = 0;
	int num_nor_devs = 0, num_nand_devs = 0;
	u_int32_t table_size, table_sects, reserved_sects;
	int i;
	int ret = SDM_ERR;

	
	switch (master->type) {
		case MTD_NORFLASH:
			devices[num_nor_devs++] = master;
			break;
		case MTD_NANDFLASH:
			devices[SDM_NOR_DEVICES + num_nand_devs++] = master;
			break;
		default:
			printk(KERN_WARNING
			       "warning: unknown mtd SDM device type !\n");
			break;
	}

	/* check master device */
	if (devices[SDM_MASTER_DEV] == NULL) {
		printk(KERN_WARNING "SDM master device is not present!\n");
		goto parse_cleanup_exit;
	}

        /* allocate sdm_table */
	mtd = devices[SDM_MASTER_DEV];

	printk(KERN_INFO "trying to parse SDM table from '%s'\n",
	       mtd->name);

	table_size = sizeof(struct sdm_table)
		+ sizeof(struct sdm_image_desc) * SDM_DEFAULT_MAXDESC;
	table_sects = (table_size + mtd->erasesize - 1) / mtd->erasesize;

        if (table_size > mtd->size || table_sects > SDM_RESERVE_TABLE_SECTS) {
                SDM_INFO("too many max number of desriptors %d\n",
			 SDM_DEFAULT_MAXDESC);
                goto parse_cleanup_exit;
        }

	/* load sdm table */
#ifdef SDM_RESERVE_TABLE_SECTS
	reserved_sects = SDM_RESERVE_TABLE_SECTS;
#else
	reserved_sects = table_sects;
#endif
	if ((ptable = load_sdm_table(mtd, SDM_TABLE_STARTSECT, table_size,
				     reserved_sects)) == NULL) {
		printk(KERN_WARNING
		       "SDM table load failed from \"%s\"\n", mtd->name);
		goto parse_cleanup_exit;
	}

	if (~ptable->cap_flags & SDM_CAP_TABLE_MULTIPLEX) {
		printk(KERN_INFO "SDM table multiplexing supported\n");
	}

	/* parse sdm table */
	img = (struct sdm_image_desc *)
		((u_int32_t)ptable + SDM_TABLE_DESC_OFFSET);

        for (i = 0, num_part = 0; i < ptable->max_descs; i++) {
                if ((img[i].magic == SDM_TABLE_MAGIC) &&
				(img[i].dev == SDM_MASTER_DEV)) {
			namelen_total += strlen(img[i].name) + 1;
			num_part++;
		}
	}

	ppart = kmalloc(sizeof(struct mtd_partition) * num_part
			+ namelen_total, GFP_KERNEL);
	if (ppart == NULL) {
		SDM_INFO("cannot allocate MTD partition info table\n");
		goto parse_cleanup_exit;
	}
	memset(ppart, 0,
	       sizeof(struct mtd_partition) * num_part + namelen_total);
	pname = (char *)&ppart[num_part];

	
	for (i = 0, num_part = 0; i < ptable->max_descs; i++, img++) {
                if ((img->magic == SDM_TABLE_MAGIC) &&
				(img->dev == SDM_MASTER_DEV)) {

			/*
			 * TODO: we must use get_mtd_device() again
			 *       to increment the use count.
			 */
			mtd = devices[img->dev];

			ppart[num_part].name = pname;
			ppart[num_part].size = img->sectors * mtd->erasesize;
			ppart[num_part].offset = img->start * mtd->erasesize;
			pname += sprintf(pname, "%s", img->name) + 1;
//			add_mtd_partitions(mtd, &ppart[num_part], 1);
			num_part++;
		}
	}

	printk(KERN_INFO "found %d sdm partition(s)\n", num_part);

	*pparts = ppart;
	ret = num_part;

parse_cleanup_exit:

	if (ptable)
		kfree(ptable);

	/*
	 * XXX: There's no one who do kfree(ppart). If NOR and/or NAND driver
	 *      is removed, we must unregister all sdm partitions once,
	 *      and then re-parse and registere using add_mtd_partitions().
	 *      If the MTD device corresponding to device 'SDM_MASTER_DEV' is
	 *      unregistered, all sdm partitions need to be unregistered and
	 *      do kfree(ppart).
	 *      Currently, we assume that NOR driver and NAND driver are once
	 *      installed, they would never be removed.
	 */

        return ret;
}

static struct mtd_part_parser sdm_parser = {
	.owner		= THIS_MODULE,
	.parse_fn	= parse_sdm_partitions,
	.name		= "SDM",
};

static int __init sdm_parser_init(void)
{
	return register_mtd_parser(&sdm_parser);
}

static void __exit sdm_parser_exit(void)
{
	deregister_mtd_parser(&sdm_parser);
}

module_init(sdm_parser_init);
module_exit(sdm_parser_exit);

MODULE_AUTHOR("Sony NSC");
MODULE_DESCRIPTION("parse sdm table and translate to mtd partition");
MODULE_LICENSE("GPL");
