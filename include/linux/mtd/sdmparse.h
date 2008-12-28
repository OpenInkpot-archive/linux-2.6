/*
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
 *  $Id: sdmparse.h,v 1.4.16.2 2003/04/17 17:45:00 takuzo Exp $
 */

#ifndef _SDM_PARSE_H_
#define _SDM_PARSE_H_

typedef void *     sdmaddr_t;

#define SDM_IMGDESC_SIZE             512        /* total descriptor size      */
#define SDM_IMGNAME_LEN              16         /* image name length          */
#define SDM_IMGSTROPT_SIZE           256        /* image string option size   */

/* image descriptor */
struct sdm_image_desc {
        char       name[SDM_IMGNAME_LEN];
        u_int32_t  start;
        sdmaddr_t  _0;
        u_int32_t  sectors;
        sdmaddr_t  _1;
        u_int32_t  _2;
        u_int32_t  magic;
        u_int32_t  _3;
        u_int8_t   dev;
        u_int8_t   _rsvd[64-(SDM_IMGNAME_LEN 
                             + sizeof(sdmaddr_t) * 2 + sizeof(u_int32_t) * 6
                             + sizeof(u_int8_t) * 1)];
        u_int32_t  cksum;
        char       _4[SDM_IMGSTROPT_SIZE];
        u_int32_t  _5;
        int8_t     _6;
        u_int8_t   _7;
        char       _8[SDM_IMGNAME_LEN];
        u_int8_t   _rsvd2[SDM_IMGDESC_SIZE
                         - (SDM_IMGSTROPT_SIZE + 64 + sizeof(u_int32_t) 
			    + SDM_IMGNAME_LEN
                            + sizeof(u_int8_t) * 2)];
};

#define SDM_TABLE_DESC_OFFSET        SDM_IMGDESC_SIZE

/* sdm table */
struct sdm_table {
        u_int32_t  magic;
        u_int32_t  cksum;
        u_int32_t  max_descs;
        int32_t    ver_major;
        char       _1[SDM_IMGNAME_LEN];
        char       _2[SDM_IMGNAME_LEN];
        int32_t    ver_minor;
        u_int32_t  cap_flags;
	u_int32_t  rsv_table_sects;
	u_int16_t  color;
        u_int8_t   _rsvd[SDM_TABLE_DESC_OFFSET
                        -(SDM_IMGNAME_LEN * 2 + sizeof(u_int32_t) * 8 + sizeof(u_int16_t))];
	u_int32_t  table_cksum;
        struct sdm_image_desc desc[0];
};

#define SDM_TABLE_MAGIC              0x53446973 /* SDis */
#define SDM_TABLE_CKSUM_SEED         0x5344
#define SDM_VERSION2                 2

#define SDM_CAP_TABLE_MULTIPLEX      0x10       /* sdm table multiplexing */


/* error code */
#define SDM_OK                       0
#define SDM_ERR                      -1

#ifdef SDM_VERBOSE
#define SDM_INFO    printk
#else
#define SDM_INFO(arg...)
#endif

#endif /* _SDM_PARSE_H_ */
