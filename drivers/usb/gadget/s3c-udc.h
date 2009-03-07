/*
 * drivers/usb/gadget/s3c-udc.h
 * Samsung S3C on-chip full/high speed USB device controllers
 * $Id: s3c-udc.h,v 1.1 2007/08/10 07:00:56 dasan Exp $*
 * Copyright (C) 2005 for Samsung Electronics
 *              - by Jaswinder Singh Brar <jassi@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __S3C_USB_GADGET
#define __S3C_USB_GADGET


// Max packet size
#define EP0_FIFO_SIZE           64
#define EP_FIFO_SIZE            512
#define EP_FIFO_SIZE2           1024
#define S3C_MAX_ENDPOINTS       9

#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
#define DATA_STATE_RECV         4

/* ********************************************************************************************* */
/* IO
 */

typedef enum ep_type {
        ep_control, ep_bulk_in, ep_bulk_out, ep_interrupt
} ep_type_t;

struct s3c24xx_udc2_ep {
        struct usb_ep ep;
        struct s3c24xx_udc2 *dev;

        const struct usb_endpoint_descriptor *desc;
        struct list_head queue;
        unsigned long pio_irqs;
	unsigned int num;

        u8 stopped;
        u8 bEndpointAddress;

        void __iomem *fifo;
#ifdef CONFIG_USB_GADGET_S3C_FS
        u32 csr1;
        u32 csr2;
#endif
};

struct s3c24xx_udc2_request {
        struct usb_request req;
        struct list_head queue;
};

struct s3c24xx_udc2 {
        struct usb_gadget gadget;
        struct usb_gadget_driver *driver;
        struct platform_device *dev;
        spinlock_t lock;

        int ep0state;
        struct s3c24xx_udc2_ep ep[S3C_MAX_ENDPOINTS];

        unsigned char usb_address;
	u16 devstatus;

        unsigned req_pending:1, req_std:1;
};

extern struct s3c24xx_udc2 *the_controller;

#define ep_is_in(EP)            (((EP)->bEndpointAddress & USB_DIR_IN)==USB_DIR_IN)
#define ep_index(EP)            ((EP)->bEndpointAddress & 0xF)
#define ep_maxpacket(EP)        ((EP)->ep.maxpacket)

#endif

