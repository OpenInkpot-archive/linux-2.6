/*
 * drivers/usb/gadget/s3c-udc-hs.c
 * Samsung S3C on-chip full/high speed USB device controllers
 *
 * $Id: s3c-udc-hs.c,v 1.1 2007/08/10 07:00:56 dasan Exp $*
 *
 * Copyright (C) 2006 for Samsung Electronics
 *
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


//#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>

#include <mach/hardware.h>
#include <mach/regs-gpio.h>
#include <mach/regs-udc2.h>
#include <mach/regs-s3c2416-clock.h>
#include <mach/regs-irq.h>



#include "s3c-udc.h"

//#define DEBUG_S3C24XX_UDC2_SETUP
//#define DEBUG_S3C24XX_UDC2_EP0
//#define DEBUG_S3C24XX_UDC2

// USB Device DMA support
#define RX_DMA_MODE 0
#define TX_DMA_MODE 0
static int tx_dmaStart = 0;
static int rx_dmaStart = 0;

#ifdef DEBUG_S3C24XX_UDC2_SETUP

static char *state_names[] = {
	"WAIT_FOR_SETUP",
	"DATA_STATE_XMIT",
	"DATA_STATE_NEED_ZLP",
	"WAIT_FOR_OUT_STATUS",
	"DATA_STATE_RECV"
};
#define DEBUG_SETUP(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_SETUP(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C24XX_UDC2_EP0
#define DEBUG_EP0(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG_EP0(fmt,args...) do {} while(0)
#endif

#ifdef DEBUG_S3C24XX_UDC2
#define DPRINTK(fmt,args...) printk(fmt, ##args)
#else
#define DPRINTK(fmt,args...) do {} while(0)
#endif

#define DRIVER_DESC             "Samsung Dual-speed USB Device Controller"
#define DRIVER_VERSION          __DATE__

struct s3c24xx_udc2  *the_controller;
static struct clk		*udc_clock;
static struct clk		*usb_bus_clock;
static void __iomem		*base_addr;
static u64			rsrc_start;
static u64			rsrc_len;

static const char driver_name[] = "s3c-udc-hs";
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

// Max packet size
static u32 ep0_fifo_size = 64;
static u32 ep_fifo_size =  512;
static u32 ep_fifo_size2 = 1024;

/*
   Local declarations.
   */
static int s3c24xx_udc2_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *);
static int s3c24xx_udc2_ep_disable(struct usb_ep *ep);
static struct usb_request *s3c24xx_udc2_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
static void s3c24xx_udc2_free_request(struct usb_ep *ep, struct usb_request *);
static int s3c24xx_udc2_queue(struct usb_ep *ep, struct usb_request *, gfp_t gfp_flags);
static int s3c24xx_udc2_dequeue(struct usb_ep *ep, struct usb_request *);
static int s3c24xx_udc2_set_halt(struct usb_ep *ep, int);
static int s3c24xx_udc2_fifo_status(struct usb_ep *ep);
static void s3c24xx_udc2_fifo_flush(struct usb_ep *ep);
static void s3c24xx_udc2_ep0_kick(struct s3c24xx_udc2 *dev, struct s3c24xx_udc2_ep *ep);
static inline void s3c24xx_udc2_handle_ep0(struct s3c24xx_udc2 *dev);

static void done(struct s3c24xx_udc2_ep *ep, struct s3c24xx_udc2_request *req,
		int status);
static void stop_activity(struct s3c24xx_udc2 *dev,
		struct usb_gadget_driver *driver);
static int udc_enable(struct s3c24xx_udc2 *dev);
static void udc_set_address(struct s3c24xx_udc2 *dev, unsigned char address);
static void reconfig_usbd(void);

static inline void udc_writel(u32 value, unsigned long reg)
{
	writel(value, base_addr + reg);
}

static inline u32 udc_readl(unsigned long reg)
{
	return readl(base_addr + reg);
}

static inline u32 udc_epn_read(u32 port, u8 ind)
{
	unsigned long flags;
	u32 val;

	local_irq_save(flags);
	udc_writel(ind, S3C24XX_UDC2_INDEX_REG);
	val = udc_readl(port);
	local_irq_restore(flags);

	return val;
}

static inline void udc_epn_write(u32 val, u32 port, u8 ind)
{
	unsigned long flags;

	local_irq_save(flags);
	udc_writel(ind, S3C24XX_UDC2_INDEX_REG);
	udc_writel(val, port);
	local_irq_restore(flags);
}

static inline void udc_epn_set(u32 val, u32 port, u8 ind)
{
	unsigned long flags;

	local_irq_save(flags);
	udc_writel(ind, S3C24XX_UDC2_INDEX_REG);
	udc_writel(udc_readl(port) | val, port);
	local_irq_restore(flags);
}

static inline void udc_epn_clear(u32 val, u32 port, u8 ind)
{
	unsigned long flags;

	local_irq_save(flags);
	udc_writel(ind, S3C24XX_UDC2_INDEX_REG);
	udc_writel(udc_readl(port) & ~val, port);
	local_irq_restore(flags);
}

static inline void modify_io_reg(const void __iomem *reg, unsigned long clear, unsigned long set)
{
	unsigned long flags;
	unsigned long val;

	local_irq_save(flags);
	val = __raw_readl(reg);
	val &= ~clear;
	val |= set;
	__raw_writel(val, reg);
	local_irq_restore(flags);

}


static struct usb_ep_ops s3c24xx_udc2_ep_ops = {
	.enable = s3c24xx_udc2_ep_enable,
	.disable = s3c24xx_udc2_ep_disable,

	.alloc_request = s3c24xx_udc2_alloc_request,
	.free_request = s3c24xx_udc2_free_request,

	.queue = s3c24xx_udc2_queue,
	.dequeue = s3c24xx_udc2_dequeue,

	.set_halt = s3c24xx_udc2_set_halt,
	.fifo_status = s3c24xx_udc2_fifo_status,
	.fifo_flush = s3c24xx_udc2_fifo_flush,
};


/* Inline code */
static inline int write_packet(struct s3c24xx_udc2_ep *ep,
		struct s3c24xx_udc2_request *req, int max)
{
	u16 *buf;
	int length, count;
	void __iomem *fifo = ep->fifo;

	buf = req->req.buf + req->req.actual;
	prefetch(buf);

	length = req->req.length - req->req.actual;
	length = min(length, max);
	req->req.actual += length;

	DPRINTK("%s: Write %d (max %d), fifo=0x%x\n",
			__FUNCTION__, length, max, fifo);

	udc_epn_write(length, S3C24XX_UDC2_BYTE_WRITE_CNT_REG, ep_index(ep));


	for (count = 0; count < length; count += 2) {
		writel(*buf++, fifo);
	}

	return length;
}

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

static const char proc_node_name[] = "driver/udc";

	static int
udc_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	struct s3c24xx_udc2 *dev = _dev;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* basic device status */
	t = scnprintf(next, size,
			DRIVER_DESC "\n"
			"%s version: %s\n"
			"Gadget driver: %s\n"
			"\n",
			driver_name, DRIVER_VERSION,
			dev->driver ? dev->driver->driver.name : "(none)");
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev)
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else   /* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif  /* CONFIG_USB_GADGET_DEBUG_FILES */

/*
 *      udc_disable - disable USB device controller
 */
static void udc_disable(struct s3c24xx_udc2 *dev)
{
	DEBUG_SETUP("%s: %p\n", __FUNCTION__, dev);

	udc_set_address(dev, 0);

	dev->ep0state = WAIT_FOR_SETUP;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->usb_address = 0;
	modify_io_reg(S3C2416_PWRCFG, S3C2416_PWRCFG_nSW_PHY_OFF_USB, 0);
}

/*
 *      udc_reinit - initialize software state
 */
static void udc_reinit(struct s3c24xx_udc2 *dev)
{
	u32 i;

	DEBUG_SETUP("%s: %p\n", __FUNCTION__, dev);

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = WAIT_FOR_SETUP;

	/* basic endpoint records init */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c24xx_udc2_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = 0;
	}

	/* the rest was statically initialized, and is read-only */
}

#define BYTES2MAXP(x)   (x / 8)
#define MAXP2BYTES(x)   (x * 8)

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static int udc_enable(struct s3c24xx_udc2 *dev)
{
	DEBUG_SETUP("%s: %p\n", __FUNCTION__, dev);
	s3c2410_gpio_cfgpin(S3C2410_GPC4, S3C2410_GPC4_OUTP);
	s3c2410_gpio_cfgpin(S3C2410_GPD12, S3C2410_GPD12_OUTP);
	s3c2410_gpio_setpin(S3C2410_GPC4, 1);
	s3c2410_gpio_setpin(S3C2410_GPD12, 1);
	udelay(1000);
#warning FIXME	

#ifdef CONFIG_CPU_S3C2416
	/* if reset by sleep wakeup, control the retention I/O cell */
	if (__raw_readl(S3C2416_RSTSTAT) & S3C2416_RSTSAT_SLEEP)
		modify_io_reg(S3C2416_RSTCON, 0, S3C2416_RSTCON_PWROFF_SLP);


	/* USB Port is Normal mode */
	modify_io_reg(S3C2410_MISCCR, S3C2416_MISCCR_SEL_SUSPND, 0);

	/* PHY power enable */
	modify_io_reg(S3C2416_PWRCFG, 0, S3C2416_PWRCFG_nSW_PHY_OFF_USB);

	/* USB device 2.0 must reset like bellow,
	 * 1st phy reset and after at least 10us, func_reset & host reset
	 * phy reset can reset bellow registers.
	 */
	/* PHY 2.0 S/W reset */
	__raw_writel(S3C2416_USB_RSTCON_FUNC_RESET |
			S3C2416_USB_RSTCON_HOST_RESET |
			S3C2416_USB_RSTCON_PHY_RESET, S3C2416_USB_RSTCON);

	udelay(20); /* phy reset must be asserted for at 10us */

	/*Function 2.0, Host 1.1 S/W reset*/
	__raw_writel(S3C2416_USB_RSTCON_FUNC_RESET |
			S3C2416_USB_RSTCON_HOST_RESET, S3C2416_USB_RSTCON);
	__raw_writel(0, S3C2416_USB_RSTCON);

	/* 48Mhz,Oscillator,External X-tal,device */
	__raw_writel(S3C2416_USB_PHYCTRL_CLK_SEL_48MHZ |
//			S3C2416_USB_PHYCTRL_EXT_CLK |
			S3C2416_USB_PHYCTRL_INT_PLL_SEL, S3C2416_USB_PHYCTRL);

	/* 48Mhz clock on ,PHY2.0 analog block power on
	 * XO block power on,XO block power in suspend mode,
	 * PHY 2.0 Pll power on ,suspend signal for save mode disable
	 */
//	__raw_writel((1<<31)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0), S3C2416_USB_PHYPWR);
	__raw_writel(S3C2416_USB_PHYPWR_RESERVED_DEFAULTS, S3C2416_USB_PHYPWR);

	/* D+ pull up disable(VBUS detect), USB2.0 Function clock Enable,
	 * USB1.1 HOST disable, USB2.0 PHY test enable
	 */
	//        __raw_writel((0<<31)|(1<<2)|(0<<1)|(1<<0), S3C_UCLKCON);
	__raw_writel(S3C2416_USB_CLKCON_FUNC_CLK_EN |
			S3C2416_USB_CLKCON_HOST_CLK_EN*0, S3C2416_USB_CLKCON);

	//        __raw_writel(IRQ_USBD, S3C_INTPND);
	//        __raw_writel(IRQ_USBD, S3C_SRCPND);

	reconfig_usbd();

	//        __raw_writel(__raw_readl(S3C_INTMSK)&~(IRQ_USBD), S3C_INTMSK);
//	enable_irq(IRQ_USBD);

	/* D+ pull up , USB2.0 Function clock Enable,
	 * USB1.1 HOST disable,USB2.0 PHY test enable
	 */
	//        __raw_writel((1<<31)|(1<<2)|(0<<1)|(1<<0), S3C_UCLKCON);
	__raw_writel(S3C2416_USB_CLKCON_DETECT_VBUS |
			S3C2416_USB_CLKCON_FUNC_CLK_EN |
			S3C2416_USB_CLKCON_HOST_CLK_EN, S3C2416_USB_CLKCON);

#else
#error "only S3C2416 SoC is supported now"
#endif

	DEBUG_SETUP("S3C24XX USB 2.0 Controller Core Initialized\n");

	dev->gadget.speed = USB_SPEED_UNKNOWN;

	return 0;
}

/*
   Register entry point for the peripheral controller driver.
   */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct s3c24xx_udc2 *dev = the_controller;
	int retval;

	DEBUG_SETUP("%s: %s\n", __FUNCTION__, driver->driver.name);

	DEBUG_SETUP("%s: bind = %p, unbind = %p, disconnect = %p\n", __FUNCTION__, driver->bind, driver->unbind, driver->disconnect);
	if (!driver
			|| (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH)
			|| !driver->bind
			|| !driver->disconnect)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;
	/* Bind the driver */
	if ((retval = device_add(&dev->gadget.dev)) != 0) {
		printk(KERN_ERR "Error in device_add() : %d\n",retval);
		goto register_error;
	}

	retval = driver->bind(&dev->gadget);
	if (retval) {
		printk("%s: bind to driver %s --> error %d\n", dev->gadget.name,
				driver->driver.name, retval);
		device_del(&dev->gadget.dev);

		dev->driver = 0;
		dev->gadget.dev.driver = 0;
		return retval;
	}

	printk("Registered gadget driver '%s'\n", driver->driver.name);
	udc_enable(dev);
	return 0;

register_error:
	dev->driver = NULL;
	dev->gadget.dev.driver = NULL;
	return retval;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

/*
   Unregister entry point for the peripheral controller driver.
   */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct s3c24xx_udc2 *dev = the_controller;
	unsigned long flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);
	dev->driver = 0;
	stop_activity(dev, driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);
	device_del(&dev->gadget.dev);

//	disable_irq(IRQ_USBD);

	printk("Unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

/** Write request to FIFO (max write == maxp size)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int write_fifo(struct s3c24xx_udc2_ep *ep, struct s3c24xx_udc2_request *req)
{
	u32 max;
	unsigned count;
	int is_last = 0, is_short = 0;

#if TX_DMA_MODE
	if(tx_dmaStart)
		return 0;

	if ((req->req.actual == 0) && (req->req.length > ep->ep.maxpacket) && (!tx_dmaStart)) {

		DPRINTK("TX_DMA_Start:: %s: read %s, bytes req %p %d/%d\n",
				__FUNCTION__, ep->ep.name, req, req->req.actual, req->req.length);

		udc_epn_set(S3C24XX_UDC2_FIFO_CONTROL_DMAEN,
				S3C24XX_UDC2_FIFO_CONTROL_REG, ep_index(ep));

		udc_epn_write(ep->ep.maxpacket, S3C24XX_UDC2_MAX_PACKET_REG, ep_index(ep));
		udc_epn_write(ep->ep.maxpacket, S3C24XX_UDC2_BYTE_WRITE_CNT_REG, ep_index(ep));

		udc_epn_write((u16)req->req.length,
				S3C24XX_UDC2_DMA_TOTAL_CNT1_REG, ep_index(ep));

		udc_epn_write((u16)((req->req.length)>>16),
				S3C24XX_UDC2_DMA_TOTAL_CNT2_REG, ep_index(ep));

		udc_epn_write(virt_to_phys(req->req.buf),
				S3C24XX_UDC2_DMA_MEM_BASE_ADDR_REG, ep_index(ep));

		udc_epn_write(ep->ep.maxpacket,
				S3C24XX_UDC2_DMA_CNT_REG, ep_index(ep));

		udc_epn_write(ep->ep.maxpacket,
				S3C24XX_UDC2_DMA_FIFO_CNT_REG, ep_index(ep));

		udc_epn_set(S3C24XX_UDC2_MASTER_IF_CTL_MAX_BURST_INCR16,
				S3C24XX_UDC2_MASTER_IF_CONTROL_REG, ep_index(ep));

		udc_epn_set(S3C24XX_UDC2_DMA_CONTROL_FMDE |
				S3C24XX_UDC2_DMA_CONTROL_TDR |
				S3C24XX_UDC2_DMA_CONTROL_DEN,
				S3C24XX_UDC2_DMA_CONTROL_REG, ep_index(ep));
		tx_dmaStart = 1;

		udelay(600);

	}
	else {
#endif
		max = le16_to_cpu(ep->desc->wMaxPacketSize);
		count = write_packet(ep, req, max);

		/* last packet is usually short (or a zlp) */
		if (unlikely(count != max))
			is_last = is_short = 1;
		else {
			if (likely(req->req.length != req->req.actual)
					|| req->req.zero)
				is_last = 0;
			else
				is_last = 1;
			/* interrupt/iso maxpacket may not fill the fifo */
			is_short = unlikely(max < ep_maxpacket(ep));
		}

		DPRINTK("%s: wrote %s %d bytes%s%s req %p %d/%d\n", __FUNCTION__,
				ep->ep.name, count,
				is_last ? "/L" : "", is_short ? "/S" : "",
				req, req->req.actual, req->req.length);

#if TX_DMA_MODE
	}
#endif

	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		if(!ep_index(ep)){
			DPRINTK("%s: --> Error EP0 must not come here!\n",
					__FUNCTION__);
			BUG();
		}
		done(ep, req, 0);
		return 1;
	}

	return 0;
}



/** Read to request from FIFO (max read == bytes in fifo)
 *  Return:  0 = still running, 1 = completed, negative = errno
 */
static int read_fifo(struct s3c24xx_udc2_ep *ep, struct s3c24xx_udc2_request *req)
{
	u32 csr;
	u16 *buf;
	unsigned bufferspace, count, count_bytes, is_short = 0;
	void __iomem *fifo = ep->fifo;

	csr = udc_epn_read( S3C24XX_UDC2_EP_STATUS_REG, ep_index(ep));

	/* make sure there's a packet in the FIFO. */
	if (!(csr & S3C24XX_UDC2_EP_STATUS_RPS)) {
		DPRINTK("%s: Packet NOT ready!\n", __FUNCTION__);
		return -EINVAL;
	}

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	count = udc_epn_read(S3C24XX_UDC2_BYTE_READ_CNT_REG, ep_index(ep));
	if (csr & S3C24XX_UDC2_EP_STATUS_LWO)
		count_bytes = count * 2 -1;
	else
		count_bytes = count * 2;


#if RX_DMA_MODE
	if(rx_dmaStart) return 0;

	if ((req->req.actual == 0) && (req->req.length > ep->ep.maxpacket) && (!rx_dmaStart)) {

		DPRINTK("RX_DMA_Start :: %s: read %s, %d bytes req %p %d/%d CSR::0x%x\n",
				__FUNCTION__, ep->ep.name, count_bytes, req, req->req.actual, req->req.length, csr);


		udc_epn_set(S3C24XX_UDC2_FIFO_CONTROL_DMAEN,
				S3C24XX_UDC2_FIFO_CONTROL_REG, ep_index(ep));

		udc_epn_write((u16)req->req.length,
				S3C24XX_UDC2_DMA_TOTAL_CNT1_REG, ep_index(ep));

		udc_epn_write((u16)((req->req.length)>>16),
				S3C24XX_UDC2_DMA_TOTAL_CNT2_REG, ep_index(ep));

		udc_epn_write(ep->ep.maxpacket,
				S3C24XX_UDC2_DMA_CNT_REG, ep_index(ep));
		//udc_epn_set(512,
		//S3C24XX_UDC2_DMA_FIFO_CNT_REG, ep_index(ep));

		udc_epn_set(S3C24XX_UDC2_MASTER_IF_CTL_MAX_BURST_INCR16,
				S3C24XX_UDC2_MASTER_IF_CONTROL_REG, ep_index(ep));

		udc_epn_write(virt_to_phys(buf),
				S3C24XX_UDC2_DMA_MEM_BASE_ADDR_REG, ep_index(ep));

		udc_epn_set(S3C24XX_UDC2_DMA_CONTROL_FMDE | S3C24XX_UDC2_DMA_CONTROL_RDR | S3C24XX_UDC2_DMA_CONTROL_DEN,
				S3C24XX_UDC2_DMA_CONTROL_REG, ep_index(ep));
		rx_dmaStart = 1;


	}
	else {
#endif
		req->req.actual += min(count_bytes, bufferspace);

		is_short = (count_bytes < ep->ep.maxpacket);
		DPRINTK("%s: read %s, %d bytes%s req %p %d/%d\n",
				__FUNCTION__,
				ep->ep.name, count_bytes,
				is_short ? "/S" : "", req, req->req.actual, req->req.length);

		while (likely(count-- != 0)) {
			u16 byte = (u16) readl(fifo);

			if (unlikely(bufferspace == 0)) {
				/* this happens when the driver's buffer
				 * is smaller than what the host sent.
				 * discard the extra data.
				 */
				if (req->req.status != -EOVERFLOW)
					printk("%s overflow %d\n", ep->ep.name, count);
				req->req.status = -EOVERFLOW;
			} else {
				*buf++ = byte;
				bufferspace--;
			}
		}

#if RX_DMA_MODE
	}
#endif

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/*
 *      done - retire a request; caller blocked irqs
 */
static void done(struct s3c24xx_udc2_ep *ep, struct s3c24xx_udc2_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	DPRINTK("%s: %p\n", __FUNCTION__, ep);

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		printk("complete %s req %p stat %d len %u/%u\n",
				ep->ep.name, &req->req, status,
				req->req.actual, req->req.length);

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);

	ep->stopped = stopped;
}

/*
 *      nuke - dequeue ALL requests
 */
void nuke(struct s3c24xx_udc2_ep *ep, int status)
{
	struct s3c24xx_udc2_request *req;

	DPRINTK("%s: %p\n", __FUNCTION__, ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct s3c24xx_udc2_request, queue);
		done(ep, req, status);
	}
}

/**
 * s3c24xx_udc2_in_epn - handle IN interrupt
 */
static void s3c24xx_udc2_in_epn(struct s3c24xx_udc2 *dev, u32 ep_idx)
{
	u32 csr;
	struct s3c24xx_udc2_ep *ep = &dev->ep[ep_idx];
	struct s3c24xx_udc2_request *req;

	csr = udc_epn_read(S3C24XX_UDC2_EP_STATUS_REG, ep_idx);
	DPRINTK("%s: S3C24XX_UDC2_EP%d_STATUS_REG=0x%x\n", __FUNCTION__, ep_idx, csr);

	if (csr & S3C24XX_UDC2_EP_STATUS_FFS) {
		DPRINTK("%s: S3C24XX_UDC2_EP_FFS\n", __FUNCTION__);
		udc_epn_set(S3C24XX_UDC2_EP_STATUS_FFS, S3C24XX_UDC2_EP_STATUS_REG, ep_idx);
	}

	if (csr & S3C24XX_UDC2_EP_STATUS_FSC) {
		DPRINTK("%s: S3C24XX_UDC2_EP_STALL\n", __FUNCTION__);
		udc_epn_set(S3C24XX_UDC2_EP_STATUS_FSC, S3C24XX_UDC2_EP_STATUS_REG, ep_idx);
		return;
	}

	if (!ep->desc) {
		DPRINTK("%s: NO EP DESC\n", __FUNCTION__);
		return;
	}


	if (csr & S3C24XX_UDC2_EP_STATUS_DTCZ) {
		DPRINTK("%s: TX_DMA :: DMA Total Count Zero:: S3C24XX_UDC2_EP%d_STATUS_REG=0x%x\n",
				__FUNCTION__, ep_index(ep), csr);

		tx_dmaStart = 0;
		udc_epn_clear(S3C24XX_UDC2_FIFO_CONTROL_DMAEN,
				S3C24XX_UDC2_FIFO_CONTROL_REG, ep_idx);

		udc_epn_clear(S3C24XX_UDC2_DMA_CONTROL_TDR | S3C24XX_UDC2_DMA_CONTROL_DEN,
				S3C24XX_UDC2_DMA_CONTROL_REG, ep_idx);

		udc_epn_set(S3C24XX_UDC2_EP_STATUS_DTCZ, S3C24XX_UDC2_EP_STATUS_REG, ep_idx);


		if (list_empty(&ep->queue))
			req = 0;
		else
			req = list_entry(ep->queue.next,
					struct s3c24xx_udc2_request, queue);
		if (unlikely(!req)) {
			DPRINTK("%s: TX_DMA :: NULL REQ\n", __FUNCTION__);
			//return;
		} else {
			DPRINTK("%s: TX_DMA_DONE - REQ is %p (%d/%d) CSR:0x%x\n", __FUNCTION__, req, req->req.actual, req->req.length, csr);
			req->req.actual = req->req.length;
			done(ep, req, 0);
		}

	}


	if (csr & S3C24XX_UDC2_EP_STATUS_TPS) {
		udc_epn_set(S3C24XX_UDC2_EP_STATUS_TPS,
				S3C24XX_UDC2_EP_STATUS_REG, ep_idx);

		if (list_empty(&ep->queue))
			req = NULL;
		else
			req = list_entry(ep->queue.next, struct s3c24xx_udc2_request, queue);

		if (unlikely(!req)) {
			DPRINTK("%s:NULL REQ :: EP_TX_SUCCESS, req = %p CSR:0x%x\n", __FUNCTION__, req, csr);
			return;
		}
		else {
			DPRINTK("%s: EP_TX_SUCCESS, req = %p CSR:0x%x\n", __FUNCTION__, req, csr);
			if ((write_fifo(ep, req)==0) && (csr & S3C24XX_UDC2_EP_STATUS_PSIF_TWO))
				write_fifo(ep, req);

		}
	}

}

/* ********************************************************************************************* */
/* Bulk OUT (recv)
*/

static void s3c24xx_udc2_out_epn(struct s3c24xx_udc2 *dev, u32 ep_idx)
{
	struct s3c24xx_udc2_ep *ep = &dev->ep[ep_idx];
	struct s3c24xx_udc2_request *req;
	u32 csr;

	csr=udc_epn_read(S3C24XX_UDC2_EP_STATUS_REG, ep_index(ep));
	DPRINTK("%s: S3C24XX_UDC2_EP%d_STATUS_REG=0x%x\n",
			__FUNCTION__, ep_index(ep), csr);

	if (unlikely(!(ep->desc))) {
		/* Throw packet away.. */
		printk("%s: No descriptor?!?\n", __FUNCTION__);
		return;
	}

	if (csr & S3C24XX_UDC2_EP_STATUS_FSC) {
		DPRINTK("%s: stall sent\n", __FUNCTION__);
		udc_epn_set(S3C24XX_UDC2_EP_STATUS_FSC, S3C24XX_UDC2_EP_STATUS_REG, ep_idx);
		return;
	}

	if (csr & S3C24XX_UDC2_EP_STATUS_FFS) {
		DPRINTK("%s: fifo flush \n", __FUNCTION__);
		udc_epn_set(S3C24XX_UDC2_EP_STATUS_FFS, S3C24XX_UDC2_EP_STATUS_REG, ep_idx);
		return;
	}


	if (csr & S3C24XX_UDC2_EP_STATUS_DTCZ) {
		DPRINTK("%s: DMA Total Count Zero:: S3C24XX_UDC2_EP%d_STATUS_REG=0x%x\n",
				__FUNCTION__, ep_index(ep), csr);

		rx_dmaStart = 0;
		udc_epn_clear(S3C24XX_UDC2_FIFO_CONTROL_DMAEN,
				S3C24XX_UDC2_FIFO_CONTROL_REG, ep_idx);

		udc_epn_clear(S3C24XX_UDC2_DMA_CONTROL_RDR | S3C24XX_UDC2_DMA_CONTROL_DEN,
				S3C24XX_UDC2_DMA_CONTROL_REG, ep_idx);

		udc_epn_set(S3C24XX_UDC2_EP_STATUS_DTCZ, S3C24XX_UDC2_EP_STATUS_REG, ep_idx);

		if (list_empty(&ep->queue))
			req = 0;
		else
			req = list_entry(ep->queue.next,
					struct s3c24xx_udc2_request, queue);
		if (unlikely(!req)) {
			DPRINTK("RX_DMA :: %s: NULL REQ\n", __FUNCTION__);
		} else {
			DPRINTK("%s : RX_DMA_DONE - REQ : %p\n", __FUNCTION__, req);
			req->req.actual = req->req.length;
			done(ep, req, 0);
		}
	}


	if (csr & S3C24XX_UDC2_EP_STATUS_RPS) {
		if(!rx_dmaStart) {
			if (list_empty(&ep->queue))
				req = 0;
			else
				req = list_entry(ep->queue.next,
						struct s3c24xx_udc2_request, queue);

			if (unlikely(!req)) {
				DPRINTK("%s: NULL REQ on ISR %d\n", __FUNCTION__, ep_idx);
				return;
			} else {
				if(((read_fifo(ep, req))==0) && (csr & S3C24XX_UDC2_EP_STATUS_PSIF_TWO))
					read_fifo(ep, req);
			}
		}
	}

}

static void stop_activity(struct s3c24xx_udc2 *dev,
		struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c24xx_udc2_ep *ep = &dev->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

static void reconfig_usbd(void)
{
	/* EP0 Interrupt enable */
	udc_writel(0x1, S3C24XX_UDC2_EP_INT_EN_REG);

	udc_writel(0x0000, S3C24XX_UDC2_TEST_REG);

	/* error interrupt enable, 16bit bus, Little format,
	 * suspend&reset enable
	 */
	udc_writel(S3C24XX_UDC2_SYS_CONTROL_DTZIEN |
			S3C24XX_UDC2_SYS_CONTROL_HSUSPEND | S3C24XX_UDC2_SYS_CONTROL_HRESET |
			S3C24XX_UDC2_SYS_CONTROL_EIE | (1<<5),
			S3C24XX_UDC2_SYS_CONTROL_REG);

	udc_writel(0x0000, S3C24XX_UDC2_EP0_CONTROL_REG);

	/* EP1, EP2 dual FIFO mode enable */
	udc_epn_write(S3C24XX_UDC2_EP_CONTROL_DUEN, S3C24XX_UDC2_EP_CONTROL_REG, 1);
	udc_epn_write(S3C24XX_UDC2_EP_CONTROL_DUEN, S3C24XX_UDC2_EP_CONTROL_REG, 2);

	udc_writel(0, S3C24XX_UDC2_INDEX_REG);

}

void set_max_pktsize(struct s3c24xx_udc2 *dev, enum usb_device_speed speed)
{
	if (speed == USB_SPEED_HIGH)
	{
		ep0_fifo_size = 64;
		ep_fifo_size = 512;
		ep_fifo_size2 = 1024;
		dev->gadget.speed = USB_SPEED_HIGH;
	}
	else
	{
		ep0_fifo_size = 64;
		ep_fifo_size = 64;
		ep_fifo_size2 = 64;
		dev->gadget.speed = USB_SPEED_FULL;
	}

	dev->ep[0].ep.maxpacket = ep0_fifo_size;
	dev->ep[1].ep.maxpacket = ep_fifo_size;
	dev->ep[2].ep.maxpacket = ep_fifo_size;
	dev->ep[3].ep.maxpacket = ep_fifo_size;
	dev->ep[4].ep.maxpacket = ep_fifo_size;
	dev->ep[5].ep.maxpacket = ep_fifo_size2;
	dev->ep[6].ep.maxpacket = ep_fifo_size2;
	dev->ep[7].ep.maxpacket = ep_fifo_size2;
	dev->ep[8].ep.maxpacket = ep_fifo_size2;

	udc_epn_write(ep0_fifo_size, S3C24XX_UDC2_MAX_PACKET_REG, 0);
	udc_epn_write(ep_fifo_size, S3C24XX_UDC2_MAX_PACKET_REG, 1);
	udc_epn_write(ep_fifo_size, S3C24XX_UDC2_MAX_PACKET_REG, 2);
	udc_epn_write(ep_fifo_size, S3C24XX_UDC2_MAX_PACKET_REG, 3);
	udc_epn_write(ep_fifo_size, S3C24XX_UDC2_MAX_PACKET_REG, 4);
	udc_epn_write(ep_fifo_size2, S3C24XX_UDC2_MAX_PACKET_REG, 5);
	udc_epn_write(ep_fifo_size2, S3C24XX_UDC2_MAX_PACKET_REG, 6);
	udc_epn_write(ep_fifo_size2, S3C24XX_UDC2_MAX_PACKET_REG, 7);
	udc_epn_write(ep_fifo_size2, S3C24XX_UDC2_MAX_PACKET_REG, 8);

}

/*
 *      elfin usb client interrupt handler.
 */
static irqreturn_t s3c24xx_udc2_irq(int irq, void *_dev)
{
	struct s3c24xx_udc2 *dev = _dev;
	u32 intr_ep_status;
	register u32 intr_status, intr_status_chk;
	int i;
	register u32 csr;

	spin_lock(&dev->lock);


	intr_status = udc_readl(S3C24XX_UDC2_SYS_STATUS_REG);
	intr_ep_status = udc_readl(S3C24XX_UDC2_EP_INT_REG);

	DPRINTK("\n\n%s: S3C24XX_UDC2_EP_INT_REG=0x%x, S3C24XX_UDC2_SYS_STATUS_REG=0x%x (on state %s)\n",
			__FUNCTION__, intr_ep_status, intr_status, state_names[dev->ep0state]);

	csr = udc_readl(S3C24XX_UDC2_EP0_STATUS_REG);

	intr_status_chk = intr_status & S3C24XX_UDC2_SYS_STATUS_INT_CHECK;

	if (!intr_ep_status && !intr_status_chk){
		spin_unlock(&dev->lock);
		return IRQ_HANDLED;
	}

	if (intr_status_chk)  {
		if (intr_status & S3C24XX_UDC2_SYS_STATUS_VBUSON) {
			DEBUG_SETUP("%s: VBUSON interrupt\n", __FUNCTION__);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_VBUSON, S3C24XX_UDC2_SYS_STATUS_REG);
		}

		if (intr_status & S3C24XX_UDC2_SYS_STATUS_VBUSOFF) {
			DEBUG_SETUP("%s: VBUSON interrupt\n", __FUNCTION__);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_VBUSOFF, S3C24XX_UDC2_SYS_STATUS_REG);
		}


		if (intr_status & S3C24XX_UDC2_SYS_STATUS_ERR_ALL) {
			DEBUG_SETUP("%s: Error interrupt: STATUS=%08x\n", __FUNCTION__, intr_status);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_ERR_ALL, S3C24XX_UDC2_SYS_STATUS_REG);
		}

		if (intr_status & S3C24XX_UDC2_SYS_STATUS_SDE) {
			DEBUG_SETUP("%s: Speed Detection interrupt\n",
					__FUNCTION__);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_SDE, S3C24XX_UDC2_SYS_STATUS_REG);

			if (intr_status & S3C24XX_UDC2_SYS_STATUS_HSP) {
				DEBUG_SETUP("%s: High Speed Detection\n",
						__FUNCTION__);
				set_max_pktsize(dev, USB_SPEED_HIGH);
			}
			else {
				DEBUG_SETUP("%s: Full Speed Detection\n",
						__FUNCTION__);
				set_max_pktsize(dev, USB_SPEED_FULL);
			}
		}

		if (intr_status & S3C24XX_UDC2_SYS_STATUS_HFSUSP) {
			DEBUG_SETUP("%s: SUSPEND interrupt\n", __FUNCTION__);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_HFSUSP, S3C24XX_UDC2_SYS_STATUS_REG);
			if (dev->gadget.speed != USB_SPEED_UNKNOWN
					&& dev->driver
					&& dev->driver->suspend) {
				dev->driver->suspend(&dev->gadget);
			}
		}

		if (intr_status & S3C24XX_UDC2_SYS_STATUS_HFRM) {
			DEBUG_SETUP("%s: RESUME interrupt\n", __FUNCTION__);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_HFRM, S3C24XX_UDC2_SYS_STATUS_REG);
			if (dev->gadget.speed != USB_SPEED_UNKNOWN
					&& dev->driver
					&& dev->driver->resume) {
				dev->driver->resume(&dev->gadget);
			}
		}

		if (intr_status & S3C24XX_UDC2_SYS_STATUS_HFRES) {
			DEBUG_SETUP("%s: RESET interrupt\n", __FUNCTION__);
			udc_writel(S3C24XX_UDC2_SYS_STATUS_HFRES, S3C24XX_UDC2_SYS_STATUS_REG);
			reconfig_usbd();
			dev->ep0state = WAIT_FOR_SETUP;
		}
	}

	if (intr_ep_status & S3C24XX_UDC2_EP_INT_EP0){
		udc_writel(S3C24XX_UDC2_EP_INT_EP0, S3C24XX_UDC2_EP_INT_REG);
		s3c24xx_udc2_handle_ep0(dev);
	}

	/* endpoint data transfers */
	for (i = 1; i < 8; i++) {
		u32 tmp = 1 << i;
		if (intr_ep_status & tmp) {
			DPRINTK("USB ep%d irq\n", i);
			/* Clear the interrupt bit by setting it to 1 */
			udc_writel(tmp, S3C24XX_UDC2_EP_INT_REG);
			if (dev->ep[i].bEndpointAddress & USB_DIR_IN)
				s3c24xx_udc2_in_epn(dev, i);
			else
				s3c24xx_udc2_out_epn(dev, i);

		}
	}


	intr_status = udc_readl(S3C24XX_UDC2_SYS_STATUS_REG);
	intr_ep_status = udc_readl(S3C24XX_UDC2_EP_INT_REG);

	csr = udc_readl(S3C24XX_UDC2_EP0_STATUS_REG);


	spin_unlock(&dev->lock);

	return IRQ_HANDLED;
}

static int s3c24xx_udc2_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct s3c24xx_udc2_ep *ep;
	struct s3c24xx_udc2 *dev;
	unsigned long flags;
	u32 int_en_reg;

	DEBUG_SETUP("%s: %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct s3c24xx_udc2_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| (ep->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK) != (desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)
			|| ep_maxpacket(ep) < le16_to_cpu(desc->wMaxPacketSize)) {
		printk("%s: bad ep or descriptor\n", __FUNCTION__);
		printk("%s: ep->desc = %p, _ep->name = %s, desc->bDescriptorType = 0x%02x, ep->bEndpointAddress = 0x%02x, desc->bEndpointAddress = 0x%02x, ep_maxpacket(ep) = %u\n",
				__FUNCTION__, ep->desc, _ep->name, desc->bDescriptorType, ep->bEndpointAddress, desc->bEndpointAddress, ep_maxpacket(ep));
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
				&& le16_to_cpu(desc->wMaxPacketSize) != ep_maxpacket(ep))
			|| !desc->wMaxPacketSize) {
		DPRINTK("%s: bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DPRINTK("%s: bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	ep->bEndpointAddress = desc->bEndpointAddress;

	/* Reset halt state */
	s3c24xx_udc2_set_halt(_ep, 0);

	if (desc->bEndpointAddress & USB_DIR_IN) {
		udc_writel(udc_readl(S3C24XX_UDC2_EP_DIR_REG) | (1 << ep->num), S3C24XX_UDC2_EP_DIR_REG);
		udc_epn_write(S3C24XX_UDC2_EP_CONTROL_DUEN, S3C24XX_UDC2_EP_CONTROL_REG, ep->num);

	} else {
		udc_writel(udc_readl(S3C24XX_UDC2_EP_DIR_REG) & ~(1 << ep->num), S3C24XX_UDC2_EP_DIR_REG);
		udc_epn_write(S3C24XX_UDC2_EP_CONTROL_DUEN, S3C24XX_UDC2_EP_CONTROL_REG, ep->num);
	}

	if (desc->bmAttributes == USB_ENDPOINT_XFER_INT)
		udc_epn_set(S3C24XX_UDC2_EP_CONTROL_IEMS, S3C24XX_UDC2_EP_CONTROL_REG, ep->num);
	else
		udc_epn_clear(S3C24XX_UDC2_EP_CONTROL_IEMS, S3C24XX_UDC2_EP_CONTROL_REG, ep->num);

	int_en_reg = udc_readl(S3C24XX_UDC2_EP_INT_EN_REG);
	udc_writel(int_en_reg | (1 << ep->num), S3C24XX_UDC2_EP_INT_EN_REG);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DPRINTK("%s: enabled %s, S3C24XX_UDC2_EP_INT_EN_REG = %08x, S3C24XX_UDC2_EP_DIR_REG = %08x\n", __FUNCTION__, _ep->name, udc_readl(S3C24XX_UDC2_EP_INT_EN_REG), udc_readl(S3C24XX_UDC2_EP_DIR_REG));
	return 0;
}

/** Disable EP
*/
static int s3c24xx_udc2_ep_disable(struct usb_ep *_ep)
{
	struct s3c24xx_udc2_ep *ep;
	unsigned long flags;
	u32 int_en_reg;

	DPRINTK("%s: %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct s3c24xx_udc2_ep, ep);
	if (!_ep || !ep->desc) {
		DPRINTK("%s: %s not enabled\n", __FUNCTION__,
				_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Nuke all pending requests */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	int_en_reg = udc_readl(S3C24XX_UDC2_EP_INT_EN_REG);
	udc_writel(int_en_reg & ~(1 << ep->num), S3C24XX_UDC2_EP_INT_EN_REG);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DPRINTK("%s: disabled %s\n", __FUNCTION__, _ep->name);
	return 0;
}

static struct usb_request *s3c24xx_udc2_alloc_request(struct usb_ep *ep,
		gfp_t gfp_flags)
{
	struct s3c24xx_udc2_request *req;

	DPRINTK("%s: %p\n", __FUNCTION__, ep);

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void s3c24xx_udc2_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct s3c24xx_udc2_request *req;

	DPRINTK("%s: %p\n", __FUNCTION__, ep);

	req = container_of(_req, struct s3c24xx_udc2_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/** Queue one request
 *  Kickstart transfer if needed
 */
static int s3c24xx_udc2_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t gfp_flags)
{
	struct s3c24xx_udc2_request *req;
	struct s3c24xx_udc2_ep *ep;
	struct s3c24xx_udc2 *dev;
	unsigned long flags;

	DPRINTK("%s: %p\n", __FUNCTION__, _ep);

	req = container_of(_req, struct s3c24xx_udc2_request, req);
	if (unlikely
			(!_req || !_req->complete || !_req->buf
			 || !list_empty(&req->queue))) {
		DPRINTK("%s: bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct s3c24xx_udc2_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DPRINTK("%s: bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely(!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		DPRINTK("%s: bogus device state %p\n", __FUNCTION__, dev->driver);
		return -ESHUTDOWN;
	}

	DPRINTK("%s: %s queue req %p, len %d buf %p\n",
			__FUNCTION__, _ep->name, _req, _req->length, _req->buf);

	spin_lock_irqsave(&dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	DPRINTK("%s: Add to ep=%d, Q empty=%d, stopped=%d\n",
			__FUNCTION__, ep_index(ep), list_empty(&ep->queue), ep->stopped);
	if (list_empty(&ep->queue) && likely(!ep->stopped)) {
		u32 csr;

		if (unlikely(ep_index(ep) == 0)) {
			/* EP0 */
			list_add_tail(&req->queue, &ep->queue);
			s3c24xx_udc2_ep0_kick(dev, ep);
			req = NULL;

		} else if (ep_is_in(ep)) {
			csr = udc_epn_read(S3C24XX_UDC2_EP_STATUS_REG, ep_index(ep));
			DPRINTK("%s: ep_is_in, S3C24XX_UDC2_EP%d_STATUS_REG=0x%x\n",
					__FUNCTION__, ep_index(ep), csr);

			if(!(csr & S3C24XX_UDC2_EP_STATUS_TPS)
					&& (write_fifo(ep, req) == 1))
				req = NULL;
			else
				DPRINTK("IN-list_add_taill :: req =%p\n", req);

		} else {
			csr = udc_epn_read(S3C24XX_UDC2_EP_STATUS_REG, ep_index(ep));
			DPRINTK("%s: ep_is_out, S3C24XX_UDC2_EP%d_STATUS_REG=0x%x\n",
					__FUNCTION__, ep_index(ep),csr);

			if((csr & S3C24XX_UDC2_EP_STATUS_RPS)
					&& (read_fifo(ep, req) == 1))
				req = NULL;
			else
				DPRINTK("OUT-list_add_taill :: req =%p\n", req);
		}
	}
	/* pio or dma irq handler advances the queue. */
	if (likely(req != NULL))
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}

/* dequeue JUST ONE request */
static int s3c24xx_udc2_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct s3c24xx_udc2_ep *ep;
	struct s3c24xx_udc2_request *req;
	unsigned long flags;

	DPRINTK("%s: %p\n", __FUNCTION__, _ep);

	ep = container_of(_ep, struct s3c24xx_udc2_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/** Halt specific EP
 *  Return 0 if success
 */
static int s3c24xx_udc2_set_halt(struct usb_ep *_ep, int value)
{
	return 0;
}

/** Return bytes in EP FIFO
*/
static int s3c24xx_udc2_fifo_status(struct usb_ep *_ep)
{
	u32 csr;
	int count = 0;
	struct s3c24xx_udc2_ep *ep;

	ep = container_of(_ep, struct s3c24xx_udc2_ep, ep);
	if (!_ep) {
		DPRINTK("%s: bad ep\n", __FUNCTION__);
		return -ENODEV;
	}

	DPRINTK("%s: %d\n", __FUNCTION__, ep_index(ep));

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;

	csr = udc_epn_read(S3C24XX_UDC2_EP_STATUS_REG, ep_index(ep));
	if (ep->dev->gadget.speed != USB_SPEED_UNKNOWN ||
			csr & S3C24XX_UDC2_EP_STATUS_RPS) {

		count = udc_epn_read(S3C24XX_UDC2_BYTE_READ_CNT_REG, ep_index(ep));

		if (udc_epn_read(S3C24XX_UDC2_EP_STATUS_REG, ep_index(ep))
				& S3C24XX_UDC2_EP_STATUS_LWO)
			count = count * 2 -1;
		else
			count = count * 2;
	}

	return count;
}

/** Flush EP FIFO
*/
static void s3c24xx_udc2_fifo_flush(struct usb_ep *_ep)
{
	struct s3c24xx_udc2_ep *ep;

	ep = container_of(_ep, struct s3c24xx_udc2_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DPRINTK("%s: bad ep\n", __FUNCTION__);
		return;
	}

	DPRINTK("%s: %d\n", __FUNCTION__, ep_index(ep));
}

/****************************************************************/
/* End Point 0 related functions                                */
/****************************************************************/

/* return:  0 = still running, 1 = completed, negative = errno */
static int write_fifo_ep0(struct s3c24xx_udc2_ep *ep, struct s3c24xx_udc2_request *req)
{
	u32 max;
	unsigned count;
	int is_last;

	max = ep_maxpacket(ep);

	DEBUG_EP0("%s: max = %d\n", __FUNCTION__, max);

	count = write_packet(ep, req, max);

	/* last packet is usually short (or a zlp) */
	if (unlikely(count != max))
		is_last = 1;
	else {
		if (likely(req->req.length != req->req.actual) || req->req.zero)
			is_last = 0;
		else
			is_last = 1;
	}

	DEBUG_EP0("%s: wrote %s %d bytes%s %d left %p\n", __FUNCTION__,
			ep->ep.name, count,
			is_last ? "/L" : "", req->req.length - req->req.actual, req);

	/* requests complete when all IN data is in the FIFO */
	if (is_last) {
		return 1;
	}

	return 0;
}

static inline int s3c24xx_udc2_fifo_read(struct s3c24xx_udc2_ep *ep, u16 *cp, int max)
{
	int bytes;
	int count;
	int i;
	void __iomem *fifo = ep->fifo;

	count = udc_epn_read(S3C24XX_UDC2_BYTE_READ_CNT_REG, ep_index(ep));
	DEBUG_EP0("%s: count=%d, ep_index=%d, fifo=0x%x\n",
			__FUNCTION__, count, ep_index(ep), ep->fifo);
	bytes = count * 2;

	while (count--) {
		*cp++ = (u16) readl(fifo);
	}
	udc_writel(S3C24XX_UDC2_EP0_STATUS_RSR, S3C24XX_UDC2_EP0_STATUS_REG);/* clear */

	return bytes;
}

static int read_fifo_ep0(struct s3c24xx_udc2_ep *ep, struct s3c24xx_udc2_request *req)
{
	u32 csr;
	u16 *buf;
	unsigned bufferspace, count, is_short, bytes;
	void __iomem *fifo = ep->fifo;

	DEBUG_EP0("%s\n", __FUNCTION__);

	csr = udc_readl(S3C24XX_UDC2_EP0_STATUS_REG);
	if (!(csr & S3C24XX_UDC2_EP0_STATUS_RSR))
		return 0;

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);
	bufferspace = req->req.length - req->req.actual;

	/* read all bytes from this packet */
	if (likely(csr & S3C24XX_UDC2_EP0_STATUS_RSR)) {
		count = udc_epn_read(S3C24XX_UDC2_BYTE_READ_CNT_REG, ep_index(ep));
		if (csr & S3C24XX_UDC2_EP0_STATUS_LWO)
			bytes = count * 2 -1;
		else
			bytes = count * 2;

		req->req.actual += min(bytes, bufferspace);
	} else  {               // zlp
		count = 0;
		bytes = 0;
	}

	is_short = (bytes < ep->ep.maxpacket);
	DEBUG_EP0("%s: read %s %02x, %d bytes%s req %p %d/%d\n",
			__FUNCTION__,
			ep->ep.name, csr, bytes,
			is_short ? "/S" : "", req, req->req.actual, req->req.length);

	while (likely(count-- != 0)) {
		u16 byte = (u16) readl(fifo);

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DEBUG_EP0("%s overflow %d\n", ep->ep.name,
						count);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = byte;
			bufferspace = bufferspace - 2;
		}
	}

	udc_writel(S3C24XX_UDC2_EP0_STATUS_RSR, S3C24XX_UDC2_EP0_STATUS_REG);/* clear */

	/* completion */
	if (is_short || req->req.actual == req->req.length) {
		return 1;
	}

	return 0;
}

/**
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function
 * after it decodes a set address setup packet.
 */
static void udc_set_address(struct s3c24xx_udc2 *dev, unsigned char address)
{
	DEBUG_EP0("%s: address=%d, S3C24XX_UDC2_FUNC_ADDR_REG=0x%x\n",
			__FUNCTION__, address, udc_readl(S3C24XX_UDC2_FUNC_ADDR_REG));
	dev->usb_address = address;
}

/*
 * DATA_STATE_RECV (OUT_PKT_RDY)
 */
static int first_time = 1;

static void s3c24xx_udc2_ep0_read(struct s3c24xx_udc2 *dev)
{
	struct s3c24xx_udc2_request *req;
	struct s3c24xx_udc2_ep *ep = &dev->ep[0];
	int ret;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct s3c24xx_udc2_request, queue);
	else {
		DEBUG_EP0("%s: ---> BUG\n", __FUNCTION__);
		BUG();  //logic ensures         -jassi
		return;
	}

	DEBUG_EP0("%s: req.length = 0x%x, req.actual = 0x%x\n",
			__FUNCTION__, req->req.length, req->req.actual);

	if(req->req.length == 0) {
//		udc_writel(S3C24XX_UDC2_EP0_STATUS_RSR, S3C24XX_UDC2_EP0_STATUS_REG);
		dev->ep0state = WAIT_FOR_SETUP;
		first_time = 1;
		done(ep, req, 0);
		return;
	}

	if(!req->req.actual && first_time){     //for SetUp packet
//		udc_writel(S3C24XX_UDC2_EP0_STATUS_RSR, S3C24XX_UDC2_EP0_STATUS_REG);
		first_time = 0;
		return;
	}

	ret = read_fifo_ep0(ep, req);
	if (ret) {
		dev->ep0state = WAIT_FOR_SETUP;
		first_time = 1;
		done(ep, req, 0);
		return;
	}

}

/*
 * DATA_STATE_XMIT
 */
static int s3c24xx_udc2_ep0_write(struct s3c24xx_udc2 *dev)
{
	struct s3c24xx_udc2_request *req;
	struct s3c24xx_udc2_ep *ep = &dev->ep[0];
	int ret, need_zlp = 0;

	DEBUG_EP0("%s: ep0 write\n", __FUNCTION__);

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct s3c24xx_udc2_request, queue);

	if (!req) {
		DEBUG_EP0("%s: NULL REQ\n", __FUNCTION__);
		return 0;
	}

	DEBUG_EP0("%s: req.length = 0x%x, req.actual = 0x%x\n",
			__FUNCTION__, req->req.length, req->req.actual);

	if (req->req.length == 0) {
		dev->ep0state = WAIT_FOR_SETUP;
		done(ep, req, 0);
		return 1;
	}

	if (req->req.length - req->req.actual == ep0_fifo_size) {
		/* Next write will end with the packet size, */
		/* so we need Zero-length-packet */
		need_zlp = 1;
	}

	ret = write_fifo_ep0(ep, req);

	if ((ret == 1) && !need_zlp) {
		/* Last packet */
		DEBUG_EP0("%s: finished, waiting for status\n", __FUNCTION__);
		dev->ep0state = WAIT_FOR_SETUP;
	} else {
		DEBUG_EP0("%s: not finished\n", __FUNCTION__);
	}

	if (need_zlp) {
		DEBUG_EP0("%s: Need ZLP!\n", __FUNCTION__);
		dev->ep0state = DATA_STATE_NEED_ZLP;
	}

	if(ret)
		done(ep, req, 0);

	return 1;
}

/*
 *	s3c2410_udc_set_selfpowered
 */
static int s3c24xx_udc2_set_selfpowered(struct usb_gadget *gadget, int value)
{
	struct s3c24xx_udc2 *udc = the_controller;

	DPRINTK("%s()\n", __func__);

	if (value)
		udc->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		udc->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);

	return 0;
}

static int s3c24xx_udc2_get_status(struct s3c24xx_udc2 *dev,
		struct usb_ctrlrequest *crq)
{
	u16 status = 0;
//	u8 ep_num = crq->wIndex & 0x7F;
//	u8 is_in = crq->wIndex & USB_DIR_IN;

	switch (crq->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_INTERFACE:
		break;

	case USB_RECIP_DEVICE:
		status = dev->devstatus;
		break;

	case USB_RECIP_ENDPOINT:
		status = 0;
		break;

	default:
		return 1;
	}

	udc_epn_write(2, S3C24XX_UDC2_BYTE_WRITE_CNT_REG, 0);
	udc_writel(status, S3C24XX_UDC2_EP0_BUFFER_REG);

	return 0;
}

/*
 * WAIT_FOR_SETUP (OUT_PKT_RDY)
 */
static void s3c24xx_udc2_ep0_setup(struct s3c24xx_udc2 *dev, u32 csr)
{
	struct s3c24xx_udc2_ep *ep = &dev->ep[0];
	struct usb_ctrlrequest ctrl;
	int i, bytes, is_in;

	DEBUG_SETUP("%s: csr = 0x%x\n", __FUNCTION__, csr);

	/* Nuke all previous transfers */
	nuke(ep, -EPROTO);

	/* read control req from fifo (8 bytes) */
	bytes = s3c24xx_udc2_fifo_read(ep, (u16 *)&ctrl, 8);

	DEBUG_SETUP("Read CTRL REQ %d bytes\n", bytes);
	DEBUG_SETUP("CTRL.bRequestType = 0x%x (is_in %d)\n", ctrl.bRequestType,
			ctrl.bRequestType & USB_DIR_IN);
	DEBUG_SETUP("CTRL.bRequest = 0x%x\n", ctrl.bRequest);
	DEBUG_SETUP("CTRL.wLength = 0x%x\n", ctrl.wLength);
	DEBUG_SETUP("CTRL.wValue = 0x%x (%d)\n", ctrl.wValue, ctrl.wValue >> 8);
	DEBUG_SETUP("CTRL.wIndex = 0x%x\n", ctrl.wIndex);

	/* Set direction of EP0 */
	if (likely(ctrl.bRequestType & USB_DIR_IN)) {
		ep->bEndpointAddress |= USB_DIR_IN;
		is_in = 1;
	} else {
		ep->bEndpointAddress &= ~USB_DIR_IN;
		is_in = 0;
	}

	dev->req_pending = 1;
	dev->req_std = (ctrl.bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD;

	/* Handle some SETUP packets ourselves */
	switch (ctrl.bRequest) {
		case USB_REQ_SET_ADDRESS:
			if (ctrl.bRequestType
					!= (USB_TYPE_STANDARD | USB_RECIP_DEVICE))
				break;

			DEBUG_SETUP("%s: USB_REQ_SET_ADDRESS (%d)\n",
					__FUNCTION__, ctrl.wValue);
			udc_set_address(dev, ctrl.wValue);
			return;
		case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_GET_CONFIGURATION:
			break;
		case USB_REQ_GET_STATUS:
			if (dev->req_std) {
				if (!s3c24xx_udc2_get_status(dev, &ctrl))
					return;
			}
			break;
		default:
			DEBUG_SETUP("%s: DFAULT\n", __FUNCTION__);
			break;
	}

	if (likely(dev->driver)) {
		/* device-2-host (IN) or no data setup command,
		 * process immediately */
		spin_unlock(&dev->lock);
		i = dev->driver->setup(&dev->gadget, &ctrl);
		spin_lock(&dev->lock);

		if (i < 0)  {
			/* setup processing failed, force stall */
			DEBUG_SETUP("%s:  --> ERROR: gadget setup FAILED (stalling), setup returned %d\n",
					__FUNCTION__, i);
			DEBUG_SETUP("%s: bRequestType = 0x%02x, bRequest = 0x%02x\n", __FUNCTION__, ctrl.bRequestType, ctrl.bRequest);
			DEBUG_SETUP("CTRL.wLength = 0x%x\n", ctrl.wLength);
			DEBUG_SETUP("CTRL.wValue = 0x%x (%d)\n", ctrl.wValue, ctrl.wValue >> 8);
			DEBUG_SETUP("CTRL.wIndex = 0x%x\n", ctrl.wIndex);
			/* ep->stopped = 1; */
			dev->ep0state = WAIT_FOR_SETUP;
		}
	}
}

/*
 * handle ep0 interrupt
 */
static inline void s3c24xx_udc2_handle_ep0(struct s3c24xx_udc2 *dev)
{
	struct s3c24xx_udc2_ep *ep = &dev->ep[0];
	struct s3c24xx_udc2_request *req;
	u32 csr;
	u32 count;

	csr = udc_readl(S3C24XX_UDC2_EP0_STATUS_REG);
	count = udc_epn_read(S3C24XX_UDC2_BYTE_READ_CNT_REG, 0);

	DEBUG_EP0("%s: S3C24XX_UDC2_EP0_STATUS_REG = 0x%x\n", __FUNCTION__, csr);
	DEBUG_EP0("%s: S3C24XX_UDC2_BYTE_READ_CNT_REG[0] = 0x%x\n", __FUNCTION__, count);

	/*
	 * if SENT_STALL is set
	 *      - clear the SENT_STALL bit
	 */
	if (csr & S3C24XX_UDC2_EP0_STATUS_SHT) {
		printk("%s: S3C24XX_UDC2_EP0_STALL\n", __FUNCTION__);
		udc_writel(S3C24XX_UDC2_EP0_STATUS_SHT, S3C24XX_UDC2_EP0_STATUS_REG); /* clear */
		nuke(ep, -ECONNABORTED);
		dev->ep0state = WAIT_FOR_SETUP;
		DEBUG_EP0("%s: end\n", __FUNCTION__);
		return;
	}

	if (csr & S3C24XX_UDC2_EP0_STATUS_TST) {
		DEBUG_EP0("%s: EP0_TX_SUCCESS \n", __FUNCTION__);
		udc_writel(S3C24XX_UDC2_EP0_STATUS_TST, S3C24XX_UDC2_EP0_STATUS_REG); /* clear */
		if (list_empty(&ep->queue))
			req = NULL;
		else
			req = list_entry(ep->queue.next, struct s3c24xx_udc2_request, queue);
		if (req)
			s3c24xx_udc2_ep0_write(dev);
	}

	if ((csr & S3C24XX_UDC2_EP0_STATUS_RSR)) {
		if (dev->ep0state == WAIT_FOR_SETUP) {
			DEBUG_EP0("%s: WAIT_FOR_SETUP\n", __FUNCTION__);
			s3c24xx_udc2_ep0_setup(dev, csr);
		} else {
			DEBUG_EP0("%s: strange state!!(state = %s)\n",
					__FUNCTION__, state_names[dev->ep0state]);
		}
	}
	DEBUG_EP0("%s: end\n", __FUNCTION__);
}

static void s3c24xx_udc2_ep0_kick(struct s3c24xx_udc2 *dev, struct s3c24xx_udc2_ep *ep)
{
	DEBUG_EP0("%s: ep_is_in = %d\n", __FUNCTION__, ep_is_in(ep));
	if (ep_is_in(ep)) {
		dev->ep0state = DATA_STATE_XMIT;
		s3c24xx_udc2_ep0_write(dev);
	} else {
		dev->ep0state = DATA_STATE_RECV;
		s3c24xx_udc2_ep0_read(dev);
	}
}

/* ---------------------------------------------------------------------------
 *      device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int s3c24xx_udc2_get_frame(struct usb_gadget *_gadget)
{
#warning add properly support of various CPUs
#ifdef CONFIG_CPU_S3C2416
	return -ENOTSUPP;
#else
	/* frame count number [10:0] */
	u32 frame = udc_readl(S3C24XX_UDC2_FRAME_NUM_REG);
	DPRINTK("%s: %p\n", __FUNCTION__, _gadget);
	return (frame & 0x7ff);
#endif
}

static int s3c24xx_udc2_wakeup(struct usb_gadget *_gadget)
{
	return -ENOTSUPP;
}

static const struct usb_gadget_ops s3c24xx_udc2_ops = {
	.get_frame = s3c24xx_udc2_get_frame,
	.wakeup = s3c24xx_udc2_wakeup,
	.set_selfpowered = s3c24xx_udc2_set_selfpowered,
	/* current versions must always be self-powered */
};

static void nop_release(struct device *dev)
{
	DPRINTK("%s %s\n", __FUNCTION__, dev->bus_id);
}

static struct s3c24xx_udc2 memory = {
	.usb_address = 0,

	.gadget = {
		.ops = &s3c24xx_udc2_ops,
		.ep0 = &memory.ep[0].ep,
		.name = driver_name,
		.dev = {
			.bus_id = "gadget",
			.release = nop_release,
		},
	},

	/* control endpoint */
	.ep[0] = {
		.num = 0,
		.ep = {
			.name = ep0name,
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP0_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 0,
	},

	/* first group of endpoints */
	.ep[1] = {
		.num = 1,
		.ep = {
			.name = "ep1",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 1,
	},

	.ep[2] = {
		.num = 2,
		.ep = {
			.name = "ep2",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 2,
	},

	.ep[3] = {
		.num = 3,
		.ep = {
			.name = "ep3",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 3,
	},
	.ep[4] = {
		.num = 4,
		.ep = {
			.name = "ep4",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 4,
	},
	.ep[5] = {
		.num = 5,
		.ep = {
			.name = "ep5",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,

		.bEndpointAddress = 5,
	},
	.ep[6] = {
		.num = 6,
		.ep = {
			.name = "ep6",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,

		.bEndpointAddress = 6,
	},
	.ep[7] = {
		.num = 7,
		.ep = {
			.name = "ep7",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,

		.bEndpointAddress = 7,
	},
	.ep[8] = {
		.num = 8,
		.ep = {
			.name = "ep8",
			.ops = &s3c24xx_udc2_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,

		.bEndpointAddress = 8,
	},
};


static irqreturn_t vbus_irq(int irq, void *_dev)
{
	printk("%s!\n", __FUNCTION__);
	return IRQ_HANDLED;
}

/*
 *      probe - binds to the platform device
 */
static int s3c24xx_udc2_probe(struct platform_device *pdev)
{
	struct s3c24xx_udc2 *udc = &memory;
	struct device *dev = &pdev->dev;
	int retval;
	int i;

	dev_dbg(dev, "%s()\n", __func__);

	usb_bus_clock = clk_get(NULL, "usb-bus-gadget");
	if (IS_ERR(usb_bus_clock)) {
		dev_err(dev, "failed to get usb bus clock source\n");
		return PTR_ERR(usb_bus_clock);
	}

	clk_enable(usb_bus_clock);

	udc_clock = clk_get(NULL, "usb-device");
	if (IS_ERR(udc_clock)) {
		dev_err(dev, "failed to get udc clock source\n");
		return PTR_ERR(udc_clock);
	}

	clk_enable(udc_clock);

	mdelay(10);

	dev_dbg(dev, "got and enabled clocks\n");

	spin_lock_init(&udc->lock);
//	udc_info = pdev->dev.platform_data;

	rsrc_start = S3C24XX_PA_USB2DEV;
	rsrc_len   = S3C24XX_SZ_USB2DEV;

	if (!request_mem_region(rsrc_start, rsrc_len, driver_name))
		return -EBUSY;

	base_addr = ioremap(rsrc_start, rsrc_len);
	if (!base_addr) {
		retval = -ENOMEM;
		goto err_mem;
	}

	for (i = 0; i <= 8; i++)
		memory.ep[i].fifo = base_addr + S3C24XX_UDC2_EP0_BUFFER_REG + i * 4;

	udc->dev = pdev;

	device_initialize(&udc->gadget.dev);
	udc->gadget.dev.parent = dev;

	udc->gadget.is_dualspeed = 1;   // Hack only
	udc->gadget.is_otg = 0;
	udc->gadget.is_a_peripheral = 0;
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	the_controller = udc;
	platform_set_drvdata(pdev, udc);

	udc_reinit(udc);

	//FIXME
	//        local_irq_disable();

	/* irq setup after old hardware state is cleaned up */
	retval =
		request_irq(IRQ_USBD, s3c24xx_udc2_irq, IRQF_DISABLED, driver_name,
				udc);
	if (retval != 0) {
		dev_err(dev, "cannot get irq %i, err %d\n", IRQ_USBD, retval);
		retval = -EBUSY;
		goto err_map;
	}

	dev_dbg(dev, "got irq %i\n", IRQ_USBD);

//	s3c2410_gpio_cfgpin(S3C2410_GPC4, S3C2410_GPC4_OUTP);
//	s3c2410_gpio_setpin(S3C2410_GPC4, 1);
//	s3c2410_gpio_cfgpin(S3C2410_GPD12, S3C2410_GPD12_OUTP);
//	s3c2410_gpio_setpin(S3C2410_GPD12, 1);
//	s3c2410_gpio_cfgpin(S3C2410_GPF6, S3C2410_GPF6_EINT6);
	printk("GPF6 = %u\n", s3c2410_gpio_getpin(S3C2410_GPF6));
	//FIXME
	//        create_proc_files();
	//
	request_irq(IRQ_EINT6, vbus_irq, IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED, driver_name, udc);
	
	dev_dbg(dev, "probe ok\n");

	return 0;

	free_irq(IRQ_USBD, udc);
err_map:
	iounmap(base_addr);
err_mem:
	release_mem_region(rsrc_start, rsrc_len);

	return retval;
}

static int s3c24xx_udc2_remove(struct platform_device *pdev)
{
	struct s3c24xx_udc2 *dev = platform_get_drvdata(pdev);

	DPRINTK("%s: %p\n", __FUNCTION__, pdev);

	udc_disable(dev);
	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	free_irq(IRQ_USBD, dev);

	platform_set_drvdata(pdev, 0);

	the_controller = 0;

	return 0;
}

/*-------------------------------------------------------------------------*/
static struct platform_driver s3c24xx_udc2_driver = {
	.probe          = s3c24xx_udc2_probe,
	.remove         = s3c24xx_udc2_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "s3c24xx-usb2gadget",
	},
};

static int __init s3c24xx_udc2_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c24xx_udc2_driver);
	if(!ret)
		printk("Loaded %s version %s\n", driver_name, DRIVER_VERSION);

	return ret;
}

static void __exit s3c24xx_udc2_exit(void)
{
	platform_driver_unregister(&s3c24xx_udc2_driver);
	printk("Unloaded %s version %s\n", driver_name, DRIVER_VERSION);
}

module_init(s3c24xx_udc2_init);
module_exit(s3c24xx_udc2_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Yauhen Kharyzhy <jekhor@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c24xx-usb2gadget");
