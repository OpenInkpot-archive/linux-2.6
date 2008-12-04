/**
 * @file	s1r72xxx_udc.c
 * @brief	S1R72xxx USB Contoroller Driver(USB Device)
 * @date	2008/01/24
 * Copyright (C)SEIKO EPSON CORPORATION 2003-2008. All rights reserved.
 * This file is licenced under the GPLv2
 */

/******************************************
 * Declarations of function prototype
 ******************************************/
static int s1r72xxx_ep_enable(struct usb_ep *_ep,
	const struct usb_endpoint_descriptor *desc);
static int s1r72xxx_ep_disable(struct usb_ep *_ep);
static struct usb_request * s1r72xxx_ep_alloc_request(struct usb_ep *_ep,
	unsigned int gfp_flags);
void s1r72xxx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req);
static int s1r72xxx_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
	unsigned int gfp_flags);
static int s1r72xxx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req);
static int s1r72xxx_ep_set_halt(struct usb_ep *_ep, int value);
static int s1r72xxx_ep_fifo_status(struct usb_ep *_ep);
static void s1r72xxx_ep_fifo_flush(struct usb_ep *_ep);
static int s1r72xxx_usbc_get_frame(struct usb_gadget *_gadget);
static int s1r72xxx_usbc_wakeup(struct usb_gadget *_gadget);
static int s1r72xxx_usbc_set_selfpowered(struct usb_gadget *_gadget,
	int value);
static int s1r72xxx_usbc_vbus_session(struct usb_gadget *_gadget, int value);
static int s1r72xxx_usbc_vbus_draw(struct usb_gadget *_gadget, unsigned mA);
static int s1r72xxx_usbc_pullup(struct usb_gadget *_gadget, int value);
static int s1r72xxx_usbc_ioctl(struct usb_gadget *, unsigned code,
	unsigned long param);
static int __init s1r72xxx_usbc_probe(struct device *dev);
static int s1r72xxx_usbc_remove(struct device *dev);
void s1r72xxx_usbc_shutdown(struct device *dev);
static int s1r72xxx_usbc_suspend(struct device *dev, pm_message_t state);
static int s1r72xxx_usbc_resume(struct device *dev);
int usb_gadget_register_driver(struct usb_gadget_driver *driver);
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver);
static irqreturn_t s1r72xxx_usbc_irq(int irq, void *_dev);
static void s1r72xxx_usbc_release(struct device *dev);

static int usbc_irq_VBUS_CHG(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_VBUS(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_SIE(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_Bulk(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_RcvEP0SETUP(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_EP0(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_EP(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_irq_FPM(S1R72XXX_USBC_DEV *usbc_dev);
static int usbc_pullup(int d_pull_up);

static int handle_ep_in(S1R72XXX_USBC_EP *ep, S1R72XXX_USBC_REQ *req);
static int handle_ep_out(S1R72XXX_USBC_EP *ep, S1R72XXX_USBC_REQ *req);
static int change_ep0_state(S1R72XXX_USBC_EP *ep, unsigned char flag);
static int change_epx_state(S1R72XXX_USBC_EP *ep, unsigned char flag);
static void request_done(S1R72XXX_USBC_EP *ep, S1R72XXX_USBC_REQ *req,
	int status);
static int usbc_start_handshake(S1R72XXX_USBC_DEV *usbc_dev);
int usbc_lsi_init(S1R72XXX_USBC_DEV *usbc_dev);
void usbc_ep_init(S1R72XXX_USBC_DEV *usbc_dev);
static void all_request_done(S1R72XXX_USBC_DEV *usbc_dev, int status);
unsigned char change_driver_state(S1R72XXX_USBC_DEV *usbc_dev,
	unsigned int state, unsigned int event);
static void stop_activity(S1R72XXX_USBC_DEV *usbc_dev);
extern int usbc_lsi_init(S1R72XXX_USBC_DEV *usbc_dev);
extern void usbc_ep_init(S1R72XXX_USBC_DEV *usbc_dev);
extern void usbc_fifo_init(S1R72XXX_USBC_DEV *usbc_dev);
static inline unsigned char check_fifo_remain(S1R72XXX_USBC_DEV *usbc_dev);
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
static inline unsigned char check_fifo_cache(S1R72XXX_USBC_DEV *usbc_dev);
#endif
static inline void usb_disconnected(S1R72XXX_USBC_DEV *usbc_dev);
static void send_ep0_short_packet(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep);
static void remote_wakeup_timer(unsigned long _dev);
static void chirp_cmp_timer(unsigned long _dev);
static void vbus_timer(unsigned long _dev);

#ifdef CONFIG_EBOOK5_LED
#define EBOOK5_LED_TOGGLE		2
#define EBOOK5_LED_ON			1
#define EBOOK5_LED_OFF			0
unsigned int ebook5_led = 0;
EXPORT_SYMBOL(ebook5_led);
static unsigned long next_led_time = 0;
static unsigned int next_led = 0;
static void usbtg_led(int);
#endif /* CONFIG_EBOOK5_LED */

static const unsigned char driver_name[] = S1R72_DEVICE_NAME;
S1R72XXX_USBC_DEV *the_controller;

extern S1R72XXX_USBC_DRV_STATE_TABLE_ELEMENT
	s1r72xxx_drv_state_tbl[S1R72_GD_MAX_DRV_STATE][S1R72_GD_MAX_DRV_EVENT];
extern unsigned short epx_max_packet_tbl[S1R72_MAX_ENDPOINT] ;
extern S1R72XXX_USBC_EP_IRQ_TBL usbc_ep_irq_tbl[S1R72_MAX_ENDPOINT + 1];

/* TEST Packet data */
static const unsigned char test_mode_packet[TEST_MODE_PACKET_SIZE + 1] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
	0xEE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xBF, 0xDF,
	0xEF, 0xF7, 0xFB, 0xFD, 0xFC, 0x7E, 0xBF, 0xDF,
	0xEF, 0xF7, 0xFB, 0xFD, 0x7E, 0x00
};

/* for debug */
#ifdef DEBUG_PROC_LOG
static unsigned int queue_dbg_counter = 0;
static unsigned int queue_dbg_carry = 0;
static unsigned int queue_dbg_seq_no = 0;

/* debub log table */
#define	S1R72_DEBUG_TABLE_SIZE	2048
S1R72XXX_USBC_QUE_DEBUG		queue_dbg_tbl[S1R72_DEBUG_TABLE_SIZE];

/* debub log string table (/proc/driver/udc) */
S1R72XXX_USBC_QUE_DEBUG_TITLE dbg_name_tbl[S1R72_DEBUG_ITEM_MAX] = {
	{"enable  ","ep","atr=","sof="},	/* 0x00 */
	{"disable ","ep","atr=","sof="},
	{"allc_req","--","----","sof="},
	{"free_req","ep","len=","sof="},
	{"allc_buf","ep","len=","sof="},
	{"free_buf","ep","len=","sof="},	/* 0x05 */
	{"queue   ","ep","len=","sof="},	/* 0x06 */
	{"queue   ","ep","sno=","sof="},
	{"dequeue ","ep","sno=","sof="},
	{"dequeue ","ep","len=","sof="},
	{"set_halt","ep","val=","sof="},	/* 0x0A */
	{"probe   ","--","----","sof="},
	{"remove  ","--","----","sof="},
	{"shutdown","--","----","sof="},
	{"reg_drv ","--","----","sof="},
	{"ureg_drv","--","----","sof="},
	{"irq_s   ","ad","dat=","sof="},	/* 0x10 */
	{"irq_e   ","--","dat=","sof="},
	{"irq_dev ","ad","dat=","sof="},
	{"irq_pow ","ad","dat=","sof="},
	{"irq_vbus","ad","dat=","sof="},
	{"irq_sie ","ad","dat=","sof="},	/* 0x15 */
	{"irq_stup","rt","req=","sof="},
	{"irq_ep0 ","ad","dat=","sof="},
	{"irq_epx ","ad","dat=","sof="},
	{"irq_fpm ","ad","dat=","sof="},
	{"chg_drv ","dt","dat=","sof="},	/* 0x1A */
	{"chg_ep0 ","ep","sts=","sof="},
	{"chg_epx ","ep","sts=","sof="},
	{"handle  ","ep","len=","sof="},	/* 0x1D */
	{"handle  ","ep","sno=","sof="},
	{"compelte","ep","len=","sof="},
	{"compelte","ep","sno=","sof="},	/* 0x20 */
	{"chg_drv ","st","pow=","sof="},
	{"register","0x","dat=","sof="},
	{"short   ","ep","len=","sof="}		/* 0x23 */
};

void s1r72xxx_queue_log(
	unsigned char	func_name,
	unsigned int	data1,
	unsigned int	data2
)
{
	queue_dbg_tbl[queue_dbg_counter].func_name = func_name;
	queue_dbg_tbl[queue_dbg_counter].data1 = data1;
	queue_dbg_tbl[queue_dbg_counter].data2 = data2;
	queue_dbg_tbl[queue_dbg_counter].sof_no
		= le16_to_cpu(rsD_RegRead16(the_controller, rsD_S1R72_FrameNumber))
			& S1R72_FrameNumber;

	queue_dbg_counter ++;
	if (queue_dbg_counter > S1R72_DEBUG_TABLE_SIZE - 1){
		queue_dbg_counter = 0;
		queue_dbg_carry = 1;
	}
}

#define	S1R72_USBC_INC_QUE_NO	queue_dbg_seq_no++
#define	S1R72_USBC_SET_QUE_NO(x)	(x)->queue_seq_no = queue_dbg_seq_no

#else

#define s1r72xxx_queue_log2(x, y, z)
#define s1r72xxx_queue_log(x, y, z)
#define	S1R72_USBC_INC_QUE_NO
#define	S1R72_USBC_SET_QUE_NO(x)

#endif

#ifdef DEBUG_PROC

#include <linux/seq_file.h>

static const char proc_filename[] = "driver/udc";

static int proc_udc_show(struct seq_file *s , void * a)
{
	S1R72XXX_USBC_DEV	*dev_temp = the_controller;
	unsigned char		*buf;
	unsigned int		i;
	unsigned long		flags;

#ifdef DEBUG_REMOTE_WAKEUP
	s1r72xxx_usbc_wakeup(&dev_temp->gadget);
#endif

#ifdef DEBUG_SUSPEND_RESUME
	pm_message_t		dummy_pm;
	static unsigned char	toggle = 0;
	if (toggle == 0){
		s1r72xxx_usbc_suspend(dev_temp->dev, dummy_pm);
		toggle = 1;
		seq_printf(s, "suspend \n");
	} else {
		s1r72xxx_usbc_resume(dev_temp->dev);
		toggle = 0;
		seq_printf(s, "resume \n");
	}
#else

	spin_lock_irqsave(&dev_temp->lock, flags);

	buf =  dev_temp->base_addr;
	seq_printf(s,"driver state = 0x%02x\n",dev_temp->usbcd_state);
	seq_printf(s,"Address: +00 +01 +02 +03 +04 +05 +06 +07");
	seq_printf(s," +08 +09 +0A +0B +0C +0D +0E +0F\n");
	for (i = 0 ; i < 0x190 ; i += 16){
		switch( i ){
//		case 0x020:
		case 0x030:
		case 0x040:
		case 0x050:
		case 0x060:
		case 0x070:
//		case 0x080:
//		case 0x090:
//		case 0x0A0:
			break;
		default:
			seq_printf(s,
				"0x%04X :  %02x  %02x  %02x  %02x  %02x  %02x  %02x  %02x"
				"  %02x  %02x  %02x  %02x  %02x  %02x  %02x  %02x\n",
				i,
				rcD_RegRead8(dev_temp, i     ), rcD_RegRead8(dev_temp, i + 1 ),
				rcD_RegRead8(dev_temp, i + 2 ), rcD_RegRead8(dev_temp, i + 3 ),
				rcD_RegRead8(dev_temp, i + 4 ), rcD_RegRead8(dev_temp, i + 5 ),
				rcD_RegRead8(dev_temp, i + 6 ), rcD_RegRead8(dev_temp, i + 7 ),
				rcD_RegRead8(dev_temp, i + 8 ), rcD_RegRead8(dev_temp, i + 9 ),
				rcD_RegRead8(dev_temp, i + 10), rcD_RegRead8(dev_temp, i + 11),
				rcD_RegRead8(dev_temp, i + 12), rcD_RegRead8(dev_temp, i + 13),
				rcD_RegRead8(dev_temp, i + 14), rcD_RegRead8(dev_temp, i + 15));
		}
	}
#ifdef DEBUG_PROC_LOG
	if ((i == 0) && (queue_dbg_carry == 0)){
		spin_unlock_irqrestore(&dev_temp->lock, flags);
		return 0;
	}

	if ( queue_dbg_carry != 0 ){
		i = queue_dbg_counter + 1;
	} else {
		i = 0;
	}

        while ( i != queue_dbg_counter ){
		if (i > S1R72_DEBUG_TABLE_SIZE - 1){
			i = 0;
		}
		seq_printf(s, "%s: %s%x\t: %s%x\t\t: %s%x\t\n",
			dbg_name_tbl[queue_dbg_tbl[i].func_name].action_string,
			dbg_name_tbl[queue_dbg_tbl[i].func_name].target_string,
			queue_dbg_tbl[i].data1,
			dbg_name_tbl[queue_dbg_tbl[i].func_name].value_string,
			queue_dbg_tbl[i].data2,
			dbg_name_tbl[queue_dbg_tbl[i].func_name].time_string,
			queue_dbg_tbl[i].sof_no
		);
		++i;
	}
#endif /* DEBUG_PROC_LOG */
	spin_unlock_irqrestore(&dev_temp->lock, flags);
#endif /* DEBUG_SUSPEND_RESUME */

	return 0;
}
static int proc_udc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_udc_show, NULL);
}

static struct file_operations proc_ops = {
	.open           = proc_udc_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

inline static void create_proc_file(void)
{
	struct proc_dir_entry *pde;

	pde = create_proc_entry (proc_filename, 0, NULL);
	if (pde)
		pde->proc_fops = &proc_ops;
}

inline static void remove_proc_file(void)
{
	remove_proc_entry(proc_filename, NULL);
}

#else	/* DEBUG_PROC */
#define create_proc_file(x)
#define remove_proc_file(x)
#endif	/* DEBUG_PROC */

/******************************************
 * definitions of "s1r72xxx_ep_ops"
 ******************************************/
/**
 * @struct s1r72xxx_ep_ops
 * @brief	USB Endpoint operation functions.
 */
static struct usb_ep_ops s1r72xxx_ep_ops = {
	.enable			= s1r72xxx_ep_enable,
	.disable		= s1r72xxx_ep_disable,

	.alloc_request	= s1r72xxx_ep_alloc_request,
	.free_request	= s1r72xxx_ep_free_request,

	.queue			= s1r72xxx_ep_queue,
	.dequeue		= s1r72xxx_ep_dequeue,

	.set_halt		= s1r72xxx_ep_set_halt,
	.fifo_status	= s1r72xxx_ep_fifo_status,
	.fifo_flush		= s1r72xxx_ep_fifo_flush,
};

/******************************************
 * definition of "s1r72xxx_gadget_ops"
 ******************************************/
/** @struct s1r72xxx_gadget_ops
 * @brief	USB non-Endpoint operation functions.
 */
static struct usb_gadget_ops s1r72xxx_gadget_ops = {
	.get_frame			= s1r72xxx_usbc_get_frame,
	.wakeup				= s1r72xxx_usbc_wakeup,
	.set_selfpowered	= s1r72xxx_usbc_set_selfpowered,
	.vbus_session		= s1r72xxx_usbc_vbus_session,
	.vbus_draw			= s1r72xxx_usbc_vbus_draw,
	.pullup				= s1r72xxx_usbc_pullup,
	.ioctl			= s1r72xxx_usbc_ioctl,
};

/******************************************
 * definition of "s1r72xxx_usbcd_driver"
 ******************************************/
/** @struct s1r72xxx_usbcd_driver
 * @brief	driver common functions.
 */
static struct device_driver s1r72xxx_usbcd_driver = {
    .name    = driver_name,
    .bus     = &platform_bus_type,
    .probe   = s1r72xxx_usbc_probe,
    .remove  = __exit_p(s1r72xxx_usbc_remove),
    .shutdown= s1r72xxx_usbc_shutdown,
    .suspend = s1r72xxx_usbc_suspend,
    .resume  = s1r72xxx_usbc_resume,
};

/* max packet size table (USB spec) */
unsigned int packet_size_tbl[S1R72_SUPPORT_BUS_TYPE][S1R72_SUPPORT_TRN_TYPE] =
{
	{	S1R72_HS_CNT_MAX_PKT,
		S1R72_HS_BLK_MAX_PKT,
		S1R72_HS_INT_MAX_PKT,
		S1R72_HS_ISO_MAX_PKT},
	{
		S1R72_FS_CNT_MAX_PKT,
		S1R72_FS_BLK_MAX_PKT,
		S1R72_FS_INT_MAX_PKT,
		S1R72_FS_ISO_MAX_PKT}
};

/* irq handler table */
S1R72XXX_USBC_IRQ_TBL usbc_dev_irq_tbl[] =
{
	{D_VBUS_Changed,		usbc_irq_VBUS_CHG},
	{D_SIE_IntStat,			usbc_irq_SIE},
	{D_BulkIntStat,			usbc_irq_Bulk},
	{D_EP0IntStat,			usbc_irq_EP0},
	{D_EPrIntStat,			usbc_irq_EP},
	{D_RcvEP0SETUP,			usbc_irq_RcvEP0SETUP},
	{S1R72_DEV_IRQ_NONE,	NULL}
};

/* SIE interrupt source table */
unsigned char	sie_irq_tbl[] =
{
	S1R72_NonJ,
/*	S1R72_RcvSOF,*/
	S1R72_DetectRESET,
	S1R72_DetectSUSPEND,
	S1R72_ChirpCmp,
	S1R72_RestoreCmp,
	S1R72_SetAddressCmp
};

/* "function" sysfs attribute */
static ssize_t
show_function(struct device *_dev, struct device_attribute *attr, char *buf)
{
	S1R72XXX_USBC_DEV *dev = dev_get_drvdata(_dev);
	if ( (dev->driver == NULL) || (dev->driver->function == NULL)
		|| (strlen(dev->driver->function) > PAGE_SIZE) ) {
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", dev->driver->function);
}

static DEVICE_ATTR(function, S_IRUGO, show_function, NULL);

/****************************************************
 * Declarations of inline function *
 ****************************************************/
/* ========================================================================= */
/**
 * @brief	- check read FIFO is enabled.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @retval	_RD_REMAIN_OK / _RD_REMAIN_NG
 */
/* ========================================================================= */
static inline unsigned char check_fifo_remain(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	timeout_ct = 0;		/* timeout counter */

	/**
	 * - 1. Wait RdRemainValid is set:
	 */
	/* check RdRemainValid bit */
	while ( ((unsigned int)rsD_RegRead16(usbc_dev, rsFIFO_RdRemain)
		& RdRemainValid)
		!= RdRemainValid) {
		/**
		 * - 2. Check timeout counter:
		 */
		timeout_ct ++;
		if (timeout_ct > S1R72_RD_REMAIN_TIMEOUT){
			/* in case of time out, return NG */
			 return S1R72_RD_REMAIN_NG;
		}
	}
	return S1R72_RD_REMAIN_OK;
}

#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
/* ========================================================================= */
/**
 * @brief	- check FIFO cache is flushed.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @retval	S1R72_CACHE_REMAIN_OK / S1R72_CACHE_REMAIN_NG
 */
/* ========================================================================= */
static inline unsigned char check_fifo_cache(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned int	timeout_ct = 0;		/* timeout counter */

	/**
	 * - 1. Wait to flush cache:
	 */
	/* check CacheRemain bit */
	while ( ((unsigned int)rcD_RegRead8(usbc_dev, rcCacheRemain)
		& WrFIFO_C_Remain)
		== WrFIFO_C_Remain) {
		/**
		 * - 2. Check timeout counter:
		 */
		timeout_ct ++;
		if (timeout_ct > S1R72_CACHE_REMAIN_TIMEOUT){
			/* in case of time out, return NG */
			 return S1R72_CACHE_REMAIN_NG;
		}
	}
	return S1R72_CACHE_REMAIN_OK;
}
#endif

/* ========================================================================= */
/**
 * @brief	- set device when USB bus disconnected.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void usb_disconnected(S1R72XXX_USBC_DEV *usbc_dev)
{
	/**
	 * - 1. Clear USB bus speed:
	 */
	usbc_dev->gadget.speed = USB_SPEED_UNKNOWN;

	/**
	 * - 2. Set interrupt enable to initial state:
	 */
	rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, (S1R72_NonJ | S1R72_RcvSOF
		| S1R72_DetectRESET | S1R72_DetectSUSPEND | S1R72_ChirpCmp
		| S1R72_RestoreCmp | S1R72_SetAddressCmp));
	rcD_RegWrite8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_REG_ALL_CLR);
	rcD_IntClr8(usbc_dev, rcMainIntStat, (DevInt | FinishedPM));
	rcD_RegSet8(usbc_dev, rcMainIntEnb, (DevInt | FinishedPM));
	rcD_IntClr8(usbc_dev, rcDeviceIntStat, (D_VBUS_Changed | D_SIE_IntStat));
	rcD_RegWrite8(usbc_dev, rcDeviceIntEnb, (D_VBUS_Changed | D_SIE_IntStat));

	/**
	 * - 3. Clear auto negotiation settings:
	 */
	rcD_RegWrite8(usbc_dev, rcD_S1R72_NegoControl, S1R72_REG_ALL_CLR);

	/**
	 * - 4. Set bus termination settings to disconnected state:
	 */
	rcD_RegWrite8(usbc_dev, rcD_S1R72_XcvrControl,
		(S1R72_XcvrSelect | S1R72_OpMode_NonDriving));

}

/* ========================================================================= */
/**
 * @brief	- set EnShortPkt bit.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void set_ep_enshortpkt(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	reg_addr;		/* register address */

	if (ep->ep_subname == S1R72_GD_EP0){
		reg_addr = ep->reg.ep0.EP0ControlIN;
	} else {
		reg_addr = ep->reg.epx.EPxControl;
	}
	if ( (rcD_RegRead8(usbc_dev, reg_addr)
		& S1R72_EnShortPkt) == S1R72_EnShortPkt ){
		return ;
	}
	s1r72xxx_queue_log(S1R72_DEBUG_EP_DEQUEUE, reg_addr,
		rcD_RegRead8(usbc_dev, reg_addr));

	/**
	 * - 1. Set EnShortPkt bit:
	 */
	rcD_RegSet8(usbc_dev, reg_addr, S1R72_EnShortPkt);

}

/* ========================================================================= */
/**
 * @brief	- clear EnShortPkt bit.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void clear_ep_enshortpkt(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */
	unsigned short	reg_addr;		/* register address */

	if (ep->ep_subname == S1R72_GD_EP0){
		reg_addr = ep->reg.ep0.EP0ControlIN;
	} else {
		reg_addr = ep->reg.epx.EPxControl;
	}
	if ( (rcD_RegRead8(usbc_dev, reg_addr)
		& S1R72_EnShortPkt) != S1R72_EnShortPkt ){
		return ;
	}
	s1r72xxx_queue_log(S1R72_DEBUG_EP_DEQUEUE, reg_addr,
		rcD_RegRead8(usbc_dev, reg_addr));

	/**
	 * - 1. Clear EnShortPkt bit:
	 */
	rcD_RegClear8(usbc_dev, reg_addr, S1R72_EnShortPkt);

	/**
	 * - 2. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_ENSHORTPKT_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, reg_addr)
			& S1R72_EnShortPkt) != S1R72_EnShortPkt ){
			break;
		}
		timeout_ct ++;
	}
	s1r72xxx_queue_log(S1R72_DEBUG_EP_DEQUEUE, reg_addr,
		rcD_RegRead8(usbc_dev, reg_addr));
}

/* ========================================================================= */
/**
 * @brief	- set ForceNAK bit to IN endpoint0.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void set_ep0_in_force_nak(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */

	if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlIN)
		& S1R72_ForceNAK) == S1R72_ForceNAK ){
		return ;
	}

	/**
	 * - 1. Clear ForceNAK bit:
	 */
	rcD_RegClear8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
	rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0ControlIN,
		S1R72_ForceNAK);

	/**
	 * - 2. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_FORCENAK_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlIN)
			& S1R72_ForceNAK) == S1R72_ForceNAK ){
			break;
		}
		timeout_ct ++;
	}
	rcD_RegSet8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
}


/* ========================================================================= */
/**
 * @brief	- clear ForceNAK bit to IN endpoint 0.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void clear_ep0_in_force_nak(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */

	/**
	 * - 1. Clear ForceNAK bit:
	 */
	if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlIN)
		& S1R72_ForceNAK) != S1R72_ForceNAK ){
		return ;
	}

	rcD_RegClear8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
	rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0ControlIN,
		S1R72_ForceNAK);

	/**
	 * - 2. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_FORCENAK_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlIN)
			& S1R72_ForceNAK) != S1R72_ForceNAK ){
			break;
		}
		timeout_ct ++;
	}
	rcD_RegSet8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
}

/* ========================================================================= */
/**
 * @brief	- set ForceNAK bit to OUT endpoint0.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void set_ep0_out_force_nak(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */

	if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlOUT)
		& S1R72_ForceNAK) == S1R72_ForceNAK ){
		return ;
	}

	/**
	 * - 1. Clear ForceNAK bit:
	 */
	rcD_RegClear8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
	rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0ControlOUT,
		S1R72_ForceNAK);

	/**
	 * - 2. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_FORCENAK_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlOUT)
			& S1R72_ForceNAK) == S1R72_ForceNAK ){
			break;
		}
		timeout_ct ++;
	}
	rcD_RegSet8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
}

/* ========================================================================= */
/**
 * @brief	- clear ForceNAK bit to OUT endpoint0.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void clear_ep0_out_force_nak(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */

	/**
	 * - 1. Clear ForceNAK bit:
	 */
	if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlOUT)
		& S1R72_ForceNAK) != S1R72_ForceNAK ){
		return ;
	}

	rcD_RegClear8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
	rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0ControlOUT,
		S1R72_ForceNAK);

	/**
	 * - 2. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_FORCENAK_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlOUT)
			& S1R72_ForceNAK) != S1R72_ForceNAK ){
			break;
		}
		timeout_ct ++;
	}
	rcD_RegSet8(usbc_dev, ep->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
}

/* ========================================================================= */
/**
 * @brief	- set ForceNAK bit to endpoint x.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void set_epx_force_nak(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */

	if ( (rcD_RegRead8(usbc_dev, ep->reg.epx.EPxControl)
		& S1R72_ForceNAK) == S1R72_ForceNAK ){
		return ;
	}

	/**
	 * - 1. Clear ForceNAK bit:
	 */
	rcD_RegSet8(usbc_dev, ep->reg.epx.EPxControl,
		S1R72_ForceNAK);

	/**
	 * - 2. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_FORCENAK_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, ep->reg.epx.EPxControl)
			& S1R72_ForceNAK) == S1R72_ForceNAK ){
			break;
		}
		timeout_ct ++;
	}
}

/* ========================================================================= */
/**
 * @brief	- clear ForceNAK bit to OUT endpoint x.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static inline void clear_epx_force_nak(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep)
{
	unsigned short	timeout_ct = 0;		/* timeout counter */

#if	defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
	unsigned char	nakRetryFlag = S1R72_FORCENAK_RETRY_OFF ;	/* ForceNak Retry Flag */
#endif

#if	defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
	/**
	 * - 1. Clear TranNAK bit:
	 */
	rcD_IntClr8(usbc_dev, ep->reg.epx.EPxIntStat, S1R72_EnOUT_TranNAK);
#endif

	/**
	 * - 2. Clear ForceNAK bit:
	 */
	if ( (rcD_RegRead8(usbc_dev, ep->reg.epx.EPxControl)
		& S1R72_ForceNAK) != S1R72_ForceNAK ){
		return;
	}
	rcD_RegClear8(usbc_dev, ep->reg.epx.EPxControl,
		S1R72_ForceNAK);

	/**
	 * - 3. Check timeout counter:
	 */
	while ( timeout_ct < S1R72_FORCENAK_TIMEOUT ) {
		if ( (rcD_RegRead8(usbc_dev, ep->reg.epx.EPxControl)
			& S1R72_ForceNAK) != S1R72_ForceNAK ){
			break;
		}

#if defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
		else if( (rcD_RegRead8(usbc_dev, ep->reg.epx.EPxIntStat)
				& S1R72_EnOUT_TranNAK) == S1R72_EnOUT_TranNAK ){
			if( nakRetryFlag == S1R72_FORCENAK_RETRY_OFF  ) {
				rcD_RegClear8(usbc_dev, ep->reg.epx.EPxControl,
								S1R72_ForceNAK);
				nakRetryFlag = S1R72_FORCENAK_RETRY_ON ;
			}
		}
#endif
		timeout_ct++;
	}
}

/****************************************************
 * Declarations of function exposed to other drivers *
 ****************************************************/
/* ========================================================================= */
/**
 * @brief	- configure endpoint, making it usable.
 * @par		usage:
 *				API(see functional specifications: 8.1.1.)
 * @param	*_ep;	endpoint structure that specifies the endpoint.
 * @param	*desc;	endpoint descriptor structure that includes endpoint
 *					specifications.
 * @retval	return 0 when enabling ep complete successfull.
 *			another situation return error code.
 */
/* ========================================================================= */
static int s1r72xxx_ep_enable(struct usb_ep *_ep,
	const struct usb_endpoint_descriptor *desc)
{
	S1R72XXX_USBC_DEV	*usbc_dev;				/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;					/* USB endpoint informations */
	unsigned long		flags;					/* spin lock flag */
	unsigned short		max_pkt;				/* Max Packet Size */
	unsigned short		usb_speed = S1R72_HS;			/* USB bus speed */
	unsigned char		reg_value = S1R72_REG_ALL_CLR;	/* register value */
	unsigned char		ep_attr;				/* bmAttributes */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - param != NULL, _ep is not ep0, bEndpointAddress matches,
	 *    FIFO size matches wMaxPacketSize, Transfer type, speed...
	 *  - in case of error, return -EINVAL, -ERANGE or -ESHUTDOWN.
	 */
	/* parameter check */
	if ( (_ep == NULL) || (desc == NULL) ) {
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}
	if (desc->bDescriptorType != USB_DT_ENDPOINT) {
		DEBUG_MSG("%s, bad descriptor\n", __FUNCTION__);
		return -EINVAL;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	if ( ep->ep_desc != NULL) {
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}
	/* endpoint number, address pakcet size check */
	if ( (ep->ep_subname == S1R72_GD_EP0)
			|| (ep->fifo_size < (le16_to_cpu(desc->wMaxPacketSize)
				& S1R72_USB_MAX_PACKET)) ) {
		DEBUG_MSG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}
	/* endpoint transfer attributes check */
	ep_attr = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	if ( ( ep_attr != USB_ENDPOINT_XFER_BULK)
		&& ( ep_attr != USB_ENDPOINT_XFER_INT)
		&& ( ep_attr != USB_ENDPOINT_XFER_ISOC) ) {
		DEBUG_MSG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}
	ep->bmAttributes = desc->bmAttributes;
	ep->bEndpointAddress = desc->bEndpointAddress;
	ep->fifo_data = 0;

	/* max packet size and get current usb speed */
	max_pkt = le16_to_cpu(desc->wMaxPacketSize) & S1R72_USB_MAX_PACKET;
	DEBUG_MSG("%s, max pkt=%d\n", __FUNCTION__, max_pkt);

	usbc_dev = ep->dev;
	/* '1'=FS, '0'=HS */
	if ( ( (rcD_RegRead8(usbc_dev, rcD_USB_Status)) & S1R72_FSxHS )
		== S1R72_Status_FS ) {
		usb_speed = S1R72_FS;
		DEBUG_MSG("%s, speed=FS\n", __FUNCTION__ );
	}

	/* parameter check */
	if (usbc_dev->driver == NULL) {
		DEBUG_MSG("%s, usbc_dev->driver=NULL\n", __FUNCTION__);
		return -ESHUTDOWN;
	}
	if (usbc_dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DEBUG_MSG("%s, speed=USB_SPEED_UNKNOWN\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	/**
	 * - 2. Initialize parameter:
	 *  - initialize _ep.
	 */
	ep->ep_desc = desc;
	ep->ep.maxpacket = max_pkt;
	ep->wMaxPacketSize = max_pkt;

	/**
	 * - 3. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&ep->dev->lock, flags);

	/**
	 * - 4. Disable EP interrupt:
	 *  - disables interrupt registers on the usb device.
	 *  - DeviceIntEnb
	 */
	rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxIntEnb, S1R72_REG_ALL_CLR);

	/**
	 * - 5. Configure endpoint registers on the usb device:
	 *  - D_EPaStartAdrs_H/L, D_EPaMaxSize_H/L,
	 *    D_EPaConfig_0, D_EPaControl, D_EPaJoin
	 */
	change_epx_state(ep, S1R72_GD_EP_IDLE);
	rsD_RegWrite16(usbc_dev, ep->reg.epx.EPxMaxSize, max_pkt);

	reg_value = desc->bEndpointAddress;
	if ( ((desc->bEndpointAddress) & USB_ENDPOINT_DIR_MASK ) == USB_DIR_OUT) {
		/* OUT endpoint */
		DEBUG_MSG("%s, OUT\n", __FUNCTION__);
		switch(desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
		case USB_ENDPOINT_XFER_BULK:
			DEBUG_MSG("%s, Bulk\n", __FUNCTION__);
			break;
		case USB_ENDPOINT_XFER_INT:
			DEBUG_MSG("%s, INT\n", __FUNCTION__);
			reg_value |= S1R72_IntEP_Mode;
			break;
/* V17 supports ISO transfer */
#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		case USB_ENDPOINT_XFER_ISOC:
			DEBUG_MSG("%s, ISO\n", __FUNCTION__);
			reg_value |= S1R72_ISO;
			break;
#endif
		default:
			spin_unlock_irqrestore(&ep->dev->lock, flags);
			DEBUG_MSG("%s, bad bmAttributes =%d\n",
				__FUNCTION__, desc->bmAttributes);
			return -EINVAL;
		}
	} else {
/* V17 supports ISO transfer */
#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		switch(desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
		case USB_ENDPOINT_XFER_ISOC:
			DEBUG_MSG("%s, ISO\n", __FUNCTION__);
			reg_value |= S1R72_ISO;
			break;
		default:
			break;
		}
#endif
	}
#if defined(CONFIG_USB_S1R72V05_GADGET)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_MODULE)
	reg_value |= S1R72_EnEndpoint;
#endif
	rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxConfig_0, reg_value);
	rcD_RegSet8(usbc_dev, ep->reg.epx.EPxControl, S1R72_ToggleClr);
	reg_value = S1R72_REG_ALL_CLR;

	/**
	 * - 6. Clear endpoint FIFO:
	 *  - D_EPrFIFO_Clr
	 */
	S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	/**
	 * - 6.1. clear FIFO addup:
	 */
	ep->fifo_addup = FIFO_ADDUP_CLR;
#endif

	/**
	 * - 7. Interrupt enable:
	 *  - enables interrupt registers on the usb device.
	 *    DeviceIntEnb.D_BulkIntEnb, D_EPrIntEnb, D_EPaIntEnb.
	 */
	rcD_IntClr8(usbc_dev, rcD_S1R72_EPrIntStat,
		S1R72_EPINT_ENB(ep->ep_subname));
	rcD_RegSet8(usbc_dev, rcD_S1R72_EPrIntEnb,
		S1R72_EPINT_ENB(ep->ep_subname));
	rcD_IntClr8(usbc_dev, rcDeviceIntStat, D_EPrIntStat);
	rcD_RegSet8(usbc_dev, rcDeviceIntEnb, D_EPrIntStat);

	/* set EnEndpoint or AREAx_Join1 */
#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	S1R72_AREAxJOIN_ENB(usbc_dev, ep->ep_subname);
#endif

	if ( ((desc->bEndpointAddress) & USB_ENDPOINT_DIR_MASK ) == USB_DIR_IN) {
		/* IN endpoint */
		rcD_IntClr8(usbc_dev, ep->reg.epx.EPxIntStat, S1R72_IN_TranACK);
		rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxIntEnb, S1R72_EnIN_TranACK);
		DEBUG_MSG("%s, set IN Tran ack \n", __FUNCTION__);
	} else {
		/* OUT endpoint */
		rcD_IntClr8(usbc_dev, ep->reg.epx.EPxIntStat,
			S1R72_OUT_ShortACK | S1R72_OUT_TranACK);
		rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxIntEnb,
			S1R72_EnOUT_ShortACK | S1R72_EnOUT_TranACK);
		DEBUG_MSG("%s, set OUT Tran ack \n", __FUNCTION__);
	}

	/**
	 * - 8. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	/**
	 * - 9. Return.:
	 *  - return 0;
	 */
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- endpoint is no longer usable
 * @par		usage:
 *				API(see functional specifications: 8.1.2.)
 * @param	*_ep;	endpoint structure that specifies the endpoint.
 * @retval	return 0 when disabling ep complete successfull.
 *			another situation return error code.
 */
/* ========================================================================= */
static int s1r72xxx_ep_disable(struct usb_ep *_ep)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;			/* USB endpoint informations */
	unsigned long		flags;			/* spin lock flag */
	DEBUG_MSG("%s, enter\n", __FUNCTION__);

	/**
	 * - 1. Check parameter:
	 *  - _ep != NULL, _ep->desc != NULL
	 *  - in case of error, return -EINVAL.
	 */
	if (_ep == NULL) {
		DEBUG_MSG("%s, ep not enabled\n", __FUNCTION__);
		return -EINVAL;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	if ( ep->ep_desc == NULL) {
		DEBUG_MSG("%s, %s not enabled\n", __FUNCTION__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	/**
	 * - 2. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&ep->dev->lock, flags);

	/**
	 * - 3. Initialize ep structures and change state to INIT:
	 */
	ep->ep_desc = NULL;
	ep->fifo_data = 0;
	usbc_dev = ep->dev;
	change_epx_state(ep, S1R72_GD_EP_INIT);
	DEBUG_MSG("%s, change_epx_state call\n", __FUNCTION__);
#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	S1R72_AREAxJOIN_DIS(usbc_dev, ep->ep_subname);
#endif

	/**
	 * - 4. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&ep->dev->lock, flags);


	/**
	 * - 5. Return:
	 *  - return 0;
	 */
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- allocate a request object to use with this endpoint
 * @par		usage:
 *				API(see functional specifications: 8.1.3.)
 * @param	*_ep;		endpoint structure that specifies the endpoint.
 * @param	*gfp_flags;	flags to allocate memory.
 * @retval	return pointer to usb_request.
 *			in case of error, then return NULL.
 */
/* ========================================================================= */
static struct usb_request * s1r72xxx_ep_alloc_request(struct usb_ep *_ep,
	unsigned int gfp_flags)
{
	S1R72XXX_USBC_REQ	*req;			/* USB request */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _ep != NULL
	 *  - in case of error, return -EINVAL.
	 */
	if (_ep == NULL) {
		DEBUG_MSG("%s, this ep is not enabled\n", __FUNCTION__);
		return NULL;
	}

	/**
	 * - 2. Allocate memory:
	 *  - call kmalloc. where size is s1r72xxx_request structure.
	 *  - in case of error, return NULL.
	 */
	req = kmalloc (sizeof(S1R72XXX_USBC_REQ), gfp_flags);
	if (req == NULL)
	{
		DEBUG_MSG("%s, kmalloc failed\n", __FUNCTION__);
		return NULL;
	}

	/**
	 * - 3. Initialize the allocated memory:
	 *  - call memset and use INIT_LIST_HEAD.
	 */
	memset(req, 0x00, sizeof(S1R72XXX_USBC_REQ));
	INIT_LIST_HEAD (&req->queue);

	/**
	 * - 4. Normaly return:
	 *  - return pointer to usb_request in allocated memory.;
	 */
	return &req->req;

}

/* ========================================================================= */
/**
 * @brief	- free a request object
 * @par		usage:
 *				API(see functional specifications: 8.1.4.)
 * @param	*_ep;	endpoint structure that specifies the endpoint.
 * @param	*_req;	pointer to allocated memory by xxx_ep_alloc_request.
 * @retval	none
 */
/* ========================================================================= */
void s1r72xxx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	S1R72XXX_USBC_REQ	*req;			/* USB request */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - usb_request != NULL.
	 *  - in case of error, return.
	 */
	if (_req == NULL) {
		DEBUG_MSG("%s, bad parameter\n", __FUNCTION__);
		return;
	}
	req = container_of(_req,S1R72XXX_USBC_REQ,req);	/* get usb_ep structure */

	/**
	 * - 2. Free memory:
	 *  - call kfree.
	 */
	kfree(req);

}

/* ========================================================================= */
/**
 * @brief	- queues (submits) an I/O request to an endpoint.
 * @par		usage:
 *				API(see functional specifications: 8.1.7.)
 * @param	*_ep;		endpoint structure that specifies the endpoint.
 * @param	*_req;		request to be submited
 * @param	gfp_flags;	flags to allocate memory.
 * @retval	return 0 when the queue is processed successfully.
 *			in case of error, then return error code.
 */
/* ========================================================================= */
static int s1r72xxx_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
	unsigned int gfp_flags)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;			/* USB endpoint informations */
	S1R72XXX_USBC_REQ	*req;			/* USB request */
	unsigned long		flags;			/* spin lock flag */
	int					handle_ret;		/* handle_ep() return value */

	handle_ret = S1R72_QUEUE_REMAIN;
	DEBUG_MSG("%s, enter\n", __FUNCTION__);

	S1R72_USBC_INC_QUE_NO;
	/**
	 * - 1. Check parameter:
	 *  - param != NULL, _ep is not ep0, bmAttributes, speed...
	 *  - in case of error, return -EINVAL, -ERANGE,
	 *    -ESHUTDOWN or -EMSGSIZE.
	 */
	if ( (_req == NULL) || (_req->complete == NULL) || (_req->buf == NULL) ){
		DEBUG_MSG("%s, bad params(_req)\n", __FUNCTION__);
		return -EINVAL;
	}
	req = container_of(_req, S1R72XXX_USBC_REQ, req);

	if ( _ep == NULL ){
		DEBUG_MSG("%s, bad params(_ep)\n", __FUNCTION__);
		return -EINVAL;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	usbc_dev = ep->dev;
	if (!list_empty(&req->queue)){
		return -EINVAL;
	}
	if ( (!usbc_dev->driver)
		|| (usbc_dev->gadget.speed == USB_SPEED_UNKNOWN) ){
		return -ESHUTDOWN;
	}

	S1R72_USBC_SET_QUE_NO(req);
	s1r72xxx_queue_log(S1R72_DEBUG_EP_QUEUE, ep->ep_subname, _req->length);
	s1r72xxx_queue_log(S1R72_DEBUG_EP_QUEUE_SN, ep->ep_subname,
		req->queue_seq_no);

	/**
	 * - 2. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&ep->dev->lock, flags);

	/**
	 * - 3. Set request initial values:
	 *  - _req->status = -EINPROGRESS, _req->actual = 0.
	 */
	_req->status = -EINPROGRESS;
	_req->actual = 0;
	req->send_zero = _req->zero;
	DEBUG_MSG("%s, len=0x%d, zero=%d, actual=%d\n",
		__FUNCTION__, _req->length, _req->zero, _req->actual);

	/**
	 * - 4. Check queue list:
	 *  - if queue is empty, proceed to 4.1. or 4.2
	 *  - if queue is not empty, proceed to 5.
	 */
	if ((list_empty(&ep->queue) != S1R72_LIST_ISNOT_EMPTY )
		|| (ep->is_stopped == S1R72_EP_ACTIVE)){
		DEBUG_MSG("%s, ep=%x\n", __FUNCTION__,ep->ep_subname);
		DEBUG_MSG("%s, state=%x\n", __FUNCTION__,ep->ep_state);
		/**
		 * - 4.1. Handle ep0:
		 *  - send or recieve data to/from endpoint0.(calls handle_ep_in/out())
		 */
		if (ep->ep_subname == S1R72_GD_EP0){
			if ( (ep->ep_state == S1R72_GD_EP_DIN) || (_req->length == 0) ){
				/**
				 * - 4.1.1. Handle IN ep0:
				 *  - if state is DIN, already recieved data.
				 *  - length 0 means status stage .
				 */
				handle_ret = handle_ep_in(ep, req);
			} else if ( ep->ep_state == S1R72_GD_EP_DOUT ){
				/**
				 * - 4.1.2. Handle OUT ep0:
				 *  - if state is DOUT, already recieved data.
				 */
				handle_ret = handle_ep_out(ep, req);
			} else {
				/* error case */
				spin_unlock_irqrestore(&ep->dev->lock, flags);
				return -EINVAL;
			}
		/**
		 * - 4.2. Handle epx:
		 *  - send or recieve data to/from endpoint x.(call handle_ep_in/out())
		 */
		} else {
			switch((ep->bEndpointAddress) & USB_ENDPOINT_DIR_MASK){
			case USB_DIR_IN:
				/**
				 * - 4.2.1. Handle IN epx:
				 *  - if DIR is IN, send data.
				 */
				if (ep->ep_state == S1R72_GD_EP_IDLE){
					DEBUG_MSG("%s, DIN\n", __FUNCTION__);
					change_epx_state(ep, S1R72_GD_EP_DIN);
					handle_ret = handle_ep_in(ep, req);
				}
				break;
			default:
				DEBUG_MSG("%s, DOUT\n", __FUNCTION__);
				/**
				 * - 4.2.2. Change ep state to IDLE:
				 *  - if state is IDLE, no recieve data.
				 */
				if (ep->ep_state == S1R72_GD_EP_IDLE){
					change_epx_state(ep, S1R72_GD_EP_DOUT);
				/**
				 * - 4.2.3. Handle OUT epx:
				 *  - if state is DOUT, already recieved data.
				 */
				} else if (ep->ep_state == S1R72_GD_EP_DOUT) {
					handle_ret = handle_ep_out(ep, req);
				}
				break;
			}
		}
	}

	/**
	 * - 5. Add this queue to queue list:
	 *  - if this queue isn't completed above,
	 *    then add to queue list(list_add_tail).
	 */
	if (handle_ret == S1R72_QUEUE_REMAIN) {
		if (req != list_entry(ep->queue.next, S1R72XXX_USBC_REQ , queue)){
			list_add_tail(&req->queue, &ep->queue);
			DEBUG_MSG("%s, add queue\n", __FUNCTION__);
		}
	}
	DEBUG_MSG("%s, list_empty=%x\n", __FUNCTION__, list_empty(&ep->queue));

	/**
	 * - 6. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	/**
	 * - 7. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- dequeues (cancels, unlinks) an I/O request from an endpoint.
 * @par		usage:
 *				API(see functional specifications: 8.1.8.)
 * @param	*_ep;		endpoint structure that specifies the endpoint.
 * @param	*_req;		request to be dequeued
 * @retval	return 0 when the queue is aborted successfully.
 *			in case of error, then return error code.
 */
/* ========================================================================= */
static int s1r72xxx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;			/* USB endpoint informations */
	S1R72XXX_USBC_REQ	*req;			/* USB request */
	unsigned long		flags;			/* spin lock flag */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - param != NULL, _ep is not ep0.
	 *  - in case of error, return -EINVAL.
	 */
	if ( _ep == NULL ) {
		DEBUG_MSG("%s, parameter error\n", __FUNCTION__ );
		return -EINVAL;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	req = container_of(_req, S1R72XXX_USBC_REQ, req);
	usbc_dev = ep->dev;
	s1r72xxx_queue_log(S1R72_DEBUG_EP_DEQUEUE, ep->ep_subname, _req->length);
	s1r72xxx_queue_log(S1R72_DEBUG_EP_DEQUEUE_SN, ep->ep_subname,
		req->queue_seq_no);
	DEBUG_MSG("%s, ep%d\n", __FUNCTION__, ep->ep_subname);

	/**
	 * - 2. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&ep->dev->lock, flags);

	/**
	 * - 3. Find available queue:
	 *  - find the _req from _ep->queue.
	 *  - if available _req isn't found, enables IRQ and return -EINVAL.
	 */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req){
			break;
		}
	}
	if (&req->req != _req) {
		DEBUG_MSG("%s, queue is not exist\n", __FUNCTION__);
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	/**
	 * - 4. Dequeue the remaining data:
	 *  - delete queue from list.(calls request_done() with -ECONNRESET)
	 */
	if (req->req.actual != 0){
		S1R72_EPFIFO_CLR(ep->dev, ep->ep_subname);
		clear_ep_enshortpkt(usbc_dev, ep);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/**
		 * - 4.1. clear FIFO addup:
		 */
		ep->fifo_addup = FIFO_ADDUP_CLR;
#endif
	}
	request_done(ep, req, -ECONNRESET);

	/**
	 * - 5. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	/**
	 * - 6. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- sets the endpoint halt feature.
 * @par		usage:
 *				API(see functional specifications: 8.1.9.)
 * @param	*_ep;	endpoint structure that specifies the endpoint.
 * @param	value;	flag indicates to set halt or clear halt.
 * @retval	return 0 when the ep is halted successfully.
 *			in case of error, then return error code.
 */
/* ========================================================================= */
static int s1r72xxx_ep_set_halt(struct usb_ep *_ep, int value)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;			/* USB endpoint informations */
	unsigned long		flags;			/* spin lock flag */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - param != NULL, _ep is not ep0, value is 0 or 1.
	 *  - in case of error, return -EINVAL.
	 */
	if ( (_ep == NULL)
		|| (value > S1R72_SET_STALL) || (value < S1R72_CLR_STALL)){
		DEBUG_MSG("%s, bad ep1\n", __FUNCTION__);
		return -EINVAL;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	usbc_dev = ep->dev;

	/**
	 * - 2. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&ep->dev->lock, flags);

	/**
	 * - 3. Check a param 'value':
	 *  - if 'value' is 1, then process 3.1.1 . else 3.2.1 .
	 */
	if (value == S1R72_SET_STALL) {
		DEBUG_MSG("%s, set halt\n", __FUNCTION__);
		/**
		 * - 3.1.1. Check remaining data:
		 *  - queue is remaining, return error.
		 */
		if ( list_empty(&ep->queue) == S1R72_LIST_ISNOT_EMPTY) {
			DEBUG_MSG("%s, -EAGAIN\n", __FUNCTION__);
			spin_unlock_irqrestore(&ep->dev->lock, flags);
			return -EAGAIN;
		}

		/**
		 * - 3.1.2. Force set 'STALL' to device:
		 *  - set hardware register to return 'stall'.(D_EPxControl.ForceSTALL)
		 */
		ep->is_stopped = S1R72_EP_STALL;
		if (ep->ep_subname == S1R72_GD_EP0){
			rcD_RegClear8(usbc_dev, ep->reg.ep0.SETUP_Control,
				S1R72_ProtectEP0);
			rcD_RegWrite8(usbc_dev, ep->reg.ep0.EP0ControlIN,
				 S1R72_ForceSTALL);
			rcD_RegWrite8(usbc_dev, ep->reg.ep0.EP0ControlOUT,
				 S1R72_ForceSTALL);
			rcD_RegSet8(usbc_dev, ep->reg.ep0.SETUP_Control,
				S1R72_ProtectEP0);
		} else {
			rcD_RegSet8(usbc_dev, ep->reg.epx.EPxControl, S1R72_ForceSTALL);
		}
	} else {
		DEBUG_MSG("%s, clear halt\n", __FUNCTION__);
		/**
		 * - 3.2.1. Force clear 'STALL' to device:
		 *  - set hardware register to turn nomal state.
		 *    D_EPxControl.ForceSTALL
		 */
		ep->is_stopped = S1R72_EP_ACTIVE;
		if (ep->ep_subname == S1R72_GD_EP0){
			rcD_RegClear8(usbc_dev, ep->reg.ep0.SETUP_Control,
				S1R72_ProtectEP0);
			rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0ControlIN,
				S1R72_ForceSTALL);
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0ControlIN,
				S1R72_ToggleClr);
			rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0ControlOUT,
				S1R72_ForceSTALL);
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0ControlOUT,
				S1R72_ToggleClr);
			rcD_RegSet8(usbc_dev, ep->reg.ep0.SETUP_Control,
				S1R72_ProtectEP0);
		} else {
			rcD_RegSet8(usbc_dev, ep->reg.epx.EPxControl, S1R72_ToggleClr);
			rcD_RegClear8(usbc_dev, ep->reg.epx.EPxControl, S1R72_ForceSTALL);
		}
	}
	/**
	 * - 4. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	/**
	 * - 5. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- return number of bytes in fifo.
 * @par		usage:
 *				API(see functional specifications: 8.1.10.)
 * @param	*_ep;		endpoint structure that specifies the endpoint.
 * @retval	return number of bytes in fifo.
 *			in case of error, then return error code.
 */
/* ========================================================================= */
static int s1r72xxx_ep_fifo_status(struct usb_ep *_ep)
{
	S1R72XXX_USBC_DEV	*usbc_dev;	/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;		/* USB endpoint informations */
	int		fifo_remain;			/* fifo remaining data */
	unsigned short	fifo_addr;		/* fifo address */
	unsigned short	join_addr;		/* join address */
	unsigned char	inout;			/* ep0 in or out */
	int		ret_value;				/* return value */
	unsigned char	chk_ret;				/* return value */
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	unsigned short	dummy_read;
#endif

	fifo_addr = 0;
	join_addr = 0;
	inout = 0;

	/**
	 * - 1. Check parameter:
	 *  - _ep is available...
	 *  - in case of error, return -EINVAL.
	 */
	if (_ep == NULL) {
		DEBUG_MSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	usbc_dev = ep->dev;

	/**
	 * - 2. Check direction at endpoint:
	 *  - if direction is 'IN' then process 3.1 .
	 *  - if direction is 'OUT' then process 3.2 .
	 */
	S1R72_EPJOINCPU_CLR(usbc_dev);

	if (ep->ep_subname == S1R72_GD_EP0) {
		join_addr = ep->reg.ep0.EP0Join;
		inout = rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0Control)
			& S1R72_EP0INxOUT;
	} else {
		join_addr = ep->reg.epx.EPxJoin;
		inout = (unsigned char)(ep->ep_desc->bEndpointAddress
			& USB_ENDPOINT_DIR_MASK);
	}

	if ( (inout == S1R72_EP0INxOUT_IN) || (inout == USB_DIR_IN) ){
		/**
		 * - 3.1. Read registers FIFO_WrRemain_H/L:
		 *  - number of data in fifo is wMaxPacketSize - FIFO_WrRemain_H/L.
		 */
		rcD_RegSet8(usbc_dev, join_addr, S1R72_JoinCPU_Wr);
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/* wait for coming */
		dummy_read = rsD_RegRead16(usbc_dev, rsFIFO_WrRemain);
#endif
		fifo_remain = rsD_RegRead16(usbc_dev, rsFIFO_WrRemain);
		ret_value = epx_max_packet_tbl[ep->ep_subname] - fifo_remain;
	} else {
		/**
		 * - 3.2. Read registers FIFO_RdRemain_H/L:
		 *  - number of data in fifo is FIFO_RdRemain_H/L.
		 */
		rcD_RegSet8(usbc_dev, join_addr, S1R72_JoinCPU_Rd);
		chk_ret = check_fifo_remain(usbc_dev);
		if (chk_ret == S1R72_RD_REMAIN_NG){
			fifo_remain = S1R72_FIFO_EMPTY;
		} else {
			fifo_remain = rsD_RegRead16(usbc_dev, rsFIFO_RdRemain);
		}
		ret_value = fifo_remain & (~RdRemainValid);
	}
	S1R72_EPJOINCPU_CLR(usbc_dev);

	/**
	 * - 4. Return:
	 *  - return number of data.
	 */
	return ret_value;

}

/* ========================================================================= */
/**
 * @brief	- flushes contents of a fifo.
 * @par		usage:
 *				API(see functional specifications: 8.1.11.)
 * @param	*_ep;	endpoint structure that specifies the endpoint.
 * @retval	none
 */
/* ========================================================================= */
static void s1r72xxx_ep_fifo_flush(struct usb_ep *_ep)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_EP	*ep;			/* USB endpoint informations */
	/**
	 * - 1. Check parameter:
	 *  - param != NULL, _ep is available...
	 *  - in case of error, return.
	 */
	if (_ep == NULL){
		DEBUG_MSG("%s, bad ep\n", __FUNCTION__);
		return;
	}
	ep = container_of(_ep, S1R72XXX_USBC_EP, ep);	/* get usb_ep structure */
	usbc_dev = ep->dev;
	if ( list_empty(&ep->queue) == S1R72_LIST_ISNOT_EMPTY ) {
		DEBUG_MSG("%s, bad ep=%d\n", __FUNCTION__, ep->ep_subname);
		return;
	}

	/**
	 * - 2. Clear endpoint FIFO:
	 *  - D_EPrFIFO_Clr
	 */
	S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);
	if (ep->ep_subname == S1R72_GD_EP0){
		change_ep0_state(ep, S1R72_GD_EP_IDLE);
	} else {
		change_epx_state(ep, S1R72_GD_EP_IDLE);
	}
	ep->fifo_data = 0;

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	ep->fifo_addup = FIFO_ADDUP_CLR;
#endif
}

/* ========================================================================= */
/**
 * @brief	- return the current frame number.
 * @par		usage:
 *				API(see functional specifications: 8.2.1.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @retval	USB frame number(SOF)
 */
/* ========================================================================= */
static int s1r72xxx_usbc_get_frame(struct usb_gadget *_gadget)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	int		frame_number;				/* USB frame number */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if ( _gadget == NULL ) {
		DEBUG_MSG("%s, bad gadget\n", __FUNCTION__);
		return -EINVAL;
	}
	usbc_dev = container_of(_gadget, S1R72XXX_USBC_DEV, gadget);

	/**
	 * - 2. return frame number:
	 *  - return D_FrameNumber_H/L.
	 */
	frame_number = rsD_RegRead16(usbc_dev, rsD_S1R72_FrameNumber)
		& S1R72_FrameNumber;
	DEBUG_MSG("%s, frame =0x%08x\n", __FUNCTION__,frame_number);
	return(frame_number);
}

/* ========================================================================= */
/**
 * @brief	- tries to wake up the host connected to this gadget.
 * @par		usage:
 *				API(see functional specifications: 8.2.2.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @retval	return 0 when the device was waked up.
 *			in case of error, then return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_wakeup(struct usb_gadget *_gadget)
{
	S1R72XXX_USBC_DEV	*usbc_dev;			/* USB hardware informations */
	unsigned char		dummy;				/* dummy data */
	DEBUG_MSG("%s, enter\n", __FUNCTION__);

	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if ( _gadget == NULL ) {
		DEBUG_MSG("%s, bad gadget\n", __FUNCTION__);
		return -EINVAL;
	}
	usbc_dev = container_of(_gadget, S1R72XXX_USBC_DEV, gadget);

	if ( usbc_dev->remote_wakeup_enb != S1R72_RT_WAKEUP_ENB) {
		DEBUG_MSG("%s,remote wakeup is not enabled\n", __FUNCTION__);
		return -EAGAIN;
	}

	if ( usbc_dev->usbcd_state != S1R72_GD_USBSUSPD ) {
		DEBUG_MSG("%s, state is not USB SUSPEND\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Set remote wakeup flag:
	 */
	usbc_dev->remote_wakeup_prc = S1R72_RT_WAKEUP_PROC;

	/**
	 * - 3. Set device interrupt:
	 *  - Enable FinishedPM and clear NonJ interrupt.
	 *  - USB device sends K signal to USB host, so NonJ interrupt
	 *  -  is not needed.
	 */
	dummy = rcD_RegRead8(usbc_dev, rcMainIntEnb);	/* escape CPU_Cut */
	rcD_RegSet8(usbc_dev, rcMainIntEnb, FinishedPM);
	rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);
	rcD_RegClear8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_EnNonJ);
	rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, S1R72_NonJ);

	/**
	 * - 4. Wake up hardware:
	 */
	usbc_dev->usbcd_state = change_driver_state(usbc_dev,
		usbc_dev->usbcd_state, S1R72_GD_API_RESUME);

	/**
	 * - 5. Return:
	 *  - return 0.
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- sets the device selfpowered feature.
 * @par		usage:
 *				API(see functional specifications: 8.2.3.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @param	value;		'1' means set to self powerd state, '0' means another.
 * @retval	return 0 when the device was turned to the state.
 *			in case of error or not supported state, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_set_selfpowered(struct usb_gadget *_gadget, int value)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL. if value != 1, then return -EOPNOTSUPP.
	 *  - this driver is made to selfpowered-only system.
	 */
	if ( (_gadget == NULL) || (value != 1) ) {
		DEBUG_MSG("%s, bad param\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Return:
	 * - return 0.
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- Notify controller that VBUS is powered.
 * @par		usage:
 *				API(see functional specifications: 8.2.4.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @param	value;		'1' means set to VBUS supplied, '0' means another.
 * @retval	return 0 when the device was turned to the state.
 *			in case of error or not supported state, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_vbus_session(struct usb_gadget *_gadget, int value)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	unsigned long		flags;			/* spin lock flag */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL.
	 *  - if value != 0 or 1, then return -EINVAL.
	 */
	if ( (_gadget == NULL) || (value < 0) || (value > 1)) {
		DEBUG_MSG("%s, bad param\n", __FUNCTION__);
		return -EINVAL;
	}
	usbc_dev = container_of(_gadget, S1R72XXX_USBC_DEV, gadget);

	/**
	 * - 2. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 3. Check VBUS state:
	 *  - check VBUS state and change device state.
	 */
	usbc_irq_VBUS(usbc_dev);

	/**
	 * - 4. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	/**
	 * - 5. Return:
	 *  - return 0.
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- constrain controller's VBUS power usage.
 * @par		usage:
 *				API(see functional specifications: 8.2.5.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @param	mA;			how much current draw.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_vbus_draw(struct usb_gadget *_gadget, unsigned mA)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL.
	 */
	if ( _gadget == NULL )  {
		DEBUG_MSG("%s, bad param\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Return:
	 *  - return -EOPNOTSUPP.
	 *  - this system does not support this function.
	 */
	return -EOPNOTSUPP;
}

/* ========================================================================= */
/**
 * @brief	- software-controlled connect to USB host.
 * @par		usage:
 *				API(see functional specifications: 8.2.6.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @param	value;		indicates pullup or not.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_pullup(struct usb_gadget *_gadget, int value)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL.
	 *  - if value != 0 or 1, then return -EINVAL.
	 */
	if ( (_gadget == NULL) || (value < 0) || (value > 1)) {
		DEBUG_MSG("%s, bad param\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Pullup D+(or D-):
	 *  - Pullup D+(or D-).
	 *  - calls usbc_pullup()
	 */
	usbc_pullup(value);

	/**
	 * - 3. Return:
	 *  - return 0.
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- IOCTL function. this inplements functions that isn't
 *			included other APIs.
 * @par		usage:
 *				API(see functional specifications: 8.2.7.)
 * @param	*_gadget;	usb gadget structure that specifies the 'USB Device'.
 * @param	code;		control code.
 * @param	param;		data.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_ioctl (struct usb_gadget *_gadget, unsigned code,
	unsigned long param)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - _gadget != NULL.
	 *  - return -EINVAL.
	 */
	if ( _gadget == NULL )  {
		DEBUG_MSG("%s, bad param\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Return:
	 *  - return -ENOTTY.
	 *  - this driver does not support any ioctl.
	*/
	return -ENOTTY;
}

/* ========================================================================= */
/**
 * @brief	- initialize driver and register irq_handler.
 * @par		usage:
 *				API(see functional specifications: 8.3.1.)
 * @param	*dev;	device structure that specifies device drivers.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int __init s1r72xxx_usbc_probe(struct device *dev)
{
	S1R72XXX_USBC_DEV		*usbc_dev;	/* USB hardware informations */
	struct platform_device*	pdev;		/* platform device structure */
	struct resource*		res;		/* platform device resource */
	unsigned char			ep_counter;	/* endpoint counter */
	int						retval;		/* returned value */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - dev != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if ( dev == NULL )  {
		DEBUG_MSG("%s, bad param\n", __FUNCTION__);
		return -EINVAL;
	}
	pdev = to_platform_device(dev);

	/**
	 * - 2. Allocate memory for device infomation structure:
	 *  - call kmalloc. where size is s1r72xxx_usbc_dev, flag is SLAB_KERNEL.
	 *  - in case of error, return -ENOMEM.
	 */
	usbc_dev = kmalloc(sizeof(S1R72XXX_USBC_DEV), GFP_KERNEL);
	if ( usbc_dev == NULL ) {
		DEBUG_MSG("%s, kmalloc error\n", __FUNCTION__);
		return -ENOMEM;
	}
	the_controller = usbc_dev;

	/**
	 * - 3. Initialize the allocated memory:
	 *  - 0 clear allocated memory using memset.
	 */
	memset(usbc_dev, 0x00, sizeof(S1R72XXX_USBC_DEV));

	/**
	 * - 4. Set values to member of usb_gadget structure:
	 *  - gadget.ops, gadget.ep0, gadget.name, gadget.dev.bus_id="gadget"
	 *    , gadget.dev.release, gadget.eplist
	 */
	usbc_dev->gadget.ops			= &s1r72xxx_gadget_ops;
	usbc_dev->gadget.ep0			= &usbc_dev->usbc_ep[S1R72_GD_EP0].ep;
	usbc_dev->gadget.name			= driver_name;
	usbc_dev->gadget.is_dualspeed	= 1;
	strcpy(usbc_dev->gadget.dev.bus_id, "gadget");
	usbc_dev->gadget.dev.release	= s1r72xxx_usbc_release;

	usbc_dev->remote_wakeup_prc		= S1R72_RT_WAKEUP_NONE;
	usbc_dev->remote_wakeup_enb		= S1R72_RT_WAKEUP_DIS;
	usbc_dev->usbcd_state			= S1R72_GD_INIT;
	usbc_dev->next_usbcd_state		= S1R72_GD_DONT_CHG;
	usbc_dev->previous_usbcd_state = S1R72_GD_DONT_CHG;
	INIT_LIST_HEAD(&usbc_dev->gadget.ep_list);
	INIT_LIST_HEAD(&usbc_dev->gadget.ep0->ep_list);
	for ( ep_counter = 0 ; ep_counter < S1R72_MAX_ENDPOINT ; ep_counter++) {
		if ( ep_counter != S1R72_GD_EP0 ) {
			list_add_tail(	&usbc_dev->usbc_ep[ep_counter].ep.ep_list,
				&usbc_dev->gadget.ep_list);
		}
		usbc_dev->usbc_ep[ep_counter].ep_desc = NULL;
		INIT_LIST_HEAD(&usbc_dev->usbc_ep[ep_counter].queue);
	}

	/**
	 * - 5. Get address to hardware:
	 *  - get hardware base address.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	usbc_dev->base_addr = (void*)ioremap(res->start,res->end-res->start);
	printk(KERN_DEBUG "vir add is 0x %x , phy add is 0x %x", usbc_dev->base_addr, res->start);
	if(!usbc_dev->base_addr){
		kfree(usbc_dev);
		DEBUG_MSG("ioremap error \n");
		return -ENODEV;
	}

	/**
	 * - 6. Initialize hardware:
	 *  - initialize hardware registers.
	 */
	s1r72xxx_queue_log(S1R72_DEBUG_PROBE, 0, 0);
	retval = usbc_lsi_init(usbc_dev);
	if (retval != S1R72_RET_OK){
		rcD_IntClr8(usbc_dev, rcMainIntStat, (DevInt | FinishedPM));
		rcD_RegSet8(usbc_dev, rcMainIntEnb, (DevInt | FinishedPM));
		rcD_IntClr8(usbc_dev, rcDeviceIntStat,
			(D_VBUS_Changed | D_SIE_IntStat));
		rcD_RegWrite8(usbc_dev, rcDeviceIntEnb,
			(D_VBUS_Changed | D_SIE_IntStat));
		kfree(usbc_dev);
		iounmap(usbc_dev->base_addr);
		return retval;
	}
	usbc_ep_init(usbc_dev);
	printk("%s: ioaddr: 0x%04X, Chip Rev: 0x%04X \n", driver_name,
	(unsigned)usbc_dev->base_addr, rcD_RegRead8(usbc_dev, rcRevisionNum ));

	/**
	 * - 7. Initialize device structure:
	 *  - using device_initialize(), initialize the device structure.
	 */
	device_initialize(&usbc_dev->gadget.dev);

	/**
	 * - 8. Set values related to member of device structure:
	 *  - ->dev, gadget.dev.parent, gadget.dev.release, gadget.dev.driver_data
	 */
	usbc_dev->dev					= dev;
	usbc_dev->gadget.dev.parent		= dev;
	usbc_dev->gadget.dev.dma_mask	= dev->dma_mask;
	dev_set_drvdata(dev, usbc_dev);
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_PROBE);

	/**
	 * - 9. Initialize timer:
	 */
	init_timer(&usbc_dev->rm_wakeup_timer);
	usbc_dev->rm_wakeup_timer.function = remote_wakeup_timer;
	usbc_dev->rm_wakeup_timer.data = (unsigned long)usbc_dev;
	init_timer(&usbc_dev->wait_j_timer);
	usbc_dev->wait_j_timer.function = chirp_cmp_timer;
	usbc_dev->wait_j_timer.data = (unsigned long)usbc_dev;

	init_timer(&usbc_dev->vbus_timer);
	usbc_dev->vbus_timer.function = vbus_timer;
	usbc_dev->vbus_timer.data = (unsigned long)usbc_dev;
	usbc_dev->vbus_timer_state = S1R72_VBUS_TIMER_STOPPED;

	/**
	 * - 10. Register IRQ handler to kernel:
	 *  - register IRQ handler to kernel using request_irq() with SA_SHIRQ.
	 *  - if fault request_irq, then return -EBUSY.
	 */
	res  = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	retval = request_irq(res->start, s1r72xxx_usbc_irq, IRQF_SHARED,
		driver_name, usbc_dev);
	if (retval != 0) {
		DEBUG_MSG("%s: can't get irq %i, err %d\n",
			driver_name, (int)res->start, retval);
		s1r72xxx_usbc_remove(dev);
		return -EBUSY;
	}
	create_proc_file();

	/**
	 * - 11. Return:
	 *  - return 0;
	 */
	printk(KERN_DEBUG "exit s1r72xx probe");
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- remove driver from kernel.
 * @par		usage:
 *				API(see functional specifications: 8.3.2.)
 * @param	*dev;	device structure that specifies device drivers.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_remove(struct device *dev)
{
	S1R72XXX_USBC_DEV		*usbc_dev;	/* USB hardware informations */
	struct platform_device*	pdev;		/* platform device structure */
	struct resource*		res;		/* platform device resource */
	unsigned long			flags;		/* spin lock flag */

	usbc_dev = the_controller;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check parameter:
	 *  - dev != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if ( ( dev == NULL) || (dev->driver_data == NULL) ) {
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Change driver state:
	 */
	rcD_RegClear8(usbc_dev, rcMainIntEnb, (DevInt | FinishedPM));
	rcD_RegWrite8(usbc_dev, rcDeviceIntEnb, S1R72_REG_ALL_CLR);
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_REMOVE);

	/**
	 * - 3. Unregister gadget driver from kernel:
	 */
	if (usbc_dev->driver != NULL){
		usb_gadget_unregister_driver(usbc_dev->driver);
	}

	/**
	 * - 4. Disable IRQ:
	 *  - call spin_unlock_irqresave, disable rcMainIntEnb.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 5. Unregister IRQ handler from kernel:
	 *  - using free_irq, unregister IRQ handler.
	 */
	pdev = to_platform_device(dev);
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
		S1R72_DEVICE_NAME);
	free_irq(res->start, usbc_dev);

	/**
	 * - 5.1. Delete VBUS timer handler:
	 */
	del_timer_sync(&usbc_dev->vbus_timer);
	usbc_dev->vbus_timer_state = S1R72_VBUS_TIMER_STOPPED;

	/**
	 * - 5.2. Delete Wait J timer handler:
	 */
	del_timer_sync(&usbc_dev->wait_j_timer);

	/**
	 * - 5.3. Delete Remote Wakeup timer handler:
	 */
	del_timer_sync(&usbc_dev->rm_wakeup_timer);

	/**
	 * - 6. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	/**
	 * - 7. Free allocated memory at probe:
	 *  - using kfree, free allocated memory.
	 */
	iounmap(usbc_dev->base_addr);
	kfree(usbc_dev);

	/**
	 * - 8. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- system shutdown.
 * @par		usage:
 *				API(see functional specifications: 8.3.3.)
 * @param	*dev;	device structure that specifies device drivers.
 * @retval	none
 */
/* ========================================================================= */
void s1r72xxx_usbc_shutdown(struct device *dev)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	unsigned long		flags;			/* spin lock flag */

	usbc_dev = the_controller;
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	s1r72xxx_queue_log(S1R72_DEBUG_SHUTDOWN, 0, 0);

	/**
	 * - 1. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 2. Disable interrupt:
	 */
	rcD_RegClear8(usbc_dev, rcMainIntEnb, (DevInt | FinishedPM));
	rcD_RegWrite8(usbc_dev, rcDeviceIntEnb, S1R72_REG_ALL_CLR);
	remove_proc_file();

	/**
	 * - 2.1. Delete VBUS timer handler:
	 */
	del_timer_sync(&usbc_dev->vbus_timer);
	usbc_dev->vbus_timer_state = S1R72_VBUS_TIMER_STOPPED;

	/**
	 * - 2.2. Delete Wait J timer handler:
	 */
	del_timer_sync(&usbc_dev->wait_j_timer);

	/**
	 * - 2.3. Delete Remote Wakeup timer handler:
	 */
	del_timer_sync(&usbc_dev->rm_wakeup_timer);

	/**
	 * - 3. Change state:
	 */
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_SHUTDOWN);

	/**
	 * - 4. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	/**
	 * - 5. free allocated area:
	 */
	kfree(usbc_dev);
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
}

/* ========================================================================= */
/**
 * @brief	- system suspend.
 * @par		usage:
 *				API(see functional specifications: 8.3.4.)
 * @param	*dev;	device structure that specifies device drivers.
 * @param	state;	suspend state.
 * @param	level;	suspend level.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_suspend(struct device *dev, pm_message_t state)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	unsigned int		pm_change_ct;	/* PM_Control change wait */
	unsigned long		flags;			/* spin lock flag */
	unsigned char		dummy;			/* dummy */

	usbc_dev = the_controller;
	pm_change_ct = 0;
	DEBUG_MSG("%s, enter\n", __FUNCTION__);

	/**
	 * - 1. Check parameter:
	 *  - dev != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if ( dev == NULL) {
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Disable interrupt:
	 */
	dummy = rcD_RegRead8(usbc_dev, rcPM_Control_0);
	rcD_RegClear8(usbc_dev, rcMainIntEnb, FinishedPM);

	/**
	 * - 3. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 4. Change state:
	 */
	usbc_dev->previous_usbcd_state = usbc_dev->usbcd_state;
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_SUSPEND);

	/**
	 * - 5. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	/**
	 * - 6. Enable interrupt:
	 */
	while( (rcD_RegRead8(usbc_dev, rcMainIntStat) & FinishedPM)
		!= FinishedPM){
		pm_change_ct ++;
		if ( (pm_change_ct > S1R72_PM_CHANGE_TIMEOUT)
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
			|| ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
				== S1R72_PM_State_SLEEP_01)
#endif
			|| ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
			== S1R72_PM_State_SLEEP)){
			break;
		}
	}
	rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);
	rcD_RegSet8(usbc_dev, rcMainIntEnb, FinishedPM);

#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	/**
	 * - 7. Change device state to CPU Cut mode:
	 */
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
	if (( (rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)== SLEEP)
	  ||( (rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)== SLEEP_01)){
#else
	if ( (rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
		== S1R72_PM_State_SLEEP){
#endif
		rcD_RegWrite8(usbc_dev, rcPM_Control_0, GoCPU_Cut);
	}
#endif
	/**
	 * - 8. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- system resume.
 * @par		usage:
 *				API(see functional specifications: 8.3.5.)
 * @param	*dev;	device structure that specifies device drivers.
 * @param	level;	resume level.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int s1r72xxx_usbc_resume(struct device *dev)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	unsigned long		flags;			/* spin lock flag */
	unsigned int		retval;			/* return value */
	unsigned char		dummy;			/* dummy */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	usbc_dev = the_controller;

	/**
	 * - 1. Check parameter:
	 *  - dev != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if ( dev == NULL) {
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}
	if ( usbc_dev->usbcd_state != S1R72_GD_SYSSUSPD){
		DEBUG_MSG("%s, driver is not in suspend\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 3. Change state:
	 */
	dummy = rcD_RegRead8(usbc_dev, rcPM_Control_0);
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_RESUME);
	if(usbc_dev->driver == NULL){
		usbc_dev->usbcd_state = S1R72_GD_REGD;
	}

	/**
	 * - 4. Check previous state:
	 *  - if previous state is USB suspend, only do changing state.
	 *  - if previous state is not USB suspend, call usbc_irq_VBUS.
	 */
	if ( ( usbc_dev->usbcd_state == S1R72_GD_BOUND)
			&& (usbc_dev->previous_usbcd_state != S1R72_GD_USBSUSPD) ){
		retval = usbc_irq_VBUS(usbc_dev);
#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/* change to CPU Cut mode */
		if ( (retval == S1R72_PM_CHANGE_TO_SLEEP)
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
			&& ( ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
			     == S1R72_PM_State_SLEEP)
			   ||((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
			     == S1R72_PM_State_SLEEP_01) )) {
#else
			&& ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
			== S1R72_PM_State_SLEEP) ) {
#endif
			rcD_RegWrite8(usbc_dev, rcPM_Control_0, GoCPU_Cut);
		}
#endif
	} else {
		usbc_dev->usbcd_state = S1R72_GD_USBSUSPD;

#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/* change to CPU Cut mode */
		rcD_RegWrite8(usbc_dev, rcPM_Control_0, GoCPU_Cut);
#endif
	}
	usbc_dev->previous_usbcd_state = S1R72_GD_DONT_CHG;

	/**
	 * - 5. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	/**
	 * - 6. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;
}

#ifdef CONFIG_MACH_SONY_PRS505
static void s1r72v_start_gpio_setting(void)
{
	unsigned long flags;
	DEBUG_MSG("enter set gpio");

	local_irq_save(flags);

	__REG(IMX_EIM_BASE + 0x20) = 0x00000900;
	__REG(IMX_EIM_BASE + 0x24) = 0x54540d01;

	imx_gpio_mode(GPIO_PORTC|GPIO_GIUS|GPIO_IN|GPIO_AIN|GPIO_AOUT|GPIO_BOUT|3);
	imx_gpio_mode(GPIO_PORTB|GPIO_GIUS|GPIO_IN|GPIO_AIN|GPIO_AOUT|GPIO_BOUT|19);   //USB_VBUS
#ifdef CONFIG_EBOOK5_LED
		imx_gpio_mode(GPIO_PORTB|GPIO_GIUS|GPIO_OUT | GPIO_DR |8);
		usbtg_led(EBOOK5_LED_OFF);
#endif /* CONFIG_EBOOK5_LED */

	imx_gpio_mode(GPIO_PORTA|GPIO_GIUS|GPIO_OUT|GPIO_DR|GPIO_PUEN|6);  //usb charge

//	vbus = dragonball_gpio_get_bit(USBD_S1R72_GPIO_PORT, pin);

	/* set interrupt trigger mode */
	if(set_irq_type((IRQ_GPIOC(3)), IRQF_TRIGGER_LOW)){
		printk("cannot set type\n");
		return ;
	}

	return;
	/* enable VBUS interrupt */
//	dragonball_gpio_unmask_intr(USBD_S1R72_GPIO_PORT, pin);
}
#endif

/* ========================================================================= */
/**
 * @brief	- register a driver structure to kernel.
 * @par		usage:
 *				API(see functional specifications: 8.3.6.)
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int __init USBC_INIT(void)
{
	struct device_driver *pdev;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. register a driver structure and return:
	 *  - register a driver sturcture calling by driver_register() and return.
	 */
	printk("%s module init\n",s1r72xxx_usbcd_driver.name);
	if((pdev = driver_find(s1r72xxx_usbcd_driver.name, s1r72xxx_usbcd_driver.bus))
		!= NULL){

		/**
		 * - 1.1. put driver:
		 */
		put_driver(pdev);
		return -EBUSY;
	}

#ifdef CONFIG_MACH_SONY_PRS505
	s1r72v_start_gpio_setting();
#endif
	return driver_register(&s1r72xxx_usbcd_driver);
}

/* ========================================================================= */
/**
 * @brief	- unregister a driver structure from kernel.
 * @par		usage:
 *				API(see functional specifications: 8.3.7.)
 * @retval	none.
 */
/* ========================================================================= */
static void __exit USBC_EXIT(void)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. unregister a driver structure and return:
	 *  - unregister a driver sturcture calling by driver_unregister()
	 *    and return.
	 */
	printk("%s module exit\n",s1r72xxx_usbcd_driver.name);
	driver_unregister(&s1r72xxx_usbcd_driver);
}

/* ========================================================================= */
/**
 * @brief	- register a gadget driver to usbcd.
 * @par		usage:
 *				API(see functional specifications: 8.3.8.)
 * @param	*driver;	usb_gadget_driver structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	S1R72XXX_USBC_DEV	*usbc_dev;	/* USB hardware informations */
	int					retval;		/* returned value */

	usbc_dev = the_controller;

	/**
	 * - 1. Check parameter:
	 *  - driver, driver->bind, driver->unbind, driver->disconnect
	 *    driver->setup != NULL.
	 *  - in case of error, return -EINVAL.
	 */
	if (driver)
		printk("driver->bind = %p, driver->unbind = %p,"
				"driver->disconnect = %p, driver->setup = %p\n", driver->bind, driver->unbind,
				driver->disconnect, driver->setup);
	if ( (driver == NULL)
		|| (driver->bind == NULL)
		|| (driver->disconnect == NULL) || (driver->setup == NULL)){
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}

	/**
	 * - 2. Set some information to device infomation structure:
	 *  - dev->driver, dev.gadget.dev.driver.
	 */
	usbc_dev->driver = driver;
	usbc_dev->gadget.dev.driver = &driver->driver;
	device_add(&usbc_dev->gadget.dev);

	/**
	 * - 3. Bind to gadget driver:
	 *  - call bind() to bind gadget driver. in case of error, return error
	 *    code from bind().
	 */
	retval = driver->bind(&usbc_dev->gadget);
	if (retval) {
		DEBUG_MSG("bind to driver %s --> error %d\n",
			 driver->driver.name, retval);
		device_del(&usbc_dev->gadget.dev);

		usbc_dev->driver = 0;
		usbc_dev->gadget.dev.driver = 0;
		return retval;
	}

	/**
	 * - 4. Create attribute file:
	 *  - call device_create_file().
	 */
	device_create_file(usbc_dev->dev, &dev_attr_function);

	/**
	 * - 5. Change state to BOUND:
	 */
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_REGISTER);

	/**
	 * - 6. Check VBUS level:
	 *  - if VBUS = High, must start handshake.
	 */
	retval = usbc_irq_VBUS(usbc_dev);
#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	if ( (retval == S1R72_PM_CHANGE_TO_SLEEP)
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
		&& ( ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
		     == S1R72_PM_State_SLEEP)
		   ||((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
		     == S1R72_PM_State_SLEEP_01) )) {
#else
		&& ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
		== S1R72_PM_State_SLEEP) ) {
#endif
		rcD_RegWrite8(usbc_dev, rcPM_Control_0, GoCPU_Cut);
	}
#endif
	/**
	 * - 7. Return:
	 *  - return 0;
	 */
	return 0;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

/* ========================================================================= */
/**
 * @brief	- unregister a gadget driver to usbcd.
 * @par		usage:
 *				API(see functional specifications: 8.3.9.)
 * @param	*driver;	usb_gadget_driver structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	S1R72XXX_USBC_DEV	*usbc_dev;	/* USB hardware informations */
	unsigned char		reg_value;	/* register value */
	unsigned long		flags;		/* spin lock flag */

	s1r72xxx_queue_log(S1R72_DEBUG_UNREG_DRV, 0, 0);
	usbc_dev = the_controller;
	reg_value = S1R72_REG_ALL_CLR;

	/**
	 * - 1. Check parameter:
	 *  - s1r72xxx_usbc_dev != NULL, driver != NULL.
	 *  - in case of error, return -ENODEV or -EINVAL.
	 */
	if (driver == NULL){
		DEBUG_MSG("%s, driver is NULL\n", __FUNCTION__);
		return -EINVAL;
	}
	if ( driver != usbc_dev->driver ){
		DEBUG_MSG("%s, pointer error\n", __FUNCTION__);
		return -EINVAL;
	}
	if (usbc_dev == NULL){
		DEBUG_MSG("%s, usbc error\n", __FUNCTION__);
		return -ENODEV;
	}

	/**
	 * - 2. Delete all request:
	 *  - call request_done.
	 */
	usbc_dev->gadget.speed = USB_SPEED_UNKNOWN;
	all_request_done(usbc_dev, -ESHUTDOWN);

	/**
	 * - 3. Disconnect to gadget driver:
	 *  - call disconnect().
	 */
	driver->disconnect(&usbc_dev->gadget);
	usb_disconnected(usbc_dev);

	/**
	 * - 4. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 5. Change state:
	 *  - change driver state and ep0 state.
	 */
	usbc_dev->usbcd_state
		= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
			S1R72_GD_API_UNREGISTER);
	change_ep0_state(&usbc_dev->usbc_ep[S1R72_GD_EP0], S1R72_GD_EP_INIT);

	/**
	 * - 6. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	/**
	 * - 7. Unbind from gadget driver:
	 *  - call unbind() from unbind gadget driver.
	 */
	driver->unbind(&usbc_dev->gadget);

	/**
	 * - 8. Delete attribute file:
	 *  - call device_remove_file().
	 */
	device_del(&usbc_dev->gadget.dev);
	device_remove_file(usbc_dev->dev, &dev_attr_function);

	/**
	 * - 9. Clear pointer:
	 */
	usbc_dev->driver = NULL;
	usbc_dev->gadget.dev.driver = NULL;

	/**
	 * - 10. Return:
	 *  - return 0;
	 */
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

/* ========================================================================= */
/**
 * @brief	- interrupt handler.
 * @par		usage:
 *				API(see functional specifications: 8.3.10.)
 * @param	irq;	irq number.
 * @param	*_dev;	usb_gadget_driver structure.
 * @param	*r;		stored cpu register before interrupt has occured.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static irqreturn_t s1r72xxx_usbc_irq(int irq, void *_dev)
{
	S1R72XXX_USBC_DEV	*usbc_dev;	/* USB hardware informations */
	int					retval;		/* return value */
	unsigned long		flags;		/* spin lock flag */
	unsigned char		device_int;	/* DeviceIntStat */
	unsigned char		powmng_int;	/* MainIntStat.FinishedPM */
	unsigned char		irq_ct;		/* loop counter */
	unsigned char		irq_done;	/* irq done flag */
	unsigned char		dummy;		/* dummy */
	unsigned char		main_irq;	/* irq flag */
	unsigned int		pm_change;	/* pm state change flag */

#ifdef CONFIG_EBOOK5_LED
	usbtg_led(EBOOK5_LED_TOGGLE);
#endif /* CONFIG_EBOOK5_LED */

	retval		= 0;
	device_int	= 0;
	powmng_int	= 0;
	main_irq	= 0;
	irq_done	= S1R72_IRQ_NONE;
	pm_change	= 0;

	/**
	 * - 1. Check parameter:
	 *  - _dev != NULL.
	 *  - in case of error, return IRQ_NONE.
	 */
	if (_dev == NULL){
		DEBUG_MSG("%s, irq error\n", __FUNCTION__);
		return IRQ_NONE;
	}
	usbc_dev = (S1R72XXX_USBC_DEV*)_dev;
	spin_lock_irqsave(&usbc_dev->lock, flags);
	/**
	 * - 2. Interrupt source check on MainIntStat:
	 *  - read DeviceIntStat and FinishedPM.
	 *  - continue this by loop, until all interrupt source cleared.
	 */
	dummy = rcD_RegRead8(usbc_dev, rcMainIntStat);
	main_irq = rcD_RegRead8(usbc_dev, rcMainIntStat) & (DevInt | FinishedPM);

	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_S, rcMainIntStat, main_irq);
	while ((main_irq) != S1R72_IRQ_NONE){
		/**
		 * - 2.1. Interrupt source check on DeviceIntStat:
		 *  - read MainIntStat.DeviceIntStat.
		 *  - when VBUS_Changed   is set, call usbc_irq_VBUS().
		 *  - when D_SIE_IntStat  is set, call usbc_irq_SIE().
		 *  - when D_BulkIntStat  is set, call usbc_irq_Bulk().
		 *  - when RcvEP0SETUP    is set, call usbc_irq_RcvEP0SETUP().
		 *  - when D_EP0IntStat   is set, call usbc_irq_EP0().
		 *  - when D_EPIntStat    is set, call usbc_irq_EP().
		 *  - when D_EPIntStat    is set, call usbc_irq_EP().
		 */
		device_int = rcD_RegRead8(usbc_dev, rcDeviceIntStat);
		powmng_int
			= (rcD_RegRead8(usbc_dev, rcMainIntStat) &  FinishedPM);
		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_DEVINT, rcDeviceIntStat,device_int);
		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_POWINT, rcMainIntStat, powmng_int);
		DEBUG_MSG("%s, dev_int=0x%04x\n", __FUNCTION__, device_int);
		DEBUG_MSG("%s, fpm_int=0x%04x\n", __FUNCTION__, powmng_int);

		/* DeviceIntStat */
		irq_ct = 0;
		while(usbc_dev_irq_tbl[irq_ct].irq_bit != S1R72_DEV_IRQ_NONE){
			if ( (device_int & usbc_dev_irq_tbl[irq_ct].irq_bit)
			== usbc_dev_irq_tbl[irq_ct].irq_bit ) {
				retval = usbc_dev_irq_tbl[irq_ct].func(usbc_dev);
				irq_done = S1R72_IRQ_DONE;
			}
			irq_ct ++;
		}
		/**
		 * - 2.2. Interrupt source check from FinishedPM:
		 */
		/* Power Management interrupt */
		if ( powmng_int != S1R72_IRQ_NONE ) {
			DEBUG_MSG("%s, FPM irq\n", __FUNCTION__);
			retval = usbc_irq_FPM(usbc_dev);
			irq_done = S1R72_IRQ_DONE;
		}
		/**
		 * - 2.3. Interrupt source is not for this driver:
		 *  - DeviceIntStat and FinishedPM.
		 *  - when they are 0x00, return IRQ_NONE.
		 */
		if (irq_done == S1R72_IRQ_NONE) {
			DEBUG_MSG("%s, irq NONE\n", __FUNCTION__);
			spin_unlock_irqrestore(&usbc_dev->lock, flags);
			return IRQ_NONE;
		}
		main_irq = (rcD_RegRead8(usbc_dev, rcMainIntStat)
			& (DevInt | FinishedPM) );

		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_DEVINT, rcDeviceIntStat,
			rcD_RegRead8(usbc_dev, rcDeviceIntStat));
		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_POWINT, rcMainIntStat,
			rcD_RegRead8(usbc_dev, rcMainIntStat));
		DEBUG_MSG("%s, main_int=0x%02x\n", __FUNCTION__, main_irq);
	}
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_E, rcMainIntStat, main_irq);

#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	/**
	 * - 3. Change device state to CPU_Cut:
	 */
	if ( (retval == S1R72_PM_CHANGE_TO_SLEEP)
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
		&& ( ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
		     == S1R72_PM_State_SLEEP)
		   ||((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
		     == S1R72_PM_State_SLEEP_01) )) {

#else
		&& ((rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
			== S1R72_PM_State_SLEEP) ) {
#endif
		rcD_RegWrite8(usbc_dev, rcPM_Control_0, GoCPU_Cut);
	}
#endif
	/**
	 * - 4. Return:
	 *  - return IRQ_HANDLED;
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);
	DEBUG_MSG("%s, irq exit\n", __FUNCTION__);
	return IRQ_HANDLED;

}
/* ========================================================================= */
/**
 * @brief	- release function for device structure.
 * @par		usage:
 *				internal use only.
 * @param	*dev;	device structure that specifies device drivers.
 * @retval	none.
 */
/* ========================================================================= */
static void s1r72xxx_usbc_release(struct device *dev)
{
	/**
	 * - 1. Check parameter:
	 *  - dev != NULL.
	 */
	DEBUG_MSG("%s, \n", __FUNCTION__);

	/**
	 * - 2. Return:
	 *  - this driver does not support this.
	 */
	return;
}


/***************************************************
 * Declarations of functions only called this file *
 ***************************************************/
/* ========================================================================= */
/**
 * @brief	- process VBUS_Changed interrupt and set timer.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_VBUS_CHG(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	retval;		/* return value */
	retval = 0;

	/**
	 * - 1. Clear interrupt source:
	 *	- to clear interrupt, write '1' to VBUS_Changed.
	 */
	DEBUG_MSG("%s, clear irq\n", __FUNCTION__);
	rcD_IntClr8(usbc_dev, rcDeviceIntStat, D_VBUS_Changed);

	/**
	 * - 2. Check VBUS state:
	 *	- if VBUS state is high, then start VBUS timer.
	 *	- in other case, start disconnect process.
	 */
	if ( (rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_VBUS)
		== S1R72_VBUS_H){
		/**
		 * - 2.1. Check VBUS timer state:
		 *	- if vbus timer is running, then delete the timer.
		 */
		if (usbc_dev->vbus_timer_state == S1R72_VBUS_TIMER_STOPPED ){
			usbc_dev->vbus_timer_state = S1R72_VBUS_TIMER_RUNNING;
		}else{
			del_timer_sync(&usbc_dev->vbus_timer);
		}
		/**
		 * - 2.2. Start VBUS timer:
		 */
		mod_timer(&usbc_dev->vbus_timer, S1R72_WAIT_VBUS_TIMEOUT_TM);
	}else{
		/**
		 * - 2.3. Check VBUS timer state:
		 *	- if vbus timer is running, then delete the timer.
		 */
		if (usbc_dev->vbus_timer_state == S1R72_VBUS_TIMER_RUNNING ){
			del_timer_sync(&usbc_dev->vbus_timer);
			usbc_dev->vbus_timer_state = S1R72_VBUS_TIMER_STOPPED;
		}

		/**
		 * - 2.4. disconnect process:
		 */
		usbc_irq_VBUS(usbc_dev);
	}

	/**
	 * - 3. Return:
	 *	- return 0;
	 */
	return retval;
}

/* ========================================================================= */
/**
 * @brief	- process VBUS_Changed interrupt.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation complete successfully.
 *					or status of _PM_CHANGE_TO_SLEEP.
 */
/* ========================================================================= */
static int usbc_irq_VBUS(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	ep_ct;		/* endpoint counter */
	unsigned char	retval;		/* return value */
	unsigned int	timeout_ct;
	timeout_ct = 0;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	retval = 0;

	/**
	 * - 1 Check current VBUS state:
	 *  - process 1.1. when VBUS state = High, another process 2.1.
	 */
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_VBUS,rcD_USB_Status ,
		rcD_RegRead8(usbc_dev, rcD_USB_Status));

	if ( (rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_VBUS)
		== S1R72_VBUS_H ) {
		/**
		 * - 1.1. Enable interrupt:
		 */
		DEBUG_MSG("%s, VBUS = H\n", __FUNCTION__);
		rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);
		rcD_RegSet8(usbc_dev, rcMainIntEnb, FinishedPM);

		/**
		 * - 1.2. Change state:
		 *  - change driver state, and changes hardware to ACTIVE.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_VBUS_H);

	} else {
		/**
		 * - 2.1. Disconnect from USB bus:
		 *  - stop timer, stop all request, set hardware to disconnect status.
		 *  - clear ep structure, stop test mode, join settings.
		 */
		del_timer_sync(&usbc_dev->wait_j_timer);
		rcD_RegClear8(usbc_dev, rcMainIntEnb, FinishedPM);

		if( (rcD_RegRead8(usbc_dev, rcPM_Control_0) & PM_State_Mask)
				!= S1R72_PM_State_ACT_DEVICE){
			rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);
			rcD_RegWrite8(usbc_dev, rcPM_Control_0, GoActive);

			while( (rcD_RegRead8(usbc_dev, rcMainIntStat) & FinishedPM)
					!= FinishedPM){
				timeout_ct++;
				if (timeout_ct > S1R72_PM_CHANGE_TIMEOUT){
					break;
				}
			}
			rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);
		}

		stop_activity(usbc_dev);
		usb_disconnected(usbc_dev);
		usbc_ep_init(usbc_dev);
		rcD_RegWrite8(usbc_dev, rcD_S1R72_USB_Test, S1R72_REG_ALL_CLR );
		usbc_dev->previous_usbcd_state = S1R72_GD_DONT_CHG;
		for (ep_ct = S1R72_GD_EP0 ; ep_ct < S1R72_MAX_ENDPOINT
			; ep_ct++){
			S1R72_AREAxJOIN_DIS(usbc_dev, ep_ct);
		}
		rcD_RegSet8(usbc_dev, rcMainIntEnb, FinishedPM);

		/**
		 * - 2.2. Change state:
		 *  - change driver state, and changes hardware to SLEEP.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_VBUS_L);
		retval = S1R72_PM_CHANGE_TO_SLEEP;
		DEBUG_MSG("%s, VBUS = L\n", __FUNCTION__);
	}

	/**
	 * - 3. Clear interrupt source:
	 *  - to clear interrupt, write '1' to VBUS_Changed.
	 */
	DEBUG_MSG("%s, clear irq\n", __FUNCTION__);
	rcD_IntClr8(usbc_dev, rcDeviceIntStat, D_VBUS_Changed);

	/**
	 * - 4. Return:
	 *  - return 0 or _PM_CHANGE_TO_SLEEP;
	 */
	return retval;
}

/* ========================================================================= */
/**
 * @brief	- process SIE interrupt.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_SIE(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	sie_int;		/* SIE_IntStat */
	unsigned char	int_src_ct;		/* interrupt source counter*/
	unsigned char	int_src;		/* interrupt source */
	unsigned char	timeout_ct;		/* time out counter */
	unsigned char	new_line_state;	/* line_state */
	unsigned char	old_line_state;	/* line_state */
	timeout_ct = 0;

	int_src = S1R72_IRQ_DONE;
	DEBUG_MSG("%s, enter\n", __FUNCTION__);

	/**
	 * - 1. Check current SIE state:
	 *  - process 1.1. to 6.1 according to D_SIE_IntStat state.
	 */
	sie_int = rcD_RegRead8(usbc_dev, rcD_S1R72_SIE_IntStat);
	for ( int_src_ct = 0 ; int_src_ct < S1R72_SIE_INT_MAX ; int_src_ct++ ){
		if( (sie_int & sie_irq_tbl[int_src_ct]) == sie_irq_tbl[int_src_ct] ){
			int_src = sie_irq_tbl[int_src_ct];
			break;
		}
	}
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_SIE, rcD_S1R72_SIE_IntStat , sie_int);

	switch(int_src){
	case S1R72_NonJ:
		/**
		 * - 1.1. Check LineState:
		 *	- in this case, LineState must indicate SE0 or K.
		 *	- if LineState is J or SE1, driver should wait state changing to
		 *	- SE0/K.
		 */
		old_line_state
			= rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_LineState;
		while( timeout_ct < S1R72_WAIT_NONJ_TIMEOUT_TM_MAX ){
			new_line_state
				= rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_LineState;
			if ( ((new_line_state == S1R72_LineState_J)
					|| (new_line_state == S1R72_LineState_SE1))
				|| (old_line_state != new_line_state) ){
				rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);
				return 0;
			}
			timeout_ct++;
		}

		/**
		 * - 1.2. Clear NonJ interrupt:
		 */
		rcD_RegClear8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_EnNonJ);
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);
		DEBUG_MSG("%s, NonJ irq\n", __FUNCTION__);

		/**
		 * - 1.3. Clear suspend state:
		 *  - when remote wakeup is proceeding, clear hardware suspend state.
		 */
		if (usbc_dev->remote_wakeup_prc == S1R72_RT_WAKEUP_PROC){
			usbc_dev->remote_wakeup_prc = S1R72_RT_WAKEUP_NONE;
			/* return from USB suspend */
			rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl, S1R72_InSUSPEND);
		}

		/**
		 * - 1.4. Report resume to gadget driver:
		 */
		if ((usbc_dev->driver != NULL)
			&&(usbc_dev->driver->resume != NULL)){
			usbc_dev->driver->resume(&usbc_dev->gadget);
		}

		/**
		 * - 1.5. Change state:
		 *  - change driver state, and changes hardware to ACTIVE.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_NONJ);
		break;

	case S1R72_DetectRESET:
		/**
		 * - 2.1. Stop all activity:
		 *  - flush all request and report disconnect to gadget driver.
		 */
		DEBUG_MSG("%s, DetectReset irq\n", __FUNCTION__);

		/**
		 * - 2.2. Clear interrupt:
		 */
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);

		/**
		 * - 2.3. Change state:
		 *  - change driver state.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_RESET);
		break;

	case S1R72_DetectSUSPEND:
		/**
		 * - 3.1. Report suspend to gadget driver:
		 */
		DEBUG_MSG("%s, DetectSUSPEND irq\n", __FUNCTION__);
		if ((usbc_dev->driver != NULL)
			&&(usbc_dev->driver->suspend != NULL)){
			usbc_dev->driver->suspend(&usbc_dev->gadget);
		}

		/**
		 * - 3.2. Set interrupt:
		 *  - set NonJ interrupt to response resume request, clear interrupt.
		 */
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, S1R72_NonJ);
		rcD_RegSet8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_EnNonJ);
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);

		/**
		 * - 3.3. Change state:
		 *  - change driver state, and changes hardware to SLEEP.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_SUSPEND);
		if ( usbc_dev->usbcd_state == S1R72_GD_SYSSUSPD){
			usbc_dev->previous_usbcd_state = S1R72_GD_USBSUSPD;
		}
		break;

	case S1R72_ChirpCmp:
		/**
		 * - 4.1. Clear interrupt:
		 */
		DEBUG_MSG("%s, ChirpCmp irq\n", __FUNCTION__);
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);

		/**
		 * - 4.2. Delete bus status check timer:
		 */
		del_timer(&usbc_dev->wait_j_timer);

		/**
		 * - 4.3. Change state:
		 *  - change driver state.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_CHIRP);

		/**
		 * - 4.4. Check bus speed:
		 *  - check bus speed and set gadget.speed value to report this
		 *  - information to gadget driver.
		 */
		if ((rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_FSxHS)
			== S1R72_Status_FS ) {
			/* FS mode */
			usbc_dev->gadget.speed = USB_SPEED_FULL;
			DEBUG_MSG("%s, FS mode\n", __FUNCTION__);
		} else {
			/* HS mode */
			usbc_dev->gadget.speed = USB_SPEED_HIGH;
			DEBUG_MSG("%s, HS mode\n", __FUNCTION__);
		}
		break;

	case S1R72_RestoreCmp:
		/**
		 * - 5.1. Clear interrupt:
		 */
		DEBUG_MSG("%s, RestoreCmp irq\n", __FUNCTION__);
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);
		/**
		 * - 5.2. Change state:
		 *  - change driver state.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_RESTORE);
		break;

	case S1R72_SetAddressCmp:
		/**
		 * - 6.1. Clear interrupt:
		 */
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, int_src);
		rcD_IntClr8(usbc_dev,
			usbc_dev->usbc_ep[S1R72_GD_EP0].reg.ep0.EP0IntStat,
			S1R72_IN_TranACK);
		DEBUG_MSG("%s, SetAddressCmp irq\n", __FUNCTION__);

		/**
		 * - 6.2. Change state:
		 *  - change driver state and ep0 state.
		 */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_ADDRESS);
		change_ep0_state(&usbc_dev->usbc_ep[S1R72_GD_EP0], S1R72_GD_EP_IDLE);
		break;

	default:
		/* This should not occur */
		DEBUG_MSG("%s, interrupt source is none %x\n", __FUNCTION__, sie_int);
		rcD_IntClr8(usbc_dev,  rcD_S1R72_SIE_IntStat, int_src);
		rcD_IntClr8(usbc_dev,  rcD_S1R72_SIE_IntStat, S1R72_RcvSOF);
		break;
	}

	/**
	 * - 7. Return:
	 *  - return 0;
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- process Bulk transfer interrupt.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_Bulk(S1R72XXX_USBC_DEV *usbc_dev)
{
	/**
	 * - 1. Do nothing:
	 *  - this routine is made for bulk-only mode support.
	 *    but I don't use this, so no process here.
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- process recieve ep0 setup interrupt.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_RcvEP0SETUP(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	recip_req_type;		/* recieved request type */
	unsigned char	recip_req;			/* recieved request */
	unsigned short	recip_ind_type;		/* recieved index */
	unsigned char	req_ct;				/* endpoint number */
	unsigned char	reg_value;			/* register value */
	unsigned char	ep_num;				/* endpoint number */
	unsigned char	ep_ct;				/* endpoint number */
	unsigned char	term;				/* termination */
	unsigned short	wr_data;			/* data to be send */
	unsigned short	*fifo_16;			/* fifo pointer for 16 bit access */
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
	unsigned short	dummy_data;			/* fifo data includes dummy data */
#endif
	unsigned char	fifo_ct;			/* counter to fifo */
	S1R72XXX_USBC_EP	*ep0;			/* USB endpoint0 informations */
	S1R72XXX_USBC_EP	*epx;			/* USB endpointx informations */
	union {								/* usb control request */
		struct usb_ctrlrequest r;
		u8 raw[8];
		u32 word[2];
	} u;								/* control request data */
	S1R72XXX_USBC_REQ	*queued_req;	/* request */
	int				retval;				/* return value from setup() */

	retval = 0;
	reg_value = 0;
	ep_num = 0;
	wr_data = 0;
	term = S1R72_OpMode_DisableBitStuffing;
	ep0 = &usbc_dev->usbc_ep[S1R72_GD_EP0];
	epx = NULL;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Clear interrupt:
	 */
	if ( (rcD_RegRead8(usbc_dev, rcDeviceIntStat) & D_RcvEP0SETUP)
		!= D_RcvEP0SETUP){
		return retval;
	}
	rcD_IntClr8(usbc_dev, rcDeviceIntStat, D_RcvEP0SETUP);

	/**
	 * - 2. Check parameter:
	 *  - dev != NULL.
	 */
	if ( (usbc_dev->driver == NULL)
		|| (usbc_dev->driver->setup == NULL)){
		return -ENODEV;
	}

	/**
	 * - 3. Read setup data from ep0 FIFO:
	 *  - read D_EP0SETUP_[0..7].
	 */
	for (req_ct = S1R72_REQ_TYPE ; req_ct < S1R72_DEV_REQ_MAX ; req_ct ++){
		u.raw[req_ct] = rcD_RegRead8(usbc_dev, rcD_S1R72_EP0SETUP_0 + req_ct);
	}
	DEBUG_MSG(
		"%s, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
		__FUNCTION__, u.raw[0],u.raw[1], u.raw[2], u.raw[3], u.raw[4],
		u.raw[5], u.raw[6], u.raw[7] );
	if ((rcD_RegRead8(usbc_dev, ep0->reg.ep0.EP0ControlIN) & S1R72_EnShortPkt)
			== S1R72_EnShortPkt) {
		clear_ep_enshortpkt(usbc_dev, ep0);
	}
	while ( list_empty(&ep0->queue) == S1R72_LIST_ISNOT_EMPTY ) {
		queued_req = list_entry(ep0->queue.next, S1R72XXX_USBC_REQ, queue);
		request_done(ep0, queued_req, -ECONNRESET);
	}
	S1R72_EP0FIFO_CLR(usbc_dev);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	/**
	 * - 3.1. clear FIFO addup:
	 */
	ep0->fifo_addup = FIFO_ADDUP_CLR;
#endif

	/**
	 * - 4. Change ep0 state:
	 *  - changes ep0 state to S1R72_GD_EP_DIN or S1R72_GD_EP_DOUT.
	 */
	if ( (u.r.bRequestType & USB_DIR_IN) || (le16_to_cpu(u.r.wLength) == 0) ) {
		change_ep0_state(ep0, S1R72_GD_EP_DIN);
	} else {
		change_ep0_state(ep0, S1R72_GD_EP_DOUT);
	}

	/**
	 * - 5. Check bmRequestType:
	 *  - check bmRequestType. if bmRequestType is USB_REQ_CLEAR_FEATURE,
	 *    USB_REQ_SET_FEATURE or USB_REQ_GET_STATUS, then process here.
	 *    other bmRequestType is processed gadget driver.
	 */
	recip_req_type = u.r.bRequestType;
	recip_req = u.r.bRequest;
	recip_ind_type = u.r.wIndex;
	ep_num = u.r.wIndex & USB_ENDPOINT_NUMBER_MASK;

	DEBUG_MSG("%s, bRequestType=0x%02x\n", __FUNCTION__, recip_req_type);
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_RCVEP0, u.r.bRequestType, u.r.bRequest);

	if ( (ep_num != S1R72_GD_EP0) && (ep_num < S1R72_MAX_ENDPOINT) ){
		epx = &usbc_dev->usbc_ep[ep_num];
	}

	if( (recip_req_type & USB_TYPE_MASK) == USB_TYPE_STANDARD){
		switch(recip_req) {
		/**
		 * - 5.1. GET_STATUS:
		 */
		case USB_REQ_GET_STATUS:
			DEBUG_MSG("%s, USB_REQ_GET_STATUS\n", __FUNCTION__);
			/**
			 * - 5.1.1. Set direction IN:
			 *  - Set EP0Join to write, EP0Control IN, clear FIFO.
			 */
			change_ep0_state(ep0, S1R72_GD_EP_DIN);
			S1R72_EPJOINCPU_CLR(usbc_dev);
			rcD_RegSet8(usbc_dev, ep0->reg.ep0.EP0Join, S1R72_JoinCPU_Wr);
			rcD_RegWrite8(usbc_dev, ep0->reg.ep0.EP0Control,
				S1R72_EP0INxOUT_IN);
			S1R72_EP0FIFO_CLR(usbc_dev);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	/**
	 * - 5.1.1.2. clear FIFO addup:
	 */
	ep0->fifo_addup = FIFO_ADDUP_CLR;
#endif

			/**
			 * - 5.1.2. Set data to FIFO:
			 *  - Set current status to FIFO and set EnShortPkt bit
			 *    at EP0ControlIN.
			 */
			switch(recip_req_type & USB_RECIP_MASK) {
				case USB_RECIP_DEVICE:
					DEBUG_MSG("%s, DEVICE\n", __FUNCTION__);
					wr_data = S1R72_SELF_POW | usbc_dev->remote_wakeup_enb;
					break;
				case USB_RECIP_INTERFACE:
					DEBUG_MSG("%s, INTERFACE\n", __FUNCTION__);
					break;
				case USB_RECIP_ENDPOINT:
					DEBUG_MSG("%s, ENDPOINT\n", __FUNCTION__);
					if ( (ep_num != S1R72_GD_EP0)
						&& (ep_num < S1R72_MAX_ENDPOINT) ){
						wr_data |= rcD_RegRead8(usbc_dev,
							epx->reg.epx.EPxControl)
							& S1R72_ForceSTALL;
					} else if (ep_num == S1R72_GD_EP0) {
						wr_data = 0;
					} else {
						retval = -EINVAL;
					}
					break;
				default:
					retval = -EINVAL;
					break;
			}
			DEBUG_MSG("%s, value=0x%04x\n", __FUNCTION__, wr_data);
			rsD_RegWrite16(usbc_dev, rsFIFO_Wr, wr_data);
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
			/**
			 * - 5.1.2.1. wait to flush cache:
			 */
			if( check_fifo_cache(usbc_dev)==S1R72_CACHE_REMAIN_NG){
				s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rcCacheRemain,
					rcD_RegRead8(usbc_dev, rcCacheRemain));
			}
#endif
			send_ep0_short_packet(usbc_dev, ep0);
			change_ep0_state(ep0, S1R72_GD_EP_DIN_CHG);
			S1R72_EPJOINCPU_CLR(usbc_dev);
			break;

		/**
		 * - 5.2. CLEAR_FEATURE:
		 */
		case USB_REQ_CLEAR_FEATURE:
			DEBUG_MSG("%s, USB_REQ_CLEAR_FEATURE\n", __FUNCTION__);
			switch(u.r.wValue) {
			case USB_ENDPOINT_HALT:
				/**
				 * - 5.2.1. Set endpoint stall:
				 *  - set ForceSTALL bit at EPxControl.
				 */
				DEBUG_MSG("%s, USB_ENDPOINT_HALT\n", __FUNCTION__);
				if ( (ep_num != S1R72_GD_EP0)
					&& (ep_num < S1R72_MAX_ENDPOINT) ){
					if ( (rcD_RegRead8(usbc_dev, epx->reg.epx.EPxControl)
						& S1R72_ForceSTALL) == S1R72_ForceSTALL ){
						rcD_RegClear8(usbc_dev, epx->reg.epx.EPxControl,
							S1R72_ForceSTALL);
					}
					rcD_RegSet8(usbc_dev, epx->reg.epx.EPxControl,
						S1R72_ToggleClr);
					epx->is_stopped = S1R72_EP_ACTIVE;
					change_ep0_state(ep0, S1R72_GD_EP_SIN);
				} else {
					retval = -EINVAL;
				}
				break;

			case USB_DEVICE_REMOTE_WAKEUP:
				/**
				 * - 5.2.2. Clear remote wakeup enable flag:
				 */
				change_ep0_state(ep0, S1R72_GD_EP_SIN);
				usbc_dev->remote_wakeup_enb = S1R72_RT_WAKEUP_DIS;
				break;

			default:
				/* This should not occur */
				DEBUG_MSG("%s, USB_REQ_CLEAR_FEATURE error\n", __FUNCTION__);
				retval = -EINVAL;
				break;
			}
			break;

		/**
		 * - 5.3. SET_FEATURE:
		 */
		case USB_REQ_SET_FEATURE:
			DEBUG_MSG("%s, USB_REQ_SET_FEATURE\n", __FUNCTION__);
			switch(u.r.wValue) {
			case USB_ENDPOINT_HALT:
				/**
				 * - 5.3.1. Clear endpoint stall:
				 *  - clear ForceSTALL bit at EPxControl.
				 */
				DEBUG_MSG("%s, USB_ENDPOINT_HALT\n", __FUNCTION__);
				if ( (ep_num != S1R72_GD_EP0)
					&& (ep_num < S1R72_MAX_ENDPOINT) ){
					if ( (rcD_RegRead8(usbc_dev, epx->reg.epx.EPxControl)
						& S1R72_ForceSTALL) != S1R72_ForceSTALL ){
						rcD_RegSet8(usbc_dev, epx->reg.epx.EPxControl,
							S1R72_ForceSTALL);
					}
					epx->is_stopped = S1R72_EP_STALL;
					change_ep0_state(ep0, S1R72_GD_EP_SIN);
				} else {
					retval = -EINVAL;
				}
				break;

			case USB_DEVICE_TEST_MODE:
				/**
				 * - 5.3.2. Set test mode:
				 */
				DEBUG_MSG("%s, USB_DEVICE_TEST_MODE\n", __FUNCTION__);
				change_ep0_state(ep0, S1R72_GD_EP_SIN);
				rcD_RegSet8(usbc_dev, ep0->reg.ep0.EP0Control,
					S1R72_EP0INxOUT_IN);
				S1R72_EP0FIFO_CLR(usbc_dev);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	/**
	 * - 5.3.2.1. clear FIFO addup:
	 */
	ep0->fifo_addup = FIFO_ADDUP_CLR;
#endif

				send_ep0_short_packet(usbc_dev, ep0);

				/**
				 * - 5.3.3. Disable auto negotiation :
				 */
				if (u.raw[S1R72_INDEX_H] >= TEST_MODE_J
						|| u.raw[S1R72_INDEX_H] <= TEST_MODE_PACKET){
					rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl,
						S1R72_EnAutoNego );
					rcD_RegWrite8(usbc_dev, rcD_S1R72_NegoControl,
						(S1R72_ActiveUSB | S1R72_DisBusDetect) );
				}

				switch(u.raw[S1R72_INDEX_H]) {
				case TEST_MODE_J:
					/**
					 * - 5.3.3.1. TEST_MODE_J:
					 *  - set FS mode to XcvrControl and set Test J.
					 */
					DEBUG_MSG("%s, TEST_MODE_J\n", __FUNCTION__);
					term =S1R72_OpMode_DisableBitStuffing;
					if( (rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_FSxHS)
						== S1R72_Status_FS){
						term |= S1R72_TermSelect | S1R72_XcvrSelect;
					}
					rcD_RegWrite8(usbc_dev, rcD_S1R72_XcvrControl, term);
					rcD_RegSet8(usbc_dev, rcD_S1R72_USB_Test, S1R72_Test_J);
					break;

				case TEST_MODE_K:
					/**
					 * - 5.3.3.2. TEST_MODE_K:
					 *  - set FS mode to XcvrControl and set Test K.
					 */
					DEBUG_MSG("%s, TEST_MODE_K\n", __FUNCTION__);
					term =S1R72_OpMode_DisableBitStuffing;
					if( (rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_FSxHS)
						== S1R72_Status_FS){
						term |= S1R72_TermSelect | S1R72_XcvrSelect;
					}
					rcD_RegWrite8(usbc_dev, rcD_S1R72_XcvrControl, term);
					rcD_RegSet8(usbc_dev, rcD_S1R72_USB_Test, S1R72_Test_K);
					break;

				case TEST_MODE_SE0_NAK:
					/**
					 * - 5.3.3.3. TEST_MODE_SE0_NAK:
					 *  - set Test SE0 NAK.
					 */
					DEBUG_MSG("%s, TEST_MODE_SE0_NAK\n", __FUNCTION__);
					rcD_RegClear8(usbc_dev, rcD_S1R72_XcvrControl,
						S1R72_TermSelect);
					rcD_RegSet8(usbc_dev, rcD_S1R72_USB_Test,
						S1R72_Test_SE0_NAK);
					break;

				case TEST_MODE_PACKET:
					/**
					 * - 5.3.3.4.1. TEST_MODE_PACKET:
					 */
					DEBUG_MSG("%s, TEST_MODE_PACKET\n", __FUNCTION__);

					/**
					 * - 5.3.3.4.2. Clear all endpoint settings:
					 */
					for (ep_ct = S1R72_GD_EPA ; ep_ct < S1R72_MAX_ENDPOINT
						; ep_ct++){
						rcD_RegWrite8(usbc_dev,
							usbc_dev->usbc_ep[ep_ct].reg.epx.EPxControl,
							S1R72_REG_ALL_CLR);
						rcD_RegWrite8(usbc_dev,
							usbc_dev->usbc_ep[ep_ct].reg.epx.EPxConfig_0,
							S1R72_REG_ALL_CLR);
						rcD_RegWrite8(usbc_dev,
							usbc_dev->usbc_ep[ep_ct].reg.epx.EPxMaxSize,
							S1R72_REG_ALL_CLR);
					}
					rcD_RegSet8(usbc_dev,
						usbc_dev->usbc_ep[S1R72_GD_EPA].reg.epx.EPxConfig_0,
						(S1R72_DIR_IN | TEST_MODE_EP_NUM));

#if defined(CONFIG_USB_S1R72V05_GADGET)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_MODULE)
					rcD_RegSet8(usbc_dev,
						usbc_dev->usbc_ep[S1R72_GD_EPA].reg.epx.EPxConfig_0,
						S1R72_EnEndpoint);
#endif

					rsD_RegWrite16(usbc_dev,
						usbc_dev->usbc_ep[S1R72_GD_EPA].reg.epx.EPxMaxSize,
						TEST_MODE_EP_PACKET_SIZE);

#if defined(CONFIG_USB_S1R72V17_GADGET)\
	|| defined(CONFIG_USB_S1R72V17_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
					/* FIFO Clear */
					rcD_RegWrite8(usbc_dev, rcAREAnFIFO_clr,
						S1R72_ClrAREA_ALL);
#endif
					/* clear join settings */
					for (ep_ct = S1R72_GD_EP0 ; ep_ct < S1R72_MAX_ENDPOINT
						; ep_ct++){
						S1R72_AREAxJOIN_DIS(usbc_dev, ep_ct);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
						usbc_dev->usbc_ep[ep_ct].fifo_addup = FIFO_ADDUP_CLR;
#endif
					}

					/* set AREA Join_1 */
					S1R72_AREAxJOIN_ENB(usbc_dev, S1R72_GD_EPA);

					/* set join register */
					S1R72_EPJOINCPU_CLR(usbc_dev);
					rcD_RegSet8(usbc_dev,
						usbc_dev->usbc_ep[S1R72_GD_EPA].reg.epx.EPxJoin,
						S1R72_JoinCPU_Wr);

					/**
					 * - 5.3.3.4.3. Write test packet to FIFO:
					 */
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
					fifo_ct = 1;
					fifo_16 = (unsigned short *)&test_mode_packet[1];
					dummy_data = S1R72_ADD_DUMMY(test_mode_packet[0]);
					rsD_RegWrite16(usbc_dev, rsFIFO_Wr,
						dummy_data);
#else
					fifo_ct = 0;
					fifo_16 = (unsigned short *)test_mode_packet;
#endif
					while ( fifo_ct <  TEST_MODE_PACKET_SIZE - 1){
						rsD_RegWrite16(usbc_dev, rsFIFO_Wr, *fifo_16);
						fifo_16++;
						fifo_ct += 2;
					}
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
					S1R72_EPJOINCPU_CLR(usbc_dev);
					rcD_RegSet8(usbc_dev,
						usbc_dev->usbc_ep[S1R72_GD_EPA].reg.epx.EPxJoin,
						S1R72_JoinCPU_Rd);
					check_fifo_remain(usbc_dev);
					dummy_data = rcD_RegRead8(usbc_dev, rcFIFO_ByteRd);
#else	/* _16BIT_FIX */
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
					/**
					 * - 5.3.3.4.3.2. manage against FIFO fraction:
					 */
					rcD_RegWrite8(usbc_dev, rcFIFO_ByteWr,
						test_mode_packet[TEST_MODE_PACKET_SIZE -1 ]);

#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
					/**
					 * - 5.3.3.4.3.3. wait to flush cache:
					 */
					if( check_fifo_cache(usbc_dev) == S1R72_CACHE_REMAIN_NG){
						s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rcCacheRemain,
							rcD_RegRead8(usbc_dev, rcCacheRemain));
					}
#endif

					/**
					 * - 5.3.3.4.3.4. renew FIFO addup:
					 */
					usbc_dev->usbc_ep[S1R72_GD_EPA].fifo_addup
						^= (TEST_MODE_PACKET_SIZE & FIFO_ADDUP_MASK);
#else
					rcD_RegWrite8(usbc_dev, rcFIFO_Wr_0,
						test_mode_packet[TEST_MODE_PACKET_SIZE - 1]);
#endif
#endif	/* _16BIT_FIX */
					S1R72_EPJOINCPU_CLR(usbc_dev);

					/**
					 * - 5.3.3.4.5. Clear IN_TranErr:
					 */
					rcD_RegWrite8(usbc_dev,
						usbc_dev->usbc_ep[S1R72_GD_EPA].reg.epx.EPxIntStat,
						S1R72_IN_TranErr);

					/**
					 * - 5.3.3.4.6. Set Test_Mode_Packet:
					 */
					rcD_RegWrite8(usbc_dev, rcD_S1R72_USB_Test,
						S1R72_Test_Packet);
					break;
				default:
					/* This should not occur */
					DEBUG_MSG("%s, TEST_MODE error\n", __FUNCTION__);
					retval = -EINVAL;
					break;
				}
				if ( retval < S1R72_RET_OK){
					break;
				}
				/**
				 * - 5.3.3.5. Disable interrupt :
				 */
				rcD_RegWrite8(usbc_dev, rcD_S1R72_SIE_IntEnb,
					S1R72_REG_ALL_CLR);

				/**
				 * - 5.3.3.6. Enable test mode :
				 */
				rcD_RegSet8(usbc_dev, rcD_S1R72_USB_Test, S1R72_EnHS_Test );
				break;

			case USB_DEVICE_REMOTE_WAKEUP:
				/**
				 * - 5.3.4. Set remote wakeup enable flag:
				 */
				change_ep0_state(ep0, S1R72_GD_EP_SIN);
				usbc_dev->remote_wakeup_enb = S1R72_RT_WAKEUP_ENB;
				break;

			default:
				/* This should not occur */
				DEBUG_MSG("%s, USB_REQ_SET_FEATURE error\n", __FUNCTION__);
				retval = -EINVAL;
				break;
			}
			break;

		default :
			/**
			 * - 5.4. bmRequestType != above request:
			 *  - set setup data to gadget driver calling by setup() at
			 *    gadget driver. in case of error, set hardware ForceSTALL.
			 */
			DEBUG_MSG("%s, call gadget driver setup=0x%02x\n", __FUNCTION__,
				recip_req_type);
			retval = usbc_dev->driver->setup(&usbc_dev->gadget, &u.r);
			break;
		}
	} else {
		DEBUG_MSG("%s, this is not standard request\n", __FUNCTION__);
		retval = usbc_dev->driver->setup(&usbc_dev->gadget, &u.r);
	}

	/**
	 * - 6. Set STALL :
	 *  - if returned value is error code, set STALL endpoint 0.
	 */
	if ( retval < S1R72_RET_OK ) {
		/* force stall ep0 */
		rcD_RegClear8(usbc_dev, ep0->reg.ep0.SETUP_Control,
			S1R72_ProtectEP0);
		rcD_RegWrite8(usbc_dev, ep0->reg.ep0.EP0ControlIN,
			 S1R72_ForceSTALL);
		rcD_RegWrite8(usbc_dev, ep0->reg.ep0.EP0ControlOUT,
			 S1R72_ForceSTALL);
		rcD_RegSet8(usbc_dev, ep0->reg.ep0.SETUP_Control,
			S1R72_ProtectEP0);
	}

	/**
	 * - 7. Return:
	 *  - return 0;
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- process EP0 interrupt.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_EP0(S1R72XXX_USBC_DEV *usbc_dev)
{
	S1R72XXX_USBC_REQ	*queued_req;		/* request */
	S1R72XXX_USBC_EP	*ep;				/* ep0 structure */
	unsigned char		ep0_int;			/* EP0IntStat */
	unsigned char		inout_flag;			/* IN/OUT */
	int					handle_ret;			/* handle_ep return value */
	unsigned char		chk_ret;			/* return value */

	ep = &usbc_dev->usbc_ep[S1R72_GD_EP0];
	ep->intenb = S1R72_IRQ_NOT_OCCURED;
	inout_flag = S1R72_IN_TranACK;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Check current EP0Int state:
	 *  - set ForceNAK to stop transaction before get interrupt source.
	 *  - read D_EP0IntStat to get interrupt source.
	 */
	set_ep0_out_force_nak(usbc_dev, ep);
	ep0_int = rcD_RegRead8(usbc_dev, rcD_S1R72_EP0IntStat);

	DEBUG_MSG("%s, EP0IntStat=0x%02x\n", __FUNCTION__, ep0_int);
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP0, rcD_S1R72_EP0IntStat , ep0_int);

	/**
	 * - 1.1. IN TranACK:
	 */
	if( (ep0_int & S1R72_IN_TranACK) == S1R72_IN_TranACK ){
		DEBUG_MSG("%s, IN\n", __FUNCTION__);
		/**
		 * - 1.1.1. State is waiting data stage completion:
		 */
		clear_ep0_out_force_nak(usbc_dev, ep);
		if (ep->ep_state == S1R72_GD_EP_DIN_CHG){
			/**
			 * - 1.1.1.1. Check data remain in FIFO:
			 *  - set Join register and read RdRemain.
			 */
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Join, S1R72_JoinCPU_Rd);

			chk_ret = check_fifo_remain(usbc_dev);
			if (chk_ret == S1R72_RD_REMAIN_NG){
				S1R72_EPJOINCPU_CLR(usbc_dev);
				rcD_IntClr8(usbc_dev, rcD_S1R72_EP0IntStat, S1R72_IN_TranACK);
				return 0;
			}

			/**
			 * - 1.1.1.2. Complete data stage:
			 *  - complete data stage, and change state to status stage.
			 */
			if ( (rsD_RegRead16(usbc_dev,rsFIFO_RdRemain) & S1R72_FIFO_EMPTY)
				== S1R72_FIFO_EMPTY){
				queued_req
					= list_entry(ep->queue.next, S1R72XXX_USBC_REQ , queue);
				/* transfer completed */
				if ( (queued_req != NULL)
					&& (queued_req->req.actual == queued_req->req.length) ){
					if (queued_req->send_zero == S1R72_SEND_ZERO){
						send_ep0_short_packet(usbc_dev, ep);
						queued_req->send_zero = S1R72_DONT_SEND_ZERO;
					}
					request_done(ep, queued_req, 0);
				}
			}
			S1R72_EPJOINCPU_CLR(usbc_dev);
			change_ep0_state(ep, S1R72_GD_EP_SOUT);

		/**
		 * - 1.1.2. State is waiting status stage completion:
		 */
		} else if (ep->ep_state == S1R72_GD_EP_SIN){
			/**
			 * - 1.1.2.1. Complete status stage:
			 */
			change_ep0_state(ep, S1R72_GD_EP_IDLE);

		/**
		 * - 1.1.3. State is data stage:
		 */
		} else {
			/**
			 * - 1.1.3.1. Set interrupt occured flag:
			 *  - driver has IN data, set intenb flag to call handle_ep_in.
			 */
			ep->intenb = S1R72_IRQ_OCCURED;
		}
		ep0_int = S1R72_IN_TranACK;
		inout_flag = S1R72_IN_TranACK;

		/**
		 * - 1.1.4. Clear interrupt source:
		 *  - to clear interrupt, write '1' to register.
		 */
		rcD_IntClr8(usbc_dev, rcD_S1R72_EP0IntStat, ep0_int);

	/**
	 * - 1.2. OUT TranACK:
	 */
	} else if ((ep0_int & S1R72_OUT_TranACK) == S1R72_OUT_TranACK) {
		DEBUG_MSG("%s, OUT\n", __FUNCTION__);
		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP0, rcD_S1R72_EP0IntStat, ep0_int);
		inout_flag = S1R72_OUT_TranACK;

		/**
		 * - 1.2.1. State is waiting status stage completion:
		 */
		if (ep->ep_state == S1R72_GD_EP_SOUT){
			/**
			 * - 1.2.1.1. Complete status stage:
			 */
			change_ep0_state(ep, S1R72_GD_EP_IDLE);

			/**
			 * - 1.2.1.2. Clear interrupt source:
			 *  - to clear interrupt, write '1' to register.
			 */
			rcD_IntClr8(usbc_dev, rcD_S1R72_EP0IntStat,
					(S1R72_OUT_TranACK | S1R72_OUT_ShortACK));

		/**
		 * - 1.2.2. State is data stage:
		 */
		} else {
			/**
			 * - 1.2.2.1. Get recieved data size and set flags:
			 *  - get data size of recieved data and set intenb flag to
			 *  - call handle_ep_out.
			 *  - if data is short packet, set short packet flag.
			 */
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Join, S1R72_JoinCPU_Rd);

			chk_ret = check_fifo_remain(usbc_dev);
			if (chk_ret == S1R72_RD_REMAIN_NG){
				S1R72_EPJOINCPU_CLR(usbc_dev);
				if ( (ep0_int & S1R72_OUT_ShortACK) != S1R72_OUT_ShortACK ) {
					clear_ep0_out_force_nak(usbc_dev, ep);
				}
				return 0;
			}

			ep->fifo_data = (unsigned int)rsD_RegRead16(usbc_dev,
				rsFIFO_RdRemain) & FIFO_RdRemain;

			/**
			 * - 1.2.2.2. Clear interrupt source:
			 *  - to clear interrupt, write '1' to register.
			 */
			rcD_IntClr8(usbc_dev, rcD_S1R72_EP0IntStat,
					(S1R72_OUT_TranACK | S1R72_OUT_ShortACK));

			if ( (ep0_int & S1R72_OUT_ShortACK) == S1R72_OUT_ShortACK) {
				ep->last_is_short = S1R72_IRQ_SHORT;
			} else {
				ep->last_is_short = S1R72_IRQ_IS_NOT_SHORT;
				clear_ep0_out_force_nak(usbc_dev, ep);
			}
			S1R72_EPJOINCPU_CLR(usbc_dev);
			ep->intenb = S1R72_IRQ_OCCURED;
		}
	}

	/**
	 * - 2. Read/Write data from/to FIFO:
	 *  - Read data from FIFO by handle_ep_in().
	 *  - Write data to FIFO by handle_ep_out().
	 */
	if ( ( ep0_int != S1R72_IRQ_NONE) && (ep->ep_state != S1R72_GD_EP_IDLE)
		&& (ep->ep_state != S1R72_GD_EP_SOUT) ){
		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP0, rcD_S1R72_EP0IntStat, ep0_int);

		if (inout_flag == S1R72_IN_TranACK){
			handle_ret = handle_ep_in(ep, NULL);
		} else {
			handle_ret = handle_ep_out(ep, NULL);
		}
	}

	/**
	 * - 3. Return:
	 *  - return 0;
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- process EPx interrupt.(except EP0)
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_EP(S1R72XXX_USBC_DEV *usbc_dev)
{
	S1R72XXX_USBC_REQ	*queued_req;	/* request */
	S1R72XXX_USBC_EP	*epx;			/* endpoint */
	unsigned char		epr_int;		/* EPrIntStat */
	unsigned char		epx_int;		/* EPxIntStat */
	unsigned char		up_low_mask;	/* register mask */
	unsigned char		reg_value;		/* register mask */
	unsigned char		shift_bit;		/* shirt bit number */
	unsigned char		ep_num;			/* endpoint number */
	unsigned char		chk_ret;				/* return value */
	int					handle_ret;		/* handle_ep() return value */
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	unsigned short		dummy_read;
#endif

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	up_low_mask = 0xFF;
	shift_bit = 0x00;
	reg_value = 0x00;

	/**
	 * - 1. Check current EPrInt state:
	 *  - read D_EPrIntStat call handle_ep() that is set parameter related to
	 *    interrupt source. loop this routine until EPrInt != 0x00.
	 */
	epr_int = rcD_RegRead8(usbc_dev, rcD_S1R72_EPrIntStat);
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP, D_EPrIntStat , epr_int);
	DEBUG_MSG("%s, EPrIntStat=0x%02x\n", __FUNCTION__, epr_int);

	while ((epr_int) != S1R72_IRQ_NONE){
		DEBUG_MSG("%s, EPrIntStat=0x%02x\n", __FUNCTION__, epr_int);
		/**
		 * - 1.1. Search interrupt source:
		 */
		for(ep_num = S1R72_GD_EPA ; ep_num < S1R72_MAX_ENDPOINT ; ep_num++ ){
			if ( (epr_int & usbc_ep_irq_tbl[ep_num].irq_bit)
				== usbc_ep_irq_tbl[ep_num].irq_bit ){
				break;
			}
		}
		if (ep_num == S1R72_MAX_ENDPOINT){
			break;
		}
		epx = &usbc_dev->usbc_ep[ep_num];

		epx->intenb = S1R72_IRQ_OCCURED;

		switch((epx->bEndpointAddress) & USB_ENDPOINT_DIR_MASK){
		/**
		 * - 1.2. IN endpoint:
		 */
		case USB_DIR_IN:
			epx_int = rcD_RegRead8(usbc_dev, epx->reg.epx.EPxIntStat);

			s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP, ep_num, epx_int);
			DEBUG_MSG("%s, ep in=%d, 0x%04x\n", __FUNCTION__, ep_num, epx_int);
			if ( (epx_int & S1R72_IN_TranACK) == S1R72_IN_TranACK ) {
				/**
				 * - 1.2.1. Clear interrupt:
				 */
				rcD_IntClr8(usbc_dev, epx->reg.epx.EPxIntStat,
					S1R72_IN_TranACK);

				/**
				 * - 1.2.2. Check data remain in FIFO:
				 *  - set Join register and read RdRemain.
				 */
				rcD_RegSet8(usbc_dev, epx->reg.epx.EPxJoin,
					S1R72_JoinCPU_Wr);
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
				/* wait for coming */
				dummy_read = rsD_RegRead16(usbc_dev,rsFIFO_WrRemain);
#endif
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
				if ( rsD_RegRead16(usbc_dev,rsFIFO_WrRemain) > 0 ){
#else
				if ( rsD_RegRead16(usbc_dev,rsFIFO_WrRemain)
					== epx_max_packet_tbl[ep_num]){
#endif
					/**
					 * - 1.2.2.1. Check data remain in reqest queue:
					 *  - if all data in one queue is send to host,
					 *    complete transfer. after that request queue
					 *    is not empty, send data to host.
					 *  - if data remain in one queue, send data to host.
					 */
					/* get queue */
					queued_req = list_entry(epx->queue.next,
						S1R72XXX_USBC_REQ , queue);
					if (queued_req == NULL) {
						break;
					}
					/* transfer completed */
					if (queued_req->req.actual == queued_req->req.length){
						DEBUG_MSG("%s, complete\n", __FUNCTION__);
						if ((queued_req->send_zero == S1R72_SEND_ZERO)
								&& ((queued_req->req.length
									% epx->wMaxPacketSize ) == 0)){
							clear_epx_force_nak(usbc_dev, epx);
							set_ep_enshortpkt(usbc_dev, epx);
							queued_req->send_zero = S1R72_DONT_SEND_ZERO;
						} else {
							request_done(epx, queued_req, 0);
							change_epx_state(epx, S1R72_GD_EP_IDLE);
							/* queue is exist, should handle it */
							if ( list_empty(&epx->queue)
								== S1R72_LIST_ISNOT_EMPTY) {
								change_epx_state(epx, S1R72_GD_EP_DIN);
								handle_ret = handle_ep_in(epx, queued_req);
							}
						}
					/* FIFO is empty and data remain in queue to be send */
					} else {
						DEBUG_MSG("%s, write data remain\n", __FUNCTION__);
						handle_ret = handle_ep_in(epx, queued_req);
					}
				}
				S1R72_EPJOINCPU_CLR(usbc_dev);
			}
			break;
		/**
		 * - 1.3. OUT endpoint:
		 */
		default:
			/**
			 * - 1.3.1. Clear interrupt and set flag:
			 */

			set_epx_force_nak(usbc_dev, epx);
			epx_int = rcD_RegRead8(usbc_dev, epx->reg.epx.EPxIntStat);

			s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP, ep_num, epx_int);
			DEBUG_MSG("%s, ep out=%d, 0x%04x\n",
				__FUNCTION__, ep_num, epx_int);

			if ( (epx_int & S1R72_OUT_ShortACK) == S1R72_OUT_ShortACK ) {
				epx_int
					= ( S1R72_OUT_ShortACK | S1R72_OUT_TranACK );
				epx->last_is_short = S1R72_IRQ_SHORT;
			} else if ((epx_int & S1R72_OUT_TranACK) == S1R72_OUT_TranACK) {
				epx_int = S1R72_OUT_TranACK;
				epx->last_is_short= S1R72_IRQ_IS_NOT_SHORT;
			} else {
				/* clear interrupt */
				rcD_IntClr8(usbc_dev, epx->reg.epx.EPxIntStat, epx_int);
				clear_epx_force_nak(usbc_dev, epx);
				return -EINVAL;
			}

			rcD_RegSet8(usbc_dev, epx->reg.epx.EPxJoin, S1R72_JoinCPU_Rd);

			chk_ret = check_fifo_remain(usbc_dev);
			if (chk_ret == S1R72_RD_REMAIN_NG){
				S1R72_EPJOINCPU_CLR(usbc_dev);
				if ( epx->last_is_short == S1R72_IRQ_IS_NOT_SHORT){
					clear_epx_force_nak(usbc_dev, epx);
				}
				return 0;
			}

			/**
			 * - 1.3.2. Force NAK:
			 *  - if short packet recieved, force NAK transfer.
			 */

			epx->fifo_data = rsD_RegRead16(usbc_dev, rsFIFO_RdRemain)
				& FIFO_RdRemain;

			/* clear interrupt */
			rcD_IntClr8(usbc_dev, epx->reg.epx.EPxIntStat, epx_int);

			s1r72xxx_queue_log(S1R72_DEBUG_IRQ_EP, ep_num, epx->fifo_data);

			if( epx->last_is_short == S1R72_IRQ_IS_NOT_SHORT){
				clear_epx_force_nak(usbc_dev, epx);
			}
			S1R72_EPJOINCPU_CLR(usbc_dev);
			epx->intenb = S1R72_IRQ_OCCURED;
			DEBUG_MSG("%s, OUT\n", __FUNCTION__);

			/**
			 * - 1.3.3. Read data from FIFO:
			 *  - call handle_ep_out to read data from FIFO.
			 */
			handle_ret = handle_ep_out(epx ,NULL);

			/**
			 * - 1.3.4. Change state:
			 *  - if queue is empty, change endpoint state to DOUT.
			 *    it means data recieved.
			 */
			if ( list_empty(&epx->queue) != S1R72_LIST_ISNOT_EMPTY) {
				change_epx_state(epx, S1R72_GD_EP_DOUT);
			}
			break;
		}

		/**
		 * - 2. Check interrupt source:
		 *  - to check interrupt.
		 */
		epr_int = rcD_RegRead8(usbc_dev, rcD_S1R72_EPrIntStat);
	}

	/**
	 * - 3. Return:
	 *  - return 0;
	 */
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- process FinishPM interrupt.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure .
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_irq_FPM(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned int current_pmstate;	/* MainIntStat */
	unsigned int handshake_retval;	/* handshake status */
	unsigned int retval;			/* return value */

	current_pmstate = rcD_RegRead8(usbc_dev, rcPM_Control_1) & PM_State_Mask;
	handshake_retval = 0;
	retval = 0;
	rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);
	s1r72xxx_queue_log(S1R72_DEBUG_IRQ_FPM, rcPM_Control_1, current_pmstate);

	/**
	 * - 1. Check current PM_Control_1:
	 *  - read PM_Control_1 and process below.
	 */
	DEBUG_MSG("%s, PM_Control=0x%02x\n", __FUNCTION__, current_pmstate);
	switch(current_pmstate){
	/**
	 * - 1.1. ACT_DEVICE:
	 */
	case S1R72_PM_State_ACT_DEVICE:
		/**
		 * - 1.1.1. Enable SIE interrupt:
		 *  - enable sie interrupt, DetectReset, DetectSuspend, ChirpCmp
		 *    RestoreCmp, SetAddressCmp.
		 */
		s1r72xxx_queue_log(S1R72_DEBUG_IRQ_VBUS,rcD_USB_Status ,
				(rcD_RegRead8(usbc_dev, rcD_USB_Status)));
		s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rcD_S1R72_NegoControl ,
				rcD_RegRead8(usbc_dev, rcD_S1R72_NegoControl));
		s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rcD_S1R72_XcvrControl ,
				rcD_RegRead8(usbc_dev, rcD_S1R72_XcvrControl));

		usbc_fifo_init(usbc_dev);
		rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat,
			(S1R72_NonJ | S1R72_EnDetectRESET | S1R72_EnDetectSUSPEND
				| S1R72_EnChirpCmp | S1R72_EnRestoreCmp
				| S1R72_EnSetAddressCmp));
		rcD_RegSet8(usbc_dev, rcD_S1R72_SIE_IntEnb,
			(S1R72_EnDetectRESET | S1R72_EnDetectSUSPEND
				| S1R72_EnChirpCmp | S1R72_EnRestoreCmp
				| S1R72_EnSetAddressCmp));

		/**
		 * - 1.1.2. Resume device:
		 *  - if driver state is USBSUSPD, resume device.
		 */
		/* Driver activated */
		DEBUG_MSG("%s, ACT_DEVICE\n", __FUNCTION__);
		if ( (usbc_dev->usbcd_state == S1R72_GD_USBSUSPD)
				&& (usbc_dev->remote_wakeup_prc != S1R72_RT_WAKEUP_PROC) ){
			DEBUG_MSG("%s, USBSUSPD\n", __FUNCTION__);
			/* start resume */
			if ((rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_LineState)
					!= S1R72_LineState_J){
			DEBUG_MSG("%s, non J\n", __FUNCTION__);
				rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl,S1R72_InSUSPEND);
			} else {
			DEBUG_MSG("%s, J\n", __FUNCTION__);
				rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, S1R72_NonJ);
				rcD_RegSet8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_NonJ);
				usbc_dev->usbcd_state
					= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
						S1R72_GD_H_W_SUSPEND);
				break;
			}
		}

		/**
		 * - 1.1.3. Send wakeup signal:
		 *  - if gadget driver call wakeup API, send wakeup signal to host.
		 */
		/* remote wakeup has occured */
		if (usbc_dev->remote_wakeup_prc == S1R72_RT_WAKEUP_PROC){
			DEBUG_MSG("%s, Remote wakeup\n", __FUNCTION__);
			/* send wakeup signal */
			rcD_RegSet8(usbc_dev, rcD_S1R72_NegoControl, S1R72_SendWakeup);

			/* start send wakeup timer */
			mod_timer(&usbc_dev->rm_wakeup_timer, S1R72_SEND_WAKEUP_TIME);

		/**
		 * - 1.1.4. Start handshake:
		 *  - start handshake to host. if handshake failed, start
		 *    wait_j_timer to check USB bus status.
		 */
		} else {
			/* start handshake */
			handshake_retval = usbc_start_handshake(usbc_dev);

			if (handshake_retval == S1R72_RET_OK){
				DEBUG_MSG("%s, start handshake\n", __FUNCTION__);

				/* change state */
				usbc_dev->usbcd_state
					= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
						S1R72_GD_H_W_ACT_DEVICE);

				/* change ep0 state to IDLE */
				change_ep0_state(&usbc_dev->usbc_ep[S1R72_GD_EP0],
					S1R72_GD_EP_IDLE);
			} else {
				DEBUG_MSG("%s, fail handshake\n", __FUNCTION__);
				/* wait J state timer */
				mod_timer(&usbc_dev->wait_j_timer,
					S1R72_WAIT_J_TIMEOUT_TM);
			}
		}
		break;

	/**
	 * - 1.2. SLEEP:
	 */
	case S1R72_PM_State_SLEEP:
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
	case S1R72_PM_State_SLEEP_01:
#endif
#if defined(CONFIG_USB_S1R72V05_GADGET) \
			|| defined(CONFIG_USB_S1R72V05_GADGET_MODULE)
	case S1R72_PM_State_ACTIVE60:
#endif
		/**
		 * - 1.2.1. Change state:
		 *  - change driver state.
		 */
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
		if ((current_pmstate == S1R72_PM_State_SLEEP)
		  ||(current_pmstate == S1R72_PM_State_SLEEP_01)){
#else
		if (current_pmstate == S1R72_PM_State_SLEEP){
#endif
			usbc_dev->usbcd_state
				= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
					S1R72_GD_H_W_SLEEP);

#if defined(CONFIG_USB_S1R72V05_GADGET) \
			|| defined(CONFIG_USB_S1R72V05_GADGET_MODULE)
		} else {
			usbc_dev->usbcd_state
				= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
					S1R72_GD_H_W_ACTIVE60);
#endif
		}
		/**
		 * - 1.2.2. Set interrupt:
		 *  - make the interrupts enable.
		 */
		if (usbc_dev->usbcd_state == S1R72_GD_USBSUSPD){
			rcD_IntClr8(usbc_dev, rcD_S1R72_SIE_IntStat, S1R72_NonJ);
			rcD_RegWrite8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_NonJ);
		} else {
			rcD_RegWrite8(usbc_dev, rcD_S1R72_SIE_IntEnb, S1R72_REG_ALL_CLR);
		}

		/**
		 * - 1.2.3. Set return value:
		 *  - set return value that means changed to sleep.
		 */
		retval = S1R72_PM_CHANGE_TO_SLEEP;
		DEBUG_MSG("%s, SLEEP\n", __FUNCTION__);
		break;

	default:
		/**
		 * - 1.3. another:
		 *  - nothing to do.
		 */
		/* This should not occur */
		DEBUG_MSG("%s, error 0x%02x\n", __FUNCTION__,current_pmstate);
		break;
	}

	/**
	 * - 2. Return:
	 *  - return 0;
	 */
	return retval;
}

/* ========================================================================= */
/**
 * @brief	- Pull up D+ Line.
 * @par		usage:
 *				internal use only.
 * @param	d_pull_up;	pull up or open.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int usbc_pullup(int d_pull_up)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - note: S1Rxxx device pull up D+ line automaticaly.
	 *   external devices(ex. CPU) couldn't operation this.
	 */

	/**
	 * - 1. Check parameter:
	 *  - if d_pull_up isn't 1, then return -EOPNOTSUPP.
	 */
	if (d_pull_up != 1) {
		/* s1r72xxx devices pullup automatically */
		DEBUG_MSG("%s, pullup disable is not supported\n", __FUNCTION__);
		return -EOPNOTSUPP;
	}

	/**
	 * - 2. Return:
	 *  - return 0;
	 */
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- process EP.
 * @par		usage:
 *				internal use only.
 * @param	*ep;	endpoint informations structure.
 * @param	*req;	reqest to this endpoint.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int handle_ep_in(S1R72XXX_USBC_EP *ep, S1R72XXX_USBC_REQ *req)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_REQ	*queued_req;	/* request */
	unsigned int	access_length;		/* write or read data length */
	unsigned int	max_packet;			/* max packet size */
	unsigned int	max_fifo;			/* max fifo size */
	unsigned int	length_ct;			/* write data length counter */
	unsigned int	write_length;		/* write data length */
	unsigned int	fifo_remain;		/* fifo remain */
	unsigned int	is_odd;				/* data length is odd (=1) */
	unsigned short	*fifo_16;			/* fifo pointer for 16 bit access */
	unsigned char	*fifo_8;			/* fifo pointer for 8 bit access */
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
	unsigned short	dummy_data;			/* dummy data for 16bit fixed access */
#endif
	unsigned char	reg_value;			/* register value */
	int				ret_val;			/* return value */
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
	unsigned char	chk_ret;			/* return value */
#endif
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	unsigned short	dummy_read;
#endif

	access_length = 0;
	length_ct = 0;
	fifo_remain = 0;
	is_odd = S1R72_DATA_EVEN;
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
	dummy_data = 0;
#endif
	reg_value = 0;
	ret_val = S1R72_QUEUE_REMAIN;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	usbc_dev = the_controller;

	/**
	 * - 1. Find queued data:
	 *  - serch queue list.
	 */
	if ( list_empty(&ep->queue) == S1R72_LIST_ISNOT_EMPTY ) {
		DEBUG_MSG("%s, already queued\n", __FUNCTION__);
		queued_req = list_entry(ep->queue.next, S1R72XXX_USBC_REQ , queue);
	} else {
		DEBUG_MSG("%s, new queue\n", __FUNCTION__);
		queued_req = req;
	}
	s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP, ep->ep_subname, 0);
	if ( queued_req == NULL) {
		DEBUG_MSG("%s, queue is not exist\n", __FUNCTION__);
		return ret_val;
	}

	/**
	 * - 2. Check state.:
	 *  - check epx driver state.
	 */
	DEBUG_MSG("%s, ep_state=%d\n", __FUNCTION__, ep->ep_state);
	switch(ep->ep_state){
	/**
	 * - 2.1. Write data to FIFO:
	 *  - data size to be written is equal or smaller than FIFO size ,
	 *    then write all data and set EnShortPkt.
	 *    if data size to be written is larger than FIFO size ,
	 *    then write data size of FIFO.
	 *    add written length to 'actual'.
	 */
	case S1R72_GD_EP_DIN:
		ep->intenb = S1R72_IRQ_NOT_OCCURED;

		DEBUG_MSG("%s, IN ack\n", __FUNCTION__);

		/**
		 * - 2.1.1. Set Join register:
		 *  - set join register to write FIFO.
		 */
		DEBUG_MSG("%s, ep=%d\n", __FUNCTION__, ep->ep_subname);
		if (ep->ep_subname == S1R72_GD_EP0) {
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Join, S1R72_JoinCPU_Wr);
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Control, S1R72_EP0INxOUT_IN);
			max_packet = S1R72_EP0_MAX_PKT;
		} else {
			rcD_RegSet8(usbc_dev, ep->reg.epx.EPxJoin, S1R72_JoinCPU_Wr);
			max_packet = ep->wMaxPacketSize;
		}

		/**
		 * - 2.1.2. Calculate write data length:
		 *  - write data length is max FIFO size or all data size.
		 *    if max FIFO size is larger than all data size, write
		 *    all data.
		 */
		/* FIFO data check */
		max_fifo =epx_max_packet_tbl[ep->ep_subname] ;
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/* wait for coming */
		dummy_read = rsD_RegRead16(usbc_dev, rsFIFO_WrRemain);
#endif
		fifo_remain =
			(unsigned int)rsD_RegRead16(usbc_dev, rsFIFO_WrRemain);
		DEBUG_MSG("%s, fifo_remain=%d\n", __FUNCTION__, fifo_remain);

		/* data length to be written */
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		access_length = queued_req->req.length - queued_req->req.actual;
		if( access_length >= fifo_remain ){
			if( fifo_remain < max_packet){
				access_length = 0;
			}else{
				if( max_packet > 0 ){
					/* how many packets in the remain */
					access_length = fifo_remain / max_packet;
					access_length *= max_packet;
				}else{
					access_length = 0;
					DEBUG_MSG("%s, wrong max_packet=%d\n", __FUNCTION__, max_packet);
				}
			}
		}
#else
		access_length = min(
			queued_req->req.length - queued_req->req.actual , max_fifo );
#endif
		s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP,
			ep->ep_subname, access_length);
		s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP,
			ep->ep_subname, queued_req->req.actual);
		s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP_SN,
			ep->ep_subname, queued_req->queue_seq_no);
		DEBUG_MSG("%s, access_length=%d\n", __FUNCTION__, access_length);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		if ((( ep->fifo_addup == FIFO_ADDUP_EVEN ) && ( access_length == 0 )
			/* exept zero length packet */
			&& ( queued_req->req.length > 0 ))
		 || (( ep->fifo_addup == FIFO_ADDUP_ODD )  && ( fifo_remain < max_fifo ))
		 ){
#else
		/* FIFO is not empty */
		if ( fifo_remain < max_fifo ) {
#endif
			DEBUG_MSG("%s, fifo has data\n", __FUNCTION__);
			/* clear Join */
			if (ep->ep_subname == S1R72_GD_EP0) {
				rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0Control,
					S1R72_EP0INxOUT_IN);
			}
			S1R72_EPJOINCPU_CLR(usbc_dev);
			return ret_val;
		}
		s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rsFIFO_WrRemain,
			le16_to_cpu(rsD_RegRead16(usbc_dev,rsFIFO_WrRemain)));

		/**
		 * - 2.1.3. Write data to FIFO:
		 */
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/**
		 * - 2.1.3.1. manage against FIFO fraction:
		 */
		if(ep->fifo_addup == FIFO_ADDUP_ODD){
			/**
			 * - 2.1.3.1.1. clear FIFO:
			 */
			S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);

			/**
			 * - 2.1.3.1.2. clear FIFO addup
			 */
			ep->fifo_addup = FIFO_ADDUP_CLR;
		}
#else
		/* FIFO clear */
		S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);
#endif
		length_ct = access_length;
		write_length = access_length;
#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
		if ( ( length_ct % 2 ) != 0 ) {
			if (ep->ep_subname == S1R72_GD_EP0) {
				set_ep0_in_force_nak(usbc_dev, ep);
			} else {
				set_epx_force_nak(usbc_dev, ep);
			}
			fifo_8 = (unsigned char*) (queued_req->req.buf
				+ queued_req->req.actual);
			dummy_data = (*fifo_8) << 8;
			rsD_RegWrite16(usbc_dev, rsFIFO_Wr, dummy_data);
			access_length--;
			length_ct--;
			queued_req->req.actual++;
			is_odd = S1R72_DATA_ODD;
			DEBUG_MSG("%s, data=0x%04x\n", __FUNCTION__, *fifo_8);
		}
#endif
		fifo_16 = (unsigned short*) (queued_req->req.buf
			+ queued_req->req.actual);
		while( length_ct > 1 ) {
			rsD_RegWrite16(usbc_dev, rsFIFO_Wr, *fifo_16);
			DEBUG_MSG("%s, data=0x%04x\n", __FUNCTION__, *fifo_16);
			fifo_16++;
			length_ct -= 2;
		}

#if defined(CONFIG_USB_S1R72V17_GADGET_16BIT_FIX)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
		if(is_odd == S1R72_DATA_ODD) {
			DEBUG_MSG("%s, odd\n", __FUNCTION__);
			/* clear Join */
			S1R72_EPJOINCPU_CLR(usbc_dev);
			if (ep->ep_subname == S1R72_GD_EP0) {
				rcD_RegWrite8(usbc_dev, ep->reg.ep0.EP0Join, S1R72_JoinCPU_Rd);
			} else {
				rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxJoin, S1R72_JoinCPU_Rd);
			}

			chk_ret = check_fifo_remain(usbc_dev);
			dummy_data = rcD_RegRead8(usbc_dev, rcFIFO_ByteRd);
			S1R72_EPJOINCPU_CLR(usbc_dev);
			if (ep->ep_subname == S1R72_GD_EP0) {
				rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Join, S1R72_JoinCPU_Wr);
			} else {
				rcD_RegSet8(usbc_dev, ep->reg.epx.EPxJoin, S1R72_JoinCPU_Wr);
			}
			DEBUG_MSG("%s, data=0x%04x\n", __FUNCTION__, dummy_data);
		}
#else	/* _16BIT_FIX */
		if(length_ct == 1) {
			DEBUG_MSG("%s, odd\n", __FUNCTION__);
			fifo_8 = (unsigned char*) (queued_req->req.buf
				+ queued_req->req.actual+ access_length - 1);
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)

			rcD_RegWrite8(usbc_dev, rcFIFO_ByteWr, *fifo_8);

			/**
			 * - 2.1.3.2. renew FIFO addup:
			 */
			ep->fifo_addup = FIFO_ADDUP_ODD;
#else
			rcD_RegWrite8(usbc_dev, rcFIFO_Wr_0, *fifo_8);
#endif
			DEBUG_MSG("%s, data=0x%02x\n", __FUNCTION__, *fifo_8);
		}
#endif	/* _16BIT_FIX */
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/**
		 * - 2.1.3.3. wait to flush cache:
		 */
		if( check_fifo_cache(usbc_dev) == S1R72_CACHE_REMAIN_NG){
			s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rcCacheRemain,
				rcD_RegRead8(usbc_dev, rcCacheRemain));
		}

		/* data size is smaller than FIFO size */
		dummy_read = rsD_RegRead16(usbc_dev, rsFIFO_WrRemain);
#endif
		fifo_remain =
			(unsigned int)rsD_RegRead16(usbc_dev, rsFIFO_WrRemain);
		s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rsFIFO_WrRemain, fifo_remain);
		DEBUG_MSG("%s, fifo_remain=%d\n", __FUNCTION__, fifo_remain);
		DEBUG_MSG("%s, access_length=%d,max_packet=%d\n",
			__FUNCTION__, access_length, max_packet);

		/**
		 * - 2.1.4. Complete write data:
		 *  - if write data length is not packet length, set
		 *    EnShortPkt and clear ForceNAK.
		 *  - if write data length is packet length, clear ForceNAK
		 */
		if ( ( ( write_length % max_packet ) != 0 )
			|| ( write_length  == 0 ) ) {
			DEBUG_MSG("%s, access_length < max_packet\n", __FUNCTION__);
			/* send short packet */
			if (ep->ep_subname == S1R72_GD_EP0) {
				/* clear ForceNAK bit and set EnShortPkt */
				clear_ep0_in_force_nak(usbc_dev, ep);
			} else {
				clear_epx_force_nak(usbc_dev, ep);
			}
			set_ep_enshortpkt(usbc_dev, ep);
			queued_req->send_zero = S1R72_DONT_SEND_ZERO;
			s1r72xxx_queue_log(S1R72_DEBUG_SHORT, ep->ep_subname,
				access_length);
		} else {
			if (ep->ep_subname == S1R72_GD_EP0) {
				clear_ep0_in_force_nak(usbc_dev, ep);
			} else {
				clear_epx_force_nak(usbc_dev, ep);
			}
		}
		queued_req->req.actual += access_length;

		/* clear Join */
		S1R72_EPJOINCPU_CLR(usbc_dev);

		/**
		 * - 2.1.5. Change state(only EP0):
		 *  - At EP0 data write is completed, change state to DIN CHG
		 *    that means waiting status stage.
		 */
		DEBUG_MSG("%s, actual=%d,length=%d\n",
			__FUNCTION__, queued_req->req.actual,queued_req->req.length);
		if ( (queued_req->req.actual) == (queued_req->req.length) ){
			if (ep->ep_subname == S1R72_GD_EP0) {
				change_ep0_state(ep, S1R72_GD_EP_DIN_CHG);
			}
		}
		break;

	default:
		/**
		 * - 2.2. error:
		 */
		DEBUG_MSG("%s, illegal request recieved.\n", __FUNCTION__);
		DEBUG_MSG("%s, state = 0x%02x, flag=0x%02x\n", __FUNCTION__,
			ep->ep_state, 0);
		if (ep->ep_subname == S1R72_GD_EP0) {
			clear_ep0_in_force_nak(usbc_dev, ep);
		} else {
			clear_epx_force_nak(usbc_dev, ep);
		}
		break;
	}
	/**
	 * - 3. Return:
	 *  - return;
	 */
	return ret_val;
}

/* ========================================================================= */
/**
 * @brief	- process OUT EP.
 * @par		usage:
 *				internal use only.
 * @param	*ep;	endpoint informations structure.
 * @param	*req;	reqest to this endpoint.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int handle_ep_out(S1R72XXX_USBC_EP *ep, S1R72XXX_USBC_REQ *req)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_REQ	*queued_req;	/* request */
	unsigned int	access_length;		/* write or read data length */
	unsigned int	remain_length;		/* queued remaining data length */
	unsigned int	length_ct;			/* write or read data length */
	unsigned int	retry_flag;			/* fifo read retry flag */
	unsigned int	fifo_remain;		/* fifo remain */
	unsigned int	is_odd;				/* data length is odd (=1) */
	unsigned short	*fifo_16;			/* fifo pointer for 16 bit access */
	unsigned char	*fifo_8;			/* fifo pointer for 8 bit access */
	int				ret_val;			/* return value */
	unsigned char	chk_ret;			/* return value */
	unsigned char	call_src;			/* called source */

	access_length = 0;
	length_ct = 0;
	fifo_remain = 0;
	is_odd = S1R72_DATA_EVEN;
	ret_val = S1R72_QUEUE_REMAIN;
	call_src = S1R72_CALLED_IRQ;

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	usbc_dev = container_of(ep, S1R72XXX_USBC_DEV, usbc_ep[ep->ep_subname]);

	/**
	 * - 1. Find queued data:
	 *  - serch queue list.
	 */
	if (req != NULL){
		call_src = S1R72_CALLED_EP_QUEUE;
	}
	if ( list_empty(&ep->queue) == S1R72_LIST_ISNOT_EMPTY ) {
		DEBUG_MSG("%s, already queued\n", __FUNCTION__);
		queued_req = list_entry(ep->queue.next, S1R72XXX_USBC_REQ , queue);
	} else {
		DEBUG_MSG("%s, new queue\n", __FUNCTION__);
		queued_req = req;
	}
	s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP, ep->ep_subname, 0);
	if ( queued_req == NULL) {
		return ret_val;
	}

	/**
	 * - 2. Check state.:
	 *  - check epx driver state.
	 */
	DEBUG_MSG("%s, ep_state=%d\n", __FUNCTION__, ep->ep_state);
	switch(ep->ep_state){
	/**
	 * - 2.1. Read data from FIFO:
	 *  - read data from FIFO and write to buffer
	 *    and add written length to 'actual' member.
	 */
	case S1R72_GD_EP_DOUT:
		/**
		 * - 2.1.1. Initialize flags:
		 *   - initialize interrupt flag and FIFO read retry flag.
		 */
		DEBUG_MSG("%s, Out ack\n", __FUNCTION__ );
		s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP,
			ep->intenb, ep->last_is_short);

		if (ep->intenb == S1R72_IRQ_NOT_OCCURED){
			DEBUG_MSG("%s, did not recieve any data.\n", __FUNCTION__ );
			return ret_val;
		}
		retry_flag = S1R72_FIFO_READ_RETRY;

		/**
		 * - 2.1.2. Read data loop:
		 *  - read data from FIFO until FIFO or data queue will be
		 *    empty.
		 */
		while(retry_flag == S1R72_FIFO_READ_RETRY){
			/**
			 * - 2.1.2.1. Set Join register:
			 *  - set join register to read FIFO.
			 */
			ep->intenb = S1R72_IRQ_NOT_OCCURED;
			retry_flag = S1R72_FIFO_READ_COMP;
			/* set Join */
			DEBUG_MSG("%s, ep=%d\n", __FUNCTION__, ep->ep_subname);
			if (ep->ep_subname == S1R72_GD_EP0) {
				rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Join, S1R72_JoinCPU_Rd);
				rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0Control,
					S1R72_EP0INxOUT_IN);
			} else {
				rcD_RegSet8(usbc_dev, ep->reg.epx.EPxJoin, S1R72_JoinCPU_Rd);
			}
			s1r72xxx_queue_log(S1R72_DEBUG_REGISTER,ep->reg.epx.EPxControl,
				rcD_RegRead8(usbc_dev, ep->reg.epx.EPxControl));
			s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP_SN,
				ep->ep_subname, queued_req->queue_seq_no);

			/**
			 * - 2.1.2.2. Calculate read data length:
			 *  - read data length is data in FIFO or queue buffer size.
			 *    if data in FIFO is smaller than or equal to
			 *    queue buffer size, read all data.
			 */
			fifo_remain = ep->fifo_data;
			DEBUG_MSG("%s, RdRemain=0x%02x\n", __FUNCTION__, fifo_remain);
			/* data length to be read */
			remain_length = queued_req->req.length - queued_req->req.actual;
			access_length = min((unsigned int)fifo_remain, remain_length);

			/**
			 * - 2.1.2.3. Read data from FIFO:
			 */
			chk_ret = check_fifo_remain(usbc_dev);
			if (chk_ret == S1R72_RD_REMAIN_NG){
				S1R72_EPJOINCPU_CLR(usbc_dev);
				if ( ep->last_is_short == S1R72_IRQ_IS_NOT_SHORT){
					clear_epx_force_nak(usbc_dev, ep);
				}
				return ret_val;
			}
			/* data read */
			DEBUG_MSG("%s, access_length=%d, fifo_remain=%d\n",
				__FUNCTION__, access_length, fifo_remain);
			fifo_16 = (unsigned short*) (queued_req->req.buf
				+ queued_req->req.actual);
			length_ct = access_length;
			while( length_ct > 1 ) {
				*fifo_16 = rsD_RegRead16(usbc_dev, rsFIFO_Rd);
				DEBUG_MSG("%s, data=0x%04x\n", __FUNCTION__, *fifo_16);
				fifo_16++;
				length_ct -= 2;
			}

			if(length_ct == 1) {
				DEBUG_MSG("%s, odd\n", __FUNCTION__);
				fifo_8 = (unsigned char*) (queued_req->req.buf
					+ queued_req->req.actual+ access_length - 1);
				*fifo_8 = (unsigned char)rcD_RegRead8(usbc_dev,
					rcFIFO_ByteRd);
				DEBUG_MSG("%s, data=0x%02x\n", __FUNCTION__, *fifo_8);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
				/**
				 * - 2.1.2.3.1. set FIFO addup:
				 */
				ep->fifo_addup = FIFO_ADDUP_ODD;
#endif
			}
			queued_req->req.actual += access_length;
			ep->fifo_data -= access_length;

			/* clear FIFO */
			if ( ( ep->last_is_short == S1R72_IRQ_SHORT )
				&& (ep->fifo_data == S1R72_FIFO_EMPTY) ){
				S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
				/**
				 * - 2.1.2.3.2. clear FIFO addup:
				 */
				ep->fifo_addup = FIFO_ADDUP_CLR;
#endif
				if (ep->ep_subname != S1R72_GD_EP0) {
					clear_epx_force_nak(usbc_dev, ep);
				}
			}

			/**
			 * - 2.1.2.4. Complete transfer:
			 *  - if short packet recieved, complete OUT transfer.
			 *    and if another request is queued, check FIFO data again.
			 */
			s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP,
				ep->ep_subname, access_length);
			DEBUG_MSG("%s, actual=%d, length=%d\n", __FUNCTION__,
				queued_req->req.actual, queued_req->req.length);

			if ( ( ep->last_is_short == S1R72_IRQ_SHORT )
				|| ( (queued_req->req.actual)
					== (queued_req->req.length) ) ){
				s1r72xxx_queue_log(S1R72_DEBUG_HANDLE_EP,
					queued_req->req.actual, queued_req->req.length);
				request_done(ep, queued_req, 0);
				fifo_remain = ep->fifo_data;

				/* if queue exist, should not change state */
				DEBUG_MSG("%s, fifo_rem=%d\n", __FUNCTION__, fifo_remain);
				if ( list_empty(&ep->queue) != S1R72_LIST_ISNOT_EMPTY) {
					DEBUG_MSG("%s, queue is empty\n", __FUNCTION__);
					if (fifo_remain == S1R72_FIFO_EMPTY) {
						if (ep->ep_subname == S1R72_GD_EP0) {
							/* change to status stage */
							change_ep0_state(ep, S1R72_GD_EP_SIN);
						} else {
							change_epx_state(ep, S1R72_GD_EP_IDLE);
						}
						ep->last_is_short = S1R72_IRQ_IS_NOT_SHORT;
					} else {
						ep->intenb = S1R72_IRQ_OCCURED;
					}
				} else {
					DEBUG_MSG("%s, queue is not empty\n", __FUNCTION__);
					queued_req = list_entry(ep->queue.next,
						S1R72XXX_USBC_REQ , queue);
					ep->intenb = S1R72_IRQ_OCCURED;
					if (fifo_remain == S1R72_FIFO_EMPTY) {
						if ( ep->last_is_short == S1R72_IRQ_SHORT ) {
							if (ep->ep_subname == S1R72_GD_EP0) {
								/* change to status stage */
								change_ep0_state(ep, S1R72_GD_EP_SIN);
							} else {
								change_epx_state(ep, S1R72_GD_EP_IDLE);
								change_epx_state(ep, S1R72_GD_EP_DOUT);
							}
						}
						ep->last_is_short = S1R72_IRQ_IS_NOT_SHORT;
					} else {
						retry_flag = S1R72_FIFO_READ_RETRY;
					}
				}
				DEBUG_MSG("%s, queue done\n", __FUNCTION__);
				ret_val = S1R72_QUEUE_DONE;
			}
			S1R72_EPJOINCPU_CLR(usbc_dev);
		} /* while end */
		break;

	default:
		/**
		 * - 2.2. error:
		 */
		DEBUG_MSG("%s, illegal request recieved.\n", __FUNCTION__);
		DEBUG_MSG("%s, state = 0x%02x, flag=0x%02x\n", __FUNCTION__,
			ep->ep_state, 0);
		break;
	}
	/**
	 * - 3. Return:
	 *  - return;
	 */
	return ret_val;
}


/* ========================================================================= */
/**
 * @brief	- change EP0 state and set hardware.
 * @par		usage:
 *				internal use only.
 * @param	*ep;	endpoint informations structure.
 * @param	flag;	flag to change state.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int change_ep0_state(S1R72XXX_USBC_EP *ep, unsigned char flag)
{
	S1R72XXX_USBC_REQ	*req;			/* USB request */
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	unsigned char	reg_value;			/* register value */

	reg_value = 0;
	usbc_dev = the_controller;

	DEBUG_MSG("%s,enter\n",__FUNCTION__);
	s1r72xxx_queue_log(S1R72_DEBUG_CHG_EP0, 0, flag);

	/**
	 * - 1. Check flag:
	 */
	switch(flag){

	/**
	 * - 1.1. flag = S1R72_GD_EP_INIT:
	 *  - stop ep0 and change state to S1R72_GD_EP_INIT.
	 */
	case S1R72_GD_EP_INIT:
		DEBUG_MSG("%s, S1R72_GD_EP_INIT\n",__FUNCTION__);

		/**
		 * - 1.1.1. Interrupt disabling:
		 *  - disables interrupt registers on the usb device.
		 *    D_EP0IntEnb.
		 */
		rcD_RegClear8(usbc_dev, rcDeviceIntEnb, D_RcvEP0SETUP);
		rcD_IntClr8(usbc_dev, rcDeviceIntStat, D_RcvEP0SETUP);

		rcD_IntClr8(usbc_dev, ep->reg.ep0.EP0IntStat, S1R72_EP_ALL_INT);
		rcD_RegWrite8(usbc_dev, ep->reg.ep0.EP0IntEnb, S1R72_REG_ALL_CLR);
		DEBUG_MSG("%s, EP0IntEnb\n",__FUNCTION__);

		/**
		 * - 1.1.2. Clear FIFO:
		 */
		S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);
		DEBUG_MSG("%s, Stop endpoint0\n",__FUNCTION__);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/**
		 * - 1.1.2.1. clear FIFO_addup:
		 */
		ep->fifo_addup = FIFO_ADDUP_CLR;
#endif
		/**
		 * - 1.1.3. Dequeue the remaining data:
		 *  - delete all queues from list.
		 */
		while ( list_empty(&ep->queue) == S1R72_LIST_ISNOT_EMPTY ) {
			req = list_entry(ep->queue.next, S1R72XXX_USBC_REQ, queue);
			request_done(ep, req, -ESHUTDOWN);
		}
		DEBUG_MSG("%s, Dequeue0\n",__FUNCTION__);

		/**
		 * - 1.1.4. Change state initialize:
		 *  - change a state to S1R72_GD_EP_INIT.
		 */
		ep->ep_state = S1R72_GD_EP_INIT;
		DEBUG_MSG("%s, Change state initialize\n",__FUNCTION__);
		break;

	/**
	 * - 1.2. flag = S1R72_GD_EP_IDLE:
	 *  - configure ep0 and change state to S1R72_GD_EP_IDLE.
	 */
	case S1R72_GD_EP_IDLE:
		DEBUG_MSG("%s, S1R72_GD_EP_IDLE\n",__FUNCTION__);
		/**
		 * - 1.2.1. Configure ep0 registers:
		 *  - enable ep0. EP0IntEnb, DeviceIntEnb, EP0MaxSize.
		 */
		if (ep->ep_state == S1R72_GD_EP_INIT){
			rcD_IntClr8(usbc_dev, ep->reg.ep0.EP0IntStat,
				 S1R72_IN_TranACK | S1R72_OUT_TranACK
					| S1R72_OUT_ShortACK);
			rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0IntEnb,
				 S1R72_EnIN_TranACK | S1R72_EnOUT_TranACK
					| S1R72_EnOUT_ShortACK);
			rcD_IntClr8(usbc_dev, rcDeviceIntStat,
				 D_EP0IntStat | D_RcvEP0SETUP);
			rcD_RegSet8(usbc_dev, rcDeviceIntEnb,
				 D_EP0IntStat | D_RcvEP0SETUP);
			rcD_RegWrite8(usbc_dev, ep->reg.ep0.EP0MaxSize,
				S1R72_HS_CNT_MAX_PKT);
			S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);
		}

		/**
		 * - 1.2.2. Change state to IDLE:
		 *  - change state to S1R72_GD_EP_IDLE.
		 */
		ep->ep_state = S1R72_GD_EP_IDLE;
		break;

	/**
	 * - 1.3. flag = S1R72_GD_EP_DOUT:
	 *  - configure ep0 OUT and change state to S1R72_GD_EP_DOUT.
	 */
	case S1R72_GD_EP_DOUT:
		DEBUG_MSG("%s, S1R72_GD_EP_DOUT\n",__FUNCTION__);

		/**
		 * - 1.3.1. Configure ep0 registers:
		 *  - configure D_SETUP_Control.ProtectEP0,
		 *    D_EP0ControlOUT.ForceNAK.
		 */
		rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0Control, S1R72_EP0INxOUT_IN);
		clear_ep0_out_force_nak(usbc_dev, ep);

		/**
		 * - 1.3.2. Change state to S1R72_GD_EP_DOUT:
		 *  - change state to S1R72_GD_EP_DOUT.
		 */
		ep->ep_state = S1R72_GD_EP_DOUT;
		break;

	/**
	 * - 1.4. flag = S1R72_GD_EP_DIN:
	 *  - configure ep0 IN and change state to S1R72_GD_EP_DIN.
	 */
	case S1R72_GD_EP_DIN:
		DEBUG_MSG("%s, S1R72_GD_EP_DIN\n",__FUNCTION__);

		/**
		 * - 1.4.1. Configure ep0 registers:
		 *  - configure D_EP0Control direction IN
		 */
		rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Control, S1R72_EP0INxOUT_IN);

		/**
		 * - 1.4.2. Change state to S1R72_GD_EP_DIN:
		 *  - change state to S1R72_GD_EP_DIN.
		 */
		ep->ep_state = S1R72_GD_EP_DIN;
		break;

	/**
	 * - 1.5. flag = S1R72_GD_EP_DIN_CHG:
	 *  - change state to S1R72_GD_EP_DIN_CHG.
	 */
	case S1R72_GD_EP_DIN_CHG:
		DEBUG_MSG("%s, S1R72_GD_EP_DIN_CHG\n",__FUNCTION__);
		/**
		 * - 1.5.1. Change state to S1R72_GD_EP_DIN_CHG:
		 *  - change state to S1R72_GD_EP_DIN_CHG.
		 */
		ep->ep_state = S1R72_GD_EP_DIN_CHG;
		break;

	/**
	 * - 1.6. flag = S1R72_GD_EP_SOUT:
	 *  - change state to S1R72_GD_EP_SOUT.
	 */
	case S1R72_GD_EP_SOUT:
		DEBUG_MSG("%s, S1R72_GD_EP_SOUT\n",__FUNCTION__);

		/**
		 * - 1.6.1. Configure ep0 registers:
		 *  - configure D_EP0Control direction IN, clear ForceNAK.
		 */
		rcD_RegClear8(usbc_dev, ep->reg.ep0.EP0Control, S1R72_EP0INxOUT_IN);
		S1R72_EP0FIFO_CLR(usbc_dev);
		clear_ep0_out_force_nak(usbc_dev, ep);

		/**
		 * - 1.6.2. Change state to S1R72_GD_EP_SOUT:
		 *  - change state to S1R72_GD_EP_SOUT.
		 */
		ep->ep_state = S1R72_GD_EP_SOUT;
		break;

	/**
	 * - 1.7. flag = S1R72_GD_EP_SIN:
	 *  - change state to S1R72_GD_EP_SIN.
	 */
	case S1R72_GD_EP_SIN:
		DEBUG_MSG("%s, S1R72_GD_EP_SIN\n",__FUNCTION__);

		/**
		 * - 1.7.1. Send status IN packet:
		 *  - send zero length IN packet that means status IN
		 *    in control transfer status stage.
		 */
		S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);
		rcD_RegSet8(usbc_dev, ep->reg.ep0.EP0Control, S1R72_EP0INxOUT_IN);

#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/**
		 * - 1.7.1.1. wait to flush cache:
		 */
		if( check_fifo_cache(usbc_dev) == S1R72_CACHE_REMAIN_NG){
			s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, rcCacheRemain,
				rcD_RegRead8(usbc_dev, rcCacheRemain));
		}

#endif
		clear_ep0_in_force_nak(usbc_dev, ep);
		set_ep_enshortpkt(usbc_dev, ep);
		s1r72xxx_queue_log(S1R72_DEBUG_CHG_EP0,
			rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlIN),
			rcD_RegRead8(usbc_dev, ep->reg.ep0.EP0ControlOUT));

		/**
		 * - 1.7.2. Change state to S1R72_GD_EP_SIN:
		 *  - change state to S1R72_GD_EP_SIN.
		 */
		ep->ep_state = S1R72_GD_EP_SIN;
		break;
	/**
	 * - 1.8. error:
	 *  - error. return -EINVAL.
	 */
	default:
		DEBUG_MSG("%s, default\n",__FUNCTION__);
		return -EINVAL;
		break;
	}

	/**
	 * - 2. Return:
	 *  - return 0;
	 */
	return 0;

}

/* ========================================================================= */
/**
 * @brief	- change EPx state and set hardware.
 * @par		usage:
 *				internal use only.
 * @param	*ep;	endpoint informations structure.
 * @param	flag;	flag to change state.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static int change_epx_state(S1R72XXX_USBC_EP *ep, unsigned char flag)
{
	S1R72XXX_USBC_DEV	*usbc_dev;		/* USB hardware informations */
	S1R72XXX_USBC_REQ	*req;			/* USB request */
	unsigned char		reg_value;		/* register value */

	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	usbc_dev = the_controller;
	reg_value = S1R72_REG_ALL_CLR;
	s1r72xxx_queue_log(S1R72_DEBUG_CHG_EPX, ep->ep_subname, flag);

	/**
	 * - 1. Check flag:
	 */
	switch(flag){

	/**
	 * - 1.1. flag = S1R72_GD_EP_INIT:
	 *  - stop epx and change state to S1R72_GD_EP_INIT.
	 */
	case S1R72_GD_EP_INIT:

		/**
		 * - 1.1.1. Disable EP interrupt:
		 *  - disables interrupt registers on the usb device.
		 *    D_EPrIntEnb, D_EPxIntEnb.
		 */
		rcD_RegClear8(usbc_dev, rcD_S1R72_EPrIntEnb,
			S1R72_EPINT_ENB(ep->ep_subname));
		rcD_IntClr8(usbc_dev, ep->reg.epx.EPxIntStat, S1R72_EP_ALL_INT);
		rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxIntEnb, S1R72_REG_ALL_CLR);

		/**
		 * - 1.1.2. Stop endpointx:
		 *  - D_EPrFIFO_Clr, D_EPxConfig0, D_EPxControl, D_EPxJoin
		 */
		S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);
		rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxConfig_0, S1R72_REG_ALL_CLR);
		rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxControl, S1R72_REG_ALL_CLR);
		rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxJoin, S1R72_REG_ALL_CLR);

		/**
		 * - 1.1.3. Dequeue the remaining data:
		 *  - delete all queues from list.
		 */
		while ( list_empty(&ep->queue) == S1R72_LIST_ISNOT_EMPTY ) {
			req = list_entry(ep->queue.next, S1R72XXX_USBC_REQ, queue);
			request_done(ep, req, -ESHUTDOWN);
		}

		/**
		 * - 1.1.4. Clear endpoint FIFO:
		 *  - D_EPrFIFO_Clr
		 */
		S1R72_EPFIFO_CLR(usbc_dev, ep->ep_subname);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/**
		 * - 1.1.4.1. clear FIFO addup:
		 */
		ep->fifo_addup = FIFO_ADDUP_CLR;
#endif
		/**
		 * - 1.1.5. Change state initialize:
		 *  - change a state to S1R72_GD_EP_INIT.
		 */
		ep->ep_state = S1R72_GD_EP_INIT;
		break;

	/**
	 * - 1.2. flag = S1R72_GD_EP_IDLE:
	 *  - configure epx and change state to S1R72_GD_EP_IDLE.
	 */
	case S1R72_GD_EP_IDLE:

		/**
		 * - 1.2.1. Configure epx registers:
		 *  - enable epx. D_EPxMaxSize_H/L, D_EPxConfig_0, D_EPxControl,
		 *    D_EPxJoin
		 */
		if (ep->ep_state == S1R72_GD_EP_INIT){
			rsD_RegWrite16(usbc_dev, ep->reg.epx.EPxMaxSize,
				ep->wMaxPacketSize);
			reg_value = ep->ep_desc->bEndpointAddress;
			if ( ((ep->ep_desc->bEndpointAddress) &USB_ENDPOINT_DIR_MASK )
				== USB_DIR_IN){
				/* IN endpoint */
			} else {
				/* OUT endpoint */
				rcD_IntClr8(usbc_dev, ep->reg.epx.EPxIntStat,
					S1R72_OUT_ShortACK | S1R72_OUT_TranACK );
				rcD_RegSet8(usbc_dev, ep->reg.epx.EPxIntEnb,
					S1R72_EnOUT_ShortACK | S1R72_EnOUT_TranACK );
			}
#if defined(CONFIG_USB_S1R72V05_GADGET) \
	|| defined(CONFIG_USB_S1R72V05_GADGET_MODULE)
			reg_value |= S1R72_EnEndpoint;
#endif
			rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxConfig_0, reg_value);
			rcD_RegWrite8(usbc_dev, ep->reg.epx.EPxControl, S1R72_REG_ALL_CLR);
		}

		if ( ((ep->ep_desc->bEndpointAddress) &USB_ENDPOINT_DIR_MASK )
			== USB_DIR_OUT){
			clear_epx_force_nak(usbc_dev, ep);
			s1r72xxx_queue_log(S1R72_DEBUG_REGISTER, ep->reg.epx.EPxControl,
					rcD_RegRead8(usbc_dev, ep->reg.epx.EPxControl));
		}

		/**
		 * - 1.2.2. Change state to IDLE:
		 *  - change state to S1R72_GD_EP_IDLE.
		 */
		ep->is_stopped = S1R72_EP_ACTIVE;
		ep->ep_state = S1R72_GD_EP_IDLE;
		break;

	/**
	 * - 1.3. flag = S1R72_GD_EP_DOUT:
	 *  - change state to S1R72_GD_EP_DOUT.
	 */
	case S1R72_GD_EP_DOUT:
		/**
		 * - 1.3.1. Change state to DOUT:
		 *  - change state to S1R72_GD_EP_DOUT.
		 */
		ep->ep_state = S1R72_GD_EP_DOUT;
		break;

	/**
	 * - 1.4. flag = S1R72_GD_EP_DIN:
	 *  - change state to S1R72_GD_EP_DIN.
	 */
	case S1R72_GD_EP_DIN:
		/**
		 * - 1.4.1. Change state to DIN:
		 *  - change state to S1R72_GD_EP_DIN.
		 */
		ep->ep_state = S1R72_GD_EP_DIN;
		break;

	/**
	 * - 1.5. error:
	 *  - error. return -EINVAL.
	 */
	default:
		return -EINVAL;
		break;
	}

	/**
	 * - 2. Return:
	 *  - return 0;
	 */
	DEBUG_MSG("%s, exit\n", __FUNCTION__);
	return 0;
}

/* ========================================================================= */
/**
 * @brief	- complete request and report to gadget driver.
 * @par		usage:
 *				internal use only.
 * @param	*ep;	endpoint informations structure.
 * @param	req;	USB device request structure.
 * @param	status;	request complete status.
 * @retval	return 0 operation completed successfully.
 *			in case of error, return error code.
 */
/* ========================================================================= */
static void request_done(S1R72XXX_USBC_EP *ep, S1R72XXX_USBC_REQ *req,
	int status)
{
	DEBUG_MSG("%s, enter\n", __FUNCTION__);
	/**
	 * - 1. Delete request from list:
	 *  - delete a request from list using list_del_init.
	 */
	list_del_init(&req->queue);
	DEBUG_MSG("%s, list_del_init\n", __FUNCTION__);

	/**
	 * - 2. Set status to request structure:
	 *  - to report status to gadget driver, set 'status' to request structure.
	 */
	req->req.status = status;
	DEBUG_MSG("%s, status\n", __FUNCTION__);

	/**
	 * - 3. Report request completed to gadget driver:
	 *  - by calling complete(), report request completion to gadget driver.
	 */
	s1r72xxx_queue_log(S1R72_DEBUG_COMP, ep->ep_subname, req->req.actual);
	s1r72xxx_queue_log(S1R72_DEBUG_COMP_SN, ep->ep_subname, req->queue_seq_no);
	req->req.complete(&ep->ep, &req->req);
	DEBUG_MSG("%s, complete\n", __FUNCTION__);

}

/* ========================================================================= */
/**
 * @brief	- start handshake.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @retval	_RET_OK / -EAGAIN
 */
/* ========================================================================= */
static int usbc_start_handshake(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	timeout_ct;			/* time out counter */
	timeout_ct = 0;

	/**
	 * - 1. Set register to start handshake:
	 *  - initialize member of endpoint structures depends on hardware.
	 *  - ep_subname, ep_state, queue, reg.ep0.xxx or reg.epx.xxx
	 */
	if ( usbc_dev->usbcd_state == S1R72_GD_BOUND ) {
		DEBUG_MSG("%s, state = BOUND\n", __FUNCTION__);
		if ((rcD_RegRead8(usbc_dev, rcPM_Control_1) & PM_State_Mask)
			== S1R72_PM_State_ACT_DEVICE) {
			/**
			 * - 2. Set termination settings:
			 *  - set termination registers.
			 */
			rcD_RegWrite8(usbc_dev, rcD_USB_Status, S1R72_Status_FS);
			rcD_RegWrite8(usbc_dev, rcD_S1R72_NegoControl,
				(S1R72_ActiveUSB | S1R72_DisBusDetect));
			rcD_RegWrite8(usbc_dev, rcD_S1R72_XcvrControl,
				 (S1R72_TermSelect | S1R72_XcvrSelect
				| S1R72_OpMode_DisableBitStuffing) );

			/**
			 * - 3. Check bus state:
			 *  - if termination register is set correctly,
			 *    bus will be J state.
			 */
			while( timeout_ct < S1R72_WAIT_J_TIMEOUT_CT ) {
				if ((rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_LineState)
					!= S1R72_LineState_J){
					return -EAGAIN;
				}
				timeout_ct ++;
			}

			/**
			 * - 4. Set auto negotiation:
			 */
			rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl, S1R72_DisBusDetect);
			rcD_RegSet8(usbc_dev, rcD_S1R72_NegoControl, S1R72_EnAutoNego);
		}
	}
	return S1R72_RET_OK;
}

/* ========================================================================= */
/**
 * @brief	- flush all request.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	status;		status returened to gadget driver.
 * @retval	none.
 */
/* ========================================================================= */
static void all_request_done(S1R72XXX_USBC_DEV *usbc_dev, int status)
{
	S1R72XXX_USBC_REQ	*req;			/* USB request */
	unsigned char	ep_counter;		/* endpoint counter */

	/**
	 * - 1. clear all request:
	 */
	for (ep_counter = 0; ep_counter < S1R72_MAX_ENDPOINT ; ep_counter++){
		DEBUG_MSG("%s, ep_counter = %d\n", __FUNCTION__, ep_counter);
		while ( list_empty(&usbc_dev->usbc_ep[ep_counter].queue)
			== S1R72_LIST_ISNOT_EMPTY ) {
			req = list_entry(usbc_dev->usbc_ep[ep_counter].queue.next,
				S1R72XXX_USBC_REQ, queue);
			request_done(&usbc_dev->usbc_ep[ep_counter], req, status);
			DEBUG_MSG("%s, dequeued\n", __FUNCTION__);
		}
	}
}

/* ========================================================================= */
/**
 * @brief	- change driver state.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	state;	current driver state.
 * @param	event;	event.
 * @retval	state info;
 */
/* ========================================================================= */
unsigned char change_driver_state(S1R72XXX_USBC_DEV *usbc_dev,
	unsigned int state, unsigned int event)
{
	unsigned char	next_driver_state;
	unsigned char	next_power_state;
	unsigned char	ret_val;
	unsigned int	current_pmstate;	/* MainIntStat */

	ret_val = state;

	DEBUG_MSG("%s, current=%d\n", __FUNCTION__,state);
	DEBUG_MSG("%s, event=%d\n", __FUNCTION__,event);

	/**
	 * - 1. Get next state:
	 *  - get next driver state and hardware state.
	 */
	next_driver_state
		= s1r72xxx_drv_state_tbl[state][event].next_driver_state;
	next_power_state
		= s1r72xxx_drv_state_tbl[state][event].next_power_state;

	s1r72xxx_queue_log(S1R72_DEBUG_CHG_DRV, state,	event);
	s1r72xxx_queue_log(S1R72_DEBUG_CHG_DRV, next_driver_state,
		next_power_state);

	/**
	 * - 2. Change state:
	 *  - it needs changing state, set ret_val and PM_Control register.
	 */
	/* state check */
	if (next_driver_state != S1R72_GD_DONT_CHG){
		ret_val = next_driver_state;
		DEBUG_MSG("%s, next_s=%d\n", __FUNCTION__,next_driver_state);
	}

	/* hardware state check */
	if (next_power_state != S1R72_GD_POWER_DONT_CHG){
		DEBUG_MSG("%s, next_p=%d\n", __FUNCTION__,next_power_state);
#if defined(CONFIG_USB_S1R72V05_GADGET)\
	|| defined(CONFIG_USB_S1R72V05_GADGET_MODULE)
		if (next_power_state == GoSleep){
			if ( (rcD_RegRead8(usbc_dev, rcIDE_Config_1) & ActiveIDE)
					== ActiveIDE ) {
				next_power_state = GoActive60;
			}
		}
#endif
		current_pmstate = rcD_RegRead8(usbc_dev, rcPM_Control_1)
						& PM_State_Mask;

		switch(current_pmstate){
		case S1R72_PM_State_ACT_DEVICE:
			if (next_power_state == GoActive) {
				return ret_val;
			}
			break;
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)
		case S1R72_PM_State_SLEEP_01:
#endif
		case S1R72_PM_State_SLEEP:
			if (next_power_state == GoSleep) {
				return ret_val;
			}
			break;
		default:
			break;
		}
		rcD_IntClr8(usbc_dev, rcMainIntStat, FinishedPM);

		rcD_RegWrite8(usbc_dev, rcPM_Control_0, next_power_state);
	}

	return ret_val;
}

/* ========================================================================= */
/**
 * @brief	- flush all request and endpoint FIFO.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static void stop_activity(S1R72XXX_USBC_DEV *usbc_dev)
{
	unsigned char	ep_counter;	/* endopoint loop counter */

	/**
	 * - 1. Report disconnected to gadget driver:
	 */
	if (usbc_dev->driver == NULL){
		return;
	}
	usbc_dev->driver->disconnect(&usbc_dev->gadget);
	usbc_dev->remote_wakeup_prc	= S1R72_RT_WAKEUP_NONE;

	/**
	 * - 2. Cancel all request:
	 */
	all_request_done(usbc_dev, -ECONNRESET);

	/**
	 * - 3. Clear FIFO on H/W:
	 */
	for (ep_counter = S1R72_GD_EP0 ; ep_counter < S1R72_MAX_ENDPOINT
		; ep_counter++) {
		S1R72_EPFIFO_CLR(usbc_dev,ep_counter);

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
		/* - 3.1. clear FIFO addup:
		 */
		usbc_dev->usbc_ep[ep_counter].fifo_addup = FIFO_ADDUP_CLR ;
#endif
	}
}

/* ========================================================================= */
/**
 * @brief	- send zero packet at ep0.
 * @par		usage:
 *				internal use only.
 * @param	*usbc_dev;	device informations structure.
 * @param	*ep0;		endpoint informations structure.
 * @retval	none.
 */
/* ========================================================================= */
static void send_ep0_short_packet(S1R72XXX_USBC_DEV *usbc_dev,
	S1R72XXX_USBC_EP *ep0)
{
	/**
	 * - 1. Send short packet:
	 *  - set EnShortPkt bit to send short packet.
	 */
	rcD_RegClear8(usbc_dev, ep0->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
	rcD_RegWrite8(usbc_dev, ep0->reg.ep0.EP0ControlIN,
		S1R72_EnShortPkt);
	rcD_RegSet8(usbc_dev, ep0->reg.ep0.SETUP_Control,
		S1R72_ProtectEP0);
}

/* ========================================================================= */
/**
 * @brief	- remote wakeup timer handler.
 * @par		usage:
 *				internal use only.
 * @param	_dev;	device informations structure.
 * @retval	none.
 */
/* ========================================================================= */
void remote_wakeup_timer(unsigned long _dev)
{
	/* USB hardware informations */
	S1R72XXX_USBC_DEV	*usbc_dev = (void*)_dev;
	unsigned long		flags;			/* spin lock flag */

	/**
	 * - 1. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 2. Clear resume signal:
	 *  - clear SendWakeup bit to stop sending resume signal
	 *    and set NonJ interrupt.
	 */
	rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl, S1R72_SendWakeup);
	rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl,S1R72_InSUSPEND);

	/**
	 * - 3. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	DEBUG_MSG("%s, Remote wakeup end\n", __FUNCTION__);
}

/* ========================================================================= */
/**
 * @brief	- chirp cmp timer handler.
 * @par		usage:
 *				internal use only.
 * @param	_dev;	device informations structure.
 * @retval	none.
 */
/* ========================================================================= */
void chirp_cmp_timer(unsigned long _dev)
{
	/* USB hardware informations */
	S1R72XXX_USBC_DEV	*usbc_dev = (void*)_dev;
	unsigned long		flags;			/* spin lock flag */
	unsigned char		timeout_ct;
	unsigned char		j_state_flag;
	timeout_ct = 0;
	j_state_flag = 0;
	DEBUG_MSG("%s, enter\n", __FUNCTION__);

	/**
	 * - 1. Disable IRQ:
	 *  - call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 2. Check bus state:
	 */
	while( timeout_ct < S1R72_WAIT_J_TIMEOUT_CT ) {
		j_state_flag = rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_LineState;
		if (j_state_flag != S1R72_LineState_J){
			break;
		}
		timeout_ct++;
	}

	if( j_state_flag == S1R72_LineState_J){
		DEBUG_MSG("%s, start handshake\n", __FUNCTION__);
		/**
		 * - 2.1. Set auto negotiation:
		 */
		rcD_RegClear8(usbc_dev, rcD_S1R72_NegoControl, S1R72_DisBusDetect);
		rcD_RegSet8(usbc_dev, rcD_S1R72_NegoControl, S1R72_EnAutoNego);

		/* change state */
		usbc_dev->usbcd_state
			= change_driver_state(usbc_dev, usbc_dev->usbcd_state,
				S1R72_GD_H_W_ACT_DEVICE);

		/* change ep0 state to IDLE */
		change_ep0_state(&usbc_dev->usbc_ep[S1R72_GD_EP0],
			S1R72_GD_EP_IDLE);

	} else {
		DEBUG_MSG("%s, restart timer\n", __FUNCTION__);
		/**
		 * - 2.2. Set timer:
		 */
		if ( (rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_VBUS)
			== S1R72_VBUS_H ) {
			mod_timer(&usbc_dev->wait_j_timer,
				S1R72_WAIT_J_TIMEOUT_TM);
		} else {
			usbc_irq_VBUS(usbc_dev);
		}
	}

	/**
	 * - 3. Enable IRQ:
	 *  - call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	DEBUG_MSG("%s, exit\n", __FUNCTION__);
}

/* ========================================================================= */
/**
 * @brief	- VBUS wait timer handler.
 * @par		usage:
 *				internal use only.
 * @param	_dev;	device informations structure.
 * @retval	none.
 */
/* ========================================================================= */
void vbus_timer(unsigned long _dev)
{
	/* USB hardware informations */
	S1R72XXX_USBC_DEV	*usbc_dev = (void*)_dev;
	unsigned long		flags;			/* spin lock flag */

	/**
	 * - 1. Disable IRQ:
	 *	- call spin_lock_irqsave.
	 */
	spin_lock_irqsave(&usbc_dev->lock, flags);

	/**
	 * - 2. Check VBUS state:
	 *	- if VBUS state is high, then start handshake.
	 */
	usbc_dev->vbus_timer_state = S1R72_VBUS_TIMER_STOPPED;
	if ( (rcD_RegRead8(usbc_dev, rcD_USB_Status) & S1R72_VBUS)
		== S1R72_VBUS_H){
		usbc_irq_VBUS(usbc_dev);
	}

	/**
	 * - 3. Enable IRQ:
	 *	- call spin_unlock_irqrestore.
	 */
	spin_unlock_irqrestore(&usbc_dev->lock, flags);

	DEBUG_MSG("%s, VBUS timer end\n", __FUNCTION__);
}

#ifdef CONFIG_EBOOK5_LED

static unsigned long usbtg_led_next_time(unsigned long now, unsigned long time)
{
	unsigned long ret = 0;
	unsigned long differ = 0;

	differ = (0xffffffff - now);

	if( differ < time ) {
		ret = (time - differ);
	} else {
		ret = (now + time);
	}
	return ret;
}


static void usbtg_led(int flag)
{
	unsigned long now = 0;

	switch(flag) {
	case EBOOK5_LED_OFF:
		next_led = EBOOK5_LED_OFF;
		__imx_gpio_set_value((GPIO_PORTB|8), next_led);
		break;

	case EBOOK5_LED_ON:
		next_led = EBOOK5_LED_ON;
		__imx_gpio_set_value((GPIO_PORTB|8), next_led);
		break;

	case EBOOK5_LED_TOGGLE:
		now = jiffies;
		if (ebook5_led) {
			if (next_led_time <= now) {
				next_led = (next_led ? EBOOK5_LED_OFF : EBOOK5_LED_ON);
				__imx_gpio_set_value((GPIO_PORTB|8), next_led);
				next_led_time = usbtg_led_next_time(now, (next_led ? 2 : 1));
			}
		} else {
			if ((next_led_time <= now) && (next_led)) {
				next_led = EBOOK5_LED_OFF;
				__imx_gpio_set_value((GPIO_PORTB|8), next_led);
				next_led_time = 0;
			}
		}
		break;
	}
}
#endif /* CONFIG_EBOOK5_LED */
