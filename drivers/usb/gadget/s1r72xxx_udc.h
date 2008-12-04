/**
 * @file	s1r72xxx_udc.h
 * @brief	S1R72XXX USB Contoroller Driver(USB Device)
 * @date	2008/02/01
 * Copyright (C)SEIKO EPSON CORPORATION 2003-2008. All rights reserved.
 * This file is licenced under the GPLv2
 */
#ifndef S1R72XXX_USBC_H	/* include guard */
#define S1R72XXX_USBC_H
//#define CONFIG_USB_S1R72V17_GADGET_16BIT_FIX
//#define CONFIG_USB_S1R72V05_GADGET_16BIT_FIX
//#define CONFIG_USB_S1R72V27_GADGET_16BIT_FIX
//#define DEBUG_PRINTK	/** enable console log */
#define DEBUG_PROC	/** enable proc file reg dump >cat /proc/driver/udc */
#define DEBUG_PROC_LOG	/** enable proc file log. >cat /proc/driver/udc */
//#define DEBUG_REMOTE_WAKEUP	/** enable remote wakeup test use proc file */
//#define DEBUG_SUSPEND_RESUME	/** enable suspend/resume test use proc file */
//#define S1R72_MASS_STORAGE_ONLY
//#define PTU

/******************************************
 * macro for debug
 ******************************************/
/**
 * @name	DEBUG_MSG
 * @brief	debug macro.
*/
#ifdef DEBUG_PRINTK
#define DEBUG_MSG(args...) \
		printk(args)
#else /* DEBUG_PRINTK */
#define DEBUG_MSG(args...)
#endif /* DEBUG_PRINTK */
/*@}*/

struct	tag_S1R72XXX_USBC_DEV;

/******************************************
 * definitions of "define"
 ******************************************/
/**
 * @name S1R72_XXX
 * @brief	definitions of constant value.
*/
#define S1R72_EP_NAME_LEN	12			/** ep name max length */
#define S1R72_RET_OK		0			/** return OK */
#define S1R72_LIST_ISNOT_EMPTY	0		/** list_empty returns is not empty */

#define S1R72_HS	0							/** connected to HS bus */
#define S1R72_FS	1							/** connected to FS bus */
#define S1R72_SUPPORT_BUS_TYPE	S1R72_FS + 1	/** number of bus type  */
#define S1R72_BIG_ENDIAN	0			/** big endian */
#define S1R72_LITTLE_ENDIAN	1			/** little endian */

#define S1R72_ADD_DUMMY(x)	((x) << 8)

#define S1R72_DONT_SEND_ZERO	0	/** gadget does not request zero packet */
#define S1R72_SEND_ZERO		1		/** gadget requests zero packet */

/* set halt API parameter */
#define S1R72_CLR_STALL		0			/** gadget requests to clear stall */
#define S1R72_SET_STALL		1			/** gadget requests to stall */

#define S1R72_REG_ALL_CLR		0x00			/** zero clear register */

/** number of bus transfer type  */
#define S1R72_SUPPORT_TRN_TYPE	USB_ENDPOINT_XFER_INT + 1

/* endpoint FIFO */
#define S1R72_FIFO_EMPTY		0x00		/** fifo empty */
#define S1R72_FIFO_READ_COMP	0x00		/** fifo read completed */
#define S1R72_FIFO_READ_RETRY	0x01		/** needs retry fifo read */
#define S1R72_RD_REMAIN_TIMEOUT	0x0A		/** RdRemain loop counter */
#define S1R72_RD_REMAIN_OK		0x00		/** RdRemain loop ok */
#define S1R72_RD_REMAIN_NG		0xFF		/** RdRemain loop timeout */
#if defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
#define S1R72_CACHE_REMAIN_TIMEOUT	0x100	/** CacheRemain loop counter */
#define S1R72_CACHE_REMAIN_OK		0x00	/** CacheRemain loop ok */
#define S1R72_CACHE_REMAIN_NG		0xFF	/** CacheRemain loop timeout */
#endif
#define S1R72_FORCENAK_TIMEOUT		0xFFFF	/** ForceNAK change wait counter */
#define S1R72_ENSHORTPKT_TIMEOUT	0xFFFF	/** EnShortPkt change wait counter */

/* queue control */
#define S1R72_QUEUE_DONE		0x00	/** queue is done */
#define S1R72_QUEUE_REMAIN		0x01	/** queue remain */
#define S1R72_CALLED_IRQ		0x00	/** handle_ep called by irq handler */
#define S1R72_CALLED_EP_QUEUE	0x01	/** handle_ep called by ep_queue */

/* power management control */
#define S1R72_PM_CHANGE_TO_SLEEP	0x01	/** pm state changed to sleep */
#define S1R72_PM_CHANGE_TIMEOUT		0xBBCC	/** pm state change timeout counter */

/* irq */
#define	S1R72_SIE_INT_MAX		0x07			/** max sie interrupt source */
#define S1R72_DEV_IRQ_NONE		0x00			/** device IRQ list flag */
#define S1R72_IRQ_NONE			0x00			/** none IRQs occured */
#define S1R72_IRQ_DONE			0x01			/** IRQs are cleared */
#define S1R72_IRQ_NOT_OCCURED	0x00			/** IRQ has not occured */
#define S1R72_IRQ_OCCURED		0x01			/** IRQ has occured */
#define S1R72_IRQ_SHORT			0x00			/** out packet is short */
#define S1R72_IRQ_IS_NOT_SHORT	0x01			/** out packet is not short */

/* endpoint is stopped or not */
#define S1R72_EP_ACTIVE			0x00			/** endpoint is not stalled */
#define S1R72_EP_STALL			0x01			/** endpoint is stalled */

/* remote wakeup */
#define S1R72_RT_WAKEUP_NONE	0x00			/** remote wakeup */
#define S1R72_RT_WAKEUP_PROC	0x01			/** remote wakeup */
#define S1R72_RT_WAKEUP_DIS		0x00			/** remote wakeup disable */
#define S1R72_RT_WAKEUP_ENB		0x02			/** remote wakeup enable */
/** wakeup signal time 10ms */
#define S1R72_SEND_WAKEUP_TIME	(jiffies + msecs_to_jiffies(1))

/* data is even or odd */
#define S1R72_ODD_MASK			0xFE			/** address odd check mask */
#define S1R72_ADDR_ODD			0x01			/** address odd check */
#define S1R72_DATA_EVEN			0x00			/** data is even */
#define S1R72_DATA_ODD			0x01			/** data is odd */

#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
#define FIFO_ADDUP_CLR	0
#define FIFO_ADDUP_EVEN	0
#define	FIFO_ADDUP_ODD	1
#define FIFO_ADDUP_MASK	0x01
#endif

#define S1R72_WAIT_J_TIMEOUT_CT		0x0D		/** time out is 1msec */

/** wait pull up time */
#define S1R72_WAIT_J_TIMEOUT_TM		(jiffies + msecs_to_jiffies(1))
#define S1R72_WAIT_J_TIMEOUT_TM_MAX	0x0B		/** time out is 100msec */

#define S1R72_WAIT_VBUS_TIMEOUT_TM	(jiffies + msecs_to_jiffies(60))
#define S1R72_VBUS_TIMER_RUNNING	0			/** VBUS timer state */
#define S1R72_VBUS_TIMER_STOPPED	1
#define S1R72_WAIT_NONJ_TIMEOUT_TM_MAX	0x0A	/** check bus state */

#define S1R72_VBUS_H			S1R72_VBUS		/** VBUS = High */

/** USB spec */
#define S1R72_USB_MAX_PACKET	0x07FF	/** wMaxPacketSize */
#define S1R72_USB_MAX_TRANSC	0x1800	/** max transactions per one frame */
#define S1R72_USB_MIN_PKT		4		/** min packet size of usb */
#define S1R72_HS_CNT_MAX_PKT	64		/** max packet size of control */
#define S1R72_HS_BLK_MAX_PKT	512		/** max packet size of bulk */
#define S1R72_HS_INT_MAX_PKT	1024	/** max packet size of interrupt */
#define S1R72_HS_ISO_MAX_PKT	1024	/** max packet size of isochronous */
#define S1R72_FS_CNT_MAX_PKT	64		/** max packet size of control */
#define S1R72_FS_BLK_MAX_PKT	64		/** max packet size of bulk */
#define S1R72_FS_INT_MAX_PKT	64		/** max packet size of interrupt */
#define S1R72_FS_ISO_MAX_PKT	1023	/** max packet size of isochronous */

#define TEST_MODE_J					0x0001	/** TEST_MODE Test_J */
#define TEST_MODE_K					0x0002	/** TEST_MODE Test_K */
#define TEST_MODE_SE0_NAK			0x0003	/** TEST_MODE Test_SE0_NAK */
#define TEST_MODE_PACKET			0x0004	/** TEST_MODE Test_Packet */
#define TEST_MODE_FORCE_ENABLE		0x0005	/** TEST_MODE Test_Force_Enable */
#define TEST_MODE_PACKET_SIZE		53		/** Test mode packet size */
#define TEST_MODE_EP_NUM			0x0F	/** Test mode endpoint number */
#define TEST_MODE_EP_PACKET_SIZE	64	/** Test mode endpoint PacketSize */

/** I/O register access */
#define rcD_REGS8(x, addr)  \
	*((volatile unsigned char *)(((S1R72XXX_USBC_DEV *)x)->base_addr + addr))
#define rsD_REGS16(x, addr) \
	*((volatile unsigned short *)(((S1R72XXX_USBC_DEV *)x)->base_addr + addr))

#ifdef CONFIG_USB_S1R72V05_GADGET_16BIT_FIX
	#define S1R72XXX_GADGET_16BIT_FIX
#endif

#ifdef CONFIG_USB_S1R72V17_GADGET_16BIT_FIX
	#define S1R72XXX_GADGET_16BIT_FIX
#endif

#ifdef CONFIG_USB_S1R72V18_GADGET_16BIT_FIX
	#define S1R72XXX_GADGET_16BIT_FIX
#endif

#ifdef CONFIG_USB_S1R72V27_GADGET_16BIT_FIX
	#define S1R72XXX_GADGET_16BIT_FIX
#endif

#ifdef S1R72XXX_GADGET_16BIT_FIX
	#define S1R72_UP_MASK			0xFF00		/** upper data mask */
	#define S1R72_LW_MASK			0x00FF		/** lower data mask */
	#define S1R72_EP0FIFO_INC		0x02		/** FIFO counter increment */
#else
	#define S1R72_UP_MASK			0xFF		/** upper data mask */
	#define S1R72_LW_MASK			0xFF		/** lower data mask */
	#define S1R72_EP0FIFO_INC		0x01		/** FIFO counter increment */
#endif

#ifdef S1R72XXX_GADGET_16BIT_FIX
	#define rcD_IntClr8(x, addr, data)	\
		( rsD_REGS16(x, ((addr) & 0xFFFE))\
			= ((addr) & 0x01) \
				? ((data) << 8) \
				: (data) )

	#define rcD_RegWrite8(x, addr, data)	\
		 ( rsD_REGS16(x, ((addr) & 0xFFFE))\
			= ((addr) & 0x01) \
				? ((rsD_REGS16(x, ((addr) & 0xFFFE)) & S1R72_LW_MASK)\
						| ((data) << 8)) \
				: ((rsD_REGS16(x, addr) & S1R72_UP_MASK) | (data)) )

	#define rsD_RegFWrite16(x, data)	\
		*((volatile unsigned short*)(x)) = (data)

	#define rsD_RegWrite16(x, addr, data)	\
		rsD_REGS16(x, (addr)) = (data)

	#define rcD_RegRead8(x, addr)	\
			( (((addr) & 0x01) \
				? (unsigned char)((rsD_REGS16(x, ((addr) & 0xFFFE))\
					& S1R72_UP_MASK) >> 8) \
				: (unsigned char)(rsD_REGS16(x, (addr)) & S1R72_LW_MASK) ) )

	#define rsD_RegRead16(x, addr)	\
			( (unsigned short)rsD_REGS16(x, addr) )

	#define rsD_RegFRead16(x)	\
			( *((volatile unsigned short*)(x)) )

	#define rcD_RegSet8(x, addr, data)	\
		( rsD_REGS16(x, ((addr) & 0xFFFE)) \
			= ((addr) & 0x01) \
				? (rsD_REGS16(x, ((addr) & 0xFFFE)) | ((data) << 8) ) \
				: ((rsD_REGS16(x, (addr)) | (data))) )

	#define rcD_RegClear8(x, addr, data)	\
		( rsD_REGS16(x, ((addr) & 0xFFFE))\
			= ((addr) & 0x01) \
				? ( rsD_REGS16(x, ((addr) & 0xFFFE)) & (~((data) << 8)) )  \
				: ( rsD_REGS16(x,  (addr)) & (~(data)) ) )

	#define INT_BIT_SET(x, y)	( ((x) & 0x01) ? ((y) << 8) : (y) )
#else
	#define rcD_IntClr8(x, addr, data)	\
		  ( rcD_REGS8(x, (addr) ) =  (data) )

	#define rsD_RegFWrite16(x, data)	\
		*((volatile unsigned short*)(x)) = (data)

	#define rcD_RegWrite8(x, addr, data)	\
		  ( rcD_REGS8(x, (addr) ) =  (data) )

	#define rsD_RegWrite16(x, addr, data)	\
		(rsD_REGS16(x, (addr)) = (data) )

	#define rcD_RegRead8(x, addr)	\
		(rcD_REGS8(x, (addr)) )

	#define rsD_RegRead16(x, addr)	\
		( rsD_REGS16(x, addr) )

	#define rsD_RegFRead16(x)	\
		( *((volatile unsigned short*)(x)) )

	#define rcD_RegSet8(x, addr, data)	\
		( rcD_REGS8(x, (addr)) |= (data))

	#define rcD_RegClear8(x, addr, data)	\
		( rcD_REGS8(x, (addr)) &= (~(data) ) )  \

	#define INT_BIT_SET(x, y)	(y)
#endif

/** self powered macro for GET_STATUS */
#define S1R72_SELF_POW	\
	(1 << USB_DEVICE_SELF_POWERED)
/*@}*/

/*****************************************
 * definitions of "enum"
 *****************************************/
/**
 * @name S1R72_DEV_REQ
 * @brief	device request.
 */
 enum	S1R72_DEV_REQ
{
	S1R72_REQ_TYPE	= 0,
	S1R72_REQ,
	S1R72_VALUE_L,
	S1R72_VALUE_H,
	S1R72_INDEX_L,
	S1R72_INDEX_H,
	S1R72_LENGTH_L,
	S1R72_LENGTH_H,
	S1R72_DEV_REQ_MAX
};

/**
 * @name S1R72_GD_DRV_STATE
 * @brief	driver states.
 */
enum	S1R72_GD_DRV_STATE
{
	S1R72_GD_INIT		= 0,
	S1R72_GD_REGD,
	S1R72_GD_BOUND,
	S1R72_GD_ATTACH,
	S1R72_GD_RUN,
	S1R72_GD_USBSUSPD,
	S1R72_GD_SYSSUSPD,
	S1R72_GD_MAX_DRV_STATE,
	S1R72_GD_DONT_CHG
};

/**
 * @name S1R72_GD_EP_STATE
 * @brief	endpoint states.
 */
enum	S1R72_GD_EP_STATE
{
	S1R72_GD_EP_INIT		= 0,
	S1R72_GD_EP_IDLE,
	S1R72_GD_EP_DOUT,
	S1R72_GD_EP_DIN,
	S1R72_GD_EP_DIN_CHG,
	S1R72_GD_EP_SOUT,
	S1R72_GD_EP_SIN,
	S1R72_GD_MAX_EP_STATE,
	S1R72_GD_EP_DONT_CHG
};

/**
 * @name S1R72_GD_POWER_STATE
 * @brief	device power states.
 */
enum	S1R72_GD_POWER_STATE
{
	S1R72_GD_GOSNOOZE		= 0,
	S1R72_GD_GOSLEEP,
	S1R72_GD_GOACTIVE60,
	S1R72_GD_GOACTDEVICE,
	S1R72_GD_MAX_POW_STATE,
	S1R72_GD_POWER_DONT_CHG
};

/**
 * @name S1R72_GD_DRIVER_EVENT
 * @brief	driver events.
 */
enum	S1R72_GD_DRIVER_EVENT
{
	S1R72_GD_API_PROBE	= 0,
	S1R72_GD_API_REGISTER,
	S1R72_GD_API_UNREGISTER,
	S1R72_GD_API_REMOVE,
	S1R72_GD_API_SUSPEND,
	S1R72_GD_API_RESUME,
	S1R72_GD_API_SHUTDOWN,
	S1R72_GD_H_W_NONJ,
	S1R72_GD_H_W_RESET,
	S1R72_GD_H_W_RESUME,
	S1R72_GD_H_W_SUSPEND,
	S1R72_GD_H_W_CHIRP,
	S1R72_GD_H_W_ADDRESS,
	S1R72_GD_H_W_RESTORE,
	S1R72_GD_H_W_VBUS_H,
	S1R72_GD_H_W_VBUS_L,
	S1R72_GD_H_W_SLEEP,
	S1R72_GD_H_W_SNOOZE,
	S1R72_GD_H_W_ACTIVE60,
	S1R72_GD_H_W_ACT_DEVICE,
	S1R72_GD_ERR_CHIRP_FAIL,
	S1R72_GD_MAX_DRV_EVENT
};

/**
 * @name S1R72_GD_EP_EVENT
 * @brief	endpoint events.
 */
enum	S1R72_GD_EP_EVENT
{
	S1R72_GD_API_EP_ENABLE		= 0,
	S1R72_GD_API_EP_DISABLE,
	S1R72_GD_API_EP_QUEUE_OUT,
	S1R72_GD_API_EP_QUEUE_IN,
	S1R72_GD_API_EP_DEQUEUE,
	S1R72_GD_API_FIFO_FLUSH,
	S1R72_GD_API_FIFO_STATUS,
	S1R72_GD_API_SET_HALT_DIS,
	S1R72_GD_API_SET_HALT_ENB,
	S1R72_GD_H_W_PM_SLEEP,
	S1R72_GD_H_W_PM_SNOOZE,
	S1R72_GD_H_W_PM_ACT_DEVICE,
	S1R72_GD_H_W_RCV_SETUP_OUT,
	S1R72_GD_H_W_RCV_SETUP_IN,
	S1R72_GD_H_W_EP0INT_OUT_ACK,
	S1R72_GD_H_W_EP0INT_IN_ACK,
	S1R72_GD_MAX_EP_EVENT
};

/**
 * @name S1R72_DEBUG_ITEM
 * @brief	debug items.
 */
enum S1R72_DEBUG_ITEM {
	S1R72_DEBUG_EP_ENABLE	= 0,
	S1R72_DEBUG_EP_DISABLE,
	S1R72_DEBUG_ALLOC_REQ,
	S1R72_DEBUG_FREE_REQ,
	S1R72_DEBUG_ALLOC_BUF,
	S1R72_DEBUG_FREE_BUF,
	S1R72_DEBUG_EP_QUEUE,
	S1R72_DEBUG_EP_QUEUE_SN,
	S1R72_DEBUG_EP_DEQUEUE,
	S1R72_DEBUG_EP_DEQUEUE_SN,
	S1R72_DEBUG_SET_HALT,
	S1R72_DEBUG_PROBE,
	S1R72_DEBUG_REMOVE,		/* 10 */
	S1R72_DEBUG_SHUTDOWN,
	S1R72_DEBUG_REG_DRV,
	S1R72_DEBUG_UNREG_DRV,
	S1R72_DEBUG_IRQ_S,
	S1R72_DEBUG_IRQ_E,
	S1R72_DEBUG_IRQ_DEVINT,
	S1R72_DEBUG_IRQ_POWINT,
	S1R72_DEBUG_IRQ_VBUS,
	S1R72_DEBUG_IRQ_SIE,
	S1R72_DEBUG_IRQ_RCVEP0,
	S1R72_DEBUG_IRQ_EP0,
	S1R72_DEBUG_IRQ_EP,		/* 20 */
	S1R72_DEBUG_IRQ_FPM,
	S1R72_DEBUG_CHG_DRV,
	S1R72_DEBUG_CHG_EP0,
	S1R72_DEBUG_CHG_EPX,
	S1R72_DEBUG_HANDLE_EP,
	S1R72_DEBUG_HANDLE_EP_SN,
	S1R72_DEBUG_COMP,
	S1R72_DEBUG_COMP_SN,
	S1R72_DEBUG_CHG_DRV_STS,
	S1R72_DEBUG_REGISTER,
	S1R72_DEBUG_SHORT,
	S1R72_DEBUG_ITEM_MAX
};

#if	defined(CONFIG_USB_S1R72V05_GADGET_16BIT_FIX)
enum	S1R72_FORCENAK_STATE {
	 S1R72_FORCENAK_RETRY_OFF = 0
	,S1R72_FORCENAK_RETRY_ON
};
#endif


/******************************************
 * definitions of "struct"
 ******************************************/
/**
 * @struct tag_S1R72XXX_USBC_EP0_REGS
 * @brief Structure of the S1R72xxx endpoint 0 informations
 */
typedef struct tag_S1R72XXX_USBC_EP0_REGS {
	u32	EP0IntStat;		/** EP0IntStat */
	u32	EP0IntEnb;		/** EP0IntEnb */
	u32	EPnControl;		/** EPnControl */
	u32	EP0SETUP_0;		/** EP0SETUP_0 */
	u32	SETUP_Control;	/** SETUP_Control */
	u32	EP0MaxSize;		/** EP0MaxSize */
	u32	EP0Control;		/** EP0Control */
	u32	EP0ControlIN;	/** EP0ControlIN */
	u32	EP0ControlOUT;	/** EP0ControlOUT */
	u32	EP0Join;		/** EP0Join */
	u32	EP0StartAdrs;	/** EPxStartAdrs */
	u32	EP0EndAdrs;		/** EPxEndAdrs */
} S1R72XXX_USBC_EP0_REGS;

/**
 * @struct tag_S1R72XXX_USBC_EPX_REGS
 * @brief Structure of the S1R72xxx endpoint X informations
 */
typedef struct tag_S1R72XXX_USBC_EPX_REGS {
	u32	EPxIntStat;		/** EPxIntStat */
	u32	EPxIntEnb;		/** EPxIntEnb */
	u32	EPxFIFOClr;		/** EPxFIFOClr */
	u32	EPxMaxSize;		/** EPxMaxSize */
	u32	EPxConfig_0;	/** EPxConfig_0 */
	u32	EPxControl;		/** EPxControl */
	u32	EPxJoin;		/** EPxJoin */
	u32	EPxStartAdrs;	/** EPxStartAdrs */
	u32	EPxEndAdrs;		/** EPxEndAdrs */
	u32	dummy1;
	u32	dummy2;
	u32	dummy3;
} S1R72XXX_USBC_EPX_REGS;

/**
 * @union tag_S1R72XXX_EP_REGS
 * @brief	Union of endopoint registers
 *
 */
typedef union tag_S1R72XXX_EP_REGS {
	S1R72XXX_USBC_EP0_REGS ep0;     /** ep0 registers */
	S1R72XXX_USBC_EPX_REGS epx;     /** epx registers */
} S1R72XXX_EP_REGS;

/**
 * @struct tag_S1R72XXX_USBC_EP
 * @brief	Structure of the S1R72xxx endpoint informations
 */
typedef struct tag_S1R72XXX_USBC_EP {
	int ep_subname;				/** Endpoint Name */
	unsigned char ep_state;		/** Endpoint State */
	struct usb_ep ep;				/** USB Endpoint Structure */
	struct tag_S1R72XXX_USBC_DEV *dev;	/** S1R72XXX Information Structure */
	struct list_head queue;		/** Head of queues list */
	/** Endpoint Descriptor */
	const struct usb_endpoint_descriptor *ep_desc;
	u8 bEndpointAddress;			/** Endpoint Address */
	u8 bmAttributes;				/** Attributes */
	u16 wMaxPacketSize;			/** Maximum packet size of this endpoint */
	u8 bInterval;					/** Polling interval */
	unsigned short fifo_size;		/** Hardware specific FIFO size */
	unsigned int fifo_data;		/** recieved data */
#if defined(CONFIG_USB_S1R72V18_GADGET) || defined(CONFIG_USB_S1R72V18_GADGET_MODULE)\
	|| defined(CONFIG_USB_S1R72V27_GADGET) || defined(CONFIG_USB_S1R72V27_GADGET_MODULE)
	unsigned int fifo_addup;		/** FIFO addup */
#endif
	unsigned char intenb;			/** Interrupt occured or not */
	unsigned char last_is_short;	/** short packet flag */
	unsigned char is_stopped;		/** stalled or not  */
	S1R72XXX_EP_REGS reg;			/** endpoint registers */
} S1R72XXX_USBC_EP;

/**
 * @struct tag_S1R72XXX_USBC_DEV
 * @brief	Structure of the S1R72xxx device informations
 */
typedef struct tag_S1R72XXX_USBC_DEV {
	/** General gadget driver informations */
	struct usb_gadget gadget;			/** Gadget informations */
	struct usb_gadget_driver *driver;	/** Gadget driver informations */
	unsigned char usbcd_state;			/** USBCD State */
	unsigned char remote_wakeup_prc;	/** remote wakeup flag */
	unsigned char remote_wakeup_enb;	/** remote wakeup flag */
	unsigned char next_usbcd_state;		/** next driver state */
	unsigned char previous_usbcd_state;	/** previous driver state */
	struct device *dev;					/** device driver information */
	void *base_addr;					/** base address */
	spinlock_t lock;					/** spin lock */
	struct timer_list rm_wakeup_timer;	/** remote wakeup timer */
	struct timer_list wait_j_timer;		/** chirp cmp timer */
	struct timer_list vbus_timer;		/** VBUS timer */
	unsigned char vbus_timer_state;		/** VBUS timer state */
	unsigned char previous_vbus_state;	/** VBUS state */
	/** Definitions of Hardware Specific */
	S1R72XXX_USBC_EP usbc_ep[S1R72_MAX_ENDPOINT];
} S1R72XXX_USBC_DEV;

/**
 * @struct tag_S1R72XXX_USBC_REQ
 * @brief	Structure of the S1R72xxx for usb device request
 */
typedef struct  tag_S1R72XXX_USBC_REQ {
	struct usb_request req;		/** USB request structure */
	struct list_head queue;		/** queue list */
	unsigned char send_zero;		/** send zero  */
	unsigned int queue_seq_no;	/** sequential number(for debug) */
}S1R72XXX_USBC_REQ;

/**
 * @struct tag_S1R72XXX_USBC_IRQ_TBL
 * @brief	Structure of the S1R72xxx irq informations
 */
typedef struct tag_S1R72XXX_USBC_IRQ_TBL {
	unsigned char   irq_bit;			/** irq register value */
	int (*func)(S1R72XXX_USBC_DEV *);	/** irq handler */
} S1R72XXX_USBC_IRQ_TBL;

/**
 * @struct tag_S1R72XXX_USBC_DRV_STATE_TABLE_ELEMENT
 * @brief	Structure of the S1R72xxx driver state matrix table element
 */
typedef struct tag_S1R72XXX_USBC_DRV_STATE_TABLE_ELEMENT{
	unsigned char	next_driver_state;	/** next driver state */
	unsigned char	next_power_state;	/** next power state */
}S1R72XXX_USBC_DRV_STATE_TABLE_ELEMENT;

/**
 * @struct tag_S1R72XXX_USBC_EP_IRQ_TBL
 * @brief	Structure of the S1R72xxx endpoint irq informations
 */
typedef struct tag_S1R72XXX_USBC_EP_IRQ_TBL {
	unsigned char	irq_bit;
	unsigned char	ep_num;
}S1R72XXX_USBC_EP_IRQ_TBL;

/**
 * @struct tag_S1R72XXX_USBC_QUE_DEBUG
 * @brief	Structure of the S1R72xxx debug information
 */
typedef struct tag_S1R72XXX_USBC_QUE_DEBUG {
	unsigned char	func_name;	/** function name */
	unsigned int	data1;		/** data 1 */
	unsigned int	data2;		/** data 2 */
	unsigned short	sof_no;		/** SOF */
}S1R72XXX_USBC_QUE_DEBUG;

/**
 * @struct tag_S1R72XXX_USBC_QUE_DEBUG_TITLE
 * @brief	Structure of the S1R72xxx debug title string
 */
typedef struct tag_S1R72XXX_USBC_QUE_DEBUG_TITLE {
	unsigned char	*action_string;
	unsigned char	*target_string;
	unsigned char	*value_string;
	unsigned char	*time_string;
}S1R72XXX_USBC_QUE_DEBUG_TITLE;

#endif	/* S1R72XXX_USBC_H */
