/**
 * @file	s1r72v17_udc-regs.h
 * @brief	S1R72V17 USB Contoroller Driver(USB Device)
 * @date	2007/03/20
 * Copyright (C)SEIKO EPSON CORPORATION 2003-2006. All rights reserved.
 */
#ifndef S1R72V17_USBC_H		/* include guard */
#define S1R72V17_USBC_H

/*-------------------/
/ USB Device         /
/-------------------*/
/**
 * @name S1R72_EP0_XXX
 * @brief	definitions of FIFO address.
*/
#ifdef S1R72_MASS_STORAGE_ONLY
#define S1R72_EP0_MAX_PKT		64		/** max packet size of endpoint 0 */
#define S1R72_EPA_MAX_PKT		2048	/** max packet size of endpoint A */
#define S1R72_EPB_MAX_PKT		2048	/** max packet size of endpoint B */
#define S1R72_EPC_MAX_PKT		8		/** max packet size of endpoint C */
#define S1R72_EPD_MAX_PKT		8		/** max packet size of endpoint D */
#define S1R72_EPE_MAX_PKT		8		/** max packet size of endpoint E */
#else
#define S1R72_EP0_MAX_PKT		64		/** max packet size of endpoint 0 */
#define S1R72_EPA_MAX_PKT		1024	/** max packet size of endpoint A */
#define S1R72_EPB_MAX_PKT		1024	/** max packet size of endpoint B */
#define S1R72_EPC_MAX_PKT		1024	/** max packet size of endpoint C */
#define S1R72_EPD_MAX_PKT		512		/** max packet size of endpoint D */
#define S1R72_EPE_MAX_PKT		512		/** max packet size of endpoint E */
#endif
/** EP0 FIFO address */
#define S1R72_EP0_START_ADRS		0x030
#define S1R72_EP0_END_ADRS		S1R72_EP0_START_ADRS + S1R72_EP0_MAX_PKT
#define S1R72_EPA_START_ADRS		S1R72_EP0_END_ADRS
#define S1R72_EPA_END_ADRS		S1R72_EPA_START_ADRS + S1R72_EPA_MAX_PKT
#define S1R72_EPB_START_ADRS		S1R72_EPA_END_ADRS
#define S1R72_EPB_END_ADRS		S1R72_EPB_START_ADRS + S1R72_EPB_MAX_PKT
#define S1R72_EPC_START_ADRS		S1R72_EPB_END_ADRS
#define S1R72_EPC_END_ADRS		S1R72_EPC_START_ADRS + S1R72_EPC_MAX_PKT
#define S1R72_EPD_START_ADRS		S1R72_EPC_END_ADRS
#define S1R72_EPD_END_ADRS		S1R72_EPD_START_ADRS + S1R72_EPD_MAX_PKT
#define S1R72_EPE_START_ADRS		S1R72_EPD_END_ADRS
#define S1R72_EPE_END_ADRS		S1R72_EPE_START_ADRS + S1R72_EPE_MAX_PKT
/*@}*/

/**
 * @name S1R72_WAKEUP_TIM_2MSEC
 * @brief	definitions of OSC wakeup timer value.
*/
#define S1R72_WAKEUP_TIM_2MSEC	0xBBCC	/* = BBCC = 48,076 x 41.67ns = 2msec */
/*@}*/

#define S1R72_WAKEUP_TIM_0MSEC	0x5DE6	/* 20070627 Change T.Katsumi */

/**
 * @name S1R72_EPXXX
 * @brief	definitions of endpoint settings macro.
*/
/** FIFO clear macro */
#define S1R72_EPFIFO_CLR(x,y)	\
	rcD_RegWrite8(x, rcD_S1R72_EPrFIFO_Clr, (1 << (y)) )
#define S1R72_EP0FIFO_CLR(x)	\
	S1R72_EPFIFO_CLR(x, 0)
/** FIFO Join clear macro */
#define S1R72_EPJOINCPU_CLR(x)	\
	rcD_RegWrite8(x, rcD_S1R72_ClrAllEPnJoin , \
		(S1R72_ClrJoinCPU_Rd | S1R72_ClrJoinCPU_Wr))
/** epx interrupt enabling macro */
#define S1R72_EPINT_ENB(x)	(1 << ((x) - 1))
/** epx AREAxJoin_1 enabling macro */
#define S1R72_AREAxJOIN_ENB(x,y)	\
	rcD_RegWrite8(x, rcAREA0Join_1 + ( (y) << 1 ), (1 << y) )
/** epx AREAxJoin_1 disable macro */
#define S1R72_AREAxJOIN_DIS(x,y)	\
	rcD_RegWrite8(x, rcAREA0Join_1 + ( (y) << 1 ) , S1R72_REG_ALL_CLR)
/** ep offset address */
#define S1R72_EPxOFFSET8(x)	\
	((x - 1) * 0x08)
#define S1R72_EPxOFFSET4(x)	\
	((x - 1) * 0x04)
#define S1R72_EPxOFFSET2(x)	\
	((x - 1) * 0x02)
/*@}*/

/**
 * @name rcD_S1R72_XXX
 * @brief	definitions of register address and bits.
*/
#define rcD_S1R72_SIE_IntStat					(0x0B0)
	#define S1R72_NonJ				BIT(6)
	#define S1R72_RcvSOF			BIT(5)
	#define S1R72_DetectRESET		BIT(4)
	#define S1R72_DetectSUSPEND		BIT(3)
	#define S1R72_ChirpCmp			BIT(2)
	#define S1R72_RestoreCmp		BIT(1)
	#define S1R72_SetAddressCmp		BIT(0)
	#define S1R72_ALL_SIE_IntStat		(S1R72_NonJ | S1R72_RcvSOF\
			| S1R72_DetectRESET | S1R72_DetectSUSPEND\
			| S1R72_DetectSUSPEND | S1R72_ChirpCmp\
			| S1R72_RestoreCmp | S1R72_SetAddressCmp)

#define rcD_S1R72_BulkIntStat					(0x0B3)
	#define S1R72_CBW_Cmp			BIT(7)
	#define S1R72_CBW_LengthErr		BIT(6)
	#define S1R72_CBW_Err			BIT(5)
	#define S1R72_CSW_Cmp			BIT(3)
	#define S1R72_CSW_Err			BIT(2)

#define rcD_S1R72_EPrIntStat					(0x0B4)
	#define S1R72_EPeIntStat		BIT(4)
	#define S1R72_EPdIntStat		BIT(3)
	#define S1R72_EPcIntStat		BIT(2)
	#define S1R72_EPbIntStat		BIT(1)
	#define S1R72_EPaIntStat		BIT(0)
	#define S1R72_EPxIntStat_NONE	(0x00)

#define rcD_S1R72_EP0IntStat					(0x0B5)
#define rcD_S1R72_EPaIntStat					(0x0B6)
#define rcD_S1R72_EPbIntStat					(0x0B7)
#define rcD_S1R72_EPcIntStat					(0x0B8)
#define rcD_S1R72_EPdIntStat					(0x0B9)
#define rcD_S1R72_EPeIntStat					(0x0BA)
#define rcD_S1R72_EPxIntStat(index)				(0x0B6 + index)
	#define S1R72_DescriptorCmp		BIT(7)
	#define S1R72_OUT_ShortACK		BIT(6)
	#define S1R72_IN_TranACK		BIT(5)
	#define S1R72_OUT_TranACK		BIT(4)
	#define S1R72_IN_TranNAK		BIT(3)
	#define S1R72_OUT_TranNAK		BIT(2)
	#define S1R72_IN_TranErr		BIT(1)
	#define S1R72_OUT_TranErr		BIT(0)
	#define S1R72_EP_ALL_INT		( S1R72_DescriptorCmp\
			| S1R72_OUT_ShortACK	| S1R72_IN_TranACK\
			| S1R72_OUT_TranACK		| S1R72_IN_TranNAK\
			| S1R72_OUT_TranNAK		| S1R72_IN_TranErr\
			| S1R72_OUT_TranErr)
#define rcD_S1R72_AlarmIN_IntStat_H				(0x0BC)
#define rcD_S1R72_AlarmIN_IntStat_L				(0x0BD)

#define rcD_S1R72_AlarmOUT_IntStat_H			(0x0BE)
#define rcD_S1R72_AlarmOUT_IntStat_L			(0x0BF)

#define rcD_S1R72_SIE_IntEnb					(0x0C0)
	#define S1R72_EnNonJ			BIT(6)
	#define S1R72_EnRcvSOF			BIT(5)
	#define S1R72_EnDetectRESET		BIT(4)
	#define S1R72_EnDetectSUSPEND	BIT(3)
	#define S1R72_EnChirpCmp		BIT(2)
	#define S1R72_EnRestoreCmp		BIT(1)
	#define S1R72_EnSetAddressCmp	BIT(0)

#define rcD_S1R72_BulkIntEnb					(0x0C3)

#define rcD_S1R72_EPrIntEnb						(0x0C4)

#define rcD_S1R72_EP0IntEnb						(0x0C5)
#define rcD_S1R72_EPaIntEnb						(0x0C6)
#define rcD_S1R72_EPbIntEnb						(0x0C7)
#define rcD_S1R72_EPcIntEnb						(0x0C8)
#define rcD_S1R72_EPdIntEnb						(0x0C9)
#define rcD_S1R72_EPeIntEnb						(0x0CA)
#define rcD_S1R72_EPxIntEnb(index)				(0x0C6 + index)
	#define S1R72_EnOUT_ShortACK	BIT(6)
	#define S1R72_EnIN_TranACK		BIT(5)
	#define S1R72_EnOUT_TranACK		BIT(4)
	#define S1R72_EnIN_TranNAK		BIT(3)
	#define S1R72_EnOUT_TranNAK		BIT(2)
	#define S1R72_EnIN_TranErr		BIT(1)
	#define S1R72_EnOUT_TranErr		BIT(0)

#define rcD_S1R72_AlarmIN_IntEnb_H				(0x0CC)
#define rcD_S1R72_AlarmIN_IntEnb_L				(0x0CD)

#define rcD_S1R72_AlarmOUT_IntEnb_H				(0x0CE)
#define rcD_S1R72_AlarmOUT_IntEnb_L				(0x0CF)

#define rcD_S1R72_NegoControl					(0x0D0)
	#define S1R72_DisBusDetect		BIT(7)
	#define S1R72_EnAutoNego		BIT(6)
	#define S1R72_InSUSPEND			BIT(5)
	#define S1R72_DisableHS			BIT(4)
	#define S1R72_SendWakeup		BIT(3)
	#define S1R72_RestoreUSB		BIT(2)
	#define S1R72_GoChirp			BIT(1)
	#define S1R72_ActiveUSB			BIT(0)

#define rcD_S1R72_XcvrControl					(0x0D3)
	#define S1R72_TermSelect		BIT(7)
	#define S1R72_XcvrSelect		BIT(6)
	#define S1R72_OpMode			(0x03)
	#define S1R72_OpMode_PowerDown	(0x03)
	#define S1R72_OpMode_DisableBitStuffing	(0x02)
	#define S1R72_OpMode			(0x03)
	#define S1R72_OpMode_NonDriving	(0x01)
	#define S1R72_OpMode_Normal		(0x00)

#define rcD_S1R72_USB_Test						(0x0D4)
	#define S1R72_EnHS_Test			BIT(7)
	#define S1R72_Test_SE0_NAK		BIT(3)
	#define S1R72_Test_J			BIT(2)
	#define S1R72_Test_K			BIT(1)
	#define S1R72_Test_Packet		BIT(0)

#define rcD_S1R72_EPnControl					(0x0D6)
	#define S1R72_AllForceNAK		BIT(7)
	#define S1R72_EPrForceSTALL		BIT(6)

#define rcD_S1R72_BulkOnlyControl				(0x0D8)

#define rcD_S1R72_BulkOnlyConfig				(0x0D9)

#define rcD_S1R72_EP0SETUP_0					(0x0E0)
#define rcD_S1R72_EP0SETUP_1					(0x0E1)
#define rcD_S1R72_EP0SETUP_2					(0x0E2)
#define rcD_S1R72_EP0SETUP_3					(0x0E3)
#define rcD_S1R72_EP0SETUP_4					(0x0E4)
#define rcD_S1R72_EP0SETUP_5					(0x0E5)
#define rcD_S1R72_EP0SETUP_6					(0x0E6)
#define rcD_S1R72_EP0SETUP_7					(0x0E7)
#define rcD_S1R72_EP0SETUP(index)				(0x0E0 + index)

#define rcD_S1R72_USB_Address					(0x0E8)
#define rcD_S1R72_SETUP_Control					(0x0EA)
	#define S1R72_ProtectEP0		BIT(0)

#define rsD_S1R72_FrameNumber					(0x0EE)
#define rcD_S1R72_FrameNumber_H					(0x0EE)
#define rcD_S1R72_FrameNumber_L					(0x0EF)
	#define S1R72_FrameNumber		(0x07FF)
	#define S1R72_FrameNumber_H		(0x07)
	#define S1R72_FrameNumber_L		(0xFF)

#define rcD_S1R72_EP0MaxSize					(0x0F0)
#define rcD_S1R72_EP0Control					(0x0F1)
	#define S1R72_ReplyDescriptor	BIT(0)
	#define S1R72_EP0INxOUT			BIT(7)
	#define S1R72_EP0INxOUT_IN			(0x80)
	#define S1R72_EP0INxOUT_OUT		(0x00)

#define rcD_S1R72_EP0ControlIN					(0x0F2)
	#define S1R72_EnShortPkt		BIT(6)
	#define S1R72_ToggleStat		BIT(4)
	#define S1R72_ToggleSet			BIT(3)
	#define S1R72_ToggleClr			BIT(2)
	#define S1R72_ForceNAK			BIT(1)
	#define S1R72_ForceSTALL		BIT(0)

#define rcD_S1R72_EP0ControlOUT					(0x0F3)
	#define S1R72_AutoForceNAK		BIT(7)

#define rcD_S1R72_EPaMaxSize_H					(0x0F8)
#define rcD_S1R72_EPaMaxSize_L					(0x0F9)
#define rcD_S1R72_EPaConfig_0					(0x0FA)
	#define S1R72_INxOUT			BIT(7)
	#define S1R72_IntEP_Mode		BIT(6)
	#define S1R72_ISO				BIT(5)		/* V05 - S1R72_EnEndpoint */
	#define S1R72_EndpointNumber	(0x0F)
	#define S1R72_DIR_IN		(0x80)
#define rcD_S1R72_EPaControl					(0x0FC)
#define rcD_S1R72_EPbMaxSize_H					(0x100)
#define rcD_S1R72_EPbMaxSize_L					(0x101)
#define rcD_S1R72_EPbConfig_0					(0x102)
#define rcD_S1R72_EPbControl					(0x104)
#define rcD_S1R72_EPcMaxSize_H					(0x108)
#define rcD_S1R72_EPcMaxSize_L					(0x109)
#define rcD_S1R72_EPcConfig_0					(0x10A)
#define rcD_S1R72_EPcControl					(0x10C)
#define rcD_S1R72_EPdMaxSize_H					(0x110)
#define rcD_S1R72_EPdMaxSize_L					(0x111)
#define rcD_S1R72_EPdConfig_0					(0x112)
#define rcD_S1R72_EPdControl					(0x114)
#define rcD_S1R72_EPeMaxSize_H					(0x118)
#define rcD_S1R72_EPeMaxSize_L					(0x119)
#define rcD_S1R72_EPeConfig_0					(0x11A)
#define rcD_S1R72_EPeControl					(0x11C)
#define rcD_S1R72_EPxMaxSize_H(index)			(0x0F8 + (index<<3))
#define rcD_S1R72_EPxMaxSize_L(index)			(0x0F9 + (index<<3))
#define rcD_S1R72_EPxConfig(index)				(0x0FA + (index<<3))

#define rcD_S1R72_EPxControl(index)				(0x0FC + (index<<3))

#define rcD_S1R72_DescAdrs_H					(0x120)
#define rcD_S1R72_DescAdrs_L					(0x121)
#define rcD_S1R72_DescSize_H					(0x122)
#define rcD_S1R72_DescSize_L					(0x123)
#define rcD_S1R72_EP_DMA_Ctrl					(0x126)

#define rcD_S1R72_EnEP_IN_H						(0x128)
#define rcD_S1R72_EnEP_IN_L						(0x129)
#define rcD_S1R72_EnEP_OUT_H					(0x12A)
#define rcD_S1R72_EnEP_OUT_L					(0x12B)
#define rcD_S1R72_EnEP_IN_ISO_H					(0x12C)
#define rcD_S1R72_EnEP_IN_ISO_L					(0x12D)
#define rcD_S1R72_EnEP_OUT_ISO_H				(0x12E)
#define rcD_S1R72_EnEP_OUT_ISO_L				(0x12F)

#define rsD_S1R72_AlarmIN_IntStat				(0x0BC)
#define rsD_S1R72_AlarmOUT_IntStat				(0x0BE)
#define rsD_S1R72_AlarmIN_IntEnb				(0x0CC)
#define rsD_S1R72_AlarmOUT_IntEnb				(0x0CE)

#define rsD_S1R72_EPaMaxSize					(0x0F8)
#define rsD_S1R72_EPbMaxSize					(0x100)
#define rsD_S1R72_EPcMaxSize					(0x108)
#define rsD_S1R72_EPdMaxSize					(0x110)
#define rsD_S1R72_EPeMaxSize					(0x118)
#define rsD_S1R72_EPxMaxSize(index)				(0x0F8 + (index<<3))

#define rsD_S1R72_DescAdrs						(0x120)
#define rsD_S1R72_DescSize						(0x122)

#define rsD_S1R72_EnEP_IN						(0x128)
#define rsD_S1R72_EnEP_OUT						(0x12A)
#define rsD_S1R72_EnEP_IN_ISO					(0x12C)
#define rsD_S1R72_EnEP_OUT_ISO					(0x12E)
/*@}*/

/**
 * @name rcD_XXX
 * @brief	definitions of change defines V05 to V17.
*/
#define rcD_S1R72_EP0Join	rcAREA0Join_0	/* (0x125) Device EP0 Join */
#define rcD_S1R72_EPaJoin	rcAREA1Join_0	/* (0x135) Device EPa Join */
#define rcD_S1R72_EPbJoin	rcAREA2Join_0	/* (0x145) Device EPb Join */
#define rcD_S1R72_EPcJoin	rcAREA3Join_0	/* (0x155) Device EPc Join */
#define rcD_S1R72_EPdJoin	rcAREA4Join_0	/* (0x165) Device EPc Join */
#define rcD_S1R72_EPeJoin	rcAREA5Join_0	/* (0x175) Device EPc Join */
	#define S1R72_JoinFIFO_Stat		BIT(7)
	#define S1R72_JoinDMA0			BIT(2)
	#define S1R72_JoinCPU_Rd		BIT(1)
	#define S1R72_JoinCPU_Wr		BIT(0)

/* (0x80) Device EP0 Start Address */
#define rsD_S1R72_EP0StartAdrs		rcAREA0StartAdrs_H
#define rcD_S1R72_EP0StartAdrs_H	rcAREA0StartAdrs_H
#define rcD_S1R72_EP0StartAdrs_L	rcAREA0StartAdrs_L
/* (0x82) Device EP0 End Address */
#define rsD_S1R72_EP0EndAdrs		rcAREA0EndAdrs_H
#define rcD_S1R72_EP0EndAdrs_H		rcAREA0EndAdrs_H
#define rcD_S1R72_EP0EndAdrs_L		rcAREA0EndAdrs_L
/* (0x84) Device EPa Start Address */
#define rsD_S1R72_EPaStartAdrs		rcAREA1StartAdrs_H
#define rcD_S1R72_EPaStartAdrs_H	rcAREA1StartAdrs_H
#define rcD_S1R72_EPaStartAdrs_L	rcAREA1StartAdrs_L
/* (0x86) Device EPa End Address */
#define rsD_S1R72_EPaEndAdrs		rcAREA1EndAdrs_H
#define rcD_S1R72_EPaEndAdrs_H		rcAREA1EndAdrs_H
#define rcD_S1R72_EPaEndAdrs_L		rcAREA1EndAdrs_L
/* (0x88) Device EPb Start Address */
#define rsD_S1R72_EPbStartAdrs		rcAREA2StartAdrs_H
#define rcD_S1R72_EPbStartAdrs_H	rcAREA2StartAdrs_H
#define rcD_S1R72_EPbStartAdrs_L	rcAREA2StartAdrs_L
/* (0x8a) Device EPb End Address */
#define rsD_S1R72_EPbEndAdrs		rcAREA2EndAdrs_H
#define rcD_S1R72_EPbEndAdrs_H		rcAREA2EndAdrs_H
#define rcD_S1R72_EPbEndAdrs_L		rcAREA2EndAdrs_L
/* (0x8c) Device EPc Start Address */
#define rsD_S1R72_EPcStartAdrs		rcAREA3StartAdrs_H
#define rcD_S1R72_EPcStartAdrs_H	rcAREA3StartAdrs_H
#define rcD_S1R72_EPcStartAdrs_L	rcAREA3StartAdrs_L
/* (0x8e) Device EPc End Address */
#define rsD_S1R72_EPcEndAdrs		rcAREA3EndAdrs_H
#define rcD_S1R72_EPcEndAdrs_H		rcAREA3EndAdrs_H
#define rcD_S1R72_EPcEndAdrs_L		rcAREA3EndAdrs_L
/* (0x90) Device EPd Start Address */
#define rsD_S1R72_EPdStartAdrs		rcAREA4StartAdrs_H
#define rcD_S1R72_EPdStartAdrs_H	rcAREA4StartAdrs_H
#define rcD_S1R72_EPdStartAdrs_L	rcAREA4StartAdrs_L
/* (0x92) Device EPd End Address */
#define rsD_S1R72_EPdEndAdrs		rcAREA4EndAdrs_H
#define rcD_S1R72_EPdEndAdrs_H		rcAREA4EndAdrs_H
#define rcD_S1R72_EPdEndAdrs_L		rcAREA4EndAdrs_L
/* (0x94) Device EPe Start Address */
#define rsD_S1R72_EPeStartAdrs		rcAREA5StartAdrs_H
#define rcD_S1R72_EPeStartAdrs_H	rcAREA5StartAdrs_H
#define rcD_S1R72_EPeStartAdrs_L	rcAREA5StartAdrs_L
/* (0x96) Device EPe End Address */
#define rsD_S1R72_EPeEndAdrs		rcAREA5EndAdrs_H
#define rcD_S1R72_EPeEndAdrs_H		rcAREA5EndAdrs_H
#define rcD_S1R72_EPeEndAdrs_L		rcAREA5EndAdrs_L

#define	rcD_CH0StartAdrs_H		rcAREA0StartAdrs_H
#define	rcD_CH0StartAdrs_L		rcAREA0StartAdrs_L
#define	rcD_CH0EndAdrs_H		rcAREA0EndAdrs_H
#define	rcD_CH0EndAdrs_L		rcAREA0EndAdrs_L
#define	rcD_CHaStartAdrs_H		rcAREA1StartAdrs_H
#define	rcD_CHaStartAdrs_L		rcAREA1StartAdrs_L
#define	rcD_CHaEndAdrs_H		rcAREA1EndAdrs_H
#define	rcD_CHaEndAdrs_L		rcAREA1EndAdrs_L
#define	rcD_CHbStartAdrs_H		rcAREA2StartAdrs_H
#define	rcD_CHbStartAdrs_L		rcAREA2StartAdrs_L
#define	rcD_CHbEndAdrs_H		rcAREA2EndAdrs_H
#define	rcD_CHbEndAdrs_L		rcAREA2EndAdrs_L
#define	rcD_CHcStartAdrs_H		rcAREA3StartAdrs_H
#define	rcD_CHcStartAdrs_L		rcAREA3StartAdrs_L
#define	rcD_CHcEndAdrs_H		rcAREA3EndAdrs_H
#define	rcD_CHcEndAdrs_L		rcAREA3EndAdrs_L
#define	rcD_CHdStartAdrs_H		rcAREA4StartAdrs_H
#define	rcD_CHdStartAdrs_L		rcAREA4StartAdrs_L
#define	rcD_CHdEndAdrs_H		rcAREA4EndAdrs_H
#define	rcD_CHdEndAdrs_L		rcAREA4EndAdrs_L
#define	rcD_CHeStartAdrs_H		rcAREA5StartAdrs_H
#define	rcD_CHeStartAdrs_L		rcAREA5StartAdrs_L
#define	rcD_CHeEndAdrs_H		rcAREA5EndAdrs_H
#define	rcD_CHeEndAdrs_L		rcAREA5EndAdrs_L

#define	rcD_S1R72_ClrAllEPnJoin		rcClrAREAnJoin_0
#define rcD_S1R72_Reset			rcChipReset		/* (0x100) Device Reset */
	#define S1R72_ResetDTM			ResetMTM
/*@}*/

/*****************************************
 * definitions of "enum"
 *****************************************/
enum S1R72_GD_EP_NAME {
	S1R72_GD_EP0	= 0,
	S1R72_GD_EPA,
	S1R72_GD_EPB,
	S1R72_GD_EPC,
	S1R72_GD_EPD,
	S1R72_GD_EPE,
	S1R72_MAX_ENDPOINT
};

#endif	/* S1R72V17_USBC_H */

