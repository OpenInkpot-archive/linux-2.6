/**
 * @file	s1r72v17.h
 * @brief	S1R72V17 Driver
 * @date	2006/12/11
 * Copyright (C)SEIKO EPSON CORPORATION 2003-2006. All rights reserved.
 */
/*
 *  rcXXX = Register 8Bit Access
 *  rsXXX = Register 16Bit Access
 *
 */


#ifndef __S1R72V17_USB_H__
#define	__S1R72V17_USB_H__

#define _BIT(x)		(1 << x)

/*==========================================================================================================/
/ Register Access Define																				/
/==========================================================================================================*/

#define rcMainIntStat			(0x00)	/* Main Interrupt Status			*/
#define rcMainIntEnb			(0x08)	/* Main Interrupt Enable			*/
#define DevInt				_BIT(7)
#define HostInt				_BIT(6)
#define CPUInt				_BIT(5)
#define IDEInt				_BIT(4)
#define FinishedPM			_BIT(0)

#define rcDeviceIntStat			(0x01)	/* EPr Interrupt Status				*/
#define D_VBUS_Changed		_BIT(7)
#define D_SIE_IntStat		_BIT(5)
#define D_BulkIntStat		_BIT(4)
#define D_RcvEP0SETUP		_BIT(3)
#define D_FIFO_IntStat		_BIT(2)
#define D_EP0IntStat		_BIT(1)
#define D_EPrIntStat		_BIT(0)

#define rcDeviceIntEnb			(0x09)	/* EPr Interrupt Enable			*/

#define rcHostIntStat			(0x02)	/* SIE Interrupt Status		*/
#define rcHostIntEnb			(0x0A)	/* SIE Interrupt Enable		*/
#define VBUSErr				_BIT(7)
#define LineChg				_BIT(6)
#define H_SIE_Int1			_BIT(5)
#define H_SIE_Int0			_BIT(4)
#define H_FrameInt			_BIT(3)
#define H_FIFO_Int			_BIT(2)
#define H_CH0Int			_BIT(1)
#define H_CHrInt			_BIT(0)

#define rcCPU_IntStat			(0x03)	/* CPU Interrupt Status			*/
#define rcCPU_IntEnb			(0x0B)	/* CPU Interrupt Enable			*/

#define rcFIFO_IntStat			(0x04)	/* CPU Interrupt Status			*/
#define rcFIFO_IntEnb			(0x0C)	/* CPU Interrupt Enable			*/

#define rcRevisionNum			(0x10)	/* Revision Number				*/
#define RevisionNum		0x10
#define rcChipReset				(0x11)	/* Chip Reset					*/
#define ResetMTM			_BIT(7)
#define AllReset			_BIT(0)

#define rcPM_Control_0			(0x12)	/* Power Management Control 0	*/
#define GoSleep				_BIT(7)
#define GoActive			_BIT(6)
#define GoCPU_Cut			_BIT(5)
#define PM_State_Mask		(0x03)
#define SLEEP				(0x00)
#define SNOOZE				(0x01)
#define ACTIVE				(0x03)
#define	S1R72_PM_State_ACT_DEVICE	ACTIVE
#define S1R72_PM_State_SLEEP		SLEEP
#define S1R72_PM_State_SNOOZE		SNOOZE

#define rsWakeupTim				(0x14)	/* Wake Up Timer				*/
#define rcWakeupTim_H			(0x14)	/* Wake Up Timer High			*/
#define rcWakeupTim_L			(0x15)	/* Wake Up Timer Low			*/
#define rcH_USB_Control			(0x16)	/* Host USB Control				*/
#define VBUS_Enb			_BIT(7)

#define rcH_XcvrControl			(0x17)	/* Host Xcvr Control			*/
#define rcD_USB_Status			(0x18)	/* Device USB Status			*/
#define S1R72_VBUS				_BIT(7)
#define S1R72_FSxHS				_BIT(6)
#define S1R72_Status_FS			(0x40)
#define S1R72_Status_HS			(0x00)
#define S1R72_LineState			(0x03)
#define S1R72_LineState_SE0		(0x00)
#define S1R72_LineState_J		(0x01)
#define S1R72_LineState_K		(0x02)
#define S1R72_LineState_SE1		(0x03)

#define rcH_USB_Status			(0x19)	/* Host USB Status				*/

#define rcHostDeviceSel			(0x1F)	/* Host Device Select			*/
#define HOST_MODE			_BIT(0)

#define rsFIFO_Rd				(0x20)	/* FIFO Read					*/
#define rcFIFO_Rd_0				(0x20)	/* FIFO Read High				*/
#define rcFIFO_Rd_1				(0x21)	/* FIFO Read Low				*/

#define rsFIFO_Wr				(0x22)	/* FIFO Write					*/
#define rcFIFO_Wr_0				(0x22)	/* FIFO Write High				*/
#define rcFIFO_Wr_1				(0x23)	/* FIFO Write Low				*/

#define rsFIFO_RdRemain			(0x24)	/* FIFO Read Remain High		*/
#define rcFIFO_RdRemain_H		(0x24)	/* FIFO Read Remain High		*/
#define rcFIFO_RdRemain_L		(0x25)	/* FIFO Read Remain Low			*/
#define FIFO_RdRemain			(0x1FFF)	/* FIFO Read Remain mask	*/
#define RdRemainValid			_BIT(15)	/* FIFO Read Remain valid		*/
#define rsFIFO_WrRemain			(0x26)	/* FIFO Write Remain High		*/
#define rcFIFO_WrRemain_H		(0x26)	/* FIFO Write Remain High		*/
#define rcFIFO_WrRemain_L		(0x27)	/* FIFO Write Remain Low		*/
#define rcFIFO_ByteRd			(0x28)	/* FIFO Byte Read				*/


#define rcRAM_RdAdrs_H			(0x30)	/* RAM Read Address High		*/
#define rcRAM_RdAdrs_L			(0x31)	/* RAM Read Address Low			*/
#define rcRAM_RdControl			(0x32)	/* RAM Read Control				*/
#define rcRAM_RdCount			(0x35)	/* RAM Read Counter				*/
#define rcRAM_WrAdrs_H			(0x38)	/* RAM Write Address High		*/
#define rcRAM_WrAdrs_L			(0x39)	/* RAM Write Address Low		*/
#define rcRAM_WrDoor_0			(0x3A)	/* RAM Write Door High			*/
#define rcRAM_WrDoor_1			(0x3B)	/* RAM Write Door Low			*/


#define rcRAM_Rd_00				(0x40)	/* RAM Read 00					*/
#define rcRAM_Rd_01				(0x41)	/* RAM Read 01					*/
#define rcRAM_Rd_02				(0x42)	/* RAM Read 02					*/
#define rcRAM_Rd_03				(0x43)	/* RAM Read 03					*/
#define rcRAM_Rd_04				(0x44)	/* RAM Read 04					*/
#define rcRAM_Rd_05				(0x45)	/* RAM Read 05					*/
#define rcRAM_Rd_06				(0x46)	/* RAM Read 06					*/
#define rcRAM_Rd_07				(0x47)	/* RAM Read 07					*/
#define rcRAM_Rd_08				(0x48)	/* RAM Read 08					*/
#define rcRAM_Rd_09				(0x49)	/* RAM Read 09					*/
#define rcRAM_Rd_0A				(0x4A)	/* RAM Read 0A					*/
#define rcRAM_Rd_0B				(0x4B)	/* RAM Read 0B					*/
#define rcRAM_Rd_0C				(0x4C)	/* RAM Read 0C					*/
#define rcRAM_Rd_0D				(0x4D)	/* RAM Read 0D					*/
#define rcRAM_Rd_0E				(0x4E)	/* RAM Read 0E					*/
#define rcRAM_Rd_0F				(0x4F)	/* RAM Read 0F					*/

#define rcRAM_Rd_10				(0x50)	/* RAM Read 10					*/
#define rcRAM_Rd_11				(0x51)	/* RAM Read 11					*/
#define rcRAM_Rd_12				(0x52)	/* RAM Read 12					*/
#define rcRAM_Rd_13				(0x53)	/* RAM Read 13					*/
#define rcRAM_Rd_14				(0x54)	/* RAM Read 14					*/
#define rcRAM_Rd_15				(0x55)	/* RAM Read 15					*/
#define rcRAM_Rd_16				(0x56)	/* RAM Read 16					*/
#define rcRAM_Rd_17				(0x57)	/* RAM Read 17					*/
#define rcRAM_Rd_18				(0x58)	/* RAM Read 18					*/
#define rcRAM_Rd_19				(0x59)	/* RAM Read 19					*/
#define rcRAM_Rd_1A				(0x5A)	/* RAM Read 1A					*/
#define rcRAM_Rd_1B				(0x5B)	/* RAM Read 1B					*/
#define rcRAM_Rd_1C				(0x5C)	/* RAM Read 1C					*/
#define rcRAM_Rd_1D				(0x5D)	/* RAM Read 1D					*/
#define rcRAM_Rd_1E				(0x5E)	/* RAM Read 1E					*/
#define rcRAM_Rd_1F				(0x5F)	/* RAM Read 1F					*/


#define rcDMA0_Config			(0x61)	/* DMA0 Configuration			*/
#define rcDMA0_Control			(0x62)	/* DMA0 Control					*/

#define rcDMA0_Remain_H			(0x64)	/* DMA0 FIFO Remain High		*/
#define rcDMA0_Remain_L			(0x65)	/* DMA0 FIFO Remain Low			*/

#define rcDMA0_Count_HH		(0x68)	/* DMA0 Transfer Byte Counter High/High	*/
#define rcDMA0_Count_HL		(0x69)	/* DMA0 Transfer Byte Counter High/Low	*/
#define rcDMA0_Count_LH		(0x6A)	/* DMA0 Transfer Byte Counter Low/High	*/
#define rcDMA0_Count_LL		(0x6B)	/* DMA0 Transfer Byte Counter Low/Low	*/

#define rcDMA0_RdData_0			(0x6C)	/* DMA0 Read Data High				*/
#define rcDMA0_RdData_1			(0x6D)	/* DMA0 Read Data Low				*/
#define rcDMA0_WrData_0			(0x6E)	/* DMA0 Write Data High				*/
#define rcDMA0_WrData_1			(0x6F)	/* DMA0 Write Data Low				*/

#define rcModeProtect			(0x71)	/* Mode Protection					*/
#define Protect				(0x00)
#define unProtect			(0x56)

#define rcClkSelect				(0x73)	/* Clock Select						*/
#define ClkSource			_BIT(7)
#define CLK_12				(0)
#define CLK_24				(1)
#define CLK_48				(3)

#define rcCPUConfig				(0x75)	/* Chip Configuration				*/
#define LEV_HIGH			_BIT(7)
#define MODE_Hiz			_BIT(6)
#define DREQ_HIGH			_BIT(5)
#define DACK_HIGH			_BIT(4)
#define MODE_CS				_BIT(3)
#define BUS_SWAP			_BIT(2)	/* CPU_Endian */
#define XBEH				_BIT(1)
#define MODE_8_BIT			_BIT(0)

#define rcChgEndian				(0x77)	/* Chip Configuration				*/

#define rcAREA0StartAdrs		(0x80)	/* AREA 0 Start Address				*/
#define rcAREA0StartAdrs_H		(0x80)	/* AREA 0 Start Address High		*/
#define rcAREA0StartAdrs_L		(0x81)	/* AREA 0 Start Address Low			*/
#define rcAREA0EndAdrs			(0x82)	/* AREA 0 End Address				*/
#define rcAREA0EndAdrs_H		(0x82)	/* AREA 0 End Address High			*/
#define rcAREA0EndAdrs_L		(0x83)	/* AREA 0 End Address Low			*/
#define rcAREA1StartAdrs		(0x84)	/* AREA 1 Start Address				*/
#define rcAREA1StartAdrs_H		(0x84)	/* AREA 1 Start Address High		*/
#define rcAREA1StartAdrs_L		(0x85)	/* AREA 1 Start Address Low			*/
#define rcAREA1EndAdrs			(0x86)	/* AREA 1 End Address				*/
#define rcAREA1EndAdrs_H		(0x86)	/* AREA 1 End Address High			*/
#define rcAREA1EndAdrs_L		(0x87)	/* AREA 1 End Address Low			*/
#define rcAREA2StartAdrs		(0x88)	/* AREA 2 Start Address				*/
#define rcAREA2StartAdrs_H		(0x88)	/* AREA 2 Start Address High		*/
#define rcAREA2StartAdrs_L		(0x89)	/* AREA 2 Start Address Low			*/
#define rcAREA2EndAdrs			(0x8A)	/* AREA 2 End Address				*/
#define rcAREA2EndAdrs_H		(0x8A)	/* AREA 2 End Address High			*/
#define rcAREA2EndAdrs_L		(0x8B)	/* AREA 2 End Address Low			*/
#define rcAREA3StartAdrs		(0x8C)	/* AREA 3 Start Address				*/
#define rcAREA3StartAdrs_H		(0x8C)	/* AREA 3 Start Address High		*/
#define rcAREA3StartAdrs_L		(0x8D)	/* AREA 3 Start Address Low			*/
#define rcAREA3EndAdrs			(0x8E)	/* AREA 3 End Address				*/
#define rcAREA3EndAdrs_H		(0x8E)	/* AREA 3 End Address High			*/
#define rcAREA3EndAdrs_L		(0x8F)	/* AREA 3 End Address Low			*/
#define rcAREA4StartAdrs		(0x90)	/* AREA 4 Start Address				*/
#define rcAREA4StartAdrs_H		(0x90)	/* AREA 4 Start Address High		*/
#define rcAREA4StartAdrs_L		(0x91)	/* AREA 4 Start Address Low			*/
#define rcAREA4EndAdrs			(0x92)	/* AREA 4 End Address				*/
#define rcAREA4EndAdrs_H		(0x92)	/* AREA 4 End Address High			*/
#define rcAREA4EndAdrs_L		(0x93)	/* AREA 4 End Address Low			*/
#define rcAREA5StartAdrs		(0x94)	/* AREA 5 Start Address				*/
#define rcAREA5StartAdrs_H		(0x94)	/* AREA 5 Start Address High		*/
#define rcAREA5StartAdrs_L		(0x95)	/* AREA 5 Start Address Low			*/
#define rcAREA5EndAdrs			(0x96)	/* AREA 5 End Address				*/
#define rcAREA5EndAdrs_H		(0x96)	/* AREA 5 End Address High			*/
#define rcAREA5EndAdrs_L		(0x97)	/* AREA 5 End Address Low			*/

#define	rcAREAnFIFO_clr			(0x9F)
#define S1R72_ClrAREA_ALL		(0x3F)
#define	rcAREA0Join_0			(0xA0)
#define	rcAREA0Join_1			(0xA1)
#define	rcAREA1Join_0			(0xA2)
#define	rcAREA1Join_1			(0xA3)
#define	rcAREA2Join_0			(0xA4)
#define	rcAREA2Join_1			(0xA5)
#define	rcAREA3Join_0			(0xA6)
#define	rcAREA3Join_1			(0xA7)
#define	rcAREA4Join_0			(0xA8)
#define	rcAREA4Join_1			(0xA9)
#define	rcAREA5Join_0			(0xAA)
#define	rcAREA5Join_1			(0xAB)

#define	rcClrAREAnJoin_0		(0xAE)
#define S1R72_ClrJoinFIFO_Stat	_BIT(7)
#define S1R72_ClrJoinDMA		_BIT(2)
#define S1R72_ClrJoinDMA0		_BIT(2)
#define S1R72_ClrJoinCPU_Rd		_BIT(1)
#define S1R72_ClrJoinCPU_Wr		_BIT(0)

#define	rcClrAREAnJoin_1		(0xAF)

#define	rcChipConfig			rcCPUConfig
/* (0x12) Power Management Control 0	*/
#define rcPM_Control_1			rcPM_Control_0

#define	rcD_S1R72_FIFO_IntStat		rcFIFO_IntStat
#define	rcD_S1R72_EPrFIFO_Clr		rcAREAnFIFO_clr

#define	S1R72_AllFIFO_Clr		(0xFF)

#endif

