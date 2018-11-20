/* $Id$ */

/**
 * @file phy.h
 * @date 2007-Dez-4
 * @author S.Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Functions for data transmission and reception.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This module contains all functions to operate the nanoLOC chip.
 *
 * $Revision: 7207 $
 * $Date: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 */
/*
 * $Log$
 */

#ifndef PHY_H
#define PHY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def PHY_DEBUG
 * @brief Enables debugging mode.
 */
//#define PHY_DEBUG	1

/**
 * @def DEBUG_PHY
 * @brief Debugging output using printf. Can be globally disabled with
 * @ref PHY_DEBUG .
 */
#ifdef PHY_DEBUG
#include <portation.h>
#	define DEBUG_PHY	PRINTF
#else  /*PHY_DEBUG*/
#	define DEBUG_PHY	NoDebugging
#endif /*PHY_DEBUG*/

#include "ntrxtypes.h"

#define PHY_INIT_PAGE				0
#define PHY_PAGES_SUPPORTED			0
#define PHY_CHANNEL_MIN 			11
#define PHY_CHANNEL_COUNT			16
#define PHY_TX_POWER_TOLERANCE		0x01

#define	PHY_BUSY 					0x00
#define PHY_BUSY_RX 				0x01
#define PHY_BUSY_TX 				0x02
#define PHY_FORCE_TRX_OFF			0x03
#define	PHY_IDLE					0x04
#define	PHY_INVALID_PARAMETER 		0x05
#define	PHY_RX_ON					0x06
#define PHY_SUCCESS					0x07
#define PHY_TRX_OFF 				0x08
#define	PHY_TX_ON 					0x09
#define PHY_UNSUPPORTED_ATTRIBUTE 	0x0a
#define PHY_READ_ONLY				0x0b
#define PHY_NO_ACK					0x0c
#define PHY_CONFIGURATION_ERROR		0x0d
#define PHY_FP_NOT_SET		0x0e
#define PHY_NO_MEMORY 0x0f
#define PHY_USER_DATA_OVERFLOW 0x10

#define PHY_CURRENT_CHANNEL			0x00
#define PHY_CANNELS_SUPPORTED		0x01
#define PHY_TX_POWER				0x02
#define PHY_CCA_MODE				0x03
#define PHY_CURRENT_PAGE			0x04
#define PHY_MAX_FRAME_DURATION		0x05
#define PHY_SHR_DURATION			0x06
#define PHY_SYMBOLS_PER_OCTET		0x07
#define PHY_ARQ						0x70
#define PHY_ARQ_MAX					0x71
#define PHY_FEC						0x72
#define PHY_MAC_ADDRESS1			0x73
#define PHY_MAC_ADDRESS2			0x74
#define PHY_TX_ADDR_SELECT			0x75
#define PHY_ADDR_MATCHING			0x76
#define PHY_RAW_MODE				0x77
#define PHY_RECALIBRATION			0x78
#define PHY_TRX_MODE				0x79
#define PHY_PWR_DOWN_MODE			0x7a
#define PHY_FRAME_TYPE				0x7b
#define PHY_TESTMODE				0x7c
#define PHY_RX_STATE				0x7d
#define PHY_RSSI					0x7e
#define PHY_LOG_CHANNEL				0x7f
#define PHY_SYNCWORD				0x90
#define PHY_RX_CMD					0x91
#define PHY_RECAL_INTERVAL      	0x94

#define PHY_RX_TIMEOUT				0x80 /* new */
#define PHY_POWERDOWN_PAD_MODE		0x81

#define PHY_NORMAL_MODE				0x00
#define PHY_CONTINUOUS_MODE			0x01
#define PHY_CARRIER_MODE			0x02

#define PHY_CCA_OFF					0x00
#define PHY_CCA_MODE_1				0x01
#define PHY_CCA_MODE_2				0x02
#define PHY_CCA_MODE_3				0x03
#define PHY_CCA_MODE_DEFAULT		PHY_CCA_OFF

#define PD_DATA_REQUEST				0x01
#define PD_DATA_CONFIRM				0x02
#define PD_DATA_INDICATION			0x03
#define PD_STATUS_INDICATION		0x04 /* new */
#define PLME_CCA_REQUEST			0x05
#define PLME_CCA_CONFIRM			0x06
#define PLME_ED_REQUEST				0x07
#define PLME_ED_CONFIRM				0x08
#define PLME_GET_REQUEST			0x09
#define PLME_GET_CONFIRM			0x0a
#define PLME_SET_REQUEST			0x0b
#define PLME_SET_CONFIRM			0x0c /* TODO: is this correct?! */
#define PLME_SET_TRX_STATE_REQUEST  0x0d
#define PLME_SET_TRX_STATE_CONFIRM  0x0c /* TODO: is this correct?! */
/* ranging defines */
#define PD_RANGING_REQUEST			0x0e
#define PD_RANGING_REQUEST_EXECUTE	0x0f
#define PD_RANGING_ANSWER1_EXECUTE	0x10
#define PD_RANGING_ANSWER2_EXECUTE	0x11
#define PD_RANGING_CONFIRM			0x12
#define PD_RANGING_INDICATION		0x13
#define RG_STAT_DEFAULT  		0x00
#define RG_STAT_NO_ERROR 		0x0F

#define RG_STAT_T1		0x01
#define RG_STAT_T2		0x02
#define RG_STAT_T3		0x04
#define RG_STAT_T4		0x08

#define RG_STAT_TIMEOUT			0x40
#define RG_STAT_VALUE_ERROR		0x80

#define PHY_FRAME_TYPE_DATA		0x00 /* NA_TypeCodeData_VC_C */
#define PHY_FRAME_TYPE_ACK		0x01 /* NA_TypeCodeAck_VC_C */
#define PHY_FRAME_TYPE_TIMEB	0x02 /* NA_TypeCodeTimeB_VC_C */
#define PHY_FRAME_TYPE_BRDC		0x03 /* NA_TypeCodeBrdcast_VC_C */
#define PHY_FRAME_TYPE_REQ2S	0x04 /* NA_TypeCodeReq2s_VC_C */
#define PHY_FRAME_TYPE_CLR2S	0x05 /* NA_TypeCodeClr2s_VC_C */
#define PHY_FRAME_TYPE_UNDEF	0xFF /* NA_TypeCodeUndef_VC_C */
#define PHY_BRDC_ADDR	{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
extern AddrT 		brdcAddr;

/**
 * @def TO_CSMA
 * @brief Timeout for any CSMA/CCA process. [ms]
 *
 * This timeout runs after sending a packet. If the timer expire
 * the transmission is cancelled and restarted with disabled CSMA/CCA.
 *
 */
#define TO_CSMA 20

bool_t NTRXInit		(void);
void PHYInit 		(void);
void PHYPoll	(int sending);
void PDCallback 	(void);
int PDSap 			(MsgT *msg);			// remove data confirm and rather provide a function return. 
void PLMESap 		(MsgT *msg);
void NTRXSetDefaultMode( void );
void NTRXSendPacket (MsgT *msg);
void PHYRegisterCallback(callbackfn_t cbMsg);
void NTRXPostBBIrq( void );
bool_t PHYIsIDLE(void);
void UpdateLEDs (void);
void PHYAutoCalibration( void );

/**
 * @def SendMsgUp
 * @brief Callback function of upper layer.
 *
 * This function is called when the physical layer wants to forward received
 * data to the next upper layer. This define is used to be able to add
 * another layer.
 *
 */
#define SendMsgUp (*phySendMsgUp)

#ifdef __cplusplus
}
#endif


#endif

