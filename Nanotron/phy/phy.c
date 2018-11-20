/* $Id$ */
/**
 * @file phy.c
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
#include <string.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include <portation.h>
#include "phy.h"
#include "ntrxutil.h"
#include "nnspi.h"
#include "hwclock.h"
#include "ntrxranging.h"
#define DEVKIT201_CHANGE 1

/**
 * @def SendCfgUp
 * @brief Callback function of upper layer.
 *
 * This function is called when the physical layer wants to send configuration
 * data to the next upper layer. This define is used to be able to add
 * another layer.
 *
 */
#define SendCfgUp (*phySendMsgUp)

void NTRXInterrupt(void);
extern void APLCallback(MsgT *msg);
callbackfn_t phySendMsgUp = &APLCallback;

/****************************************************************************
 *  Local stuff
 ****************************************************************************/
MsgT upMsg; /**< message struct for all received data. */
int16_t rcwd = 0; /**< retransmission counter for dynamic recalibration. */
uint32_t tiRecal; /**< timestamp of last calibration. */
uint32_t tiPhyRxTimeout; /**< timestamp of last rx. */
bool_t tiPhyRxTimeout_once; /**< report only once time */
uint8_t lastArqCount; /**< number of retransmissions of last transmitted message. */
MsgT *txSendMsg = NULL; /**< pointer to store last transmitted message. */
bool_t trxPollMode = TRUE; /**< flag for irq polling. */
uint8_t buffSwapped = FALSE; /**< swap buffer state for accessing the right memory block */
AddrT cacheAddr; /**< destination address */
AddrT brdcAddr = PHY_BRDC_ADDR; /**< broadcast address */
uint8_t last_frameType; /**< last frameType */
volatile uint8_t txIrq; /**< tx interrupt status register. */
volatile uint8_t rxIrq; /**< rx interrupt status register. */

/** @brief structure for all layer configuration settings  */
PhyPIB phyPIB;

/**
 * @brief Initializing of module global variables.
 *
 * This function initializes all module global variables located in the
 * physical layers parameter information base (phyPIB). The nanoLOC chip
 * has to be already initialized and in ready (idle) mode. The receiver is started
 * and interrupts for CRC2 and Tx End and Rx End are enabled.
 *
 */
/****************************************************************************/
void PHYInit(void)
/****************************************************************************/
{
	phyPIB.currentChannel = PHY_CHANNEL_MIN;
	phyPIB.CAMode = PHY_CCA_MODE_DEFAULT;
	phyPIB.titxstart = 0;
	phyPIB.rxState = PHY_TRX_OFF;
	phyPIB.rxOn = FALSE;
	phyPIB.txPower = 0x3f;
	phyPIB.pwrDownMode = 2;
	txSendMsg = NULL;
	phyPIB.recalInterval = CONFIG_NTRX_RECAL_DELAY;
	phyPIB.phyRxTimeout = CONFIG_NTRX_PHY_RX_TIMEOUT;
	tiPhyRxTimeout = hwclock() + phyPIB.phyRxTimeout;
	tiPhyRxTimeout_once = FALSE;
	phyPIB.arqMax = CONFIG_MAX_ARQ;
	phyPIB.testmode = 0;
	phyPIB.fec = FALSE;
	phyPIB.frameType = last_frameType = PHY_FRAME_TYPE_DATA;
	phyPIB.ant = 0;
	trxPollMode = TRUE;
	buffSwapped = FALSE;
	/*
	 * Clear and reset all interrupt flags of the nanoLOC chip
	 */
	ntrxShadowReg[NA_TxIntsReset_O] = 0x3f;
	ntrxShadowReg[NA_RxIntsReset_O] = 0x7f;
	/*
	 * enable tx and rx interrupts
	 */
	ntrxShadowReg[NA_RxIrqEnable_O] |= ((1 << NA_RxIrqEnable_B) | (1 << NA_TxIrqEnable_B));

	NTRXResetSettings();

	/*
	 * enable CRC checks on received messages. This will cause corrupt frames to be dropped
	 * silently by the nanoLOC chip. The receiver is restarted automatically.
	 */
	if ((ntrxShadowReg[NA_RxCrc2Mode_O] & (1 << NA_RxCrc2Mode_B)) != 0)
	{
		ntrxShadowReg[NA_RxIntsEn_O] = (0x01 << NA_RxEnd_B);
	}
	else
	{
		ntrxShadowReg[NA_RxIntsEn_O] = ((0x01 << NA_RxEnd_B) | (0x01 << NA_RxOverflow_B));
	}
	ntrxShadowReg[NA_TxIntsEn_O] = (1 << NA_TxEnd_B);

	NTRXSPIWrite(NA_RxIrqEnable_O, ntrxShadowReg + 0x0f, 6);
	/*
	 * initialize layer global variables
	 */
	txIrqStatus = 0;
	rxIrqStatus = 0;
	txIrq = 0;
	rxIrq = 0;
	ntrxShadowReg[NA_TxIntsReset_O] = 0;
	ntrxShadowReg[NA_RxIntsReset_O] = 0;

	/*
	 * start the receiver of the TRX chip
	 */
	if (phyPIB.rxState == PHY_RX_ON)
	{
		NTRXRXEnable(TRUE);
	}

	ntrxState = TxIDLE;

	NTRXRangingInit();
#	ifdef CONFIG_NTRX_IRQ
	NTRXIrqEnable(TRUE);
#	endif /* CONFIG_NTRX_IRQ */
}

/**
 * @brief This function send packets out.
 * @param *msg this is the message pointer
 *
 * This function is called ever on sending packets (e.g. PDSap).
 *
 */
/****************************************************************************/
void NTRXSendPacket(MsgT *msg)
/****************************************************************************/
{
	uint8_t txLen[2];

	/* if ntrx is in sleep mode, dont use ntrx */
	if (phyPIB.pwrDown == TRUE)
		return;

#	ifdef CONFIG_NTRX_AUTO_RECALIB
	if (phyPIB.recalInterval != 0)
	{
		if (tiRecal < hwclock())
		{
			/* INFO: If the TRX sends a packet, calibration failes!
			 * In this case rcwd is not reset, but tiRecal is.
			 */
			/* normal operation mode */
			if (phyPIB.testmode == 0)
			{
				if (NTRXAllCalibration())
				{
					tiRecal = hwclock() + phyPIB.recalInterval;
					rcwd = 0;
					//TRIGGER_LED_CAL();
				}
			}
		}
	}
#	endif /* CONFIG_NTRX_AUTO_RECALIB */

	/* only messages with payload > 0 allowed, PDSap allready cover this! */
	if (msg->len == 0)
		return;

	/* check which buffer is free to transmit data */
	if (buffSwapped == TRUE)
	{
		/* write user data to transmit buffer in ntrx chip */
		NTRXSetIndexReg(2);
		NTRXSPIWrite((uint8_t)(NA_RamTxBuffer_O & 0xff), msg->data, (uint8_t)(msg->len & 0xff));

	}
	else
	{
		/* write user data to transmit buffer in ntrx chip */
		NTRXSetIndexReg(3);
		NTRXSPIWrite((uint8_t)(NA_RamTxBuffer_O & 0xff), msg->data, (uint8_t)(msg->len & 0xff));
	}

	NTRXSetIndexReg(0);

	memcpy(cacheAddr, msg->addr, 6);
	NTRXSPIWrite(NA_RamTxDstAddr_O, cacheAddr, 6);

	/*switch frametype: unicast / broadcast*/
	if (((memcmp(brdcAddr, msg->addr, 6) == 0) && (phyPIB.frameType == PHY_FRAME_TYPE_DATA)))
	{
		last_frameType = phyPIB.frameType; /* backup of the frame type */
		phyPIB.frameType = PHY_FRAME_TYPE_BRDC;
		ntrxShadowReg[NA_TxPacketType_O] &= (uint8_t)(~(0x0F));
		ntrxShadowReg[NA_TxPacketType_O] |= phyPIB.frameType;
		NTRXSPIWriteByte(NA_TxPacketType_O, ntrxShadowReg[NA_TxPacketType_O]);

		DEBUG_PHY("frame type : BRDC\n");

		ntrxShadowReg[NA_TxArq_O] &= (uint8_t)(~(1 << NA_TxArq_B));
		NTRXSPIWriteByte(NA_TxArq_O, ntrxShadowReg[NA_TxArq_O]);
	}

	/* TODO DEBUG REMOVE */
	DEBUG_PHY("send : ");
	{
		uint8_t o;
		for (o = 0; o < msg->len; o++)
		{
			DEBUG_PHY("0x%X ", (uint16_t) msg->data[o]);
		}DEBUG_PHY("\n");
	}

	/* TODO DEBUG REMOVE */
	DEBUG_PHY("dest MAC : ");
	{
		uint8_t a;
		for (a = 0; a < sizeof(AddrT); a++)
		{
			DEBUG_PHY("0x%X ", (uint16_t) msg->addr[a]);
		}DEBUG_PHY("\n");
	}

	/* packet length max. CONFIG_MAX_PACKET_SIZE byte */
	txLen[0] = (uint8_t) ((msg->len > CONFIG_MAX_PACKET_SIZE) ? CONFIG_MAX_PACKET_SIZE : msg->len);
	/* headerbits for identify data or rangingpackets (xxx- ----) */
	txLen[1] = (uint8_t) ((msg->attribute << 5) & 0xE0);

	NTRXSPIWrite(NA_RamTxLength_O, txLen, 2);
	ntrxState = TxSEND;

	/* reset bit 1,2 */
	ntrxShadowReg[NA_TxPhCarrSenseMode_O] &= (uint8_t)(~(0x03 << NA_TxPhCarrSenseMode_LSB));

	/* set bit 1,2 with CAMode */
	ntrxShadowReg[NA_TxPhCarrSenseMode_O] |= (uint8_t)(phyPIB.CAMode << NA_TxPhCarrSenseMode_LSB);

	/* write to nanoLOC */
	NTRXSPIWriteByte(NA_TxPhCarrSenseMode_O, ntrxShadowReg[NA_TxPhCarrSenseMode_O]);

	if (phyPIB.CAMode != PHY_CCA_OFF)
	{
		DEBUG_PHY("CSMA enabled\n");
		phyPIB.titxstart = hwclock();
	}

	/* mark buffers as valid and start transmission */
	NTRXSPIWriteByte(NA_TxBufferCmd_O, (1 << NA_TxCmdStart_B) | (0x03 << NA_TxBufferCmd_LSB));

	txSendMsg = msg;

	do
	{
		PHYPoll(1);
		if (hwclock() > (phyPIB.titxstart + 4 * TO_CSMA))
			ntrxState = TxIDLE;
	} while (ntrxState != TxIDLE);
}

/**
 * @brief This function processes all data request.
 * @param *msg this is the message pointer
 *
 * This function represents the service access point for data requests and
 * ranging requests. When the primitive is set to @em PD_DATA_REQUEST all
 * necessary actions are taken to copy the payload to the nanoLOC chip
 * and set the MAC header parameter in the corresponding registers.
 * When all this is done the transmission will be initiated.
 * In case of a transmission in progress, @em PDSap will reject the message
 * and indicate a failure with the status @em PHY_BUSY_TX.
 *
 */
/****************************************************************************/
int PDSap(MsgT *msg)
/****************************************************************************/
{
	int ret  = PHY_BUSY;											// return with no error. 
	
	/* if ntrx is in sleep mode, dont use ntrx */
	if (phyPIB.pwrDown == TRUE)
		return PHY_FORCE_TRX_OFF;									// add returns 

	/*
	 * check the message length. If the message length is bigger than
	 * the allowed buffer size the packet will be rejected.
	 */
	if (msg->len > CONFIG_MAX_PACKET_SIZE)
	{
		DEBUG_PHY("packet size > %d\n", CONFIG_MAX_PACKET_SIZE);
		return PHY_INVALID_PARAMETER;								// add returns
	}

	switch (msg->prim)
	{
	case PD_DATA_REQUEST:
		/* transmitter still busy */
		if (txSendMsg != NULL)
		{
			DEBUG_PHY("busy tx\n");
			return PHY_BUSY_TX;										// add returns
		}

		if (msg->len == 0)
		{
			DEBUG_PHY("packet length <= 0\n");
			return PHY_INVALID_PARAMETER;
		}

		msg->attribute = MSG_TYPE_DATA;
		NTRXSendPacket(msg);
		break;

	case PD_RANGING_REQUEST:

		ret = NTRXRangingRequest(msg);
		break;

	default:
		break;
	}
	return ret;														// add call return values. 
}

/**
 * @brief Helper function to switch off test modes.
 *
 * This function switches off a previously enabled test mode.
 * Note: Only one test mode at a time can be enabled.
 */
/****************************************************************************/
void TestmodeOff(void)
/****************************************************************************/
{
	switch (phyPIB.testmode)
	{
	case PHY_CONTINUOUS_MODE:
		NTRXSetTestChirpMode(FALSE);
		break;

	case PHY_CARRIER_MODE:
		NTRXSetTestCarrierMode(FALSE);
		break;

	default:
		break;
	}
}

/**
 * @brief Physical layer management entety
 * @param *msg this is the the message struct
 *
 * This function represents the service access point to configure the
 * physical layer. All configuration parameters are stored in the physical layer
 * parameter information base @em phyPIB. This function hides all necessarey steps
 * to enable or disable certain features. With this function it is possible to set values but
 * also to query the following settings:
 *
 * @ref PHY_CURRENT_CHANNEL : the channel is a transmission mode defined in PHY_TRX_MODE
 * @ref PHY_TRX_MODE : a set of three parameters bandwidth, symbol duration and symbol rate
 * @ref PHY_LOG_CHANNEL : is a predefined group of chip settings
 * @ref PHY_TX_POWER : output power setting [ 0 - 63 ]
 * @ref PHY_ARQ : hardware acknowledgement
 * @ref PHY_FEC : forward error correction
 * @ref PHY_MAC_ADDRESS1 : 1st MAC address for address matching
 * @ref PHY_MAC_ADDRESS2 : 2nd MAC address for address matching
 * @ref PHY_TX_ADDR_SELECT : select the MAC address as sender address
 * @ref PHY_ADDR_MATCHING : address matching or promiscuous mode
 * @ref PHY_PWR_DOWN_MODE : set operational mode
 * @ref PHY_RECALIBRATION : recalibration interval
 * @ref PHY_FRAME_TYPE : set frame types that will be handled by the nanoLOC chip
 */
/****************************************************************************/
void PLMESap(MsgT *msg)
/****************************************************************************/
{
	uint8_t bw = 0, sd = 0, br = 0, cf = 0, tuned = 0;

	/* if ntrx is in sleep mode, dont use ntrx */
	if (phyPIB.pwrDown == TRUE)
		return;

	switch (msg->prim)
	{
	case PLME_GET_REQUEST:
		msg->prim = PLME_GET_CONFIRM;
		msg->status = PHY_SUCCESS;
		switch (msg->attribute)
		{
		case PHY_CURRENT_CHANNEL:
			msg->value = phyPIB.currentChannel;
			break;

		case PHY_TRX_MODE:
			msg->value = phyPIB.trxMode;
			break;

		case PHY_LOG_CHANNEL:
			msg->value = phyPIB.logChannel;
			break;

		case PHY_TX_POWER:
			msg->value = phyPIB.txPower;
			break;

		case PHY_ARQ:
			msg->value = phyPIB.arqMode;
			break;

		case PHY_ARQ_MAX:
			msg->value = phyPIB.arqMax;
			break;

		case PHY_FEC:
			msg->value = phyPIB.fec;
			break;

		case PHY_MAC_ADDRESS1:
			memcpy(msg->data, phyPIB.macAddr0, 6);
			break;

		case PHY_MAC_ADDRESS2:
			memcpy(msg->data, phyPIB.macAddr0, 6);
			break;

		case PHY_TX_ADDR_SELECT:
			msg->value = phyPIB.txAddrSel;
			break;

		case PHY_ADDR_MATCHING:
			msg->value = phyPIB.addrMatch;
			break;

		case PHY_PWR_DOWN_MODE:
			msg->value = phyPIB.pwrDownMode;
			break;

		case PHY_RECALIBRATION:
			msg->value = (uint16_t) (phyPIB.lastRecalibration / 1000);
			break;

		case PHY_RECAL_INTERVAL:
			msg->value = (uint16_t) (phyPIB.recalInterval / 1000);
			break;

		case PHY_FRAME_TYPE:
			msg->value = phyPIB.frameType;
			break;

		case PHY_TESTMODE:
			msg->value = phyPIB.testmode;
			break;

		case PHY_RX_STATE:
			msg->value = phyPIB.rxState;
			break;

		case PHY_RSSI:
			msg->value = NTRXGetRssi();
			break;

		default:
			msg->status = PHY_UNSUPPORTED_ATTRIBUTE;
			break;
		}
		msg->pdu = msg->data;
		SendCfgUp(msg);
		break;

	case PLME_SET_REQUEST:
		msg->prim = PLME_SET_CONFIRM;
		msg->status = PHY_SUCCESS;
		switch (msg->attribute)
		{
		case PHY_CCA_MODE:
			if (msg->value > 3)
			{
				msg->status = PHY_INVALID_PARAMETER;
				phyPIB.CAMode = PHY_CCA_OFF;
			}
			else
			{
				phyPIB.CAMode = (uint8_t) (msg->value);
			}
			break;
		case PHY_CURRENT_CHANNEL:
			if (msg->value > 16)
			{
				msg->status = PHY_INVALID_PARAMETER;
			}
			else
			{
				phyPIB.currentChannel = (uint8_t) (msg->value);
				switch (phyPIB.currentChannel)
				{
				case 0:
					bw = NA_80MHz;
					sd = NA_1us;
					br = NA_1M_S;
					break;
				default:
					bw = NA_22MHz;
					sd = NA_4us;
					br = NA_250k_S;
					break;
				}
				NTRXSetChannel(phyPIB.currentChannel);
				NTRXSetupTrxMode(bw, sd, br);
			}
			break;

		case PHY_LOG_CHANNEL:
			if (msg->value > CONFIG_MAX_LOG_CHANNEL - 1)
			{
				msg->status = PHY_INVALID_PARAMETER;
			}
			else
			{
				phyPIB.logChannel = (uint8_t) (msg->value);
				switch (phyPIB.logChannel)
				{
				case 1:
					bw = NA_22MHz;
					sd = NA_4us;
					br = NA_250k_S;
					cf = 1;
					ntrxShadowReg[NA_UseFec_O] &= (uint8_t) (~(1 << NA_UseFec_B));
					break;

				case 2:
					bw = NA_22MHz;
					sd = NA_4us;
					br = NA_250k_S;
					cf = 7;
					ntrxShadowReg[NA_UseFec_O] &= (uint8_t) (~(1 << NA_UseFec_B));
					break;

				case 3:
					bw = NA_22MHz;
					sd = NA_4us;
					br = NA_250k_S;
					cf = 13;
					ntrxShadowReg[NA_UseFec_O] &= (uint8_t) (~(1 << NA_UseFec_B));
					break;

				default:
					bw = NA_80MHz;
					sd = NA_1us;
					br = NA_1M_S;
					cf = 0;
					ntrxShadowReg[NA_UseFec_O] &= (uint8_t) (~(1 << NA_UseFec_B));
					break;
				}
				NTRXSPIWriteByte(NA_UseFec_O, ntrxShadowReg[NA_UseFec_O]);
				NTRXSetChannel(cf);
				NTRXSetupTrxMode(bw, sd, br);
			}
			break;

		case PHY_TRX_MODE:
			phyPIB.trxMode = (uint8_t) (msg->value);
			switch (msg->value)
			{
#																		ifdef CONFIG_NTRX_80MHZ_500NS
			case 10:
				bw = NA_80MHz;
				sd = NA_500ns;
				br = NA_125k_S;
				break;
			case 11:
				bw = NA_80MHz;
				sd = NA_500ns;
				br = NA_1M_S;
				break;
			case 12:
				bw = NA_80MHz;
				sd = NA_500ns;
				br = NA_2M_S;
				break;
#																		ifdef DEVKIT201_CHANGE
			case 13:
				bw = NA_80MHz;
				sd = NA_500ns;
				br = NA_125k_S;
				tuned = 1;
				break;
			case 14:
				bw = NA_80MHz;
				sd = NA_500ns;
				br = NA_1M_S;
				tuned = 1;
				break;
			case 15:
				bw = NA_80MHz;
				sd = NA_500ns;
				br = NA_2M_S;
				tuned = 1;
				break;
#																		endif /* DEVKIT201_CHANGE */
#																		endif /* CONFIG_NTRX_80MHZ_500NS */

#																		ifdef CONFIG_NTRX_80MHZ_1000NS
			case 20:
				bw = NA_80MHz;
				sd = NA_1us;
				br = NA_500k_S;
				break;
			case 21:
				bw = NA_80MHz;
				sd = NA_1us;
				br = NA_1M_S;
				break;
#																		ifdef DEVKIT201_CHANGE
			case 22:
				bw = NA_80MHz;
				sd = NA_1us;
				br = NA_500k_S;
				tuned = 1;
				break;
			case 23:
				bw = NA_80MHz;
				sd = NA_1us;
				br = NA_1M_S;
				tuned = 1;
				break;
#																		endif /* DEVKIT201_CHANGE */
#																		endif /* CONFIG_NTRX_80MHZ_1000NS */

#																		ifdef CONFIG_NTRX_80MHZ_2000NS
			case 30:
				bw = NA_80MHz;
				sd = NA_2us;
				br = NA_125k_S;
				break;
			case 31:
				bw = NA_80MHz;
				sd = NA_2us;
				br = NA_250k_S;
				break;
			case 32:
				bw = NA_80MHz;
				sd = NA_2us;
				br = NA_500k_S;
				break;
#																		ifdef DEVKIT201_CHANGE
			case 33:
				bw = NA_80MHz;
				sd = NA_2us;
				br = NA_125k_S;
				tuned = 1;
				break;
			case 34:
				bw = NA_80MHz;
				sd = NA_2us;
				br = NA_250k_S;
				tuned = 1;
				break;
			case 35:
				bw = NA_80MHz;
				sd = NA_2us;
				br = NA_500k_S;
				tuned = 1;
				break;
#																		endif /* DEVKIT201_CHANGE */
#																		endif /* CONFIG_NTRX_80MHZ_2000NS */

#																		ifdef CONFIG_NTRX_80MHZ_4000NS
			case 40:
				bw = NA_80MHz;
				sd = NA_4us;
				br = NA_125k_S;
				break;
			case 41:
				bw = NA_80MHz;
				sd = NA_4us;
				br = NA_250k_S;
				break;
#																		endif /* CONFIG_NTRX_80MHZ_4000NS */

#																		ifdef CONFIG_NTRX_22MHZ_1000NS
			case 50:
				bw = NA_22MHz;
				sd = NA_1us;
				br = NA_1M_S;
				break;
#																		endif /* CONFIG_NTRX_22MHZ_1000NS */

#																		ifdef CONFIG_NTRX_22MHZ_2000NS
			case 60:
				bw = NA_22MHz;
				sd = NA_2us;
				br = NA_125k_S;
				break;
			case 61:
				bw = NA_22MHz;
				sd = NA_2us;
				br = NA_250k_S;
				break;
			case 62:
				bw = NA_22MHz;
				sd = NA_2us;
				br = NA_500k_S;
				break;
#																		endif /* CONFIG_NTRX_22MHZ_2000NS */

#																		ifdef CONFIG_NTRX_22MHZ_4000NS
			case 70:
				bw = NA_22MHz;
				sd = NA_4us;
				br = NA_125k_S;
				break;
			case 71:
				bw = NA_22MHz;
				sd = NA_4us;
				br = NA_250k_S;
				break;
#																		endif /* CONFIG_NTRX_22MHZ_4000NS */
			default:
				DEBUG_PHY ("Valid modes: 10-43 (80Mhz) 50-84 (22Mhz) 90-93 (22HR)\n");
				msg->status = PHY_INVALID_PARAMETER;
				msg->pdu = msg->data;
				SendCfgUp(msg);
				return;
			}
#																	ifdef DEVKIT201_CHANGE
			if (bw == NA_80MHz)
			{
				if (tuned == 0)
				{
					NTRXSetChannel(17);
				}
				else
				{
					NTRXSetChannel(0);
				}
			}
#																	endif
			NTRXSetupTrxMode(bw, sd, br);

			switch (bw)
			{
			case NA_80MHz:
				DEBUG_PHY ("New Mode 80 Mhz ");
				break;
			case NA_22MHz:
				DEBUG_PHY ("New Mode 22 Mhz ");
				break;
			default:
				DEBUG_PHY ("Unknown mode ??? ");
				break;
			}
			switch (sd)
			{
			case NA_500ns:
				DEBUG_PHY ("500 ns, ");
				break;
			case NA_1us:
				DEBUG_PHY ("1 us, ");
				break;
			case NA_2us:
				DEBUG_PHY ("2 us, ");
				break;
			case NA_4us:
				DEBUG_PHY ("4 us, ");
				break;
			default:
				break;
			}
			switch (br)
			{
			case NA_125k_S:
				DEBUG_PHY ("125 kSym\n");
				break;
			case NA_250k_S:
				DEBUG_PHY ("250 kSym\n");
				break;
			case NA_500k_S:
				DEBUG_PHY ("500 kSym\n");
				break;
			case NA_1M_S:
				DEBUG_PHY ("1 MSym\n");
				break;
			case NA_2M_S:
				DEBUG_PHY ("2 MSym\n");
				break;
			default:
				break;
			}
			break;

		case PHY_TX_POWER:
			if (msg->value > 63)
			{
				msg->status = PHY_INVALID_PARAMETER;
			}
			else
			{
				phyPIB.txPower = (uint8_t) ((msg->value) & 0x3f);
				ntrxShadowReg[NA_TxOutputPower0_O] &= (uint8_t) (~(0x3f << NA_TxOutputPower0_LSB));
				ntrxShadowReg[NA_TxOutputPower0_O] |= (uint8_t) (phyPIB.txPower << NA_TxOutputPower0_LSB);
				if (phyPIB.txPower != 0x3f)
				{ /* This way no table for the transmission output power is necessary */
					ntrxShadowReg[NA_TxOutputPower0_O] &= (uint8_t) (~(1 << NA_TxOutputPower0_LSB));
				}
				NTRXSPIWriteByte(NA_TxOutputPower0_O, ntrxShadowReg[NA_TxOutputPower0_O]);

				ntrxShadowReg[NA_TxOutputPower1_O] &= (uint8_t) (~(0x3f << NA_TxOutputPower1_LSB));
				ntrxShadowReg[NA_TxOutputPower1_O] |= (uint8_t) ((phyPIB.txPower << NA_TxOutputPower1_LSB));
				if (phyPIB.txPower != 0x3f)
				{ /* This way no table for the transmission output power is necessary */
					ntrxShadowReg[NA_TxOutputPower1_O] &= (uint8_t) (~(1 << NA_TxOutputPower1_LSB));
				}
				NTRXSPIWriteByte(NA_TxOutputPower1_O, ntrxShadowReg[NA_TxOutputPower1_O]);
			}
			break;

		case PHY_ARQ:
			NTRXRXEnable(FALSE);
			if (msg->value == 0)
			{
				ntrxShadowReg[NA_TxArq_O] &= (uint8_t) (~(1 << NA_TxArq_B));
				ntrxShadowReg[NA_RxArqMode_O] &= (uint8_t)(~((0x03 << NA_RxArqMode_LSB) | (1 << NA_RxCrc2Mode_B)));
			}
			else
			{
				ntrxShadowReg[NA_TxArq_O] |= (1 << NA_TxArq_B);
				ntrxShadowReg[NA_RxArqMode_O] |= ((NA_RxArqModeCrc2_VC_C << NA_RxArqMode_LSB) | (NA_RxCrc2ModeTrigOn_BC_C << NA_RxCrc2Mode_B));
			}
			NTRXSPIWriteByte (NA_TxArq_O, ntrxShadowReg[NA_TxArq_O]);
			NTRXSPIWriteByte (NA_RxArqMode_O, ntrxShadowReg[NA_RxArqMode_O]);
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
			break;

		case PHY_ARQ_MAX:
			if (msg->value > 14)
			{
				msg->status = PHY_INVALID_PARAMETER;
			}
			else
			{
				NTRXRXEnable(FALSE);
				phyPIB.arqMax = (uint8_t)(msg->value);
				if (phyPIB.arqMax == 0)
				{
					ntrxShadowReg[NA_TxArqMax_O] &= (uint8_t)( ~( 0x0f << NA_TxArqMax_LSB ));
				}
				else
				{
					ntrxShadowReg[NA_TxArqMax_O] &= (uint8_t)( ~(0x0f << NA_TxArqMax_LSB ));
					ntrxShadowReg[NA_TxArqMax_O] |= ((uint8_t)(msg->value << NA_TxArqMax_LSB));
				}
				NTRXSPIWriteByte (NA_TxArqMax_O, ntrxShadowReg[NA_TxArqMax_O]);
				if (phyPIB.rxState == PHY_RX_ON)
				{
					NTRXRXEnable(TRUE);
				}
			}
			break;

			case PHY_FEC:
			NTRXRXEnable(FALSE);
			if (msg->value == TRUE)
			{
				ntrxShadowReg[NA_UseFec_O] |= (1 << NA_UseFec_B);
			}
			else
			{
				ntrxShadowReg[NA_UseFec_O] &= (uint8_t)( ~(1 << NA_UseFec_B));
			}
			NTRXSPIWriteByte (NA_UseFec_O, ntrxShadowReg[NA_UseFec_O]);

			phyPIB.fec = (uint8_t)( msg->value );
			modeSet.fec = (uint8_t)( msg->value );

			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
			break;

			case PHY_MAC_ADDRESS1:
			NTRXRXEnable(FALSE);
			memcpy (phyPIB.macAddr0, msg->data, 6);
			NTRXSetIndexReg (0);
			NTRXSPIWrite (NA_RamStaAddr0_O, msg->data , 6);
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
			break;

			case PHY_MAC_ADDRESS2:
			NTRXRXEnable(FALSE);
			memcpy (phyPIB.macAddr1, msg->data, 6);
			NTRXSetIndexReg (0);
			NTRXSPIWrite (NA_RamStaAddr1_O, msg->data, 6);
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
			break;

			case PHY_TX_ADDR_SELECT:
			NTRXRXEnable(FALSE);
			phyPIB.txAddrSel = (uint8_t)(msg->value);
			if (msg->value == 0)
			{
				ntrxShadowReg[NA_TxAddrSlct_O] &= (uint8_t)( ~(1 << NA_TxAddrSlct_B ));
			}
			else
			{
				ntrxShadowReg[NA_TxAddrSlct_O] |= (1 << NA_TxAddrSlct_B);
			}
			NTRXSPIWriteByte (NA_TxAddrSlct_O, ntrxShadowReg[NA_TxAddrSlct_O]);
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
			break;

			case PHY_ADDR_MATCHING:
			NTRXRXEnable(FALSE);
			phyPIB.addrMatch = (uint8_t)(msg->value);
			if (msg->value == 0)
			{
				/* promiscuous mode */
				ntrxShadowReg[NA_RxAddrMode_O] &= (uint8_t)( ~( 1 << NA_RxAddrMode_B));
			}
			else
			{
				ntrxShadowReg[NA_RxAddrMode_O] |= (1 << NA_RxAddrMode_B);
			}
			NTRXSPIWriteByte (NA_RxAddrMode_O, ntrxShadowReg[NA_RxAddrMode_O]);
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
			break;

			case PHY_RECALIBRATION:
			if (NTRXAllCalibration ())
			{
				phyPIB.lastRecalibration = hwclock();
				tiRecal = hwclock() + phyPIB.recalInterval;
				rcwd = 0;
			}
			else
			{
				msg->status = PHY_BUSY_RX;
			}
			break;

			case PHY_RECAL_INTERVAL: /* delay time for recalibration */
			phyPIB.recalInterval = msg->value * 1000;
			break;
		case PHY_PWR_DOWN_MODE:
			if (msg->value > 1)
			{
				msg->status = PHY_INVALID_PARAMETER;
			}
			else
			{

				DEBUG_PHY("PowerDown : %ld\n", *(uint32_t*)(msg->data));
				trxPollMode = (bool_t)((phyPIB.recalInterval == 0) ? FALSE : TRUE);
				phyPIB.pwrDownMode = (uint8_t)( msg->value );
				NTRXResetSettings ();
				NTRXPowerdownMode ((uint8_t)( msg->value ), *(uint32_t*)(msg->data));
				memset( cacheAddr, 0, 6 );
				phyPIB.pwrDown = TRUE;
			}
			break;
		case PHY_POWERDOWN_PAD_MODE:
			if (msg->value == 0)
			{
				NTRXPowerupFromPadMode();
				trxPollMode = TRUE;
				phyPIB.pwrUp = FALSE;
				phyPIB.pwrDown = FALSE;
				if (phyPIB.rxState == PHY_RX_ON)
				{
					NTRXRXEnable(TRUE);
				}
			}
			else
			{
				// do not poll nanoLOC while in sleep mode
				NTRXPowerdownModeDIO( 1, (uint8_t)( msg->value - 1 ));
				trxPollMode = FALSE;
				phyPIB.pwrDown = TRUE;
			}
			break;

		case PHY_FRAME_TYPE:
			phyPIB.frameType = (uint8_t)(msg->value & 0x07);
			ntrxShadowReg[NA_RxDataEn_O] = (uint8_t)( ~( 0x07 ));
			ntrxShadowReg[NA_RxDataEn_O] |= phyPIB.frameType;
			NTRXSPIWriteByte (NA_RxDataEn_O , ntrxShadowReg[NA_RxDataEn_O]);
			break;

		case PHY_TESTMODE:
			if (msg->value > 2)
			{
				msg->status = PHY_INVALID_PARAMETER;
			}
			else
			{
				switch (msg->value)
				{
				case 1:
					TestmodeOff();
					NTRXSetTestChirpMode (TRUE);
					/* this blocks the transmission path of the PDSap */
					txSendMsg = &upMsg;
					break;

				case 2:
					TestmodeOff();
					NTRXSetTestCarrierMode (TRUE);
					/* this blocks the transmission path of the PDSap */
					txSendMsg = &upMsg;
					break;

					default:/* this enables the transmission path of the PDSap */
					TestmodeOff ();
					txSendMsg = NULL;
				}

				phyPIB.testmode = (uint8_t)( msg->value );

			}
			break;

			case PHY_RX_CMD:
			switch (msg->value)
			{
				case PHY_TRX_OFF:
				case PHY_RX_ON:
				phyPIB.rxState = (uint8_t)( msg->value );
				/*stop the receiver and clear the buffers */
				NTRXRXEnable(FALSE);
				/* clear interrupts */
				NTRXSPIWriteByte (NA_RxIntsReset_O, 0x7F);
				rxIrq = 0;
#																							ifdef CONFIG_NTRX_SNIFFER
						upMsg.count=0;
#																							endif
						if (msg->value == PHY_RX_ON)
						{
							NTRXRXEnable(TRUE);
						}

						break;

			default:
				msg->status = PHY_INVALID_PARAMETER;
						break;
					}
					break;

		case PHY_RX_STATE:
			switch (msg->value)
					{
						case PHY_RX_ON:
			case PHY_TRX_OFF:
				phyPIB.rxState = (uint8_t)(msg->value);
						break;

			default:
				msg->status = PHY_INVALID_PARAMETER;
						break;
					}
					break;

					case PHY_SYNCWORD:

					NTRXRXEnable(FALSE);
					memcpy (phyPIB.syncword, (uint8_t *)(msg->data), 8);
					NTRXSetSyncWord(phyPIB.syncword);
					if (phyPIB.rxState == PHY_RX_ON)
					{
						NTRXRXEnable(TRUE);
					}
					break;

		default:
			msg->status = PHY_UNSUPPORTED_ATTRIBUTE;
					break;
				}
				msg->pdu = msg->data;
				break;

	default:
		break;
			}
			/* ieee config stuff */
		}

				/**
				 * @brief Callback function for received messages
				 *
				 * This function reads out the payload of a received message and
				 * calls the upper layer/application
				 *
				 */
				/****************************************************************************/
void PDCallback(void)
/****************************************************************************/
{
	uint8_t status;
	uint8_t reg[2];
	uint8_t header_bits;

	/*
	 * use 1 led to indicate message reception on the devBoard
	 * The led will stay on for a 50 ms.
	 */
#   ifdef CONFIG_TRAFFIC_LED
	TRIGGER_LED_RX ();
#   endif /* CONFIG_TRAFFIC_LED */

	/* on packet reception the rx is switched off automaticly from hardware */
	phyPIB.rxOn = FALSE;

	/*
	 * read the crc2 status register
	 */
	NTRXSPIReadByte(NA_RxCrc2Stat_O, &status);
	rxIrq = 0;

	/* check if data is valid */
	if ((status & (1 << NA_RxCrc2Stat_B)) != 0)
	{
		NTRXSetIndexReg(0);
		/* read source address */
		NTRXSPIRead(NA_RamRxSrcAddr_O, upMsg.addr, 6);
		/* read length plus additionl bits */
		NTRXSPIRead(NA_RamRxLength_O, reg, 2);

		header_bits = (uint8_t) (reg[1] >> 5);

		/* read destination address */
#		ifdef CONFIG_NTRX_SNIFFER
		NTRXSPIRead (NA_RamRxDstAddr_O, upMsg.rxAddr, 6);
		upMsg.count++;
		upMsg.extBits = header_bits;
		upMsg.frameType = (status & 0x0f);
#		endif

		upMsg.len = reg[0];
		if (upMsg.len > CONFIG_MAX_PACKET_SIZE)
		{
			/* restart receiver */
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}
		}
		else
		{

			if (buffSwapped == TRUE)
			{
				buffSwapped = FALSE;
				/* SWAP BUFFER for receive*/
				ntrxShadowReg[NA_SwapBbBuffers_O] &= (uint8_t) (~(1 << NA_SwapBbBuffers_B));
				NTRXSPIWriteByte(NA_SwapBbBuffers_O, ntrxShadowReg[NA_SwapBbBuffers_O]);

				NTRXSetIndexReg(3);
			}
			else
			{
				buffSwapped = TRUE;
				/* SWAP BUFFER for receive*/
				ntrxShadowReg[NA_SwapBbBuffers_O] |= (1 << NA_SwapBbBuffers_B);
				NTRXSPIWriteByte(NA_SwapBbBuffers_O, ntrxShadowReg[NA_SwapBbBuffers_O]);

				NTRXSetIndexReg(2);
			}

			tiPhyRxTimeout_once = FALSE;
			tiPhyRxTimeout = hwclock() + phyPIB.phyRxTimeout;

			/*
			 * restart receiver and than read rx buffer. This is ok because we use
			 * buffer swapping.
			 */
			if (phyPIB.rxState == PHY_RX_ON)
			{
				NTRXRXEnable(TRUE);
			}

			NTRXSPIRead((uint8_t) (NA_RamRxBuffer_O & 0xFF), upMsg.data, upMsg.len);
			NTRXSetIndexReg(0);

#			ifdef CONFIG_NTRX_SNIFFER
			upMsg.value = 0xff;
			upMsg.prim = PD_DATA_INDICATION;
			upMsg.pdu = upMsg.data;
			SendMsgUp (&upMsg);
#			else

			/*
			 * if address matching off, ignore rangingstates
			 * this path is used for normal data reception
			 * ranging is handled in the else path
			 */
			if (((ntrxShadowReg[NA_RxAddrMode_O] & (1 << NA_RxAddrMode_B))== 0)||(header_bits == MSG_TYPE_DATA))
			{
				upMsg.value = 0xff;
				upMsg.prim = PD_DATA_INDICATION;
				upMsg.pdu = upMsg.data;
				SendMsgUp (&upMsg);
			}
			else if (header_bits == MSG_TYPE_RANGING)
			{
				NTRXRangingRX(&upMsg);
			}
#			endif /* CONFIG_NTRX_SNIFFER */
		}
	}
	else
	{
		/* restart receiver */
		if (phyPIB.rxState == PHY_RX_ON)
		{
			NTRXRXEnable(TRUE);
		}
	}
}

			/**
			 * @brief polls the nanoLOC chip for incomming messages
			 *
			 * This function is the main part of the the physical layer. It processes
			 * all interrupt flags and supervises the periodic recalibration.
			 *
			 */
			/***************************************************************************/
int error_count = 0;
void PHYPoll(int sending)
/***************************************************************************/
{
	uint8_t arqCount;
	MsgT *tMsg;

	/* if ntrx is in sleep mode, dont use ntrx */
	if (phyPIB.pwrDown == TRUE)
		return;

	if ((phyPIB.CAMode != PHY_CCA_OFF) && (ntrxState == TxSEND) && (hwclock() > phyPIB.titxstart + TO_CSMA))
	{
		DEBUG_PHY("CSMA Timeout\n");
		error_count++;
		
		/// Kobus - Just check and make sure TX is still busy...
		NTRXInterrupt();
		
		if(ntrxState == TxSEND)
		{
			/* cancel transmission */
			NTRXSPIWriteByte(NA_TxBufferCmd_O, (1 << NA_TxCmdStop_B));
	
			/// Rather just stop. forceful sending seldom seems to help. Kobus. 
			ntrxState = TxIDLE;
			txSendMsg = NULL;
		}
		else
			error_count++;

	}

#	ifdef CONFIG_NTRX_IRQ
	//cli();
	//nnIrq = 0;
	//xSemaphoreTake(nnIrq, 0);
	//sei();
#	endif /* CONFIG_NTRX_IRQ */

	if (tiPhyRxTimeout < hwclock() && tiPhyRxTimeout_once == FALSE)
	{
		tiPhyRxTimeout_once = TRUE;

		upMsg.prim = PD_STATUS_INDICATION;
		upMsg.attribute = PHY_RX_TIMEOUT;
		upMsg.pdu = upMsg.data;
		SendMsgUp(&upMsg);
	}

#	ifndef CONFIG_NTRX_IRQ
	if (trxPollMode == TRUE)
	{
		NTRXInterrupt();
	}
#	endif /* CONFIG_NTRX_IRQ */

	if (ntrxState == TxWAIT)
	{
		if (txSendMsg != NULL)					// check for null pointer to avoid hardfault)
		{
			/*
			 * get number of transmissions needed to last message
			 * This information is used to determine if the last
			 * transmission was successful or failed.
			 */
			NTRXSPIReadByte(NA_TxArqCnt_O, &arqCount);
			arqCount &= 0x0F;

			if (arqCount > phyPIB.arqMax)
			{

#	   		ifdef CONFIG_NTRX_AUTO_RECALIB
				if(( ++rcwd > 3 ) && ( phyPIB.recalInterval != 0 ))
				{
					/* INFO: If the TRX sends a packet, calibration fails!
					 * In this case rcwd is not reset, but tiRecal is.
					 */
					/* normal operation mode */
					if (phyPIB.testmode == 0)
					{
						if (NTRXAllCalibration ())
						{
							tiRecal = hwclock() + phyPIB.recalInterval;
							rcwd = 0;
							//TRIGGER_LED_CAL();
						}
					}
					else
					{
						/*
						 * in case of an enabled testmode recalibration is a bit trickier.
						 * We first have to disable the testmode, recalibrate and then
						 * enable the testmode again.
						 */

						if (phyPIB.testmode == 1)
						{
							NTRXSetTestChirpMode (FALSE);
							NTRXAllCalibration ();
							//TRIGGER_LED_CAL();
							NTRXSetTestChirpMode (TRUE);
						}
						else
						{
							NTRXSetTestCarrierMode (FALSE);
							NTRXAllCalibration ();
							//TRIGGER_LED_CAL();
							NTRXSetTestCarrierMode (TRUE);
						}
					}
				}
#				endif
				txSendMsg->status = PHY_NO_ACK;
			}
			else
			{
				txSendMsg->status = PHY_SUCCESS;
			}

			//////////////////////////////////////////////////////////////////////////
			txIrq &= (uint8_t) (~(0x01 << NA_TxEnd_B));
			ntrxState = TxIDLE;
			txSendMsg->value = arqCount;
			//////////////////////////////////////////////////////////////////////////

			if (phyPIB.frameType != last_frameType)
			{
				phyPIB.frameType = last_frameType;
				ntrxShadowReg[NA_TxPacketType_O] &= (uint8_t) (~(0x0F));
				ntrxShadowReg[NA_TxPacketType_O] |= phyPIB.frameType;
				NTRXSPIWriteByte(NA_TxPacketType_O, ntrxShadowReg[NA_TxPacketType_O]);

				DEBUG_PHY("frame type restore\n");

				ntrxShadowReg[NA_TxArq_O] |= (1 << NA_TxArq_B);
				NTRXSPIWriteByte(NA_TxArq_O, ntrxShadowReg[NA_TxArq_O]);
			}

			switch (txSendMsg->prim)
			{
			case PD_RANGING_REQUEST:
				NTRXRangingACK();

				break;
			default:
				txSendMsg->prim = PD_DATA_CONFIRM;

				tMsg = txSendMsg;
				txSendMsg = NULL;
				tMsg->pdu = tMsg->data;
				SendMsgUp(tMsg);
				break;
			}
		}
		else
		{
			HWDelayus(200);
			txIrq &= (uint8_t) (~(0x01 << NA_TxEnd_B));
			ntrxState = TxIDLE;
			error_count--;
			if (!NTRXRangingIsIDLE())
				NTRXRangingInit();			// reset the ranging module if it was the one active...
		}
	}

	/* check if nanoNET TRX chip has received valid data */
	if (ntrxState == TxIDLE && !sending) ///////////// this was changed. prevents a read before sending completes/////////////////
	{
		if ((rxIrq & (0x01 << NA_RxEnd_B)) != 0)
		{
#			ifdef CONFIG_DEFAULT_TRX_80MHZ_4000NS
//			HWDelayus(300); /* make sure that hwack is transmitted before PDCallback */
#			endif
			PDCallback();
		}
	}

#   ifdef CONFIG_NTRX_AUTO_RECALIB
	if (phyPIB.recalInterval != 0)
	{
		if ((rcwd > 3) || (tiRecal < hwclock()))
		{
			/* INFO: If the TRX sends a packet, calibration failes!
			 * In this case rcwd is not reset, but tiRecal is. */
			if (phyPIB.testmode == 0)
			{
				if (NTRXAllCalibration ())
				{
					tiRecal = hwclock() + phyPIB.recalInterval;
					rcwd = 0;
					//TRIGGER_LED_CAL();
				}
			}
			else
			{
				/*
				 * in case of an enabled testmode recalibration is a bit trickier.
				 * We first have to disable the testmode, recalibrate and then
				 * enable the testmode again.
				 */
				if (phyPIB.testmode == 1)
				{
					NTRXSetTestChirpMode (FALSE);
					NTRXAllCalibration ();
					//TRIGGER_LED_CAL();
					NTRXSetTestChirpMode (TRUE);
				}
				else
				{
					NTRXSetTestCarrierMode (FALSE);
					NTRXAllCalibration ();
					//TRIGGER_LED_CAL();
					NTRXSetTestCarrierMode (TRUE);
				}
			}
		}
	}
#   endif /* CONFIG_NTRX_AUTO_RECALIB */

}

/**
 * @brief interrupt service routine for the nanoLOC chip
 *
 * This function is an interrupt service routine of the nanochip.
 * It updates the TX and RX status flags.
 *
 */
/**************************************************************************/
void NTRXInterrupt(void)
/**************************************************************************/
{
	/* if ntrx is in sleep mode, dont use ntrx */
	if (phyPIB.pwrDown == TRUE)
		return;

	/* we have received an interrupt and need to find out what caused it */
	NTRXSPIRead(NA_TxIntsRawStat_O, ntrxIrqStatus, 2);

	/* check if it was the transmitter */
	if (txIrqStatus!= 0)
	{
		/* clear interrupts */
		NTRXSPIWriteByte (NA_TxIntsReset_O, txIrqStatus);
		txIrq |= (uint8_t)( txIrqStatus & (0x01 << NA_TxEnd_B));
		if ((txIrq & (0x01 << NA_TxEnd_B)) != 0)
		{
			ntrxState = TxWAIT;
			if(txSendMsg == NULL)
			{
				HWDelayus(1);
				ntrxState = TxIDLE;
			}

#			ifdef CONFIG_NTRX_IRQ
//			nnIrq = 1;
//			xQueueSendFromISR(Q_nnIrq, &NI, &priority);
#			endif /* CONFIG_NTRX_IRQ */
		}
	}
	/* check if it was the receiver */
	if (rxIrqStatus!= 0)
	{
		/* clear interrupts */
		NTRXSPIWriteByte (NA_RxIntsReset_O, rxIrqStatus);
		rxIrq |= (uint8_t)( rxIrqStatus & (0x01 << NA_RxEnd_B));
#		ifdef CONFIG_NTRX_IRQ
		
//			nnIrq = 1;
//			xQueueSendFromISR(Q_nnIrq, &NI, &priority);
#		endif /* CONFIG_NTRX_IRQ */
		}
	//BASEBAND
	NTRXSPIRead(NA_BbTimerIrqStatus_O, &ntrxBBStatus, 1);
	/* check if it was the basebandtimer */
	if ((ntrxBBStatus & 0x40) != 0)
	{
		NTRXStopBbTimer();
		NTRXRangingInterrupt();

		ntrxBBStatus = 0;
	}
}

/**************************************************************************/
/**
 * @brief Configuration of callback functions.
 * @param cbMsg Callback function pointer for the (protocol) message interface.
 * If not set the PHY will call the function @ref APLCallback by default.
 */
void PHYRegisterCallback(callbackfn_t cbMsg)
{
	if (cbMsg == NULL)
	{
		phySendMsgUp = &APLCallback;
	}
	else
	{
		phySendMsgUp = cbMsg;
	}
}

/**************************************************************************/
/**
 * @brief Returns status of PHY and RANGING.
 * @return TRUE if phy and ranging in status IDLE.
 * Used to check the status of the driver before going power down.
 */
bool_t PHYIsIDLE(void)
{
	if ((ntrxState == TxIDLE) && (NTRXRangingIsIDLE()))
		return TRUE;

	return FALSE;
}

#ifdef CONFIG_NTRX_AUTO_RECALIB
/**************************************************************************/
void PHYAutoCalibration( void )
/**************************************************************************/
{
	if (phyPIB.recalInterval != 0)
	{
		if (tiRecal < hwclock())
		{
			/* INFO: If the TRX sends a packet, calibration failes!
			 * In this case rcwd is not reset, but tiRecal is. */
			if (phyPIB.testmode == 0)
			{
				if (NTRXAllCalibration ())
				{
					tiRecal = hwclock() + phyPIB.recalInterval;
					rcwd = 0;
					//TRIGGER_LED_CAL();
				}
			}
			else
			{
				/*
				 * in case of an enabled testmode recalibration is a bit trickier.
				 * We first have to disable the testmode, recalibrate and then
				 * enable the testmode again.
				 */
				if (phyPIB.testmode == 1)
				{
					NTRXSetTestChirpMode (FALSE);
					NTRXAllCalibration ();
					//TRIGGER_LED_CAL();
					NTRXSetTestChirpMode (TRUE);
				}
				else
				{
					NTRXSetTestCarrierMode (FALSE);
					NTRXAllCalibration ();
					//TRIGGER_LED_CAL();
					NTRXSetTestCarrierMode (TRUE);
				}
			}
		}
	}
}
#endif /* CONFIG_NTRX_AUTO_RECALIB */
