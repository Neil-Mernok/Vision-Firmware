/**
 * @file ntrxranging.c
 * @date 2009-07-07
 * @author Christian Bock
 * @c (C) 2009 Nanotron Technologies
 * @brief Ranging support functions.
 *
 * This file contains the source code for the implementation of the
 * NTRX ranging functions.
 */

#include	"portation.h"
#include	<string.h>
#include	"ntrxtypes.h"
#include	"ntrxranging.h"
#include	"phy.h"
#include	"registermap.h"
#include	"nnspi.h"
#include	"ntrxutil.h"
#include 	<math.h>
#include	<stdio.h> /* FIXME only for debugging */


#define RDBG_len 4096
uint8_t RDBG_buff[RDBG_len];
int RDBG_point = 0;

//#define RANGING_DEBUG	PRINTF
#define RANGING_DEBUG	NoDebugging
//#define RANGING_DEBUG	printf

/* TODO remove me */
//#define RANGE_GET_MEMORY_WORKAROUND
/* ranging state: idle (free for ranging requests) */
#define RANGING_IDLE				0

/* ranging state: running
 * Blocks other ranging requests until timeout or successfull completed ranging)
 */
#define RANGING_RUNNING		1

/* protocol for ranging register values */
#define	PHASEOFFSETACK		0
#define	TOAOFFSETMEANACK_L	1
#define	TOAOFFSETMEANACK_H	2
#define	TXRESPTIME_L		3
#define	TXRESPTIME_H		4
#define	PHASEOFFSETDATA		5
#define	TOAOFFSETMEANDATA_L	6
#define	TOAOFFSETMEANDATA_H	7
#define	RANGING_ERROR		8
#define	COUNTER				9
#define ACTTXID				10
#define CABLE_OFFSET		11				// Kobus: 1 byte for cable offset in 0.1m increments. 

/* length of ranging register values */
#define RANGING_DATA_LEN	12				// Kobus: add one byte for cable offset adjustment. 



/* speed of our signal in AIR or CABLE [um/ps] */
#define SPEED_OF_AIR 299.792458

/* max difference between remote und local measurement
 * - if the difference is > MAX_DIFF_ALLOWED ranging fails */
#define MAX_DIFF_ALLOWED 10

extern MsgT upMsg; /**< for sending packets to upper layer => phy.c */
extern MsgT *txSendMsg; /**< pointer to store last transmitted message. => phy.c */
extern PhyModeSetT modeSet; /**< selects the right ranging const and timeout */
extern callbackfn_t phySendMsgUp; /**< used to send msg to upper layer => phy.c */

/* ranging type whichs able to store all information for one p2p ranging */
typedef struct
{
	uint8_t lrv[RANGING_DATA_LEN]; /**< local ranging values */
	uint8_t rrv[RANGING_DATA_LEN]; /**< remote ranging values */
	uint8_t set1[RANGING_DATA_LEN]; /**< this holds collected data for calculation */
	uint8_t set2[RANGING_DATA_LEN]; /**< this holds collected data for calculation */
	uint8_t errors; /**< this stores occuring errors during ranging */
	AddrT addr; /**< address of remote ranging station */
	uint8_t state; /**< local state of ranging process */
} RangingT;

static RangingMsgT rangingMSG; /**< used to communicate with upper layer */
static RangingT rangingMEM; /**< local ranging memory for 3W_A & 3W_B */
static RangingT *prMEM; /**< pointer to ranging memory */

//static MsgT downRngMsg;  /**< for sending packets over air */
static MsgT recBuffer; /**< buffers recently recieved msg */

/* function pointer in upper layer, called only on remote ranging requests
 * and is used to decline/accept ranging requests */
static permissionfn_t rangingPermission;

/* function pointer in upper layer, called on local or remote ranging requests
 * and only if 2W_PP ranging is chosen. Is used to gather memory
 * from application to store ranging informations */
static mem_assignfn_t rangingGetMemory;

/* counter for occured timeouts */
static uint16_t to_cnt_down = 0;
/* modulo of timeout time */
static uint16_t to_mod = 0;
/* flag which cancel running timeout */
static bool_t to_stop = FALSE;

///// Kobus: added antenna offset value to send to other tag while ranging. 
uint16_t local_antenna_offset = 0;				// dummy 1m antenna cable. 

extern void error_handler(int16_t err);

/**
 * @brief Reads the ack ranging registers after a packet transmission to remote ranging values.
 *
 */
static void NTRXReadRegisterACK(void);

/**
 * @brief Reads the rx ranging registers after a packet reception to local ranging values.
 *
 */
static void NTRXReadRegisterRX(void);

/**
 * @brief Takes a snapshot from local and remote ranging values for later distance calculation.
 *
 */
static void NTRXSet0(void);

/**
 * @brief Takes a snapshot from local and remote ranging values for later distance calculation.
 *
 */
static void NTRXSet1(void);

/**
 * @brief Calculates airtime and subtract all processing delays.
 * @param p is one set of register values
 * @return airtime
 *
 */
static double delay(uint8_t *p);

/**
 * @brief Calculates the distance between two stations.
 * @param p1 is one set of register values
 * @param p2 is one set of register values
 * @return distance in [m]
 *
 */
static double dist(uint8_t *p1, uint8_t *p2);
static double remote_dist(uint8_t *p1);
/**
 * @brief Cancel running timeout.
 *
 */
void NTRXStopTimeout(void);

/**
 * @brief Start timeout.
 * @param to_ms in milliseconds from 1ms up to 200ms.
 */
void NTRXStartTimeout(uint16_t to_ms);

/***************************************************************************/
bool_t NTRXRangingIsIDLE(void)
/***************************************************************************/
{
	if (prMEM == NULL)
		return TRUE;

	if (prMEM->state == RANGING_IDLE)
		return TRUE;

	return FALSE;
}

/***************************************************************************/
void NTRXRangingInit()
/***************************************************************************/
{
	ntrxShadowReg[NA_BbTimerIrqEnable_O] |= (uint8_t) (1 << NA_BbTimerIrqEnable_B);
	NTRXSPIWriteByte(NA_BbTimerIrqEnable_O, ntrxShadowReg[NA_BbTimerIrqEnable_O]);

	prMEM = &rangingMEM;
	prMEM->state = RANGING_IDLE;
}

/***************************************************************************/
int NTRXRangingRequest(MsgT *msg)
/***************************************************************************/
{
	/* transmitter still busy? */
	if (txSendMsg != NULL)
	{
		RANGING_DEBUG("req. - tx busy\n");
		return PHY_BUSY_TX;										// add phy returns. 
	}

	/* only start a new ranging process if no ranging is allready running */
	if (prMEM->state != RANGING_IDLE)
	{
		RANGING_DEBUG("req. - phy busy\n");
		return PHY_BUSY_TX;
	}

	switch (msg->attribute)
	{
	default: /* attribute not known, report error */
	{
		RANGING_DEBUG("req. - unsupported attribute\n");
		return PHY_UNSUPPORTED_ATTRIBUTE;
	}
		break;

	case RANGING_TYPE_3W_A:
	{
		/* use internal memory for A */
		prMEM = &rangingMEM;
		/* reset counter if target address changed */
		//prMEM->lrv[COUNTER] = 0;
		prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
	}
		break;

	case RANGING_TYPE_3W_B: /* use internal memory for B */
	{
		/* use internal memory for B */
		prMEM = &rangingMEM;
		/* reset counter if target address changed */
		//prMEM->lrv[COUNTER] = 0;
		prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
	}
		break;

	case RANGING_TYPE_2W_PP:
	{
		/* try to get memory from upper Layer */
#ifdef RANGE_GET_MEMORY_WORKAROUND
		prMEM = &rangingMEM;
#else
		if (rangingGetMemory == NULL)
		{
			RANGING_DEBUG("req. - fp not set\n");
			return PHY_FP_NOT_SET;
		}
 
		/* TODO get memory from upper Layer */
		prMEM = (RangingT*) (*rangingGetMemory)(sizeof(RangingT), msg->addr);
		RANGING_DEBUG("TX prMEM = %p (%d)\n", prMEM, (uint16_t) sizeof(RangingT));
#endif
		if (prMEM == NULL)
		{
			RANGING_DEBUG("req. - no memory\n");
			return PHY_NO_MEMORY;
		}
#if 0
		if (memcmp(prMEM->addr, msg->addr, sizeof(AddrT)) != 0)
		{
			/* reset counter if target address changed */
			prMEM->lrv[COUNTER] = 0;
		}
#endif
		if (memcmp(prMEM->addr, msg->addr, sizeof(AddrT)) != 0)
		{
			prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
		}

	}
		break;
	}

	/* start new ranging */
	if (msg->len > 0)
	{
		/* handle user data */
		if (msg->len > CONFIG_MAX_PACKET_SIZE - RANGING_DATA_LEN)
		{
			RANGING_DEBUG("req. - overflow\n");
			/* to much user data, return error */
			return PHY_USER_DATA_OVERFLOW;
		}

		/* move userdata to userdata space */
		memcpy(msg->data + RANGING_DATA_LEN + RANGING_PROTOCOL_LEN, msg->data, msg->len);
	}
	
	//// Kobus: set the local antenna offset to the value passed in the request. 
	local_antenna_offset = msg->value;

//	prMEM->errors = STAT_NO_ERROR;
#if 0
	printf("prMEM->lrv[COUNTER] == %d\n", prMEM->lrv[COUNTER]);
	if (prMEM->lrv[COUNTER] == 0)
	{
		printf("request: reset lrv + rrv\n");
		memset(prMEM->lrv, 0, sizeof(prMEM->lrv));
		memset(prMEM->rrv, 0, sizeof(prMEM->rrv));
		/* reset error flags */
		prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
	}
#endif
	/* state of ranging cycle set to begin */
	prMEM->lrv[ACTTXID] = 0;
	
	prMEM->lrv[CABLE_OFFSET] = local_antenna_offset;			// Kobus: add antenna offset to local values.
	
	/* copy destination address */
	memcpy(prMEM->addr, msg->addr, sizeof(AddrT));
	/* copy rangingprotocol into datafield */
	msg->data[0] = msg->attribute;
	memcpy(msg->data + RANGING_PROTOCOL_LEN, prMEM->lrv, RANGING_DATA_LEN);
	msg->len += RANGING_PROTOCOL_LEN + RANGING_DATA_LEN;

	/* set local state to running */
	prMEM->state = RANGING_RUNNING;

	/* attribute is used to identify ranging or datapacket */
	msg->attribute = MSG_TYPE_RANGING;

	RANGING_DEBUG("req. - send down\n");

	/* send ranging packet */
	NTRXSendPacket(msg);

	/* count every tx packet */
	//prMEM->lrv[COUNTER]++;
	
	return 0;
}

/***************************************************************************/
void NTRXRangingACK(void)
/***************************************************************************/
{
	MsgT *tMsg;

	switch (txSendMsg->data[0])
	{
	default: /* attribute not known, report error */
	{
		return;
	}
		break;

	case RANGING_TYPE_3W_A: /* use internal memory for A */
	{

		RANGING_DEBUG("ack(%d) 3W_A - %s\n", (uint16_t) prMEM->lrv[ACTTXID],
				txSendMsg->status == PHY_SUCCESS? "success": "no ack");

		switch (prMEM->lrv[ACTTXID])
		{
		case 0:

			if (txSendMsg->status == PHY_SUCCESS)
			{
				/* read ranging values from chip */
				NTRXReadRegisterACK();

				//NTRXStartBbTimer(modeSet.rangingTimeout);
				NTRXStartTimeout(RANGING_TIMEOUT);
			}
			else /* PHY_NO_ACK */
			{
				/* no ack received, cancel ranging process */
				prMEM->state = RANGING_IDLE;

				/* set error */
				//prMEM->errors |= STAT_NO_REMOTE_STATION;
				//prMEM->lrv[RANGING_ERROR] |= STAT_NO_REMOTE_STATION;
			}

			/* copy local values in up message */
			rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));

			/* copy into payload */
			memcpy(txSendMsg->data, &rangingMSG, sizeof(RangingMsgT));

			/* call call-back function to inform the user */
			txSendMsg->prim = PD_RANGING_CONFIRM;
			txSendMsg->attribute = RANGING_TYPE_3W_A;

			/* send up */
			tMsg = txSendMsg;
			txSendMsg = NULL;
			tMsg->pdu = tMsg->data;
			SendMsgUp(tMsg);

			break;
		case 1:
			if (txSendMsg->status == PHY_SUCCESS)
			{
				/* read ranging values from chip */
				NTRXReadRegisterACK();
			}
			else /* PHY_NO_ACK */
			{
				/* set error */
				//prMEM->errors |= STAT_NO_ANSWER1;
				//prMEM->lrv[RANGING_ERROR] |= STAT_NO_ANSWER1;
			}

			/* dont send msg up, but release storage */
			txSendMsg = NULL;

			/* new state of ranging process */
			prMEM->lrv[ACTTXID] = 2;

			/* generate second answer packet */
			upMsg.prim = PD_RANGING_REQUEST;
			
			///// Kobus: add our antenna offset to the LRV data sent back. 
			prMEM->lrv[CABLE_OFFSET] = local_antenna_offset;
			           
			/* copy rangingprotocol into datafield */
			upMsg.data[0] = RANGING_TYPE_3W_A;
			memcpy(upMsg.data + RANGING_PROTOCOL_LEN, prMEM->lrv, RANGING_DATA_LEN);
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			upMsg.len = RANGING_PROTOCOL_LEN + RANGING_DATA_LEN;
			/* attribute is used to identify ranging or datapacket */
			upMsg.attribute = MSG_TYPE_RANGING;

			/* send ranging packet */
			NTRXSendPacket(&upMsg);

			break;
		case 2:
			/* ranging done, stop timeout */
			NTRXStopTimeout();

			/* ranging finish, set state to IDLE */
			prMEM->state = RANGING_IDLE;

			/* dont send msg up, but release storage */
			txSendMsg = NULL;
			
			
#if 0
			/* check if the last packet contains user data */
			if (recBuffer.len > RANGING_PROTOCOL_LEN + RANGING_DATA_LEN)
			{
				/* give only user payload to upper layer */
				upMsg.prim = PD_DATA_INDICATION;
				memcpy(upMsg.addr, recBuffer.addr, sizeof(AddrT));
				upMsg.len = (uint8_t)(recBuffer.len - RANGING_PROTOCOL_LEN - RANGING_DATA_LEN);
				RANGING_DEBUG("rx - userdata (%d bytes)\n", (uint16_t) upMsg.len);
				memcpy(upMsg.data, recBuffer.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, upMsg.len);
				upMsg.pdu = upMsg.data;
				SendMsgUp(&upMsg);
			}
#endif

			break;
		}
	}
		break;

	case RANGING_TYPE_3W_B: /* use internal memory for B */
	{
		RANGING_DEBUG("ack(%d) 3W_B - %s\n", (uint16_t) prMEM->lrv[ACTTXID],
				txSendMsg->status == PHY_SUCCESS? "success": "no ack");

		switch (prMEM->lrv[ACTTXID])
		{
		case 0:

			if (txSendMsg->status == PHY_SUCCESS)
			{
				/* read ranging values from chip */
				NTRXReadRegisterACK();

				//NTRXStartBbTimer(modeSet.rangingTimeout);
				NTRXStartTimeout(RANGING_TIMEOUT);
			}
			else /* PHY_NO_ACK */
			{
				/* no ack recieved, cancel ranging process */
				prMEM->state = RANGING_IDLE;

				/* set error */
				//prMEM->errors |= STAT_NO_REMOTE_STATION;
				//prMEM->lrv[RANGING_ERROR] |= STAT_NO_REMOTE_STATION;
			}

			/* copy local valus in up message */
			//rangingMSG.error = prMEM->errors;
			rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			rangingMSG.distance = 0;
			memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));

			/* copy into payload */
			memcpy(txSendMsg->data, &rangingMSG, sizeof(RangingMsgT));

			/* call callbackfuntion to inform the user */
			txSendMsg->prim = PD_RANGING_CONFIRM;
			txSendMsg->attribute = RANGING_TYPE_3W_B;

			/* send up */
			tMsg = txSendMsg;
			txSendMsg = NULL;
			tMsg->pdu = tMsg->data;
			SendMsgUp(tMsg);

			break;
		case 1:

			if (txSendMsg->status == PHY_SUCCESS)
			{
				/* read ranging values from chip */
				NTRXReadRegisterACK();
			}
			else /* PHY_NO_ACK */
			{
				/* set error */
				//prMEM->errors |= STAT_NO_ANSWER1;
				//prMEM->lrv[RANGING_ERROR] |= STAT_NO_ANSWER1;
			}

			/* dont send msg up, but release storage */
			txSendMsg = NULL;
			break;

		case 2:
			/* ranging done, stop timeout */
			NTRXStopTimeout();

			/* ranging finish, set state to IDLE */
			prMEM->state = RANGING_IDLE;

			/* dont send msg up, but release storage */
			txSendMsg = NULL;
			
			////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////
			////////		Kobus: 
			////////			Note, at this point we have what's necessary to calculate a crude distance 
			////////			At the initiating node. 
			////////				(propagation 0, from previous ACK) 
			////////				(processing 0 received in packet.) 
			////////			Get this with NTRXSet0
			////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////
			double t1;
			NTRXSet0();				

			t1 = remote_dist(&prMEM->set1[0]);

			/* copy local values in up message */
			rangingMSG.distance = t1;
			rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
			upMsg.pdu = upMsg.data;

			/* call callbackfuntion to inform the user */
			upMsg.prim = PD_RANGING_INDICATION;

			/* send up */
			SendMsgUp(&upMsg);
					
			////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////
													
			break;
		}
	}
		break;

	case RANGING_TYPE_2W_PP: /* try to get memory from upper Layer */
	{
		//memset(prMEM->lrv, 0, 8); /* reset only ranging data */
		//memset(prMEM->rrv, 0, 8); /* reset only ranging data */

		RANGING_DEBUG("ack(%d) 2W_PP - %s\n", (uint16_t) prMEM->lrv[ACTTXID],
				txSendMsg->status == PHY_SUCCESS? "success": "no ack");

		switch (prMEM->lrv[ACTTXID])
		{
		case 0:
			/******** ** *******/
			/******** T1 *******/
			/******** ** *******/

			/* WARNING: cbo: Reset error flags only if a ranging is started, not in anycase. CHECK THIS!!! */
			/* reset error flags */
			//prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
			if (txSendMsg->status == PHY_SUCCESS)
			{
				/* reset error flags only on successfull started ranging */
				prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;

				/* read ranging values from chip */
				NTRXReadRegisterACK();

				//NTRXStartBbTimer(modeSet.rangingTimeout);
				NTRXStartTimeout(RANGING_TIMEOUT);
			}
			else /* PHY_NO_ACK */
			{
				/* no ack recieved, cancel ranging process */
				prMEM->state = RANGING_IDLE;

				/* set error */
				//prMEM->errors |= STAT_NO_REMOTE_STATION;
				//prMEM->lrv[RANGING_ERROR] |= STAT_NO_REMOTE_STATION;
			}

			/* copy local valus in up message */
			//rangingMSG.error = (uint8_t)(prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			//memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));
			/* copy into payload */
			//memcpy(txSendMsg->data, &rangingMSG, sizeof(RangingMsgT));
			/* call callbackfuntion to inform the user */
			txSendMsg->prim = PD_RANGING_CONFIRM;
			txSendMsg->attribute = RANGING_TYPE_2W_PP;

			/* send up */
			tMsg = txSendMsg;
			txSendMsg = NULL;
			tMsg->pdu = tMsg->data;
			SendMsgUp(tMsg);

			break;
		case 1:
			/******** ** *******/
			/******** T3 *******/
			/******** ** *******/

			if (txSendMsg->status == PHY_SUCCESS)
			{
				/* read ranging values from chip */
				NTRXReadRegisterACK();
			}

			/* ranging done, stop timeout */
			NTRXStopTimeout();

			prMEM->state = RANGING_IDLE;

			txSendMsg = NULL;
#if 0
			/* call callbackfuntion to inform the user */
			txSendMsg->prim = PD_RANGING_CONFIRM;
			txSendMsg->attribute = RANGING_TYPE_2W_PP;

			/* send up */
			tMsg = txSendMsg;
			txSendMsg = NULL;
			tMsg->pdu = tMsg->data;
			SendMsgUp (tMsg);
#endif

			break;
		}
	}
		break;
	}
}

/***************************************************************************/
void NTRXRangingRX(MsgT *msg)
/***************************************************************************/
{
#if 0
	/* check if the packet contains user data */
	if (msg->len > RANGING_PROTOCOL_LEN + RANGING_DATA_LEN)
	{
		/* give only user payload to upper layer */
		upMsg.prim = PD_DATA_INDICATION;
		memcpy(upMsg.addr, msg->addr, sizeof(AddrT));
		upMsg.len = (uint8_t)(msg->len - RANGING_PROTOCOL_LEN - RANGING_DATA_LEN);
		RANGING_DEBUG("rx - userdata (%d bytes)\n", (uint16_t) upMsg.len);
		memcpy(upMsg.data, msg->data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, upMsg.len);
		upMsg.pdu = upMsg.data;
		SendMsgUp(&upMsg);
	}
#endif

	/* buffer recieved data and call the callback functions after sending answers */
	memcpy(&recBuffer, msg, sizeof(MsgT));

	/* ranging type is @data[0] (attribute) */
	switch (msg->data[0])
	{
	default: /* attribute not known, report error */
	{
		return;
	}
		break;

	case RANGING_TYPE_3W_A: /* use internal memory for A */
	{
		/* use internal memory for A */
		prMEM = &rangingMEM;

		/* copy recieved data into rrv */
		memcpy(prMEM->rrv, msg->data + RANGING_PROTOCOL_LEN, RANGING_DATA_LEN);

		/* it is a ranging start cmd? */
		if (prMEM->rrv[ACTTXID] == 0)
		{
			uint8_t permLen = 0; /* length of user data on permission */

			////////	Kobus: seed user length value with the actual user data length	  //////////
			if(msg->len > (RANGING_PROTOCOL_LEN + RANGING_DATA_LEN)) 
				permLen = msg->len - (RANGING_PROTOCOL_LEN + RANGING_DATA_LEN);
			////////////////////////////////////////////////////////////////////////////////////////

			/* if function pointer is set, ask upper layer on incoming
			 * ranging requests if answer is to perform */
			if (rangingPermission != NULL)
			{
				if ((*rangingPermission)(msg->addr, upMsg.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, &permLen) == FALSE)
				{
					RANGING_DEBUG("rx ranging permission NOT granted\n");
					return; /* no answer allowed */
				}
			}

			/* ranging request, start ranging, start timeout */
			NTRXStartTimeout(RANGING_TIMEOUT);

			/* its the first packet of a ranging (ranging request) */
			/* => reset local variables and errors */
			//prMEM->errors = STAT_NO_ERROR;
			prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
			memcpy(prMEM->addr, msg->addr, sizeof(msg->addr));

			/* set local state to running */
			prMEM->state = RANGING_RUNNING;

			//memset(prMEM->lrv, 0, 8); /* reset only ranging data */
			//memset(prMEM->rrv, 0, 8); /* reset only ranging data */

			/* read the rx register (stored in lrv) */
			NTRXReadRegisterRX();

			/* ranging progresses 0 => 1 */
			prMEM->lrv[ACTTXID] = 1;

#if 0
			if (permLen > 0)
			{
				/* handle user data */
				if (permLen > CONFIG_MAX_PACKET_SIZE - RANGING_PROTOCOL_LEN - RANGING_DATA_LEN)
				{
					RANGING_DEBUG("rx 3W_A - userdata overflow\n");

					/* to much user data, return error */
					upMsg.prim = PD_RANGING_CONFIRM;
					upMsg.status = PHY_USER_DATA_OVERFLOW;
					memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
					upMsg.len = sizeof(rangingMSG);
					upMsg.pdu = upMsg.data;
					SendMsgUp (&upMsg);
					return;
				}

				/* move userdata to userdata space */
				memcpy(upMsg.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, permData, permLen);
			}
#endif

			/* generate first answer packet */
			upMsg.prim = PD_RANGING_REQUEST;
			upMsg.data[0] = RANGING_TYPE_3W_A;
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.data + RANGING_PROTOCOL_LEN, prMEM->lrv, RANGING_DATA_LEN);
			upMsg.len = (uint8_t) (permLen + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN);

			if (upMsg.len > CONFIG_MAX_PACKET_SIZE)
			{
				RANGING_DEBUG("rx 2W_PP - userdata overflow\n");
				error_handler(1);
			}

			/* attribute is used to identify ranging or datapacket */
			upMsg.attribute = MSG_TYPE_RANGING;

			/* send ranging packet */
			NTRXSendPacket(&upMsg);

		}
		else if (prMEM->rrv[ACTTXID] > 0)
		{

			/* if no ranging is ongoing, abort */
			if (prMEM->state != RANGING_RUNNING)
				return;

			/* continue only if its the same address */
			if (memcmp(prMEM->addr, msg->addr, sizeof(msg->addr)) == 0)
			{

				RANGING_DEBUG("rx 3W_A - %d\n", (uint16_t) prMEM->rrv[ACTTXID]);

				switch (prMEM->rrv[ACTTXID])
				{
				case 1: /* ranging answer1 received */
					NTRXReadRegisterRX();
					break;
				case 2: /* ranging answer2 received */
					NTRXSet0();
					NTRXSet1();

					NTRXStopTimeout();

					/* copy local values in up message */
					rangingMSG.distance = dist(&prMEM->set1[0], &prMEM->set2[0]);
					rangingMSG.antenna_offset = prMEM->rrv[CABLE_OFFSET];			/// Kobus: copy the remote antenna offset value into the rnage resonse. 
					rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
					memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));
					memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
					memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
					upMsg.pdu = upMsg.data;

					/* call callbackfuntion to inform the user */
					upMsg.prim = PD_RANGING_INDICATION;

					/* ranging done, new ranging possible */
					prMEM->state = RANGING_IDLE;

					/* send up */
					SendMsgUp(&upMsg);
					break;
				}
			}
		}
	}
		break;

	case RANGING_TYPE_3W_B: /* use internal memory for B */
	{
		/* use internal memory for A */
		prMEM = &rangingMEM;

		/* copy recieved data into rrv */
		memcpy(prMEM->rrv, msg->data + RANGING_PROTOCOL_LEN, RANGING_DATA_LEN);

		/* it is a ranging start cmd? */
		if (prMEM->rrv[ACTTXID] == 0)
		{
			uint8_t permLen = 0; /* length of user data on permission */

			////////	Kobus: seed user length value with the actual user data length	  //////////
			if(msg->len > (RANGING_PROTOCOL_LEN + RANGING_DATA_LEN)) 
				permLen = msg->len - (RANGING_PROTOCOL_LEN + RANGING_DATA_LEN);
			////////////////////////////////////////////////////////////////////////////////////////

			/* if function pointer is set, ask upper layer on incoming
			 * ranging requests if answer is to perform */
			if (rangingPermission != NULL)
			{
				if ((*rangingPermission)(msg->addr, upMsg.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, &permLen) == FALSE)
				{
					RANGING_DEBUG("rx ranging permission NOT granted\n");
					return; /* no answer allowed */
				}
			}

			/* ranging request, start ranging, start timeout */
			NTRXStartTimeout(RANGING_TIMEOUT);

			/* its the first packet of a ranging (ranging request) */
			/* => reset local variables and errors */
			//prMEM->errors = STAT_NO_ERROR;
			prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;
			memcpy(prMEM->addr, msg->addr, sizeof(msg->addr));

			/* set local state to running */
			prMEM->state = RANGING_RUNNING;

			//memset(prMEM->lrv, 0, sizeof(prMEM->lrv));
			//memset(prMEM->rrv, 0, sizeof(prMEM->rrv));
			//memset(prMEM->set1, 0, sizeof(prMEM->set1));
			//memset(prMEM->set2, 0, sizeof(prMEM->set2));

			/* read the rx register (stored in lrv) */
			NTRXReadRegisterRX();

			/* ranging progresses 0 => 1 */
			prMEM->lrv[ACTTXID] = 1;

#if 0
			if (permLen > 0)
			{
				/* handle user data */
				if (permLen > CONFIG_MAX_PACKET_SIZE - RANGING_PROTOCOL_LEN - RANGING_DATA_LEN)
				{
					RANGING_DEBUG("rx 3W_B - userdata overflow\n");

					/* to much user data, return error */
					upMsg.prim = PD_RANGING_CONFIRM;
					upMsg.status = PHY_USER_DATA_OVERFLOW;
					memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
					upMsg.len = sizeof(rangingMSG);
					upMsg.pdu = upMsg.data;
					SendMsgUp (&upMsg);
					return;
				}

				/* move userdata to userdata space */
				memcpy(upMsg.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, permData, permLen);
			}
#endif

			/* generate first answer packet */
			upMsg.prim = PD_RANGING_REQUEST;
			upMsg.data[0] = RANGING_TYPE_3W_B;
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.data + RANGING_PROTOCOL_LEN, prMEM->lrv, RANGING_DATA_LEN);
			upMsg.len = (uint8_t) (permLen + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN);

			if (upMsg.len > CONFIG_MAX_PACKET_SIZE)
			{
				RANGING_DEBUG("rx 2W_PP - userdata overflow\n");
				error_handler(1);
			}

			/* attribute is used to identify ranging or datapacket */
			upMsg.attribute = MSG_TYPE_RANGING;

			/* send ranging packet */
			NTRXSendPacket(&upMsg);
		}
		else if (prMEM->rrv[ACTTXID] > 0)
		{

			/* if no ranging is ongoing, abort */
			if (prMEM->state != RANGING_RUNNING)
				return;

			/* continue only if its the same address */
			if (memcmp(prMEM->addr, msg->addr, sizeof(msg->addr)) == 0)
			{

				RANGING_DEBUG("rx 3W_B - %d\n", (uint16_t) prMEM->rrv[ACTTXID]);

				switch (prMEM->rrv[ACTTXID])
				{
				case 1: /* ranging answer1 received */

					/* set local state to running */
					prMEM->state = RANGING_RUNNING;

					NTRXReadRegisterRX();

					prMEM->lrv[ACTTXID] = 2;

					///// Kobus: add our antenna offset to the LRV data sent back. 
					prMEM->lrv[CABLE_OFFSET] = local_antenna_offset;
					
					/* generate second answer packet */
					upMsg.prim = PD_RANGING_REQUEST;
					upMsg.data[0] = RANGING_TYPE_3W_B;
					memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
					memcpy(upMsg.data + RANGING_PROTOCOL_LEN, prMEM->lrv, RANGING_DATA_LEN);
					upMsg.len = RANGING_PROTOCOL_LEN + RANGING_DATA_LEN;

					/* attribute is used to identify ranging or datapacket */
					upMsg.attribute = MSG_TYPE_RANGING;

					/* send ranging packet */
					NTRXSendPacket(&upMsg);
					
					////////////////////////////////////////////////////////////////////////////////////////////
					////////////////////////////////////////////////////////////////////////////////////////////
					////////		Kobus: 
					////////			Note, at this point we have what's necessary to calculate a crude distance 
					////////			At the initiating node. 
					////////				(propagation 0, from previous ACK) 
					////////				(processing 0 received in packet.) 
					////////			Get this with NTRXSet0. we will do this in the next step once things have been finalised (ack2)
					////////////////////////////////////////////////////////////////////////////////////////////
					////////////////////////////////////////////////////////////////////////////////////////////
					
										
					break;
				case 2: /* ranging answer2 received */
					NTRXSet0();
					NTRXSet1();

					NTRXStopTimeout();

					/* copy local valus in up message */
					rangingMSG.distance = dist(&prMEM->set1[0], &prMEM->set2[0]);
					rangingMSG.antenna_offset = prMEM->rrv[CABLE_OFFSET];			/// Kobus: copy the remote antenna offset value into the range response. 
					rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
					memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));
					memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
					memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
					upMsg.pdu = upMsg.data;

					/* call callback function to inform the user */
					upMsg.prim = PD_RANGING_INDICATION;

					/* ranging done, new ranging possible */
					prMEM->state = RANGING_IDLE;

					/* send up */
					SendMsgUp(&upMsg);
					break;
				}
			}
		}
	}
		break;

	case RANGING_TYPE_2W_PP: /* try to get memory from upper Layer */
	{
		/* try to get memory from upper Layer */
#ifdef RANGE_GET_MEMORY_WORKAROUND
		prMEM = &rangingMEM;
#else
		if (rangingGetMemory == NULL)
		{
			/* get memory failed, report error to upper layer */
			msg->prim = PD_RANGING_CONFIRM;
			msg->status = PHY_FP_NOT_SET;
			memcpy(msg->data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
			msg->len = sizeof(rangingMSG);
			msg->pdu = msg->data;
			SendMsgUp(msg);
			return;
		}

		/* TODO get memory from upper Layer */
		prMEM = (RangingT*) rangingGetMemory(sizeof(RangingT), msg->addr);
		RANGING_DEBUG("RX prMEM = %p (%d)\n", prMEM, (uint16_t) sizeof(RangingT));
#endif

		if (prMEM == NULL)
		{
			/* get memory failed, report error to upper layer */
			msg->prim = PD_RANGING_CONFIRM;
			msg->status = PHY_NO_MEMORY;
			memcpy(msg->data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
			msg->len = sizeof(rangingMSG);
			msg->pdu = msg->data;
			SendMsgUp(msg);
			return;
		}

		/* copy recieved data into rrv */
		memcpy(prMEM->rrv, msg->data + RANGING_PROTOCOL_LEN, RANGING_DATA_LEN);

		if (memcmp(prMEM->addr, msg->addr, sizeof(AddrT)) != 0)
		{
			RANGING_DEBUG("rx 2W_PP - addr changed\n");

			memcpy(prMEM->addr, msg->addr, sizeof(AddrT));
		}
#if 0
		/* if remote address changed, reset ranging */
		if ((memcmp(prMEM->addr, msg->addr, sizeof(AddrT)) != 0) ||
				prMEM->lrv[COUNTER] == 0 ||
				prMEM->rrv[COUNTER] == 0)
		{
			RANGING_DEBUG("rx 2W_PP - reset ranging\n");

			memcpy(prMEM->addr, msg->addr, sizeof(AddrT));
			memset(prMEM->lrv, 0, sizeof(prMEM->lrv));
		}
#endif

		/* it is a ranging start cmd? */
		if (prMEM->rrv[ACTTXID] == 0)
		{
			/******** ** *******/
			/******** T2 *******/
			/******** ** *******/
			uint8_t permLen = 0; /* length of user data on permission */

			////////	Kobus: seed user length value with the actual user data length	  //////////
			if(msg->len > (RANGING_PROTOCOL_LEN + RANGING_DATA_LEN)) 
				permLen = msg->len - (RANGING_PROTOCOL_LEN + RANGING_DATA_LEN);
			////////////////////////////////////////////////////////////////////////////////////////

			/* if function pointer is set, ask upper layer on incoming
			 * ranging requests if answer is to perform */
			if (rangingPermission != NULL)
			{
				if ((*rangingPermission)(msg->addr, upMsg.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, &permLen) == FALSE)
				{
					RANGING_DEBUG("rx ranging permission NOT granted\n");
					return; /* no answer allowed */
				}
			}

			/* ranging request, start ranging, start timeout */
			//NTRXStartTimeout(RANGING_TIMEOUT);
			/* its the first packet of a ranging (ranging request) */
			/* => reset local variables and errors */
			prMEM->state = RANGING_RUNNING;

//					if (prMEM->rrv[COUNTER] > 0 && prMEM->lrv[COUNTER] > 0)
//					{
			RANGING_DEBUG("rx 2W_PP - succ range\n");

			/* copy lrv and rrv now into set0/1 */
			NTRXSet0();
			NTRXSet1();
			/* copy error flags into rangingMSG which is later provided to user */
			rangingMSG.distance = dist(&prMEM->set1[0], &prMEM->set2[0]);
			rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));

#if 0
			/* copy local valus in up message */
			rangingMSG.distance = dist(&prMEM->set1[0],&prMEM->set2[0]);
			rangingMSG.error = (uint8_t)(prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.data, (uint8_t*)&rangingMSG, sizeof(RangingMsgT));
			upMsg.pdu = upMsg.data;

			/* call callbackfuntion to inform the user */
			upMsg.prim = PD_RANGING_INDICATION;

			/* send up */
			SendMsgUp (&upMsg);
#endif

//					}
//					else
//					{
//						RANGING_DEBUG("rx 2W_PP - no result, counter == 0\n");
//					}

			//prMEM->errors = STAT_NO_ERROR;
			prMEM->lrv[RANGING_ERROR] = RG_STAT_DEFAULT;

			/* read the rx register (stored in lrv) */
			NTRXReadRegisterRX();

			/* ranging progresses 0 => 1 */
			prMEM->lrv[ACTTXID] = 1;

			//prMEM->lrv[COUNTER]++;

			/* generate first answer packet */
			upMsg.prim = PD_RANGING_REQUEST;
			upMsg.data[0] = RANGING_TYPE_2W_PP;
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.data + RANGING_PROTOCOL_LEN, prMEM->lrv, RANGING_DATA_LEN);

#if 0
			if (permLen > 0)
			{
				/* handle user data */
				if (permLen > CONFIG_MAX_PACKET_SIZE - RANGING_PROTOCOL_LEN - RANGING_DATA_LEN)
				{
					RANGING_DEBUG("rx 2W_PP - userdata overflow\n");

					/* to much user data, return error */
					upMsg.prim = PD_RANGING_CONFIRM;
					upMsg.status = PHY_USER_DATA_OVERFLOW;
					memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
					upMsg.len = sizeof(rangingMSG);
					upMsg.pdu = upMsg.data;
					SendMsgUp (&upMsg);
					return;
				}

				/* move userdata to userdata space */
				memcpy(upMsg.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, permData, permLen);
			}
#endif

			upMsg.len = (uint8_t) (permLen + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN);

			if (upMsg.len > CONFIG_MAX_PACKET_SIZE)
			{
				RANGING_DEBUG("rx 2W_PP - userdata overflow\n");
				error_handler(1);
			}

			/* attribute is used to identify ranging or datapacket */
			upMsg.attribute = MSG_TYPE_RANGING;

			/* send ranging packet */
			NTRXSendPacket(&upMsg);

			/* give ranging result to upper layer */
			memcpy(upMsg.addr, rangingMSG.addr, sizeof(AddrT));
			memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
			upMsg.pdu = upMsg.data;
			/* call callbackfuntion to inform the user */
			upMsg.prim = PD_RANGING_INDICATION;
			/* send up */
			SendMsgUp(&upMsg);

		}
		else if (prMEM->rrv[ACTTXID] == 1)
		{
			/******** ** *******/
			/******** T4 *******/
			/******** ** *******/

			/* if no ranging is ongoing, abort */
			if (prMEM->state != RANGING_RUNNING)
			{
				RANGING_DEBUG("rx ERROR - another ranging is running\n");
				return;
			}

			/* continue only if its the same address */
			if (memcmp(prMEM->addr, msg->addr, sizeof(AddrT)) != 0)
			{
				RANGING_DEBUG("rx ERROR - remote addr changed\n");
				return;
			}

			NTRXReadRegisterRX();

			/* ranging done, stop timeout */
			NTRXStopTimeout();

			/* ranging done, new ranging possible */
			prMEM->state = RANGING_IDLE;

			/* report successful ranging to user */
			upMsg.prim = PD_RANGING_INDICATION;
			upMsg.status = PHY_SUCCESS;
			memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
			upMsg.len = sizeof(rangingMSG);
			upMsg.pdu = upMsg.data;
			SendMsgUp(&upMsg);
		}
	}
		break;
	}

	/* check if the last packet contains user data */
	if (recBuffer.len > RANGING_PROTOCOL_LEN + RANGING_DATA_LEN)
	{
		/* give only user payload to upper layer */
		upMsg.prim = PD_DATA_INDICATION;
		memcpy(upMsg.addr, recBuffer.addr, sizeof(AddrT));
		upMsg.len = (uint8_t) (recBuffer.len - RANGING_PROTOCOL_LEN - RANGING_DATA_LEN);
		RANGING_DEBUG("rx - userdata (%d bytes)\n", (uint16_t) upMsg.len);
		memcpy(upMsg.data, recBuffer.data + RANGING_PROTOCOL_LEN + RANGING_DATA_LEN, upMsg.len);
		upMsg.pdu = upMsg.data;
		SendMsgUp(&upMsg);
	}

}

/**************************************************************************/
void NTRXReadRegisterACK(void)
/**************************************************************************/
{
	uint8_t ToaOffsetMeanAckValid;

	/* Read Tx Ranging Registers */
	NTRXSPIReadByte(NA_ToaOffsetMeanAckValid_O, &ToaOffsetMeanAckValid);
	ToaOffsetMeanAckValid &= (0x01 << NA_ToaOffsetMeanAckValid_B);
	ToaOffsetMeanAckValid = (uint8_t) (ToaOffsetMeanAckValid >> NA_ToaOffsetMeanAckValid_B);

	if (ToaOffsetMeanAckValid != 1)
	{
		RANGING_DEBUG("ERROR ReadRegisterACK\n");/* error */
	}
	else
	{
		NTRXSPIReadByte(NA_PhaseOffsetAck_O, &(prMEM->lrv[PHASEOFFSETACK]));
		prMEM->lrv[PHASEOFFSETACK] &= (0x07 << NA_PhaseOffsetAck_LSB);
		prMEM->lrv[PHASEOFFSETACK] = (uint8_t) (prMEM->lrv[PHASEOFFSETACK] >> NA_PhaseOffsetAck_LSB);

		NTRXSPIRead(NA_ToaOffsetMeanAck_O, &(prMEM->lrv[TOAOFFSETMEANACK_L]), 2);
		prMEM->lrv[TOAOFFSETMEANACK_H] &= 0x1f;

		NTRXSPIRead(NA_TxRespTime_O, &(prMEM->lrv[TXRESPTIME_L]), 2);

		if (prMEM->lrv[ACTTXID] == 0)
		{
			/* T1 read success */
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_T1;
			//printf("T1\n");
		}
		else if (prMEM->lrv[ACTTXID] == 1)
		{
			/* T3 read success */
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_T3;
			//printf("T3\n");
		}
	}
}

/**************************************************************************/
double dist(uint8_t *p1, uint8_t *p2)
/**************************************************************************/
{
	static double speedofmedium = SPEED_OF_AIR;
	double distanceD2R;
	double distanceR2D;
	double avg = -1.0;

	/* calculate the one way airtime for local station */
	distanceD2R = delay(p1);

	/* calculate the distance in [m] for local station */
	distanceD2R *= (speedofmedium);

	/* calculate the one way airtime for remote station */
	distanceR2D = delay(p2);

	/* calculate the distance in [m] for remote station */
	distanceR2D *= (speedofmedium);

	/* the difference between the measurement results from local-
	 * and remote station should not be to large */
	if (distanceR2D > distanceD2R)
	{
		if ((distanceR2D - distanceD2R) > MAX_DIFF_ALLOWED)
		{
			//prMEM->errors |= STAT_RANGING_VALUE_ERROR;
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_VALUE_ERROR;
		}
	}
	else
	{
		if ((distanceD2R - distanceR2D) > MAX_DIFF_ALLOWED)
		{
			//prMEM->errors |= STAT_RANGING_VALUE_ERROR;
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_VALUE_ERROR;
		}
	}

	//avg = ((distanceD2R + distanceR2D) / 2.0); /* [m] */
	avg = fmin(distanceD2R, distanceR2D);

	return avg;
}

/**************************************************************************/
double remote_dist(uint8_t *p1)
/**************************************************************************/
{
	static double speedofmedium = SPEED_OF_AIR;
	double distanceD2R;
	
	/* calculate the one way airtime for local station */
	distanceD2R = delay(p1);

	/* calculate the distance in [m] for local station */
	distanceD2R *= (speedofmedium);

	return distanceD2R;
}

#define CLK_4MHZ 4
#define CLK_32MHZ 32
#define CLK_LOD20 (2000.0/244175)
#define PULSE_DET_UC_MAX (2*24)

/**************************************************************************/
double delay(uint8_t *p)
/**************************************************************************/
{
	/* clock period [MHz] */
	//const uint8_t clk_4MHz  = 4;
	//const uint8_t clk_32MHz = 32;
	/* Scaled 1:20 divider's clock period [MHz] */
	//const double clk_lod20 = (2000.0/244175);
	//const uint16_t PulseDetUcMax = 5;
	//const uint16_t PulseDetUcMax_table[16] = {1, 2, 4, 8, 16, 24, 32, 40, 48, 56, 64, 1, 1, 1, 1, 1};
	//const uint8_t PulseDetUcMax_table = 24;
	double res;

	uint32_t TxRespTime = (p[TXRESPTIME_H] << 8) | (p[TXRESPTIME_L]);
	int32_t RxUcSum = (p[TOAOFFSETMEANACK_H] << 8) | (p[TOAOFFSETMEANACK_L]);
	int32_t TxUcSum = (p[TOAOFFSETMEANDATA_H] << 8) | (p[TOAOFFSETMEANDATA_L]);

	int32_t RxGateOff = p[PHASEOFFSETACK] == 7 ? 7 : 6 - p[PHASEOFFSETACK];
	int32_t TxGateOff = p[PHASEOFFSETDATA] == 7 ? 7 : 6 - p[PHASEOFFSETDATA];

	double rangingConst = (modeSet.fec) ? modeSet.rangingConst_FECon : modeSet.rangingConst_FECoff;

//	printf("%ld:%ld:%ld:%ld:%ld::", TxRespTime, RxUcSum, TxUcSum, RxGateOff, TxGateOff);

	if (modeSet.fdma == 0)
	{
		res = ((TxRespTime) / (double) CLK_4MHZ - ((TxGateOff + RxGateOff)) / (double) CLK_32MHZ - (TxUcSum + RxUcSum) * CLK_LOD20 / PULSE_DET_UC_MAX) / 2.0 - rangingConst;
	}
	else
	{
		res = (TxRespTime / (double) CLK_4MHZ - (TxGateOff + RxGateOff) / (double) CLK_32MHZ - (TxUcSum + RxUcSum) / CLK_32MHZ * PULSE_DET_UC_MAX) / 2.0 - rangingConst;
	}

	return res;
}

/**************************************************************************/
void NTRXSet0(void)
/**************************************************************************/
{
	/* combine data for locally initiated round trip */
	prMEM->set1[PHASEOFFSETACK] = prMEM->lrv[PHASEOFFSETACK];
	prMEM->set1[TOAOFFSETMEANACK_H] = prMEM->lrv[TOAOFFSETMEANACK_H];
	prMEM->set1[TOAOFFSETMEANACK_L] = prMEM->lrv[TOAOFFSETMEANACK_L];
	prMEM->set1[TXRESPTIME_H] = prMEM->lrv[TXRESPTIME_H];
	prMEM->set1[TXRESPTIME_L] = prMEM->lrv[TXRESPTIME_L];

	prMEM->set1[PHASEOFFSETDATA] = prMEM->rrv[PHASEOFFSETDATA];
	prMEM->set1[TOAOFFSETMEANDATA_H] = prMEM->rrv[TOAOFFSETMEANDATA_H];
	prMEM->set1[TOAOFFSETMEANDATA_L] = prMEM->rrv[TOAOFFSETMEANDATA_L];

#if 0
	printf("%d:%d:%d:%d:%d:",prMEM->set1[PHASEOFFSETACK],
			prMEM->set1[TOAOFFSETMEANACK_H] << 8 | prMEM->set1[TOAOFFSETMEANACK_L],
			prMEM->set1[TXRESPTIME_H] << 8 | prMEM->set2[TXRESPTIME_L],
			prMEM->set1[PHASEOFFSETDATA],
			prMEM->set1[TOAOFFSETMEANDATA_H] << 8 | prMEM->set1[TOAOFFSETMEANDATA_L]
	);
#endif
}

/**************************************************************************/
void NTRXSet1(void)
/**************************************************************************/
{
	/* combine data for remotely initiated round trip */
	prMEM->set2[PHASEOFFSETACK] = prMEM->rrv[PHASEOFFSETACK];
	prMEM->set2[TOAOFFSETMEANACK_H] = prMEM->rrv[TOAOFFSETMEANACK_H];
	prMEM->set2[TOAOFFSETMEANACK_L] = prMEM->rrv[TOAOFFSETMEANACK_L];
	prMEM->set2[TXRESPTIME_H] = prMEM->rrv[TXRESPTIME_H];
	prMEM->set2[TXRESPTIME_L] = prMEM->rrv[TXRESPTIME_L];

	prMEM->set2[PHASEOFFSETDATA] = prMEM->lrv[PHASEOFFSETDATA];
	prMEM->set2[TOAOFFSETMEANDATA_H] = prMEM->lrv[TOAOFFSETMEANDATA_H];
	prMEM->set2[TOAOFFSETMEANDATA_L] = prMEM->lrv[TOAOFFSETMEANDATA_L];

#if 0
	printf("%d:%d:%d:%d:%d:",prMEM->set2[PHASEOFFSETACK],
			prMEM->set2[TOAOFFSETMEANACK_H] << 8 | prMEM->set2[TOAOFFSETMEANACK_L],
			prMEM->set2[TXRESPTIME_H] << 8 | prMEM->set2[TXRESPTIME_L],
			prMEM->set2[PHASEOFFSETDATA],
			prMEM->set2[TOAOFFSETMEANDATA_H] << 8 | prMEM->set2[TOAOFFSETMEANDATA_L]
	);
#endif
}

/**************************************************************************/
void NTRXReadRegisterRX(void)
/**************************************************************************/
{
	uint8_t ToaOffsetMeanDataValid;

	/* Read Rx Ranging Registers */
	NTRXSPIReadByte(NA_ToaOffsetMeanDataValid_O, &ToaOffsetMeanDataValid);
	ToaOffsetMeanDataValid &= (0x01 << NA_ToaOffsetMeanDataValid_B);
	ToaOffsetMeanDataValid = (uint8_t) (ToaOffsetMeanDataValid >> NA_ToaOffsetMeanDataValid_B);

	if (ToaOffsetMeanDataValid != 1)
	{
		RANGING_DEBUG("ERROR ReadRegisterRX\n");/* error */
	}
	else
	{
		NTRXSPIReadByte(NA_PhaseOffsetData_O, &(prMEM->lrv[PHASEOFFSETDATA]));
		prMEM->lrv[PHASEOFFSETDATA] &= (0x07 << NA_PhaseOffsetData_LSB);
		prMEM->lrv[PHASEOFFSETDATA] = (uint8_t) (prMEM->lrv[PHASEOFFSETDATA] >> NA_PhaseOffsetData_LSB);

		NTRXSPIRead(NA_ToaOffsetMeanData_O, &(prMEM->lrv[TOAOFFSETMEANDATA_L]), 2);
		prMEM->lrv[TOAOFFSETMEANDATA_H] &= 0x1f;

		if (prMEM->rrv[ACTTXID] == 0)
		{
			/* T2 read success */
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_T2;
			//printf("T2\n");
		}
		else if (prMEM->rrv[ACTTXID] == 1)
		{
			/* T4 read success */
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_T4;
			//printf("T4\n");
		}

	}

}

/** @brief structure for all layer configuration settings  */
extern PhyPIB phyPIB;
/**************************************************************************/
void NTRXStartTimeout(uint16_t to_ms)
/**************************************************************************/
{
	to_stop = FALSE;

	if (phyPIB.CAMode != PHY_CCA_OFF)
		to_ms += TO_CSMA;

	if (to_ms > 20)
	{
		to_cnt_down = to_ms / 20;
		to_mod = to_ms % 20;
		NTRXStartBbTimer(20 * 1000);
	}
	else
	{
		to_cnt_down = 0;
		to_mod = 0;
		NTRXStartBbTimer(to_ms * 1000);
	}
}

/**************************************************************************/
void NTRXStopTimeout(void)
/**************************************************************************/
{
	to_stop = TRUE;
}

/**************************************************************************/
void NTRXRangingInterrupt(void)
/**************************************************************************/
{

	if (to_stop == TRUE)
	{
		//RANGING_DEBUG("interr - to abort\n");
		return;
	}

	RANGING_DEBUG("interr - cnt %d, mod %d\n", to_cnt_down, to_mod);

	if (to_cnt_down > 0)
	{
		NTRXStartBbTimer(20 * 1000);
		to_cnt_down--;
	}
	else
	{
		if (to_mod > 0)
		{
			NTRXStartBbTimer(to_mod * 1000);
			to_mod = 0;
		}
		else
		{
			/* timeout! */
			RANGING_DEBUG("interr - timeout\n");

			//prMEM->errors |= STAT_TIMEOUT;
			prMEM->lrv[RANGING_ERROR] |= RG_STAT_TIMEOUT;

			/* copy local valus in up message */
			rangingMSG.error = (uint8_t) (prMEM->lrv[RANGING_ERROR] | prMEM->rrv[RANGING_ERROR]);
			RANGING_DEBUG("interr, indication, 0x%02X\n", (uint16_t) rangingMSG.error);
			rangingMSG.distance = -1.0;
			memcpy(rangingMSG.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.addr, prMEM->addr, sizeof(AddrT));
			memcpy(upMsg.data, (uint8_t*) &rangingMSG, sizeof(RangingMsgT));
			upMsg.pdu = upMsg.data;

			/* call callbackfuntion to inform the user */
			upMsg.prim = PD_RANGING_INDICATION;

			/* ranging done, new ranging possible */
			prMEM->state = RANGING_IDLE;

			/* send up */
			SendMsgUp(&upMsg);
		}
	}
}

/**************************************************************************/
void RangingRegisterCallback(permissionfn_t rgPermission, mem_assignfn_t rgGetMemory)
/**************************************************************************/
{
	rangingPermission = rgPermission;
	rangingGetMemory = rgGetMemory;
}

