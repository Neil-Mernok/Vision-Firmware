/* $Id$ */

/**
 * @file app.c
 * @date 2007-11-29
 * @c Copyright 2007- Nanotron Technologies
 * @author Christian Bock
 *
 * @brief This file contains the source code for the demo
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * $Revision: 7617 $
 * $Date: 2010-03-29 12:15:31 +0200 (Mo, 29 Mär 2010) $
 * $LastChangedBy: hgu $
 * $LastChangedDate: 2010-03-29 12:15:31 +0200 (Mo, 29 Mär 2010) $
 */
/*
 * $Log$
 */

#include	"hwclock.h"
#include	"ntrxutil.h"
#include	"ntrxranging.h"

#include	"nnspi.h"
#include	<stdio.h>
#include	<string.h>
#include	<stdlib.h>
#include	"app.h"
#include	"phy.h"
#include 	"hwclock.h"
#include 	"Vision_Parameters.h"
#include 	"transponders.h"
//#include 	"master_interface.h"
//#include	"Tasks.h"


//#define 	apl_debug	printf
#define apl_debug	NoDebugging

/** @brief Structure type for all layer configuration settings
 *
 * To be IEEE layer complient, all configuration data is stored
 * in one data structure.
 */

/** @brief Message structure for data or ranging requests.  */
MsgT downMsg;
/** @brief Message structure for payload of received ranging packets.  */
RangingMsgT *upRangingMsg;
/** @brief Ranging type which is used */
uint32_t last_calib = 0;

bool_t Permission(uint8_t *macAddr, uint8_t *data, uint8_t *len);
void *GetMemForPHY(uint16_t memSize, uint8_t *macAddr);
int nApl_Parse_message(uint8_t* buffer, uint32_t sender, uint8_t len);
void Range_handler(_Transpondert* t, uint16_t dist, bool forward);

extern PhyPIB phyPIB;
#ifdef CONFIG_ALIVE_LED
#include "led.h"
/**
 * @brief Periodicaly flash one LED.
 *
 * Hartbeat to indicate a normal running application
 */
/****************************************************************************/
void IsAlive(void)
/****************************************************************************/
{
	static int flag = 0;
	static unsigned long last = 0;
	unsigned long now;

	now = hwclock();
	if (now - last > 500)
	{
		last = now;
		if (flag == 0)
		{
			flag = 1;
			LED_ALIVE(LED_ON);
		}
		else
		{
			flag = 0;
			LED_ALIVE(LED_OFF);
		}
	}
}
#endif

/*******************************************************************/
/**
 * @brief Upstream message handling.
 * @par payload containing data
 * @par len length of payload
 *
 * ApplCallback represents the upstream part of the application.
 *
 */
char serial_print_buffer[100];
void APLCallback(MsgT *msg)
{
//	int i;
	_Transpondert* T;

	switch (msg->prim)
	{
	case PD_DATA_INDICATION:
		//// data received
		//// data length store in msg->len
		//// data stored on msg->data
		//// sender stored in msg->addr
		Vision_Status.LastRF = time_now();
		apl_debug("--Data Received. %d bytes\n", msg->len);
		// parse the various types of data packets.
		nApl_Parse_message(msg->data, (*((uint32_t*) (msg->addr))), msg->len);

		break;

	
	case PD_DATA_CONFIRM:
		// note: data confirmed seems to indicate nothing. it does not signal that another devices has received the signal, merely that tx was successful. 
		Vision_Status.LastRF = time_now();
		apl_debug("--Data Confirmed\n");
		break;
	case PD_RANGING_CONFIRM:
		upRangingMsg = (RangingMsgT*) msg->data;
		
		// get a reference to the tag in our list. 
		T = get_tag(*((uint32_t*) (upRangingMsg->addr)));
		
		apl_debug("--Range confirm\n");
		switch (msg->status)
		{
		case PHY_SUCCESS:
			/* hwack received, ranging start successfully */
			Vision_Status.LastRF = time_now();
			//apl_debug("--RC: range started\n");
			apl_debug("--\tRC: sucess\n");
			break;
		case PHY_NO_ACK:
			/* no hwack received, ranging didnt start */
			apl_debug("--\tRC: no hw-ack!!\n");
////////////////			break;
		case PHY_BUSY:
		case PHY_BUSY_TX:
			/* measurement is allready running (BUSY), wait
			 for PD_RANGING_INDICATION first! */
			apl_debug("--\tRC: range busy\n");
////////////////			break;
		case PHY_CONFIGURATION_ERROR:
			/* driver isnt correct initialized for ranging */
			apl_debug("--driver isn't correct initialised for ranging.\n");
			////////////////			break;
		default:
			apl_debug("--\tRC: error\n");
			/// check if the id is present in the ID list. if so, increment its retry count and send again.get_tag 
			if (T != NULL)
			{
				if(T->range_retries++ < 3)
				{
					add_ID_to_be_ranged(T->UID);
				}
			}

			break;
		}
		break;
	case PD_RANGING_INDICATION:
		upRangingMsg = (RangingMsgT*) msg->data;
		T = get_tag(*((uint32_t*) (upRangingMsg->addr)));
		
		if (T != NULL)
		{
			//////////////////////////////////////////////////////////////////
			if (upRangingMsg->distance > -8.0)
			{
				upRangingMsg->distance -= (upRangingMsg->antenna_offset * 0.1);
				upRangingMsg->distance -= (local_antenna_offset * 0.1);				// compensate for antenna cable on both receiver and transmitter side.
				
				if (upRangingMsg->distance < 0.1)
				{
					upRangingMsg->distance = upRangingMsg->prev_distance;
					upRangingMsg->distance_error++;
				}
				else
				{
					upRangingMsg->prev_distance = upRangingMsg->distance;
					upRangingMsg->distance_error = 0;
				}

				// If upRangingMsg->distance were more than two times incorrectly in a row, send -1 as error.
				if (upRangingMsg->distance_error >= 2)
				{
					upRangingMsg->distance = -1;
				}

				Range_handler(T, (int)(upRangingMsg->distance * 10.0), false);

				apl_debug("--Range DIST:success.\t0x%08X = %0.1f m\n", T.UID, upRangingMsg->distance);
			}
			else
			{
			
				//T->Dist = 0;
				apl_debug("--Range DIST:invalid.\t0x%08X = %0.1f m\n", T.UID, upRangingMsg->distance);

				//Ranging_Transp_handler(T);
			}
		}
		//////////////////////////////////////////////////////////////////
		Vision_Status.LastRF = time_now();
		break;
	default:
		break;
	}
}

/*******************************************************************/
/**
 * @brief Poll function of the application.
 *
 * Will be called from the system main loop in frequent intervals.
 */
//void APLPoll(void)
//{
//#ifndef TIMER_LIB
//	uint32_t now = hwclock();
//	static uint32_t last = 0;
//	static uint32_t last_changed = 0;
//	static int data_count = 0;
//#endif
//
//#ifdef TIMER_LIB
//	TimerPoll();
//#endif
//
//	/* check key 2, if its pressed => wait until released and
//	 after that toggle the ranging type between all 3 possible
//	 types */
//	//if (now > last_changed + 3000)
//	//if (GetKeyState(1) == KEY_REL_AFTER_10MS)
//	//{
//	//last_changed = now;
//	if (ranging_type == RANGING_TYPE_3W_A)
//		ranging_type = RANGING_TYPE_3W_B;
//	else
//		ranging_type = RANGING_TYPE_3W_A;
//	//apl_debug("--RangingType : %s\n", ranging_type == RANGING_TYPE_3W_A ? "3W_A" : ranging_type == RANGING_TYPE_3W_B ? "3W_B" : "ERROR");
//
//#ifdef TIMER_LIB
//	TimerCancel(t_recal);
//
//	t_recal = TimerStart( 5000, /*[ms]*/
//			&TimerRecal, /*callback function*/
//			NULL); /* parameter */
//
//	apl_debug("--RecalTimer restarted\n");
//#endif
//	//}
//
//#ifndef TIMER_LIB
//	/* send ranging every 125ms */
//	if (now > last + 400)
//	{
//		/* recalibration every 5sec */
//		if (now > last_calib + 5000)
//		{
//			downMsg.prim = PLME_SET_REQUEST;
//			downMsg.attribute = PHY_RECALIBRATION;
//			PLMESap(&downMsg);
//			last_calib = now;
//		}
//
//		/* store last timestamp */
//		last = now;
//		memcpy(downMsg.addr, apl->dest, sizeof(AddrT));
//
////		if (data_count++ > 10)
////		{
////			data_count = 0;
//////			downMsg.prim = PD_DATA_REQUEST;
//////			/* dummy userdata only for demonstration */
//////			downMsg.data[0] = 0x11;
//////			downMsg.data[1] = 0x12;
//////			downMsg.data[2] = 0x13;
//////			downMsg.data[3] = 0x14;
//////			downMsg.data[4] = 0x22;
//////			downMsg.data[5] = 0x32;
//////			downMsg.data[6] = 0x42;
//////			downMsg.data[7] = 0x52;
//////			/* dummy data length */
//////			downMsg.len = 8;
//////			
//////			PDSap(&downMsg);
////		}
////		else
////		{
//		downMsg.prim = PD_RANGING_REQUEST;
//		downMsg.len = 0;
//
//		downMsg.attribute = ranging_type;
//		PDSap(&downMsg);
////		}
//	}
//#endif /* TIMER_LIB */
//
//#ifdef CONFIG_ALIVE_LED
//	IsAlive();
//#endif
//}
void APL_Calibrate(void)
{
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_RECALIBRATION;
	PLMESap(&downMsg);
}

/**
 * @brief: send data to the specified UID. 
 * @param UID			target to send to 
 * @param data			data to send 
 * @param len			number of bytes to send
 */
int APL_SendMessage(uint32_t Dest, uint8_t* data, uint8_t len, uint8_t message_type)
{
	memcpy(downMsg.addr, &Dest, sizeof(uint32_t));				// set to destination
	// detect broadcast
	if (Dest == 0xFFFFFFFF)
	{
		downMsg.addr[4] = 0xFF;
		downMsg.addr[5] = 0xFF;
	}
	else
	{
		downMsg.addr[4] = 0;
		downMsg.addr[5] = 0;
	}
	
	// sanity check on message length. 
	if(len >= (CONFIG_MAX_PACKET_SIZE-1))
			len = CONFIG_MAX_PACKET_SIZE-1;

	downMsg.prim = PD_DATA_REQUEST;

	downMsg.data[0] = message_type;
	memcpy(&downMsg.data[1], data, len);

	/* dummy data length */
	downMsg.len = len + 1;
	return PDSap(&downMsg);
}

int APL_RangeToID(uint32_t ID, uint16_t antenna_offset)
{
	memcpy(downMsg.addr, &ID, sizeof(ID));
	downMsg.addr[4] = 0;
	downMsg.addr[5] = 0;			// only care about LSB's		
	downMsg.prim = PD_RANGING_REQUEST;
	downMsg.len = 0;
	downMsg.attribute = RANGING_TYPE_3W_A;			// result locally generated.
//	downMsg.attribute = RANGING_TYPE_3W_B;			// result remotely generated.
	
	downMsg.value = antenna_offset;					// send the antenna offset to the ranging request. 
	
	return PDSap(&downMsg);
}

void APL_Sleep(uint32_t ms)
{
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_PWR_DOWN_MODE;
	downMsg.value = 1; 	// pad mode. ~600uA drawn.
	*((uint32_t*) downMsg.data) = ms;
	PLMESap(&downMsg);
}

void APL_RX_EN(uint8_t on_off)
{
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_RX_CMD;
	if (on_off)
		downMsg.value = PHY_RX_ON;
	else
		downMsg.value = PHY_TRX_OFF;
	PLMESap(&downMsg);
}

void APL_Wake(void)
{
	phyPIB.pwrDown = FALSE;

	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_POWERDOWN_PAD_MODE;
	downMsg.value = 0; 	// 0 means power-up.
	PLMESap(&downMsg);
}

void APL_SetTXPow(uint8_t power)
{
	// program TX power
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_TX_POWER;
	downMsg.value = power;// >> 2;
	PLMESap(&downMsg);
}

/****************************************************************************/
void *phyMemP = NULL;
void *GetMemForPHY(uint16_t memSize, uint8_t *macAddr)
{
	static uint8_t first = 1;

	if (phyMemP == NULL)
	{
		phyMemP = malloc(memSize);
	}

	if (first)
		memset(phyMemP, 0, memSize);
	first = 0;

	return phyMemP;
}

/****************************************************************************/
bool_t Permission(uint8_t *macAddr, uint8_t *data, uint8_t *len)
{
#ifdef SIMULATE_USER_DATA
	uint8_t return_data[] =
	{	0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

	*len = 5;
	memcpy(data, return_data, *len);
#endif
	return TRUE;
}

#ifdef TIMER_LIB
/*******************************************************************/
void TimerRecal(void *param)
{
	static uint32_t last = 0;

	t_recal = TimerStart( 5000, /*[ms]*/
			&TimerRecal, /*callback function*/
			NULL); /* parameter */

	uint32_t time_now = hwclock();
	apl_debug("--(%ld) TimerRecal : %ld\n", time_now, time_now - last);
	last = time_now;

	do
	{
		PHYPoll();

		if (PHYIsIDLE())
		{
			downMsg.prim = PLME_SET_REQUEST;
			downMsg.attribute = PHY_RECALIBRATION;
			PLMESap (&downMsg);
		}

	}while(!PHYIsIDLE());
}

/*******************************************************************/
void TimerRanging(void *param)
{
	static uint32_t last = 0;

	TimerStart( 125, /*[ms]*/
			&TimerRanging, /*callback function*/
			NULL); /* parameter */

	uint32_t time_now = hwclock();
	apl_debug("--(%ld) TimerRanging : %ld\n", time_now, time_now - last);
	last = time_now;

	memcpy (downMsg.addr, apl->dest, sizeof(AddrT));
	downMsg.prim = PD_RANGING_REQUEST;

#ifdef SIMULATE_USER_DATA
	/* dummy userdata only for demonstration */
	downMsg.data[0] = 0x11;
	downMsg.data[1] = 0x12;
	downMsg.data[2] = 0x13;
	downMsg.data[3] = 0x14;
	downMsg.data[4] = 0x22;
	downMsg.data[5] = 0x32;
	downMsg.data[6] = 0x42;
	downMsg.data[7] = 0x52;
	/* dummy data length */
	downMsg.len = 8;
#else
	downMsg.len = 0;
#endif

	downMsg.attribute = ranging_type;
	LED0 (LED_ON);
	PDSap (&downMsg);
}
#endif

/*******************************************************************/
/**
 * @brief Initialize the application.
 *
 * Initialize the application.
 */
void APLInit(uint32_t UID, uint8_t RF_power, uint8_t antenna_offset)
{
	/***************** Chip UID. *****************/
	/* write the source address to the TRX chip */
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_MAC_ADDRESS1;
	memcpy(downMsg.data, &UID, 4);			// use 4 bytes to be a unique ID 
	downMsg.data[4] = 0;					//dummy bytes. we use only 32bit ID
	downMsg.data[5] = 0;
	PLMESap(&downMsg);

	// switch FEC off /				/////// dont change this. its not better...
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_FEC;
	downMsg.value = FALSE;
	PLMESap(&downMsg);

#ifdef CSMA
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_CCA_MODE;
	downMsg.value = CSMA;
	PLMESap(&downMsg);

#endif

	// program TX power
	APL_SetTXPow(RF_power);				// todo: fix ranger RF power
	local_antenna_offset = antenna_offset;		// set the local antenna offset value to compensate for antenna cable length. 
	
	// turn on RX 
	downMsg.prim = PLME_SET_REQUEST;
	downMsg.attribute = PHY_RX_CMD;
	downMsg.value = PHY_RX_ON;
	PLMESap(&downMsg);

	NTRXAllCalibration();

#ifdef TIMER_LIB
	apl_debug("--START_TIMER\n");

	TimerInitLib();

	TimerStart( 125, /*[ms]*/
			&TimerRanging, /*callback function*/
			NULL); /* parameter */

	t_recal = TimerStart( 5000, /*[ms]*/
			&TimerRecal, /*callback function*/
			NULL); /* parameter */
#endif

	RangingRegisterCallback(&Permission, &GetMemForPHY);
}



extern void error_handler(int16_t err)
{
	while(1)
	{
		__NOP();
	}
}
