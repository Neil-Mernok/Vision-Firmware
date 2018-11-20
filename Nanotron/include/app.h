/* $Id$ */

/*****************************************************************************
 *
 * Copyright 2007
 * Nanotron Technologies
 *
 * Author: S. Radtke
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * Description :
 *    This file contains the interface for the application layer
 *
 * $Revision: 7207 $
 * $Date: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 *
 ****************************************************************************/

/*
 * $Log$
 */

#ifndef APL_H
#define APL_H

#include "ntrxtypes.h"

#define NoDebugging(args, ...)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////		RF data interface functions			/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RF_lockout_max 1000


#ifdef __cplusplus
extern "C" {
#endif

void add_ID_to_be_ranged(uint32_t ID);
enum message_lengths{
	id_brdc_len = 8,
//	range_request_len = 8,
};
extern uint32_t RF_lockout1;	// This is set after an intent to range broadcast is received. 
								// Only ranging requesting devices must send request messages to the device that issued the intent
extern uint32_t RF_lockout2;	// This is set when a pcket is reciewved indicating the start of ranging. 
								// Only the intent device will ahve access ot the air (to send forwarding ranges).
extern uint32_t RF_UID;			// this is the UID which requested the lockout. used for conflict resolution. 


void APLMECallback (MsgT *msg);
void APLCallback (MsgT *msg);

void APLInit(uint32_t UID, uint8_t RF_power, uint8_t antenna_offset);
void APL_Calibrate(void);
int APL_SendMessage(uint32_t Dest, uint8_t* data, uint8_t len, uint8_t message_type);	// add function returns.
void APL_SetAddrMatching(uint8_t value);
int APL_RangeToID(uint32_t ID, uint16_t antenna_offset);															// add function returns.
void APL_Sleep(uint32_t ms);
void APL_Wake(void);
void APL_SetTXPow(uint8_t power);
void APL_RX_EN(uint8_t on_off);

#ifdef __cplusplus
}
#endif

#endif
