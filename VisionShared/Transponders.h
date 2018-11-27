/*
 * PDS_transps.h
 *
 *  Created on: Nov 12, 2012
 *      Author: J.L. Goosen
 */

#ifndef TRANSPONDERS1_H_
#define TRANSPONDERS1_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "list.h"
#include "LF_APL.h"
#include "GPS_APL.h"
#include "ParamValue.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum _kind
{
	unknown 		= 0,
	PDS 			= 1,
	Ranging 		= 2,
	Pulse 			= 3,
	Ranging_pulse 	= 4,
	Pulse_GPS 		= 5,
	GPS 			= 6,
	GSM				= 7
} _kind;



typedef struct _Transpondert
{
	uint32_t 		UID;
	uint8_t			type;
	uint8_t			group;
	uint8_t 		status;
	uint8_t 		rssi;
	uint8_t 		volts;
	uint8_t			SlaveID;
	uint8_t			UserParam;
	uint32_t		last_seen;
	uint32_t 		VehicleID;
	uint16_t 		RangePeriod;
	uint32_t 		last_ranged;
	uint8_t 		range_retries;
	uint8_t 		range_needed;
	int16_t			Dist;
	uint8_t			FirmwareRev;
	_kind 			kind;
	LF_message_type LF;

	// ---- Added V12 - GPS functionality ----
	GPS_Transponder_Type GPS_Data;

	//todo: Neil
	// --- Added V14 - Mantag Acknowledge
	uint32_t		Speed;
	uint8_t 		ManTagAck;
	uint8_t			Reverse;
	uint8_t			V_Width;
	uint8_t			V_lenght;
	uint8_t			Stopping_dist;

#ifdef USE_TAG_NAME
	char	 		name[STR_MAX];
#endif
#ifdef traffic_system
	uint8_t			last_gate;
	uint8_t			current_gate;
	uint32_t		last_seen_gate;
#endif
	xListItem 		listI;
} _Transpondert;


//------- Global Variables --------//
extern _Transpondert transp_log[];
extern uint32_t transp_timeout;
extern uint8_t transp_count;
extern xList POD_list;

//------- Global functions --------//
_Transpondert* get_tag(uint32_t UID);
int get_slot(uint32_t UID);
void clear_transp_log(void);
int clean_transp_log(int age);
void clear_transp(_Transpondert* T);
bool transp_type_check(uint8_t type);
void transp_type_add(uint8_t type, bool val);
//bool transp_filter(_Transpondert* transp, int restart, int max_dist, int max_age, uint8_t type, uint8_t filter_vehicles);
_Transpondert* transp_filter(int restart, int max_dist, int max_age, uint8_t type, uint8_t filter_vehicles, _kind kind);
void re_insert_transp(_Transpondert* T);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPONDERS_H_ */
