/*
 * master_interface.h
 *
 *  Created on: Aug 2014
 *      Author: J.L. Goosen
 */

#ifndef MASTER_INTERFACE1_H_
#define MASTER_INTERFACE1_H_

#include "Global_Variables.h"

//#ifdef __cplusplus
//extern "C" {
//#endif
//	
typedef enum
{
	NONE = 0,
	COM = 1,
	MUSB = 2,
	MCAN = 3,
	CODE = 4,
	RF = 5
} Master_source;

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

typedef struct
{
	Master_source Master;
	uint8_t* data;
	uint16_t len;
}_Q_MasterIF;

//#define Master_COM COM_1

int Send_to_Master(_Q_MasterIF* MIF);
int push_to_master(_Q_MasterIF MIF);

int Send_data_toMaster(uint8_t* data, int len, Master_source M);
int send_heartbeat(uint32_t last_RF);
int parse_message(_Q_MasterIF MIF);
void Boot_packet_handler(int M);
void CAN_packet_handler(uint8_t* data, uint8_t len, bool Last);
uint8_t CAN_out_handler(uint8_t* data, uint8_t len);

uint8_t* buff_alloc(uint8_t* data, int len, bool store);

int Send_POD_toMaster(void* t, Master_source M);



//#ifdef __cplusplus
//}
//#endif
	
#endif /* MASTER_INTERFACE_H_ */
