/*
 * master_interface.h
 *
 *  Created on: Aug 2014
 *      Author: J.L. Goosen
 */

#ifndef MASTER_INTERFACE1_H_
#define MASTER_INTERFACE1_H_

#include "framing.h"
#include "Global_Variables.h"
#include "Vision_Parameters.h"
#include "Transponders.h"
//#include "LF_APL.h"
#include "RF_APL.h"

#ifdef __cplusplus
extern "C" {
#endif
	
typedef enum Master_source
{
	NONE = 0,
	COM = 1,
	MUSB = 2,
	MCAN = 3,
	CODE = 4,
	RF = 5,
} Master_source;

typedef struct
{
	Master_source Master;
	uint8_t* data;
	uint16_t len;
}_Q_MasterIF;

extern Master_source Master_last;

int Send_to_Master(_Q_MasterIF* MIF);
int push_to_master(_Q_MasterIF MIF);

int Send_data_toMaster(uint8_t* data, int len, Master_source M);
int send_heartbeat(uint32_t last_RF);
int send_LF_sync(void);
int send_LF_sync_F(uint8_t SlaveID);
int send_LF_alert(int ms_delay);
int parse_message(uint8_t parse_data[], uint8_t parse_length, Master_source MIF_Source);
uint8_t MIF_Parse(_Q_MasterIF MIF);
int parse_message_old(_Q_MasterIF MIF);
void UART_packet_handler(void);
void CAN_packet_handler(uint8_t* data, uint8_t len, bool Last, uint8_t slave_id);
void GPSModule_Message_Handler(void);
uint8_t* buff_alloc(uint8_t* data, int len, bool store);

int Send_POD_toMaster(void* t, Master_source M, char command);
void forward_LF_report(LF_message_type LF);


#ifdef __cplusplus
}
#endif
	
#endif /* MASTER_INTERFACE_H_ */
