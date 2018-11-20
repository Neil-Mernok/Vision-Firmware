/*
 * RF_APL.h
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

#ifndef RF_APL_H_
#define RF_APL_H_

//#include "Global_Variables.h"
#include "Transponders.h"
#include "LF_APL.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum rf_messages
{
	rf_no_mess = 0,			// 
	rf_legcPDS = 'p',		// this is not actually used as a message header, but the enum is useful
	rf_ID_puls = 'i',		// this is a regular ID ping, but in the newer format
	rf_LF_resp = 'l',	 	// this is a response message after a tag saw an LF field
	rf_LF_send = 'L',		// this is a message indicating that someone is sending an LF message
	rf_ID_name = 'n', 		// this is a massage where the tag sends its string name
	rf_MasterP = 'M',		// this is a master message to a specific tag via RF. 
	rf_Respons = 'R',		// this is a response to a master message from a specific tag. 
	rf_Boot_M  = 'E',		// this is a master boot message to a tag via RF (on channel 3).
	rf_BootPRT = 'B',		// this is part of a master boot message to a tag via RF (on channel 3).
	rf_Boot_R  = 'b',		// this is a response from a remote bootloading device to its master. 
	rf_Data    = 'D',		// this is a data message to a specific UID/VID
	rf_RangReq = 'q',		// Ranging system, range request.
	rf_RangCal = 'C',		// Ranging distance report. Send after successful range to inform other parties of the distance. 
	rf_GPS_C   = 'g',		// GPS coordinates
} rf_messages;

typedef enum rf_message_size
{
	rf_no_mess_size = 0,
	rf_legacy_PDS_size = 8,				// this is not actually used as a message header, but the enum is useful
	rf_ID_Pulse_size = 20,				// this is a Pulse ID ping
	rf_LF_resp_size = 24,	 			// this is a response message after a tag saw an LF field
	rf_LF_send_size = 21,				// this is a message indicating that someone is sending an LF message
	rf_GPS_Coordinates_size   = 41,		// GPS coordinates
	rf_ID_Pulse_GPS_size = 42,			// this is a Pulse_GPS ID ping
	rf_Remote_message_size = 9
} rf_message_size;

typedef struct
{
	rf_messages type;
	uint8_t len;
	uint8_t* buff;
	uint32_t time_to_respond;
	LF_message_type LF;
} RF_message;

uint8_t get_status_byte();
int Rf_Info_Packet(uint8_t type, uint8_t Identifier);
void Rf_GPS_info(uint8_t Array_start);
void Apl_broadcast_ID(void);
uint8_t Apl_report_LF(LF_message_type LF);
uint8_t Apl_sync_LF();
uint8_t Apl_report_name();
uint8_t Apl_report_name_var(uint8_t type, char* name);
void Apl_Send_PDS(uint8_t type);
uint8_t Apl_report_ID(uint8_t type);

uint8_t Apl_send_master_message(uint8_t* buffer, uint8_t len);
uint8_t Apl_send_master_response(uint8_t* buffer, uint8_t len);
uint8_t Apl_send_data_message(uint8_t* buffer, uint8_t len);
uint8_t Apl_master_boot_message(uint8_t* buffer, uint8_t len);
//uint8_t Apl_send_zone_message(uint8_t* buffer);
uint8_t Apl_report_GPS_Coordinates(void);
void Apl_Parse_message(uint8_t* buffer, int len, uint8_t RSSI, bool boot_channel);
_Transpondert* Transp_RF_handler(uint8_t* buffer, uint8_t RSSI, uint8_t len);
void parse_RF_into_tag(_Transpondert* T, uint8_t* buffer, uint8_t RSSI, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* RF_APL_H_ */
