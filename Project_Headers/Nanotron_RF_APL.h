/*
 * RF_APL.h
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

#ifndef NANO_RF_APL_H_
#define NANO_RF_APL_H_

#include "Transponders.h"

//#ifdef __cplusplus
////extern "C" {
//#endif

int nApl_report_ID(uint8_t message_type);
int nApl_range_callout(uint32_t remote_UID, uint16_t distance, uint16_t range_lockout);
//uint8_t Apl_report_name_var(uint8_t type, char* name);

uint8_t nApl_send_master_message(uint8_t* buffer, uint8_t len);
uint8_t nApl_send_master_response(uint8_t* buffer, uint8_t len);
uint8_t nApl_send_data_message(uint8_t* buffer, uint8_t len);
uint8_t nApl_send_data_message(uint32_t dest, uint8_t* buffer, uint8_t len);
uint8_t nApl_master_boot_message(uint8_t* buffer, uint8_t len);
_Transpondert* Transp_nRF_handler(uint8_t* buffer, uint32_t UID, uint8_t len);

extern "C" void parse_nRF_into_tag(_Transpondert* T, uint8_t* buffer, uint8_t len);
extern "C" void add_ID_to_be_ranged(uint32_t ID);

//#ifdef __cplusplus
////}
//#endif

#endif /* RF_APL_H_ */
