/*
 * uBlox_Parse_Message.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef UBLOX_PARSE_MESSAGE_H_
#define UBLOX_PARSE_MESSAGE_H_

#include <stddef.h>
#include <stdint.h>
#include "usart.h"

#include "uBlox_General.h"
#include "master_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

uint16_t uBlox_parse_message(_Q_MasterIF uBlox_MIF, uint16_t start_pos);
uint8_t Parse_NAV_Message(_uBlox_Structure* Parse_NAV_Message);
uint8_t Parse_ACK_Message(_uBlox_Structure* Parse_ACK_Message);
uint8_t Parse_HNR_Message(_uBlox_Structure* Parse_HNR_Message);
uint8_t Parse_STA_Message(_uBlox_Structure* Parse_STA_Message);
uint8_t Parse_ESF_Message(_uBlox_Structure* Parse_ESF_Message);
uint8_t Parse_SEC_Message(_uBlox_Structure* Parse_SEC_Message);
uint8_t Parse_MON_Message(_uBlox_Structure* Parse_MON_Message);

#ifdef __cplusplus
}
#endif

#endif /* UBLOX_INTERFACE_H_ */
