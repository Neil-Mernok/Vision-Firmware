/*
 * NMEA_Parse_Message.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef NMEA_PARSE_MESSAGE_H_
#define NMEA_PARSE_MESSAGE_H_

#include <stddef.h>
#include <stdint.h>
#include "usart.h"

#include "NMEA_General.h"

#include "master_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

uint16_t NMEA_parse_message(_Q_MasterIF* NMEA_MIF, uint8_t NMEA_start);
uint8_t Parse_NMEA_GNSS_Message(_Q_MasterIF* NMEA_GNSS_MIF, uint8_t GNSS_start);
uint8_t Parse_NMEA_GLONASS_Message(_Q_MasterIF* NMEA_GLONASS_MIF, uint8_t GLONASS_start);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_PARSE_MESSAGE_H_ */
