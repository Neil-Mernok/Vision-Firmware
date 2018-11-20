/*
 * NMEA_General.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef NMEA_GENERAL_H_
#define NMEA_GENERAL_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "master_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NMEA_Offset 		6
#define Carrige_New_Line	2

typedef enum
{
	NMEA_Galileo = 0x41,	// A
	NMEA_BeiDou	= 0x42,		// B
	NMEA_GLONASS = 0x4C, 	// L
	NMEA_GNSS = 0x4E, 		// N
	NMEA_GPS = 0x50, 		// P
}_NMEA_Talker;

typedef enum
{
	ASCII_$ = 0x24,  		// $
	ASCII_star = 0x2A,  	// *

	ASCII_4 = 0x34,  		// 4
	ASCII_5 = 0x35,  		// 5
	ASCII_6 = 0x36,  		// 6
	ASCII_7 = 0x37,  		// 7

	ASCII_A = 0x41, 		// A
	ASCII_B = 0x42, 		// B
	ASCII_C = 0x43, 		// C
	ASCII_G = 0x47, 		// G
	ASCII_L = 0x4C, 		// L
	ASCII_M = 0x4D, 		// M
	ASCII_N = 0x4E, 		// N
	ASCII_R = 0x52, 		// R
	ASCII_S = 0x53, 		// S
	ASCII_T = 0x54, 		// T
	ASCII_V = 0x56, 		// V
}_NMEA_Sentence_Formatter;

typedef enum
{
	NMEA_FAIL = 0x00,
	NMEA_PASS = 0x01
}_NMEA_General;

uint16_t NMEA_Checksum(_Q_MasterIF* NMEA_Checksum_MIF, uint8_t NMEA_Checksum[3], uint8_t NMEA_Checksum_start);
#ifdef __cplusplus
}
#endif

#endif /* UBLOX_GENERAL_H_ */

