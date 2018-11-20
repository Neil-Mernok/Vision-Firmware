/*
 * NMEA_General.c
 *
 *  Created on: Mar 9, 2017
 *      Author: FrancoisHattingh
 */
#include "NMEA_General.h"

/**
 * @brief  NMEA Checksum Algorithm
 * @param  Pointer to NMEA_data structure where checksum must be determined
 * @retval Size of the NMEA_data including the checksum size
 */
uint16_t NMEA_Checksum(_Q_MasterIF* NMEA_Checksum_MIF, uint8_t NMEA_Checksum[3], uint8_t NMEA_Checksum_start)
{
	uint8_t message_crc = 0;
	message_crc += NMEA_Offset;

	// ---- The checksum algorithm ----
	while (message_crc < NMEA_Checksum_MIF->len)
	{
		if ((NMEA_Checksum_MIF->data[NMEA_Checksum_start + message_crc - 2] == NMEA_Checksum[0]) & (NMEA_Checksum_MIF->data[NMEA_Checksum_start + message_crc - 1] == NMEA_Checksum[1]) & (NMEA_Checksum_MIF->data[NMEA_Checksum_start + message_crc] == NMEA_Checksum[2]))
		{
			return message_crc + Carrige_New_Line;
		}
		else if (NMEA_Checksum_MIF->data[NMEA_Checksum_start + message_crc] == ASCII_$)
		{
			return NMEA_FAIL;
		}
		message_crc++;
	}
	return message_crc;
}
//
///**
// * @brief  uBlox Checksum Algorithm
// *         The checksum is calculated over the packet, starting and including the CLASS field.
// *         The checksum algorithm used is the 8-Bit Fletcher Algorithm
// * @param  Pointer to uBlox_Message structure where checksum must be determined
// * @retval Size of the uBlox message including the checksum size
// */
//uint8_t UBX_Checksum_Compare(_uBlox_Message* Compare_Message)
//{
//	// ---- Conjugate length array to form uint16_t ----
//	uint16_t range = UBX_Conjugate_uint16(Compare_Message->uBlox_Structure.Length[0], Compare_Message->uBlox_Structure.Length[1]);
//
//	// ---- Add the identifier offset - UBX-Class-ID-Length ----
//	range += Identifier_Offset;
//
//	uint8_t Compare_Message_Data[range];
//
//	// ---- Copy data from pointer structure into array ----
//	memcpy(Compare_Message_Data,&Compare_Message->uBlox_Structure,range);
//
//	uint8_t crc_a = 0;
//	uint8_t crc_b = 0;
//
//	// ---- The checksum algorithm used is the 8-Bit Fletcher Algorithm ----
//	for ( uint8_t i = UBX_Offset ; i < range ; i++)
//	{
//		crc_a = crc_a + Compare_Message_Data[i] ;
//		crc_b = crc_b + crc_a;
//	}
//
//	// Place values of checksum into structure passed via pointer ----
//	if ((Compare_Message->uBlox_Structure.Checksum[0] == crc_a) && (Compare_Message->uBlox_Structure.Checksum[1] == crc_b))
//	{
//		// Return the size of message structure used by UART transmit ----
//		return UBX_PASS;
//	}
//	return UBX_FAIL;
//}
//
///**
// * @brief  uBlox Conjugate  2x uint8 to uint16
// * @param  	First - data byte index 0
// * 			Second - data byte index 1
// * @retval  Returns conjugated value - uint16_t
// */
//uint16_t UBX_Conjugate_uint16(uint8_t First, uint8_t Second)
//{
//	uint16_t value = ((uint16_t) First) & 0x00FF;
//	value |= (((uint16_t) Second) << 8) & 0xFF00;
//	return value;
//}
//
///**
// * @brief  uBlox Conjugate  4x uint8 to uint32
// * @param  	First - data byte index 0
// * 			Second - data byte index 1
// * 			Third - data byte index 2
// * 			Fourth - data byte index 3
// * @retval  Returns conjugated value - uint32_t
// */
//uint32_t UBX_Conjugate_uint32(uint8_t First, uint8_t Second, uint8_t Third, uint8_t Fourth)
//{
//	uint32_t value = ((uint32_t) First) & 0x000000FF;
//	value |= (((uint32_t) Second) << 8) & 0x0000FF00;
//	value |= (((uint32_t) Third) << 16) & 0x00FF0000;
//	value |= (((uint32_t) Fourth) << 24) & 0xFF000000;
//	return value;
//}
//
///**
// * @brief  uBlox Conjugate  4x uint8 to uint32
// * @param  	First - data byte index 0
// * 			Second - data byte index 1
// * 			Third - data byte index 2
// * 			Fourth - data byte index 3
// * @retval  Returns conjugated value - uint32_t
// */
//uint32_t UBX_DecimalDeg_to_MinSec(int32_t Lat_Long)
//{
//	int degree = (int)Lat_Long;
//	int minutes = (int) ( (Lat_Long - (double)degree) * 60.0);
//	int seconds = (int) ( (Lat_Long - (double)degree - (double)minutes / 60.0) * 60.0 * 60.0 );
//}


