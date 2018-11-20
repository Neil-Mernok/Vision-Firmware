/*
 * uBlox_Generals.c
 *
 *  Created on: Mar 9, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_General.h"
#include "GPS_APL.h"
#include "Delay.h"

uint32_t SAT_Fix_Time = 0;
bool SAT_Fix_Startup = true;

/**
 * @brief  uBlox Checksum Algorithm
 *         The checksum is calculated over the packet, starting and including the CLASS field.
 *         The checksum algorithm used is the 8-Bit Fletcher Algorithm
 * @param  Pointer to uBlox_Message structure where checksum must be determined
 * @retval Size of the uBlox message including the checksum size
 */
uint16_t UBX_Checksum(_uBlox_Message* Check_Message)
{
	// ---- Conjugate length array to form uint16_t ----
	uint16_t range = UBX_Conjugate_uint16(Check_Message->uBlox_Structure.Length[0], Check_Message->uBlox_Structure.Length[1]);

	// ---- Add the identifier offset - UBX-Class-ID-Length ----
	range += Identifier_Offset;

	uint8_t Check_Message_Data[range];

	// ---- Copy data from pointer structure into array ----
	memcpy(Check_Message_Data,&Check_Message->uBlox_Structure,range);

	uint8_t crc_a = 0;
	uint8_t crc_b = 0;

	// ---- The checksum algorithm used is the 8-Bit Fletcher Algorithm ----
	for ( uint8_t i = UBX_Offset ; i < range ; i++)
	{
		crc_a = crc_a + Check_Message_Data[i] ;
		crc_b = crc_b + crc_a;
	}

	// Place values of checksum into structure passed via pointer ----
	Check_Message->uBlox_Structure.Checksum[0] = crc_a;
	Check_Message->uBlox_Structure.Checksum[1] = crc_b;

	// Return the size of message structure used by UART transmit ----
	return range + Checksum_Offset;
}

/**
 * @brief  uBlox Checksum Algorithm
 *         The checksum is calculated over the packet, starting and including the CLASS field.
 *         The checksum algorithm used is the 8-Bit Fletcher Algorithm
 * @param  Pointer to uBlox_Message structure where checksum must be determined
 * @retval Size of the uBlox message including the checksum size
 */
uint8_t UBX_Checksum_Compare(_uBlox_Message *Compare_Message)
{
	// ---- Conjugate length array to form uint16_t ----
	uint16_t range = UBX_Conjugate_uint16(Compare_Message->uBlox_Structure.Length[0], Compare_Message->uBlox_Structure.Length[1]);

	// ---- Add the identifier offset - UBX-Class-ID-Length ----
	range += Identifier_Offset;

	uint8_t Compare_Message_Data[range];

	// ---- Copy data from pointer structure into array ----
	memcpy(Compare_Message_Data,&Compare_Message->uBlox_Structure,range);

	uint8_t crc_a = 0;
	uint8_t crc_b = 0;

	// ---- The checksum algorithm used is the 8-Bit Fletcher Algorithm ----
	for ( uint16_t i = UBX_Offset ; i < range ; i++)
	{
		crc_a = crc_a + Compare_Message_Data[i] ;
		crc_b = crc_b + crc_a;
	}

	// Place values of checksum into structure passed via pointer ----
	if ((Compare_Message->uBlox_Structure.Checksum[0] == crc_a) && (Compare_Message->uBlox_Structure.Checksum[1] == crc_b))
	{
		// Return the size of message structure used by UART transmit ----
		return UBX_PASS;
	}
	return UBX_FAIL;
}

/**
 * @brief  uBlox Conjugate  2x uint8 to uint16
 * @param  	First - data byte index 0
 * 			Second - data byte index 1
 * @retval  Returns conjugated value - uint16_t
 */
uint16_t UBX_Conjugate_uint16(uint8_t First, uint8_t Second)
{
	uint16_t value = ((uint16_t) First) & 0x00FF;
	value |= (((uint16_t) Second) << 8) & 0xFF00;

	return value;
}

/**
 * @brief  uBlox Conjugate  4x uint8 to uint32
 * @param  	First - data byte index 0
 * 			Second - data byte index 1
 * 			Third - data byte index 2
 * 			Fourth - data byte index 3
 * @retval  Returns conjugated value - uint32_t
 */
uint32_t UBX_Conjugate_uint32(uint8_t First, uint8_t Second, uint8_t Third, uint8_t Fourth)
{
	uint32_t value = ((uint32_t) First)  & 0x000000FF;
	value |= (((uint32_t) Second) << 8)  & 0x0000FF00;
	value |= (((uint32_t) Third) << 16)  & 0x00FF0000;
	value |= (((uint32_t) Fourth) << 24) & 0xFF000000;
	return value;
}

/**
 * @brief  uBlox Conjugate  4x uint8 to uint32
 * @param  	First - data byte index 0
 * 			Second - data byte index 1
 * 			Third - data byte index 2
 * 			Fourth - data byte index 3
 * @retval  Returns conjugated value - uint32_t
 */
uint32_t UBX_DecimalDeg_to_MinSec(int32_t Lat_Long_deg, int32_t* Lat_Long_Min)
{
//	int degree = (int)Lat_Long;
//	int minutes = (int) ((Lat_Long - (double)degree) * 60.0);
//	int seconds = (int) ((Lat_Long - (double)degree - (double)minutes / 60.0) * 60.0 * 60.0);

	return UBX_TRUE;
}

/**
 * @brief  uBlox Compress  uint32 to uint8
 * @param  	Value - value to be compressed
 * @retval  Returns compressed value - uint8_t
 */
uint8_t UBX_Calculate_Age(uint8_t Current_Fix)
{
	// ---- Update fix age according to if a fix has been established ----
	if ((Current_Fix != No_Fix) && (Current_Fix != Time_Only))
	{
		SAT_Fix_Time = time_now();
		if (SAT_Fix_Startup == true)
			SAT_Fix_Startup = false;
	}
	if (SAT_Fix_Startup == true)
	{
		// ---- Set time since to start-up ----
		return 0xFF;
	}
	else
	{
		// ---- Calculate the time since last update ----
		return UBX_Compress_uint32(time_since(SAT_Fix_Time),0xFA);
	}
}

/**
 * @brief  uBlox Compress  uint32 to uint8
 * @param  	Value - value to be compressed, Max_Value maximum value that may be returned
 * @retval  Returns compressed value - uint8_t
 */
uint8_t UBX_Compress_uint32(uint32_t Value, uint8_t Max_Value)
{
	// ---- Calculate the time since last update ----
	return (uint8_t) MIN((Value/1000), Max_Value);
}


