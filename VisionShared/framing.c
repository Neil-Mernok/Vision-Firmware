/*
 * framing.c
 *
 *  Created on: Jun 23, 2015
 *      Author: Kobus
 */


#include "framing.h"
#include <stdio.h>
#include <string.h>

/**
 * working code to calculate crc16 CCITT
 * @param data_p
 * @param length
 */
unsigned short crc16(unsigned char* data_p, unsigned short length)
{
	unsigned char x;
	unsigned short crc = 0xFFFF;

	while (length--)
	{
		x = crc >> 8 ^ *data_p++;
		x ^= x >> 4;
		crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
	}
	return crc;
}


uint8_t buff[200];

uint8_t* get_frame(uint8_t* data, uint8_t len, uint8_t* lenout)
{
	uint16_t crc = crc16(data, len);
	
	buff[0] = SOF1;
	buff[1] = SOF2;
	buff[2] = len;
	memcpy (&buff[3], data, len);
	buff[len+3] = crc;
	buff[len+4] = (crc>>8);
	buff[len+5] = EOF1;
			
	*lenout = len+6;
	return buff;
}

bool check_frame_crc(uint8_t* data, uint8_t len)
{
	unsigned short crc; 
	if((data[0] == SOF1) && (data[1] == SOF2) && (data[2] == len))
	{
		crc = *(uint16_t*)(data + 3 + len);
		if(crc == crc16(data + 3, len))
			return true;
		else
			return false;
	}	
	return false;
}

bool check_frame(uint8_t* data)
{
//	if((data[0] == SOF1) && (data[1] == SOF2) && (data[data[2] + 5]))
//	{
//		return true;
//	}
//	return false;

	unsigned short crc;
	uint8_t len = data[2];

	if((data[0] == SOF1) && (data[1] == SOF2))
	{
		crc = *(uint16_t*)(data + 3 + len);
		if(crc == crc16(data + 3, len))
			return true;
		else
			return false;
	}
	return false;
}

uint8_t* get_RF_frame(uint8_t* data, uint8_t len, uint8_t* lenout)
{
	uint16_t crc = crc16(data, len);

	buff[0] = SOF_RF_1;
	buff[1] = len;
	memcpy (&buff[2], data, len);
	buff[len+2] = crc;
	buff[len+3] = (crc>>8);

	*lenout = len + 4;
	return buff;
}

bool check_RF_frame_crc(uint8_t* data, uint8_t len)
{
	unsigned short crc;
	uint8_t debug_buffer[60] = {0} ;

	memcpy (debug_buffer,data, len);
	if((data[0] == SOF_RF_1) && (data[1] == len))
	{
		crc = *(uint16_t*)(data + 2 + len);
		if(crc == crc16(data + 2, len))
			return true;
		else
			return false;
	}
	return false;
}
