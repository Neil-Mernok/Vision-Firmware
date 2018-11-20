/*
 * GPS_APL.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#include "GPS_APL.h"
#include "Transponders.h"
#include "master_interface.h"

/**
 * @brief  uAPL_send_master_message, push master message
 * @param  Last updated value
 * @retval Size of the uBlox message including the checksum size
 */
uint8_t uAPL_send_master_message(uint8_t Last_Fix)
{
	_Q_MasterIF MIF;
	uint8_t buffer[60];
	// ---- rf_GPS_M message identifier ----
	buffer[0] = 'G';
	// ---- Copy information into transmit buffer ----
	memcpy(&buffer[1], &Vision_Status.GPS_Data.Longitude, 4);
	memcpy(&buffer[5], &Vision_Status.GPS_Data.Latitude, 4);
	memcpy(&buffer[9], &Vision_Status.GPS_Data.HorizontalAccuracy, 4);
	memcpy(&buffer[13], &Vision_Status.GPS_Data.VerticalAccuracy, 4);
	buffer[17] = Vision_Status.GPS_Data.FixType;
	buffer[18] = Vision_Status.GPS_Data.NumberOfSat;
	memcpy(&buffer[19], &Vision_Status.GPS_Data.Speed, 4);
	memcpy(&buffer[23], &Vision_Status.GPS_Data.SpeedAccuracy, 4);
	memcpy(&buffer[27], &Vision_Status.GPS_Data.HeadingVehicle, 4);
	memcpy(&buffer[31], &Vision_Status.GPS_Data.HeadingMotion, 4);
	memcpy(&buffer[35], &Vision_Status.GPS_Data.HeadingAccuracy, 4);
	memcpy(&buffer[39], &Vision_Status.GPS_Data.SeaLevel, 4);
	memcpy(&buffer[43], &Vision_Status.GPS_Data.flags, 2);
	memcpy(&buffer[45], &Vision_Status.GPS_Data._Date, 4);
	memcpy(&buffer[49], &Vision_Status.GPS_Data._Time, 4);
	buffer[53] = Last_Fix;

	// ---- Push buffer to master -----
	MIF.Master = CODE;
	MIF.data = buffer;
	MIF.len = 54;
	push_to_master(MIF);

	return true;
}

/**
 * @brief  uAPL_antenna_message, push master message
 * @param  None
 * @retval
 */
void uAPL_antenna_message(void)
{
	_Q_MasterIF MIF;
	uint8_t buffer[10];

	// ---- q message identifier ----
	buffer[0] = 'q';;

	// ---- Copy information into transmit buffer ----
	buffer[1] = Vision_Status.GPS_Data.antenna_status;

	// ---- Push buffer to master -----
	MIF.Master = CODE;
	MIF.data = buffer;
	MIF.len = 2;
	push_to_master(MIF);
}

/**
 * @brief  uAPL_overall_distance_message, push master message
 * @param  None
 * @retval
 */
void uAPL_overall_distance_message(void)
{
	_Q_MasterIF MIF;
	uint8_t buffer[20];

	// ---- rf_GPS_q message identifier ----
	buffer[0] = 'o';

	// ---- Copy information into transmit buffer ----
	memcpy(&buffer[1], &Vision_Status.GPS_Data.Speed, 4);
	memcpy(&buffer[5], &Vision_Status.GPS_Data.SpeedAccuracy, 4);
	memcpy(&buffer[9], &Vision_Status.GPS_Data.Distance, 4);
	memcpy(&buffer[13], &Vision_Status.GPS_Data.TotalDistance, 4);

	// ---- Push buffer to master -----
	MIF.Master = CODE;
	MIF.data = buffer;
	MIF.len = 17;
	push_to_master(MIF);
}

/**
 * @brief  uAPL_Get_Flag, parse the flags of uBlox module
 * @param  uint16_t flags received from uBlox module, uBlox_Flag needed
 * @retval uint8_t the value of the flag
 */
uint8_t uAPL_Get_Flag(uint16_t uBlox_Flag, _uBlox_Flags Flag)
{
	uint8_t Flag_Return;
	switch (Flag)
	{
		case GNSS_Fix_OK:
			Flag_Return = ((uint8_t) uBlox_Flag) & 0x01;
			break;
		case Differential_Corrections:
			Flag_Return = (((uint8_t) uBlox_Flag) & 0x02) >> 1;
			break;
		case Power_Management_State:
			Flag_Return = (((uint8_t) uBlox_Flag) & 0x1C) >> 2;
			break;
		case Heading_Valid:
			Flag_Return = (((uint8_t) uBlox_Flag) & 0x20) >> 5;
			break;
		case Carrier_Phase:
			Flag_Return = (((uint8_t) uBlox_Flag) & 0xC0) >> 6;
			break;
		default:
			break;
	}
	return Flag_Return;
}
