/*
 * VisionTimeInterface.c
 *
 *  Created on: 28 Jan 2019
 *      Author: NeilPretorius
 */

#include "VisionTimeInterface.h"
#include "Vision_HAL.h"


void send_time_message(vision_device* reader_to, Time_Message_Type* Time_Message)
{
		uint8_t Time_data_length = 28;
		uint8_t Time_data[50] = {0};

		Time_data[0] = 'M';
		Time_data[1] = reader_to->kind;
		memcpy(Time_data + 2, &Time_Message->UID, 4);
		Time_data[6] = '#';
		Time_data[7] = Time_Message->seconds;
		Time_data[7] = Time_Message->minutes;
		Time_data[7] = Time_Message->hours;
		Time_data[7] = Time_Message->days;
		Time_data[7] = Time_Message->month;
		Time_data[7] = Time_Message->year;

		// ---- Send data over UART ----
		VisionSendCOM_message(reader_to->COM_Port, Time_data, Time_data_length);
}


