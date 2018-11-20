/*
 * MasterTagReader.c
 *
 *  Created on: Jul 18, 2015
 *      Author: Kobus
 */

#include "MasterTagReader.h"

inline uint32_t parse_message_UID(uint8_t* data)
{
	return *(uint32_t*) (data + 1 );
}

void parse_message_into_TAG(_Transpondert* T, uint8_t* data, int len)
{
	T->UID = parse_message_UID(data);

	if (len == 8)
	{
		if (data[0] == 'p') // shortened tag poll response
		{
			T->type = data[5];
			T->last_seen = time_now() + data[6] * 1000;
			T->kind = data[7];
			T->LF.RSSI = -1;
			T->Dist = 0;
		}
		else if (data[0] == 1) // shortened auto-forward rf ping
		{
			T->type = data[5];
			T->rssi = data[6];
			T->last_seen = time_now();
			T->kind = data[7];
			T->LF.RSSI = -1;
			T->Dist = 0;
		}
		else if (data[0] == 2) // shortened auto-forward lf response
		{
			T->type = data[5];
			T->LF.RSSI = data[6];
			T->LF.last_LF = time_now();
			T->LF.SlaveID = (data[7] & 0x0F);
			T->last_seen = time_now();
			//			T->kind = Pulse_GPS;		//TODO: Franna to explain
			//			T->LF.VehicleID = (data[7] & 0x0F) ? our_vid : 0; // TODO: make this equal to our VID if the bit is set, else 0
		}
	}
	else if (len >= 20)
	{
		T->type = data[5];
		T->volts = data[6];
		T->last_seen = MAX((long )time_now() - data[7] * 1000, 0);
		T->kind = data[8];
		T->status = data[9];

		T->VehicleID = *(uint32_t*) (data + 10);
		T->SlaveID = data[14];

		T->LF.VehicleID = *(uint16_t*) (data + 15);
		T->LF.SlaveID = data[17];

		T->rssi = data[18];
		T->LF.RSSI = data[19];

		T->group = data[20];

		if (T->kind == Ranging)
		{
			T->Dist = *(uint16_t*) (data + 21);
			T->last_ranged = MAX((long )time_now() - data[21] * 1000, 0);
			T->rssi = 0;
		}

		T->LF.last_LF = MAX((long )time_now() - data[21] * 1000, 0);
		T->FirmwareRev = 0x7F & data[22];
		T->ManTagAck = data[23];
		T->Reverse = data[24];
		T->V_lenght = data[25];
		T->V_Width = data[26];
		T->Stopping_dist = data[27];

		if (T->kind == Pulse)
		{
			if (data[21] != 0xFF)
			{
				T->LF.RSSI++;
			}
#ifdef USE_TAG_NAME
			if(len > 28)
			{
				strncpy(T->name, (char*)&data[28], MIN(STR_MAX, len-28));
			}
#endif
		}

		if (T->kind == Pulse_GPS)
		{

			memcpy(&T->GPS_Data.Longitude, data + 28, 4);
			memcpy(&T->GPS_Data.Latitude, data + 32, 4);
			memcpy(&T->GPS_Data.VerticalAccuracy, data + 36, 4);
			memcpy(&T->GPS_Data.HorizontalAccuracy, data + 40, 4);
			memcpy(&T->GPS_Data.Speed, data + 44, 4);
			memcpy(&T->GPS_Data.HeadingVehicle, data + 48, 4);
			memcpy(&T->GPS_Data.FixType, data + 52, 1);
			memcpy(&T->GPS_Data.FixAge, data + 53, 1);
			memcpy(&T->GPS_Data.SeaLevel, data + 54, 4);

#ifdef USE_TAG_NAME
			if(len > 58)
			{
				strncpy(T->name, (char*)&data[58], MIN(STR_MAX, len-58));
			}
#endif
		}
	}
}

