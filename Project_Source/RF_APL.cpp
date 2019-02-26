/*
 * RF_APL.c
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

#include "RF_APL.h"
#include "LF_APL.h"
#include "Global_Variables.h"
#include "master_interface.h"
#include "Vision_Parameters.h"

uint8_t RF_Buffer[64];
uint8_t ii;

///////////////////////////////////////////////////////////////////////////////////////////
////////////		CC1101 based functions		///////////////////////////////////////////

uint8_t get_status_byte()
{
	uint8_t dat = vision_settings.rf_power;
	if (vision_settings.getActivities().send_name)
		dat |= 0x08;
	if (Vision_Status.LF_current_zone != zone_none)
		dat |= 0x10;
	if (Vision_Status.sts.EXT_Power)
		dat |= 0x80;
	if(Vision_Status.exclusion > time_now())
		dat |= 0x40;
	if(Vision_Status.sts.RF_Receiving)
		dat |= 0x20;
	return dat;
}

uint8_t get_status_byte2()
{
	uint8_t dat = 0;
//	if (vision_settings.getActivities().send_name)
//		dat |= 0x08;
//	if (Vision_Status.LF_current_zone != zone_none)
//		dat |= 0x10;
//	if (Vision_Status.sts.EXT_Power)
//		dat |= 0x80;
	if(Vision_Status.Reverse>0)
		dat |= 0x02;
	if(Vision_Status.ManTagAck)
		dat |= 0x01;
	return dat;
}

/**
 * OLD PDS type message
 */
void Apl_Send_PDS(uint8_t type)
{
	*((uint32_t*) RF_Buffer) = Vision_Status.UID;

	RF_Buffer[4] = type;
	RF_Buffer[5] = get_status_byte();
	RF_Buffer[6] = (Vision_Status.Vbat/50);
	RF_Buffer[7] = Firmware_rev | 0x80;

	txSendPacket(RF_Buffer, rf_legacy_PDS_size);
}

int Rf_Info_Packet(uint8_t type, uint8_t Identifier)
{
	RF_Buffer[0] = Identifier;
	RF_Buffer[1] = Vision_Status.kind;
	*((uint32_t*) &RF_Buffer[2]) = Vision_Status.UID;
	RF_Buffer[6] = type;
	RF_Buffer[7] = vision_settings.MernokAsset_Groups[type-1];
	RF_Buffer[8] = get_status_byte();
	RF_Buffer[9] = get_status_byte2();
	RF_Buffer[10] = (Vision_Status.Vbat/50);
	RF_Buffer[11] = Firmware_rev | 0x80;
	RF_Buffer[12] = vision_settings.Product_ID;
	if(type == Person)
	{
		*((uint32_t*) &RF_Buffer[13]) = Vision_Status.UID;
		vision_settings.vehic_id._value = Vision_Status.UID;
	}
	else
		*((uint32_t*) &RF_Buffer[13]) = vision_settings.vehic_id;

	RF_Buffer[17] = vision_settings.slave_id;
	RF_Buffer[18] = vision_settings.V_length;
	RF_Buffer[19] = vision_settings.V_width;
	RF_Buffer[20] = Vision_Status.Stopping_dist;
	*((uint16_t*) &RF_Buffer[21]) = Vision_Status.Speed/10;

	return 23;
}

void Rf_GPS_info(uint8_t Array_start)
{
	*((int32_t*) &RF_Buffer[Array_start]) = Vision_Status.GPS_Data.Longitude;
	*((int32_t*) &RF_Buffer[Array_start+=4]) = Vision_Status.GPS_Data.Latitude;
	*((int32_t*) &RF_Buffer[Array_start+=4]) = Vision_Status.GPS_Data.SeaLevel;

	*((uint16_t*) &RF_Buffer[Array_start+=4]) = (Vision_Status.GPS_Data.VerticalAccuracy/10>0xFFFF) ? 0xFFFF : Vision_Status.GPS_Data.VerticalAccuracy/10;

	*((uint16_t*) &RF_Buffer[Array_start+=4]) = (Vision_Status.GPS_Data.HorizontalAccuracy/10>0xFFFF) ? 0xFFFF : Vision_Status.GPS_Data.HorizontalAccuracy/10;

	*((uint16_t*) &RF_Buffer[Array_start+=4]) = (Vision_Status.GPS_Data.HeadingVehicle/10>0xFFFF) ? 0xFFFF : Vision_Status.GPS_Data.HeadingVehicle/10;

	RF_Buffer[Array_start+=4] = Vision_Status.GPS_Data.FixType;
	RF_Buffer[Array_start+=4] =	Vision_Status.GPS_Data.FixAge;
}

/**
 * This sends all regular Tag information. New extended ID message
 * @return
 */

uint8_t Apl_report_ID(uint8_t type)
{
	uint8_t packet_size = 0 ;
	uint8_t info_packet_size;

	// ---- Packet Identifier ----
	if (Vision_Status.kind == Pulse_GPS)
		packet_size = rf_ID_Pulse_GPS_size;
	else
		packet_size = rf_ID_Pulse_size;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(type, rf_ID_puls);

	// ---- GPS Coordinates ----
	Rf_GPS_info(info_packet_size);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}

/**
 * @brief This message sends the tag ID information,as well as the tag name. 
 * @return
 */
uint8_t Apl_report_name()
{
	uint8_t info_packet_size;
	// don't send a name message if there is none. 
	if(vision_settings.name_str.len() == 0)
		return Apl_report_ID(Vision_Status.TagTypeHolder);

	uint8_t packet_size = 0 ;

	if (Vision_Status.kind == Pulse_GPS)
		packet_size = rf_ID_Pulse_GPS_size;
	else
		packet_size = rf_ID_Pulse_size;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(Vision_Status.TagTypeHolder, rf_ID_name);

	// ---- GPS Coordinates ----
	Rf_GPS_info(info_packet_size);


	// ---- TAG Name ----
	int len = vision_settings.name_str.len();
	if(len > STR_MAX)
		memcpy(&RF_Buffer[packet_size], vision_settings.name_str, STR_MAX);
	else
		memcpy(&RF_Buffer[packet_size], vision_settings.name_str, len);

	if (len <= STR_MAX)
		memset(&RF_Buffer[packet_size + len], 0, STR_MAX - len);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size + STR_MAX);

	return 0;
}

/**
 * @brief This message sends the tag ID information,as well as the tag name. 
 * @return
 */
uint8_t Apl_report_name_var(uint8_t type, char* name)
{
	uint8_t packet_size = rf_ID_Pulse_size ;
	// don't send a name message if there is none. 
	int len = strnlen(name, STR_MAX);

	if(name == 0 || len == 0)
		return Apl_report_ID(type);


	// ---- TAG Information ----
	Rf_Info_Packet(type, rf_ID_name);

	// ---- TAG Name ----
	memcpy(&RF_Buffer[packet_size], name, len);
	if (len != STR_MAX)
		memset(&RF_Buffer[packet_size + len], 0, STR_MAX - len);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size + STR_MAX);

	return 0;
}

/**
 * Send an RF ID packet. depending on the settings, this can be a PDS, ID or name message.
 */
void Apl_broadcast_ID(void)
{
	static int ID_count = 0;

	if ((vision_settings.getActivities().send_name) )//&& ((ID_count & 1) == 1)
		Apl_report_name();
	else
	{
		if (vision_settings.getActivities().legacy_PDS)
			Apl_Send_PDS(Vision_Status.TagTypeHolder);
		else
			Apl_report_ID(Vision_Status.TagTypeHolder);
	}
	ID_count++;
}


uint8_t Apl_broadcast_Time(void)
{

	uint8_t packet_size = 0 ;
	uint8_t info_packet_size;
	packet_size = rf_Time_mess_size;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(Vision_Status.TagTypeHolder, rf_Time);

	//----  Over-write with Time data ----
	Get_RTCTime();
	RF_Buffer[info_packet_size] = Vision_Status.DateTime.Seconds;
	RF_Buffer[info_packet_size+=1] = Vision_Status.DateTime.Minutes;
	RF_Buffer[info_packet_size+=1] = Vision_Status.DateTime.Hours;

	Get_RTCDate();
	RF_Buffer[info_packet_size+=1] = Vision_Status.DateTime.Date;
	RF_Buffer[info_packet_size+=1] = Vision_Status.DateTime.Month;
	RF_Buffer[info_packet_size+=1] = Vision_Status.DateTime.Year;

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}

uint8_t Apl_report_LF(LF_message_type LF)
{
	uint8_t packet_size = 0 ;
	uint8_t info_packet_size;
	packet_size = rf_LF_resp_size;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(Vision_Status.TagTypeHolder, rf_LF_resp);

	//----  Over-write with LF data ----
	*((uint16_t*) &RF_Buffer[info_packet_size]) = LF.VehicleID;
	RF_Buffer[info_packet_size+=2] = LF.SlaveID;
	RF_Buffer[info_packet_size+=1] = LF.RSSI;

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}

uint8_t Apl_sync_LF()
{
	uint8_t packet_size = 0 ;
	uint8_t info_packet_size = 0;
	packet_size = rf_LF_send_size;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(Vision_Status.TagTypeHolder, rf_LF_send);
	RF_Buffer[info_packet_size] = vision_settings.lf_power;

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}

/**
 * @brief This sends a master message to a remote slave device, pointed to  by the UID in the message.
 * @param buffer message from master. 
 * @param len (minimum 10 bytes)
 * @return
 */
uint8_t Apl_send_master_message(uint8_t* buffer, uint8_t len)
{
	uint8_t packet_size = len;

	packet_size = MIN(packet_size, 54); // make sure it fits in an RF packet

	// ---- Copy data ----
	memcpy(RF_Buffer, buffer, len);

	Vision_Status.last_slave_id = *((uint32_t*) &RF_Buffer[2]);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size + 2);

	return 0;
}

/**
 * @brief This sends a replay to the remote master, with my UID indicating it was me they wanted.
 * @param buffer message to send to master. 
 * @param len length og master replay (excluding my UID and 'R')
 * @return
 */
uint8_t Apl_send_master_response(uint8_t* buffer, uint8_t len)
{
	DelayUs(600);
	uint8_t packet_size = len + 6 ;

	packet_size = MIN(packet_size, 54);				// make sure it fits in an RF packet

	// ---- Packet Identifier ----
	RF_Buffer[0] = rf_Respons;
	RF_Buffer[1] = Vision_Status.kind;

	*((uint32_t*) &RF_Buffer[2]) = Vision_Status.UID;

	memcpy(&RF_Buffer[6], buffer, len);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size + 2);
	return 0;
}

/**
 * @brief This sends data packet to a remote device, pointed to by the UID/VID in the message.
 * @param buffer message from master. 
 * @param len (minimum 5 bytes)
 * @return
 */
uint8_t Apl_send_data_message(uint8_t* buffer, uint8_t len)
{
	uint8_t packet_size = len;

	packet_size = MIN(packet_size, 56);

	// ---- Copy data into buffer ----
	memcpy(RF_Buffer, buffer, len);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}

/**
 * @brief This sends a master boot message to a remote bootable slave device
 * @param buffer message from master. 
 * @param len
 * @return
 */
uint8_t Apl_master_boot_message(uint8_t* buffer, uint8_t len)
{
	DelayUs(600);				// there needs to be a small delay. The new L4 boards seem to send boot packets out too quickly
	txSendPacket(buffer, len);	 
	return 0;
}

///**
// * This sends all GPS coordinate information
// * @return
// */
uint8_t Apl_report_GPS_Coordinates(void)
{
	uint8_t info_packet_size;
	uint8_t packet_size = 0 ;
	packet_size = rf_GPS_Coordinates_size;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(Vision_Status.TagTypeHolder, rf_GPS_C);

	// ---- GPS Information ----
	Rf_GPS_info(info_packet_size);

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}

uint8_t Apl_report_Distress(uint8_t distressByte)
{
	uint8_t packet_size = 0 ;
	uint8_t info_packet_size;
	packet_size = rf_Distress_broadcast;

	// ---- TAG Information ----
	info_packet_size = Rf_Info_Packet(Vision_Status.TagTypeHolder, rf_Distress);

	//----  Over-write with LF data ----
	RF_Buffer[info_packet_size] = distressByte;
	*((int32_t*) &RF_Buffer[info_packet_size+=1]) = Vision_Status.GPS_Data.Longitude;
	*((int32_t*) &RF_Buffer[info_packet_size+=4]) = Vision_Status.GPS_Data.Latitude;
	*((int32_t*) &RF_Buffer[info_packet_size+=4]) = Vision_Status.GPS_Data.SeaLevel;
	RF_Buffer[info_packet_size+=4] = Vision_Status.GPS_Data.FixType;
	RF_Buffer[info_packet_size+=4] =	Vision_Status.GPS_Data.FixAge;

	// ---- Transmit RF packet ----
	txSendPacket(RF_Buffer, packet_size);

	return 0;
}


void Apl_Parse_message(uint8_t* buffer, int len, uint8_t RSSI, bool boot_channel)
{
	_Transpondert* TR;
	_Q_MasterIF MIF;
	uint32_t UID;
	bool valid = true;

	if (len == 8) // this is a legacy PDS/ presence message
	{
		TR = Transp_RF_handler(buffer, RSSI, len);
		if (vision_settings.getActivities().forward_RF)
			Send_POD_toMaster(TR, CODE, 1);		
	}
	else		// some other message. interpret it. 
	{
		switch (buffer[0])
		{
		case rf_LF_send:
			Vision_Status.LastLF_TX = time_now();	// this is an LF sync message. log the LF time and treat as any other message 
		case rf_ID_puls:							// Regular Pulse tag ID packet 
		case rf_ID_name:
			if (len == rf_LF_send_size || len == rf_ID_Pulse_size || len == (rf_ID_Pulse_size + STR_MAX) || len == rf_ID_Pulse_GPS_size || len == (rf_ID_Pulse_GPS_size + STR_MAX))
			{
				// add the tag details to the list
				TR = Transp_RF_handler(buffer, RSSI, len);

				if (vision_settings.getActivities().forward_RF)
					Send_POD_toMaster(TR, CODE, 1);
				// if the message is LF response, capture it and store the LF alert holder. 
				if(buffer[0] == rf_LF_send)
				{
					Vision_Status.LF_alert.last_LF = time_now();
					Vision_Status.LF_alert.SlaveID = TR->SlaveID;
					Vision_Status.LF_alert.VehicleID = TR->VehicleID;
				}
			}
			break;
			// this is a Pulse response to a LF field. 
		case rf_LF_resp:
			if (len == rf_LF_resp_size)
			{
				TR = Transp_RF_handler(buffer, RSSI, len);
				if (vision_settings.getActivities().forward_RF)
					Send_POD_toMaster(TR, CODE, 2);
				else if (vision_settings.getActivities().forward_dists)
					Send_POD_toMaster(TR, CODE, 2);


				//////  Todo: This can still be improved by including filter struct in each list item.
				//send RSSI in for to the RSSI watcher task
				pipe_put(&LF_RSSI.p, &(TR->LF));
				// send the current RSSI to the display/LED processor
#ifdef use_HMI
				pipe_put(&HMI.p, &(TR->LF.RSSI));
#endif
			}
			break;
			// LF sync message. 
		case rf_MasterP:							// this is a master command from a reader device. 
			UID = *(uint32_t*) &buffer[2];			// UID of the requested tag.
			if (UID == Vision_Status.UID)			// this is for me! Parse the message. 
			{
				uint8_t rf_message_length = len - 8;				// -1(start char 'M') -1(_kind) -4 (UID) -2 (pad bytes)
				parse_message(&buffer[6], rf_message_length, RF);
				Vision_Status.Force_RF = time_now() + 30000;		//keep the RF on as long as the master needs us...
			}
			break;

			// this is a remote slave response from an RF device. check if the request was sent through us, and if so forward response to master. 
		case rf_Respons:
			UID = *(uint32_t*) &buffer[2];			// UID of the remote tag.
			if (UID == Vision_Status.last_slave_id)	// we sent the request, so forward it. 
			{
				MIF.Master = CODE;					// this needs to go back to the UART/USB master, so indicate that it comes form code...
				MIF.data = buffer;
				MIF.len = len - 2;					//  -2 (pad bytes), but allow the initial 'R', kind and UID to pass to the master.
				push_to_master(MIF);
			}
			break;

			// this is data from a remote device. If the UID/VID matches ours, forward response to master. 
		case rf_Data:
			if (vision_settings.getActivities().accept_data)
			{
				UID = *(uint32_t*) &buffer[2];			// UID or VID of the remote tag.
				if (UID == Vision_Status.UID)
					buffer[4] = 'U';					// UID match
				else if (UID == (uint32_t) vision_settings.vehic_id)
					buffer[4] = 'V';					// VID match
				else if (UID == 0)		
					buffer[4] = 'G';					// global message
				else 
					break;								// not a match for us. 		

				buffer[3] = 'd';						// 'd' indicates its incoming data message
				MIF.Master = CODE;						// this needs to go back to the UART/USB master, so indicate that it comes form code...
				MIF.data = &buffer[3];					// data starts at buf[5], buf[4] is source indicator, buf[3] is message indicator.	
				MIF.len = len - 4;						// -1(start char 'D') -1(_kind) -4 (UID) + 2 ('d' and dest marker 'U|V|G')

				push_to_master(MIF);
			}
			break;

			// this is a remote slave response from a bootloading device. check if we're on the boot channel, and if so forward response to master. 
		case rf_Boot_R:
			if (boot_channel)	// we sent the request, so forward it. 
			{
				MIF.Master = CODE;					// this needs to go back to the UART/USB master, so indicate that it comes form code...
				MIF.data = buffer;
				MIF.len = len;
				DelayUs(600);
				push_to_master(MIF);
			}
			break;

		case rf_GPS_C:
			if (len == rf_GPS_Coordinates_size)
			{
				// add the tag details to the list
				TR = Transp_RF_handler(buffer, RSSI, len);
				if (vision_settings.getActivities().forward_RF)
					Send_POD_toMaster(TR, CODE, 1);
			}
			break;

		case rf_Time:
			if (len == rf_Time_mess_size)
			{
				// add the tag details to the list
				TR = Transp_RF_handler(buffer, RSSI, len);
				if (vision_settings.getActivities().forward_RF)
					Send_POD_toMaster(TR, CODE, 1);
			}
			break;

		case rf_Distress:
			if (len == rf_Distress_broadcast)
			{
				// add the tag details to the list
				TR = Transp_RF_handler(buffer, RSSI, len);
				if (vision_settings.getActivities().forward_RF)
					Send_POD_toMaster(TR, CODE, 1);
			}
			break;

		default:
			valid = false;
			break;
		}
	}
	if (valid)
		Vision_Status.sts.RF_Working = true;
}

/**
 * @ the purpose of this task is to figure out what to do with transponder packets after they come in.
 *  Each packet is logged if it is an ID or range packet, and depending on the application settings it is either
 *  ranged to, sent to the master or both. 
 * @param pvParameters
 */
_Transpondert* Transp_RF_handler(uint8_t* buffer, uint8_t RSSI, uint8_t len)
{
	int list_item;
	uint32_t UID = 0;
	static int first = 0;
	_Transpondert* T = NULL;

	if (first == 0)
	{
		clear_transp_log();
		first++;
	}

	if (len == 8) // this is a legacy PDS/ presence message
	{
		UID = *(uint32_t*) &buffer[0];
	}
	else
	{
		// some other message. grab the UID so we can get/assign a slot. 
		UID = *((uint32_t*) (&buffer[2]));
	}
	if (UID == 0)
		return T;

	list_item = get_slot(UID);
	T = &transp_log[list_item];
	parse_RF_into_tag(T, buffer, RSSI, len);	
	re_insert_transp(T);
	return T;
}

void parse_RF_into_tag(_Transpondert* T, uint8_t* buffer, uint8_t RSSI, uint8_t len)
{
	if (len == 8) // this is a legacy PDS/ presence message
	{
		T->UID = *(uint32_t*) &buffer[0];
		T->type = buffer[4];
		T->group = vision_settings.MernokAsset_Groups[(uint8_t)buffer[4]-1];
		T->status = buffer[5];
		T->volts = buffer[6];
		if (buffer[7] > 2)
			T->kind = Pulse;
		else
			T->kind = PDS;

		T->rssi = RSSI;
		T->last_seen = time_now();
		T->LF.RSSI = -1;
		T->Dist = 0;		
	}
	else		// some other message. interpret it. 
	{

		// ---- TAG Information ----
		T->rssi = RSSI;
		T->kind = (_kind)(buffer[1]);
		T->UID = *((uint32_t*) (&buffer[2]));
		T->type = buffer[6];
		T->group = buffer[7];
		T->last_seen = time_now();
		T->status = buffer[8];
		T->ManTagAck = buffer[9]&0x01;
		T->Reverse = (buffer[9]&0x02)>>1;
		T->volts = buffer[10];
		T->FirmwareRev = (buffer[11] & 0x7F);
		T->ProductID = buffer[12];
		T->VehicleID = *((uint32_t*) (&buffer[13]));
		T->SlaveID = buffer[17];
		T->V_lenght = buffer[18];
		T->V_Width = buffer[19];
		T->Stopping_dist = buffer[20];
		T->Speed = *((uint16_t*) (&buffer[21]))*10;

		switch (buffer[0])
		{
		case rf_LF_resp:			// ---- LF response message ----

			// ---- LF Information ----
			T->LF.VehicleID = *((uint16_t*) (&buffer[23]));
			T->LF.SlaveID = buffer[25];
			T->LF.RSSI = buffer[26];
			T->LF.last_LF = time_now();
			T->Dist = 35 - T->LF.RSSI;
			T->range_needed = 0;
			break;

		case rf_GPS_C:				// ---- GPS coordinates broadcast message ----

			// ---- GPS Information ----
			T->GPS_Data.Longitude = *((int32_t*) &buffer[18]);
			T->GPS_Data.Latitude = *((int32_t*) &buffer[22]);
			T->GPS_Data.SeaLevel = *((int32_t*) &buffer[26]);
			T->GPS_Data.VerticalAccuracy = *((uint16_t*) &buffer[30])*10;
			T->GPS_Data.HorizontalAccuracy = *((uint16_t*) &buffer[32])*10;
			T->GPS_Data.HeadingVehicle = *((int16_t*) &buffer[34])*10;
			T->GPS_Data.Speed = *((int16_t*) &buffer[36])*10;
			T->GPS_Data.FixType = buffer[38];
			T->GPS_Data.FixAge = buffer[39];
			break;

		case rf_ID_name:			// ---- Name broadcast ID message ----
#ifdef USE_TAG_NAME
			strncpy(T->name, (char*)&buffer[len - STR_MAX], STR_MAX);
#endif
		case rf_ID_puls:			// ---- Broadcast ID message ----

			if (T->kind == Pulse_GPS)
			{

				// ---- GPS Information ----
				T->GPS_Data.Longitude = *((int32_t*) &buffer[23]);
				T->GPS_Data.Latitude = *((int32_t*) &buffer[27]);
				T->GPS_Data.SeaLevel = *((int32_t*) &buffer[31]);
				T->GPS_Data.VerticalAccuracy = *((uint16_t*) &buffer[35])*10;
				T->GPS_Data.HorizontalAccuracy = *((uint16_t*) &buffer[37])*10;
				T->GPS_Data.HeadingVehicle = *((uint16_t*) &buffer[39])*10;
//				T->GPS_Data.Speed = *((int16_t*) &buffer[38])*10;
				T->GPS_Data.FixType = buffer[41];
				T->GPS_Data.FixAge = buffer[42];

			}
			else
			{
				T->GPS_Data.Longitude = 0;
				T->GPS_Data.Latitude = 0;
				T->GPS_Data.SeaLevel = 0;
				T->GPS_Data.VerticalAccuracy = 0;
				T->GPS_Data.HorizontalAccuracy = 0;
//				T->GPS_Data.Speed = 0;
				T->GPS_Data.HeadingVehicle = 0;
				T->GPS_Data.FixType = 0;
				T->GPS_Data.FixAge = 0xFF;
			}

		case rf_LF_send:			// ---- LF Sync message ----

			// ---- TAG Information ----
			T->Dist = 0;
			break;

		case rf_Time:			// ---- Time message ----

			// ---- TAG Information ----
			T->Seconds = buffer[23];
			T->Minutes = buffer[24];
			T->Hours = buffer[25];
			T->Day = buffer[26];
			T->Month = buffer[27];
			T->Year = buffer[28];

			break;

		case rf_Distress:			// ---- Time message ----

			// ---- TAG Information ----
			T->Distress = buffer[23];
			T->GPS_Data.Longitude = *((int32_t*) &buffer[24]);
			T->GPS_Data.Latitude = *((int32_t*) &buffer[28]);
			T->GPS_Data.SeaLevel = *((int32_t*) &buffer[32]);
			T->GPS_Data.FixType = buffer[36];
			T->GPS_Data.FixAge = buffer[37];

			break;

		default:
			break;
		}
	}
}

//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


