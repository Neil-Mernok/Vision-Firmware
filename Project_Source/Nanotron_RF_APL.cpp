/*
 * RF_APL.c
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

#include "Nanotron_RF_APL.h"
#include "RF_APL.h"
#include "app.h"
#include "Vision_Parameters.h"
#include "master_interface.h"


uint8_t nrf_buf[CONFIG_MAX_PACKET_SIZE*2];


extern "C" void Range_handler(_Transpondert* t, uint16_t dist, bool forward);


/**
 * @brief This message sends the tag ID information,as well as the tag name, if defined. 
 * @return
 */
int nApl_report_ID(uint8_t message_type)
{
	int len = 12;
	memset(nrf_buf, 0, STR_MAX + len);
	
	if(!(message_type == rf_ID_name || message_type == rf_ID_puls || message_type == rf_RangReq))
		return 1;
	
	nrf_buf[0] = vision_settings.tag_type;
	nrf_buf[1] = get_status_byte();
	nrf_buf[2] = (Vision_Status.Vbat/50);
	nrf_buf[3] = Firmware_rev | 0x80;
	*((uint16_t*) &nrf_buf[4]) = vision_settings.vehic_id;
	nrf_buf[6] = vision_settings.slave_id;
	nrf_buf[7] = vision_settings.usrParam;					// add dummy user param to RF. 
	*((uint16_t*) &nrf_buf[8]) = vision_settings.interval;	// Send interval so other tags know when to request a new range with forwarded ranges  
	nrf_buf[10] = 0;											// dummy bytes
	nrf_buf[11] = 0;											// dummy bytes
	
	//next comes the string
	if(vision_settings.name_str.len())
	{
		strncpy((char*)&nrf_buf[len], vision_settings.name_str, STR_MAX);
		len += STR_MAX;
	}
	return APL_SendMessage(0xFFFFFFFF, nrf_buf,len, message_type);
}


/**
 * @brief This message sends a range result globally so that other tags are aware of the distance. 
 * This is primarily so that the remote tag does not need to perform a second range request to get 
 * the same distance, but can be used by sniffer devices to know the distance between all nodes nearby. 
 * @return
 */
int nApl_range_callout(uint32_t remote_UID, uint16_t distance, uint16_t range_lockout)
{
	int len = 10;
	memset(nrf_buf, 0, len);
	
	*((uint32_t*) &nrf_buf[0]) = remote_UID;
	*((uint16_t*) &nrf_buf[4]) = distance;
	*((uint16_t*) &nrf_buf[6]) = range_lockout;
	nrf_buf[8] = 0;						// dummy byte
	nrf_buf[9] = 0;						// dummy byte
	
	return APL_SendMessage(0xFFFFFFFF, nrf_buf,len, rf_RangCal);
}


/**
 * @brief: takes an RF data-packet and decides what to do with it. 
 * @param data: buffer filled with data packet
 * @param len: length of the data-packet. 
 * @return	data packet type
 */
extern "C" int nApl_Parse_message(uint8_t* buffer, uint32_t sender, uint8_t len)
{
	_Transpondert* TR;
	_Q_MasterIF MIF;
//	bool brdcast;
	bool valid = true;
	uint32_t remote_tag;
	uint16_t lockout;
			
//	if(sender == 0xFFFFFFFF)
//		brdcast = true;
//	else
//		brdcast = false;
	
	// figure out what sort of message it is...
	switch (buffer[0])
	{
	case rf_ID_puls:							// Regular tag ID packet 
	case rf_ID_name:							// Tag name packet
	case rf_RangReq:							// range request message
		// add the tag details to the list
		TR = Transp_nRF_handler(buffer,sender, len);
		if (vision_settings.getActivities().forward_RF)
			Send_POD_toMaster(TR, CODE, 1);
		// if the message is a range request, add it to the list for processing. 
		if (buffer[0] == rf_RangReq)
		{
			Vision_Status.ranging_exclusion = time_now();						// another tag is busy with its range requests, so don't interrupt it. 
			if(TR->VehicleID != (uint32_t)vision_settings.vehic_id)					// don't range to the other tags on our vehicle.
			{
				if (vision_settings.getActivities().get_range_all)					// if we're supposed to range, get range
					add_ID_to_be_ranged(TR->UID);
				else if (vision_settings.getActivities().get_range_select)
				{
					if (transp_type_check(TR->type))								// if we're supposed to only range to some types of tags, check
						add_ID_to_be_ranged(TR->UID);
				}
			}
		}
	
		break;
	case rf_RangCal:
		remote_tag = *(uint32_t*)(&buffer[1]);
		// Check the range lockout time sent, and make sure we dont range before then 
		lockout = *(uint16_t*)(&buffer[7]);
		Vision_Status.ranging_exclusion = MAX(Vision_Status.ranging_exclusion, lockout + time_now());
						
		// check if the message is for me :)
		if(remote_tag == Vision_Status.UID)
		{
			TR = get_tag(sender);
			if (TR != NULL)
			{
				uint16_t distance =  *(uint16_t*)(&buffer[5]);
				Range_handler(TR, distance, true);
			}
		}
		
		break;
	
		// this is a remote slave response from an RF device. check if the request was sent through us, and if so forward response to master. 
	case rf_Respons:
		if (sender == Vision_Status.last_slave_id)	// we sent the request, so forward it. 
		{
			MIF.Master = CODE;						// this needs to go back to the UART/USB master, so indicate that it comes form code...
			memmove(&buffer[5], &buffer[1], len-1);
			memcpy(&buffer[1], &sender, 4);
			MIF.data = buffer;
			MIF.len = len+4;						// remove the 3 padding bytes, but allow the initial 'R' and UID to pass to the master. 
			push_to_master(MIF);
		}
		break;
		
		// this is a remote master message from another RF device. Save the Master UID and process command 
	case rf_MasterP:
		Vision_Status.last_master_id = sender;
		MIF.data = &buffer[1];
		MIF.len = len - 1;							// length is the RF length, minus the 5 bytes for UID and 'M' command and 3 padded bytes.
		MIF.Master = RF;
		parse_message_old(MIF);
		Vision_Status.Force_RF = time_now()+30000;	//keep the RF on as long as the master needs us...
		break;
		
		// this is data from a remote device. If the UID/VID matches ours, forward response to master. 
	case rf_Data:
		if (vision_settings.getActivities().accept_data)
		{
			// This is a remote data frame. check the target, it could be for me (UID), for us (VID) or not. 
			uint32_t dest = *(uint32_t*) &buffer[1];			// UID or VID of the remote tag.
			if (dest == Vision_Status.UID)
				buffer[4] = 'U';					// UID match
			else if (dest == (uint32_t) vision_settings.vehic_id)
				buffer[4] = 'V';					// VID match
			else if (dest == 0 || dest == 0xFFFFFFFF)
				buffer[4] = 'G';					// global message
			else
				break;								// not a match for us. 		

			buffer[3] = 'd';						// 'd' indicates its incoming data message
			MIF.Master = CODE;						// this needs to go back to the UART/USB master, so indicate that it comes form code...
			MIF.data = &buffer[3];					// data starts at buf[5], buf[4] is source indicator, buf[3] is message indicator.	
			MIF.len = len - 3;						// -1(start char 'D') -4 (UID) + 2 ('d' and dest marker 'U|V|G') 

			push_to_master(MIF);
		}
		break;

		// this is a remote slave response from a bootloading device. check if we're on the boot channel, and if so forward response to master. 
	case rf_Boot_R:
		//todo: check this handling...
		if (true/*boot_channel*/)	// we sent the request, so forward it. 
		{
			MIF.Master = CODE;					// this needs to go back to the UART/USB master, so indicate that it comes form code...
			MIF.data = buffer;
			MIF.len = len;
			push_to_master(MIF);
		}
		break;

	default:
		valid = false;
		break;
	}

	if (valid)
		Vision_Status.sts.RF_Working = true;	
	return valid ? 1:0;
}

/**
 * @brief This message sends the tag ID information,as well as the tag name. 
 * @return
 *
uint8_t nApl_report_name_var(uint8_t type, char* name)
{
	// don't send a name message if there is none. 
	if(name == 0)
		return Apl_report_ID(type);
	
	int len = strnlen(name, STR_MAX);
	
	if(len == 0)
		return Apl_report_ID(type);
	
	nrf_buf[0] = rf_ID_name;
	*((uint32_t*) &nrf_buf[1]) = Vision_Status.UID;

	nrf_buf[5] = type;
	nrf_buf[6] = get_status_byte();
	nrf_buf[7] = (Vision_Status.Vbat/50);
	nrf_buf[8] = Firmware_rev | 0x80;
	*((uint16_t*) &nrf_buf[9]) = vision_settings.vehic_id;
	nrf_buf[11] = vision_settings.slave_id;
	nrf_buf[12] = vision_settings.usrParam;	// add dummy user param to RF. 

	//next comes the string
	memcpy(&nrf_buf[13], name, len);
	if (len != STR_MAX)
		memset(&nrf_buf[13 + len], 0, STR_MAX - len);
	txSendPacket(nrf_buf, STR_MAX + 13);
	return 0;
}*/

/**
 * Send an RF ID packet. depending on the settings, this can be a PDS, ID or name message.
 *
void nApl_broadcast_ID(void)
{
	static int ID_count = 0;

	if (vision_settings.getActivities().broadcast_ID)
	{
		if ((vision_settings.getActivities().send_name) && ((ID_count & 1) == 1))
			Apl_report_name();
		else
		{
			if (vision_settings.getActivities().legacy_PDS)
				Apl_Send_PDS(vision_settings.tag_type);
			else
				Apl_report_ID(vision_settings.tag_type);
		}
		ID_count++;
	}
}*/

/**
 * @brief This sends a master message to a remote slave device, pointed to  by the UID in the message.
 * @param buffer message from master. 
 * @param len
 * @return
 */
uint8_t nApl_send_master_message(uint8_t* buffer, uint8_t len)
{
	uint32_t slaveUID = *((uint32_t*) &buffer[1]);
	Vision_Status.last_slave_id = slaveUID;
	
	len = MIN(len, 100);				/// make sure it fits in an RF packet
	APL_SendMessage(slaveUID, &buffer[5],len-5, rf_MasterP); 
	return 0;
}

/**
 * @brief This sends a replay to the remote master, with my UID indicating it was me they wanted.
 * @param buffer message to send to master. 
 * @param len length og master replay (excluding my UID and 'R')
 * @return
 */
uint8_t nApl_send_master_response(uint8_t* buffer, uint8_t len)
{
	len = MIN(len, 100);				/// make sure it fits in an RF packet
	APL_SendMessage(Vision_Status.last_master_id, buffer,len, rf_Respons); 
	return 0;
}

/**
 * @brief This sends data packet to a remote device, pointed to  by the UID/VID in the message.
 * @param buffer message from master. 
 * @param len
 * @return
 */
uint8_t nApl_send_data_message(uint8_t* buffer, uint8_t len)
{
	len = MIN(len, 100);			/// make sure it fits in an RF packet
	uint32_t dest = *((uint32_t*) &buffer[1]);
	if(dest == 0)
		dest = 0xFFFFFFFF;
	
	if(dest <= 0xFFFF)				/// this targets a specific VID, so broadcast glabally so all of that VID catch it. 
		dest = 0xFFFFFFFF;		

	/// other destinations (global and UID) are used directly (i.e. send globally=0xFFFFFFFF, or sent to secified UID) 
	APL_SendMessage(dest, &buffer[1],len-1, rf_Data);
	return 0;
}

uint8_t nApl_send_data_message(uint32_t dest, uint8_t* buffer, uint8_t len)
{
	len = MIN(len, 100);			/// make sure it fits in an RF packet
	memcpy(&nrf_buf[4], buffer, len);
	memcpy(&nrf_buf[0], &dest, 4);
	
	APL_SendMessage(dest, nrf_buf, len+4, rf_Data);
	return 0;
}

/**
 * @brief This sends a master boot message to a remote bootable slave device
 * @param buffer message from master. 
 * @param len
 * @return
 */
uint8_t nApl_master_boot_message(uint8_t* buffer, uint8_t len)
{
	txSendPacket(buffer, len);	 
	return 0;
}

/**
 * @ the purpose of this task is to figure out what to do with transponder packets after they come in.
 *  Each packet is logged if it is an ID or range packet, and depending on the application settings it is either
 *  ranged to, sent to the master or both. 
 * @param pvParameters
 */
_Transpondert* Transp_nRF_handler(uint8_t* buffer, uint32_t UID, uint8_t len)
{
	int list_item;
	static int first = 0;
	_Transpondert* T = NULL;

	if (first == 0)
	{
		clear_transp_log();
		first++;
	}

	if (UID == 0 || UID == 0xFFFFFFFF)
		return T;

	list_item = get_slot(UID);
	T = &transp_log[list_item];
	T->UID = UID;
	parse_nRF_into_tag(T, buffer, len);	
	re_insert_transp(T);
	return T;
}



extern "C" void parse_nRF_into_tag(_Transpondert* T, uint8_t* buffer, uint8_t len)
{
	T->type = buffer[1];
	T->status = buffer[2];
	T->volts = buffer[3];
	T->FirmwareRev = buffer[4] &0x7F;
	T->VehicleID = *((uint16_t*) &buffer[5]);
	T->SlaveID = buffer[6];
	T->group = buffer[7];
	T->RangePeriod = *((uint16_t*) &buffer[9]);
//	buffer[11] = 0;							// dummy bytes  
//	buffer[12] = 0;							// dummy bytes  
	T->last_seen = time_now();
	T->kind = Ranging;
	T->range_needed = 0;

#ifdef USE_TAG_NAME
	if (len > 13)
		strncpy(T->name, (char*)&buffer[13], STR_MAX);
#endif
}







//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
////static LF_message_type past_RSSIs[16];
//RssiTracker local_RSSI;
//LF_acknowledge LF_ack, LF_last;
//
//zone zone_from_rssi(int RSSI)
//{
//	zone z = zone_none;
//	if (RSSI >= (int) vision_settings.rssiCrit)
//		z = zone_crit;
//	else if (RSSI >= (int) vision_settings.rssiWarn)
//		z = zone_warn;
//	else if (RSSI >= (int) vision_settings.rssiPres)
//		z = zone_pres;
//	return z;
//}
//
///**
// * @brief generates warning outputs based on the zone.
// * @param c_zone
// */
//void Output_zone(zone c_zone)
//{
//	int time = 2;
//	bool buz = false, vib = false, led = false;
//	int lamp = 0;
//
//	if (vision_settings.getActivities().output_critical)
//	{
//		switch (c_zone)
//		{
//		case zone_crit:
//			SetLed(&LED1, Red, time);
//			led = true;
//			buz = true;
//			vib = true;
//			lamp = 10;
//			break;
//		case zone_warn:
//			SetLed(&LED1, Yellow, time);
//			led = true;
//			vib = true;
//			lamp = 2;
//			break;
//		case zone_pres:
//			SetLed(&LED1, Blue, time);
//			led = true;
//			vib = true;
//			buz = true;
//			lamp = 2;
//			break;
//		default:
//			break;
//		}
//
//#ifdef WARNING_OUTPUTS
////		if ((time_now() > LF_ack.block_till) || ((LFM.VehicleID != LF_ack.LF.VehicleID) && LF_ack.LF.VehicleID != 0xFFFF))
////		{
//		if (lamp)
//			SetGPO(&LAMP_OUT, lamp);
//		if (buz)
//			SetGPO(&BUZ_OUT, 10);
//		if (vib)
//			SetGPO(&VIB_OUT, 10);
//		if (led)
//			SetGPO(&LED_OUT, 10);
////		}
//#endif 
//	}
//}
//
//void Apl_handle_RSSI(LF_message_type LFM)
//{
//	/// log the RSSI and timestamp it. 
//	LFM.last_seen = time_now();
//	local_RSSI.add_RSSI(LFM);
//
//	int RSSI = local_RSSI.get_highest_RSSI(vision_settings.lf_filtr);
//
////	last_zone = current_zone;
//
//	current_zone = zone_from_rssi(RSSI);
//
//	// keep track of the most recent LF here. in case the button gets pressed
//	LF_last.LF = LFM;
//	LF_last.zone_last = current_zone;
//
////	if (last_zone == current_zone)
////	{
////		LF_ack.block_till = 0;
////		Output_zone(current_zone);
////	}
//}
//
///**
// * @brief this function most be called in regular intervals. 
// * checks the filtered RSSI and outputs it. 
// */
//void Apl_Checkzone_and_output(void)
//{
//	static int count = 0;
//
//	int RSSI = local_RSSI.get_highest_RSSI(vision_settings.lf_filtr);
//
//	current_zone = zone_from_rssi(RSSI);
//
//	if (last_zone != current_zone)
//		LF_ack.block_till = 0;
//
//	if (current_zone != zone_none)
//	{
//		int force = count % (int) vision_settings.ack_intv;		// generate a warning every so often
//		if ((time_now() > LF_ack.block_till) || ((LF_last.LF.VehicleID != LF_ack.LF.VehicleID) && LF_ack.LF.VehicleID != 0xFFFF))
//		{
//			Output_zone(current_zone);
//		}
//		else if (force == 0)
//		{
//			// force the output every few times, but only warning.
//			Output_zone(zone_warn);
//		}
//	}
//	last_zone = current_zone;
//
//	count++;
//}
//
//void Apl_acknowledge_LF(void)
//{
//	uint16_t vid_last = LF_last.LF.VehicleID;
//
//	LF_ack = LF_last;
//	LF_ack.block_till = time_now() + (int) vision_settings.ack_time;
//
//	// there is already a standing acknowledge, so make it a global vehicle ack. 
//	if (LF_ack.block_till > time_now() && LF_ack.LF.VehicleID != vid_last)
//		LF_ack.LF.VehicleID = 0xFFFF;
//}
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
////void Apl_handle_RF_RSSI(int RSSI)
////{
////	int time = 200;
////	static int lst_rssi = -1, age = 1000;
////	static uint32_t last_RSSI_seen = 0;
////
////	if (time_since(last_RSSI_seen) > age)
////		lst_rssi = -20;
////
////	if (RSSI >= lst_rssi)
////	{
////		if (RSSI > 200)
////			SetLed(&LED1, Red, time);
////		else if (RSSI > 150)
////			SetLed(&LED1, Yellow, time);
////		else if (RSSI > 100)
////			SetLed(&LED1, Green, time);
////		else if (RSSI > 50)
////			SetLed(&LED1, Cyan, time);
////		else if (RSSI >= 0)
////			SetLed(&LED1, Blue, time);
////		else
////			SetLed(&LED1, Off, time);
////
////		lst_rssi = RSSI;
////		last_RSSI_seen = time_now();
////	}
////}
////
////int get_highest_RSSI(void)
////{
////	int i, highest = -1;
////	int32_t too_old;
////
////	too_old = time_now() - RSSI_timeout;
////
////	for (i = 0; i < 16; i++)
////	{
////		if (past_RSSIs[i].last_seen > too_old)
////		{
////			if (past_RSSIs[i].RSSI > highest)
////				highest = past_RSSIs[i].RSSI;
////		}
////	}
////	return highest;
////}

