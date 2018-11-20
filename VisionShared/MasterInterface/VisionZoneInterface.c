/*
 * VisionZoneInterface.c
 *
 *  Created on: May 10, 2017
 *      Author: Francois Hattingh
 */

#include "VisionZoneInterface.h"
#include "Vision_HAL.h"

/**
 * @brief This function takes a valid Vision response or auto-forwarded message and interprets it.
 * @param vision_device reader_to copy data to reader
 * @param Zone_Message_Type
 * @return
 */
void send_zone_message(vision_device* reader_to, Zone_Message_Type* Zone_Message)
{
		uint8_t zone_data_length = 28;
		uint8_t zone_data[50] = {0};

//		zone_data[0] = 'z';
//		zone_data[1] = Zone_Message->Zone;
//		// FIXME: Find better way to obtain VID, UID
//		memcpy(zone_data + 2, &reader_to->UID, 4);
//		memcpy(zone_data + 6, &reader_to->VID, 2);
//		memcpy(zone_data + 8, &reader_to->SlaveID, 1);
//		memcpy(zone_data + 9, &Zone_Message->Distance, 4);
//		memcpy(zone_data + 13, &Zone_Message->Heading, 4);
//		zone_data[17] = 0xFF;
//		zone_data[18] = 0xFF;
//		zone_data[19] = 0xFF;

		zone_data[0] = 'M';
		zone_data[1] = reader_to->kind;
		// FIXME: Find better way to obtain VID, UID
		memcpy(zone_data + 2, &Zone_Message->UID, 4);
		zone_data[6] = 'z';
		zone_data[7] = Zone_Message->Zone;
		memcpy(zone_data + 8, &reader_to->UID, 4);
		memcpy(zone_data + 12, &reader_to->VID, 4);
		memcpy(zone_data + 16, &reader_to->SlaveID, 1);
		memcpy(zone_data + 17, &Zone_Message->Distance, 4);
		memcpy(zone_data + 21, &Zone_Message->Heading, 4);
		zone_data[25] = 0xFF;
		zone_data[26] = 0xFF;
		zone_data[27] = 0xFF;
		// ---- Send data over UART ----
		VisionSendCOM_message(reader_to->COM_Port, zone_data, zone_data_length);
}

/**
 * @brief This function gets the index of UID in the list and return index if found
 * 		  if not found it returns 0xFF.
 * @param UID of message to be added
 * @return index of UID
 */
uint8_t get_zone_message_index(uint32_t UID)
{
	// ---- Step through the zone message log ----
	for (uint8_t zm_index = 0; zm_index < num_zone_messages_total; zm_index++)
	{
		if (zone_messages_log[zm_index].UID == UID)
		{
			// ---- Return index ----
			return zm_index;
		}
	}
	// ---- Return if index not found ----
	return 0xFF ;
}

/**
 * @brief This function adds a new RF zone message to the list
 * @param Zone_Message_Type of type struct that contains all relevant information
 * @return
 */
void add_zone_message(Zone_Message_Type* Add_Zone_Message)
{
	uint8_t current_index = 0 ;
	// ---- Get index in the zone message log if already exists ----
	current_index = get_zone_message_index(Add_Zone_Message->UID);

	if (current_index == 0xFF)
	{
		// ---- Add message to list ----
		current_index = zone_messages_count++;
		zone_messages_log[current_index].UID = Add_Zone_Message->UID;
	}
	// ---- Update message in list ----
	zone_messages_log[current_index].Distance = Add_Zone_Message->Distance;
	zone_messages_log[current_index].Heading = Add_Zone_Message->Heading;
	zone_messages_log[current_index].Zone = Add_Zone_Message->Zone;
	zone_messages_log[current_index].Last_Updated = time_now();
}

/**
 * @brief This function resets an index in the zone message list
 * @param The index of the entry that must be reset
 * @return
 */
void reset_zone_message(uint8_t Zone_Message_Index)
{
	// ---- Reset information in the list ----
	zone_messages_log[Zone_Message_Index].UID = 0x00000000;
	zone_messages_log[Zone_Message_Index].Zone = 0;
	zone_messages_log[Zone_Message_Index].Distance = 0xFFFFFFFF;
	zone_messages_log[Zone_Message_Index].Heading = 0x00000000;
	zone_messages_log[Zone_Message_Index].Last_Updated = 0x00000000;
	zone_messages_log[Zone_Message_Index].Last_Sent = 0x00000000;
}

/**
 * @brief This function resets an index in the zone message list
 * @param The vision_device the information must be sent to (COM_Port)
 * @return The count of the list will be returned
 */
uint8_t vision_zone_process(vision_device* reader)
{
	uint8_t zero = 0 ;
	// ---- Loop through zone message list and sort ----
	for (uint8_t zmp_index = 0; zmp_index < zone_messages_count; zmp_index++)
	{
		// ---- If the entry is older than Remove_Zone_Message_Time remove entry ----
		if ((time_since(zone_messages_log[zmp_index].Last_Updated) > Remove_Zone_Message_Time) && (zone_messages_log[zmp_index].Last_Updated != 0))
		{
			zero++;
		}
		else
		{
			// ---- If the Last_sent zero (first instance) or If the Last_sent is older than Send_Zone_Message_Time----
			if ((zone_messages_log[zmp_index].Last_Sent == 0) || (time_since(zone_messages_log[zmp_index].Last_Sent) > Send_Zone_Message_Time))
			{
				// ---- Push RF message to module ----
				send_zone_message(reader, (zone_messages_log + zmp_index));
				zone_messages_log[zmp_index].Last_Sent = time_now();
			}
			// ---- Move index back the zero ----
			zone_messages_log[zmp_index - zero] = zone_messages_log[zmp_index];
		}
		// ---- If zero exists reset the index ----
		if (zero)
		{
			reset_zone_message(zmp_index);
		}
	}
	// ---- Decrease index count ----
	zone_messages_count -= zero;

	return zone_messages_count;
}

/**
 * @brief This function resets the zone message list
 * @param
 * @return
 */
void zone_message_init_log(void)
{
	// ---- Reset index count ----
	zone_messages_count = 0;

	// ---- Reset zone message list ----
	for (uint8_t zmi_index = 0; zmi_index < num_zone_messages_total; zmi_index++)
	{
		reset_zone_message(zmi_index);
	}
}
