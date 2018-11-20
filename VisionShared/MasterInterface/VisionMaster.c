/*
 * VisionMaster.cpp
 *
 *  Created on: Jul 17, 2015
 *      Author: Kobus
 */

#include "VisionMaster.h"
#include "framing.h"
#include "Vision_HAL.h"

/***********************
 * Global variables
 ***********************/
uint8_t message[65];
uint8_t remote_message[60];					// inbound data from a remote tag via data forwarding. introduced V11.

vision_device Readers[Reader_input_buffer_size];	//Added CANTag

static int bytes_available(vision_device* reader)
{
	int bytes_available;
	int vision_WRptr = Vision_GetWRptr(reader);
	bytes_available = VISION_BUF_MASK & (vision_WRptr - reader->vision_RDptr);
	return bytes_available;
}

static void consume_bytes(int count, vision_device* reader)
{
	reader->vision_RDptr = (VISION_BUF_MASK) & (reader->vision_RDptr+count);				////**** check how many byts need to be consumed....
}

static uint8_t getbyte(int offset, vision_device* reader)
{
	return reader->vision_inbuf[(VISION_BUF_MASK) & (reader->vision_RDptr + offset)];\
}

uint8_t* vision_get_message(int len, vision_device* reader)
{
	int i;

	if (Vision_GetWRptr(reader) > reader->vision_RDptr)	// we're in the middle of the buffer, no overflow.
		return &reader->vision_inbuf[(VISION_BUF_MASK) & (reader->vision_RDptr)];	// just return the pointer
	else									// the message overflows the buffer. 
	{
		for(i=0; i<len+6; i++)
			message[i] = getbyte(i, reader);
		return message;
	}
}

// Called from CAN interrupt to signal that a CAN packet has been received.	//added CANTag
void Vision_CAN_packet_handler(uint8_t* data, uint8_t len, bool Last, vision_device* reader)
{

	memcpy(&reader->DataIn[reader->DataInPtr], data, len);
	reader->DataInPtr += len;

	if (Last)		// packet was signaled as last/only.
	{
		// interpret message from master
		vision_parse_message(reader, reader->DataIn, reader->DataInPtr);
		reader->DataInPtr = 0;
	}
	else
	{
		len++;			/// just catch the debugger
	}
}

// this should be run continually to grab the data from the vision device. //added CANTag
int vision_process(vision_device* reader)
{
	uint8_t len;
	uint8_t* data;
	static int messages = 0;

	// make sure we have a buffer before we continue.
	if(reader->vision_inbuf == NULL)
		return messages;

	while (bytes_available(reader) > 7)				// 7 bytes is minimum command = 6 packet bytes with 1 byte instruction
	{
		if ((getbyte(0, reader) == SOF1) && (getbyte(1, reader) == SOF2))
		{
			len = getbyte(2, reader);					// read the message length;
			if (bytes_available(reader) < len + 6)	// check if the full message has arrived yet.
				break;
			else
			{
				// now we know that full a message has arrived.
				data = vision_get_message(len, reader);
				if (check_frame_crc(data, len))
				{
					// now we have a confirmed CRC checked message. just process it.  
					vision_parse_message(reader, data + 3, len);
					consume_bytes(len + 6, reader);			// move the read pointer along.
					messages++;
				}
				else
					consume_bytes(2, reader);				// something went wrong, just carry on checking for a new message
			}
		}
		else
			consume_bytes(1, reader);					// consume a byte and move the pointers
	}
	return messages;
}

/**
 * @brief This fucntion takes a valid Vision response or autoforwarded messagem and interprets it.
 * @param data
 * @param len
 * @return
 */
int vision_parse_message(vision_device* reader, uint8_t* data, int len)
{
	uint32_t UID = 0;
	uint8_t message = data[0];				// grab the message identifier
	int list_item = 0;
	_Transpondert* T;
	static vision_device remote;			// capture remote tag info in this device... for now.
	static int first = 0;

	// the tag log needs to be initialised at startup. 
	if (first == 0)
	{
		clear_transp_log();
		first++;
	}

	// starts with R, so this is a remote response. discard the R and remote UID.  
	if (message == 'R')
	{
		// TODO: use the UID in some way to indicate which remote device this is. Possibly create a new vision_device to handle the remote data...
		reader = &remote;	// point the interpreter to the remote tag struct so it doesnt save remote tag data in our reader's struct
		remote.UID = parse_message_UID(data);
		data += 5;			// grab the message in the remove message
		len -= 5;
	}

	message = data[0];


	if ((message == 'P') || (message == 'p') || (message == 'A') || (message == 1) || (message == 2)) // ---- Process TAG packets
	{
		if (len >= 8)
		{
			UID = parse_message_UID(data);

			if (UID != 0)
			{
				list_item = get_slot(UID);
				T = &transp_log[list_item];
				parse_message_into_TAG(T, data, len);

#ifdef Traffic_System
				if(len == 8)
				{
					T->SlaveID = reader->SlaveID;
				}
#endif
				reader->last_seen = time_now();
			}
		}
	}
	else if (message == 'h' && len >= 8)		//  ---- Heart-beat message ----
	{
		UID = parse_message_UID(data);
		reader->UID = UID;

		reader->last_seen = time_now();
	}
	else if (message == 'u' && len >= 6)		// ---- Device's UID message ----
	{
		reader->kind = data[1];
		UID = parse_message_UID(data);
		reader->UID = UID;

		reader->last_seen = time_now();
	}
	else if (message == 'v' && len == 3)		// ---- Firmware revision message ----
	{
		reader->fw_rev = data[1];
		reader->fw_subrev = data[2];

		reader->last_seen = time_now();

	}
	else if (message == 'k' && len == 1)		// ---- TAG count of some kind ----
	{
		reader->tag_count = data[1];

		reader->last_seen = time_now();
	}
	else if ((message ==  's' || message ==  'C') && len >= 8) // ---- Module setting response ----
	{
		uint8_t length = data[3];

		// this is a setting value we read back.....
		if (length != 0)
		{
			reader->read_setting.index = *(uint16_t*) (data + 1);
			reader->read_setting.Val   = *(uint32_t*) (data + 4);
			reader->read_setting.read = true;

			if (length <= 4)			// number type bytes returned, write it to the parameter
			{
				// check if this is an activities setting
				if(reader->read_setting.index == padr_activity)
				{
					reader->acts.word = reader->read_setting.Val;
				}
				memcpy(&reader->Settings[reader->read_setting.index], &data[4], length); //added CANTag
			}
			else // this is a string type parameter.
			{
				if (length > 20)
					length = 20;
				memcpy(&reader->Settings[reader->read_setting.index], &data[4], length);	//added CANTag
			}
		}

		reader->last_seen = time_now();
	}
	else if (message == 'c')
	{
		// this is a set setting response
		reader->last_seen = time_now();
	}
	else if (message == 'F' && len >= 2)
	{
		// these are status values. 
		// used to check LF tuned frequency
		// Status Value '9' is board ID. Can be used to detect Ranging devices. 
		if(data[1] == 9)
		{
			if(data[4] == 13)				// board ID '13' is PCB-138-03, so its a ranging module. 
			{
				//reader->kind = Ranging;
			}
		}
	}
	else if (message == 'f' && len >= 5)	// ---- Device's status word ----
	{
		reader->stat.Word = *(uint32_t*) (data + 1);
		reader->last_seen = time_now();

		reader->status_tracker = time_now();
	}
	else if (message == 'e' && len == 5)    // ---- Forwarded LF packet ----
	{   
		reader->own_LF.VehicleID = *(uint16_t*) (data + 1);
		reader->own_LF.SlaveID = data[3];
		reader->own_LF.RSSI = data[4];
		reader->own_LF.last_LF = time_now();
	}
	else if (message == 'd')          		// ---- Data message from a remote device ----
	{   
		memcpy(remote_message, &data[2], len-2);
		reader->remote_data_dest = data[1];  	// Destination address, 'U' means it matched my UID, 'V' for VID, and 'G' for global message.
		reader->remote_data_RX = remote_message;
		reader->remote_data_last = time_now();
	}
	else if (message == 'G')          // ----- GPS Data message from a remote device ----
	{
		memcpy(&reader->GPS_Data.Longitude, data + 1, 4);
		memcpy(&reader->GPS_Data.Latitude, data + 5, 4);
		memcpy(&reader->GPS_Data.HorizontalAccuracy, data + 9, 4);
		memcpy(&reader->GPS_Data.VerticalAccuracy, data + 13, 4);
		memcpy(&reader->GPS_Data.FixType, data + 17, 1);
		memcpy(&reader->GPS_Data.NumberOfSat, data + 18, 1);
		memcpy(&reader->GPS_Data.Speed, data + 19, 4);
		memcpy(&reader->GPS_Data.SpeedAccuracy, data + 23, 4);
		memcpy(&reader->GPS_Data.HeadingVehicle, data + 27, 4);
		memcpy(&reader->GPS_Data.HeadingMotion, data + 31, 4);
		memcpy(&reader->GPS_Data.HeadingAccuracy, data + 35, 4);
		memcpy(&reader->GPS_Data.SeaLevel, data + 39, 4);
		memcpy(&reader->GPS_Data.flags, data + 43, 2);
		memcpy(&reader->GPS_Data._Date, data + 45, 4);
		memcpy(&reader->GPS_Data._Time, data + 49, 4);
		reader->GPS_Data.FixAge = data[53];
	}
	else if (message == 'q')          // ---- Antenna message from a remote device ----
	{
		reader->GPS_Data.antenna_status = data[1];
	}
	else if (message == 'o')          // ---- Distance message from a remote device ----
	{
		memcpy(&reader->GPS_Data.Speed, data + 1, 4);
		memcpy(&reader->GPS_Data.SpeedAccuracy, data + 5, 4);
		memcpy(&reader->GPS_Data.Distance, data + 9, 4);
		memcpy(&reader->GPS_Data.TotalDistance, data + 13, 4);
	}
	return 0;
}

//added CANTag
vision_device* find_by_CAN_ID(uint8_t CAN_ID)
{
	int i = 0;

	for (i = 0; i < Reader_input_buffer_size; i++)
	{
		// if we've found the device, return a handle to it.
		if ((Readers[i].SlaveID == CAN_ID) && (Readers[i].UID != 0))
			return &Readers[i];
	}

	// return a null pointer if the object is not found.
	return (vision_device*) 0;
}

//added CANTag
vision_device* Add_new_Reader(vision_device V)
{
	int i = 0, oldest_tag;
	uint32_t oldest = time_now();

	// catch the event that we found a device without valid UID data
	if (V.UID == 0)
		V.UID = 0xFFFFFFFF;

	// find the first empty slot, otherwise replace the oldest entry
	for (i = 0; i < Reader_input_buffer_size; i++)
	{
		// if we've found the device, return a handle to it.
		if (Readers[i].UID == 0)
		{
			Readers[i] = V;
			return &Readers[i];
		}
		else
		{
			// keep track of the oldest entry in the list
			if (Readers[i].last_seen < oldest)
			{
				oldest = Readers[i].last_seen;
				oldest_tag = i;
			}
		}
	}

	// Assign it to the oldest slot. Override it.
	//TOdo:Neil test this function!!
	Readers[oldest_tag] = V;
	return &Readers[oldest_tag];
}

/**
 * @brief This function takes a valid Vision response or autoforwarded messagem and interprets it.
 * @param vision_device reader_from source data from reader
 * @param vision_device reader_to copy data to reader
 * @param COM_Typedef COM_port to send data to
 * @param data_type type of data to share
 * @param share_delay
 * @return
 */
uint8_t vision_share_message(vision_device* reader_from, vision_device* reader_to, uint8_t data_type)
{
	uint8_t data_length = 0;
	uint8_t data[50] = {0};

	switch (data_type) {
	case 'g':
		data_length = 43;

		memcpy(data + 1, &reader_from->GPS_Data.Longitude, 4);
		memcpy(data + 5, &reader_from->GPS_Data.Latitude, 4);
		memcpy(data + 9, &reader_from->GPS_Data.HorizontalAccuracy, 4);
		memcpy(data + 13, &reader_from->GPS_Data.VerticalAccuracy, 4);
		memcpy(data + 17, &reader_from->GPS_Data.FixType, 1);
		memcpy(data + 18, &reader_from->GPS_Data.Speed, 4);
		memcpy(data + 22, &reader_from->GPS_Data.SpeedAccuracy, 4);
		memcpy(data + 26, &reader_from->GPS_Data.HeadingVehicle, 4);
		memcpy(data + 30, &reader_from->GPS_Data.HeadingMotion, 4);
		memcpy(data + 34, &reader_from->GPS_Data.HeadingAccuracy, 4);
		memcpy(data + 38, &reader_from->GPS_Data.SeaLevel,4);
		data[42] = reader_from->GPS_Data.FixAge;
		break;

	case '@':
		data_length = 4;
		data[1] = reader_from->Reverse;
		data[2] = reader_from->stopping_dist;
		data[3] = reader_from->Speed;
		break;
	default:
		break;
	}
	data[0] = data_type;

	// ---- Send data over UART ----
	VisionSendCOM_message(reader_to->COM_Port, data, data_length);

	return true;
}

__weak int Vision_GetWRptr(vision_device* reader)
{
	return 1;
}

__weak void APP_CAN_Vision_CAN_TX(uint32_t ID, uint8_t data[], uint8_t packet_length)
{

}

__weak void APP_COM_Vision_COM_TX(uint8_t COM, uint8_t* data, uint8_t length)
{

}
