/*
 * master_interface.c
 *
 *  Created on: Aug 2014
 *      Author: J.L. Goosen
 */

#include "master_interface.h"

Master_source Master_last = NONE;


/**
 * CAN processing commands
 */

#ifdef USING_CAN
uint8_t CAN_in_buf[2048];
// Called from CAN interrupt to signal that a CAN packet has been received.
void CAN_packet_handler(uint8_t* data, uint8_t len, bool Last, uint8_t slave_id)
{
//	_Q_MasterIF MIF;
	static uint8_t message_length = 0;

	Vision_Status.sts.CAN_Working = true;
	if (((slave_id != (uint8_t) vision_settings.slave_id)&&(slave_id != (uint8_t)0xFF)) && (slave_id != 0))
		return;

	memcpy(&CAN_in_buf[message_length], data, len);
	message_length += len;

	if (Last)		// packet was signalled as last/only.
	{
		// interpret message from master
//		MIF.Master = MCAN;
//		MIF.data = CAN_in_buf;
//		MIF.len = message_length;
		//xQueueSendFromISR(Q_Master, &MIF, NULL);
		// parse_message_old(MIF);
		parse_message(CAN_in_buf, message_length, MCAN);
		/// Todo: check for buffer overflow.
		message_length = 0;
	}
	else
	{
		len++;			/// just catch the debugger
	}
}

#ifdef STM32L4
// Called from code to send data to the can bus.
uint8_t CAN_out_handler(uint8_t* data, uint8_t len)
{

	CanTxMsgTypeDef TxMessage;
	uint8_t sendsize,  sent = 0;//res,
	static int error_counter = 0;

	TxMessage.ExtId = vision_settings.slave_id;
	TxMessage.ExtId &= 0xff;
	TxMessage.ExtId |= CAN_Vision_Resp_ID;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_EXT;

	//////////////////////////////////////////////////////////////////////////////
	// change the ID for inter pod comms.
	if ((len == 2) && (data[0] == 'L'))
		TxMessage.ExtId = CAN_Vision_Sync_ID;	// | (uint32_t) Vision_Status.Slave_ID + 1;
	//////////////////////////////////////////////////////////////////////////////

	if (len > 8)
		sendsize = 8;
	else
		sendsize = len;

	len -= sendsize;

	TxMessage.DLC = sendsize;
	memcpy(TxMessage.Data, data, sendsize);

	if (len != 0)
		TxMessage.ExtId |= CAN_not_Last;
	else
		TxMessage.ExtId &= (~CAN_not_Last);

	hcan1.pTxMsg = &TxMessage;
	if (HAL_CAN_Transmit(&hcan1, 0) == HAL_ERROR)
	{
		if (error_counter++ > 100)
		{
			// after a certain number of failed attempts, just continue. tell the upper layer that we sent it...
			data += sendsize;
			sent += sendsize;
			__HAL_CAN_CANCEL_TRANSMIT(&hcan1, CAN_TXMAILBOX_0);
			__HAL_CAN_CANCEL_TRANSMIT(&hcan1, CAN_TXMAILBOX_1);
			__HAL_CAN_CANCEL_TRANSMIT(&hcan1, CAN_TXMAILBOX_2);
		}
	}
	else
	{
		/// update the system health tracker to indicator that CAN is functional.
//		Vision_Status.CAN_Working = true;

		error_counter = 0;

		data += sendsize;
		sent += sendsize;
	}
	return sent;


//	CanTxMsgTypeDef TxMessage;
//	uint8_t sendsize, sent = 0; //, res
//	static int error_counter = 0;
//	uint8_t CAN_Packet_count = 0;
//
//	TxMessage.ExtId = vision_settings.slave_id;
//	TxMessage.ExtId &= 0xff;
//	TxMessage.ExtId |= CAN_Vision_Resp_ID;
//	TxMessage.RTR = CAN_RTR_DATA;
//	TxMessage.IDE = CAN_ID_EXT;
//
//	//////////////////////////////////////////////////////////////////////////////
//	// change the ID for inter pod comms.
//	if ((len == 2) && (data[0] == 'L'))
//	{
//		TxMessage.ExtId = CAN_Vision_Sync_ID;	// | (uint32_t) Vision_Status.Slave_ID + 1;
//		TxMessage.DLC = 2;
//		memcpy(TxMessage.Data, data, 2);
//		HAL_CAN_Transmit(&hcan1, 10);
//
//		return 2;
//	}
//
//	//////////////////////////////////////////////////////////////////////////////
//
//	sendsize = len;
//
//	if(len <= 8)
//	{
//		TxMessage.DLC = len;
//		memcpy(TxMessage.Data, data, len);
//		TxMessage.ExtId &= (~CAN_not_Last);
//		HAL_CAN_Transmit(&hcan1, 0);
//
//		return len;
//	}
//
//	if(len>8)
//	{
//		CAN_Packet_count = len/8;
//		if(CAN_Packet_count != (float)len/8)
//			CAN_Packet_count++;
//
//		for(int i = 0; i < CAN_Packet_count; i++)
//		{
//			if (i != (CAN_Packet_count-1))
//			{
//				TxMessage.ExtId |= CAN_not_Last;
//				TxMessage.DLC = 8;
//				memcpy(TxMessage.Data, data, 8);
//				hcan1.pTxMsg = &TxMessage;
//				if(HAL_CAN_Transmit(&hcan1, 0) != HAL_ERROR)
//				{
//					if(sendsize>8)
//					{
//						sendsize -= 8;
//						data += 8;
//						sent += 8;
//					}
//					else
//					{
//						data += sendsize;
//					}
//					error_counter = 0;
//				}
//				else if(error_counter < 10)
//				{
//					error_counter++;
//					i--;
//				}
//				else
//					return len;
//
//
//			}
//			else
//			{
//				TxMessage.ExtId &= (~CAN_not_Last);
//				TxMessage.DLC = sendsize;
//				memcpy(TxMessage.Data, data, sendsize);
//				hcan1.pTxMsg = &TxMessage;
//				if(HAL_CAN_Transmit(&hcan1, 0) != HAL_ERROR)
//				{
//					sent += sendsize;
//					error_counter = 0;
//				}
//				else if(error_counter < 10)
//				{
//					error_counter++;
//					i--;
//				}
//			}
//
//		}
//
//		return sent;
//
//	}
//
//	return sent;
}

//uint8_t CAN_out_handler_multiple(uint8_t* data, uint8_t len)
//{
//	CanTxMsgTypeDef TxMessage;
//	uint8_t sendsize, res, sent = 0;
//	static int error_counter = 0;
//	static uint8_t message_count = 1;
//	static uint8_t message_total = 0;
//
//	if (message_total == 0)
//	{
//		message_count = 1;
//		message_total = len / 8 ;
//		if ((len % 8) != 0)
//		{
//			message_total++;
//		}
//	}
//
//	TxMessage.ExtId = CAN_VISION_Resp_ID;
//	TxMessage.ExtId  |= (message_count << 8);
//	TxMessage.ExtId  |= (uint32_t) vision_settings.slave_id;
//	TxMessage.RTR = CAN_RTR_DATA;
//	TxMessage.IDE = CAN_ID_EXT;
//
//	if (len > 8)
//	{
//		sendsize = 8;
//	}
//	else
//	{
//		sendsize = len;
//	}
//
//	len -= sendsize;
//
//	TxMessage.DLC = sendsize;
//	memcpy(TxMessage.Data, data, sendsize);
//
//
//
////	//////////////////////////////////////////////////////////////////////////////
////	// change the ID for inter pod comms.
////	if ((len == 2) && (data[0] == 'L'))
////		TxMessage.ExtId = CAN_Vision_Sync_ID;	// | (uint32_t) Vision_Status.Slave_ID + 1;
////	//////////////////////////////////////////////////////////////////////////////
//
//
//	hcan1.pTxMsg = &TxMessage;
//	if (HAL_CAN_Transmit(&hcan1, 0) == HAL_ERROR)
//	{
//		if (error_counter++ > 100)
//		{
//			// after a certain number of failed attempts, just continue. tell the upper layer that we sent it...
//			data += sendsize;
//			sent += sendsize;
//			__HAL_CAN_CANCEL_TRANSMIT(&hcan1, CAN_TXMAILBOX_0);
//			__HAL_CAN_CANCEL_TRANSMIT(&hcan1, CAN_TXMAILBOX_1);
//			__HAL_CAN_CANCEL_TRANSMIT(&hcan1, CAN_TXMAILBOX_2);
//		}
//	}
//	else
//	{
//		/// update the system health tracker to indicator that CAN is functional.
////		Vision_Status.CAN_Working = true;
//
//		error_counter = 0;
//
//		data += sendsize;
//		sent += sendsize;
//
//		if (message_count == message_total)
//		{
//			message_total = 0;
//		}
//		else
//		{
//			message_count++;
//		}
//	}
//	return sent;
//}
#else
// Called from code to send data to the can bus.
uint8_t CAN_out_handler(uint8_t* data, uint8_t len)
{
	CanTxMsg TxMessage;
	uint8_t sendsize, res, sent = 0;
	static int error_counter = 0;

	TxMessage.ExtId = vision_settings.slave_id;
	TxMessage.ExtId &= 0xff;
	TxMessage.ExtId |= CAN_Vision_Resp_ID;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_EXT;

	//////////////////////////////////////////////////////////////////////////////
	// change the ID for inter pod comms.
	if ((len == 2) && (data[0] == 'L'))
		TxMessage.ExtId = CAN_Vision_Sync_ID;	// | (uint32_t) Vision_Status.Slave_ID + 1;
	//////////////////////////////////////////////////////////////////////////////

	//	while (len > 0)
	//	{
	if (len > 8)
		sendsize = 8;
	else
		sendsize = len;

	len -= sendsize;

	TxMessage.DLC = sendsize;
	memcpy(TxMessage.Data, data, sendsize);

	if (len != 0)
		TxMessage.ExtId |= CAN_not_Last;
	else
		TxMessage.ExtId &= (~CAN_not_Last);

	res = CAN_Transmit(CAN1, &TxMessage);    // keep trying until a mailbox accepts the packet.
	if (res == CAN_TxStatus_NoMailBox)
	{
		if (error_counter++ > 100)
		{
			// after a certain number of failed attempts, just continue. tell the upper layer that we sent it...
			data += sendsize;
			sent += sendsize;
		}
	}
	else
	{
		error_counter = 0;

		data += sendsize;
		sent += sendsize;
	}
	return sent;
}
#endif
#endif

/**
 * @brief	this code is called by the master packet handler to actually forward messges to the master if necesary. 
 * The idea is that this code gets called as often as possible, as long as there are messages awaiting to go to the master.
 * the MIF data structure contains pointers to the data to go out, as well as how many bytes are left. 
 * If this function succeeds in sending them, they will return empty. otherwise the master handler must continue calling them
 * with the same data until they fail or pass. this allows thread safe waiting for long packets or troublesome communication.  
 * @param MIF
 * @return
 */
int Send_to_Master(_Q_MasterIF* MIF)
{
	int sent = 0;

	/// Make sure we don't forward comms to RF forever. stop after a few minutes.
	/// TODO: consider making this true for all cases where master comms is lost after a while.....
	if(time_since(Vision_Status.last_master_coms) > 200000 && Master_last == RF)
		Master_last = NONE;

	//////////////////////////////////////////////////////////////////////////////////////
	/// Determine where to send messages that originated from code. usually the last master we saw, 
	/// otherwise to USB/uart depending on the status of each.
	if (MIF->Master == CODE)
	{
		if(Master_last == MUSB && Vision_Status.sts.USB_Active == false)
			Master_last = NONE;					// make sure we're not sending to USB while its inactive. 
		else
			if (Master_last == NONE)
				MIF->Master = MCAN;
			else
				MIF->Master = Master_last;
	}
	//////////////////////////////////////////////////////////////////////////////////////

	//	return MIF->len;		//		kills the process by returning the right value without sending anything.

	if (MIF->len)
	{
		if (MIF->Master == COM)
		{
			/////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_HAL_DRIVER
#ifdef STM32L4
			if(HAL_UART_GetState(Master_COM) == HAL_UART_STATE_READY)
			{

				uint8_t newlen = 0;
				uint8_t* buf = get_frame(MIF->data, MIF->len, &newlen);
				HAL_UART_Transmit_DMA(Master_COM, buf, newlen);
				MIF->len = 0;
				return newlen;
			}
#else
			uint8_t newlen = 0;
			uint8_t* buf = get_frame(MIF->data, MIF->len, &newlen);

			HAL_UART_Transmit(Master_COM, buf, newlen, 100);
			sent = MIF->len;
			MIF->len = 0;
			return sent;
#endif
#else			
#ifdef VISION_TEST_MASTER
			MasterTesterPutMessage(buf, newlen);
#endif
			uint8_t newlen = 0;
			uint8_t* buf = get_frame(MIF->data, MIF->len, &newlen);
			sent = USART_SendDMA(Master_COM, buf, newlen);
			if(sent == newlen)
			{	
				MIF->len = 0;
			}
			return sent;
#endif
		}
#ifdef USING_CAN
		else if (MIF->Master == MCAN)
		{
			sent = CAN_out_handler(MIF->data, MIF->len);
			//sent = CAN_out_handler_multiple(MIF->data, MIF->len);
			MIF->len -= sent;
			MIF->data += sent;
			return sent;
		}
#endif
#ifdef  USE_USB 
		else if (MIF->Master == MUSB)
		{
#ifdef STM32F10X_HD
			if (Vision_Status.USB_Active && bDeviceState == CONFIGURED)
			{
				sent = CDC_Send_DATA(MIF->data, MIF->len);
				MIF->len -= sent;
				MIF->data += sent;
				return sent;
			}
#endif
#ifdef STM32F10X_CL
			if (Vision_Status.sts.USB_Active)
			{
				//				APP_FOPS.pIf_DataTx(MIF->data, MIF->len);
				//				sent = MIF->len;
				//				MIF->len = 0;
				//				MIF->data += sent;
				//				return sent;

				uint8_t newlen = 0;
				uint8_t* buf = get_frame(MIF->data, MIF->len, &newlen);

#ifdef VISION_TEST_MASTER
				MasterTesterPutMessage(buf, newlen);
#endif
				APP_FOPS.pIf_DataTx(buf, newlen);
				sent = MIF->len;
				MIF->len = 0;
				MIF->data += sent;
				return sent;				
			}
#endif
#ifdef STM32L4
			if (Vision_Status.sts.USB_Active)
			{
				uint8_t newlen = 0;// , res
				uint8_t* buf = get_frame(MIF->data, MIF->len, &newlen);

#ifdef VISION_TEST_MASTER
				MasterTesterPutMessage(buf, newlen);
#endif

				//res =
						CDC_Transmit_FS(buf, newlen);
				// best to assume USB has sent. otherwise it hangs upon port re-opening. 
				sent = MIF->len;
				MIF->len = 0;
				MIF->data += sent;
				return sent;
			}
#endif
		}
#endif
		else if (MIF->Master == RF)
		{
			RF_message RF;
			RF.buff = MIF->data;
			RF.len = MIF->len;
			RF.time_to_respond = time_now()-1;
			RF.type = rf_Respons;
			// send a message to the RF process to send my Master response.
			//			if ((vision_settings.getActivities().LF_response))
			pipe_put(&cc1101.p, &RF);
			sent = MIF->len;
			MIF->len = 0;
			MIF->data += sent;
			return sent;
		}
		else
		{
			// if we dont know where to send the data, just discard it... 
			sent = MIF->len;
			MIF->len = 0;
			return sent;
			//			return 0;
		}
	}
	return 0;
}

/**
 * @brief Function to allocate a temporary block for the data being sent to prevent data corruption from any following data. 
 * @param MIF container of data to bew sent
 * @param front if true the data will jump the queue and be sent first. 
 * @return
 */
int push_to_master(_Q_MasterIF MIF)
{
	if (messages_to_master.p.buf != NULL)
	{
		MIF.data = buff_alloc(MIF.data, MIF.len, true);		// get new memory block big enough to hold our data, then move it in there 
		pipe_put(&messages_to_master.p, &MIF);
	}
	return 0;
}

/**
 * 
 * @param T
 * @param M
 * @param command this can be either 'P', 'p' (long and short poll messages), 1 or 2 (autoforward).
 * @return
 */
int Send_POD_toMaster(void* T, Master_source M, char command)
{
	_Q_MasterIF MIF;
	int age;
	uint16_t dist_lf;							// compound data for LF RSSI, RSSI and ranging distance
	uint8_t buffer[80] = {0};
	_Transpondert* t = (_Transpondert*) T;

	bool shortened = false;
	bool autoforward = false;

	if (command <= 3)			// auto-forwarded message
	{
		autoforward = true;
		if (vision_settings.getActivities().use_shortened_fw)
			shortened = true;
	}
	if (command == 'p')
		shortened = true;

	// calculate the composite tag distance.   
	if (t->kind == Ranging)
	{
		dist_lf = t->Dist;
		dist_lf = MIN(dist_lf, 32766);	// limit to positive integer values, so we can use MSB to detect vision/PDS
	}
	else
	{
		dist_lf = ((uint16_t) t->LF.RSSI) << 8;	// LF strength. for PDS this will be -1 
		dist_lf |= t->rssi;				// RF signal strength  			
	}

	/// we use the shorter message type.
	if (shortened)
	{
		if (autoforward == false)
			buffer[0] = 'p';
		else
			buffer[0] = command;

		memcpy(&buffer[1], &t->UID, 4);
		buffer[5] = t->type;
		if (command == 'p')
		{
			age = time_since(t->last_seen) / 1000;
			age = MIN(age, 255);
			buffer[6] = age;
		}
		else
		{
			buffer[6] = (command == 2) ? t->LF.RSSI : t->rssi;		// is the LF message? then use LF RSSI in this byte
			if(t->kind == Ranging)
				buffer[6] = 0;
		}

		// range forward command. 
		if(command == 3)
			*(uint16_t*)&buffer[6] = t->Dist;
		// LF forward command
		else if (command != 2)
			buffer[7] = t->kind;
		else
		{
			buffer[7] = t->LF.SlaveID;
			if (t->LF.VehicleID == (uint16_t) vision_settings.vehic_id)
				buffer[7] |= 0x80;
		}

		MIF.Master = M;
		MIF.data = buffer;
		MIF.len = 8;
	}
	else	/// this is the longer message type. 
	{
		buffer[0] = (autoforward) ? 'A' : 'P';
		memcpy(&buffer[1], &t->UID, 4);
		buffer[5] = t->type;
		buffer[6] = t->volts;
		age = (time_since(t->last_seen)) / 1000;
		if (age > 255)
			age = 255;
		buffer[7] = age;

		buffer[8] = t->kind;

		buffer[9] = t->status;

		memcpy(&buffer[10], &t->VehicleID, 4);
		buffer[14] = t->SlaveID;

		memcpy(&buffer[15], &t->LF.VehicleID, 2);
		buffer[17] = t->LF.SlaveID;

		memcpy(&buffer[18], &dist_lf, 2);

		buffer[20] = t->group;
		//		buffer[18] = 10;

		////	LF age/distance age ////////////////
		if (t->kind == Ranging)
		{
			if (t->Dist == 0)
				age = 255;
			else
				age = (time_since(t->last_ranged)) / 1000;
		}
		else
		{
			if (t->LF.RSSI == -1)
				age = 255;
			else
				age = (time_since(t->LF.last_LF)) / 1000;
		}
		if (age > 255) 
			age = 255;
		buffer[21] = age;
		buffer[22] = t->FirmwareRev;

		buffer[23] = t->ManTagAck;
		buffer[24] = t->Reverse;
		buffer[25] = t->V_lenght;
		buffer[26] = t->V_Width;
		buffer[27] = t->Stopping_dist;
		memcpy(&buffer[28], &t->Speed, 4);

		////////////////////////////////////////////

		// ---- Time Info -------
		buffer[32] = t->Seconds ;
		buffer[33] = t->Minutes;
		buffer[34] = t->Hours ;
		buffer[35] = t->Day ;
		buffer[36] = t->Month ;
		buffer[37] = t->Year  ;

		////////////////////////////////////////////

		// ---- GPS functionality ----
		memcpy(buffer + 38, &t->GPS_Data.Longitude, 4);
		memcpy(buffer + 42, &t->GPS_Data.Latitude, 4);
		memcpy(buffer + 46, &t->GPS_Data.VerticalAccuracy, 4);
		memcpy(buffer + 50, &t->GPS_Data.HorizontalAccuracy, 4);
//		memcpy(buffer + 44, &t->GPS_Data.Speed, 4);
		memcpy(buffer + 54, &t->GPS_Data.HeadingVehicle, 4);
		memcpy(buffer + 58, &t->GPS_Data.FixType, 1);
		memcpy(buffer + 59, &t->GPS_Data.FixAge, 1);
		memcpy(buffer + 60, &t->GPS_Data.SeaLevel,4);

		if (t->kind == Pulse_GPS)
		{

			MIF.len = 64;
		}
		else
		{
			MIF.len = 38;
		}

		MIF.Master = M;
		MIF.data = buffer;

#ifdef USE_TAG_NAME
		strncpy((char*)&buffer[MIF.len], t->name, STR_MAX);
		MIF.len += strnlen(t->name, STR_MAX);
#endif
	}

	if((M == CODE) && (Master_last == NONE))
		MIF.Master = MCAN;
	push_to_master(MIF);

	return 0;
}

uint8_t master_rf_buf[64];

int send_heartbeat(uint32_t last_RF)
{
	_Q_MasterIF MIF;
	int age;
	uint8_t buffer[12];
	buffer[0] = 'h';
	memcpy(&buffer[1], &Vision_Status.UID, 4);
	buffer[5] = vision_settings.slave_id;
	buffer[6] = transp_count;

	age = time_since(last_RF) / 1000;
	if (age > 255)
		age = 255;
	buffer[7] = age;

	MIF.Master = CODE;
	MIF.data = buffer;
	MIF.len = 8;
	push_to_master(MIF);

	//	if (Master_last == NONE)
	//	{
	//		MIF.Master = CAN;
	//		push_to_master(MIF);
	//	}

	return 0;
}

int send_LF_sync(void)
{
	_Q_MasterIF MIF;
	uint8_t buffer[2];
	// this will be delt with in the CAN out handler to change it to the appropriate CAN ID for Poll. 
	buffer[0] = 'L';
	buffer[1] = vision_settings.slave_id;

	MIF.data = buffer;
	MIF.len = 2;
	MIF.Master = MCAN;
	push_to_master(MIF);

	// send a RF based LF sync also. 
	RF_message RF;
	RF.time_to_respond = time_now()-1;
	RF.type = rf_LF_send;
	// send a message to the RF process to send my LF sync message
	pipe_put(&cc1101.p, &RF);

	return 0;
}

int send_LF_sync_F(uint8_t SlaveID)
{
	_Q_MasterIF MIF;
	uint8_t buffer[2];
	// this will be delt with in the CAN out handler to change it to the appropriate CAN ID for Poll.
	buffer[0] = 'L';
	buffer[1] = SlaveID;

	MIF.data = buffer;
	MIF.len = 2;
	MIF.Master = MCAN;
	push_to_master(MIF);

	return 0;
}

int send_LF_alert(int ms_delay)
{
	// send a RF based LF sync message. 
	RF_message RF;
	RF.time_to_respond = time_now() + ms_delay;
	RF.type = rf_LF_send;
	// send a message to the RF process to send my LF sync message
	pipe_put(&cc1101.p, &RF);

	return 0;
}

/**
 * Brief Sends info about the currently detected LF to the master device
 * @return
 */
void forward_LF_report(LF_message_type LF)
{
	_Q_MasterIF MIF;
	uint8_t buffer[5];

	buffer[0] = 'e';
	memcpy(&buffer[1], &LF.VehicleID, 2);
	buffer[3] = LF.SlaveID;
	buffer[4] = LF.RSSI;

	MIF.data = buffer;
	MIF.len = 5;
	MIF.Master = CODE;
	push_to_master(MIF);
}

// Called from timer interrupt to signal that a uart packet has been received.
void UART_packet_handler(void)
{
	_Q_MasterIF MIF;

	// interpret message from master
	MIF.Master = COM;
	MIF.data = uart_data;
	MIF.len = uart_buf_counter;

	//	for (uint16_t uart_index = 0; uart_index < uart_buf_counter; uart_index++)
	//	{
	//		if (parse_message(MIF))
	//		{
	//			Vision_Status.sts.Uart_Working = true;
	//	#ifdef USE_USB
	//	#ifdef STM32F10X_CL
	//			if( Master_COM == COM_1)
	//			{
	//				// if UART is detected as master, disconnect the USB port. This is to prevent USB from constantly reconnecting.
	//				USB_disconnect();
	//			}
	//	#endif
	//	#endif
	//		}
	//	}
	if (MIF_Parse(MIF))
	{
		uart_buf_counter = 0;
		memset(uart_data,0,2048);

#ifdef USE_USB
#ifdef STM32F10X_CL
		if( Master_COM == COM_1)
		{
			// if UART is detected as master, disconnect the USB port. This is to prevent USB from constantly reconnecting.
			USB_disconnect();
		}
#endif
#endif
	}


}

/**
 * @brief: 	this function is used to keep track of a very large circular buffer. 
 * 			the buffer stores messages sequentially. the buffer does not track when memory is disposed of. 
 * @param data				// pointer to what you want to store
 * @param len				// length you want to store it for
 * @param store				// do want me to store it?
 * @return
 */
#define bigbuflen	5120
uint8_t big_buf[bigbuflen];

uint8_t* buff_alloc(uint8_t* data, int len, bool store)
{
	static int p = 0;

	uint8_t* retval;

	if ((p + len) >= bigbuflen)			// data is too big to fit, so wrap around
	{
		retval = big_buf;
		p = len;
	}
	else								// data will fit. return current pos and move pointer.
	{
		retval = &big_buf[p];
		p += len;
	}

	if (store)							// we need to move the data across
	{
		memcpy(retval, data, len);
	}

	return retval;						// this is where your data is or will be	
}

