/*
 * master_interface.c
 *
 *  Created on: Aug 2014
 *      Author: J.L. Goosen
 */

#include "boot_master_interface.h"



// reference to CRC function in Framing.c
unsigned short crc16(unsigned char* data_p, unsigned short length);
  
uint8_t to_master[150];
extern uint8_t RF_Buffer[64];
Master_source Master_last = NONE;

uint32_t blank_check[] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

int Send_to_Master(_Q_MasterIF* MIF)
{
	int sent = 0;//, i;

	MIF->Master = Master_last;
	//////////////////////////////////////////////////////////////////////////////////////
	if (MIF->len)
	{
		if (MIF->Master == COM)
		{
#ifdef USE_HAL_DRIVER
			HAL_UART_Transmit(Master_COM, MIF->data, MIF->len, 100);
			sent = MIF->len;
			MIF->len = 0;
			return sent;
#else			
			USART_Sendarray(Master_COM, MIF->data, MIF->len);
			/////////////////////////////////////////////////////////////////////////////////////////
			MIF->len = 0;
			return MIF->len;		//		kills the process by returning the right value without sending anything. 
#endif
		}
#ifdef USING_CAN
		else if (MIF->Master == MCAN)
		{
			sent = CAN_out_handler(MIF->data, MIF->len);
			MIF->len -= sent;
			MIF->data += sent;
			return sent;
		}
#endif
#ifdef USE_USB
		else if (MIF->Master == MUSB)
		{
#ifdef STM32L4
			
			if (CDC_Transmit_FS(MIF->data, MIF->len) == USBD_OK)
			{
				//CDC_Transmit_FS(MIF->data, MIF->len);
				sent = MIF->len;
				MIF->len = 0;
				MIF->data += sent;
				return sent;
			}
			else 
			{
				sent++;
				return 0;
			}
#endif
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
			APP_FOPS.pIf_DataTx(MIF->data, MIF->len);
			sent = MIF->len;
			MIF->len = 0;
			MIF->data += sent;
			return sent;
#endif
		}
#endif
		else if (MIF->Master == RF)
		{
			//sent = MIN(MIF->len, 60);
			sent = MIN(MIF->len, 55);

			RF_Buffer[0] = 'b';						// indicate to RF that the message is a boot response.
			memcpy(&RF_Buffer[1], MIF->data, sent);

			if (*MIF->data == 7)
			{
				Delay(250);
			}

			txSendPacket(RF_Buffer, sent + 1);
			SetLed(&LED1, Off, 0);
			MIF->len -= sent;
			MIF->data += sent;
			return sent;
		}
		else
		{
			return 0;
		}
	}
	return 0;
}

/**
 * @brief Function to allocate a temporary block for the data being sent to prevent data corruption from any following data. 
 * @param MIF container of data to bew sent 
 * @return none
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

int Send_data_toMaster(uint8_t* mastdat, int len, Master_source M)
{
	_Q_MasterIF MIF;
	uint8_t buffer[64];

	memcpy(buffer, mastdat, len);

	if (M == CODE)
	{
		MIF.Master = CODE;
		MIF.data = buffer;
		MIF.len = len;
		push_to_master(MIF);
		return 0;
	}
	return -1;
}

int parse_message(_Q_MasterIF MIF)
{
	int send_mess = 0;
	int i;
	uint8_t b;

	Master_last = MIF.Master;

	SetLed(&LED1, Off, 0);
	
#ifdef USE_HAL_DRIVER
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
#else
	IWDG_ReloadCounter();
#endif
	if ((MIF.len >= (uint16_t) 100) && (MIF.len <= BUFFER_SIZE) && (UPDATE_Flags.F_UPDATE == 1) && (UPDATE_Flags.F_UPDATE_END == 0))
	{
		uint16_t crc = 0, crc_in = 0;
		uint32_t p = 0, address;

		if (MIF.len != 1030)				// use a fixed size block for stability for now
		{
			SetLed(&LED1, Blue, 0);
			to_master[0] = 0x09;			// indicate that the message was not flashed correctly
			send_mess = 1;
		}
		else
		{
			int block_len = 1024;			// 1024 data bytes to flash
			
			//  check CRC
			crc = crc16(MIF.data, block_len);
			crc_in = *(uint16_t*) (&MIF.data[1028]);
			address = *(uint32_t*) (&MIF.data[block_len]);
			
			if (crc == crc_in)
			{
				// compare flash memory with data received, and only flash if different
				if (memcmp(MIF.data, (uint8_t*) (APPLICATION_ADDRESS + address), block_len))
				{
#ifdef USE_HAL_DRIVER
#ifdef STM32L4
					bool erased = false;
					FLASH_EraseInitTypeDef eraseinit;
					uint32_t copy_both = 0;
					uint8_t temp[1024];
					uint32_t err;

					eraseinit.Banks = ((APPLICATION_ADDRESS + address) < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_1;
					eraseinit.NbPages = 1;
					eraseinit.Page = 0xFF & (((APPLICATION_ADDRESS + address)) / FLASH_PAGE_SIZE);
					eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;

					SetLed(&LED1, Yellow, 0);

					///	the L4 devices have 2K flash blocks (erase) and data is sent in 1K chunks.				///
					///	With the boot procedure of "check block -> flash if different", this is					///
					///	a problem, but only in one specific case: when one chunk is fine, and so not erased,	///
					///	but the following chunk, which is in the same flash page is not right. 					///
					///	In this case we need to store the previous good chunk, erase the page, and flash both. 	///

					// check if the flash is blank, if so don't bother erasing. 
					if (memcmp(blank_check, (uint8_t*) (APPLICATION_ADDRESS + address), sizeof(blank_check)))
					{
						if ((address) & (FLASH_PAGE_SIZE - 1))
						{
							/// Copy the good data from the previous block
							memcpy(temp, (uint8_t*) (APPLICATION_ADDRESS + address - block_len), block_len);
							copy_both = address - block_len;
						}
						if (HAL_FLASHEx_Erase(&eraseinit, &err) == HAL_OK)
							erased = true;
					}
					else
						erased = true;
					i += FLASH_PAGE_SIZE;

					if (erased == true)
					{
						SetLed(&LED1, Red, 0);
						if (copy_both)
							Flash_Packet_at_adr(temp, block_len, copy_both);
						p = Flash_Packet_at_adr(MIF.data, block_len, address);
					}
					else
						SetLed(&LED1, Red, 0);

#else
					uint32_t err, Pages = block_len / FLASH_PAGE_SIZE;
					FLASH_EraseInitTypeDef eraseinit =
					{	0, APPLICATION_ADDRESS + address, Pages};

					SetLed(&LED1, Yellow, 0);

					if (HAL_FLASHEx_Erase(&eraseinit, &err) == HAL_OK)
					{
						SetLed(&LED1, Red, 0);
						__disable_irq();
						p = Flash_Packet_at_adr(MIF.data, block_len, address);
						__enable_irq();
					}
#endif
#else
					FLASH_Status S;
					uint32_t copy_both = 0;
					uint8_t temp[1024];

					SetLed(&LED1, Yellow, 0);

					///	the F1 devices have 2K flash blocks (erase) and data is sent in 1K chunks.				///
					///	With the boot procedure of "check block -> flash if different", this is					///
					///	a problem, but only in one specific case: when one chunk is fine, and so not erased,	///
					///	but the following chunk, which is in the same flash page is not right. 					///
					///	In this case we need to store the previous good chunk, erase the page, and flash both. 	///

					// check if the flash is blank, if so don't bother erasing. 
					if(memcmp(blank_check, (uint8_t*) (APPLICATION_ADDRESS + address), sizeof(blank_check)))
					{
						if((address) & (FLASH_PAGE_SIZE-1))
						{
							/// Copy the good data from the previous block
							memcpy(temp, (uint8_t*)(APPLICATION_ADDRESS + address - block_len), block_len);
							copy_both = address - block_len;
						}
						S = FLASH_ErasePage(APPLICATION_ADDRESS + address);
					}
					else
					S = FLASH_COMPLETE;
					i += FLASH_PAGE_SIZE;

					if(S == FLASH_COMPLETE)
					{
						SetLed(&LED1, Red, 0);
						__disable_irq();
						if(copy_both) Flash_Packet_at_adr(temp, block_len, copy_both);
						p = Flash_Packet_at_adr(MIF.data, block_len, address);
						__enable_irq();
					}
#endif
				}
				else
				{
					SetLed(&LED1, Violet, 0);
					p = block_len;
					UPDATE_Flags.Update_Pointer = address + block_len;
					//trigger update end if the total counter reaches file size
					if (UPDATE_Flags.Update_Pointer >= UPDATE_Flags.Update_FileSize)
						UPDATE_Flags.F_UPDATE_END = time_now() + 100;
				}
			}
			
			if (p == block_len)
			{
				SetLed(&LED1, Green, 0);
				to_master[0] = 0x07;
				send_mess = 1;
#ifdef USE_HAL_DRIVER
				__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
#else
				IWDG_ReloadCounter();
#endif
			}
			else
			{
				SetLed(&LED1, Blue, 0);
				to_master[0] = 0x09;			/// indicate that the message was not flashed correctly 
				send_mess = 1;
			}
		}
	}
	else
	{
		switch (MIF.data[0])
		{

		//	Run application
		case 'A':
			if (MIF.len == 1)
				Run_Application = 1;
			break;
		case 'B':
			if (MIF.len == 1)
			{
				Run_Bootloader = 1;
				F_FlashErase = 0;
			}
			break;
		case 'Q':
			if (MIF.len == 1)
			{
				send_mess = 2;
				memcpy(to_master, &boot, sizeof(boot));
			}
			break;
		case 'C':
			if (MIF.len == 1)
			{
				b = Firmware_rev;
				send_mess = 1;
				to_master[0] = b;
			}
			break;
		case 'K':
			if (MIF.len == 1)
			{
#ifdef 	GPS_MODULE
				b = GPS;
#elif	RANGING_MODULE
				b = Ranging;
#else
				b = Pulse;
#endif

				send_mess = 1;
				to_master[0] = b;
			}
			break;
		case 1:
			if ((MIF.len == 6) && (MIF.data[1] == 0x0A) && (MIF.data[5] == 1))
			{
				UPDATE_Flags.F_UPDATE = 1;
				UPDATE_Flags.F_UPDATE_END = 0;
				UPDATE_Flags.Update_Pointer = 0;
				UPDATE_Flags.Update_FileSize = (MIF.data[2] << 16);
				UPDATE_Flags.Update_FileSize += (MIF.data[3] << 8);
				UPDATE_Flags.Update_FileSize += MIF.data[4];

				F_FlashErase = 1;
				Run_Bootloader = 0;
			}
			break;
		default:
			//master packet not understood
			if (MIF.len != 0)
			{
				to_master[0] = 0x16;
				// Debug 10/10/2017
				// send_mess = 1;
			}
			break;
		}
	}

	if (send_mess)
	{
		MIF.data = to_master;
		MIF.len = send_mess;
		push_to_master(MIF);
	}
	return 0;
}

// Called from timeout keeper to signal that data is ready.
void Boot_packet_handler(int M)
{
	_Q_MasterIF MIF;

	if (boot_buf_counter == 0)
		return;

	// ---- Interpret message from master ----
	MIF.Master = M;
	MIF.data = boot_data;
	MIF.len = boot_buf_counter;

	parse_message(MIF);

	boot_buf_counter = 0;
}

#ifdef USING_CAN
// Called from CAN interrupt to signal that a CAN packet has been received.
void CAN_packet_handler(uint8_t* data, uint8_t len, bool Last)
{
	memcpy(&boot_data[boot_buf_counter], data, len);
	boot_buf_counter += len;
	UPDATE_Flags.master = MCAN;
//	UPDATE_Flags.Master_last_seen = time_now();
	UPDATE_Flags.Master_last_seen = 0;
	if (Last)		// packet was signalled as last/only.
	{
		// interpret message from master
		UPDATE_Flags.Master_last_seen = time_now() - 5;
	}
}

// Called from code to send data to the can bus.
#ifdef USE_HAL_DRIVER
uint8_t CAN_out_handler(uint8_t* data, uint8_t len)
{
	uint8_t sendsize, sent = 0;//, res
	static int error_counter = 0;
	CanTxMsgTypeDef TxMessage = {.ExtId = CAN_VisBootResp_ID, .RTR = CAN_RTR_DATA, .IDE = CAN_ID_EXT};

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
		error_counter = 0;

		data += sendsize;
		sent += sendsize;
	}
	return sent;
}
#else
uint8_t CAN_out_handler(uint8_t* data, uint8_t len)
{
	uint8_t sendsize, res, sent = 0;
	static int error_counter = 0;
	CanTxMsg TxMessage = {.ExtId = CAN_VisBootResp_ID, .RTR = CAN_RTR_DATA, .IDE = CAN_ID_EXT};

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
			// after a certain number fo failed attepts, just continue. tell the upper layer that we sent it...

			data += sendsize;
			sent += sendsize;
		}
	}
	else
	{
		/// update the system health tracker to indicater that CAN is functional. 
		error_counter = 0;

		data += sendsize;
		sent += sendsize;
	}
	return sent;
}
#endif

#endif

/**
 * @brief: 	this function is used to keep track of a very large circular buffer. 
 * 			the buffer stores messages sequencially. the buffer does not track when memory is disposed of. 
 * @param data				// pointer to what you want to store
 * @param len				// length you want to store it for
 * @param store				// do want me to store it?
 * @return
 */
#define bigbuflen	4096
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

