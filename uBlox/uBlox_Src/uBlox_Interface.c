/*
 * uBlox_Interface.c
 *
 *  Created on: Mar 9, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_Interface.h"
#include "uBlox_Parse_Message.h"
#include "NMEA_Parse_Message.h"
#include "master_interface.h"
#include "Vision_Parameters.h"


#define ASSERT_uBlox_RST		GPIO_ResetBits(uBlox_RST_PORT, uBlox_RST_PIN)
#define DEASSERT_uBlox_RST		GPIO_SetBits(uBlox_RST_PORT, uBlox_RST_PIN)

/**
 * @brief  Switch Reset pin of uBlox Module
 * @param  None
 * @retval None
 */
void UBX_Hard_Reset(void)
{
	ASSERT_uBlox_RST;
	Delay(10);
	DEASSERT_uBlox_RST;
}

/**
 * @brief  Transmit message on UART via HAL
 * @param  Pointer to the transmit message
 * @retval Successful transmit or failed
 */
uint8_t UBX_TX(_uBlox_Message* TX_Message)
{
	uint8_t TX_Message_Data[TX_Message->Size];
	memcpy(TX_Message_Data,&TX_Message->uBlox_Structure,TX_Message->Size);
	memcpy(TX_Message_Data + (TX_Message->Size - Checksum_Offset),&TX_Message->uBlox_Structure.Checksum, Checksum_Offset);

	if(HAL_UART_GetState(uBlox_COM) == HAL_UART_STATE_READY)
	{
		// FIXME: Use HAL_UART_Transmit_DMA
		//HAL_UART_Transmit_DMA(uBlox_COM, TX_Message_Data, TX_Message->Size);
		HAL_UART_Transmit(uBlox_COM, TX_Message_Data, TX_Message->Size,10);
		return UBX_TRUE;
	}
	return UBX_FALSE;
}

/**
 * @brief  Called from timer interrupt to signal that a uart packet has been received
 * @param  None
 * @retval None
 */
void GPSModule_Message_Handler(void)
{
	_Q_MasterIF MIF;

	// ---- Interpret message from uBlox Module ----
	MIF.Master = GPS;
	MIF.data = uBlox_data;
	MIF.len = uBlox_buf_counter;

	uint16_t message_size;

	// ---- Determine the starting position of the first message ----
	for(uint16_t position = 0 ; position < MIF.len ; position++)
	{
		if ((MIF.data[position] == UBX_1) && (MIF.data[position + 1] == UBX_2))
		{
			Vision_Status.sts.Module_UART_working = true;
			message_size = uBlox_parse_message(MIF, position);
			if (message_size)
			{
				position += (message_size - 1);
			}
		}
		else if ((MIF.data[position] == ASCII_$) && (MIF.data[position + 1] == ASCII_G))
		{
			Vision_Status.sts.Module_UART_working = true;
			message_size = NMEA_parse_message(&MIF, position);
			if (message_size)
			{
				position += message_size;
			}
		}
	}
#ifdef STM32L1
	memset(uBlox_data, 0, 1024);
#else
	memset(uBlox_data, 0, 2048);
#endif
	uBlox_buf_counter = 0;
}
