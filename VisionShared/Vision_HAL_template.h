/*
 * Vision_HAL.h
 *
 *  Created on: Jul 27, 2015
 *      Author: Kobus
 */

#ifndef VISION_HAL_H_
#define VISION_HAL_H_

#ifdef STM32F10X_CL

#endif
#ifdef STM32F4XX

#endif

#ifndef MAX
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

#define LF_TX_Capable

//#define FILTER_LF				// experimental
#define USE_TAG_NAME			// consumes 20 extra bytes per tag. 

//#ifdef Traffic_System

#define num_transp_total 256
#define Reader_input_buffer_size 32

// define how long a microsecond is
#define UsBase	72

// CAN port interface
// This function gets called by VisionShared to send a CAN message
void APP_CAN_Vision_CAN_TX(uint32_t ID, uint8_t data[], uint8_t packet_length);
#define VisionSendCAN_message(ID, data, len) APP_CAN_Vision_CAN_TX(ID, data, len)

/* ---- Add application level function (host) ----
void APP_CAN_Vision_CAN_TX(uint32_t ID, uint8_t data[], uint8_t packet_length)
{
		uint8_t Mailbox = 0;

		CanTxMsgTypeDef TxMessage;

		// ---- Load data onto CAN buffer ----
		for (uint8_t i = 0; i < packet_length; i++)
		{
			TxMessage.Data[i] = *data++;
		}

		// ---- Prepare massage for transmit ----
		TxMessage.ExtId = ID;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.DLC = packet_length;

		// ---- Transmit message ----
		Mailbox = HAL_CAN_Send_Message(&hcan1, &TxMessage, 200);
}
*/

// COM port interface
// This function gets called by VisionShared to send a UART message out to a module
void APP_COM_Vision_COM_TX(uint8_t COM, uint8_t* data, uint8_t length);
#define VisionSendCOM_message(COM, data, len) APP_COM_Vision_COM_TX(COM, data, len)

/* ---- Add application level function (host) ----
void APP_COM_Vision_COM_TX(uint8_t COM, uint8_t* data, uint8_t length)
{
	uint8_t* data_frame;
	uint8_t data_length = 0;

	// ---- Get message frame (SOF, CRC, EOF) ----
	data_frame = get_frame(data, length, &data_length);

	USART_Sendarray(COM, length, data_length);
}
*/

#endif /* VISION_HAL_H_ */
