/*
 * F103_CAN.c
 *
 *  Created on: Nov 1, 2012
 *      Author: Kobus Goosen
 */

#include "F103_CAN.h"

#define CAN_bit_quanta 18

/**
 * @brief  Configures the CAN.
 * @param  this is the actual baud rate of the can bus. it MUST be one of the standard values: 10k, 20k, 50k,100k, 125k, 250k
 * @retval None
 */
void CAN_Config(uint32_t baudrate)
{
	CAN_InitTypeDef				CAN_InitStructure;

#ifdef STM32F10X_CL
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
#endif
#ifdef STM32F10X_HD
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
#endif
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* GPIO clock enable */
	/* __CAN1_USED__ */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN1, ENABLE);

	/* Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN1_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CAN1, &GPIO_InitStructure);

	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN1_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIO_CAN1, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remapping_CAN1, ENABLE);

	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;			/////////
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;			/// enable TX FIFO order. this ensures that messages are sent in the correct order for segmented transmission. 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = RCC_Clocks.PCLK1_Frequency/CAN_bit_quanta/baudrate;
	CAN_Init(CAN1, &CAN_InitStructure);
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
 * @brief  Configure a specific ID as a CAN filter.
 * @param  filter_id the total CAN ID you want to be able to store.
 * @param  num the CAN filter bank you want to use. 
 * @retval None
 */
void CAN_setup_filter(uint32_t filter_id, uint8_t num)
{
	CAN_FilterInitTypeDef   	filterInit;
	uint32_t mask = 0x1FFFF4FF;
	
	/* CAN filter init */
	filterInit.CAN_FilterNumber = num;
	filterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
	filterInit.CAN_FilterScale = CAN_FilterScale_32bit;
	filterInit.CAN_FilterIdHigh = (uint16_t) (filter_id >> 13);	
	filterInit.CAN_FilterIdLow = (uint16_t) (filter_id << 3);
	filterInit.CAN_FilterMaskIdHigh = (uint16_t) (mask >> 13);
	filterInit.CAN_FilterMaskIdLow = (uint16_t) (mask << 3);
	filterInit.CAN_FilterFIFOAssignment = 0;
	filterInit.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&filterInit);
}

/**
 * @brief  Initializes a Rx Message.
 * @param  CanRxMsg *RxMessage
 * @retval None
 */
void Init_RxMes(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	RxMessage->StdId = 0x00;
	RxMessage->ExtId = 0x00;
	RxMessage->IDE = CAN_ID_EXT;
	RxMessage->DLC = 0;
	RxMessage->FMI = 0;
	for (i = 0; i < 8; i++)
	{
		RxMessage->Data[i] = 0x00;
	}
}

