/*
 * F207_Uart.c
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */
#include "F103_Uart.h"

USART_TypeDef* COM_USART[COMn] =
{ USART1, USART2, USART3, UART4 };
GPIO_TypeDef* COM_TX_PORT[COMn] =
{ USART1_TX_GPIO_PORT, USART2_TX_GPIO_PORT, USART3_TX_GPIO_PORT, UART4_TX_GPIO_PORT };
GPIO_TypeDef* COM_RX_PORT[COMn] =
{ USART1_RX_GPIO_PORT, USART2_RX_GPIO_PORT, USART3_RX_GPIO_PORT, UART4_RX_GPIO_PORT };

const uint32_t COM_USART_CLK[COMn] =
{ RCC_APB2Periph_USART1, RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_UART4 };
const uint32_t COM_RX_PORT_CLK[COMn] =
{ USART1_RX_GPIO_CLK, USART2_RX_GPIO_CLK, USART3_RX_GPIO_CLK, UART4_RX_GPIO_CLK };
const uint32_t COM_TX_PORT_CLK[COMn] =
{ USART1_TX_GPIO_CLK, USART2_TX_GPIO_CLK, USART3_TX_GPIO_CLK, UART4_TX_GPIO_CLK };
const uint16_t COM_TX_PIN[COMn] =
{ USART1_TX_PIN, USART2_TX_PIN, USART3_TX_PIN, UART4_TX_PIN };
const uint16_t COM_RX_PIN[COMn] =
{ USART1_RX_PIN, USART2_RX_PIN, USART3_RX_PIN, UART4_RX_PIN };
const uint8_t COM_TX_PIN_SOURCE[COMn] =
{ USART1_TX_SOURCE, USART2_TX_SOURCE, USART3_TX_SOURCE, UART4_TX_SOURCE };
const uint8_t COM_RX_PIN_SOURCE[COMn] =
{ USART1_RX_SOURCE, USART2_RX_SOURCE, USART3_RX_SOURCE, UART4_RX_SOURCE };
const uint32_t COM_AF[COMn] =
{ 0, 0, 0, 0 };
const uint8_t COM_IRQ[COMn] =
{ USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn };

/**
 * @brief  Configures COM port this is with receive interrupt enabled
 * @param  COM: Specifies the COM port to be configured.
 *   This parameter can be one of following parameters:
 *     @arg COM_PC
 *     @arg COM_SlaveuC
 *     @arg COM_LSR
 * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
 *   contains the configuration information for the specified USART peripheral.
 * @retval None
 */
void COMInit_INT(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
	/* Setup the USARTx Interrupt Controller */
	NVIC_InitStructure.NVIC_IRQChannel = COM_IRQ[COM];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);
	if (COM == COM_1)
		RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	else if (COM == COM_2)
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	else if (COM == COM_3)
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	else if (COM == COM_4)
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);

	/* Connect Pins */
	if (COM_AF[COM] != 0)
		GPIO_PinRemapConfig(COM_AF[COM], ENABLE);

	/* Configure USART Tx Pin  */
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

	/* Configure USART Rx Pin  */
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

	/* USART configuration */
	USART_Init(COM_USART[COM], USART_InitStruct);

	/* Enable the USART Receive Interrupt */
	USART_ITConfig(COM_USART[COM], USART_IT_RXNE, ENABLE);

	/* Enable USART */
	USART_Cmd(COM_USART[COM], ENABLE);
}

/**
 * @brief  Configures COM port.
 * @param  COM: Specifies the COM port to be configured.
 *   This parameter can be one of following parameters:
 *     @arg1 COMx
 *     @arg1 Baudrate
 * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
 *   contains the configuration information for the specified USART peripheral.
 * @retval None
 */
void USART_Configure(COM_TypeDef COM, uint32_t BaudRate)
{
	/* USARTx configured as follow:
	 - BaudRate = BaudRate
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	COMInit_INT(COM, &USART_InitStructure);
}

/**
 * @brief  sends data over the UART with DMA to save processor time.
 * @param COM uart to sent data to 
 * @param buf buffer that holds the data to be sent 
 * @param len holds the number of bytes to send.
 * @retval None
 */
int USART_SendDMA(COM_TypeDef COM, uint8_t* buf, uint32_t len)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_Channel_TypeDef* ch;
	uint32_t DMA_flag;
	static uint32_t DMA_flag_first = 0;
	static uint8_t swap[2][256];
	static uint8_t swapch = 0;

	switch (COM)
	{
	case COM_1:
		// DMA1_CH4 is used for UART1_TX. check reference manual. 
		ch = DMA1_Channel4;
		DMA_flag = DMA1_FLAG_TC4;
		break;
	case COM_2:
		// DMA1_CH7 is used for UART2_TX. check reference manual. 
		ch = DMA1_Channel7;
		DMA_flag = DMA1_FLAG_TC7;
		break;
	case COM_3:
		// DMA1_CH2 is used for UART3_TX. check reference manual. 
		ch = DMA1_Channel2;
		DMA_flag = DMA1_FLAG_TC2;
		break;
	case COM_4:
		// DMA2_CH5 is used for UART4_TX. check reference manual. 
		ch = DMA2_Channel5;
		DMA_flag = DMA2_FLAG_TC5;
		break;
	}

	// enbale clock for DMA. only really needed once 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	/* Wait until USART TX DMA1 Channel Transfer Complete */
	if ((DMA_flag & DMA_flag_first) == 0)		//this is the first time we've used this channel. 
	{
		DMA_flag_first |= DMA_flag;
	}
	else
	{
		if (DMA_GetFlagStatus(DMA_flag) == RESET)
		{
			return 0;							// DMA was busy, so just say that we sent nothing. 
		}
	}

	// copy data. the problem is that the moment the process returns, it usually starts to write data to the buffer again, currupting what is sent to the uart. 
	memcpy(swap[swapch & 0x01], buf, len);
	/* USART_Tx_DMA_Channel (triggered by USART Tx event) Config */
	DMA_DeInit(ch);
	// DMA_InitStructure.DMA_PeripheralBaseAddr = USARTy_DR_Base;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(COM_USART[COM]->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) swap[swapch & 0x01];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = len;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(ch, &DMA_InitStructure);
	swapch++;

	/* Enable the USART DMA TX request */
	USART_DMACmd(COM_USART[COM], USART_DMAReq_Tx, ENABLE);

	/* Clear the TC bit in the SR register by writing 0 to it */
	USART_ClearFlag(COM_USART[COM], USART_FLAG_TC);

	/* Enable the DMA TX Stream, USART will start sending */
	DMA_Cmd(ch, ENABLE);

	return len;
}

/**
 * @brief  Sends a string terminated by NULL
 * @param  COM: Specifies the COM port to be used
 * @param  String to be sent, must be terminated by NULL
 * @retval None
 */
void USART_SendString(COM_TypeDef COM, char Data[])
{
	short int counter = 0;

	while (Data[counter] != 0)
	{
		USART_SendData(COM_USART[COM], (uint8_t) Data[counter]);

		while (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TC) == RESET)
		{
		}
		counter++;
	}
}

/**
 * @brief  Sends an array, must specify length to send
 * @param  COM: Specifies the COM port to be used
 * @param  length in amount of bytes to send
 * @retval None
 */
void USART_Sendarray(COM_TypeDef COM, uint8_t* Data, uint8_t length)
{
	int i;
	for (i = 0; i < length; i++)
	{
		USART_SendData(COM_USART[COM], (uint8_t) Data[i]);
		while (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TC) == RESET)
		{
		}
	}

}

/**
 * @brief  Sends a single byte to the specified uart
 * @param  COM: Specifies the COM port to be used
 * @param  payload byte
 * @retval None
 */
void USART_Sendbyte(COM_TypeDef COM, uint8_t Data)
{
	USART_SendDataCustom(COM_USART[COM], (uint8_t) Data);
	while (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TC) == RESET)
	{
	}
}

/**
 * @brief  Sends a single custom byte to the specified uart
 * @param  COM: Specifies the COM port to be used
 * @param  payload byte
 * @retval None
 */
void USART_SendDataCustom(USART_TypeDef* USARTx, uint16_t Data)

{
	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_DATA(Data));

	/* Transmit Data */
	USARTx->DR = Data;
}
