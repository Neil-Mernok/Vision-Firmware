/*
 * F103_SPI.c
 * Created on: Oct 11, 2012
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#include "F103_SPI.h"

static const int SPI_Timeout = 1000;

void SPI1_Config(void)
{
	/* Configure SPI_OUTPUT pins: SCK and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_SCK | SPI1_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

	/* Configure SPI_INPUT pins: MISO ------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

	/* Enable the SPI clock */
	if(IS_RCC_APB1_PERIPH(SPI1_CLK))
		RCC_APB1PeriphClockCmd(SPI1_CLK, ENABLE);
	else
		RCC_APB2PeriphClockCmd(SPI1_CLK, ENABLE);
	
	/* Enable GPIO clocks */
	RCC_APB1PeriphClockCmd(SPI1_GPIO_CLK, ENABLE);

	SPI_change_mode(SPI1, true);

//	/* SPI configuration -------------------------------------------------------*/
//	SPI_I2S_DeInit(SPI1);
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			////*******************
//	SPI_InitStructure.SPI_CRCPolynomial = 7;
//
//	/* Initialise the SPI communication */
//	SPI_Init(SPI1, &SPI_InitStructure);
//	//enable SPI
//	SPI_Cmd(SPI1, ENABLE);
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @retval The value of the received byte.
 */
uint8_t send_spi(SPI_TypeDef* SPIx, uint8_t data)
{
	long timeout = 0;
	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		;
	/* Send byte through SPI Peripheral */
	SPI_I2S_SendData(SPIx, data);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
	{
		if (timeout++ > SPI_Timeout)
			break;
	}

	/* Return the result */
	return SPI_I2S_ReceiveData(SPIx);
}

void SPI2_Config(void)
{
	/* Configure SPI_OUTPUT pins: SCK and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI2_PIN_SCK | SPI2_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

	//// We dont need Miso for Vision. SPI 2 is used for LED shift registers. 
	/* Configure SPI_INPUT pins: MISO ------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI2_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//---------------------------- this was changed for NTRX chip
	GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(SPI2_CLK, ENABLE);

	/* Enable GPIO clocks */
	RCC_APB1PeriphClockCmd(SPI2_GPIO_CLK, ENABLE);

	SPI_change_mode(SPI2, true);

	//	/* SPI configuration -------------------------------------------------------*/
//	SPI_I2S_DeInit(SPI2);
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			////*******************
//	SPI_InitStructure.SPI_CRCPolynomial = 7;
//
//	/* Initialise the SPI communication */
//	SPI_Init(SPI2, &SPI_InitStructure);
//	//enable SPI
//	SPI_Cmd(SPI2, ENABLE);
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @retval The value of the received byte.
 */
uint8_t send_spi2(uint8_t data)
{
	long timeout = 0;
	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	/* Send byte through SPI Peripheral */
	SPI_I2S_SendData(SPI2, data);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
		if (timeout++ > SPI_Timeout)
			break;
	}

	/* Return the result */
	return SPI_I2S_ReceiveData(SPI2);
}

void SPI3_Config(void)
{
	/* Configure SPI_OUTPUT pins: SCK and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI3_PIN_SCK | SPI3_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI3_GPIO, &GPIO_InitStructure);

	/* Configure SPI_INPUT pins: MISO ------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI3_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//---------------------------- this was changed for NTRX chip
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI3_GPIO, &GPIO_InitStructure);

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(SPI3_CLK, ENABLE);

	/* Enable GPIO clocks */
	RCC_APB1PeriphClockCmd(SPI3_GPIO_CLK, ENABLE);

	SPI_change_mode(SPI3, true);
}

/**
 * @brief This function is used to configure the SPI peripheral's polarity, for the event that devices requiring inverse polarity are needed on one SPI bus. 
 * @param SPIx
 * @param rising
 */
void SPI_change_mode(SPI_TypeDef* SPIx, int rising)
{
	static SPI_InitTypeDef SPI_norm =
	{ SPI_Direction_2Lines_FullDuplex, SPI_Mode_Master, SPI_DataSize_8b, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_NSS_Soft, SPI_BaudRatePrescaler_32, SPI_FirstBit_MSB, 7 };
	static int SPI1_rising = -1, SPI2_rising = -1, SPI3_rising = -1;			// indicated whether the SPI is setup for rising edge or falling edge.

	if (SPIx == SPI1)
	{
		if (SPI1_rising == rising)
			return;
		else
			SPI1_rising = rising;
	}
	if (SPIx == SPI2)
	{
		if (SPI2_rising == rising)
			return;
		else
			SPI2_rising = rising;
	}
	if (SPIx == SPI2)
	{
		if (SPI3_rising == rising)
			return;
		else
			SPI3_rising = rising;
	}

//disable SPI
	SPI_Cmd(SPIx, DISABLE);

// reset the SPI hardware.
	SPI_I2S_DeInit(SPIx);

	if (rising == true)
		SPI_norm.SPI_CPHA = SPI_CPHA_1Edge;
	else
		SPI_norm.SPI_CPHA = SPI_CPHA_2Edge;

	/* Initialise the SPI communication */
	SPI_Init(SPIx, &SPI_norm);
//enable SPI
	SPI_Cmd(SPIx, ENABLE);
}

