/**
 ******************************************************************************
 * File Name          : USART.c
 * Description        : This file provides code for the configuration
 *                      of the USART instances.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart1 = {.Instance = USART1};
UART_HandleTypeDef huart2 = {.Instance = USART2};
UART_HandleTypeDef huart3 = {.Instance = USART3};

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;

UART_HandleTypeDef* Master_COM = &huart3;
UART_HandleTypeDef* uBlox_COM = &huart2;

/* USART3 init function */

void MX_USART_UART_Init(UART_HandleTypeDef *USARTx, int baud)
{

	//USARTx->Instance = USARTx;
	USARTx->Init.BaudRate = baud;
	USARTx->Init.WordLength = UART_WORDLENGTH_8B;
	USARTx->Init.StopBits = UART_STOPBITS_1;
	USARTx->Init.Parity = UART_PARITY_NONE;
	USARTx->Init.Mode = UART_MODE_TX_RX;
	USARTx->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	USARTx->Init.OverSampling = UART_OVERSAMPLING_16;
	USARTx->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	USARTx->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	USARTx->AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_DeInit(USARTx);
	HAL_UART_Init(USARTx);
	
	/* Enable the UART Data Register not empty Interrupt */
	LL_USART_EnableIT_RXNE(USARTx->Instance);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (huart->Instance == USART3)
	{
		/* USER CODE BEGIN USART3_MspInit 0 */

		/* USER CODE END USART3_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USART3_CLK_ENABLE();

		/**USART3 GPIO Configuration    
		 PB10     ------> USART3_TX
		 PB11     ------> USART3_RX
		 First, set RX pin as interrupt to setup EXTI, but leave ISR disabled. 
		 Then setup uart mode as usual, but leave TX without pullup  
		 */
		GPIO_InitStruct.Pin = RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(RX_GPIO_Port, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(RX_GPIO_Port, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = TX_Pin;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(TX_GPIO_Port, &GPIO_InitStruct); 
					
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		
		/* Peripheral DMA init*/

		hdma_usart3_tx.Instance = DMA1_Channel2;
		hdma_usart3_tx.Init.Request = DMA_REQUEST_2;
		hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart3_tx.Init.Mode = DMA_NORMAL;
		hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
		HAL_DMA_Init(&hdma_usart3_tx);

		__HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
		/* USER CODE BEGIN USART3_MspInit 1 */

		/* USER CODE END USART3_MspInit 1 */
	}
	else if (huart->Instance == USART2)
	{
		/* USER CODE BEGIN USART2_MspInit 0 */

		/* USER CODE END USART2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		/**USART1 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA3     ------> USART2_RX
		 */
		GPIO_InitStruct.Pin = TX2_Pin | RX2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_usart2_tx.Instance = DMA1_Channel7;
		hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
		hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_tx.Init.Mode = DMA_NORMAL;
		hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
		HAL_DMA_Init(&hdma_usart2_tx);

		__HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);

		hdma_usart2_rx.Instance = DMA1_Channel6;
		hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
		hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;

		HAL_DMA_Init(&hdma_usart2_rx);

		__HAL_LINKDMA(huart, hdmatx, hdma_usart2_rx);

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(USART2_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		/* USER CODE BEGIN USART2_MspInit 1 */

		/* USER CODE END USART2_MspInit 1 */
	}
	else if (huart->Instance == USART1)
	{
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE()
		;

		/**USART1 GPIO Configuration    
		 PA9     ------> USART1_TX
		 PA10    ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = TX1_Pin | RX1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_usart1_tx.Instance = DMA1_Channel4;
		hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
		hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart1_tx.Init.Mode = DMA_NORMAL;
		hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
		HAL_DMA_Init(&hdma_usart1_tx);

		__HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		/* USER CODE BEGIN USART1_MspInit 1 */

		/* USER CODE END USART1_MspInit 1 */
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

	if (huart->Instance == USART3)
	{
		/* USER CODE BEGIN USART3_MspDeInit 0 */

		/* USER CODE END USART3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART3_CLK_DISABLE();

		/**USART3 GPIO Configuration    
		 PB10     ------> USART3_TX
		 PB11     ------> USART3_RX 
		 */
		HAL_GPIO_DeInit(GPIOB, TX_Pin | RX_Pin);

		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(huart->hdmatx);

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(USART3_IRQn);

	}
	else if (huart->Instance == USART1)
	{
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration    
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX 
		 */
		HAL_GPIO_DeInit(GPIOA, TX1_Pin | RX1_Pin);

		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(huart->hdmatx);

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(USART1_IRQn);

	}
	/* USER CODE BEGIN USART3_MspDeInit 1 */

	/* USER CODE END USART3_MspDeInit 1 */
}


/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
