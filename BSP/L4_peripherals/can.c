/**
 ******************************************************************************
 * File Name          : CAN.c
 * Description        : This file provides code for the configuration
 *                      of the CAN instances.
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
#include "can.h"

#include "Global_Variables.h"

/* USER CODE BEGIN 0 */
#define CAN_bit_quanta 12
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void CAN_Config(uint32_t baudrate)
{
//	CAN_FilterConfTypeDef filter;
	static CanTxMsgTypeDef Tx;
	static CanRxMsgTypeDef Rx;

	hcan1.pTxMsg = &Tx;
	hcan1.pRxMsg = &Rx;
	
	/*##-1- Configure the CAN peripheral #######################################*/
	hcan1.Instance = CAN;
	hcan1.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / CAN_bit_quanta / baudrate;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_2TQ;
	hcan1.Init.BS1 = CAN_BS1_9TQ;
	hcan1.Init.BS2 = CAN_BS2_2TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = ENABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = ENABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = ENABLE;
	HAL_CAN_DeInit(&hcan1);
	HAL_CAN_Init(&hcan1);
	
	// start receiving
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}

void CAN_setup_filter(uint32_t filter_id, uint8_t num)
{
	CAN_FilterConfTypeDef filter;
	uint32_t mask = 0x1FFFF4FF;

	filter.FilterNumber = num;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = (uint16_t) (filter_id >> 13);
	filter.FilterIdLow = (uint16_t) (filter_id << 3);
	filter.FilterMaskIdHigh = (uint16_t) (mask >> 13);
	filter.FilterMaskIdLow = (uint16_t) (mask << 3);
	filter.FilterFIFOAssignment = 0;
	filter.FilterActivation = ENABLE;
	filter.BankNumber = num;

	HAL_CAN_ConfigFilter(&hcan1, &filter);
}


void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hcan->Instance == CAN)
	{
		/* USER CODE BEGIN CAN_MspInit 0 */

		/* USER CODE END CAN_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE()
		;

		/**CAN1 GPIO Configuration    
		 PB8     ------> CAN1_RX
		 PB9     ------> CAN1_TX 
		 */
		GPIO_InitStruct.Pin = CAN_RX_Pin | CAN_TX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		/* USER CODE BEGIN CAN_MspInit 1 */

		/* USER CODE END CAN_MspInit 1 */
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

	if (hcan->Instance == CAN)
	{
		/* USER CODE BEGIN CAN_MspDeInit 0 */

		/* USER CODE END CAN_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN1 GPIO Configuration    
		 PB8     ------> CAN1_RX
		 PB9     ------> CAN1_TX 
		 */
		HAL_GPIO_DeInit(GPIOB, CAN_RX_Pin | CAN_TX_Pin);

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

	}
	/* USER CODE BEGIN CAN_MspDeInit 1 */

	/* USER CODE END CAN_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
