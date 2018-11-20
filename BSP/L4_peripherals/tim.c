/**
 ******************************************************************************
 * File Name          : TIM.c
 * Description        : This file provides code for the configuration
 *                      of the TIM instances.
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
#include "tim.h"
#include "Global_Variables.h"

/* USER CODE BEGIN 0 */
// Timers are used as follows:
//		Tim2 - Uart packet
//		Tim3 - LF PWM
//		Tim4 - LF frequency tuning
//		Tim7 - 
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
//	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	//htim2.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / 1000000;
	htim2.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / 5500000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 3000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

//	sConfigOC.OCMode = TIM_OCMODE_TIMING;
//	sConfigOC.Pulse = 3000;
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	
	// start the ISR to wait for the uart timer
//	HAL_TIM_Base_Start_IT()
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	
	HAL_TIM_Base_Stop(&htim2);
}

/**
 * @brief  restart timer 2 with 0 value
 * @param  None
 * @retval None
 */
void TimerUart_Restart(void)
{
	HAL_TIM_Base_Stop(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_TIM_Base_Start(&htim2);
}

/**
 * @brief  restart timer 4 with 0 value
 * @param  None
 * @retval None
 */
void Timer_GPSModule_USART_Restart(void)
{
	HAL_TIM_Base_Stop(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Base_Start_IT(&htim4);
}

/* TIM3 init function */
void MX_TIM3_Init(int frequency)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
//	TIM_MasterConfigTypeDef sMasterConfig;
//	TIM_OC_InitTypeDef sConfigOC;
	uint32_t period = 0;
	
	period = (HAL_RCC_GetPCLK1Freq() / frequency);
		
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim3.Init.Period = period-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim3);

	PWM_pins_change(false);
	PWM_set_period(49, frequency);
}

#ifdef LF_TX_Capable 

/**
 * Brief: this enables or disables the PWM pins. 
 * @param PWM
 */
void PWM_pins_change(bool PWM)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (PWM)
	{
		/**
		 TIM3 GPIO Configuration    
		 PC6     ------> TIM3_CH1
		 PC7     ------> TIM3_CH2 
		 */
		GPIO_InitStruct.Pin = LF_PWM1_PIN | LF_PWM2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(LF_PWM1_PORT, &GPIO_InitStruct);
	}
	else
	{
		// switch to normal GPIOs
		GPIO_InitStruct.Pin = LF_PWM1_PIN | LF_PWM2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(LF_PWM1_PORT, &GPIO_InitStruct);
		// make sure they're low.
		HAL_GPIO_WritePin(LF_PWM1_PORT, LF_PWM1_PIN | LF_PWM2_PIN, GPIO_PIN_RESET);
	}
}

/**
 * @brief	set the PWM pulse width power in percentage. 0-100.
 */
void PWM_set_period(uint32_t percentage, uint32_t freq)
{
	uint32_t period = 0;
	TIM_OC_InitTypeDef sConfigOC;

	/// make sure not to use more than 100%
	if (percentage > 100)
		percentage = 100;

	/// make sure frequency is also sane
	if (freq > 150000)
		freq = 150000;
	if (freq < 30000)
		freq = 30000;

	if (percentage > 50)
	{
		period = (HAL_RCC_GetPCLK1Freq() / freq / 2);

		// normal centre aligned PWM, with 2 channels of inverse polarity 
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = percentage * period / 200 - 1;				// value is 0 -> period/2
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	}
	else
	{
		period = (HAL_RCC_GetPCLK1Freq() / freq);

		// normal PWM, with 2 different periods and the difference in period determines power 
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = ((100 + percentage) * period) / 200 - 1;		// value is Period/2 + Period/2*percentage
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = ((100 - percentage) * period) / 200 - 1;		// value is Period/2 - Period/2*percentage
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	}

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	/* Time base configuration */
	htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim3.Init.Period = period-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);
}
#endif

/* TIM4 init function */
void MX_TIM4_Init(void)
{
//	TIM_ClockConfigTypeDef sClockSourceConfig;
//	TIM_MasterConfigTypeDef sMasterConfig;
//
//	htim4.Instance = TIM4;
//	htim4.Init.Prescaler = 0;
//	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim4.Init.Period = 65535;
//	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	HAL_TIM_Base_Init(&htim4);
//
//	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//	HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
//
//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
//
//	TIM_ClockConfigTypeDef sClockSourceConfig;
//	TIM_MasterConfigTypeDef sMasterConfig;
//	TIM_OC_InitTypeDef sConfigOC;

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / 1000000;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 3000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim4);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	HAL_TIM_Base_Stop(&htim4);
}

/* TIM6 init function
 * TIM6 is used for a precision microsecond delay 
 */
void MX_TIM6_Init(void)
{
//	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	HAL_TIM_Base_Init(&htim7);
	/* Configure the OPM Mode */
	htim6.Instance->CR1 |= TIM_OPMODE_SINGLE;
}

/* TIM6 init function
 * TIM6 is used for a precision microsecond delay 
 */
void MX_TIM6_DelayUs(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	htim6.Instance->ARR = us;		// set the time to run. 
	__HAL_TIM_ENABLE(&htim6);		// start timer from 0 to US
	
	while((htim6.Instance->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)
	{
		__NOP();
	}	
}


/* TIM7 init function
 * TIM7 is used to time the LF frequecy to tune capacitors. 
 */
void MX_TIM7_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 0;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 65535;
	HAL_TIM_Base_Init(&htim7);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);
	
	HAL_TIM_Base_Start(&htim7);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if (htim_base->Instance == TIM2)
	{
		/* USER CODE BEGIN TIM2_MspInit 0 */

		/* USER CODE END TIM2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE()
		;

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		/* USER CODE BEGIN TIM2_MspInit 1 */

		/* USER CODE END TIM2_MspInit 1 */
	}
	else if (htim_base->Instance == TIM3)
	{
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE()
		;

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(TIM3_IRQn, 9, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
		
		/**TIM3 GPIO Configuration    
		 PC6     ------> TIM3_CH1
		 PC7     ------> TIM3_CH2 
		 */
		GPIO_InitStruct.Pin = LF_PWM1_PIN | LF_PWM2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(LF_PWM1_PORT, &GPIO_InitStruct);	
	}
	else if (htim_base->Instance == TIM4)
	{
		/* USER CODE BEGIN TIM4_MspInit 0 */

		/* USER CODE END TIM4_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE()
		;

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspInit 1 */

		/* USER CODE END TIM4_MspInit 1 */
	}
	else if (htim_base->Instance == TIM6)
	{
		__HAL_RCC_TIM6_CLK_ENABLE();
	}
	else if (htim_base->Instance == TIM7)
	{
		/* USER CODE BEGIN TIM7_MspInit 0 */

		/* USER CODE END TIM7_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM7_CLK_ENABLE()
		;
		/* USER CODE BEGIN TIM7_MspInit 1 */

		/* USER CODE END TIM7_MspInit 1 */
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

	if (htim_base->Instance == TIM2)
	{
		/* USER CODE BEGIN TIM2_MspDeInit 0 */

		/* USER CODE END TIM2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(TIM2_IRQn);

		/* USER CODE BEGIN TIM2_MspDeInit 1 */

		/* USER CODE END TIM2_MspDeInit 1 */
	}
	else if (htim_base->Instance == TIM3)
	{
		/* USER CODE BEGIN TIM3_MspDeInit 0 */

		/* USER CODE END TIM3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(TIM3_IRQn);

		/* USER CODE BEGIN TIM3_MspDeInit 1 */

		/* USER CODE END TIM3_MspDeInit 1 */
	}
	else if (htim_base->Instance == TIM4)
	{
		/* USER CODE BEGIN TIM4_MspDeInit 0 */

		/* USER CODE END TIM4_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM4_CLK_DISABLE();

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(TIM4_IRQn);

		/* USER CODE BEGIN TIM4_MspDeInit 1 */

		/* USER CODE END TIM4_MspDeInit 1 */
	}
	else if (htim_base->Instance == TIM6)
	{
		__HAL_RCC_TIM6_CLK_DISABLE();
	}
	else if (htim_base->Instance == TIM7)
	{
		/* USER CODE BEGIN TIM7_MspDeInit 0 */

		/* USER CODE END TIM7_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM7_CLK_DISABLE();
		/* USER CODE BEGIN TIM7_MspDeInit 1 */

		/* USER CODE END TIM7_MspDeInit 1 */
	}
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
