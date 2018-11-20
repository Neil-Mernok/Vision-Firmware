/**
 ******************************************************************************
 * @file    stm32l1xx_it.c
 * @date    22/03/2015 22:29:28
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"
/* USER CODE BEGIN 0 */
#include "Global_Variables.h"
void TimerUart_Restart(void);
/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_adc;
extern DMA_HandleTypeDef hdma_usart1_tx;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @brief This function handles Debug Monitor.
 */
void DebugMon_Handler(void)
{
	while (1)
	{
	}
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	while (1)
	{
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	while (1)
	{
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	while (1)
	{
	}
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
	while (1)
	{
	}
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/



/**
 * @brief This function handles DMA1 Channel4 global interrupt.
 */
void DMA1_Channel4_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

	/* USER CODE END DMA1_Channel4_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
	/* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

	/* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
 * @brief This function handles RTC Wakeup through EXTI line 20 interrupt.
 */
void RTC_WKUP_IRQHandler(void)
{
	/* USER CODE BEGIN RTC_WKUP_IRQn 0 */

	/* USER CODE END RTC_WKUP_IRQn 0 */
	HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
	/* USER CODE BEGIN RTC_WKUP_IRQn 1 */

	/* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI3_IRQn 0 */

	/* USER CODE END EXTI3_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	/* USER CODE BEGIN EXTI3_IRQn 1 */

	/* USER CODE END EXTI3_IRQn 1 */
}

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI4_IRQn 0 */

	/* USER CODE END EXTI4_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
	/* USER CODE BEGIN EXTI4_IRQn 1 */

	/* USER CODE END EXTI4_IRQn 1 */
}

uint8_t uart_data[2048];
int uart_buf_counter = 0;

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t tmp_flag = 0, tmp_it_source = 0;

	tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
	tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE);
	/* UART in mode Receiver ---------------------------------------------------*/
	if ((tmp_flag != RESET) && (tmp_it_source != RESET))
	{
		// 23-04-2017	Remove UART Receive
		//uart_data[uart_buf_counter++] = (uint8_t) (huart1.Instance->DR & (uint16_t) 0x00FF);
		//TimerUart_Restart();
	}
	/* USER CODE END USART1_IRQn 0 */
	/////////HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
	/* Capture compare 1 event */
	if ((__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC1) != RESET) && (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_CC1) != RESET))
	{
		__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
		
		HAL_TIM_Base_Stop(&htim4);
		
		memcpy(boot_data, uart_data, uart_buf_counter);
		boot_buf_counter = uart_buf_counter;
		uart_buf_counter = 0;
		UPDATE_Flags.master = COM;
		UPDATE_Flags.Master_last_seen = time_now()-5;
		//Boot_packet_handler();
	}
	
	/* USER CODE END TIM4_IRQn 0 */
	//////////HAL_TIM_IRQHandler(&htim4);
	/* USER CODE BEGIN TIM4_IRQn 1 */

	/* USER CODE END TIM4_IRQn 1 */
}

//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM4)
//	{
//		
//	}
//}

/**
 * @brief This function handles EXTI Line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

	/* USER CODE END EXTI9_5_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI2_IRQn 0 */

	/* USER CODE END EXTI2_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
	/* USER CODE BEGIN EXTI2_IRQn 1 */

	/* USER CODE END EXTI2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
