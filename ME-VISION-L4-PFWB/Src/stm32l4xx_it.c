/**
 ******************************************************************************
 * @file    stm32l4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include "tim.h"
#include "Global_Variables.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
//extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
//extern I2C_HandleTypeDef hi2c1;
//extern LPTIM_HandleTypeDef hlptim1;
//extern DMA_HandleTypeDef hdma_lpuart_tx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern RTC_HandleTypeDef hrtc;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN HardFault_IRQn 1 */

	/* USER CODE END HardFault_IRQn 1 */
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN MemoryManagement_IRQn 1 */

	/* USER CODE END MemoryManagement_IRQn 1 */
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN BusFault_IRQn 1 */

	/* USER CODE END BusFault_IRQn 1 */
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN UsageFault_IRQn 1 */

	/* USER CODE END UsageFault_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	while (1)
	{
	}
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles RTC wake-up interrupt through EXTI line 20.
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
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI0_IRQn 0 */

	/* USER CODE END EXTI0_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	/* USER CODE BEGIN EXTI0_IRQn 1 */

	/* USER CODE END EXTI0_IRQn 1 */
}

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI1_IRQn 0 */

	/* USER CODE END EXTI1_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	/* USER CODE BEGIN EXTI1_IRQn 1 */

	/* USER CODE END EXTI1_IRQn 1 */
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
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 */
//void DMA1_Channel1_IRQHandler(void)
//{
//	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
//
//	/* USER CODE END DMA1_Channel1_IRQn 0 */
//	HAL_DMA_IRQHandler(&hdma_adc1);
//	/* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
//
//	/* USER CODE END DMA1_Channel1_IRQn 1 */
//}

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void DMA1_Channel2_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

	/* USER CODE END DMA1_Channel2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_tx);
	/* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

	/* USER CODE END DMA1_Channel2_IRQn 1 */
}

void CAN_packet_handler(uint8_t* data, uint8_t len, bool Last);
/**
 * @brief This function handles CAN1 RX0 interrupt.
 */
void CAN1_RX0_IRQHandler(void)
{
	static CanRxMsgTypeDef RX;
//	uint8_t slave_ID = 0;
//	uint8_t data[8];
	
	/* Check End of reception flag for FIFO0 */
	if ((__HAL_CAN_GET_IT_SOURCE( &hcan1, CAN_IT_FMP0)) && (__HAL_CAN_MSG_PENDING(&hcan1, CAN_FIFO0) != 0))
	{
		/* Get the Id */
		RX.IDE = (uint8_t) 0x04 & hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RIR;
		if (RX.IDE == CAN_ID_STD)
			RX.StdId = (uint32_t) 0x000007FF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RIR >> 21);
		else
			RX.ExtId = (uint32_t) 0x1FFFFFFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RIR >> 3);
		/* Get the message params */
		RX.RTR = (uint8_t) 0x02 & hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RIR;
		RX.DLC = (uint8_t) 0x0F & hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDTR;
		RX.FMI = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDTR >> 8);
		/* Get the data field */
		RX.Data[0] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDLR);
		RX.Data[1] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 8);
		RX.Data[2] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 16);
		RX.Data[3] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDLR >> 24);
		RX.Data[4] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDHR);
		RX.Data[5] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 8);
		RX.Data[6] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 16);
		RX.Data[7] = (uint8_t) 0xFF & (hcan1.Instance->sFIFOMailBox[CAN_FIFO0].RDHR >> 24);
		__HAL_CAN_FIFO_RELEASE(&hcan1, CAN_FIFO0);
		
		if (((RX.ExtId & (~CAN_not_Last)) == (CAN_VisBootPoll_ID)) && (RX.IDE == CAN_ID_EXT))
		{
			bool last;
//			slave_ID = RX.ExtId & 0xFF;

			if (RX.ExtId & CAN_not_Last)
				last = false;
			else
				last = true;

			CAN_packet_handler(RX.Data, RX.DLC, last);

		}		
	}
	else
		HAL_CAN_IRQHandler(&hcan1);
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
//	if ((__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET) && (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC1) != RESET))
//	{
//		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
//
//		LL_TIM_DisableCounter(htim2.Instance);
//
//		Uart_packet_handler();
//	}

	HAL_TIM_IRQHandler(&htim2);
	HAL_TIM_Base_Stop(&htim2);
	
	memcpy(boot_data, uart_data, uart_buf_counter);
	boot_buf_counter = uart_buf_counter;
	uart_buf_counter = 0;
	UPDATE_Flags.master = COM;
	UPDATE_Flags.Master_last_seen = time_now()-5;
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
	/* USER CODE BEGIN TIM4_IRQn 0 */

	/* USER CODE END TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim4);
	/* USER CODE BEGIN TIM4_IRQn 1 */

	/* USER CODE END TIM4_IRQn 1 */
}

///**
// * @brief This function handles I2C1 error interrupt.
// */
//void I2C1_ER_IRQHandler(void)
//{
//	/* USER CODE BEGIN I2C1_ER_IRQn 0 */
//
//	/* USER CODE END I2C1_ER_IRQn 0 */
//	HAL_I2C_ER_IRQHandler(&hi2c1);
//	/* USER CODE BEGIN I2C1_ER_IRQn 1 */
//
//	/* USER CODE END I2C1_ER_IRQn 1 */
//}

///**
// * @brief This function handles LPTIM1 global interrupt.
// */
//void LPTIM1_IRQHandler(void)
//{
//	/* USER CODE BEGIN LPTIM1_IRQn 0 */
//
//	/* USER CODE END LPTIM1_IRQn 0 */
//	HAL_LPTIM_IRQHandler(&hlptim1);
//	/* USER CODE BEGIN LPTIM1_IRQn 1 */
//	// TODO: increment the upper word of the low power timer. 
//	/* USER CODE END LPTIM1_IRQn 1 */
//}

/**
 * @brief This function handles USB OTG FS global interrupt.
 */
void OTG_FS_IRQHandler(void)
{
	/* USER CODE BEGIN OTG_FS_IRQn 0 */

	/* USER CODE END OTG_FS_IRQn 0 */
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
	/* USER CODE BEGIN OTG_FS_IRQn 1 */

	/* USER CODE END OTG_FS_IRQn 1 */
}

///**
// * @brief This function handles DMA2 channel6 global interrupt.
// */
//void DMA2_Channel6_IRQHandler(void)
//{
//	/* USER CODE BEGIN DMA2_Channel6_IRQn 0 */
//
//	/* USER CODE END DMA2_Channel6_IRQn 0 */
//	HAL_DMA_IRQHandler(&hdma_lpuart_tx);
//	/* USER CODE BEGIN DMA2_Channel6_IRQn 1 */
//
//	/* USER CODE END DMA2_Channel6_IRQn 1 */
//}

uint8_t uart_packet_received;
uint8_t uart_data[2048];
uint16_t uart_buf_counter;

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
	/* Check RXNE flag value in ISR register */
	if (LL_USART_IsActiveFlag_RXNE(huart3.Instance) && LL_USART_IsEnabledIT_RXNE(huart3.Instance))
	{
		/* RXNE flag will be cleared by reading of RDR register (done in call) */
		uart_data[uart_buf_counter++] = LL_USART_ReceiveData8(huart3.Instance);
		uart_buf_counter &= 0x7FF;
		TimerUart_Restart();
	}
	//else
		HAL_UART_IRQHandler(&huart3);
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
	/* Check RXNE flag value in ISR register */
	if (LL_USART_IsActiveFlag_RXNE(huart1.Instance) && LL_USART_IsEnabledIT_RXNE(huart1.Instance))
	{
		/* RXNE flag will be cleared by reading of RDR register (done in call) */
		uart_data[uart_buf_counter++] = LL_USART_ReceiveData8(huart1.Instance);
		uart_buf_counter &= 0x7FF;
		TimerUart_Restart();
	}
	//else
		HAL_UART_IRQHandler(&huart1);
}

///**
// * @brief This function handles LPUART1 global interrupt.
// */
//void LPUART1_IRQHandler(void)
//{
//	/* Check RXNE flag value in ISR register */
//	if (LL_USART_IsActiveFlag_RXNE(hlpuart1.Instance) && LL_USART_IsEnabledIT_RXNE(hlpuart1.Instance))
//	{
//		/* RXNE flag will be cleared by reading of RDR register (done in call) */
//		uart_data[uart_buf_counter++] = LL_USART_ReceiveData8(hlpuart1.Instance);
//		uart_buf_counter &= 0x7FF;
//		TimerUart_Restart();
//	}
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
