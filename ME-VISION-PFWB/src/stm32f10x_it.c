/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and 
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "Global_Variables.h"

extern CanRxMsg RxMessage;

/** @addtogroup STM32F10x_StdPeriph_Template
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

//void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
//{
//	/* These are volatile to try and prevent the compiler/linker optimising them
//	 away as the variables never actually get used.  If the debugger won't show the
//	 values of the variables, make them global my moving their declaration outside
//	 of this function. */
//	volatile uint32_t r0;
//	volatile uint32_t r1;
//	volatile uint32_t r2;
//	volatile uint32_t r3;
//	volatile uint32_t r12;
//	volatile uint32_t lr; /* Link register. */
//	volatile uint32_t pc; /* Program counter. */
//	volatile uint32_t psr;/* Program status register. */
//
//	r0 = pulFaultStackAddress[0];
//	r1 = pulFaultStackAddress[1];
//	r2 = pulFaultStackAddress[2];
//	r3 = pulFaultStackAddress[3];
//
//	r12 = pulFaultStackAddress[4];
//	lr = pulFaultStackAddress[5];
//	pc = pulFaultStackAddress[6];
//	psr = pulFaultStackAddress[7];
//
//	/* When the following line is hit, the variables contain the register values. */
//	for (;;)
//		;
//}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	//	__asm volatile
	//	(
	//			" tst lr, #4                                                \n"
	//			" ite eq                                                    \n"
	//			" mrseq r0, msp                                             \n"
	//			" mrsne r0, psp                                             \n"
	//			" ldr r1, [r0, #24]                                         \n"
	//			" ldr r2, handler2_address_const                            \n"
	//			" bx r2                                                     \n"
	//			" handler2_address_const: .word prvGetRegistersFromStack    \n"
	//	);

	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

///**
// * @brief  This function handles SVCall exception.
// * @param  None
// * @retval None
// */
//void SVC_Handler(void)
//{
//}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

//////////	This gets handled by the RTOS	/////////////
/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
 * @brief  This function handles CAN1 Handler.
 * @param  None
 * @retval None
 */
#ifdef STM32F10X_HD
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	if (Vision_Status.USB_Active)
	{
		USB_Istr();
	}
	else if (CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP0 ))
	{
		uint32_t t1, t2, t3;

		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		t1 = RxMessage.ExtId & (~CAN_not_Last);
		t2 = (RangerPOD_Poll_ID | (uint32_t)(Vision_Status.Slave_ID));
		t3 = (RangerPOD_Poll_ID | 0x000f);						// use ID 15 as a global access key
		//if ((RxMessage.ExtId & (~CAN_not_Last) == (RangerPOD_Poll_ID | Vision_Status.Slave_ID)) && (RxMessage.IDE == CAN_ID_EXT ))

		if ((t1 == t2) || (t1 == t3))
		{
			if ((RxMessage.IDE == CAN_ID_EXT ))
			{
				bool last;

				if (RxMessage.ExtId & CAN_not_Last)
				last = false;
				else
				last = true;

				CAN_packet_handler(RxMessage.Data, RxMessage.DLC, last);
			}
		}
	}
}
#endif
#ifdef STM32F10X_CL
///////////////////////////    CAN ISR CODE     /////////////////////////////////////////
void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP0))
	{
		/// update the system health tracker to indicater that CAN is functional. 
				
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		if (((RxMessage.ExtId & (~CAN_not_Last)) == (CAN_VisBootPoll_ID)) && (RxMessage.IDE == CAN_ID_EXT))
		{
			bool last;

			if (RxMessage.ExtId & CAN_not_Last)
				last = false;
			else
				last = true;

			CAN_packet_handler(RxMessage.Data, RxMessage.DLC, last);
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
#endif


uint8_t uart_data[4096];
int uart_buf_counter = 0;



/**
 * @brief  generated when a Uart message times out. used to process uart messages
 * @param  None
 * @retval None
 */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{ //every 100ms
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		TIM_SetCompare1(TIM2, CCR1_Val);
		Timer2_Stop();
		memcpy(boot_data, uart_data, uart_buf_counter);
		boot_buf_counter = uart_buf_counter;
		uart_buf_counter = 0;
		UPDATE_Flags.master = COM;
		UPDATE_Flags.Master_last_seen = time_now()-5;
		//Boot_packet_handler();
	}
}



#ifdef USE_COM_1
/**
 * @brief:	Interrupt handler for Uart1
 * @param  	None
 * @retval 	None
 */
void USART1_IRQHandler(void)
{
	// USART1 RX Interrupt
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		//get value from uart
		uart_data[uart_buf_counter++] = USART_ReceiveData(USART1);
		UPDATE_Flags.master = NONE;
//		UPDATE_Flags.Master_last_seen = time_now();
		Timer2_Restart();
	}
}
#endif

#ifdef USE_COM_2
/**
 * @brief:	Interrupt handler for Uart2
 * @param  	None
 * @retval 	None
 */
void USART2_IRQHandler(void)
{
	uint8_t temp = 0;
	// USART2 RX Interrupt
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		//get value from uart
		temp = USART_ReceiveData(USART2);
	}
}
#endif

#ifdef USE_COM_3
/**
 * @brief:	Interrupt handler for Uart3
 * @param  	None
 * @retval 	None
 */
void USART3_IRQHandler(void)
{
	// USART3 RX Interrupt
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		//get value from uart
		uart_data[uart_buf_counter++] = USART_ReceiveData(USART3);
		UPDATE_Flags.master = NONE;
//		UPDATE_Flags.Master_last_seen = time_now();
		Timer2_Restart();
	}

}
#endif

#ifdef USE_COM_4
/**
 * @brief:	Interrupt handler for Uart4
 * @param  	None
 * @retval 	None
 */
void UART4_IRQHandler(void)
{
	uint8_t temp = 0;
	// USART2 RX Interrupt
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		//get value from uart
		temp = USART_ReceiveData(UART4);
	}
}
#endif

/**
 * @brief  This function handles EXTI15_10_IRQ Handler.
 * @param  None
 * @retval None
 */
#ifdef USE_USB_OTG_FS
extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE *pdev);
#endif

#ifdef USE_USB_OTG_FS  
void OTG_FS_WKUP_IRQHandler(void)
{
	if (USB_OTG_dev.cfg.low_power)
	{
		*(uint32_t *) (0xE000ED10) &= 0xFFFFFFF9;
		SystemInit();
		USB_OTG_UngateClock(&USB_OTG_dev);
	}
	EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

/**
 * @brief  This function handles EXTI15_10_IRQ Handler.
 * @param  None
 * @retval None
 */
#ifdef USE_USB_OTG_HS  
void OTG_HS_WKUP_IRQHandler(void)
{
	if(USB_OTG_dev.cfg.low_power)
	{
		*(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9;
		SystemInit();
		USB_OTG_UngateClock(&USB_OTG_dev);
	}
	EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

/**
 * @brief  This function handles OTG_HS Handler.
 * @param  None
 * @retval None
 */

#ifdef USE_USB_OTG_HS  
void OTG_HS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
#endif
#ifdef USE_USB_OTG_FS
void OTG_FS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}
#endif
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
