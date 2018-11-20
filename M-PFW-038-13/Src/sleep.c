/*
 * sleep.c
 *
 *  Created on: Jun 4, 2015
 *      Author: Kobus
 */

#include "sleep.h"
#include "as3933.h"
#include "Vision_Parameters.h"
extern int wake_count;


void GPIO_AnalogState_Config(void);

/**
 * @brief  System Power Configuration
 *         The system Power is configured as follow :
 *            + Regulator in LP mode
 *            + VREFINT OFF, with fast wakeup enabled
 *            + HSI as SysClk after Wake Up
 *            + No IWDG
 *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
 * @param  None
 * @retval None
 */
void SystemPower_Config(void)
{
//	FLASH_OBProgramInitTypeDef pOBInit;

	__HAL_DBGMCU_FREEZE_IWDG();		// tell the system to pause IWDG when debugging and core is halted;
	__HAL_DBGMCU_FREEZE_RTC();		// freeze the RTC when debug core is halted. 
	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	/* Ensure that MSI is wake-up system clock */
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

	// disable power voltage detector
	HAL_PWR_DisablePVD();
	// TODO: check for other power saving tips. 
}

/**
 * @brief  Configures system clock before STOP: disable HSI, MSI, PLL
 * @param  None
 * @retval None
 */
void SystemClockConfig_STOP(void)
{

}

/**
 * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
 *         and select PLL as system clock source.
 * @param  None
 * @retval None
 */
void SystemClockConfig_WAKE(void)
{

}

void Sleep()
{
	uint32_t loopcounter = 0;
	static uint32_t last_iwdg = 0;

	///// disable all unnecessary interrupts  
	HAL_NVIC_DisableIRQ(CC_GDO_IRQ_Channel);

	GPIO_sleep_wake(1);
	
	/*Enable wake on COMS interrupts */
	// enable UART RX wake on receive pin. 
	__HAL_GPIO_EXTI_CLEAR_IT(RX_Pin);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	uint32_t time = HAL_RTCEx_GetWakeUpTimer(&hrtc);
	if(time != last_iwdg)
	{
		last_iwdg = time;
		MX_IWDG_Init(time);			// this technically sets the IWDG to twice the wake time. (65536/8 = 4096*2) 
	}
	///// configure clocks for low power
	SystemClockConfig_STOP();
	
	//////////////////////////////////////////////////////////////////////////////////
	/* Enter Stop Mode1 */
	uint32_t start = wake_count;
	while(start == wake_count)
	{
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		loopcounter++;
		if (AS3933_LF_indication())				// check if LF has happened
			break;
		if (time_since(Vision_Status.last_master_coms) < 2000)
			break;								// TODO: clean this up somehow. 
	}
	//////////////////////////////////////////////////////////////////////////////////
	///// start the systic ISR		
	/* Configures system clock after wake-up from STOP: enable HSI, PLL */
	SystemClockConfig_WAKE();
	// restore peripherals
	///// re-enable interrupts  
	HAL_NVIC_EnableIRQ(CC_GDO_IRQ_Channel);

	/*Disable COMS pins to interrupts */
//		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	
	GPIO_sleep_wake(0);
}

