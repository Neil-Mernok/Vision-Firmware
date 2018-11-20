/*
 * sleep.c
 *
 *  Created on: Jan 25, 2016
 *      Author: Kobus
 */

#include "sleep.h"
#include "as3933.h"
#include "F103_Timers.h"

int wake_flag = 0;


//
///**
// * @brief  System Power Configuration
// *         The system Power is configured as follow :
// *            + Regulator in LP mode
// *            + VREFINT OFF, with fast wakeup enabled
// *            + HSI as SysClk after Wake Up
// *            + No IWDG
// *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
// * @param  None
// * @retval None
// */
//void SystemPower_Config(void)
//{
//	FLASH_OBProgramInitTypeDef pOBInit;
//
//	__HAL_DBGMCU_FREEZE_IWDG();		// tell the system to pause IWDG when debugging and core is halted;
////	__HAL_RCC_PWR_CLK_ENABLE();
////	HAL_FLASH_Unlock();
//	/* Select the desired V(BOR) Level ---------------------------------------*/
//	HAL_FLASHEx_OBGetConfig(&pOBInit);
////	if (pOBInit.BORLevel != OB_BOR_OFF)
////	{
////		pOBInit.OptionType = OPTIONBYTE_BOR;
////		pOBInit.BORLevel = OB_BOR_OFF;
////		pOBInit.RDPLevel = OB_BOR_LEVEL1;   		// read protect the memory
////		HAL_FLASH_OB_Unlock();
////		HAL_FLASHEx_OBProgram(&pOBInit);
////
////		/* Launch the option byte loading. This will reset the processor */
////		HAL_FLASH_OB_Launch();
////	}
//
//	/* Enable Ultra low power mode */
//	HAL_PWREx_EnableUltraLowPower();
//
//	/* Enable the fast wake up from Ultra low power mode */
//	HAL_PWREx_EnableFastWakeUp();
////	HAL_PWREx_DisableFastWakeUp();
//
//// disable power voltage detector
//	HAL_PWR_DisablePVD();
//}
//
///**
// * @brief  Configures system clock before STOP: disable HSI, MSI, PLL
// * @param  None
// * @retval None
// */
//void SystemClockConfig_STOP(void)
//{
////	RCC_ClkInitTypeDef RCC_ClkInitStruct;
////	RCC_OscInitTypeDef RCC_OscInitStruct;
//
//	/* Enable Power Control clock */
////	__PWR_CLK_ENABLE();
//	/* Disable GPIOs clock */
////	__GPIOA_CLK_DISABLE();
////	__GPIOB_CLK_DISABLE();
////	__GPIOC_CLK_DISABLE();
////	__GPIOD_CLK_DISABLE();
////	__GPIOH_CLK_DISABLE();
//	/* Get the Oscillators configuration according to the internal RCC registers */
////	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
////
////	/* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 */
////	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
////	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
////	assert_param(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK);
////
//////	/* To go into stop mode, turn off HSI and PLL*/
//////	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//////	RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
//////	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//////	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
//////	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//////	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
//////	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//////	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
//////	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
//////	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
//////	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//////	assert_param(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK);
////
////	/* Reduce voltage scaling to optimise the power consumption*/
////	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//	/* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
////	while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET)
////	{
////	};
//}
//
///**
// * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
// *         and select PLL as system clock source.
// * @param  None
// * @retval None
// */
//void SystemClockConfig_WAKE(void)
//{
//	static int first = 0;
//
//	if (first == 0)
//	{
//		RCC_ClkInitTypeDef RCC_ClkInitStruct;
//		RCC_OscInitTypeDef RCC_OscInitStruct;
//
//		/* Enable Power Control clock */
//			__HAL_RCC_PWR_CLK_ENABLE();
//		/* The voltage scaling allows optimizing the power consumption when the device is 
//		 clocked below the maximum system frequency, to update the voltage scaling value 
//		 regarding system frequency refer to product datasheet.  */
//		//	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//		/* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
//		//	while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET)
//		//	{
//		//	};
//		//
//		/* Get the Oscillators configuration according to the internal RCC registers */
//		HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
//		/* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
//		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
//		////	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//		RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//		////	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
//		////	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
//		assert_param(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK);
//		/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
//		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
//		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//		assert_param(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK);
//		HAL_InitTick(0);
//		first = 1;
//	}
//	else
//	{
//		__HAL_RCC_HSI_ENABLE();
//
//		MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_HSI);
//	}
//}

void Sleep(int millis)
{
	static int millis_last = 0;
	uint32_t loopcounter = 0, sleep_total = 0, sleep_now;

	///// disable all unnecessary interrupts  
	NVIC_DisableIRQ(CC_GDO_IRQ_Channel);
	///// Disable peripherals
	GPIO_sleep_wake(1);
	///// power down ADC during sleep as this can still be enabled.
	ADC_Cmd(ADC1, DISABLE); 
	//////////////////////////////////////////////////////////////////////////////////
	// make sure we're not stopped by IWDG...
	if(millis_last != millis)
		Watchdog_reset_and_reload(millis * 2);

	/* Enter Stop Mode */
	while (sleep_total < millis)
	{
		/* Alarm after set period */
		sleep_now = MIN((millis-sleep_total), 16000);

		RTC_SetAlarm(RTC_GetCounter() + sleep_now);
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();

		IWDG_ReloadCounter();
		wake_flag = 0;
		while (wake_flag == 0)
		{
			/* Request to enter STOP mode with regulator in low power mode*/
			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

			loopcounter++;
			if (AS3933_LF_indication())				// check if LF has happened
				break;
		}
		if (AS3933_LF_indication())				// check if LF has happened
			break;
		sleep_total += sleep_now;
	}
	//////////////////////////////////////////////////////////////////////////////////
	/* Configures system clock after wake-up from STOP: enable HSI, PLL */
	SYSCLKConfig_wake();
//	SetLed(&LED1, Red, 0);
	// restore peripherals
	///// re-enable interrupts  
	NVIC_EnableIRQ(CC_GDO_IRQ_Channel);
//	HAL_UART_Init(&huart1);
	///// power up ADC again.
	ADC_Cmd(ADC1, ENABLE); 
	GPIO_sleep_wake(0);
}

