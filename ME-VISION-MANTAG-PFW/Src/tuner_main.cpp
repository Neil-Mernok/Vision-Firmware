/**
 ******************************************************************************
 * File Name          : main.c
 * Date               : 21/03/2015 13:23:51
 * Description        : Main program body
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
#include "Global_Variables.h"

//#include <string>
//#include <map>
//#include <stdio.h>
//#include "ParamValue.h"
#include "Vision_Parameters.h"

//const char* json_param_string =
//#include "vision_settings.json"
//;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Public Variables */
task settings;
task cc1101;
task messages_to_master;
task LF_TX;
task LF_RX;
task LF_RSSI;
extern RTC_HandleTypeDef hrtc;
extern int wake_flag;
/// Container for all settings.
_vision_settings vision_settings(ME_PCB_173_02);

//task HMI;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Watchdog_reset_and_reload(uint16_t watchdog_ms);
static void SystemPower_Config(void);
static void SystemClockConfig_STOP(void);
static void SystemClockConfig_WAKE(void);
void Sleep();


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
	/* USER CODE BEGIN 1 */
	int32_t frequency;
	char freq_string[100] = {};
	uint8_t control[2] = {254, 128};
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	Config_Ports();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	/*MX_IWDG_Init();*/
	MX_SPI1_Init();
	MX_SPI2_Init();
	/*MX_TIM3_Init();*/
	MX_TIM4_Init();
	MX_TIM7_Init();
	MX_USART1_UART_Init();

	/* Configure the system Power */
	SystemPower_Config();
	LEDInit(&LED1);
	
	Vision_Status.USB_Active = false;
	SetLed(&LED1, Red, 100);
	srand(Vision_Status.UID); // seed the random generator...

	SetLed(&LED1, Off, 0);
	// run through all the state machine tasks
	GPIO_ResetBits(CHRG_EN_PORT, CHRG_EN_PIN);
	
	
	
	
	///////////////////////////////////////////////////////////////////////
	
	DelayUs(10);

	while (as3933Initialize(125000) < 0)
	{
		Delay(10);
		SetLed(&LED1, Red, 0);
	}

		
	//////////////////////////////////////////////////////////////////////
	while (true)
	{
//		HAL_UART_Transmit(&huart1, control, 2, 50); 
		as3933Initialize(125000) ;
		frequency = as3933GetFrequency(0);
		sprintf(freq_string, "freq: %6d   \n", frequency);
		HAL_UART_Transmit(&huart1, (uint8_t*)freq_string, 16,50);
		Delay(150);
	}
}

/**
 * @brief  RTC Wake Up callback
 * @param  None
 * @retval None
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	/* Clear Wake Up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	wake_flag = 1;
//	SetLed(&LED1, Red, 200);
//	SetGPO(&LED_OUT, 200);
}

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
static void SystemPower_Config(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure =
//	{ 0 };

	FLASH_OBProgramInitTypeDef pOBInit;

#ifdef DEBUG
	HAL_DisableDBGStopMode();
	HAL_DisableDBGSleepMode();
	HAL_DisableDBGStandbyMode();
#else
	HAL_DisableDBGStopMode();
	HAL_DisableDBGSleepMode();
	HAL_DisableDBGStandbyMode();
#endif

	/* Select the desired V(BOR) Level ---------------------------------------*/
	HAL_FLASHEx_OBGetConfig(&pOBInit);
	if (pOBInit.BORLevel != OB_BOR_OFF)
	{
		pOBInit.OptionType = OPTIONBYTE_BOR;
		pOBInit.BORLevel = OB_BOR_OFF;
		HAL_FLASH_OB_Unlock();
		HAL_FLASHEx_OBProgram(&pOBInit);

		/* Launch the option byte loading. This will reset the processor */
		HAL_FLASH_OB_Launch();
	}

	/* Enable Ultra low power mode */
	HAL_PWREx_EnableUltraLowPower();

	/* Enable the fast wake up from Ultra low power mode */
//	HAL_PWREx_EnableFastWakeUp();
	HAL_PWREx_DisableFastWakeUp();

	// disable power voltage detector
	HAL_PWR_DisablePVD();

	/* Enable GPIOs clock */
//	__GPIOA_CLK_ENABLE();
//	__GPIOB_CLK_ENABLE();
//	__GPIOC_CLK_ENABLE();
//	__GPIOD_CLK_ENABLE();
//	__GPIOH_CLK_ENABLE();
//
//	/* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
//	GPIO_InitStructure.Pin = GPIO_PIN_All;
//	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStructure.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
//	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
//
//	/* Disable GPIOs clock */
//	__GPIOA_CLK_DISABLE();
//	__GPIOB_CLK_DISABLE();
//	__GPIOC_CLK_DISABLE();
//	__GPIOD_CLK_DISABLE();
//	__GPIOH_CLK_DISABLE();
}

/**
 * @brief  Configures system clock before STOP: disable HSI, MSI, PLL
 * @param  None
 * @retval None
 */
static void SystemClockConfig_STOP(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE()
	;

	/* Disable GPIOs clock */
//	__GPIOA_CLK_DISABLE();
//	__GPIOB_CLK_DISABLE();
//	__GPIOC_CLK_DISABLE();
//	__GPIOD_CLK_DISABLE();
//	__GPIOH_CLK_DISABLE();
	/* Get the Oscillators configuration according to the internal RCC registers */
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	/* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	assert_param(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK);

//	/* To go into stop mode, turn off HSI and PLL*/
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//	RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
//	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
//	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
//	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//	assert_param(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK);

	/* Reduce voltage scaling to optimise the power consumption*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
//	while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET)
//	{
//	};
}

/**
 * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
 *         and select PLL as system clock source.
 * @param  None
 * @retval None
 */
static void SystemClockConfig_WAKE(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE()
	;
	/* Enable GPIOs clock */
	__GPIOA_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;
	__GPIOC_CLK_ENABLE()
	;
	__GPIOD_CLK_ENABLE()
	;
//	__GPIOH_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
	while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET)
	{
	};

	/* Get the Oscillators configuration according to the internal RCC registers */
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	/* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	assert_param(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	assert_param(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK);

}

///** System Clock Configuration
// */
//void SystemClock_Config(void)
//{
//
//	RCC_OscInitTypeDef RCC_OscInitStruct;
//	RCC_ClkInitTypeDef RCC_ClkInitStruct;
//	RCC_PeriphCLKInitTypeDef PeriphClkInit;
//
//	__PWR_CLK_ENABLE();
//
//	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
//
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; // RCC_OSCILLATORTYPE_LSE;
////	RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
//	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//	RCC_OscInitStruct.HSICalibrationValue = 16;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
//
//	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
////	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
//	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
//
//	__SYSCFG_CLK_ENABLE();
//}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	__PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
//	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
//	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__SYSCFG_CLK_ENABLE()
	;

}

//void Sleep()
//{
//	///// disable all unnecessary interrupts  
//	HAL_NVIC_DisableIRQ(CC_GDO_IRQ_Channel);
//	HAL_NVIC_DisableIRQ(CC_GDO_IRQ_Channel);
//
//	///// Disaplbe peripherals
//
//	
//	HAL_PWREx_DisableFastWakeUp();
//	HAL_PWR_DisablePVD();
//	HAL_PWREx_EnableUltraLowPower();
//
//	// Disable FLASH during SLeep  
//	//FLASH_SLEEPPowerDownCmd(ENABLE);
//
//	
//	///// Set clocks off
//	SystemClockConfig_STOP();
//		
//	///// stop the systic ISR		
//	//	HAL_SuspendTick();
//		SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
//		
//		
//	//	HAL_RTCEx_DeactivateWakeUpTimer (&hrtc);
//	//	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x2616, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
//		
//	//////////////////////////////////////////////////////////////////////////////////
//	/* Enter Stop Mode */
//	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//	//////////////////////////////////////////////////////////////////////////////////
//	///// start the systic ISR		
////	HAL_ResumeTick();
//	/* Configures system clock after wake-up from STOP: enable HSI, PLL */
//	//	SetLed(&LED1, Red, 0);
//	SystemClockConfig_WAKE();
//	SysTick->CTRL |= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
//}

void Sleep()
{
	uint32_t loopcounter = 0;

	///// disable all unnecessary interrupts  
	HAL_NVIC_DisableIRQ (CC_GDO_IRQ_Channel);

	///// Disable peripherals

	///// configure clocks for low power
	SystemClockConfig_STOP();

	///// stop the systic ISR		
//	HAL_SuspendTick();

	//////////////////////////////////////////////////////////////////////////////////
	/* Enter Stop Mode */
	wake_flag = 0;
	while (wake_flag == 0)
	{
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		loopcounter++;
	}
	//////////////////////////////////////////////////////////////////////////////////
	///// start the systic ISR		
//	HAL_ResumeTick();
	/* Configures system clock after wake-up from STOP: enable HSI, PLL */
	//	SetLed(&LED1, Red, 0);
	SystemClockConfig_WAKE();
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif




/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
