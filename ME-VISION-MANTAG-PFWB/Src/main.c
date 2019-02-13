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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Public Variables */
task settings;
task cc1101;
task messages_to_master;
task Tboot;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
	int cant_sleep;
	
	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initialises the Flash interface and the Systick. */
	HAL_Init();
	
	/* Configure the system clock */
	SystemClock_Config();
	// init the RTC so we can read the backup registers. 
	MX_RTC_Init();
//	MX_IWDG_Init(6000);
	boot = HAL_RTCEx_BKUPRead(&hrtc, 1);
	//	Try jump to code
	jump();
	
	Config_Ports();
		
	/* Initialize all configured peripherals */
	MX_DMA_Init();
//	MX_ADC_Init();
//	MX_I2C1_Init();
//	MX_IWDG_Init(6000);
	MX_SPI1_Init();
	MX_SPI2_Init();
	/*MX_TIM3_Init();*/
	MX_TIM4_Init();
	MX_TIM7_Init();
	// 23-04-2017	Remove UART Init
	//MX_USART1_UART_Init(115200);

	LEDInit(&LED1);

	SetLed(&LED1, Red, 100);
	
	SetLed(&LED1, Off, 0);
	// run through all the state machine tasks
	GPIO_ResetBits(CHRG_EN_PORT, CHRG_EN_PIN);
	CheckAllGPIO(&cant_sleep);					// checks the status of the GPIO and LEDs

	///////////////////////////////////////////////////////////////
	// bootloader code

	UPDATE_Flags.F_UPDATE = 0;
	UPDATE_Flags.F_UPDATE_END = 0;
	UPDATE_Flags.Update_FileSize = 0;
	UPDATE_Flags.Update_Pointer = 0;
	UPDATE_Flags.master = NONE;

	boot_error_count = 0;
	F_FlashErase = 0;
	boot_buf_counter = 0;

	/* Unlock the Flash Bank1 Program Erase controller */
	HAL_FLASH_Unlock();

	Run_Bootloader = 1;			// this is just for debugging. uncomment and comment out jump to force the bootloader.

//	// checks if app has failed or program has requested a bootload
	if (((boot & 0xFFF0) == Boot_reason_app_failed) && ((boot & 0x000F) < 3))
	{
	}
	else
	{
		// init the USB subsystem 
#ifdef USE_USB
		GPIO_ResetBits(USB_VBUS_PORT, USB_VBUS_PIN);		// Tell the USB to turn on.
		USB_start();
		if(GetBoardVers())
		{
			GPIO_SetBits(USB_VBUS_PORT, USB_VBUS_PIN);		// Tell the USB to turn on.
		}	
#endif
	}

	// run through all the state machine tasks
	while (true)
	{
		//Refresh_Settings_task(&settings);			// periodically read settings.
#ifdef RF_Boot
		CC1101_Task(&cc1101);						// process RF communication
#endif

		// go process master data if received. 
		if ((time_since(UPDATE_Flags.Master_last_seen) > 5) && (UPDATE_Flags.Master_last_seen != 0))
		{
			if ((boot_buf_counter != 0) && (UPDATE_Flags.master != NONE))
			{
				Boot_packet_handler(UPDATE_Flags.master);
			}
			boot_buf_counter = 0;
		}
		Master_task(&messages_to_master);			// process communication
		boot_process(&Tboot);
	}
	return 0;
}



/** System Clock Configuration
 */
void SystemClock_Config(void)
{
	bool crystal_working;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
	
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;

	// set up the LSE first, so we can check if the crystal works. 
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
		crystal_working = true;
	else
		crystal_working = false;

	// now set up the rest of the clocks.
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	if (crystal_working == false)
		RCC_OscInitStruct.OscillatorType |= RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	if (crystal_working)
		PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	else
		PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI; // make sure we have a low power oscillator, even if the crystal fails. 

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_SYSCFG_CLK_ENABLE();
}


//void HAL_IncTick(void)
//{
//  uwTick++;
//}
//uint32_t HAL_GetTick(void)
//{
//  return uwTick;
//}

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
