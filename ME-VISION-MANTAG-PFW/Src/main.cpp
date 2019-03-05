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
#include "Vision_Parameters.h"
#include "Tasks.h"
#include "sleep.h"

/* Private variables ---------------------------------------------------------*/

/* Public Variables */
task settings;
task status;
task cc1101;
task messages_to_master;
task LF_TX;
task LF_RX;
task LF_RSSI;
task Distressd;
task TimeSend;
task uBlox;
extern uint32_t uwTick;
extern int wake_flag;
extern int wake_count;

/// Container for all settings.
_vision_settings vision_settings(ME_PCB_173_03);
//_vision_settings vision_settings(vision_reader);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


#include "uBlox_Config.h"

int main(void)
{

	uint32_t loopcounter = 0;
	int cant_sleep;
	/* MCU Configuration----------------------------------------------------------*/
	Config_Ports();
	SetLed(&LED1, Off, 0);

	/* Reset of all peripherals, Initialises the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

//	// ---- V12 GPS functionality ----
//	GPIO_SetBits(uBlox_RST_PORT, uBlox_RST_PIN); // Pin is active low

	if(Vision_Status.sts.crystal_working == 0)
	{
		Delay(10);
		SystemClock_Config();
	}

	int bd = GetBoardVers();
	if (bd == 3)
		Vision_Status.board_id = ME_PCB_203_02;
	else if(bd == 2)
		Vision_Status.board_id = ME_PCB_203_01;
	else if(bd == 1)
		Vision_Status.board_id = ME_PCB_173_04;
	else
		Vision_Status.board_id = ME_PCB_173_03;

	Vision_Status.sts.USB_Active = false;
	Vision_Status.exclusion = 1000;							// keep the tags silent for the first 2 seconds. prevents reset anomalies when excluded.

	Delay(1);

	/* Initialize all configured peripherals */
	MX_DMA_Init();
	MX_ADC_Init();
//	MX_I2C1_Init();
	MX_RTC_Init();
	MX_IWDG_Init(3000);
	MX_SPI1_Init();
	MX_SPI2_Init();
	/*MX_TIM3_Init();*/
	MX_TIM4_Init();
	MX_TIM7_Init();
	MX_USART1_UART_Init(9600);

	/* Configure the system Power */
	//SystemPower_Config();
	LEDInit(&LED1);

	SetLed(&LED1, Red, 10);
	srand(Vision_Status.UID); // seed the random generator...

	SetLed(&LED1, Off, 0);
	// run through all the state machine tasks
	GPIO_ResetBits(CHRG_EN_PORT, CHRG_EN_PIN);
	CheckAllGPIO(&cant_sleep);					// checks the status of the GPIO and LEDs

	while (true)
	{
		cant_sleep = 0;								// start off assuming we can sleep. then ask each task if they need to stay awake.
		Refresh_Settings_task(&settings);			// periodically read settings.
		DetermineTagType();

		CC1101_Task(&cc1101, &wake_flag, &cant_sleep);	// process RF communication
		Master_task(&messages_to_master, &cant_sleep);	// process communication
		GetStatus(&status);			// run the status update task.



#ifdef LF_TX_Capable
		LF_TX_Task(&LF_TX);							// periodically send a LF pulse
#endif
// FIXME : LF_RX_Capable #endif
#ifdef LF_RX_Capable
		LF_Task(&LF_RX, &cant_sleep);

#ifdef GPS_CAPABLE

		uBlox_task(&uBlox, &cant_sleep);

#endif
		if (vision_settings.getActivities().tag_enable)
		{
			LF_RSSI_watcher(&LF_RSSI, wake_count, &cant_sleep);			// Check LF RSSI and set LEDs/outputs
#endif
#ifdef use_HMI
			Test_task5(&HMI); //LCD
#endif
			loopcounter++;

			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
			if (AS3933_LF_indication())
				cant_sleep++;			// check if LF has happened, if so prevent sleep.

			CheckAllGPIO(&cant_sleep);					// checks the status of the GPIO and LEDs

			if (HAL_GPIO_ReadPin(LAMP_OUT_PORT, LAMP_OUT_PIN) == 0)
				cant_sleep++;			// check if LF has happened, if so prevent sleep.

			if (vision_settings.getActivities().Always_on)
				cant_sleep++;			// prevent sleep if we need to stay alive.

			Distress_watcher(&Distressd, &cant_sleep);
			TimeBroadcast(&TimeSend, &cant_sleep);
		}

//		/* Enter Stop Mode */
//		if (cant_sleep == 0)
//		{
//			Vision_Status.sts.Sleeping = 1;
//			Sleep();
//			Vision_Status.sts.Sleeping = 0;
//		}
	}
}

/**
 * @brief  RTC Wake Up callback
 * @param  None
 * @retval None
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	static uint32_t RTC_tick = 0, last_div = 0;

	/* Clear Wake Up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	
	uint32_t time = HAL_RTCEx_GetWakeUpTimer(hrtc) / 2;
	RTC_tick += time;
	if(RTC_tick/(uint32_t)vision_settings.interval != last_div)
		wake_flag = 1;
	last_div = RTC_tick/(uint32_t)vision_settings.interval;
	
	if (AS3933_LF_indication() == 0)
		uwTick = MAX(RTC_tick, uwTick);

	HAL_GetTick();
	wake_count++;
}

extern uint16_t ADC_res[3];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static const float vref_expected = 1.22;	// this is the expected value of the internal reference. used to correct values read in case VCCA is not 3.3V  
	float Vref = ADC_res[2] / 4096.0 * 3.3;
	float Vbatf = (uint32_t) ADC_res[1] * 143.0 / 43.0 / 4096.0 * 3.3;
	float Vsysf = (uint32_t) ADC_res[0] * 143.0 / 43.0 / 4096.0 * 3.3;

	Vbatf *= vref_expected / Vref;
	Vsysf *= vref_expected / Vref;

	Vision_Status.Vsys = Vsysf * 1000.0;
	Vision_Status.Vbat = Vbatf * 1000.0;

	Vision_Status.sts.EXT_Power = (Vision_Status.Vsys > 4000) ? 1:0;

	if(Vision_Status.sts.Charging)
		Vsysf += 0.25;				// account for the diode voltage drop when charging. Also adds some hysteresis. 
	
	if (Vsysf > ((int) vision_settings.vchrgmin / 1000.0))
		GPIO_ResetBits(CHRG_EN_PORT, CHRG_EN_PIN);
	else if (Vbatf < 3.3)
		GPIO_ResetBits(CHRG_EN_PORT, CHRG_EN_PIN);
	else
		GPIO_SetBits(CHRG_EN_PORT, CHRG_EN_PIN);
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

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
		Vision_Status.sts.crystal_working = true;
	else
		Vision_Status.sts.crystal_working = false;

	// now set up the rest of the clocks.
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	if (Vision_Status.sts.crystal_working == false)
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
	if (Vision_Status.sts.crystal_working)
		PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	else
		PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI; // make sure we have a low power oscillator, even if the crystal fails. 

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_SYSCFG_CLK_ENABLE();
}


void HAL_IncTick(void)
{
  uwTick++;
}

uint32_t HAL_GetTick(void)
{
		return uwTick;
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
