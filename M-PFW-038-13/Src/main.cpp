/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include "Global_Variables.h"
#include "Vision_Parameters.h"
#include "Tasks.h"
#include "sys_clk.h"
#include "sleep.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
task settings;
task status;
task cc1101;
task messages_to_master;
task LF_TX;
task LF_RX;
task LF_RSSI;
task nanotron;
task uBlox;

extern uint32_t uwTick;
int wake_flag;
int wake_count;

/// Container for all settings.
//_vision_settings vision_settings(vision_reader);
_vision_settings vision_settings(ME_PCB_217_01);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
bool usb_starter(void);
bool USB_started = false;
	
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
	int cant_sleep, loopcounter = 0;

	/* MCU Configuration----------------------------------------------------------*/
	Config_Ports();
	setup_coms_flag(&Vision_Status.last_master_coms);
	SetLed(&LED1, Off, 0);

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	Vision_Status.sts.USB_Active = false;					// Set status bit to indicate that the board's USB is inactive (on reset)
	Vision_Status.sts.crystal_working = true;				// Set status bit to indicate that the crystal is working
	Vision_Status.sts.USB_capable = true;					// Set status bit to indicate that the board is USB communication capable
	Vision_Status.sts.CAN_capable = true;					// Set status bit to indicate that the board is CAN communication capable
	Vision_Status.sts.UART_capable = true;					// Set status bit to indicate that the board is UART communication capable
	Vision_Status.exclusion = 2000;							// Keep the tags silent for the first 2 seconds, prevents reset anomalies when excluded (Pulse400)

#ifdef 	GPS_MODULE
	Vision_Status.kind = GPS;								// ---- Firmware kind GPS (hard-coded) ----

	switch (GetBoardRevision())
	{
		case 1:
			Vision_Status.board_id = ME_PCB_217_02;			// ---- Hardware Revision 2 ----
			break;
		default:
			Vision_Status.board_id = ME_PCB_217_01;			// ---- Hardware Revision 1 ----
			break;
	}
#elif 	RANGER_MODULE
	Vision_Status.kind = Ranging;							// ---- Firmware kind Ranger (hard-coded) ----
#else
	Vision_Status.kind = get_TAG_kind();					// ---- Firmware kind Pulse/Pulse_GPS (hard-coded) ----

	switch (GetBoardRevision())
	{
		case 2:
			Vision_Status.board_id = ME_PCB_182_06;				// PULSE module revision 6
			break;
		case 1:
			Vision_Status.board_id = ME_PCB_182_04;				// PULSE module revision 4 or 5
			break;
		default:
			Vision_Status.board_id = ME_PCB_182_03;				// PULSE module revision 3
			break;
	}
#endif

	// TODO: Reset to Vision_Status.kind not Vision_Status.board_id
	vision_settings.set_defaults((boards)(Vision_Status.board_id));

	/* Initialize all configured peripherals */
	MX_DMA_Init();
	MX_ADC1_Init();

	CAN_Config(250000);
	MX_I2C1_Init(400000);
	MX_SPI2_Init();

	MX_IWDG_Init(32000);
	MX_RTC_Init();

	MX_TIM2_Init();
	MX_TIM3_Init(125000);
	MX_TIM4_Init();
	MX_TIM7_Init();

	USART_Configure(115200);
	GPSModule_USART_Configure(9600);

	/* Configure the system Power */
	SystemPower_Config();

	SetLed(&LED1, Red, 100);

	Delay(1);
	srand(465877); 											// ---- Seed the random generator ----
	SetLed(&LED1, Off, 0);

	CheckAllGPIO(&cant_sleep);								// ---- Checks the status of the GPIO and LEDs ----

	/* Infinite loop */
	while (1)
	{
		cant_sleep = 0;										// ---- Start off assuming we can sleep, then ask each task if they need to stay awake ----

		Refresh_Settings_task(&settings);					// ---- Periodically read settings ----
		Vision_Status.Group_status  = vision_settings.MernokAsset_Groups[(uint8_t)vision_settings.tag_type-1];
#ifdef GPS_MODULE
		///	GPS100 Module code	///////////////////////////////
		uBlox_task(&uBlox, &cant_sleep);					// ---- Process GPS Module Tasks ----
		///////////////////////////////////////////////////////
#elif RANGER_MODULE
		/// RANGER Module code ///////////////////////////////
		if (Vision_Status.board_id == ME_PCB_138_03)
		{
			Nanotron_task(&nanotron, &cant_sleep);
		}
		///////////////////////////////////////////////////////
#else
		/// Pulse400 Module code ///////////////////////////////
		if ((Vision_Status.board_id == ME_PCB_182_03) || (Vision_Status.board_id == ME_PCB_182_04) || (Vision_Status.board_id == ME_PCB_182_06))
		{
			CC1101_Task(&cc1101, &wake_flag, &cant_sleep);	// ---- Process RF communication ----
		}
		///////////////////////////////////////////////////////
#endif

		Master_task(&messages_to_master, &cant_sleep);		// ---- Process communication ----

		GetStatus(&status);									// ---- Run the status update task ----

#ifdef LF_TX_Capable
		if((Vision_Status.board_id != ME_PCB_138_03) && (Vision_Status.board_id != ME_PCB_217_01) && (Vision_Status.board_id != ME_PCB_217_02))
			LF_TX_Task(&LF_TX);								// ---- Periodically send a LF Pulse ----
#endif

#ifdef LF_RX_Capable
		if(Vision_Status.board_id != ME_PCB_138_03)
			LF_Task(&LF_RX, &cant_sleep);
#endif
		if (vision_settings.getActivities().tag_enable)
		{

#ifdef LF_RX_Capable
			if(Vision_Status.board_id != ME_PCB_138_03)
				LF_RSSI_watcher(&LF_RSSI, wake_count, &cant_sleep);			// Check LF RSSI and set LEDs/outputs

			if (AS3933_LF_indication())
				cant_sleep++;							// ---- Check if LF has happened, if so prevent sleep.
#endif

#ifdef use_HMI
			Test_task5(&HMI); 							// ---- LCD Output ----
#endif
			loopcounter++;

			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);			// ---- Reload the watchdog timer ----

			CheckAllGPIO(&cant_sleep);					// ---- Checks the status of the GPIO and LEDs ----

			if (vision_settings.getActivities().Always_on)
				cant_sleep++;							// ---- Prevent sleep if needed to stay alive ----
			if (Vision_Status.last_master_coms != 0 && time_since(Vision_Status.last_master_coms) < 10000)
				cant_sleep++;							// ---- Prevent for 30s after master communication ----
			if (Vision_Status.Vbat > 4350)
				cant_sleep++;							// ---- Crude USB sleep prevention ----
			if (Vision_Status.sts.EXT_Power)
				cant_sleep++;							// ---- Prevent sleep when there is CAN bus power ----
			usb_starter();
		}

		/* Enter Stop Mode */
		if (cant_sleep == 0 && Vision_Status.board_id != ME_PCB_138_03)		// ---- Disable sleep for RANGER ----
		{
			Vision_Status.sts.Sleeping = 1;
			Sleep();
			Vision_Status.sts.Sleeping = 0;
		}
	}
}

/**
 * @brief  Check if/when we can start the USB peripheral. 
 * This check is necessary as the USB peripheral draws current after setup, and can't be shut down successfully to limit current. 
 * @param  None
 * @retval None
 */
bool usb_starter(void)
{
	static uint32_t last_checked = 0;

	// no sense running this function after the USB has been started. 
	if(USB_started) return USB_started;
			
	if(time_since(last_checked) > 15)
	{
		last_checked = time_now();
		
		if ((Vision_Status.Vbat > 4350) || (vision_settings.getActivities().receive_RF) || (vision_settings.getActivities().Always_on))
		{
			MX_USB_DEVICE_Init();
			USB_started = true;
			Vision_Status.sts.USB_Active = true;
		}
	}
	return USB_started;
}

/**
 * @brief  RTC Wake Up callback
 * @param  None
 * @retval None
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	static uint32_t  RTC_tick = 0, last_div = 0;//last_tick = 0,

	/* Clear Wake Up Flag */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	
	uint32_t time = HAL_RTCEx_GetWakeUpTimer(hrtc) / 2;
	RTC_tick += time;
	if(RTC_tick/(uint32_t)vision_settings.interval != last_div)
		wake_flag = 1;
	last_div = RTC_tick/(uint32_t)vision_settings.interval;
	
	if (AS3933_LF_indication() == 0)
		uwTick = MAX(RTC_tick, uwTick);

	//last_tick =
			HAL_GetTick();
	wake_count++;
}



extern uint16_t ADC_res[3];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static const float vref_expected = 1.20;	// this is the expected value of the internal reference. used to correct values read in case VCCA is not 3.3V  
	float Vref = ADC_res[2] / 4096.0 * 3.3;
	float Vbatf = (uint32_t) ADC_res[1] * 2 / 4096.0 * 3.3;
	float Vsysf = (uint32_t) ADC_res[0] * 1100.0 / 100.0 / 4096.0 * 3.3;

	Vbatf *= vref_expected / Vref;
	Vsysf *= vref_expected / Vref;

	Vision_Status.Vsys = Vsysf * 1000.0;
	Vision_Status.Vbat = Vbatf * 1000.0;

	Vision_Status.sts.EXT_Power = (Vision_Status.Vsys > 4000) ? 1 : 0;
}

/**
 * Handle the systic event. this is used to time the LF-TX pulsing operations. 
 */
void HAL_SYSTICK_Callback(void)
{
#ifdef LF_TX_Capable
	LF_send_bits();
#endif
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
	while(1)
	{
		Delay(1);
	}
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
