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
#include "sys_clk.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Public Variables */
task settings;
task cc1101;
task messages_to_master;
task Tboot;

/* Private function prototypes -----------------------------------------------*/

void USB_plugged(int in)
{}

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	__HAL_DBGMCU_FREEZE_IWDG();		// tell the system to pause IWDG when debugging and core is halted;
	__HAL_DBGMCU_FREEZE_RTC();		// freeze the RTC when debug core is halted.

	/* Configure the system clock */
	SystemClock_Config();
	MX_RTC_Init();
	MX_IWDG_Init(3000);

	boot = HAL_RTCEx_BKUPRead(&hrtc, 1);
		//	Try jump to code
		jump();

	/* Initialize all configured peripherals */
	Config_Ports();
	GetBoardRevision();			// get the board version to setup the right uart
	SetLed(&LED1, Red, 0);

	MX_DMA_Init();
	CAN_Config(250000);
	CAN_setup_filter(CAN_VisBootPoll_ID, 0);
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();

	USART_Configure(115200);
	
	Delay(100);
	SetLed(&LED1, Off, 0);


	// ---- Bootloader code ----

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
	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 

	Run_Bootloader = 1;			// this is just for debugging. uncomment and comment out jump to force the bootloader.

//	// checks if app has failed or program has requested a bootload
	if (((boot & 0xFFF0) == Boot_reason_app_failed) && ((boot & 0x000F) < 3))
	{
	}
	else
	{
		// ---- USB subsystem ----
#ifdef USE_USB
		MX_USB_DEVICE_Init();
#endif

	}
	debug_time = time_now();
	// run through all the state machine tasks
	while (true)
	{

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


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	while(1)
	{
		Delay(1);
	}
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
