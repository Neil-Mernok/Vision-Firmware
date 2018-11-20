/*
 * boot_Tasks.c
 *
 *  Created on: Mar 18, 2015
 *      Author: KobusGoosen
 */

#include "boot_Tasks.h"

/// need for the CC1101 initialisation steps.
#include "my_rf_settings.h"
#include "framing.h"

void jump(void);

uint16_t boot;
uint16_t boot_error_count;
uint8_t boot_data[BUFFER_SIZE];
uint16_t boot_buf_counter;



/**
 * @ the purpose of this task is to wait for messages from ther master and answer them when ready. 
 * @ it can also accept messages from the code intended for the master and forward them along. 
 * @param pvParameters
 */
void Master_task(task* t)
{
	static _Q_MasterIF MIF;

	if (t->state == 0)			// this is run the first time only. 
	{
		// create a pipe to hold messages we need to forward. max 50 messages 
		pipe_init(&(t->p), 40, sizeof(_Q_MasterIF));
		t->state = 1;						// dont run this again. 
	}

	if (MIF.len != 0)		// no finished with current message, try sending again. 
	{
		/// try sending some more data 
		Send_to_Master(&MIF);
	}
	else
	{
		/// try to get a new packet for next time. 
		if (pipe_get(&(t->p), &MIF))
		{
			/// there's data available. 
			Send_to_Master(&MIF);
		}
	}
}


/**
 * @brief The boot state machine exists in 2 states (once we have established that we can't jump to code, or we need to be in bootloader)
 * 1: wait for command. either run application, run bootloader or flash erase
 * 2: loop until the memory has been flashed
 */

enum boot_state
{
	flash_wait, flash_erase, flash_write,
};

void Send_short_message(uint8_t message)
{
	_Q_MasterIF MIF; 

	MIF.len = 1;
	MIF.data = &message;
	MIF.Master = CODE;
	push_to_master(MIF);
}

void boot_process(task* t)
{
	static uint8_t colour;

	if (t->state == flash_wait)					// the default on first entry. wait for bootload command
	{
		// wait until one of the flags gets set
		if ((Run_Bootloader == 1) && (F_FlashErase == 0))
		{
			// try to jump again if nothing happens in 5 minutes.
			if (Run_Application == 1 || time_now() > 300000)
			{
				boot = Boot_reason_none;
				boot_error_count = 0;
				Delay(100);
				jump();
			}

			/////////////////////////////////////////////////
			if ((boot & 0xFFF0) == Boot_reason_app_failed)				// check if the boot reason is failure...
			{
				boot_error_count = (boot & 0x000F) + 1;
				if (boot_error_count < 4)
				{
					boot_error_count++;
					boot = Boot_reason_none;
					Delay(100);
					jump();
				}
			}
			/////////////////////////////////////////////////
			
			Delay(10);
			colour++;
			SetLed(&LED1, (RGB_State_TypeDef) colour, 0);
#ifdef USE_HAL_DRIVER
			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
#else
			IWDG_ReloadCounter();
#endif
		}
		else
			t->state = flash_erase;
	}
	else if (t->state == flash_erase) /* state == 1. we are erasing the flash*/
	{
		/****************************************************************/
		//Check if erase flag is triggered then erase all relevant flash
		/****************************************************************/
		if (F_FlashErase == 1)
		{
#ifdef USE_HAL_DRIVER	
//			uint32_t err, Pages = UPDATE_Flags.Update_FileSize /FLASH_PAGE_SIZE; //
//			FLASH_EraseInitTypeDef eraseinit = {0, APPLICATION_ADDRESS, Pages};
						
			SetLed(&LED1, Yellow, 0);
			
//			if (HAL_FLASHEx_Erase(&eraseinit, &err) == HAL_OK)
//			{
				//Send ACK if passed
				Send_short_message(6);
				F_FlashErase = 0;
				t->state = flash_write;
				// Debug 10/10/2017
				//SetLed(&LED1, Blue, 0);
//			}
//			else
//			{
//				NVIC_SystemReset();
//			}
#else
//			uint32_t i = 0;
//			FLASH_Status S = FLASH_ERROR_PG;
//			/* Clear All pending flags */
//			FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
//
//			SetLed(&LED1, Yellow, 0);
//			/* Erase the FLASH pages */
//			i = APPLICATION_ADDRESS;
//			while (i <= APPLICATION_ADDRESS + UPDATE_Flags.Update_FileSize)
//			{
//				S = FLASH_ErasePage(i);
//				i += FLASH_PAGE_SIZE;
//				IWDG_ReloadCounter();
//			}
//
//			if (S != FLASH_COMPLETE)
//			{
//				NVIC_SystemReset();
//			}
//			else
//			{
				//Send ACK if passed
				Send_short_message(6);
				F_FlashErase = 0;
				t->state = flash_write;
				SetLed(&LED1, Blue, 0);
//			}
#endif
		}
	}
	else if (t->state == flash_write) /* state == 1. we are busy writing in the data*/
	{
		if (( time_now() > UPDATE_Flags.F_UPDATE_END) && (UPDATE_Flags.F_UPDATE_END != 0))
		{
#ifdef USE_HAL_DRIVER
			HAL_RTCEx_BKUPWrite(&hrtc, 1, Boot_reason_none);
#else
			BKP_WriteBackupRegister(boot_reason_loc, Boot_reason_none);
#endif
						
			SetLed(&LED1, Blue, 0);
			NVIC_SystemReset();
		}
	}
}



/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
pFunction Jump_To_Application;
uint32_t JumpAddress;
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

///// code used to jump to the code location
void jump(void)
{
	if ((boot == Boot_reason_none) && (((*(__IO uint32_t*) APPLICATION_ADDRESS ) & 0x2FFE0000) == 0x20000000) && (Firmware_rev < 255)) //&& (Application_rev > 0)
	{
		//
		///  this is where the firmware image gets run. the chip determines if everything is ok,
		///  then it sets the message RAM to indicate that the firmware failed, resets the watchdog and jumps to code.
		///  firmware must then reset IWD and set message ram to 0 to indicate successful startup.

#ifdef USE_HAL_DRIVER
		HAL_FLASH_Lock();					// lock flash before exiting the boot-loader.
		HAL_RTCEx_BKUPWrite(&hrtc, 1, Boot_reason_app_failed | boot_error_count);
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
#else
		FLASH_Lock();						// lock flash before exiting the boot-loader.
		BKP_WriteBackupRegister(boot_reason_loc, Boot_reason_app_failed | boot_error_count);
		IWDG_ReloadCounter();
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetReload(30);// shorten the IWDG reload time so faulty app fails quiclky 
		IWDG_ReloadCounter();
#endif

		SetLed(&LED1, White, 0);
		//Jump to user application
		JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		//Initialize user application's Stack Pointer
		__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
		Jump_To_Application();
	
	}
	else
		Run_Bootloader = 1;
}

/**
 * @brief: this task starts the USB
 */
void USB_start(void)
{

#ifdef STM32F10X_HD
	USB_Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
#endif
#ifdef STM32F10X_CL
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
#endif
}

// define the states the RF state machine can adopt
#define CC_RX 2
#define CC_TX 3
#define CC_Idle 4

/**
 * @brief: this task controls the CC1101 chip. 
 * @param pvParameters
 */
#ifdef RF_Boot
uint8_t Boot_counter = 0 ;
void CC1101_Task(task* t)
{
	int timeout;
	uint8_t PayloadLength = 0, RSSI = 0, status = 0, rev;
//	_Q_MasterIF MIF;
	static uint8_t data[64];

	if (t->state == 0)			// this is run the first time only. 
	{
//		// create a pipe to hold IDs we need to range to. 
//		pipe_init(&(t->p), 5, sizeof(MIF));

		halRfResetChip();

		rev = halRfGetChipVer();
		if ((rev != 4) && (rev != 20))
		{
			t->state = -1;
			return;
		}

		/// added to indicate Healthy SPI to CC chip.
		Config_EXTI_GD0();

		halRfStrobe(CC1100_SPWD);			// power down. 

		// write RF settings to chip
		myPaTable[0] = 0xC0;				//10dB		PAsettings[PA_setting];
		myRfConfig_250.channr = 3;			// use a different frequency channel for Bootload purposes.
		halRfConfig(&myRfConfig_250, myPaTable, myPaTableLen);

		t->state = CC_Idle;
	}
	else if (t->state == -1)
		return;

	/////////////////////////////////////////////////////////////////////////////////////////////
	//// this is where the processing happens
	/////////////////////////////////////////////////////////////////////////////////////////////

	if (t->state == CC_Idle)			// cc chip is idle, so do something with it. 
	{
		timeout = 10000;

		status = rxRecvReentrant(data, &PayloadLength, &RSSI, timeout);

		if (PayloadLength > 1)
			time_now();

		if (status == 0)		// packet received, no timeout
		{
			// ---- Check if packet received is valid ----
			if (check_RF_frame_crc(data, (PayloadLength - 4)))
			{
				if(data[2] == 'B' || data[2] == 'E')			// bootloader or bootend message
				{
					SetLed(&LED1, Blue, 0);
					memcpy(&boot_data[boot_buf_counter], data + 3, PayloadLength - 5);

					if(data[2] == 'E')		// packet was signalled as last/only.
						UPDATE_Flags.Master_last_seen = time_now() - 6;
					else
					{
						UPDATE_Flags.Master_last_seen = 0;
					}
				
					boot_buf_counter += PayloadLength - 5;

					UPDATE_Flags.master = RF;
			
					SetLed(&LED1, Off, 0);
				}
			}
			t->state = CC_Idle;
		}
	}

//	if (time_since(debug_time) > 1000)
//	{
//		Send_short_message(7);
//		debug_time = time_now();
//	}
}
#endif
