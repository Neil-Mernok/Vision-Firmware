/* Includes */
#include "Global_Variables.h"

/* Public Variables */
task settings;
task cc1101;
task messages_to_master;
task Tboot;

#ifdef STM32F10X_CL
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
#endif

/* Private macro */
/* Private variables */
/* Private function prototypes */

/* Private functions */

/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
int main(void)
{
	Init_103();

	SetLed(&LED1, Violet, 0);
//	GPIO_SetBits(VRFID_PORT, VRFID_PIN);			// power the eeprom

	UPDATE_Flags.F_UPDATE = 0;
	UPDATE_Flags.F_UPDATE_END = 0;
	UPDATE_Flags.Update_FileSize = 0;
	UPDATE_Flags.Update_Pointer = 0;
	UPDATE_Flags.master = NONE;

	boot_error_count = 0;
	F_FlashErase = 0;
	boot_buf_counter = 0;

	/* Unlock the Flash Bank1 Program Erase controller */
	FLASH_Unlock();

//	// if its a power on reset and everything looks ok, go to the program.
	boot = BKP_ReadBackupRegister(boot_reason_loc);

	//	Try jump to code
	jump();
	Run_Bootloader = 1;			// this is just for debugging. uncomment and comment out jump to force the bootloader.

//	// checks if app has failed or program has requested a bootload
	if (((boot & 0xFFF0) == Boot_reason_app_failed) && ((boot & 0x000F) < 3))
	{
	}
	else
	{
		// init the USB subsystem 
#ifdef USE_USB
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

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
	while (1)
	{
	}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
	__assert_func(file, line, NULL, failedexpr);
}

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
