/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO/STM32
 **                STMicroelectronics STM32F10x Standard Peripherals Library
 **
 **  Distribution: The file is distributed “as is,” without any warranty
 **                of any kind.
 **
 **  (c)Copyright Atollic AB.
 **  You may use this file as-is or modify it according to the needs of your
 **  project. Distribution of this file (unmodified or modified) is not
 **  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
 **  rights to distribute the assembled, compiled & linked contents of this
 **  file as part of an application binary file, provided that it is built
 **  using the Atollic TrueSTUDIO(R) toolchain.
 **
 **
 *****************************************************************************
 */

/* Includes */
#include "Global_Variables.h"
#include "Vision_Parameters.h"
#include "Tasks.h"
#include "sleep.h"

//#include "VisionMaster.h"


/* Public Variables */
task settings;
task status;
task cc1101;
task messages_to_master;
task LF_TX;
task LF_RX;
task LF_RSSI;
task HMI;
//int wake_flag = 0;
int wake_count = 0;
extern uint32_t systic;
uint32_t last_UID;

#ifdef VISION_TEST_MASTER
vision_device me;
#endif

/// Container for all settings.
_vision_settings vision_settings(vision_reader);
//_vision_settings vision_settings(pulse300);
//_vision_settings vision_settings(pulse500);

/* Private macro */
/* Private variables */
/* Private function prototypes */
void error_handler(int reason);

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
	uint32_t loopcounter = 0;
	int cant_sleep = 1;
	/* Ensure all priority bits are assigned as preemption priority bits. http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	__enable_irq();
	
	if(Init_board())
		Vision_Status.sts.crystal_working = true;
	else
		Vision_Status.sts.crystal_working = false;

	// init the USB subsystem. This needs to happen before PA9 goes high to make sure the USB starts up quickly 
	USB_start();
	
	if(GetBoardVers())
	{
		Vision_Status.board_id = ME_PCB_182_04;
		GPIO_SetBits(USB_VBUS_PORT, USB_VBUS_PIN);		// Tell the USB to turn on.
	}	
	else
		Vision_Status.board_id = ME_PCB_182_03;
	
	Vision_Status.sts.USB_capable = true;					// set status bits to indicate that the board is capable of the following coms interfaces.  
	Vision_Status.sts.CAN_capable = true;
	Vision_Status.sts.UART_capable = true;
			
	Init_103(vision_settings.lf_hertz, vision_settings.can_baud, vision_settings.uartBaud);
	
	Vision_Status.sts.USB_Active = false;

	Vision_Status.UID = 0xdeadbeef;
	Vision_Status.kind = Pulse;
	
	Delay(10);
	
	SetLed(&LED1, Off, 0);
	// run through all the state machine tasks
	while (true)
	{
		cant_sleep = 0;
		Refresh_Settings_task(&settings);			// periodically read settings.
		DetermineTagType();

		CC1101_Task(&cc1101, &wake_flag, &cant_sleep);	// process RF communication
		Master_task(&messages_to_master, &cant_sleep);	// process communication
#ifdef VISION_TEST_MASTER
		vision_process(&me);
#endif
		GetStatus(&status);								// run the status update task.

		if (vision_settings.getActivities().tag_enable)
		{
#ifdef LF_TX_Capable
			LF_TX_Task(&LF_TX);							// periodically send a LF pulse
#endif
#ifdef LF_RX_Capable
			LF_Task(&LF_RX, &cant_sleep);
			LF_RSSI_watcher(&LF_RSSI, time_now()/((int)vision_settings.interval), &cant_sleep);			// Check LF RSSI and set LEDs/outputs
#endif
#ifdef use_HMI
			Test_task5(&HMI); //LCD
#endif		
			loopcounter++;

			IWDG_ReloadCounter();
			
			CheckAllGPIO(&cant_sleep);					// checks the status of the LEDs	

			if (vision_settings.getActivities().Always_on)
				cant_sleep++;			// prevent sleep if we need to stay alive.
			
			if (Vision_Status.sts.EXT_Power)
				cant_sleep++;			// prevent sleep if we have external power.
			
			if (Vision_Status.sts.USB_Active)
				cant_sleep++;			// prevent sleep if USB is connected
		}
				
		/* Enter Stop Mode */
		if (cant_sleep == 0)
		{
			Vision_Status.sts.Sleeping = 1;
			Sleep((int)vision_settings.interval);
			Vision_Status.sts.Sleeping = 0;
		}
	}

	/* Turn on LED1 */
	SetLed(&LED1, Red, 0);
	return 0;
}



extern uint16_t ADC_res[3];
void ADC_ConvCpltCallback(void)
{
	static const float vref_expected = 1.22;	// this is the expected value of the internal reference. used to correct values read in case VCCA is not 3.3V  
	float Vref = ADC_res[2] / 4096.0 * 3.3;
	float Vbatf = (uint32_t) ADC_res[1] * 2 / 4096.0 * 3.3;
	float Vsysf = (uint32_t) ADC_res[0] * 1100.0 / 100.0 / 4096.0 * 3.3;

	Vbatf *= vref_expected / Vref;
	Vsysf *= vref_expected / Vref;

	Vision_Status.Vsys = Vsysf * 1000.0;
	Vision_Status.Vbat = Vbatf * 1000.0;

	Vision_Status.sts.EXT_Power = (Vision_Status.Vsys > 4000) ? 1:0;
}


/**
 * This gets called during the RTC alarm interrupt. 
 */
void RTC_Alarm_callback(void)
{
	uint32_t time;
	if (AS3933_LF_indication() == 0)
	{
		time = RTC_GetCounter();
		systic = MAX(systic, time);
	}
	
	// Set the wake flag. tells the sleep routine its finished 
	wake_flag = 1;
	wake_count++;
}


void error_handler(int reason)
{
	while (1)
	{
		Delay(1000);
	}
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


