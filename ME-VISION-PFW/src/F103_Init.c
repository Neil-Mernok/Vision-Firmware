/*
 * F103_Init.c
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#include "F103_Init.h"

/**
 * @brief  Configures the STM32F207, all peripherals and core functions
 * also initialises all global variables
 * @param  None
 * @retval None
 */
void Init_103(int lf_freq, int can_baud, int uart_baud)
{
	//Initialise global Variables

	//Setup Delay Timer
	/* Setup SysTick Timer for 1ms interrupts. */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1)
			;
	}
	
//setup UART's
	USART_Configure(Master_COM, uart_baud);

//	#ifdef USE_COM_1
//	USART_Configure(COM_1, COM_1_BAUD);
//#endif
//#ifdef USE_COM_2
//	USART_Configure(COM_2, COM_2_BAUD);
//#endif
//#ifdef USE_COM_3
//	USART_Configure(COM_3, COM_3_BAUD);
//#endif
//#ifdef USE_COM_4
//	USART_Configure(COM_4, COM_4_BAUD);
//#endif

	//setup ADC's
#ifdef USE_ADC
	ADC_Setup((uint32_t*)ADC_res);
#endif

// setup SPI
#ifdef USING_SPI1
	SPI1_Config();
#endif	
#ifdef USING_SPI2
	SPI2_Config();
#endif
#ifdef USING_SPI3
	SPI3_Config();
#endif
		
//	F103_i2c_init();

	//enable CAN1
#ifdef STM32F10X_HD	
	if (Vision_Status.USB_Active == false)
	CAN_Config(Vision_Settings.Can_baud*1000);
#else
	CAN_Config(can_baud);
#endif
	
			
	//setup Timers
	TIM2_Config();
#ifdef LF_TX_Capable
	TIM3_PWM_Config(lf_freq);
#endif
	TIM4_Config();
	TIM5_Config();
	
	TIM6_Config();
	Timer6_Stop();

	TIM7_Config();
	
	RTC_Configuration();

	// stop the IWDG and keep debugger connected. might need to disbale this for low power states...
	DBGMCU_Config( DBGMCU_STOP | DBGMCU_STANDBY | DBGMCU_IWDG_STOP, ENABLE );
		
	/* IWDG timeout equal to 10s */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/256 */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	Watchdog_reset_and_reload(8000);

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
//	IWDG_Enable();
}

/**
 * @brief: reconfigures the system clock after stop mode.
 * @param  None
 * @retval None
 */

void Watchdog_reset_and_reload(uint16_t watchdog_ms)
{
	watchdog_ms = watchdog_ms / 6;
	if (watchdog_ms > 0xFFF)
		watchdog_ms = 0xFFF;
	// assume a wors case cenario of 60kHz LSI timer, with 256 prescaler. therefore time ~ms/4
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetReload(watchdog_ms);
	IWDG_ReloadCounter();
}
