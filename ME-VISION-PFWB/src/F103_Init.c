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
void Init_103(void)
{
	// Configure GPIOs
	/* Disable the Jtag Debug Ports (but not SW) */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

	MISC_IO_Init();
	LEDInit(&LED1);

	RCC_GetClocksFreq(&RCC_Clocks);
	
	//Setup Delay Timer
	/* Setup SysTick Timer for 1ms interrupts. */
	if (SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 1000))
	{
		/* Capture error */
		while (1)
			;
	}
	
//setup UART's
	USART_Configure(Master_COM, COM_1_BAUD);

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
	CAN_Config(250000);
#endif
				
	//setup Timers
	TIM2_Config();

	// enable usage of the backup registers. used in this case as a place-holder for Bootloader <-> Application communication
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_ClearFlag();

	/* Clear reset flags */
	RCC_ClearFlag();

	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
		
	/* IWDG timeout equal to 10s */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/256 */
	IWDG_SetPrescaler(IWDG_Prescaler_256);

	IWDG_SetReload(0x1FF);
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

