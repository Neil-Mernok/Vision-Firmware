/*
 * F103_Timers.c
 *
 *  Created on: Nov 13, 2012
 *      Author: J.L. Goosen
 */
#include "F103_Timers.h"
void PWM_pins_change(bool PWM);

uint8_t TIM4_Done = 0;
uint8_t TIM5_Done = 0;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint16_t CCR1_Val = 3000; //0.2ms?
uint16_t PrescalerValue;

//Functions made public

/**
 * @brief  Configure the TIMER. timer 2 will be used for uart timeout from now on, as tim3 is now used for PWM
 * @param  None
 * @retval None
 */
void TIM2_Config(void)
{
	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
	//this is time base of 1us

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	Timer2_Stop();
}

/**
 * @brief  Stop timer2
 * @param  None
 * @retval None
 */
void Timer2_Stop(void)
{
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, DISABLE);
}

/**
 * @brief  Allow timer 2 to run
 * @param  None
 * @retval None
 */
void Timer2_Start(void)
{
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  restart timer 2 with 0 value
 * @param  None
 * @retval None
 */
void Timer2_Restart(void)
{
	TIM_Cmd(TIM2, DISABLE);
	TIM_SetCounter(TIM2, 0);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}



#ifdef LF_TX_Capable 



/**
 * @brief  Configure the TIM3 for 125kHz pwm.
 * @param  None
 * @retval None
 */
 
void TIM3_PWM_Config(uint32_t freq)
{
	uint32_t period = 0;
	
	// switch PWMs to normal GPIOs
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	  
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);	
	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Compute the prescaler value */
	PrescalerValue = 0;		// run at sys freq

	/* Time base configuration */
	period = ((SystemCoreClock) / freq /2) - 1;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	// setup the 2 PWM channels.
	PWM_set_period(100, freq);
	
	// TIM3 enable counter
	TIM_Cmd(TIM3, ENABLE);
	// switch off outputs
	PWM_pins_change(FALSE);
}



/**
 * resets the PWMtimer to 0. this was done to avoid low frequency alliassing in the output signal. 
 */
void PWM_reset(void)
{
	TIM_Cmd(TIM3, DISABLE);
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}

/**
 * Brief: this enables or disables the PWM pins. 
 * @param PWM
 */
void PWM_pins_change(bool PWM)
{
	if (PWM)
	{
		// PWM Pin Configuration: TIM3 channel1 = PC6, channel2 = PC7
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	
	}
	else
	{
		// switch to normal GPIOs
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	
		// make sure they're low.
		GPIO_ResetBits(LF_PWM1_PORT, LF_PWM1_PIN);
		GPIO_ResetBits(LF_PWM2_PORT, LF_PWM2_PIN);
	}
}

/**
 * @brief	set the PWM pulse width power in percentage. 0-100.
 */
void PWM_set_period(uint32_t percentage, uint32_t freq)
{
	uint32_t period = 0;
	
	/// make sure not to use more than 100%
	if(percentage > 100) 
		percentage =  100;
	
	/// make sure frequency is also sane
	if(freq > 150000) 
		freq = 150000;
	if(freq < 30000)
		freq = 30000;
		
	
	if(percentage > 50)
	{
		period = ((SystemCoreClock) / freq /2);

		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = percentage * period / 200 - 1;				// value is 0 -> period/2
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		//TIM_OCInitStructure.TIM_Pulse = (200 - percentage) * period / 200 - 1;		// value is period/2 -> period.  
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	}
	else
	{
		period = ((SystemCoreClock) / freq);

		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = ((100 + percentage) * period) / 200 - 1;				// value is Period/2 + Period/2*percentage
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

		/* PWM1 Mode configuration: Channel2 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = ((100 - percentage) * period) / 200 - 1;				// value is Period/2 - Period/2*percentage  
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	}
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = period - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
}
#endif


///**
// * @brief  Stop timer3
// * @param  None
// * @retval None
// */
//void Timer3_Stop(void)
//{
//	/* TIM3 enable counter */
//	TIM_Cmd(TIM3, DISABLE);
//}
//
///**
// * @brief  Allow timer 3 to run
// * @param  None
// * @retval None
// */
//void Timer3_Start(void)
//{
//	/* TIM3 enable counter */
//	TIM_Cmd(TIM3, ENABLE);
//}
//
///**
// * @brief  restart timer 3 with 0 value
// * @param  None
// * @retval None
// */
//void Timer3_Restart(void)
//{
//	TIM_Cmd(TIM3, DISABLE);
//	TIM_SetCounter(TIM3, 0);
//
//	/* TIM3 enable counter */
//	TIM_Cmd(TIM3, ENABLE);
//}

/**
 * @brief  Configure the TIM IRQ Handler.
 * @param  None
 * @retval None
 */
void TIM4_Config(void)
{
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* Enable the TIM4 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
	//this is time base of 1us

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 4000;		// approx 4 millisecs to timeout
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);

	Timer4_Stop();
}

/**
 * @brief  Stop timer4
 * @param  None
 * @retval None
 */
void Timer4_Stop(void)
{
	/* TIM4 enable counter */
	TIM_Cmd(TIM4, DISABLE);
}


/**
 * @brief  Allow timer 4 to run
 * @param  None
 * @retval None
 */
void Timer4_Start(void)
{
	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief  restart timer 4 with 0 value
 * @param  None
 * @retval None
 */
void Timer4_Restart(void)
{
	TIM_Cmd(TIM4, DISABLE);
	TIM_SetCounter(TIM4, 0);

	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}


/**
 * @brief  Used for LF timeout
 * @param  None
 * @retval None
 */
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1 ) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1 );
		TIM_SetCompare1(TIM4, 4000);

//		AS3933_LF_timout_ISR();
		
		TIM4_Done = 1;
		Timer4_Stop();
	}
}



/**
 * @brief  Configure the TIM IRQ Handler.
 * @param  None
 * @retval None
 */
void TIM5_Config(void)
{
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 2000) - 1;
	//this is time base of 0.5ms

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM5, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;							// arbitrary value. needs to be set each time. 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);

	Timer5_Stop();
}

/**
 * @brief  Stop timer5
 * @param  None
 * @retval None
 */
void Timer5_Stop(void)
{
	/* TIM5 enable counter */
	TIM_Cmd(TIM5, DISABLE);
}

/**
 * @brief  run timer for a few milliseconds
 * @param  None
 * @retval None
 */
void Timer5_SleepFor_mS(int millisecs)
{
	TIM_Cmd(TIM5, DISABLE);
	TIM_SetCounter(TIM5, 0);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = millisecs * 2;       // run time in ms 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	/* TIM5 enable counter */
	TIM5_Done = 0;
	TIM_Cmd(TIM5, ENABLE);

	//////////////////   now wait for it to finish   /////////////////////////
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);	// sleeps. registers are preserved, wakes on interrupt. ~30uA
	while (TIM5_Done == 0)
		;
	//////////////////////////////////////////////////////////////////////////
}




















/**
 * @brief: configures timer 6. currently used for timing buzzer...
 */
void TIM6_Config(void)
{
	/* TIM6 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	//PrescalerValue = 0;
	//this is time base of 36MHz
	//72MHz/1099/65514 = 1Hz.
	//36MHz/(65514/2) = 1100Hz.
	//36MHz/(3600*3) = 3333Hz.

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (3600-1)*3;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	/* Prescaler configuration */
	//TIM_PrescalerConfig(TIM6, PrescalerValue, TIM_PSCReloadMode_Immediate);

	//TIM_OC1PreloadConfig(TIM6, TIM_OCPreload_Disable);

	TIM_Cmd(TIM6,ENABLE);

}

/**
 * @brief  Stop timer6
 * @param  None
 * @retval None
 */
void Timer6_Stop(void)
{
	/* TIM6 enable counter */
	TIM_Cmd(TIM6, DISABLE);
}

/**
 * @brief  restart timer 6 with 0 value
 * @param  None
 * @retval None
 */
void Timer6_Restart(void)
{
	TIM_Cmd(TIM6, DISABLE);
	TIM_SetCounter(TIM6, 0);

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);
}


/**
 * @brief  Used for buzzer.
 * @param  None
 * @retval None
 */
void TIM6_IRQHandler(void)
{

	  static int state = 0;

	  TIM_ClearITPendingBit(TIM6,TIM_IT_Update);

	  state ^= 1;

	  if (state) // Toggle PD12 for 1 Hz rate
	    GPIO_SetBits(GPIOC, GPIO_Pin_8);
	  else
	    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}












/**
 * @brief: configures timer 7. currently used for timing LF stuff...
 */
void TIM7_Config(void)
{
	/* TIM7 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	/* Compute the prescaler value */
	PrescalerValue = 0;
	//this is time base of 36MHz

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM7, PrescalerValue, TIM_PSCReloadMode_Immediate);

//	Timer 7 is not preload capable.
//	TIM_OC1PreloadConfig(TIM7, TIM_OCPreload_Disable);

	TIM_Cmd(TIM7, ENABLE);
}

/**
 * @brief  Stop timer7
 * @param  None
 * @retval None
 */
void Timer7_Stop(void)
{
	/* TIM7 enable counter */
	TIM_Cmd(TIM7, DISABLE);
}

/**
 * @brief  restart timer 7 with 0 value
 * @param  None
 * @retval None
 */
void Timer7_Restart(void)
{
	TIM_Cmd(TIM7, DISABLE);
	TIM_SetCounter(TIM7, 0);

	/* TIM7 enable counter */
	TIM_Cmd(TIM7, ENABLE);
}
















/***************************************************************************************
 *
 *	RTC and alarm setup. used to put the micro into low power mode. 
 *
 /**************************************************************************************/
/**
 * @brief  Configures the RTC. setup for roughly 1ms period and wake on alarm via EXTI-17  
 * @param  None
 * @retval None
 */
void RTC_Configuration(void)
{
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	
	/* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* RTC clock source configuration ------------------------------------------*/
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);
	/* Reset Backup Domain */
	BKP_DeInit();
	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);
	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}
	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/* Set the RTC time base to roughly 1ms */
	RTC_SetPrescaler(39);
	RTC_WaitForLastTask();
	
	RTC_SetCounter(0);
	RTC_WaitForLastTask();	

	// setup the RTC to generate an interrupt to wake the device. 
	/* Enable the RTC Alarm interrupt */
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
	RTC_WaitForLastTask();

	RTC_ClearFlag(RTC_FLAG_ALR|RTC_FLAG_SEC|RTC_FLAG_OW);
	
	/* Configures NVIC and Vector Table base location for RTC alarm. */
	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



/**
 * @brief  This function handles RTC Alarm interrupt request.
 * @param  None
 * @retval None
 */
void RTCAlarm_IRQHandler(void)
{
	if (RTC_GetITStatus(RTC_IT_ALR) != RESET)
	{
		/* Clear EXTI line17 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line17);

		/* Check if the Wake-Up flag is set */
		if (PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
		{
			/* Clear Wake Up flag */
			PWR_ClearFlag(PWR_FLAG_WU);
		}

		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_ALR);
		
		// handle the RTC wake events
		RTC_Alarm_callback();
	}
}

/**
 * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
 *         and select PLL as system clock source.
 * @param  None
 * @retval None
 */
void SYSCLKConfig_wake(void)
{
	ErrorStatus HSEStartUpStatus = ERROR;

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{

#ifdef STM32F10X_CL
		/* Enable PLL2 */
		RCC_PLL2Cmd(ENABLE);

		/* Wait till PLL2 is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
		{
		}

#endif

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
//    while(RCC_GetSYSCLKSource() != 0x08)
//    {
//    }
	}
}

/***************************************************************************************/

/***************************************************************************************/
