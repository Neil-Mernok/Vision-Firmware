/*
 * F103_Timers.c
 *
 *  Created on: Nov 13, 2012
 *      Author: J.L. Goosen
 */
#include "F103_Timers.h"
__IO uint16_t CCR1_Val = 5000; //5ms?
uint8_t TIM4_Done = 0;
uint8_t TIM5_Done = 0;

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







/**
 * @brief  Configures the RTC.
 * @param  None
 * @retval None
 */
void RTC_Configuration(void)
{
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
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
	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* Set RTC prescaler: set RTC period to fastest for counting RTOs tasks */
	RTC_SetPrescaler(0);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	RTC_SetCounter(0);

	// setup the RTC to generate an interrupt to wake the device. 

}











/***************************************************************************************/

/***************************************************************************************/
