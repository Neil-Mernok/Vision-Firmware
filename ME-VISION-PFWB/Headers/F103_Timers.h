/*
 * F207_Timers.h
 * Created on: Mar 12, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */

#ifndef F103_TIMERS_H_
#define F103_TIMERS_H_
//includes
#include "Global_Variables.h"

//defines

//Variables made public
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
extern __IO uint16_t CCR1_Val;
uint16_t PrescalerValue;

//Functions made public
void TIM2_Config(void);
void Timer2_Stop(void);
void Timer2_Start(void);
void Timer2_Restart(void);
void RTC_Configuration(void);
#endif
