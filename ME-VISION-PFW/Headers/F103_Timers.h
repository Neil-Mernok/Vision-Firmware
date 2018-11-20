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

#ifdef __cplusplus
extern "C" {
#endif

//defines

//Variables made public
extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
extern TIM_OCInitTypeDef  TIM_OCInitStructure;
extern __IO uint16_t CCR1_Val;
extern uint16_t PrescalerValue;
extern uint8_t TIM4_Done;
extern uint8_t TIM5_Done;

//Functions made public
void TIM2_Config(void);
void Timer2_Stop(void);
void Timer2_Start(void);
void Timer2_Restart(void);


void PWM_set_period(uint32_t percentage, uint32_t freq);
void TIM3_PWM_Config(uint32_t freq);
//void PWM_pins_change(bool PWM);


void TIM4_Config(void);
void Timer4_Stop(void);
void Timer4_Start(void);
void Timer4_Restart(void);

void TIM5_Config(void);
void Timer5_Stop(void);
void Timer5_SleepFor_mS(int millisecs);

void TIM6_Config(void);
void Timer6_Stop(void);
void Timer6_Restart(void);

void TIM7_Config(void);
void Timer7_Stop(void);
void Timer7_Restart(void);

extern int sleep_flag;
void RTC_Configuration(void);
void SYSCLKConfig_wake(void);

void RTC_Alarm_callback(void);

#ifdef __cplusplus
}
#endif
#endif /* F207_TIMERS_H_ */
