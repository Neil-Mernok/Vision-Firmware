/*
 * Delay.h
 *
 *  Created on: Jul 13, 2011
 *      Author: jvbiljon
 */

#ifndef DELAY_H_
#define DELAY_H_
//includes
#include "stm32l1xx_hal.h"
#include <stddef.h>
#include <stdint.h>

//#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif


// define how long a microsecond is
#define UsBase	8
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))



//Functions made public
#define time_now()	 	HAL_GetTick()
#define time_since(x)	(HAL_GetTick() - x)
#define Delay(x)		HAL_Delay(x)

//void Delay(__IO uint32_t nTime);
////void TimingDelay_Decrement(void);
void DelayUs( uint32_t time );
//
//uint32_t time_now(void);
//uint32_t time_since(uint32_t time);
#ifdef __cplusplus
}
#endif

#endif /* DELAY_H_ */
