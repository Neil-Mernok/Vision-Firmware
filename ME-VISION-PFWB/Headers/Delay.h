/*
 * Delay.h
 *
 *  Created on: Jul 13, 2011
 *      Author: jvbiljon
 */

#ifndef DELAY_H_
#define DELAY_H_
//includes
#include "stm32f10x.h"
#include <stddef.h>
#include <stdint.h>

//#include "Global_Variables.h"

// define how long a microsecond is
#define UsBase	16
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))


//variables made public
static __IO uint32_t TimingDelay;
extern int status_timout;

//Functions made public
void Delay(__IO uint32_t nTime);
//void TimingDelay_Decrement(void);
void DelayUs( uint32_t time );

uint32_t time_now(void);
uint32_t time_since(uint32_t time);

#endif /* DELAY_H_ */
