/*
 * Delay.h
 *
 *  Created on: Jul 13, 2011
 *      Author: jvbiljon
 */

#ifndef DELAY_H_
#define DELAY_H_
//includes
#include <stddef.h>
#include <stdint.h>

#include "Vision_HAL.h"

#ifdef __cplusplus
extern "C" {
#endif

//Functions made public
void Delay( uint32_t nTime);
void TimingDelay_Decrement(void);
void DelayUs( uint32_t time );

uint32_t time_now(void);
uint32_t time_since(uint32_t time);

#ifdef __cplusplus
}
#endif

#endif /* DELAY_H_ */
