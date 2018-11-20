/*
 * Delay.c
 *
 *  Created on: Jul 13, 2011
 *      Author: jvbiljon
 */

#include "Delay.h"

#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))

int status_timout = 0;
uint32_t systic = 0;

void Delay(uint32_t nTime)
{
	uint32_t TimingDelay = time_now() + nTime;
	while(time_now() < TimingDelay)
	{	
//		__WFI();
	}
}

void TimingDelay_Decrement(void)
{	
	systic++;		// systic for the state machines.
#ifdef LF_TX_Capable
	LF_send_bits();
#endif
}

void DelayUs(uint32_t time)
{
	uint32_t i = 0, j = 0;

	for (i = 0; i < time; i++)
	{
		while (j < UsBase)
			j++;
		j = 0;
	}
}

uint32_t time_now(void)
{
	return systic;
}

uint32_t time_since(uint32_t time)
{
	return (uint32_t) MAX((int32_t)systic - (int32_t)time, 0);	
}


