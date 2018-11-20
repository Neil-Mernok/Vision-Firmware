/*
 * Delay.c
 *
 *  Created on: Jul 13, 2011
 *      Author: jvbiljon
 */

#include "Delay.h"

int status_timout = 0;
uint32_t systic = 0;

void Delay(__IO uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0)
	{
		//__WFI();
	}
}


//void Delay(__IO uint32_t nTime)
//{
//	vTaskDelay(nTime);
//}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
	
	systic++;		// systic for the state machines.
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
	return (uint32_t) MAX((s32)systic - (s32)time, 0);	
}


