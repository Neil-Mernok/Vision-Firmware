/*
 * Delay.c
 *
 *  Created on: Jul 13, 2011
 *      Author: jvbiljon
 */
#include "Delay.h"
#include "stm32l4xx_hal.h"


int status_timout = 0;
uint32_t systic = 0;

void Delay(uint32_t nTime)
{
	HAL_Delay(nTime);
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
	return HAL_GetTick();
}

uint32_t time_since(uint32_t time)
{
	return (uint32_t) MAX((int32_t)HAL_GetTick() - (int32_t)time, 0);	
}


