/*
 * Battery_measure.c
 *
 *  Created on: Dec 7, 2013
 *  Modified 22 Aug 2014 for Vision
 *  
 *      Author: Kobus
 */

#include "Battery_measure.h"

uint16_t ADC_res[3];

void measure_volts(void)
{
	// Start the conversion 
#ifdef USE_HAL_DRIVER
#ifdef STM32L1
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_res, 3);
#else
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_res, 3);
#endif
#else
	HAL_ADC_Start_DMA();
#endif
}
