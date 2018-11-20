/*
 * Battery_measure.h
 *
 *  Created on: Dec 7, 2013
 *  Modified 22 Aug 2014 for Vision
 *      Author: Kobus
 */

#ifndef BATTERY_MEASURE_H_
#define BATTERY_MEASURE_H_

#include "Global_Variables.h" 


#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t ADC_res[3];
void measure_volts(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_MEASURE_H_ */
