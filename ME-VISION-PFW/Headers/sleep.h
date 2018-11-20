/*
 * sleep.h
 *
 *  Created on: Jan 25, 2016
 *      Author: Kobus
 */

#ifndef SLEEP_H_
#define SLEEP_H_

#include "Global_Variables.h"

extern int wake_flag;

#ifdef __cplusplus
extern "C" {
#endif


//void SystemClockConfig_STOP(void);
//void SystemClockConfig_WAKE(void);
void Sleep(int millis);

#ifdef __cplusplus
 }
#endif

#endif /* SLEEP_H_ */
