/*
 * sleep.h
 *
 *  Created on: May 2016
 *      Author: Kobus
 */

#ifndef SLEEP_H_
#define SLEEP_H_

#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

void SystemPower_Config(void);
void SystemPower_Config(void);
void SystemClockConfig_STOP(void);
void SystemClockConfig_WAKE(void);
void Sleep();

#ifdef __cplusplus
 }
#endif

#endif /* SLEEP_H_ */
