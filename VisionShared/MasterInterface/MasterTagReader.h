/*
 * MasterTagReader.h
 *
 *  Created on: Jul 18, 2015
 *      Author: Kobus
 */

#ifndef MASTERTAGREADER_H_
#define MASTERTAGREADER_H_

#ifdef STM32F10X_CL
#include "stm32f10x.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Transponders.h"
#include "Delay.h"					// Delay needs to have a definition for time_now() and time_since()

#ifdef __cplusplus
extern "C" {
#endif


uint32_t parse_message_UID(uint8_t* data);
void parse_message_into_TAG(_Transpondert* T, uint8_t* data, int len);

#ifdef __cplusplus
}
#endif
#endif /* MASTERTAGREADER_H_ */
