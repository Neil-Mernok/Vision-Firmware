/*
 * uBlox_Poll.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef UBLOX_POLL_H_
#define UBLOX_POLL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t UBX_CLASS_ID_Poll(uint8_t CLASS_Poll, uint8_t ID_Poll);
uint8_t UBX_MON_VER_Poll(void);
uint8_t UBX_NAV_PVT_Poll(void);
uint8_t UBX_NAV_POSECEF_Poll(void);
uint8_t UBX_NAV_ODO_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* UBLOX_POLL_H_ */

