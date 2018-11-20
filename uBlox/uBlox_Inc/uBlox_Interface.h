/*
 * uBlox_Interface.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef UBLOX_INTERFACE_H_
#define UBLOX_INTERFACE_H_

#include <stddef.h>
#include <stdint.h>
#include "usart.h"

#include "uBlox_General.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Checksum_Offset 	2
#define Identifier_Offset 	6

extern void UBX_Hard_Reset(void);
uint8_t UBX_TX(_uBlox_Message* TX_Message);


#ifdef __cplusplus
}
#endif

#endif /* UBLOX_INTERFACE_H_ */
