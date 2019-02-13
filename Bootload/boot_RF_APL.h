/*
 * RF_APL.h
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

#ifndef BOOT_RF_APL_H_
#define BOOT_RF_APL_H_

#include "Global_Variables.h"


#ifdef __cplusplus
extern "C" {
#endif

//typedef enum
//{
//	zone_none,
//	zone_pres,
//	zone_warn,
//	zone_crit
//} zone;


void Apl_Parse_message(uint8_t* buffer, int len, uint8_t RSSI);

#ifdef __cplusplus
}
#endif

#endif /* RF_APL_H_ */
