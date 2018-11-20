/*
 * Created on: Dec 11, 2012
 * Company: Mernok Elektronik 
 * Author: R. Shaw
 */

#ifndef UPDATE_H_
#define UPDATE_H_
//includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif


// store the boot reason in Backup register 1.
// Is 0 on power-on reset. other values should be set by application before resetting,
// i.e. to tell it to enter boot-load mode, or see if application failed

#ifdef USE_HAL_DRIVER
#define boot_reason_loc 	RTC_BKP_DR1
#else
#ifdef BKP_DR1
#define boot_reason_loc 	BKP_DR1			
#else
#define boot_reason_loc 	RTC_BKP1R
#endif
#endif


// boot reasons
#define Boot_reason_none 		0x0000
#define Boot_reason_app_failed	0x2323
#define Boot_reason_program 	0x1122


#ifdef __cplusplus
}
#endif

#endif /* UPDATE_H_ */
