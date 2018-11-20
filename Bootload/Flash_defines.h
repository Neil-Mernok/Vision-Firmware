/*
 * Flash_defines.h
 *
 *  Created on: Mar 8, 2013
 *      Author: Kobus Goosen
 */

#ifndef FLASH_DEFINES_H_
#define FLASH_DEFINES_H_

/* Includes ------------------------------------------------------------------*/
#include "Global_Variables.h"

/* Exported typedefs ---------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
typedef  void (*pFunction)(void);

/* Exported definitions ------------------------------------------------------*/
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#endif

/* Define the flash memory start address */
#define USER_FLASH_STARTADDRESS    ((uint32_t)0x08000000) /* Flash Start Address */

/* Define the address from where user application will be loaded.
   Note: the 1st sector 0x08000000-0x08007FFF is reserved for the Firmware upgrade code */
#ifdef STM32L4
#define APPLICATION_ADDRESS        (uint32_t)0x08010000
#else
#define APPLICATION_ADDRESS        (uint32_t)0x08008000
#endif

/* End of the Flash address - 512kB */
#define USER_FLASH_END_ADDRESS     ((uint32_t)0x0807FFFF)

/* Puplic macro -------------------------------------------------------------*/


/* Puplic variables ---------------------------------------------------------*/


/* Puplic function prototypes -----------------------------------------------*/
/* Puplic functions ---------------------------------------------------------*/





#endif /* FLASH_DEFINES_H_ */
