/*
 * Created on: Dec 11, 2012
 * Company: Mernok Elektronik 
 * Author: R. Shaw
 */

#ifndef BOOTUPDATE_H_
#define BOOTUPDATE_H_
//includes
#include "Global_Variables.h"

#define BUFFER_SIZE        ((uint16_t)(4096))


// store the boot reason in Backup register 1.
// Is 0 on power-on reset. other values should be set by application before resetting,
// i.e. to tell it to enter boot-load mode, or see if application failed

#ifdef BKP_DR1
#define boot_reason_loc 	BKP_DR1			
#else
#define boot_reason_loc 	RTC_BKP1R
#endif

// boot reasons
#define Boot_reason_none 		0x0000
#define Boot_reason_app_failed	0x2320
#define Boot_reason_program 	0x1122

//Flags and values for the firmware update module
struct UPDATE_Flags
{
	uint8_t F_UPDATE;				//flag to show an update is ready for sending
	uint32_t F_UPDATE_END;			//flag indicating update complete. 
	uint32_t Update_FileSize;		//size of the binary file
	uint32_t Update_Pointer;		//counter to monitor total data received
	uint32_t Master_last_seen;
	int master;
}UPDATE_Flags;

extern uint8_t	Run_Application;
extern uint8_t	Run_Bootloader;
extern uint8_t 	F_FlashErase;


//Functions made public
uint32_t Flash_Packet(uint8_t* data, uint32_t Len);
uint32_t Flash_Packet_at_adr(uint8_t* data, uint32_t Len, uint32_t start_adr);

#endif /* BOOTUPDATE_H_ */
