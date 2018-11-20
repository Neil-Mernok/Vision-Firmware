/*
 * Global_Variables.h
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_


//#define STM32L152xB
//includes
#include "stm32l1xx.h"

#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//#define USE_HAL_DRIVER

//defines

//#define USE_USB
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))

//product detail
#define Product_Name			"Vision Bootloader\r\n"
#define Firmware_rev			5
#define RF_Boot					1

#define compile_date 			__DATE__

//#define PCB_173_03

//#include "BSP.h"
#include "ME-PCB-173-03-Ports.h"

#if( defined( PCB_182_01) || defined( PCB_182_02)) 
#define LF_TX_Capable
#define use_HMI	
#else 
#endif

//////////////////////////////////////////////
#include "hal_defs.h"
#include "hal_spi.h"
#include "hal_rf.h"
#include "cc1100.h"
#include "RX.h"
#include "TX.h"
//////////////////////////////////////////////


#include "task.h"
#include "boot_Tasks.h"

#include "list.h"

#include "Delay.h"

#include "boot_RF_APL.h"
#include "boot_master_interface.h"

#include "Flash_defines.h"
#include "BootUpdate.h"

//////////////////////////////////////////////

extern uint8_t uart_packet_received;
extern uint8_t boot_data[];
extern uint16_t boot_buf_counter;

//// task structs
extern task settings;
extern task cc1101;
extern task messages_to_master;

extern uint16_t boot;

typedef  void (*pFunction)(void);

// RX data for CC1101 RF reception
//////////////////////////////////////////////


//#ifdef __cplusplus
//}
//#endif
#endif /* GLOBAL_VARIABLES_H_ */
