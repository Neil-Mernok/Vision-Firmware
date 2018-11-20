/*
 * Global_Variables.h
 * Created on: May 2016
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen 
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_


//#define STM32L476RG
//includes
#include "stm32l4xx_hal.h"
//#include "adc.h"
#include "can.h"
#include "dma.h"
//#include "i2c.h"
#include "iwdg.h"
//#include "lptim.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "tim.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//defines

//#define USE_USB
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))

//product detail
#define Product_Name			"VISION Pulse Module Bootloader\r\n"
#define Firmware_rev			(0x80 | 5)
#define RF_Boot					1

#define compile_date 			__DATE__

#define CAN_Vision_Poll_ID			0x12227100
#define CAN_Vision_Resp_ID			0x12227200
#define CAN_Vision_Sync_ID	 		0x12227300
#define CAN_VisBootPoll_ID			0x12227400
#define CAN_VisBootResp_ID			0x12227500
#define CAN_not_Last	 			0x00000800
//////////////////////////////////////////////

#include "ME-PCB-182-06-Ports.h"

//////////////////////////////////////////////
#include "hal_defs.h"
#include "hal_spi.h"
#include "hal_rf.h"
#include "cc1100.h"
#include "RX.h"
#include "TX.h"
//////////////////////////////////////////////

//////////////////////////////////////////////
#include "../Project_Headers/task.h"
#include "boot_Tasks.h"

#include "list.h"
#include "Delay.h"

#include "boot_RF_APL.h"
#include "boot_master_interface.h"

#include "Flash_defines.h"
#include "BootUpdate.h"
//////////////////////////////////////////////

uint8_t pass_counter;
uint8_t fail_counter;
uint32_t debug_time;

//#ifdef __cplusplus
//extern "C" {
//#endif

extern uint8_t uart_packet_received;
extern uint8_t uart_data[];
extern uint16_t uart_buf_counter;

typedef  void (*pFunction)(void);

//#ifdef __cplusplus
//}
//#endif
#endif /* GLOBAL_VARIABLES_H_ */
