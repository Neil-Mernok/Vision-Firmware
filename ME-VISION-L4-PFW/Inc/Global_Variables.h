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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
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
#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

///#define time_since(time)	(xTaskGetTickCount() - time)
/// this will need to be changed to 87381 hz to avoid the 125kHz frequency band. 	// 32768kHz oscillator * 16 / 6
#define AS3933_desired_freq			125000					// 31250kHz oscillator * 16 / 4
#define AS3933_RES_FREQ_MAX         (AS3933_desired_freq+5000)
#define AS3933_RES_FREQ_MIN         (AS3933_desired_freq-5000)


#define CAN_Vision_Poll_ID			0x12227100
#define CAN_Vision_Resp_ID			0x12227200
#define CAN_Vision_Sync_ID	 		0x12227300
#define CAN_not_Last	 			0x00000800

#define IWDG_reload_val				2000

//////////////////////////////////////////////
//Header files to include
#include "M24LR64.h"

//////////////////////////////////////////////
#include "list.h"
//////////////////////////////////////////////

#include "ME-PCB-182-06-Ports.h"
#include "ME-PCB-138-03-Ports.h"

//////////////////////////////////////////////
#include "hal_defs.h"
#include "hal_spi.h"
#include "hal_rf.h"
#include "cc1100.h"
#include "RX.h"
#include "TX.h"
//////////////////////////////////////////////


//////////////////////////////////////////////
//#include "as3933.h"
//#include "LF_TX.h"
//////////////////////////////////////////////

#include "task.h"
#include "Delay.h"

//////////////////////////////////////////////
// Nanotron files:
#ifdef RANGING_CAPABLE
#include	"ntrxutil.h"
#include 	"nnspi.h"
#include	"phy.h"
#include    "config.h"
#include    "ntrxtypes.h"
#include    "hwclock.h"
#include	"phy.h"
#include 	"app.h"
#endif
//////////////////////////////////////////////
#include "Battery_measure.h"
#include "Update.h"
//////////////////////////////////////////////


//#ifdef __cplusplus
//extern "C" {
//#endif

#define uart_buf_size 0x800

extern uint8_t uart_packet_received;
extern uint8_t uart_data[];
extern uint16_t uart_buf_counter;

//// task structs
extern task settings;
extern task cc1101;
extern task messages_to_master;
extern task LF_TX;
extern task LF_RX;
extern task LF_RSSI;
extern task HMI;
extern task nanotron;

extern uint32_t last_UID;
//////////////////////////////////////////////

extern char debug_log;
extern char debug_buffer[2000];
extern int debug_counter;

//Todo: neil
extern bool sizeCalculated_flag;
extern bool MernokAsset_GroupList_populated;

typedef  void (*pFunction)(void);

//#ifdef __cplusplus
//}
//#endif
#endif /* GLOBAL_VARIABLES_H_ */
