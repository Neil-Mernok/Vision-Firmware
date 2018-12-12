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
//#include "stm32l1xx.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

//#define num_transp_total 			16			// dont really need to keep track of tags...

//Functions made public




//////////////////////////////////////////////
//Header files to include
#include "M24LR64.h"

//////////////////////////////////////////////
// FreeRTOS stuff:
/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "queue.h"
//#include "timers.h"
//#include "semphr.h"
#include "list.h"
//#define USE_FULL_ASSERT 1

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
//#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

//////////////////////////////////////////////

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


//////////////////////////////////////////////
//#include "as3933.h"
//#include "LF_TX.h"
//////////////////////////////////////////////

//#include "RF_APL.h"
#include "task.h"
//#include "Tasks.h"

#include "Delay.h"
//////////////////////////////////////////////
// Nanotron files:
//#include	"ntrxutil.h"
//#include 	"nnspi.h"
//#include	"phy.h"
//#include    "config.h"
//#include    "ntrxtypes.h"
//#include    "hwclock.h"
//#include	"phy.h"
//#include 	"app.h"

//////////////////////////////////////////////

#include "Startup.h"

//#include "PDS_Tags.h"

//#include "master_interface.h"
//#include "Vision_Settings.h"
#include "Battery_measure.h"
//#include "about.h"
//#include "Transponders.h"
#include "Update.h"

//#include "Tag.h"
//#include "TagLog.h"

//////////////////////////////////////////////

#ifdef use_HMI
#include "GFX.h"
#include "SSD1306.h"
#include "H_Ports.h"
#endif



//////////////////////////////////////////////
// USB Files
#ifdef USE_USB
#ifdef STM32F10X_CL
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
extern USB_OTG_CORE_HANDLE    USB_OTG_dev;
extern CDC_IF_Prop_TypeDef APP_FOPS;
#endif
#ifdef STM32F10X_HD
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_istr.h"
#endif
#endif
//////////////////////////////////////////////



//#ifdef __cplusplus
//extern "C" {
//#endif


extern uint8_t uart_packet_received;
extern uint8_t uart_data[256];
extern uint8_t uart_buf_counter;

extern uint8_t uBlox_packet_received;
extern uint8_t uBlox_data[1024];
extern uint16_t uBlox_buf_counter;

//extern CanTxMsg TxMessage;
//extern CanRxMsg RxMessage;

//// task structs
extern task settings;
extern task cc1101;
extern task messages_to_master;
extern task LF_TX;
extern task LF_RX;
extern task LF_RSSI;
extern task HMI;

extern uint32_t last_UID;
//////////////////////////////////////////////

extern char debug_log;
extern char debug_buffer[2000];
extern int debug_counter;

typedef  void (*pFunction)(void);

//Todo: neil
extern bool MernokAsset_GroupList_populated;
extern bool sizeCalculated_flag;

// RX data for CC1101 RF reception
//////////////////////////////////////////////


//#ifdef __cplusplus
//}
//#endif
#endif /* GLOBAL_VARIABLES_H_ */
