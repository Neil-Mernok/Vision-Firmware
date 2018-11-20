/*
 * Global_Variables.h
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_


//includes
#include "stm32f10x.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include "core_cm3.h"

//defines
#define USE_USB
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))

///#define time_since(time)	(xTaskGetTickCount() - time)


#define CAN_Vision_Poll_ID			0x12227100
#define CAN_Vision_Resp_ID			0x12227200
#define CAN_Vision_Sync_ID	 		0x12227300
#define CAN_not_Last	 			0x00000800

//#define num_transp_total 256

typedef  void (*pFunction)(void);

//Functions made public

//////////////////////////////////////////////
//Header files to include
#include "F107_ADC.h"
#include "F103_Init.h"
#include "F103_Uart.h"
#include "F103_SPI.h"
#include "F103_Timers.h"
#include "F103_CAN.h"
#include "F103_i2c.h"
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

#include "Delay.h"

#include "ME-PCB-182-02-Ports.h"
//#include "ME-PCB-173-01-Ports.h"
//#include "BSP.h"

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
//#include "Tasks.h"

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
//#include "Vision_Parameters.h"
#include "Battery_measure.h"
#include "about.h"
//#include "Transponders.h"
#include "Update.h"

//#include "Tag.h"
//#include "TagLog.h"

//////////////////////////////////////////////

#include "GFX.h"
#include "SSD1306.h"
#include "H_Ports.h"




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

extern RCC_ClocksTypeDef 			RCC_Clocks;
extern EXTI_InitTypeDef   			EXTI_InitStructure;
extern GPIO_InitTypeDef   			GPIO_InitStructure;
extern FSMC_NORSRAMInitTypeDef  	FSMC_NORSRAMInitStructure;
extern USART_InitTypeDef 			USART_InitStructure;
extern NVIC_InitTypeDef   			NVIC_InitStructure;
extern ADC_InitTypeDef       		ADC_InitStructure;
extern DMA_InitTypeDef       		DMA_InitStructure;
extern SPI_InitTypeDef  			SPI_InitStructure;
extern CAN_InitTypeDef				CAN_InitStructure;
extern CAN_FilterInitTypeDef 	  	CAN_FilterInitStructure;

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

extern uint32_t last_UID;
//////////////////////////////////////////////

extern char debug_log;
extern char debug_buffer[2000];
extern int debug_counter;

// RX data for CC1101 RF reception
//////////////////////////////////////////////
extern bool sizeCalculated_flag;
extern bool MernokAsset_GroupList_populated;


#endif /* GLOBAL_VARIABLES_H_ */
