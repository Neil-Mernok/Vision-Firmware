/*
 * Global_Variables.h
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_




#ifdef __cplusplus
extern "C" {
#endif



//includes
#include "stm32f10x.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//defines
#define USE_USB
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))

//product detail
#define Product_Name			"Vision Bootloader\r\n"
#define Firmware_rev			5
#define RF_Boot					1

#define compile_date 			__DATE__



#define CAN_Vision_Poll_ID			0x12227100
#define CAN_Vision_Resp_ID			0x12227200
#define CAN_Vision_Sync_ID	 		0x12227300
#define CAN_VisBootPoll_ID			0x12227400
#define CAN_VisBootResp_ID			0x12227500
#define CAN_not_Last	 			0x00000800

typedef  void (*pFunction)(void);

//Functions made public

//////////////////////////////////////////////
//Header files to include
#include "F103_Init.h"
#include "F103_Uart.h"
#include "F103_SPI.h"
#include "F103_Timers.h"
#include "F103_CAN.h"
#include "F103_i2c.h"
#include "M24LR64.h"

//////////////////////////////////////////////
#include "list.h"
//////////////////////////////////////////////

#include "Delay.h"

//#include "ME-PCB-173-01-Ports.h"
//#include "ME-PCB-182-01-Ports.h"
#include "ME-PCB-182-02-Ports.h"

/// CC1101
//////////////////////////////////////////////
#include "hal_defs.h"
#include "hal_spi.h"
#include "hal_rf.h"
#include "cc1100.h"
#include "RX.h"
#include "TX.h"
//////////////////////////////////////////////

/// LF receive and transmit
//////////////////////////////////////////////
//#include "as3933.h"
//#include "LF_TX.h"
//////////////////////////////////////////////

#include "task.h"
#include "boot_Tasks.h"

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

//#include "Startup.h"

//#include "PDS_Tags.h"
#include "boot_RF_APL.h"
#include "boot_master_interface.h"
//#include "Vision_Settings.h"
//#include "about.h"
#include "Flash_defines.h"
#include "BootUpdate.h"

//#include "Tag.h"
//#include "TagLog.h"

//////////////////////////////////////////////

//#include "GFX.h"
//#include "SSD1306.h"
//#include "H_Ports.h"




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




RCC_ClocksTypeDef 			RCC_Clocks;
EXTI_InitTypeDef   			EXTI_InitStructure;
GPIO_InitTypeDef   			GPIO_InitStructure;
FSMC_NORSRAMInitTypeDef  	FSMC_NORSRAMInitStructure;
USART_InitTypeDef 			USART_InitStructure;
NVIC_InitTypeDef   			NVIC_InitStructure;
ADC_InitTypeDef       		ADC_InitStructure;
DMA_InitTypeDef       		DMA_InitStructure;
SPI_InitTypeDef  			SPI_InitStructure;
CAN_InitTypeDef				CAN_InitStructure;
CAN_FilterInitTypeDef   	CAN_FilterInitStructure;


extern uint8_t uart_packet_received;
extern uint8_t boot_data[];
extern uint16_t boot_buf_counter;

extern CanTxMsg TxMessage;
extern CanRxMsg RxMessage;

extern uint16_t boot;
//////////////////////////////////////////////

// RX data for CC1101 RF reception
//////////////////////////////////////////////

//Variables made public


#ifdef __cplusplus
}
#endif
#endif /* GLOBAL_VARIABLES_H_ */
