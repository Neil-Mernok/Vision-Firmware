/*
 * ME-PCB-217-02-Ports.h
 * Created on: October 2017
 * Company: Mernok Elektronik 
 * Author: F. Hattingh
 */


/// this is a HAL based file for use with newer libs. 

#ifndef ME_PCB_217_02_H_
#define ME_PCB_217_02_H_
//includes
#include <stdbool.h>
#include "stm32l4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*pFunction)(void);

// global defines
#ifndef PCB_217_02
#define PCB_217_02
#endif

//#define USING_SPI1
#define USING_SPI2
//#define USING_SPI3

#define CC_SPI hspi2
#define AS_SPI hspi1
//#define LCD_SPI SPI2

#define M24_i2c	hi2c1

#define USART_Configure(x)				MX_USART_UART_Init(Master_COM, x)
#define GPSModule_USART_Configure(x)	MX_USART_UART_Init(uBlox_COM, x)

//#define lf_freq_timer
#define lf_crystal
//#define LF_RX_Capable
//#define LF_TX_Capable

#define USING_CAN
#define USE_USB

//#define use_HMI	

#define GPIO_SetBits(port, pin) 			HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define GPIO_ResetBits(port, pin) 			HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define GPIO_ReadInputDataBit(port, pin)	HAL_GPIO_ReadPin(port, pin)

// CAN terminate 

////////////////////////////////////////////////////////////////
// RGB LED Defines
typedef enum
{
	Off 	= 0,
	Red 	= 4,
	Green 	= 2,
	Blue 	= 1,
	Violet	= 5,
	Yellow	= 6,
	Cyan	= 3,
	White	= 7,
} RGB_State_TypeDef;

typedef struct
{
	uint16_t  		R_pin;
	uint16_t  		G_pin;
	uint16_t  		B_pin;
	GPIO_TypeDef*	R_port;
	GPIO_TypeDef*	G_port;
	GPIO_TypeDef*	B_port;
	RGB_State_TypeDef last_color;
	RGB_State_TypeDef color;
	uint32_t		set_to;
	bool			inverted;
} RGB_LED_Typedef;

////////////////////////////////////////////////////////////////
// Timed GPIO Defines
typedef struct
{
	uint8_t			active_high;
	uint16_t 		Pin;            
	GPIO_TypeDef* 	Port;
	uint8_t			time_mode;
	uint32_t		set_to;
} 	_GPO;

#define LED_Inv

// LED defines		***********************************************************************
/// RGB LEDs
#define LED_1_Values  {GPIO_PIN_8, GPIO_PIN_12, GPIO_PIN_2, GPIOA, GPIOB, GPIOB, Off, Off, 0, true}
extern RGB_LED_Typedef LED1; 
// Misc IO defines 	***********************************************************************

// LF - PWM1 Output			: PC6
#define LF_PWM1_PORT		GPIOC
#define LF_PWM1_PIN			GPIO_PIN_6

// LF - PWM2 Output			: PC7
#define LF_PWM2_PORT		GPIOC
#define LF_PWM2_PIN			GPIO_PIN_7

// LF - Manchester data		: PA0
#define LF_DAT_PORT			GPIOA
#define LF_DAT_PIN			GPIO_PIN_0
#define LF_DAT_IRQ_Channel	EXTI0_IRQn

// LF - Clock				: PC1
#define LF_CLK_IRQ_PIN    	GPIO_PIN_1
#define LF_CLK_IRQ_PORT	    GPIOC
#define LF_CLK_IRQ_Channel	EXTI1_IRQn

// CC_GD0					: PA3
#define CC_GDO_IRQ_PIN   	GPIO_PIN_3
#define CC_GDO_IRQ_PORT	    GPIOA
#define CC_GDO_IRQ_Channel	EXTI3_IRQn

// HAL GPIO
#define LF_INT_Pin 			GPIO_PIN_2
#define LF_INT_GPIO_Port 	GPIOC

#define V_BAT_Pin 			GPIO_PIN_0
#define V_BAT_GPIO_Port 	GPIOA
#define V_IN_Pin 			GPIO_PIN_1
#define V_IN_GPIO_Port 		GPIOA

#define CS_EXT1_Pin 		GPIO_PIN_10
#define CS_EXT1_GPIO_Port 	GPIOA
#define CS_EXT2_Pin 		GPIO_PIN_4
#define CS_EXT2_GPIO_Port 	GPIOA

// RF & LF Interface - SPI1 : PA5, PA6, PA7
// Not used on ME-PCB-217-0x
#define SCK1_Pin 			GPIO_PIN_5
#define SCK1_GPIO_Port 		GPIOA
#define MISO1_Pin 			GPIO_PIN_6
#define MISO1_GPIO_Port 	GPIOA
#define MOSI1_Pin 			GPIO_PIN_7
#define MOSI1_GPIO_Port 	GPIOA

// SPI MISO for CC chip SOMI check : PB14
#define CC_MISO_PORT		GPIOB
#define CC_MISO_PIN			GPIO_PIN_14

// Module Interface - SPI2 : PB13, PB14, PB15
#define SCK2_Pin 			GPIO_PIN_13
#define SCK2_GPIO_Port 		GPIOB
#define MISO2_Pin 			GPIO_PIN_14
#define MISO2_GPIO_Port 	GPIOB
#define MOSI2_Pin 			GPIO_PIN_15
#define MOSI2_GPIO_Port 	GPIOB

// IO
#define IN0_Pin 			GPIO_PIN_4
#define IN0_GPIO_Port 		GPIOC
#define IN1_Pin 			GPIO_PIN_5
#define IN1_GPIO_Port 		GPIOC
#define IN2_Pin 			GPIO_PIN_0
#define IN2_GPIO_Port 		GPIOB
#define IN3_Pin 			GPIO_PIN_1
#define IN3_GPIO_Port 		GPIOB

// GPIO - Antenna Sense		: PC13
// Not used on ME-PCB-217-0x
#define ANT_SENSE_PORT		GPIOC
#define ANT_SENSE_PIN		GPIO_PIN_13

// GPIO - Critical Output	: PC8
#define CRIT_OUT_PORT		GPIOC
#define CRIT_OUT_PIN		GPIO_PIN_8

// GPIO - LF Interrupt		: PC2
// Not used on ME-PCB-217-0x
#define LF_IRQ_PIN        	GPIO_PIN_2
#define LF_IRQ_PORT	      	GPIOC
#define LF_IRQ_Channel		EXTI2_IRQn

// GPIO - Warning Output	: PC9
#define WARN_OUT_PORT		GPIOC
#define WARN_OUT_PIN		GPIO_PIN_9

// GPIO - EEPROM Power		: PA15
#define VRFID_PORT			GPIOA
#define VRFID_PIN			GPIO_PIN_15

// GPIO - Charging Indicator: PC0
#define Charging_PORT		GPIOC
#define Charging_PIN		GPIO_PIN_0

// GPIO - Version Detect 1	: PB3
#define VersionD_PORT		GPIOB
#define VersionD_PIN		GPIO_PIN_3

// GPIO - Version Detect 2	: PB4
#define VersionD2_PORT		GPIOB
#define VersionD2_PIN		GPIO_PIN_4

// GPIO - CAN Termination	: PB5
#define CAN_term_PORT		GPIOB
#define CAN_term_PIN		GPIO_PIN_5

// GPIO - CAN Low Power		: PC10
#define CAN_DIS_PORT		GPIOC
#define CAN_DIS_PIN			GPIO_PIN_10

// GPIO - LF Chip Select	: PC15
// Not used on ME-PCB-217-0x
#define LF_CS_PORT			GPIOC
#define LF_CS_PIN			GPIO_PIN_15

// GPIO - RF Chip Select 	:PC3
// Not used on ME-PCB-217-0x
#define CC_CS_PORT			GPIOC
#define CC_CS_PIN			GPIO_PIN_3

// Module Interface - UART3 : PB10, PB11
#define TX_GPIO_Port 		GPIOB
#define TX_Pin 				GPIO_PIN_10
#define RX_GPIO_Port 		GPIOB
#define RX_Pin 				GPIO_PIN_11

// Module Interface - UART3 : PC12, PD2
#define RTS_Pin 			GPIO_PIN_12
#define RTS_GPIO_Port 		GPIOC
#define CTS_Pin 			GPIO_PIN_2
#define CTS_GPIO_Port 		GPIOD

// GPS Interface - UART 2 	: PA2, PA3
#define TX2_Pin 			GPIO_PIN_2
#define TX2_GPIO_Port 		GPIOA
#define RX2_Pin 			GPIO_PIN_3
#define RX2_GPIO_Port 		GPIOA

// Module Interface - USB 	: PA11, PA12
#define USB_DM_Pin 			GPIO_PIN_11
#define USB_DM_GPIO_Port 	GPIOA
#define USB_DP_Pin 			GPIO_PIN_12
#define USB_DP_GPIO_Port 	GPIOA

// EEPROM Interface - I2C	: PB6, PB7
#define SCL_Pin 			GPIO_PIN_6
#define SCL_GPIO_Port 		GPIOB
#define SDA_Pin 			GPIO_PIN_7
#define SDA_GPIO_Port 		GPIOB

// Module Interface - CAN	: PB8, PB9
#define CAN_RX_Pin 			GPIO_PIN_8
#define CAN_RX_GPIO_Port 	GPIOB
#define CAN_TX_Pin 			GPIO_PIN_9
#define CAN_TX_GPIO_Port 	GPIOB

// GPS Time pulse			: PC1
#define uBlox_TP_IRQ_PIN     GPIO_PIN_1
#define uBlox_TP_IRQ_PORT	 GPIOC
#define uBlox_TP_IRQ_Channel EXTI1_IRQn

// GPS Hardware Reset		: PC14
#define uBlox_RST_PIN		GPIO_PIN_14
#define uBlox_RST_PORT		GPIOC

// GPS Interrupt 			: PC15
#define uBlox_OUT_PIN		GPIO_PIN_15
#define uBlox_OUT_PORT		GPIOC

#define OUT2_Pin 			GPIO_PIN_8
#define OUT2_GPIO_Port 		GPIOC
#define OUT3_Pin 			GPIO_PIN_9
#define OUT3_GPIO_Port 		GPIOC

// UART 1 - Unused on ME-PCB-217-0x
#define TX1_Pin 			GPIO_PIN_9
#define TX1_GPIO_Port 		GPIOA
#define RX1_Pin 			GPIO_PIN_10
#define RX1_GPIO_Port 		GPIOA

//Functions made public
void Config_Ports(void);
void LEDInit(RGB_LED_Typedef* LED);
void SetLed(RGB_LED_Typedef* LED, RGB_State_TypeDef state, int ms);
int CheckLed(RGB_LED_Typedef* LED);
void CheckAllGPIO(int* cant_sleep);
void SET_CC_CS(uint8_t value);
void GPIO_sleep_wake(int sleep);

int GetBoardVers();
int GetBoardRevision();

void MISC_IO_Init(void);

void Config_EXTI_LFCLK(pFunction callback);
void Config_EXTI_GD0(void);
void Start_EXTI_LFDAT(pFunction callback);
void Stop_EXTI_LFDAT(void);
void setup_coms_flag (uint32_t* flag);

#ifdef __cplusplus
}
#endif

#endif /* ME_PCB_217_02_PORTS_H_ */
