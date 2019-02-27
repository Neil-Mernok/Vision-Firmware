/*
 * ME-PCB-182-01-Ports.h
 * Created on: AUG 2014
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */

#ifndef ME_PCB_182_02_H_
#define ME_PCB_182_02_H_
//includes
//#include "Global_Variables.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_gpio.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef  void (*pFunction)(void);

// global defines
//#define PCB_173_03

#define USING_SPI1
#define USING_SPI2
//#define USING_SPI3

#define CC_SPI hspi1
#define AS_SPI hspi2
//#define LCD_SPI SPI2

#define M24_i2c	hi2c1

#define lf_crystal
//#define lf_freq_timer
 
#define USART_Configure(x)	MX_USART1_UART_Init(x)

#define LF_RX_Capable

#define WARNING_OUTPUTS

//#define use_HMI	


#define GPIO_SetBits(port, pin) 			HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define GPIO_ResetBits(port, pin) 			HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define GPIO_ReadInputDataBit(port, pin)	HAL_GPIO_ReadPin(port, pin)

/*Port map for VISION Mantag board: PCB-173-02, including names

*/

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

// LED defines		***********************************************************************
/// RGB LEDs
#define LED_1_Values  {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIOB, GPIOB, GPIOB, Off, Off, 0, true}
extern RGB_LED_Typedef LED1; 












// Misc IO defines 	***********************************************************************

//#define NN_CS_PIN                  		GPIO_Pin_15
//#define NN_CS_PORT                		GPIOC
//#define NN_CS_GPIO_CLK             		RCC_APB2Periph_GPIOC

// IO's for Nanotron module

//#define N_PON_R_PIN                  	GPIO_Pin_13
//#define N_PON_R_PORT                	GPIOC
//#define N_PON_R_GPIO_CLK             	RCC_APB2Periph_GPIOC

//#define N_uRESET_PIN                  	GPIO_Pin_0
//#define N_uRESET_PORT                	GPIOA
//#define N_uRESET_GPIO_CLK             	RCC_APB2Periph_GPIOA

//#define N_uIRQ_PIN                  	GPIO_Pin_2
//#define N_uIRQ_EXTI						EXTI_Line2
//#define N_uIRQ_PORT	                	GPIOC
//#define N_uIRQ_GPIO_CLK             	RCC_APB2Periph_GPIOC
//#define N_uIRQ_GPIO_PortSource			GPIO_PortSourceGPIOC
//#define N_uIRQ_GPIO_PinSource			GPIO_PinSource2
//#define N_uIRQ_Channel					EXTI2_IRQn
//#define N_uIRQ_Handler					EXTI2_IRQHandler

/////////////////////////////////////////////////////////////////////

#define Battery_ADC_ch 					ADC_CHANNEL_1
#define Battery_PIN						GPIO_PIN_1
#define Battery_PORT					GPIOA

#define Charging_PORT					GPIOA
#define Charging_PIN					GPIO_PIN_11

//#define USB_Pull_PORT					GPIOA
//#define USB_Pull_PIN					GPIO_Pin_8

#define EXT_POWER_PORT					GPIOA
#define EXT_POWER_PIN					GPIO_PIN_1

#define BRD_detct_PORT					GPIOA
#define BRD_detct_PIN					GPIO_PIN_15

#define VER_detct_PORT					GPIOC
#define VER_detct_PIN					GPIO_PIN_13

//#define CAN_DIS_PORT					GPIOC
//#define CAN_DIS_PIN						GPIO_Pin_10

//SPI MISO for CC chip SOMI heck
#define CC_MISO_PORT					GPIOA
#define CC_MISO_PIN						GPIO_PIN_6

// SPI CS pins
#define CC_CS_PORT						GPIOA
#define CC_CS_PIN						GPIO_PIN_4

#define LF_CS_PORT						GPIOH
#define LF_CS_PIN						GPIO_PIN_0

#define AC_CS_PORT						GPIOB				// not connected
#define AC_CS_PIN						GPIO_PIN_10

#define DISP_CS_PORT					GPIOB				// not connected
#define DISP_CS_PIN						GPIO_PIN_11

// output used to supply power to the i2c eeprom.
#define VRFID_PORT						GPIOA
#define VRFID_PIN						GPIO_PIN_8

// this pin is where Manchester data comes in. 
#define LF_DAT_PORT						GPIOB
#define LF_DAT_PIN						GPIO_PIN_5

// this pin is the AS3933 SPI clk port: PB13
#define LF_SCLK_PORT					GPIOB				
#define LF_SCLK_PIN						GPIO_PIN_13
 	
// output used to enable the charge circuit
#define CHRG_EN_PORT					GPIOA				
#define CHRG_EN_PIN						GPIO_PIN_12

// LF_PWM1
//#define LF_PWM1_PORT					GPIOC
//#define LF_PWM1_PIN						GPIO_Pin_6
// LF_PWM2
//#define LF_PWM2_PORT					GPIOC
//#define LF_PWM2_PIN						GPIO_Pin_7

// GPIO for warning output
#ifdef NOTTIliT
#define GPO0_OUT_PIN					GPIO_PIN_8
#endif
#define GPO1_OUT_PIN					GPIO_PIN_9
#define GPO2_OUT_PIN					GPIO_PIN_10
#define GPO3_OUT_PIN					GPIO_PIN_11
#define GPO_OUT_PORT					GPIOB

#define LAMP_OUT_PORT					GPO_OUT_PORT
#define LAMP_OUT_PIN					GPO3_OUT_PIN

#define BUZ_OUT_PORT					GPO_OUT_PORT
#define BUZ_OUT_PIN						GPO1_OUT_PIN

#ifdef NOTTIliT
#define LED_OUT_PORT					GPO_OUT_PORT
#define LED_OUT_PIN						GPO0_OUT_PIN
#else
#define TILT_IN_PORT					GPIOB
#define TILT_IN_PIN						GPIO_PIN_8
#endif

#define VIB_OUT_PORT					GPO_OUT_PORT
#define VIB_OUT_PIN						GPO2_OUT_PIN

#define CRIT_OUT_PIN					BUZ_OUT_PIN
#define CRIT_OUT_PORT					BUZ_OUT_PORT

#define WARN_OUT_PIN					LED_OUT_PIN
#define WARN_OUT_PORT					LED_OUT_PORT

#define BUTTON_PIN						GPIO_PIN_1
#define BUTTON_PORT						GPIOH

// GPS Hardware Reset		: PB4
#define uBlox_RST_PIN					GPIO_PIN_4
#define uBlox_RST_PORT					GPIOB

// Interrupt pins
// LF_INT			: 	PB4
#define LF_IRQ_PIN                  	GPIO_PIN_4
#define LF_IRQ_EXTI						EXTI_Line4
#define LF_IRQ_PORT	               		GPIOB
#define LF_IRQ_GPIO_PortSource			GPIO_PortSourceGPIOB
#define LF_IRQ_GPIO_PinSource			GPIO_PinSource4
#define LF_IRQ_Channel					EXTI4_IRQn
#define LF_IRQ_Handler					EXTI4_IRQHandler

// LF_dat			:	PB5
#define LF_DAT_IRQ_Channel				EXTI9_5_IRQn

// LF_clk			:	PB3
#define LF_CLK_IRQ_PIN                 	GPIO_PIN_3
#define LF_CLK_IRQ_PORT	                GPIOB
#define LF_CLK_IRQ_Channel				EXTI3_IRQn

// CC_GD0			:	PA2
#define CC_GDO_IRQ_PIN                 	GPIO_PIN_2
#define CC_GDO_IRQ_PORT	                GPIOA
#define CC_GDO_IRQ_Channel				EXTI2_IRQn

////////////////////////////////////////////////////////////////
// Timed GPIO Defines

typedef struct
{
	uint8_t			active_high;
	uint16_t 		Pin;            
	GPIO_TypeDef* 	Port;
	uint32_t		set_to;
} 	_GPO;


#define BUZ_OUT_values 	{1, BUZ_OUT_PIN, BUZ_OUT_PORT, 0}
extern _GPO		BUZ_OUT;	

#define VIB_OUT_values 	{1, VIB_OUT_PIN, VIB_OUT_PORT, 0}
extern _GPO		VIB_OUT;

#ifdef NOTTIliT
#define LED_OUT_values 	{1, LED_OUT_PIN, LED_OUT_PORT, 0}
extern _GPO		LED_OUT;			 
#endif

#define LAMP_OUT_values {0, LAMP_OUT_PIN, LAMP_OUT_PORT, 0}
extern _GPO		LAMP_OUT;


//Functions made public
void Config_Ports(void);
void LEDInit(RGB_LED_Typedef* LED);
void SetLed(RGB_LED_Typedef* LED, RGB_State_TypeDef state, int ms);
int CheckLed(RGB_LED_Typedef* LED);

void SetGPO(_GPO* GPO, int ms);
int CheckGPO(_GPO* GPO);
void CheckAllGPIO(int* cant_sleep);
int GetBoardVers();

bool GetButton(void);
bool GetTilt(void);

void SET_CC_CS(uint8_t value);

void MISC_IO_Init(void);
void GPIO_sleep_wake(int sleep);

void Config_EXTI_LFCLK(pFunction callback);
void Config_EXTI_GD0(void);
void Start_EXTI_LFDAT(pFunction callback);
void Stop_EXTI_LFDAT(void);

#ifdef __cplusplus
}
#endif


#endif /* ME_PCB_182_02_PORTS_H_ */
