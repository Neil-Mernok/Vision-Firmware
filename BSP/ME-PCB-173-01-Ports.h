/*
 * ME-PCB-173-01-Ports.h
 * Created on: AUG 2014
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */

#ifndef ME_PCB_173_01_H_
#define ME_PCB_173_01_H_
//includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	uint16_t Pin;            
	GPIOMode_TypeDef GPIO_Mode; 
	GPIO_TypeDef* Port;
	uint32_t CLK;
} STM_GPIO;


#define PCB_173_01

//#define USING_SPI1
//#define USING_SPI2
#define USING_SPI3

#define CC_SPI SPI3
#define AS_SPI SPI3
#define LCD_SPI SPI3

#define Master_COM COM_1



//#define LF_TX_Capable
#define LF_RX_Capable

#define use_HMI	
#define GFX	


/*Port map for VISION Mantag board: PCB-173-01, including names
 *

 	 *MAIN SPI pins		 	*************************************************
 *SCK				:	PB3
 *MISO				:	PB4
 *MOSI				:	PB5
 	 *chipselect pins:
 *CC1101_CS			:	PB14
 *AS3933_CS			:	PB12
 *LIS3D_CS			:	PB10
 *DISP_CS			:	PB11															
	*chip interrupt pins
 *CC_GD0			:	PB0
 *AS3933_INT		:	PC2

 	 *LEDs					*************************************************
 *LED_R				:	PA2
 *LED_G				:	PA3
 *LED_B				:	PA4

	*Interrupt pins
 *LF_int			:	PC2
 *LF_clk			:	PC4
 *CC_GD0			:	PB0

	* Analog pins
 *V_IN				:	PA0
 *V_BAT				:	PA1
 
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
	White	= 7
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
} RGB_LED_Typedef;

#define LED_Inv

#ifdef LED_Inv
#define LED_OFBIT 	BSRR
#define LED_ONBIT	BRR
#else
#define LED_ONBIT 	BSRR
#define LED_OFBIT	BRR
#endif


// LED defines		***********************************************************************
/// RGB LEDs
#define LED_1_Values  {GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4, GPIOA, GPIOA, GPIOA, Off, Off}
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

//#define USB_CAN_Sel_PORT				GPIOA
//#define USB_CAN_Sel_PIN					GPIO_Pin_15

//#define Charging_PORT					GPIOA
//#define Charging_PIN					GPIO_Pin_6

//const STM_GPIO chrg = {	GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIOA, RCC_APB2Periph_GPIOA};


//#define CHRG_PG_PIN					GPIO_Pin_15
//#define CHRG_PG_PORT				GPIOA
//#define CHRG_PG_CLK					RCC_APB2Periph_GPIOA
//
//#define CHRG_CHRG_PIN				GPIO_Pin_6
//#define CHRG_CHRG_PORT				GPIOA
//#define CHRG_CHRG_CLK				RCC_APB2Periph_GPIOA
//
//#define CHRG_DONE_PIN				GPIO_Pin_7
//#define CHRG_DONE_PORT				GPIOA
//#define CHRG_DONE_CLK				RCC_APB2Periph_GPIOA


#define USB_Pull_PORT					GPIOA
#define USB_Pull_PIN					GPIO_Pin_8

#define EXT_POWER_PORT					GPIOA
#define EXT_POWER_PIN					GPIO_Pin_15

//#define CAN_term_PORT					GPIOB
//#define CAN_term_PIN					GPIO_Pin_13

//SPI MISO for CC chip SOMI heck
#define CC_MISO_PORT					GPIOB
#define CC_MISO_PIN						GPIO_Pin_4

// SPI CS pins
#define CC_CS_PORT						GPIOB
#define CC_CS_PIN						GPIO_Pin_14

#define LF_CS_PORT						GPIOB
#define LF_CS_PIN						GPIO_Pin_12

#define AC_CS_PORT						GPIOB
#define AC_CS_PIN						GPIO_Pin_10

#define DISP_CS_PORT					GPIOB
#define DISP_CS_PIN						GPIO_Pin_11
#define DISP_DC_PORT					GPIOC
#define DISP_DC_PIN						GPIO_Pin_13
#define DISP_EN_PORT					GPIOC
#define DISP_EN_PIN						GPIO_Pin_15
                                                                             

// output used to supply power to the i2c eeprom.
#define VRFID_PORT						GPIOC
#define VRFID_PIN						GPIO_Pin_7

// output used to enable the charge circuit
#define CHRG_EN_PORT					GPIOC
#define CHRG_EN_PIN						GPIO_Pin_5

// GPIO for critical output (GPIO_7)
#define CRIT_OUT_PORT					GPIOC
#define CRIT_OUT_PIN					GPIO_Pin_11

// GPIO for critical output (GPIO_6)
//#define WARN_OUT_PORT					GPIOC
//#define WARN_OUT_PIN					GPIO_Pin_10

// Interrupt pins
// LF_int			:	PC2
#define LF_IRQ_PIN                  	GPIO_Pin_2
#define LF_IRQ_EXTI						EXTI_Line2
#define LF_IRQ_PORT	               		GPIOC
#define LF_IRQ_GPIO_PortSource			GPIO_PortSourceGPIOC
#define LF_IRQ_GPIO_PinSource			GPIO_PinSource2
#define LF_IRQ_Channel					EXTI2_IRQn
#define LF_IRQ_Handler					EXTI2_IRQHandler

// LF_clk			:	PC4
#define LF_CLK_IRQ_PIN                 	GPIO_Pin_4
#define LF_CLK_IRQ_EXTI					EXTI_Line4
#define LF_CLK_IRQ_PORT	                GPIOC
#define LF_CLK_IRQ_GPIO_PortSource		GPIO_PortSourceGPIOC
#define LF_CLK_IRQ_GPIO_PinSource		GPIO_PinSource4
#define LF_CLK_IRQ_Channel				EXTI4_IRQn
#define LF_CLK_IRQ_Handler				EXTI4_IRQHandler

// CC_GD0			:	PB0
#define CC_GDO_IRQ_PIN                 	GPIO_Pin_0
#define CC_GDO_IRQ_EXTI					EXTI_Line0
#define CC_GDO_IRQ_PORT	                GPIOB
#define CC_GDO_IRQ_GPIO_PortSource		GPIO_PortSourceGPIOB
#define CC_GDO_IRQ_GPIO_PinSource		GPIO_PinSource0
#define CC_GDO_IRQ_Channel				EXTI0_IRQn
#define CC_GDO_IRQ_Handler				EXTI0_IRQHandler

// LF_dat			:	PC3
#define LF_DAT_IRQ_PIN              	GPIO_Pin_3
#define LF_DAT_IRQ_EXTI					EXTI_Line3
#define LF_DAT_IRQ_PORT	            	GPIOC
#define LF_DAT_IRQ_GPIO_PortSource		GPIO_PortSourceGPIOC
#define LF_DAT_IRQ_GPIO_PinSource		GPIO_PinSource3
#define LF_DAT_IRQ_Channel				EXTI3_IRQn
#define LF_DAT_IRQ_Handler				EXTI3_IRQHandler
// this pin is where Manchester data is output. 
#define LF_DAT_PORT						GPIOC
#define LF_DAT_PIN						GPIO_Pin_3


//void Config_EXTI0(void);
//void Config_EXTI2(void);
//void Start_EXTI3(void);
//void Stop_EXTI3(void);
//void Config_EXTI4(void);

//Functions made public
void Config_Ports(void);
void LEDInit(RGB_LED_Typedef* LED);
void SetLed(RGB_LED_Typedef* LED, RGB_State_TypeDef state, int ms);
int CheckLed(RGB_LED_Typedef* LED);
void CheckAllGPIO(int* cant_sleep);
void SET_CC_CS(uint8_t value);

void MISC_IO_Init(void);

void Config_EXTI_LFCLK(pFunction callback);
void Config_EXTI_GD0(void);
void Start_EXTI_LFDAT(pFunction callback);
void Stop_EXTI_LFDAT(void);

#ifdef __cplusplus
}
#endif

#endif /* ME_PCB_173_01_PORTS_H_ */
