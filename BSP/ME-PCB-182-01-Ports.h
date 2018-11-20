/*
 * ME-PCB-182-01-Ports.h
 * Created on: AUG 2014
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */

#ifndef ME_PCB_182_01_H_
#define ME_PCB_182_01_H_
//includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

// Defines
#define PCB_182_01

//#define USING_SPI1
//#define USING_SPI2
#define USING_SPI3

#define CC_SPI SPI3
#define AS_SPI SPI3
#define LCD_SPI SPI3

#define USING_CAN

#define LF_TX_Capable
#define LF_RX_Capable

//#define use_HMI	

#define Master_COM COM_1


/*Port map for VISION Mantag board: PCB-182-01, including names
 *
 	 *MAIN SPI pins		 	*************************************************
 *SCK				:	PB3*
 *MISO				:	PB4*
 *MOSI				:	PB5*
 	 *chipselect pins:
 *CC1101_CS			:	PC11*
 *AS3933_CS			:	PC10*
 	*chip interrupt pins
 *CC_GD0			:	PB0*
 *AS3933_INT		:	PC0
 *AS3933_CLK		:	PC2

 	 *LEDs					*************************************************
 *LED_R				:	PA3*
 *LED_G				:	PA4*
 *LED_B				:	PA5*

	*Interrupt pins
 *LF_INT			:	PC0*
 *LF_CLK			:	PC2*
 *CC_GD0			:	PB0*

	* Analog pins
 *V_IN				:	PA0*
 *V_BAT				:	PA1*
 
*/

// CAN terminate 

////////////////////////////////////////////////////////////////
// RGB LED Defines
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
	GPIO_TypeDef*	port;
	uint32_t   		port_clk;
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
#define LED_1_Values  {GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIOA, RCC_APB2Periph_GPIOA}
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

#define Charging_PORT					GPIOA
#define Charging_PIN					GPIO_Pin_6

//const STM_GPIO chrg = {	GPIO_Pin_6, GPIO_Mode_IN_FLOATING, GPIOA, RCC_APB2Periph_GPIOA};

#define USB_Pull_PORT					GPIOA
#define USB_Pull_PIN					GPIO_Pin_8

//#define EXT_POWER_PORT					GPIOA
//#define EXT_POWER_PIN					GPIO_Pin_15

//#define CAN_term_PORT					GPIOB
//#define CAN_term_PIN					GPIO_Pin_13

//SPI MISO for CC chip SOMI heck
#define CC_MISO_PORT					GPIOB
#define CC_MISO_PIN						GPIO_Pin_4

// SPI CS pins
#define CC_CS_PORT						GPIOC
#define CC_CS_PIN						GPIO_Pin_11

#define LF_CS_PORT						GPIOC
#define LF_CS_PIN						GPIO_Pin_10

#define AC_CS_PORT						GPIOC				// not connected
#define AC_CS_PIN						GPIO_Pin_3

#define DISP_CS_PORT					GPIOC				// not connected
#define DISP_CS_PIN						GPIO_Pin_4

// output used to supply power to the i2c eeprom.
#define VRFID_PORT						GPIOA
#define VRFID_PIN						GPIO_Pin_15

// this pin is where Manchester data comes in. 
#define LF_DAT_PORT						GPIOC
#define LF_DAT_PIN						GPIO_Pin_1

// output used to enable the charge circuit
#define CHRG_EN_PORT					GPIOC				// unconnected
#define CHRG_EN_PIN						GPIO_Pin_5

// LF_PWM1
#define LF_PWM1_PORT					GPIOC
#define LF_PWM1_PIN						GPIO_Pin_6
// LF_PWM2
#define LF_PWM2_PORT					GPIOC
#define LF_PWM2_PIN						GPIO_Pin_7

// GPIO for critical output
#define CRIT_OUT_PORT					GPIOB
#define CRIT_OUT_PIN					GPIO_Pin_12

// GPIO for warning output
#define WARN_OUT_PORT					GPIOB
#define WARN_OUT_PIN					GPIO_Pin_14

// Interrupt pins
// LF_int			:	PC0				// tihs Clashes with GD0!!!
#define LF_IRQ_PIN                  	GPIO_Pin_0
#define LF_IRQ_EXTI						EXTI_Line0
#define LF_IRQ_PORT	               		GPIOC
#define LF_IRQ_GPIO_PortSource			GPIO_PortSourceGPIOC
#define LF_IRQ_GPIO_PinSource			GPIO_PinSource0
#define LF_IRQ_Channel					EXTI0_IRQn
#define LF_IRQ_Handler					EXTI0_IRQHandler

// LF_dat			:	PC1
#define LF_DAT_IRQ_PIN              	GPIO_Pin_1
#define LF_DAT_IRQ_EXTI					EXTI_Line1
#define LF_DAT_IRQ_PORT	            	GPIOC
#define LF_DAT_IRQ_GPIO_PortSource		GPIO_PortSourceGPIOC
#define LF_DAT_IRQ_GPIO_PinSource		GPIO_PinSource1
#define LF_DAT_IRQ_Channel				EXTI1_IRQn
#define LF_DAT_IRQ_Handler				EXTI1_IRQHandler

// LF_clk			:	PC2
#define LF_CLK_IRQ_PIN                 	GPIO_Pin_2
#define LF_CLK_IRQ_EXTI					EXTI_Line2
#define LF_CLK_IRQ_PORT	                GPIOC
#define LF_CLK_IRQ_GPIO_PortSource		GPIO_PortSourceGPIOC
#define LF_CLK_IRQ_GPIO_PinSource		GPIO_PinSource2
#define LF_CLK_IRQ_Channel				EXTI2_IRQn
#define LF_CLK_IRQ_Handler				EXTI2_IRQHandler

// CC_GD0			:	PB0
#define CC_GDO_IRQ_PIN                 	GPIO_Pin_0
#define CC_GDO_IRQ_EXTI					EXTI_Line0
#define CC_GDO_IRQ_PORT	                GPIOB
#define CC_GDO_IRQ_GPIO_PortSource		GPIO_PortSourceGPIOB
#define CC_GDO_IRQ_GPIO_PinSource		GPIO_PinSource0
#define CC_GDO_IRQ_Channel				EXTI0_IRQn
#define CC_GDO_IRQ_Handler				EXTI0_IRQHandler










//Functions made public
void Config_Ports(void);
void LEDInit(RGB_LED_Typedef LED);
void SetLed(RGB_LED_Typedef* LED, RGB_State_TypeDef state, int ms);
void CheckLed(RGB_LED_Typedef* LED);
void SET_CC_CS(uint8_t value);

void MISC_IO_Init(void);

void Config_EXTI_LFCLK(pFunction callback);
void Config_EXTI_GD0(void);
//void Start_EXTI3(void);
//void Stop_EXTI3(void);
//void Config_EXTI_LFINT(void);
void Start_EXTI_LFDAT(pFunction callback);
void Stop_EXTI_LFDAT(void);

#ifdef __cplusplus
}
#endif
#endif /* ME_PCB_182_01_PORTS_H_ */
