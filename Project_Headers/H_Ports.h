/*
 * H_Ports.h
 * Created on: Sept 12, 2014
 * Company: Mernok Elektronik 
 * Author: H. Kannemeyer
 */

#ifndef H_PORTS_H_
#define H_PORTS_H_
//includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

//defines
#define Inputs						3

#define CHRG_PG_PIN					GPIO_Pin_15
#define CHRG_PG_PORT				GPIOA
#define CHRG_PG_CLK					RCC_APB2Periph_GPIOA

#define CHRG_CHRG_PIN				GPIO_Pin_6
#define CHRG_CHRG_PORT				GPIOA
#define CHRG_CHRG_CLK				RCC_APB2Periph_GPIOA

#define CHRG_DONE_PIN				GPIO_Pin_7
#define CHRG_DONE_PORT				GPIOA
#define CHRG_DONE_CLK				RCC_APB2Periph_GPIOA

typedef enum
{
	CHRG_PG = 0,
	CHRG_CHRG = 1,
	CHRG_DONE = 2,
}Inputs_TypeDef;


//Flags defined for the general inputs
typedef struct _Input_Flags
{
	uint8_t CHRG_PG;
	uint8_t CHRG_CHRG;
	uint8_t CHRG_DONE;

} Input_Flags;



//Flags defined for the general inputs
//struct _Warning
//{
//	uint8_t LF;
//
//}Warning;


//#define Outputs						5
//
//#define Led_B_PIN					GPIO_Pin_0
//#define Led_B_PORT					GPIOC
//#define Led_B_CLK					RCC_APB2Periph_GPIOC
//
//#define Led_G_PIN					GPIO_Pin_1
//#define Led_G_PORT					GPIOC
//#define Led_G_CLK					RCC_APB2Periph_GPIOC
//
//#define Led_R_PIN					GPIO_Pin_2
//#define Led_R_PORT					GPIOC
//#define Led_R_CLK					RCC_APB2Periph_GPIOC
//
//#define BT_Key_PIN					GPIO_Pin_8
//#define BT_Key_PORT					GPIOB
//#define BT_Key_CLK					RCC_APB2Periph_GPIOB
//
//#define BT_Reset_PIN				GPIO_Pin_9
//#define BT_Reset_PORT				GPIOB
//#define BT_Reset_CLK				RCC_APB2Periph_GPIOB
//
//#define Buzzer_PIN					GPIO_Pin_11
//#define Buzzer_PORT					GPIOA
//#define Buzzer_CLK					RCC_APB2Periph_GPIOA
//
//typedef enum
//{
//	Led_B	= 0,
//	Led_G	= 1,
//	Led_R	= 2,
//	BT_Reset= 3,
//	BT_Key  = 4,
//	Buzzer	= 5
//
//}Outputs_TypeDef;

typedef enum
{
	ON	= true,
	OFF = false,
}STATE_TypeDef;

//Variables made public

//Functions made public
void Init_All_Port(void);
void Init_Ports(void);
void Inputs_Config(Inputs_TypeDef Input);


//bool Read_Detect(Opto_Ins_TypeDef Output);
//void Detect_Init(Detect_TypeDef Output);
//void LED_ON(Test_LED_TypeDef Output);
//void LED_OFF(Test_LED_TypeDef Output);
//void LED_Init(Test_LED_TypeDef Output);
//bool Read_Opto(Opto_Ins_TypeDef Output);
//bool Read_ISO_Opto(Opto_Ins_TypeDef Output);
//void Opto_IN_Init(Opto_Ins_TypeDef Output);
//void Opto_ISO_IN_Init(Opto_ISO_Ins_TypeDef Output);
//void Opto_IN_Init_Speed(Opto_Ins_TypeDef Output);

//void DRIVE_OUT(Outputs_TypeDef Output, bool STATE);
//void Outputs_config(Outputs_TypeDef Output);

bool Read_Input(Inputs_TypeDef Input);
void Read_Inputs(void);

#ifdef __cplusplus
}
#endif
#endif /* ME_ALSMS_SLAVEUC_PORTS_H_ */
