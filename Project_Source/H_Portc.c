/*
 * H_Ports.c
 * Created on: Sept 12, 2014
 * Company: Mernok Elektronik 
 * Author: H. Kannemeyer
 */

#include "H_Ports.h"

const uint16_t INPUT_PINS[Inputs] = {CHRG_PG_PIN,CHRG_CHRG_PIN, CHRG_DONE_PIN};
GPIO_TypeDef* INPUT_PORTS[Inputs] = {CHRG_PG_PORT, CHRG_CHRG_PORT, CHRG_DONE_PORT};
const uint32_t INPUT_CLKS[Inputs] = {CHRG_PG_CLK ,CHRG_CHRG_CLK, CHRG_DONE_CLK};

//const uint16_t OUTPUT_PINS[Inputs] = {Led_B_PIN, Led_G_PIN, Led_R_PIN, BT_Reset_PIN, BT_Key_PIN,Buzzer_PIN};
//GPIO_TypeDef* OUTPUT_PORTS[Inputs] = {Led_B_PORT, Led_G_PORT, Led_R_PORT, BT_Reset_PORT, BT_Key_PORT,Buzzer_PORT};
//const uint32_t OUTPUT_CLKS[Inputs] = {Led_B_CLK, Led_G_CLK, Led_R_CLK, BT_Reset_CLK, BT_Key_CLK,Buzzer_CLK};

void Init_All_Port(void)
{
	Inputs_Config(CHRG_PG);
	Inputs_Config(CHRG_CHRG);
	Inputs_Config(CHRG_DONE);
}

void Init_Ports(void)
{
	//clear input flags
	Input_Flags.CHRG_PG  = 0;
	Input_Flags.CHRG_CHRG = 0;
	Input_Flags.CHRG_DONE = 0;
}



/**
 * @brief  Configures Detect input pins , no interrupt.
 * @param  Output: Specifies the Output to be configured.
 *   This parameter can be one of following parameters:
 *     @arg  AMB_Detect
 *     @arg  PDS_Detect
 *     @arg  Wifi_Detect
 * @retval None
 */
void Inputs_Config(Inputs_TypeDef Input)
{
	/* Enable the GPIO Clock */
	RCC_APB2PeriphClockCmd(INPUT_CLKS[Input], ENABLE);

	/* Configure the GPIO pin */
	GPIO_InitStructure.GPIO_Pin = INPUT_PINS[Input];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(INPUT_PORTS[Input], &GPIO_InitStructure);
}


/**
 * @brief  Reads the status of a Opto_IN input. Returns true if pin is active
 * @param  Output: Specifies the Output to be read.
 *   This parameter can be one of following parameters:
 * @retval None
 */
bool Read_Input(Inputs_TypeDef Input)
{
	//get the opto input status.. High on pin means not active
	//low on pin means activated.
	if(!GPIO_ReadInputDataBit(INPUT_PORTS[Input], INPUT_PINS[Input]))
		return true;
	else
		return false;
}

void Read_Inputs(void)
{
	//In1
	if(Read_Input(CHRG_PG))
		Input_Flags.CHRG_PG = 1;
	else
		Input_Flags.CHRG_PG = 0;

	//In2
	if(Read_Input(CHRG_CHRG))
		Input_Flags.CHRG_CHRG = 1;
	else
		Input_Flags.CHRG_CHRG = 0;

	//In3
	if(Read_Input(CHRG_DONE))
		Input_Flags.CHRG_DONE= 1;
	else
		Input_Flags.CHRG_DONE= 0;
}

