/*
 * ME-PCB-173-02-PORTS.c
 * Created on: MAR 2015
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#include "ME-PCB-173-03-Ports.h"
// arrays

#ifdef PCB_173_03

extern uint8_t packetReceived;
extern uint8_t packetSent;

RGB_LED_Typedef LED1 = LED_1_Values;
#ifdef NOTTIliT
_GPO LED_OUT = LED_OUT_values;
#endif
_GPO LAMP_OUT = LAMP_OUT_values;
_GPO BUZ_OUT = BUZ_OUT_values;
_GPO VIB_OUT = VIB_OUT_values;

pFunction EXTI_LFDAT_CALLBACK = NULL;
pFunction EXTI_LFCLK_CALLBACK = NULL;


/**
 * figure out if its an old or new board...
 * @return
 */
int GetBoardVers(void)
{
	bool new, standalone, gps_reset;
	GPIO_InitTypeDef GPIO_InitStruct
	;
	//	 Configure the Version and mantag detect pins
	GPIO_InitStruct.Pin = VER_detct_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(VER_detct_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BRD_detct_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BRD_detct_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = uBlox_RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(uBlox_RST_PORT, &GPIO_InitStruct);

	HAL_Delay(10);
	
	new = !GPIO_ReadInputDataBit(VER_detct_PORT, VER_detct_PIN);
	standalone = !GPIO_ReadInputDataBit(BRD_detct_PORT, BRD_detct_PIN);
	
	new |= standalone;
	
	gps_reset =  GPIO_ReadInputDataBit(uBlox_RST_PORT, uBlox_RST_PIN);
//	gps_reset = true;

	if ((gps_reset == true) && (new == false))
	{
		new = true;
	}

	// disable pullup to save power.
	GPIO_InitStruct.Pin = VER_detct_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VER_detct_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = BRD_detct_PIN;
	HAL_GPIO_Init(BRD_detct_PORT, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 */
	GPIO_InitStruct.Pin = uBlox_RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(uBlox_RST_PORT, &GPIO_InitStruct);

	if (new)
	{
		LED1.inverted = false;		// the version 4 boards have non-inverted LEDs.
		if (gps_reset && !standalone && new)
			return 3;
		else if(standalone)
			return 2;
		else
			return 1;
	}
	else
		return 0;
}


/**
 * @brief  Configures all ports on LCU board
 * @param  None
 * @retval None
 */
void Config_Ports(void)
{
	/* Disable the Jtag Debug Ports (but not SW) */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
	MISC_IO_Init();
//	LEDInit(LED1);
}

//********* RGB LED  functions ************************************************************************//
//********************************************************************************************************//
/****************************************************************/
/**
 * @brief  Configures a RGB LED
 * @param  LED: the LED to be initialised. this should be defined beforehand
 *         All pins should be on the same port
 * @retval None
 */
/****************************************************************/
void LEDInit(RGB_LED_Typedef* LED)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pins */
	if(LED->inverted)
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	else
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = LED->R_pin;
	HAL_GPIO_Init(LED->R_port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = LED->G_pin;
	HAL_GPIO_Init(LED->G_port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = LED->B_pin;
	HAL_GPIO_Init(LED->B_port, &GPIO_InitStruct);
}

/****************************************************************/
/**
 * @brief  Set colour of an RGB Led.
 * @param  State: Colour to be set
 *   This parameter can be one of following parameters:
 *     @arg   Red
 *     @arg   Green
 *     @arg   Blue
 *     @arg   Violet
 *     @arg   Amber
 *     @arg   White
 *     @arg   Turquoise
 *     @arg   Off
 * @retval None
 */
/****************************************************************/
void SetLed(RGB_LED_Typedef* LED, RGB_State_TypeDef state, int ms)
{
	GPIO_PinState LED_OFBIT, LED_ONBIT;
	GPIO_PinState R, G, B;

	if(LED->inverted)
	{
		LED_OFBIT = GPIO_PIN_SET;
		LED_ONBIT = GPIO_PIN_RESET;
	}
	else
	{
		LED_OFBIT = GPIO_PIN_RESET;
		LED_ONBIT = GPIO_PIN_SET;
	}
		
	LED->last_color = LED->color;
	LED->color = state;
	
	R = LED_OFBIT;
	G = LED_OFBIT;
	B = LED_OFBIT;

	if (state & 0b00000100)
		R = LED_ONBIT;
	if (state & 0b00000010)
		G = LED_ONBIT;
	if (state & 0b00000001)
		B = LED_ONBIT;

	HAL_GPIO_WritePin(LED->R_port, LED->R_pin, R);
	HAL_GPIO_WritePin(LED->G_port, LED->G_pin, G);
	HAL_GPIO_WritePin(LED->B_port, LED->B_pin, B);

	if (ms!=0)
	LED->set_to = HAL_GetTick() + ms;
}

/**
 * checks if the LED has been on for a certain amount of time, then sets the LEd off if expired.
 */
int CheckLed(RGB_LED_Typedef* LED)
{
	if (HAL_GetTick() >= LED->set_to)
	{
		SetLed(LED, Off, 0);
		return 0;
	}
	else return 1;
}

/****************************************************************/
/**
 * @brief  Set GPO to active state for x milli's
 */
/****************************************************************/
void SetGPO(_GPO* GPO, int ms)
{
	if (GPO->active_high)
		HAL_GPIO_WritePin(GPO->Port, GPO->Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPO->Port, GPO->Pin, GPIO_PIN_RESET);
	GPO->set_to = HAL_GetTick() + ms;
}

/**
 * checks if the LED has been on for a certain amount of time, then sets the Led off if expired.
 */
int CheckGPO(_GPO* GPO)
{
	if (HAL_GetTick() >= GPO->set_to)
	{
		if (GPO->active_high)
			HAL_GPIO_WritePin(GPO->Port, GPO->Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPO->Port, GPO->Pin, GPIO_PIN_SET);
		return 0;
	}
	else return 1;
}

/**
 * checks if the LED has been on for a certain amount of time, then sets the LED off if expired.
 */
void CheckAllGPIO(int* cant_sleep)
{
#ifdef NOTTIliT
	*cant_sleep += CheckGPO(&LED_OUT);
#endif
	*cant_sleep += CheckGPO(&LAMP_OUT);
	*cant_sleep += CheckGPO(&VIB_OUT);
	*cant_sleep += CheckGPO(&BUZ_OUT);
	*cant_sleep += CheckLed(&LED1);

	if((*cant_sleep) != 0)
		(*cant_sleep)++;
}

bool GetButton(void)
{
	if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN))
		return false;
	else
		return true;
}

bool GetTilt(void)
{
#ifndef NOTTIliT
	if(HAL_GPIO_ReadPin(TILT_IN_PORT, TILT_IN_PIN))
		return false;
	else
		return true;
#endif
}


//********* MISC IO  functions ***************************************************************************//
//********************************************************************************************************//

/**
 * @brief  Configures normal IO pins.
 * @param  none
 * @retval None
 */
void MISC_IO_Init(void)
{
	int none;

	/// configure port clocks
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
//	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
//
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

//	/* Configure the Nanotron CS output pin */
//	GPIO_InitStructure.GPIO_Pin = NN_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(NN_CS_PORT, &GPIO_InitStructure);
//
//	/* Configure the PON_R output pin */
//	GPIO_InitStructure.GPIO_Pin = N_PON_R_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(N_PON_R_PORT, &GPIO_InitStructure);
//
//	/* Configure the Reset input pin */
//	GPIO_InitStructure.GPIO_Pin = N_uRESET_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(N_uRESET_PORT, &GPIO_InitStructure);
//	
//	/* Configure the uIRQ input pin */
//	GPIO_InitStructure.GPIO_Pin = N_uIRQ_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(N_uIRQ_PORT, &GPIO_InitStructure);

//////////////////////////////////////////////////
//////////////////////////////////////////////////

//	 Configure the External power detect pin
//	GPIO_InitStructure.GPIO_Pin = EXT_POWER_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_Init(EXT_POWER_PORT, &GPIO_InitStructure);
//
//	// Configure the CAN terminate pin
//	GPIO_InitStructure.GPIO_Pin = CAN_term_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(CAN_term_PORT, &GPIO_InitStructure);
//
//	// Configure the CAN RS pin
//	GPIO_InitStructure.GPIO_Pin = CAN_DIS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(CAN_DIS_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(CAN_DIS_PORT, CAN_DIS_PIN);
//	
//	
//	// Configure the LF_Dat pin to receive data frm the LF chip 
//	GPIO_InitStructure.GPIO_Pin = LF_DAT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(LF_DAT_PORT, &GPIO_InitStructure);
//	
//	// Configure the charge detection pin 
//	GPIO_InitStructure.GPIO_Pin = Charging_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_Init(Charging_PORT, &GPIO_InitStructure);
//
//	// configure the RFID EEPROM Power pin 
//	GPIO_InitStructure.GPIO_Pin = VRFID_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(VRFID_PORT, &GPIO_InitStructure);
//
//	// configure the charger enable output pin 
//	GPIO_InitStructure.GPIO_Pin = CHRG_EN_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(CHRG_EN_PORT, &GPIO_InitStructure);
//
//	// configure SPI CS pins
//	GPIO_InitStructure.GPIO_Pin = CC_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(CC_CS_PORT, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = LF_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(LF_CS_PORT, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = AC_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(AC_CS_PORT, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = DISP_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(DISP_CS_PORT, &GPIO_InitStructure);
//	
//	// configure critical output pin
//	GPIO_InitStructure.GPIO_Pin = CRIT_OUT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(CRIT_OUT_PORT, &GPIO_InitStructure);
//	
//	// configure warning output pin
//	GPIO_InitStructure.GPIO_Pin = WARN_OUT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(WARN_OUT_PORT, &GPIO_InitStructure);

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOH_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PH0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//	/*Configure GPIO pins : PB4 */
//	GPIO_InitStruct.Pin = uBlox_RST_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//	HAL_GPIO_Init(uBlox_RST_PORT, &GPIO_InitStruct);

	CheckAllGPIO(&none);					// checks the status of the GPIO and LEDs, before enabling them for the first time. 
#ifdef NOTTIliT
	/*Configure GPIO pins : PB10 PB11 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#else
	/*Configure GPIO pins : PB10 PB11 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#endif
	CheckAllGPIO(&none);					// checks the status of the GPIO and LEDs, before enabling them for the first time. 

	//23-04-2017	Remove
	/*Configure GPIO pin : PB12 */
//	GPIO_InitStruct.Pin = GPIO_PIN_12;
//	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure button pin : PH1 */
	GPIO_InitStruct.Pin = BUTTON_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;			// check this. this works because a handler for portx.2 pin is already defined.  
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
		
	GetBoardVers();

//	/*Configure GPIO pins : PB3 PB4 PB5 */
//	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(LF_CLK_IRQ_Channel, 0xF, 0);
//	HAL_NVIC_EnableIRQ(LF_CLK_IRQ_Channel);
//
//	HAL_NVIC_SetPriority(LF_IRQ_Channel, 0xF, 0);
//	HAL_NVIC_EnableIRQ(LF_IRQ_Channel);
//
//	HAL_NVIC_SetPriority(LF_DAT_IRQ_Channel, 0xF, 0);
//	HAL_NVIC_EnableIRQ(LF_DAT_IRQ_Channel);
//
//	HAL_NVIC_SetPriority(CC_GDO_IRQ_Channel, 0xF, 0);
//	HAL_NVIC_EnableIRQ(CC_GDO_IRQ_Channel);

// assert all CS pins
	GPIO_SetBits(CC_CS_PORT, CC_CS_PIN);
	GPIO_ResetBits(LF_CS_PORT, LF_CS_PIN);				// AC3933 CS pin is inverted; asserts high

//	GPIO_SetBits(AC_CS_PORT, AC_CS_PIN);
//	GPIO_SetBits(DISP_CS_PORT, DISP_CS_PIN);
	
	
}

void GPIO_sleep_wake(int sleep)
{
	static uint32_t MA, MB, first = 0;

	if (first == 0)
	{
		// disable all port C except used pins. this has an impact!
		GPIOC->MODER = 0b11001111111111111111111111111111;
		MA = GPIOA->MODER;
		MB = GPIOB->MODER;
//		MH = GPIOH->MODER;
		first = 1;
	}
	if (sleep)
	{
		// set all pins to analog except:
		// SWD (A13+14), CH_EN(A12), RF_CS(A4)
		////////////////////////////////////////////////////////////////
		//// for MODER register, there are 2 bits per GPIO. 		////
		//// -> 00 = Input Floating 								////
		//// -> 01 = Output 										////
		//// -> 10 = AF 											////
		//// -> 11 = Analog 										////
		////////////////////////////////////////////////////////////////
		GPIOA->MODER = 0b11101001111011111010100111111111;
//		GPIOA->MODER = 0b11111111111111111111111111111111;
		// LC_CLK (B3), GPIOs(B8-11)
		GPIOB->MODER = 0b11111111010101011111001100010101;
//		GPIOB->MODER = 0b11111111111111111111111111111111;
		// set GPO's low
//		GPIOB->ODR &= 0xF0FF;
				
	}
	else
	{
		GPIOA->MODER = MA;
		GPIOB->MODER = MB;
//		GPIOC->MODER = MC;
//		GPIOH->MODER = MH;
	}
}

/*
 * brief: 		sets the SPI bus CS pin for the EEPROM to the value given
 * param value:	either 0 or 1
 */
void SET_CC_CS(uint8_t value)
{
	HAL_GPIO_WritePin(CC_CS_PORT, CC_CS_PIN, value);
}

///*
// * brief: 		sets the SPI bus CS pin for the EEPROM to the value given
// * param value:	either 0 or 1
// */
//void SET_LF_CS(uint8_t value)
//{
//	if (value)
//		CC_CS_PORT->BSRR = CC_CS_PIN;
//	else
//		CC_CS_PORT->BRR = CC_CS_PIN;
//}
//
///*
// * brief: 		sets the SPI bus CS pin for the EEPROM to the value given
// * param value:	either 0 or 1
// */
//void SET_AC_CS(uint8_t value)
//{
//	if (value)
//		CC_CS_PORT->BSRR = CC_CS_PIN;
//	else
//		CC_CS_PORT->BRR = CC_CS_PIN;
//}
//
///*
// * brief: 		sets the SPI bus CS pin for the EEPROM to the value given
// * param value:	either 0 or 1
// */
//void SET_DISP_CS(uint8_t value)
//{
//	if (value)
//		CC_CS_PORT->BSRR = CC_CS_PIN;
//	else
//		CC_CS_PORT->BRR = CC_CS_PIN;
//}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
///
///			Set up and handle pin interrupts 
///
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  EXTI line detection callback
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	////////////////////////////////////////////////////////////////////////////////////////////////////
	///		EXTI 0 is used for CC1101 GD0 pin.
	///		GD0 pins is configured to be high during RX and TX, so falling edge indicated end of packet.
	////////////////////////////////////////////////////////////////////////////////////////////////////
	case CC_GDO_IRQ_PIN:
		// inform RX/TX task that operation is complete
		packetSent = 1;
		packetReceived = 1;
		break;

		////////////////////////////////////////////////////////////////////////////////////////////////////
		///		EXTI is used for AS3933 LF frequency tuning. 
		///		the chip is put in tuning mode, then the interrupt is started. 
		///		on the first interrup a timer is started, after that each consectutive interrupt increments
		///		a counter until the desired number of pulses have been counted, then the timer and isr are 
		///		stopped and the frequency is calculated.
		////////////////////////////////////////////////////////////////////////////////////////////////////
	case LF_DAT_PIN:
		if (EXTI_LFDAT_CALLBACK != NULL)
			EXTI_LFDAT_CALLBACK();
		break;
	case LF_CLK_IRQ_PIN:
		if (EXTI_LFCLK_CALLBACK != NULL)
			EXTI_LFCLK_CALLBACK();
		break;

	default:
		break;
	}
}

///		EXTI 0 is used for CC1101 GD0 pin.
///		GD0 pins is configured to be high during RX and TX, so falling edge indicated end of packet.
void Config_EXTI_GD0(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure LF CLK ISR pin */
	GPIO_InitStruct.Pin = CC_GDO_IRQ_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CC_GDO_IRQ_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(CC_GDO_IRQ_Channel, 0xF, 0);
	HAL_NVIC_EnableIRQ(CC_GDO_IRQ_Channel);
}

/**
 * @brief	This is the actual interrupt handler. The defines help to make it dynamic, 
 * 			but we need to ensure that it is a unique isr; i.e. below EXTI5			
 */
//void CC_GDO_IRQ_Handler(void)
//{
//	if (EXTI_GetITStatus(CC_GDO_IRQ_EXTI) != RESET)
//	{
//		// inform RX/TX task that operation is complete
//		packetSent = 1;
//		packetReceived = 1;
//
//		/* Clear the  EXTI line pending bit */
//		EXTI_ClearITPendingBit(CC_GDO_IRQ_EXTI);
//	}
//}
////////////////////////////////////////////////////////////////////////////////////////////////////
///		EXTI is used for AS3933 LF frequency tuning. 
///		the chip is put in tuning mode, then the interrupt is started. 
///		on the first interrup a timer is started, after that each consectutive interrupt increments
///		a counter until the desired number of pulses have been counted, then the timer and isr are 
///		stopped and the frequency is calculated.
////////////////////////////////////////////////////////////////////////////////////////////////////
/// pointer to the function that the isr must call
void Start_EXTI_LFDAT(pFunction callback)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure LF CLK ISR pin */
	GPIO_InitStruct.Pin = LF_DAT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LF_DAT_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(LF_DAT_IRQ_Channel, 1, 0);
	HAL_NVIC_EnableIRQ(LF_DAT_IRQ_Channel);

	EXTI_LFDAT_CALLBACK = callback;
}

void Stop_EXTI_LFDAT(void)
{
	HAL_NVIC_DisableIRQ(LF_DAT_IRQ_Channel);
	HAL_GPIO_DeInit(LF_DAT_PORT, LF_DAT_PIN);				// this is necessary to properly clear ISR initialisation
	
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure LF CLK ISR pin */
	GPIO_InitStruct.Pin = LF_DAT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LF_DAT_PORT, &GPIO_InitStruct);

//	HAL_NVIC_ClearPendingIRQ(LF_DAT_IRQ_Channel);
	__HAL_GPIO_EXTI_CLEAR_IT(LF_DAT_PIN);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///		EXTI X is used for AS3933 Manchester decoder clk.
///		clk rising edge on valid data.  
/// pointer to the function that the isr must call
//pFunction EXTI_LFCLK_CALLBACK = NULL;

void Config_EXTI_LFCLK(pFunction callback)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure LF CLK ISR pin */
	GPIO_InitStruct.Pin = LF_CLK_IRQ_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LF_CLK_IRQ_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(LF_CLK_IRQ_Channel, 1, 0);
	HAL_NVIC_EnableIRQ(LF_CLK_IRQ_Channel);

	EXTI_LFCLK_CALLBACK = callback;
}

#ifdef lf_freq_timer

#endif

#endif
