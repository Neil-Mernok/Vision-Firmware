/*
 * ME-PCB-182-01-PORTS.c
 * Created on: AUG 2014
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#include "Global_Variables.h"
// arrays

#ifdef PCB_182_01
RGB_LED_Typedef LED1 = LED_1_Values;

/**
 * @brief  Configures all ports on LCU board
 * @param  None
 * @retval None
 */
void Config_Ports(void)
{
	/* Disable the Jtag Debug Ports (but not SW) */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

	LEDInit(LED1);
	MISC_IO_Init();
	//Config_EXTI_GD0();
//	Config_EXTI_LFCLK();
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
void LEDInit(RGB_LED_Typedef LED)
{
	/* Enable the GPIO Clock */
	RCC_APB2PeriphClockCmd(LED.port_clk, ENABLE);

	/* Configure the GPIO pin */
	GPIO_InitStructure.GPIO_Pin = LED.R_pin | LED.G_pin | LED.B_pin;
#ifdef LED_Inv
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED.port, &GPIO_InitStructure);
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
	LED->last_color = LED->color;
	LED->color = state;
	switch (state & 0b00000111)
	{
	case Red:
		LED->port->LED_ONBIT = LED->R_pin;
		LED->port->LED_OFBIT = LED->G_pin;
		LED->port->LED_OFBIT = LED->B_pin;
		break;
	case Green:
		LED->port->LED_OFBIT = LED->R_pin;
		LED->port->LED_ONBIT = LED->G_pin;
		LED->port->LED_OFBIT = LED->B_pin;
		break;
	case Blue:
		LED->port->LED_OFBIT = LED->R_pin;
		LED->port->LED_OFBIT = LED->G_pin;
		LED->port->LED_ONBIT = LED->B_pin;
		break;
	case Violet:
		LED->port->LED_ONBIT = LED->R_pin;
		LED->port->LED_OFBIT = LED->G_pin;
		LED->port->LED_ONBIT = LED->B_pin;
		break;
	case Yellow:
		LED->port->LED_ONBIT = LED->R_pin;
		LED->port->LED_ONBIT = LED->G_pin;
		LED->port->LED_OFBIT = LED->B_pin;
		break;
	case White:
		LED->port->LED_ONBIT = LED->R_pin;
		LED->port->LED_ONBIT = LED->G_pin;
		LED->port->LED_ONBIT = LED->B_pin;
		break;
	case Cyan:
		LED->port->LED_OFBIT = LED->R_pin;
		LED->port->LED_ONBIT = LED->G_pin;
		LED->port->LED_ONBIT = LED->B_pin;
		break;
	case Off:
		LED->port->LED_OFBIT = LED->R_pin;
		LED->port->LED_OFBIT = LED->G_pin;
		LED->port->LED_OFBIT = LED->B_pin;
		break;
	}
	LED->set_to = time_now()+ms;
}



/**
 * checks if the LED has been on for a certain amount of time, then sets the LEd off if expired.
 */
void CheckLed(RGB_LED_Typedef* LED)
{
	if (time_now() > LED->set_to)
		SetLed(LED, Off, 0);
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
	/// configure port clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

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

	// Configure the USB or CAN selection pin
//	GPIO_InitStructure.GPIO_Pin = USB_CAN_Sel_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	GPIO_Init(USB_CAN_Sel_PORT, &GPIO_InitStructure);
//	DelayUs(1);
//	if (GPIO_ReadInputDataBit(USB_CAN_Sel_PORT, USB_CAN_Sel_PIN ))

//	else
//		Vision_Status.USB_Active = false;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(USB_CAN_Sel_PORT, &GPIO_InitStructure);

	// Configure the External power detect pin
//	GPIO_InitStructure.GPIO_Pin = EXT_POWER_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_Init(EXT_POWER_PORT, &GPIO_InitStructure);

	// Configure the CAN terminate pin
//	GPIO_InitStructure.GPIO_Pin = CAN_term_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	GPIO_Init(CAN_term_PORT, &GPIO_InitStructure);

	// Configure the LF_Dat pin to receive data frm the LF chip 
	GPIO_InitStructure.GPIO_Pin = LF_DAT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LF_DAT_PORT, &GPIO_InitStructure);
	
	// Configure the charge detection pin 
	GPIO_InitStructure.GPIO_Pin = Charging_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(Charging_PORT, &GPIO_InitStructure);

	// configure the RFID EEPROM Power pin 
	GPIO_InitStructure.GPIO_Pin = VRFID_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(VRFID_PORT, &GPIO_InitStructure);

	// configure the charger enable output pin 
	GPIO_InitStructure.GPIO_Pin = CHRG_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CHRG_EN_PORT, &GPIO_InitStructure);

	// configure SPI CS pins
	GPIO_InitStructure.GPIO_Pin = CC_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CC_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LF_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LF_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AC_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(AC_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DISP_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DISP_CS_PORT, &GPIO_InitStructure);
	
	// configure critical output pin
	GPIO_InitStructure.GPIO_Pin = CRIT_OUT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CRIT_OUT_PORT, &GPIO_InitStructure);
	
	// configure warning output pin
	GPIO_InitStructure.GPIO_Pin = WARN_OUT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(WARN_OUT_PORT, &GPIO_InitStructure);
		
	// assert all CS pins
	GPIO_SetBits(CC_CS_PORT, CC_CS_PIN);
	GPIO_ResetBits(LF_CS_PORT, LF_CS_PIN);				// AC3933 CS pin is inverted; asserts high
	GPIO_SetBits(AC_CS_PORT, AC_CS_PIN);
	GPIO_SetBits(DISP_CS_PORT, DISP_CS_PIN);
}

/*
 * brief: 		sets the SPI bus CS pin for the EEPROM to the value given
 * param value:	either 0 or 1
 */
void SET_CC_CS(uint8_t value)
{
	if (value)
		CC_CS_PORT->BSRR = CC_CS_PIN;
	else
		CC_CS_PORT->BRR = CC_CS_PIN;
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

///		EXTI 0 is used for CC1101 GD0 pin.
///		GD0 pins is configured to be high during RX and TX, so falling edge indicated end of packet.
void Config_EXTI_GD0(void)
{
	/* Configure PB.00 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = CC_GDO_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI0 Line to PB.00 pin */
	GPIO_EXTILineConfig(CC_GDO_IRQ_GPIO_PortSource, CC_GDO_IRQ_GPIO_PinSource);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = CC_GDO_IRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = CC_GDO_IRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void CC_GDO_IRQ_Handler(void)
{
	if (EXTI_GetITStatus(CC_GDO_IRQ_EXTI) != RESET)
	{
		// inform RX/TX task that operation is complete
		packetSent = 1;
		packetReceived = 1;
				
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(CC_GDO_IRQ_EXTI);		
	}
}


     
////////////////////////////////////////////////////////////////////////////////////////////////////
///		EXTI is used for AS3933 LF frequency tuning. 
///		the chip is put in tuning mode, then the interrupt is started. 
///		on the first interrup a timer is started, after that each consectutive interrupt increments
///		a counter until the desired number of pulses have been counted, then the timer and isr are 
///		stopped and the frequency is calculated.
/// pointer to the function that the isr must call
pFunction EXTI_LFDAT_CALLBACK = NULL;

void Start_EXTI_LFDAT(pFunction callback)
{
//	/* Configure XX pin as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTIX Line to XX pin */
	GPIO_EXTILineConfig(LF_DAT_IRQ_GPIO_PortSource, LF_DAT_IRQ_GPIO_PinSource);

	/* Configure EXTI3 line */
	EXTI_InitStructure.EXTI_Line = LF_DAT_IRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI3 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = LF_DAT_IRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_LFDAT_CALLBACK = callback;

}

void Stop_EXTI_LFDAT(void)
{
	/* Enable and set EXTI3 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = LF_DAT_IRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void LF_DAT_IRQ_Handler(void)
{
	if (EXTI_GetITStatus(LF_DAT_IRQ_EXTI) != RESET)
	{
		/* Clear the  EXTI line pending bit */
		EXTI_ClearITPendingBit(LF_DAT_IRQ_EXTI);

		if (EXTI_LFDAT_CALLBACK != NULL)
			EXTI_LFDAT_CALLBACK();
		//AS3933_FREQ_ISR();
	}
}



////////////////////////////////////////////////////////////////////////////////////////////////////
///		EXTI X is used for AS3933 Manchester decoder clk.
///		clk rising edge on valid data.  
/// pointer to the function that the isr must call
pFunction EXTI_LFCLK_CALLBACK = NULL;

void Config_EXTI_LFCLK(pFunction callback)
{
	/* Configure pin as input floating */
	GPIO_InitStructure.GPIO_Pin = LF_CLK_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LF_CLK_IRQ_PORT, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI Line to pin */
	GPIO_EXTILineConfig(LF_CLK_IRQ_GPIO_PortSource, LF_CLK_IRQ_GPIO_PinSource);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = LF_CLK_IRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = LF_CLK_IRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//NVIC_DisableIRQ(LF_CLK_IRQ_Channel);

	EXTI_LFCLK_CALLBACK = callback;
}



void LF_CLK_IRQ_Handler(void)
{
	if (EXTI_GetITStatus(LF_CLK_IRQ_EXTI) != RESET)
	{
		AS3933_CLK_ISR();
		/* Clear the  EXTI line pending bit */
		EXTI_ClearITPendingBit(LF_CLK_IRQ_EXTI);
	}
}


#endif
