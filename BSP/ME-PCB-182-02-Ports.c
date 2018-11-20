/*
 * ME-PCB-182-01-PORTS.c
 * Created on: AUG 2014
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#include "Global_Variables.h"
// arrays

#ifdef PCB_182_02
RGB_LED_Typedef LED1 = LED_1_Values;

COM_TypeDef Master_COM = COM_1;

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

	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	
	MISC_IO_Init();
	LEDInit(&LED1);
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
	/* Enable the GPIO Clock */
	//RCC_APB2PeriphClockCmd(LED.port_clk, ENABLE);
	//Assume its already done. 
	/* Configure the GPIO pins */
#ifdef LED_Inv
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = LED->R_pin;
	GPIO_Init(LED->R_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LED->G_pin;
	GPIO_Init(LED->G_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LED->B_pin;
	GPIO_Init(LED->B_port, &GPIO_InitStructure);
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
		LED->R_port->LED_ONBIT = LED->R_pin;
		LED->G_port->LED_OFBIT = LED->G_pin;
		LED->B_port->LED_OFBIT = LED->B_pin;
		break;
	case Green:
		LED->R_port->LED_OFBIT = LED->R_pin;
		LED->G_port->LED_ONBIT = LED->G_pin;
		LED->B_port->LED_OFBIT = LED->B_pin;
		break;
	case Blue:
		LED->R_port->LED_OFBIT = LED->R_pin;
		LED->G_port->LED_OFBIT = LED->G_pin;
		LED->B_port->LED_ONBIT = LED->B_pin;
		break;
	case Violet:
		LED->R_port->LED_ONBIT = LED->R_pin;
		LED->G_port->LED_OFBIT = LED->G_pin;
		LED->B_port->LED_ONBIT = LED->B_pin;
		break;
	case Yellow:
		LED->R_port->LED_ONBIT = LED->R_pin;
		LED->G_port->LED_ONBIT = LED->G_pin;
		LED->B_port->LED_OFBIT = LED->B_pin;
		break;
	case White:
		LED->R_port->LED_ONBIT = LED->R_pin;
		LED->G_port->LED_ONBIT = LED->G_pin;
		LED->B_port->LED_ONBIT = LED->B_pin;
		break;
	case Cyan:
		LED->R_port->LED_OFBIT = LED->R_pin;
		LED->G_port->LED_ONBIT = LED->G_pin;
		LED->B_port->LED_ONBIT = LED->B_pin;
		break;
	case Off:
		LED->R_port->LED_OFBIT = LED->R_pin;
		LED->G_port->LED_OFBIT = LED->G_pin;
		LED->B_port->LED_OFBIT = LED->B_pin;
		break;
	}
	LED->set_to = time_now() + ms;
}

/**
 * checks if the LED has been on for a certain amount of time, then sets the LEd off if expired.
 */
int CheckLed(RGB_LED_Typedef* LED)
{
	if (time_now() >= LED->set_to)
	{
		SetLed(LED, Off, 0);
		return 0;
	}
	else
		return 1;
}

/**
 * checks if the LED has been on for a certain amount of time, then sets the LED off if expired.
 */
void CheckAllGPIO(int* cant_sleep)
{
	(*cant_sleep) += CheckLed(&LED1);
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

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

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

	//	 Configure Pin A9 to pull up for USB VBUS sensing.
	GPIO_InitStructure.GPIO_Pin = USB_VBUS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(USB_VBUS_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(USB_VBUS_PORT, USB_VBUS_PIN);
		
	//	 Configure the External power detect pin
	GPIO_InitStructure.GPIO_Pin = EXT_POWER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(EXT_POWER_PORT, &GPIO_InitStructure);

	//	 Configure the External antenna detect pin
	GPIO_InitStructure.GPIO_Pin = ANT_SENSE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ANT_SENSE_PORT, &GPIO_InitStructure);

	// Configure the CAN terminate pin
	GPIO_InitStructure.GPIO_Pin = CAN_term_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CAN_term_PORT, &GPIO_InitStructure);

	// Configure the CAN RS pin
	GPIO_InitStructure.GPIO_Pin = CAN_DIS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CAN_DIS_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(CAN_DIS_PORT, CAN_DIS_PIN);

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
//	GPIO_InitStructure.GPIO_Pin = CHRG_EN_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(CHRG_EN_PORT, &GPIO_InitStructure);

	// configure SPI CS pins
	GPIO_InitStructure.GPIO_Pin = CC_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CC_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LF_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LF_CS_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = AC_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(AC_CS_PORT, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = DISP_CS_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(DISP_CS_PORT, &GPIO_InitStructure);

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
//	GPIO_SetBits(AC_CS_PORT, AC_CS_PIN);
//	GPIO_SetBits(DISP_CS_PORT, DISP_CS_PIN);
	GetBoardVers();
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

bool GetBoardVers()
{
	bool new;

	//	 Configure the Version detect pin
	GPIO_InitStructure.GPIO_Pin = VersionD_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(VersionD_PORT, &GPIO_InitStructure);

	DelayUs(1);
	new = GPIO_ReadInputDataBit(VersionD_PORT, VersionD_PIN);
	new = !new;
	// disable pullup to save power.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(VersionD_PORT, &GPIO_InitStructure);

	if (new)
		Master_COM = COM_3;
	return new;
}

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
	/* Configure GD0 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = CC_GDO_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(CC_GDO_IRQ_PORT, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI Line to pin */
	GPIO_EXTILineConfig(CC_GDO_IRQ_GPIO_PortSource, CC_GDO_IRQ_GPIO_PinSource);

	/* Configure EXTI line */
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

/**
 * @brief	This is the actual interrupt handler. The defines help to make it dynamic, 
 * 			but we need to ensure that it is a unique isr; i.e. below EXTI5			
 */
void CC_GDO_IRQ_Handler(void)
{
	if (EXTI_GetITStatus(CC_GDO_IRQ_EXTI) != RESET)
	{
		// inform RX/TX task that operation is complete
		packetSent = 1;
		packetReceived = 1;

		/* Clear the  EXTI line pending bit */
		EXTI_ClearITPendingBit(CC_GDO_IRQ_EXTI);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///		EXTI is used for AS3933 LF frequency tuning. 
///		the chip is put in tuning mode, then the interrupt is started. 
///		on the first interrup a timer is started, after that each consectutive interrupt increments
///		a counter until the desired number of pulses have been counted, then the timer and isr are 
///		stopped and the frequency is calculated.
////////////////////////////////////////////////////////////////////////////////////////////////////
/// pointer to the function that the isr must call
pFunction EXTI_LFDAT_CALLBACK = NULL;

void Start_EXTI_LFDAT(pFunction callback)
{
	/* Configure XX pin as input floating */
	GPIO_InitStructure.GPIO_Pin = LF_DAT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(LF_DAT_PORT, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTIX Line to pin */
	GPIO_EXTILineConfig(LF_DAT_IRQ_GPIO_PortSource, LF_DAT_IRQ_GPIO_PinSource);

	/* Configure EXTI3 line */
	EXTI_InitStructure.EXTI_Line = LF_DAT_IRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI3 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = LF_DAT_IRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_LFDAT_CALLBACK = callback;

	/* Enable GPIOA clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//
//	/* Configure PA.00 pin as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	/* Enable AFIO clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//
//	/* Connect EXTI0 Line to PA.00 pin */
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
//
//	/* Configure EXTI0 line */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//
//	/* Enable and set EXTI0 Interrupt to the lowest priority */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	EXTI_LFDAT_CALLBACK = callback;
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

void AS3933_FREQ_ISR(void);

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

void AS3933_CLK_ISR(void);

void LF_CLK_IRQ_Handler(void)
{
	if (EXTI_GetITStatus(LF_CLK_IRQ_EXTI) != RESET)
	{
		if (EXTI_LFCLK_CALLBACK != NULL)
			EXTI_LFCLK_CALLBACK();
		//AS3933_CLK_ISR();
		/* Clear the  EXTI line pending bit */
		EXTI_ClearITPendingBit(LF_CLK_IRQ_EXTI);
	}
}


/**
 * @brief configures the GPIO pins for lowest possible power draw.
 */
void GPIO_sleep_wake(int sleep)
{
	static uint32_t MA, MB, MC, MD, first = 0;

	if (first == 0)
	{
		// disable all port C except used pins. this has an impact!
//		GPIOC->CRH = 0b11001111111111111111111111111111;
//		MA = GPIOA->MODER;
//		MB = GPIOB->MODER;
//		MH = GPIOH->MODER;
		first = 1;
	}
	if (sleep)
	{
		// disable CAN driver
		GPIO_WriteBit(CAN_DIS_PORT, CAN_DIS_PIN, Bit_SET);
		// disable CAN terminate
		GPIO_WriteBit(CAN_term_PORT, CAN_term_PIN, Bit_RESET);
		// led off 
		SetLed(&LED1, Off, 0);
		// power off eeprom
		GPIO_ResetBits(VRFID_PORT, VRFID_PIN);		// power the eeprom off
		// set all pins to analog except:
		// SWD (A13+14), CH_EN(A12), RF_CS(A4)
		////////////////////////////////////////////////////////////////
		//// for MODER register, there are 2 bits per GPIO. 		////
		//// -> 00 = Input Floating 								////
		//// -> 01 = Output 										////
		//// -> 10 = AF 											////
		//// -> 11 = Analog 										////
		////////////////////////////////////////////////////////////////
//		GPIOA->MODER = 0b11101001111011111111110111111111;
//		GPIOA->MODER = 0b11111111111111111111111111111111;
		// LC_CLK (B3), GPIOs(B8-11)
//		GPIOB->MODER = 0b11111111010101011111001100010101;
//		GPIOB->MODER = 0b11111111111111111111111111111111;
		// set GPO's low
//		GPIOB->ODR &= 0xF0FF;
				
	}
	else
	{
		// enable CAN driver
		GPIO_WriteBit(CAN_DIS_PORT, CAN_DIS_PIN, Bit_RESET);
		// power eeprom back on. 
		GPIO_SetBits(VRFID_PORT, VRFID_PIN);		// power the eeprom on
//		GPIOA->MODER = MA;
//		GPIOB->MODER = MB;
//		GPIOC->MODER = MC;
//		GPIOH->MODER = MH;
	}
}





#endif
