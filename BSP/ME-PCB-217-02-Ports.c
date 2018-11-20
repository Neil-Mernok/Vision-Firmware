/*
 * ME-PCB-217-01-PORTS.c
 * Created on: AUG 2014
 * Company: Mernok Elektronik 
 * Author: F. Hattingh
 */
#include "ME-PCB-217-02-Ports.h"
#include "can.h"
#include "usart.h"
#include "Delay.h"
// arrays

#ifdef PCB_217_02

// external ref to the Nanotron IRQ handler
void N_uIRQ_Handler(void);

extern uint8_t packetReceived;
extern uint8_t packetSent;

RGB_LED_Typedef LED1 = LED_1_Values;

pFunction EXTI_LFDAT_CALLBACK = NULL;
pFunction EXTI_LFCLK_CALLBACK = NULL;

/**
 * @brief  Configures all ports on LCU board
 * @param  None
 * @retval None
 */
void Config_Ports(void)
{
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
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pins */
	if (LED->inverted)
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	else
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
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

	if (LED->inverted)
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

	if (ms != 0)
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
	else
		return 1;
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
	else
		return 1;
}

/**
 * checks if the LED has been on for a certain amount of time, then sets the LED off if expired.
 */
void CheckAllGPIO(int* cant_sleep)
{
//	*cant_sleep += CheckGPO(&LED_OUT);
//	*cant_sleep += CheckGPO(&LAMP_OUT);
//	*cant_sleep += CheckGPO(&VIB_OUT);
//	*cant_sleep += CheckGPO(&BUZ_OUT);
	*cant_sleep += CheckLed(&LED1);
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
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin : PD2Pin */
	GPIO_InitStruct.Pin = CTS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PC12Pin PC5Pin PC4Pin */
	GPIO_InitStruct.Pin = RTS_Pin | IN1_Pin | IN0_Pin ;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC15Pin PC14Pin PC10Pin PC9Pin PC8Pin */
	GPIO_InitStruct.Pin = uBlox_OUT_PIN | uBlox_RST_PIN | CAN_DIS_PIN | OUT3_Pin | OUT2_Pin ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC14Pin*/
	GPIO_InitStruct.Pin = uBlox_RST_PIN ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1Pin */
	GPIO_InitStruct.Pin = uBlox_TP_IRQ_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0Pin */
	GPIO_InitStruct.Pin = Charging_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB5Pin */
	GPIO_InitStruct.Pin = CAN_term_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB4Pin PB3Pin */
	GPIO_InitStruct.Pin = VersionD_PIN | VersionD2_PIN ;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1Pin PB0Pin */
	GPIO_InitStruct.Pin = IN3_Pin | IN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA15Pin PA10Pin PA4Pin */
	GPIO_InitStruct.Pin = VRFID_PIN | CS_EXT1_Pin | CS_EXT2_Pin ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA9Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void GPIO_sleep_wake(int sleep)
{
	static uint32_t MA, MB, MC, MD, first = 0;
//	GPIO_InitTypeDef GPIO_InitStruct;
	
	if (first == 0)
	{
		// disable all port C except used pins. this has an impact!
		MA = GPIOA->MODER;
		MB = GPIOB->MODER;
		MC = GPIOC->MODER;
		MD = GPIOD->MODER;
		
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
		//				 |F|E|D|C|B|A|9|8|7|6|5|4|3|2|1|0
		// SPI1 (A5-7), CS_EXT1(A10), CS_EXT2(A4), SWDIO(A13), SWCLK(A14), USB(A11-12)
		GPIOA->MODER = 0b11101010100111111010100111111111;
//		GPIOA->MODER = 0b11101011111111111111111111111111;
		//SPI2(B13-15), UART(B10-11)[RX needs to be set to input for EXTI to work. TX is 11 by default??]
		GPIOB->MODER = 0b10101011001111111111111111111111;
//		GPIOB->MODER = 0b11111111111111111111111111111111;
		// LF_CLK(C1), RF_CS(C3) outputs(C6-9), CAN_lowP(C10), LF_CS(C15)   
		GPIOC->MODER = 0b01111111110101010101111101110011;
//		GPIOC->MODER = 0b01111111111111111111111111111111;
		
		GPIOD->MODER = 0b11111111111111111111111111111111;
		
		// set CAN_DISABLE pin to disable the CAN transceiver. 
		GPIO_SetBits(CAN_DIS_PORT, CAN_DIS_PIN);
	}
	else
	{
		GPIOA->MODER = MA;
		GPIOB->MODER = MB;
		GPIOC->MODER = MC;
		GPIOD->MODER = MD;
//		GPIOH->MODER = MH;
		
		// reset CAN_DISABLE pin to enable the CAN transceiver. 
		GPIO_ResetBits(CAN_DIS_PORT, CAN_DIS_PIN);
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

int board_version = 0;

int GetBoardVers()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//	 Configure the Version detect pin
	GPIO_InitStruct.Pin = VersionD_PIN | VersionD2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(VersionD_PORT, &GPIO_InitStruct);
	
	DelayUs(1);
	if((HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_SET))
		board_version = 1;								// ME-PCB-182-04/5
	else if((HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_SET))
		board_version = 2;								// ME-PCB-138-03
	else if((HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_RESET))
		board_version = 3;								// ME-PCB-217-01
	
	// disable pull-ups to save power.
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VersionD_PORT, &GPIO_InitStruct);

	if (board_version == 0)
		Master_COM = &huart1;
	return board_version;
}

int GetBoardRevision()
{
	int board_revision = 0;

	GPIO_InitTypeDef GPIO_InitStruct;

	// ---- Configure the Version detect pin ----
	GPIO_InitStruct.Pin = VersionD_PIN | VersionD2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(VersionD_PORT, &GPIO_InitStruct);

	DelayUs(1);

	if		((HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_RESET))
		board_revision = 0;		// ME-PCB-217-01
	else if ((HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_SET))
		board_revision = 1;		// ME-PCB-217-02
	else if ((HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_RESET))
		board_revision = 2;		// ME-PCB-217-03
	else if ((HAL_GPIO_ReadPin(VersionD_PORT, VersionD_PIN) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin(VersionD2_PORT, VersionD2_PIN) == GPIO_PIN_SET))
		board_revision = 3;		// ME-PCB-217-04

	// ---- Disable pull-ups to save power ----
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VersionD_PORT, &GPIO_InitStruct);

	return board_revision;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
///
///			Set up and handle pin interrupts 
///
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t* last_master_coms_flag = NULL;
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
		
		////////////////////////////////////////////////////////////////////////////////////////////////////
		///		PA3 is also used for nanotron IRQ. 
		/// 	board revision is used to check if we need to access nanotron code.
		//////////////////////////////////////////////////////////////////////////////////////////////////// 
#ifdef RANGING_CAPABLE
		if(board_version == 2)	// ranger board
			N_uIRQ_Handler();
#endif		
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
	case RX_Pin:
	case CAN_RX_Pin:
		if(last_master_coms_flag != NULL)
			*last_master_coms_flag = time_now();		// make an indication that we have received master coms during sleep.
		break;
		// USB VBUS Detect...	
//	case GPIO_PIN_9:
//		HAL_PCDEx_BCD_VBUSDetect (&hpcd_USB_OTG_FS);
//		break;

	default:
		break;
	}
}

void setup_coms_flag (uint32_t* flag)
{
	last_master_coms_flag = flag;
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
	HAL_NVIC_SetPriority(CC_GDO_IRQ_Channel, 0x1, 0);
	HAL_NVIC_EnableIRQ(CC_GDO_IRQ_Channel);
}

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
	HAL_NVIC_SetPriority(LF_DAT_IRQ_Channel, 0, 0);
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

#endif
