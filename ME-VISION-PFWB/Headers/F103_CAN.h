/*
 * F103_CAN.h
 *
 *  Created on: Nov 1, 2012
 *      Author: Kobus Goosen
 */

#ifndef F103_CAN_H_
#define F103_CAN_H_

#include "Global_Variables.h"


#define RCC_APB2Periph_GPIO_CAN1    RCC_APB2Periph_GPIOB
#define GPIO_Remapping_CAN1         GPIO_Remap1_CAN1
#define GPIO_CAN1                   GPIOB
#define GPIO_Pin_CAN1_RX            GPIO_Pin_8
#define GPIO_Pin_CAN1_TX            GPIO_Pin_9

#define CANx                       CAN1
#define GPIO_CAN                   GPIO_CAN1
#define GPIO_Remapping_CAN         GPIO_Remapping_CAN1
#define GPIO_CAN                   GPIO_CAN1
#define GPIO_Pin_CAN_RX            GPIO_Pin_CAN1_RX
#define GPIO_Pin_CAN_TX            GPIO_Pin_CAN1_TX

// Global function prototypes //
//----------------------------//
void NVIC_Config(void);
void CAN_Config(uint32_t baudrate);
void Init_RxMes(CanRxMsg *RxMessage);

#endif /* F103_CAN_H_ */
