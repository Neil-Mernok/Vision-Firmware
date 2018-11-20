/**
 * @file STMport.h
 * @date 2016-sept
 * @author J.L. Goosen
 * @c (C) 2013 Mernok Electronik
 * @brief support for STM32.
 *
 * @note This file contains the port definitions for the STM32F103 uC
 *
 */
#ifndef STMPORT_H
#define STMPORT_H

#include "Delay.h"
#include "spi.h"
#include "ME-PCB-138-03-Ports.h"

#define Set_PON_RST			GPIO_SetBits(N_PON_R_PORT, N_PON_R_PIN)
#define Reset_PON_RST		GPIO_ResetBits(N_PON_R_PORT, N_PON_R_PIN)
#define Set_NN_CS			GPIO_SetBits(NN_CS_PORT, NN_CS_PIN)
#define Reset_NN_CS			GPIO_ResetBits(NN_CS_PORT, NN_CS_PIN)

#define Get_uIRQ			GPIO_ReadInputDataBit(N_uIRQ_PORT, N_uIRQ_PIN)
#define Get_uRST			GPIO_ReadInputDataBit(N_uRESET_PORT, N_uRESET_PIN)

void Config_EXTI_NN_uIRQ(void);
void Config__NN_pins(void);

#endif
