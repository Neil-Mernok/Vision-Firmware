/*
 * uBlox_Tasks.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#include "Tasks.h"
#include "Transponders.h"
#include "Vision_Parameters.h"
#include "GPS_APL.h"
#include "uBlox_Config.h"
#include "uBlox_Poll.h"
#include "usart.h"
//#include "uBlox_General.h"

/**
 * @ the purpose of this task is to wait for messages from the master and answer them when ready.
 * @ it can also accept messages from the code intended for the master and forward them along.
 * @param pvParameters
 */
void uBlox_task(task* t, int* cant_sleep)
{
//	static uint32_t last_Master = 0;
//	static _Q_MasterIF MIF = { };
//	static uint32_t last_Sent = 0;

	if (t->state == 0)			// ---- This is run the first time only ----
	{
		// ---- create a pipe to hold messages we need to forward max 80 messages ----
		pipe_init(&(t->p), 80, sizeof(_Q_MasterIF));

		if (UBX_MON_VER_Poll() == false)
		{
			Vision_Status.sts.Module_UART_working = false;
			t->state = -1;			// ---- indicate that the process cannot run again ----
		}

		uBlox_Init();
		t->state = 1;				// ---- Indicate that the process can run ----
	}

	if (t->state == 1)
	{

		//	---- there is an overrun error! Clear now! ----
		if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) //HAL_UART_ERROR_ORE
		{
			uint8_t temp_info = 0;
			temp_info = (HAL_UART_Receive(&huart1,&temp_info,1,10));
		}

		cant_sleep++;				// ---- Disable sleep for GPS boards ----
	}
}

/**
 * @ the purpose of this sent the initialize messages to the module at start up..
 * @param None
 * @retVal None
 */
void uBlox_Init(void)
{
	Delay(10);
	UBX_Navigation_Config();
	Delay(10);
	UBX_High_Navigation_Rate_Config();
	Delay(10);
//	UBX_Messages_Config(Class_NAV, ID_NAV_PVT);
	UBX_Messages_Config(0x01, 0x07);
	Delay(10);
//	UBX_Messages_Config(Class_HNR, ID_HNR_PVT);
	UBX_Messages_Config(0x28, 0x00);
	Delay(10);
//	UBX_Messages_Config(Class_NAV, ID_NAV_STATUS);
	UBX_Messages_Config(0x01, 0x03);
	Delay(10);
//	UBX_Messages_Config(Class_NAV, ID_NAV_ODO);
	UBX_Messages_Config(0x01, 0x09);
	Delay(10);
	//UBX_Odometer_Config();
	//Delay(10);
//	UBX_Antenna_Config();
	Delay(10);
//	UBX_Messages_Config(Class_MON, ID_MON_HW);
	UBX_Messages_Config(0x0A, 0x09);
	Delay(10);
	UBX_Save_Config();
	Delay(10);
	UBX_Reset_Odometer();
	// ---- Enable UART Receive Interrupt ----
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}
