/*
 * uBlox_Acknowledge.c
 *
 *  Created on: Apr 21, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_Acknowledge.h"
#include "uBlox_General.h"
/**
 * @brief  Add acknowledge message received from uBlox module to List
 * @param  ACK_Class - reference to Class
 * @param  ACK_ID - reference to ID
 * @param  ACK_State - reference to acknowledge state
 * @retval Successfully executed or failed
 */
uint8_t Add_ACK_Message(uint8_t ACK_Class, uint8_t ACK_ID, uint8_t ACK_State)
{
	Message_Acknowledge_List.Message_Acknowledge[Message_Acknowledge_List.Index].Class_ACK = ACK_Class;
	Message_Acknowledge_List.Message_Acknowledge[Message_Acknowledge_List.Index].ID_ACK = ACK_ID;
	Message_Acknowledge_List.Message_Acknowledge[Message_Acknowledge_List.Index++].State_ACK = ACK_State;
	return UBX_PASS;
}

/**
 * @brief  Reset acknowledge message received from uBlox module to List
 * @param  ACK_Index - reference to index to reset
 * @retval Successfully executed or failed
 */
uint8_t Reset_ACK_Message(uint8_t ACK_Index)
{
	for (int ACK_List_Index = ACK_Index; ACK_List_Index < 9; ACK_List_Index++)
	{
		Message_Acknowledge_List.Message_Acknowledge[ACK_Index].Class_ACK = Message_Acknowledge_List.Message_Acknowledge[ACK_Index + 1].Class_ACK;
		Message_Acknowledge_List.Message_Acknowledge[ACK_Index].ID_ACK = Message_Acknowledge_List.Message_Acknowledge[ACK_Index + 1].ID_ACK;
		Message_Acknowledge_List.Message_Acknowledge[ACK_Index].State_ACK = Message_Acknowledge_List.Message_Acknowledge[ACK_Index + 1].State_ACK;
	}
	Message_Acknowledge_List.Message_Acknowledge[10].Class_ACK = 0xFF;
	Message_Acknowledge_List.Message_Acknowledge[10].ID_ACK = 0xFF;
	Message_Acknowledge_List.Message_Acknowledge[10].State_ACK = 0xFF;
	return UBX_PASS;
}

/**
 * @brief  Reset acknowledge message List
 * @retval None
 */
void Reset_ACK_Message_List(void)
{
	Message_Acknowledge_List.Index = 0;
	for (int ACK_List_Index = 0; ACK_List_Index < 10; ACK_List_Index++)
	{
		Message_Acknowledge_List.Message_Acknowledge[ACK_List_Index].Class_ACK = 0xFF;
		Message_Acknowledge_List.Message_Acknowledge[ACK_List_Index].ID_ACK = 0xFF;
		Message_Acknowledge_List.Message_Acknowledge[ACK_List_Index].State_ACK = 0xFF;
	}
}

/**
 * @brief  Find acknowledge message received from uBlox module in List
 * @param  ACK_Class - reference to Class
 * @param  ACK_ID - reference to ID
 * @retval Index of message if acknowledged
 */
uint8_t Find_ACK_Message(uint8_t ACK_Class, uint8_t ACK_ID)
{
	for (int ACK_List_Index = 0; ACK_List_Index < Message_Acknowledge_List.Index; ACK_List_Index++)
	{
		if ((Message_Acknowledge_List.Message_Acknowledge[ACK_List_Index].Class_ACK == ACK_Class) && (Message_Acknowledge_List.Message_Acknowledge[ACK_List_Index].ID_ACK == ACK_ID))
		{
			if (Message_Acknowledge_List.Message_Acknowledge[ACK_List_Index].Class_ACK == ID_ACK_ACK)
			{
				Reset_ACK_Message(ACK_List_Index);
				return ACK_List_Index;
			}
		}
	}
	return 0xFF;
}
