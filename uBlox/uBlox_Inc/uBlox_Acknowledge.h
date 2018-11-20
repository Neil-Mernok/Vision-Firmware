/*
 * uBlox_COMMA.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef UBLOX_ACKNOWLEDGE_H_
#define UBLOX_Acknowledge_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _Message_Acknowledge_Type
{
	uint8_t Class_ACK;
	uint8_t ID_ACK;
	uint8_t State_ACK;
} Message_Acknowledge_Type;

typedef struct _Message_Acknowledge_List_Type
{
	Message_Acknowledge_Type Message_Acknowledge[10];
	uint8_t Index;
} Message_Acknowledge_List_Type;

Message_Acknowledge_List_Type Message_Acknowledge_List;

uint8_t Add_ACK_Message(uint8_t ACK_Class, uint8_t ACK_ID, uint8_t ACK_State);
uint8_t Reset_ACK_Message(uint8_t ACK_Index);
void Reset_ACK_Message_List(void);
uint8_t Find_ACK_Message(uint8_t ACK_Class, uint8_t ACK_ID);

#ifdef __cplusplus
}
#endif

#endif /* UBLOX_CONFIG_H_ */

