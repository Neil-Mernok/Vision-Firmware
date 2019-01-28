/*
 * VisionTimeInterface.h
 *
 *  Created on: 28 Jan 2019
 *      Author: NeilPretorius
 */

#ifndef MASTERINTERFACE_VISIONTIMEINTERFACE_H_
#define MASTERINTERFACE_VISIONTIMEINTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "list.h"
#include "VisionMaster.h"
#include "framing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _Time_Message_Type
{
	uint32_t UID;	// UID of the tag in the vehicle's RF Zone
	uint8_t seconds;	// UID of the tag in the vehicle's RF Zone
	uint8_t minutes;
	uint8_t hours;
	uint8_t days;
	uint8_t month;
	uint8_t year;

	uint32_t Last_Updated;
	uint32_t Last_Sent;

}Time_Message_Type;


void send_time_message(vision_device* reader_to, Time_Message_Type* Time_Message);

#ifdef __cplusplus
}
#endif

#endif /* MASTERINTERFACE_VISIONTIMEINTERFACE_H_ */
