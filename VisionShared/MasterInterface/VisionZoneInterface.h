/*
 * VisionZoneInterface.h
 *
 *  Created on: May 10, 2017
 *      Author: F. Hattingh
 */

#ifndef VISIONZONEINTERFACE_H_
#define VISIONZONEINTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "list.h"
#include "VisionMaster.h"
#include "framing.h"

#ifdef __cplusplus
extern "C" {
#endif

#define num_zone_messages_total 50
#define Send_Zone_Message_Time	1000
#define Remove_Zone_Message_Time 11000

typedef struct _Zone_Message_Type
{
	uint32_t UID;	// UID of the tag in the vehicle's RF Zone
	uint8_t Zone;	// UID of the tag in the vehicle's RF Zone
	uint32_t Distance; //
	uint32_t Heading; //
	uint8_t Age;

	uint32_t Last_Updated;
	uint32_t Last_Sent;

}Zone_Message_Type;

Zone_Message_Type zone_messages_log[num_zone_messages_total];
uint8_t zone_messages_count;

void send_zone_message(vision_device* reader_to, Zone_Message_Type* Zone_Message);
uint8_t get_zone_message_index(uint32_t UID);
void add_zone_message(Zone_Message_Type* Add_Zone_Message);
void reset_zone_message(uint8_t Zone_Message_Index);
uint8_t vision_zone_process(vision_device* reader);
void zone_message_init_log(void);

#ifdef __cplusplus
}
#endif

#endif /* VISIONZONEINTERFACE_H_ */
