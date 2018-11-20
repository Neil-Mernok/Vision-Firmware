/*
 * PDS_Tags.h
 *
 *  Created on: Nov 12, 2012
 *      Author: J.L. Goosen
 */

#ifndef PDS_TAGS_H_
#define PDS_TAGS_H_

#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	uint32_t 	UID;
	uint8_t		type;
	uint8_t 	status;
	uint8_t 	voltage;
	uint8_t 	User1;
	uint8_t 	RSSI;
	uint8_t 	last_seen;
} _tag;


//------- Global Variables --------//
extern _tag tag_log[256];
extern uint8_t tag_timeout;
extern uint8_t tag_count;
extern uint8_t tag_sense_limit;


//------- Global functions --------//
int add_tag_to_log( _tag tag);
void inc_tag_logs(void);
int get_tag_index(int N);
void clear_tag_log(void);
int check_tag(_tag tag);
_tag parse_packet_into_tag(uint8_t* array, uint8_t RSSI);

#ifdef __cplusplus
}
#endif

#endif /* PDS_TAGS_H_ */
