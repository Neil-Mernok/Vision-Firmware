/*
 * VisionMaster.h
 *
 *  Created on: Jul 17, 2015
 *      Author: Kobus
 */

#ifndef VISIONMASTER_H_
#define VISIONMASTER_H_

#include "MasterTagReader.h"
#include "Vision_Parameters.h"
#include "GPS_APL.h"
#include "Time_APL.h"

// VisionShared folder location
// PROJECT_LOC\..\VisionShared

#ifdef __cplusplus
extern "C" {
#endif

typedef enum Master_source
{
	NONE = 0,
	COM = 1,
	MUSB = 2,
	MCAN = 3,
	CODE = 4,
	RF = 5,
	//GPS = 6
} Master_source;

typedef struct {
	uint32_t Val;
	uint16_t index;
	bool read;
//	bool valid;	
} vision_setting;

typedef struct {
	uint32_t UID;
	uint32_t VID;
	char name[STR_MAX];
	uint32_t last_seen;
	uint32_t status_tracker;
	StatusWord stat;
	uint8_t fw_rev;
	uint8_t fw_subrev;
	int tag_count;
	union activities acts;
	vision_setting read_setting;
	uint8_t SlaveID;
//	vision_setting save_setting;
	int vision_RDptr;							// read pointer for the UART devices
	uint8_t* vision_inbuf;						// buffer location for UART devices. Must be VISION_BUF_LEN bytes long
	LF_message_type own_LF;
	uint8_t	DataIn[Reader_input_buffer_size];	// added CANTag
	int	DataInPtr;								// added CANTag
	uint8_t Settings[112];						// added CANTag
	Master_source interface;
	uint8_t* 	remote_data_RX;					//// Added remote data transmission commands V11.
	uint32_t 	remote_data_last;					// timed flag indicating remote data reception.
	uint8_t  	remote_data_dest;					// destination byte for the remote data received. ('U' = my UID, 'V' = my VID, 'G' = global.)
	uint8_t  	COM_Port;
	_kind 		kind;
	GPS_Data_Type GPS_Data;

	//V14 additions

	uint8_t Reverse;
	uint8_t stopping_dist;
	uint16_t Speed;
	TimeDate_Data_Type DateTime;

} vision_device;

// this is just to enable the testbench. disable this in production code 
//#define VISION_TEST_MASTER
#ifdef VISION_TEST_MASTER
void MasterTesterPutMessage(uint8_t* data, int len);			// this is defined in the mastertest.c file
#endif

#define VISION_BUF_MASK 0x1FFF
#define VISION_BUF_LEN (VISION_BUF_MASK+1)

// global functions
int vision_process(vision_device* reader);									// run the vision state-machine
uint8_t vision_share_message(vision_device* reader_from, vision_device* reader_to, uint8_t data_type);
int Vision_GetWRptr(vision_device* reader);									// external function that returns the buffer write counter.  
int vision_parse_message(vision_device* reader, uint8_t* data, int len);	// interpret the message
void Vision_CAN_packet_handler(uint8_t* data, uint8_t len, bool Last, vision_device* reader);	//

void Vision_CAN_packet_handler(uint8_t* data, uint8_t len, bool Last, vision_device* reader);
vision_device* find_by_CAN_ID(uint8_t CAN_ID);
vision_device* Add_new_Reader(vision_device V);

#ifdef __cplusplus
}
#endif
#endif /* VISIONMASTER_H_ */
