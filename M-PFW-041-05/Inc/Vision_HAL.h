/*
 * Vision_HAL.h
 *
 *  Created on: Jul 27, 2015
 *      Author: Kobus
 */

#ifndef VISION_HAL_H_
#define VISION_HAL_H_

#ifndef MAX
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

#define LF_TX_Capable

//#define FILTER_LF				// experimental
#define USE_TAG_NAME			// consumes 20 extra bytes per tag. 

#define num_transp_total 256
#define Reader_input_buffer_size 32

// define how long a microsecond is
#define UsBase	48


////////////////////////////////////////////////////////////
// CAN port interface
// this function gets called by Vision Libs to send a can message out.
void APP_CAN_Vision_CAN_TX(uint32_t ID, uint8_t data[], uint8_t packet_length);
#define VisionSendCAN_message(ID, data, len) APP_CAN_Vision_CAN_TX(ID, data, len)
////////////////////////////////////////////////////////////


#endif /* VISION_HAL_H_ */
