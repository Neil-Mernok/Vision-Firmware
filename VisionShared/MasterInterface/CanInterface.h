/*CanInterface.h
 *Created on: Nov 24, 2015
 *Company: Mernok Elektronik
 *Author: H. Kannemeyer
*/

#ifndef CANINTERFACE_H_
#define CANINTERFACE_H_

//Includes
#include "VisionMaster.h"



// CAN abstraction layer

//Defines
#define CAN_ID_Vision_Poll 0x12227100
#define CAN_ID_Vision_Resp 0x12227200
#define CAN_ID_Vision_Sync 0x12227300
#define CAN_ID_VisionBootResp 0x12227500
#define CAN_ID_VisionBootPoll 0x12227400
#define CAN_ID_Vision_NotLast 0x00000800
//Variables made public

//Functions made public
int VisionCAN_send(vision_device* reader, uint8_t* data, int len);

#endif /* CANINTERFACE_H_ */

