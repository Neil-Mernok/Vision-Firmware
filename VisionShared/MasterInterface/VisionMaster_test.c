/*
 * VisionMaster_test.cpp
 *
 *  Created on: Jul 17, 2015
 *      Author: Kobus
 */

#include "VisionMaster.h"

#ifdef VISION_TEST_MASTER
int WR = 0;

// push comms data onto the buffer....
// this would be done by DMA normally...
void MasterTesterPutMessage(uint8_t* data, int len)
{
	while(len--)
	{
		vision_inbuf[WR++] = *data++;
		WR &= VISION_BUF_MASK;
	}
}


inline int Vision_GetWRptr(vision_device* reader)
{
	return WR;
}



#endif
