/*CanInterface.c
 *Created on: Nov 24, 2015
 *Company: Mernok Elektronik
 *Author: H. Kannemeyer
*/

#include "CanInterface.h"
#include "Vision_HAL.h"

/****************************************************************/
/**
 * @brief  
 * @param  
 *   This parameter can be one of following parameters:
 *     @arg 
 * @retval None
 */
/****************************************************************/
/***********************************************/
//Local Variables

//Global Variables



/**
 * @brief  This function breaks up the data into messages that is sent over the CAN
 *
 * @param  None
 * @retval None
 */
int VisionCAN_send(vision_device* reader, uint8_t* data, int len)
{
//	uint32_t CANID = 0;
	int bytes_to_send = len, bytes_sent = 0, outlen = 0;

	while(bytes_to_send > 0)
	{
		if (bytes_to_send > 8)
		{
			VisionSendCAN_message(CAN_ID_Vision_Poll | CAN_ID_Vision_NotLast | reader->SlaveID, data, 8);
			bytes_sent = 8;
		}
		else
		{
			VisionSendCAN_message(CAN_ID_Vision_Poll | reader->SlaveID, data, bytes_to_send);
			bytes_sent = bytes_to_send;
		}

		bytes_to_send -= bytes_sent;
		data += bytes_sent;
		outlen += bytes_sent;
	}
    return outlen;
}
