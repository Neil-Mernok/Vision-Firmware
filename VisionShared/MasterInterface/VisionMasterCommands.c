/*VisionMasterCommands.c
 *Created on: 10 July, 2017
 *Company: Mernok Elektronik
 *Author: F. Hattingh
 */

#include "VisionMasterCommands.h"
#include "VisionMaster.h"
#include "CanInterface.h"
#include "Vision_HAL.h"

/**
 * Send message to the specified Vision device through its specific hardware port.
 * @param reader the dive the message is to be sent to
 * @param data array containing message
 * @param len bytes to send
 * @return
 */
int VCSendMessage(vision_device* reader, uint8_t* data, int len)
{
	int lenout = 0;

	if (len > 0)
	{
		// ---- Send to CAN Bus for module ----
		if (reader->interface == MCAN)
		{
			lenout = VisionCAN_send(reader, data, len);
		}

		// ---- Send to COM port for module ----
		if (reader->interface == COM)
		{
			VisionSendCOM_message(reader->COM_Port, data, len);
		}
	}
	return lenout;
}

/**
 * @brief This function sends a 'save setting' commands to the VISION module.
 * @brief Only applicable PULSE400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @param param name="all">if true, all valid parameters will be sent. Otherwise only changed values are sent
 * @return false if the params are not valid to send
 */
bool VCSaveSetting(vision_device* v, uint16_t setting_adr, uint8_t* d_in, uint8_t len)
{
	uint8_t data[25];
	// sanity check
	if(len>20) len = 20;

	data[0] = 'c';
	data[1] = setting_adr;
	data[2] = setting_adr >> 8;
	memcpy(&data[3], d_in, len);

	//if(len<4) len = 4;

	return VCSendMessage(v, data, len + 3);
}

/**
 * @brief This function sets default activities to the VISION module.
 * @brief Only applicable PULSE400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCSetPulseMantagActivities(vision_device* v)
{
	uint8_t data[] = {'c', 0xFF, 0xFF, 1};

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 4);
}

void VCSetPulse300Activities(vision_device* v)
{
	uint8_t data[] = {'c', 0xFF, 0xFF, 3};

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 4);
}

void VCSetPulse400Activities(vision_device* v)
{
	uint8_t data[] = {'c', 0xFF, 0xFF, 2};

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 4);
}

void VCSetPulse500Activities(vision_device* v)
{
	uint8_t data[] = {'c', 0xFF, 0xFF, 4};

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 4);
}

void VCSetRangerActivities(vision_device* v)
{
	uint8_t data[] = {'c', 0xFF, 0xFF, 5};

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 4);
}

void VCSetGPS100Activities(vision_device* v)
{
	uint8_t data[] = {'c', 0xFF, 0xFF, 6};

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 4);
}

/**
 * @brief This function force user defined data over RF of the of the VISION module.
 * @brief Only applicable PULSE400 and Ranging modules
 * @param vision_device v to whom the message must be sent to
 * @param dest the UID of tag to whom the data must be sent to (0x00000000 broadcast)
 * @param message pointer to the data that must be sent
 * @param len the length of the data to be sent ( MIN 4, MAX 50)
 * @return
 */
void VCSendData(vision_device* v, uint32_t dest, uint8_t* message, uint8_t len)
{
	uint8_t data[60];
	data[0] = 'D';
	data[1] = 0; 			//kind
	data[2] = dest;
	data[3] = dest >> 8;
	data[4] = dest >> 16;
	data[5] = dest >> 24;
	if(len > 50) len = 50;
	memcpy(&data[6], message, len);

	// --- Send message to the specified Vision device ---
	VCSendMessage(v, data, len + 6);
}

/**
 * @brief This function request the status of the VISION module.
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetStatus(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v,"f", 1);
}

//bool VCGetStatusVals(vision_device* v, )
//{
////            foreach (var P in current_TAG.Params.StatusVals)
////            {
////                if (RetrySendReceiveMessage(P.GetStatusMessage()) == false)
////                    break;
////            }
////            return true;
//}

/**
 * @brief This function request the GPS Coordinates of the VISION module.
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetGNSSCoordinates(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "G", 1);
}

/**
 * @brief This function requests the count of the transponder log of the of the VISION module.
 * @brief Only applicable PULSE400 and Ranging modules
 * @param vision_device v to whom the message must be sent to
 * @param type of the tag (refer to TagTypes.h)
 * @param age the maximum allowable age of the tags to be returned
 * @param dist the maximum allowable LF RSSI/distance of the tags to be returned
 * @return
 */
void VCGetTransponderLogCount(vision_device* v, uint8_t type, uint8_t age, uint16_t dist)
{
	uint8_t data[5];

	data[0] = 'k';
	data[1] = dist & 0x00FF ;
	data[2] = dist >> 8;
	data[3] = age;
	data[4] = type;

	// --- Send message to the specified Vision device ---
	VCSendMessage(v, data, 5);
}

/**
 * @brief This function requests the ODO of the of the VISION module.
 * @brief Only applicable PULSE400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetODO(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "o", 1);
}

/**
 * @brief This function requests the transponder log of the of the VISION module.
 * @brief Only applicable PULSE400 and Ranging modules
 * @param vision_device v to whom the message must be sent to
 * @param type of the tag (refer to TagTypes.h)
 * @param age the maximum allowable age of the tags to be returned
 * @param dist the maximum allowable LF RSSI/distance of the tags to be returned
 * @return
 */
void VCGetTransponderLog(vision_device* v, uint8_t type, uint8_t age, uint16_t dist)
{
	uint8_t data[5];

	data[0] = 'p';
	data[1] = dist & 0x00FF ;
	data[2] = dist >> 8;
	data[3] = age;
	data[4] = type;

	// request shortened responses from CAN bus
	if (v->interface == MCAN)
		data[0] = 'P';

	// --- Send message to the specified Vision device ---
	VCSendMessage(v, data, 5);
}

/**
 * @brief This function requests the GNSS Antenna status of the VISION module.
 * @brief Only applicable to GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetGNSSAntennaStatus(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "q", 1);
}

/**
 * @brief This function software resets of the VISION module.
 * @brief Only applicable to Pulse400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCResetModule(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "r", 1);
}

/**
 * @brief This sends the reverse byte and the stopping distance to the VISION module.
 * @brief Only applicable to Pulse400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCDrivingInfo(vision_device* v, uint8_t Reverse, uint8_t Stopping_distance, uint32_t Speed)
{
	// --- Send message to the vision_device ---
	uint8_t data[7];

	data[0] = '@';
	data[1] = Reverse;
	data[2] = Stopping_distance;
	data[3] = Speed;
	data[4] = Speed>>8;
	data[5] = Speed>>16;
	data[6] = Speed>>24;
	VCSendMessage(v, data, 7);
}

/**
 * @brief This function request the setting of the VISION module.
 * @brief Only applicable to Pulse400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @param setting_adr index of the setting (refer to Vision_Parameters.h
 * @return
 */
void VCGetSetting(vision_device* v, uint16_t setting_adr)
{
	uint8_t data[3];

	data[0] = 's';
	data[1] = setting_adr;
	data[2] = setting_adr>>8;

	// --- Send message to the vision_device ---
	VCSendMessage(v, data, 3);
}

/**
 * @brief This function requests the UID of the VISION module.
 * @brief Only applicable to PULSE400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetUID(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "u", 1);
}

/**
 * @brief This function requests the Firmware revision of the VISION module.
 * @brief Only applicable to PULSE400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetFWRev(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "v", 1);
}

/**
 * @brief This function requests the ID string of the VISION module.
 * @brief Only applicable to PULSE400, Ranging and GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCGetWhoAmI(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "w", 1);
}

/**
 * @brief This function resets the transponder log of the VISION module.
 * @brief Only applicable to PULSE400 and Ranging modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCResetTransponderLog(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "x", 1);
}

/**
 * @brief This function resets ODO of the VISION module.
 * @brief Only applicable to GPS100 modules
 * @param vision_device v to whom the message must be sent to
 * @return
 */
void VCResetODO(vision_device* v)
{
	// --- Send message to the vision_device ---
	VCSendMessage(v, "y", 1);
}

