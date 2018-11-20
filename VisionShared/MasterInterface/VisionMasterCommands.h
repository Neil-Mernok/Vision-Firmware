/*VisionMasterCommands.h
 *Created on: Nov 24, 2015
 *Company: Mernok Elektronik
 *Author: H. Kannemeyer
*/

#ifndef VISIONMASTERCOMMANDS_H_
#define VISIONMASTERCOMMANDS_H_

//Includes
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "VisionMaster.h"

//Defines

//Variables made public

//Functions made public
int VCSendMessage(vision_device* reader, uint8_t* data, int len);
void VCGetStatus(vision_device* v);
void VCGetTransponderLogCount(vision_device* v, uint8_t type, uint8_t age,  uint16_t dist);
void VCGetTransponderLog(vision_device* v, uint8_t type, uint8_t age,  uint16_t dist);
bool VCSaveSetting(vision_device* v, uint16_t setting_adr, uint8_t* d_in, uint8_t len);
void VCGetSetting(vision_device* v, uint16_t setting_adr);
void VCResetModule(vision_device* v);
void VCDrivingInfo(vision_device* v, uint8_t Reverse, uint8_t Stopping_distance, uint32_t Speed);
void VCGetFWRev(vision_device* v);
void VCSetReaderActivities(vision_device* v);
void VCSetPulse500Activities(vision_device* v);
void VCSetPulse300Activities(vision_device* v);
void VCSetPulse400Activities(vision_device* v);
void VCSetGPS100Activities(vision_device* v);
void VCSetRangerActivities(vision_device* v);
void VCSetPulseMantagActivities(vision_device* v);
void VCSendData(vision_device* v, uint32_t dest, uint8_t* message, uint8_t len);
void VCResetODO(vision_device* v);
void VCResetTransponderLog(vision_device* v);
void VCGetWhoAmI(vision_device* v);
void VCGetUID(vision_device* v);
void VCGetGNSSAntennaStatus(vision_device* v);
void VCGetGNSSCoordinates(vision_device* v);
void VCGetODO(vision_device* v);

#endif /* VISIONMASTERCOMMANDS_H_ */

