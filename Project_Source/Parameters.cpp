/*
 * Parameters.c
 *
 *  Created on: May 2015
 *      Author: Kobus Goosen
 */

#include "Global_Variables.h"
#include "Vision_Parameters.h"
//#include "M24LR64.h"

_Vision_Status Vision_Status = {};

uint8_t UID_bytes[8];

int SaveSetting(int index, uint8_t* data, uint8_t len)
{
	uint8_t buff[STR_MAX+1] = {};
	//	uint32_t start = time_now();
	//
	//	while(RFID_Check_FieldOn())			// check if the field is powered, if so, wait a bit. otherwise exit
	//	{
	//		if(time_since(start) > 400)
	//			return 0;
	//	}

	if (vision_settings[index])
	{
		// save the minimum of the sent bytes and the data length through
		len = MIN(len, vision_settings[index]->maxlen());
		// copy the data into the setting space. 
		memcpy(buff, data, len);
		memcpy(vision_settings[index]->ptr(), buff, vision_settings[index]->maxlen());
		vision_settings[index]->checkbounds();
		return RFID_Write_bytes(vision_settings[index]->address, vision_settings[index]->ptr(), vision_settings[index]->maxlen());
	}
	else
		return 0;
}

int SaveSettings(void)
{
	/// write the settings into eeprom.
	for (int i = 0; i < max_params; i++)
	{
		if (vision_settings.list[i] == NULL)
			break;
		vision_settings.list[i]->checkbounds();
		if (RFID_Write_bytes(vision_settings.list[i]->address, vision_settings.list[i]->ptr(), vision_settings.list[i]->maxlen()) == 0)
			return 0;
	}
	if (!MernokAsset_GroupList_populated)
	{
		Get_MernokAsset_GroupValues();
	}
	return 1;
}

int GetSettings(void)
{
	Vision_Status.sts.RFID_Working = FALSE;
	Set_Default_MernokAsset_GroupValues();
	if(RFID_Check_FieldOn())			// check if the field is powered, if so, just carry on...
	{
		//TOdo: Neil, fix this so it actually does something
		// if the field is on, allow RF for testing, etc. 
		//Vision_Status.Force_RF = time_now() + 120000;
		return 0;
	}

	if (RFID_Read_UID(UID_bytes))
	{
		if ((RFID_Read_byte(vision_settings.firmware.address) == Firmware_rev)&&(RFID_Read_byte(vision_settings.firmware_sub.address) == Firmware_subrev))
		{
			/// read in the settings
			for (int i = 0; i < max_params; i++)
			{
				if (vision_settings.list[i] == NULL)
					break;

				if (RFID_Read_bytes(vision_settings.list[i]->address, vision_settings.list[i]->ptr(), vision_settings.list[i]->maxlen()) == 0)
					return 0;
				vision_settings.list[i]->checkbounds();

			}
			Vision_Status.sts.RFID_Working = true;
		}
		else
		{
			vision_settings.slave_id = UID_bytes[0];			/// set the slave ID to a <relatively> unique number, so newly installed devices dont clash. 
			SaveSettings();
			Vision_Status.sts.RFID_Working = true;
		}

		Vision_Status.UID = *((uint32_t*) (UID_bytes));
		return 1;
	}
	return 0;
}


/**
 * This function is called when a USB device is plugged/unplugged. Says that the USB is working, and whether it is connected for Serial port interference.
 * @param in
 */
void USB_plugged(int in)
{
	if (in)
	{
		Vision_Status.sts.USB_Active = 1;
		Vision_Status.sts.USB_Working = true;
	}
	else
	{
		Vision_Status.sts.USB_Active = 0;
	}
}

void DetermineTagType(void)
{
	if ((vision_settings.getActivities().CAN_Heartbeat_monitor)&&((uint32_t)vision_settings.CAN_Timeout._value*1000>time_since(Vision_Status.Last_CAN)))
	{
		Vision_Status.TagTypeHolder = vision_settings.Type_revert;
	}
	else if(!vision_settings.getActivities().CAN_Heartbeat_monitor)
	{
		Vision_Status.TagTypeHolder = vision_settings.tag_type;
	}

	Vision_Status.Group_status  = vision_settings.MernokAsset_Groups[(uint8_t)vision_settings.tag_type-1];
}
