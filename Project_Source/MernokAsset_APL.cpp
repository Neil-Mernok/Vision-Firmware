/*
 * MernokAsset_APL.cpp
 *
 *  Created on: 28 Mar 2018
 *      Author: NeilPretorius
 */


#include "MernokAsset_APL.h"

uint8_t GroupTemp = 0;
uint8_t MernokAsset_GroupDef[255] = {0};
int Get_MernokAsset_GroupValues(void)
{
	for (int i= 0; i<255; i++)
	{
		GroupTemp = RFID_Read_byte((uint16_t)(0x100 + i));
		vision_settings.MernokAsset_Groups[i] = GroupTemp;
	}
	MernokAsset_GroupList_populated = true;
	return 1;
}

int Set_Default_MernokAsset_GroupValues(void)
{
	int DefaultArr[60] =
	{
			9,
			9,
			9,
			5,
			9,
			9,
			8,
			9,
			9,
			9,
			1,
			1,
			1,
			1,
			2,
			2,
			2,
			2,
			3,
			4,
			4,
			6,
			7,
			4,
			4,
			6,
			1,
			1,
			2,
			3,
			3,
			4,
			4,
			5,
			5,
			7,
			7,
			7,
			9,
			9,
			7,
			7,
			7,
			7,
			9,
			9,
			9,
			9,
			9,
			9,
			7,
			9,
			9,
			9,
			9,
			9,
			9,
			7,
			7
	};
	for(int i = 0; i<56;i++)
	{
		vision_settings.MernokAsset_Groups[i] = DefaultArr[i];
	}

	MernokAsset_GroupList_populated = true;
	Vision_Status.MernokAssetFile_Def = 1;
	return 1;
}
