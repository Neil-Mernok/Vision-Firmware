/*
 * RF_APL.c
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

//#include "RF_APL.h"
#include "Global_Variables.h"

uint8_t RF_Buffer[64];

///////////////////////////////////////////////////////////////////////////////////////////
////////////		CC1101 based functions		///////////////////////////////////////////

void Apl_Parse_message(uint8_t* buffer, int len, uint8_t RSSI)
{
	_Q_MasterIF MIF;
	
	/// Pass directly to the boot process.
	if(RSSI < 120 && len < 10)			// this is a instruction message, but its too far way
	{
		
	}
	else
	{
		MIF.len = len;
		MIF.Master = RF;
		MIF.data = buffer;

		parse_message(MIF);
	}
}
