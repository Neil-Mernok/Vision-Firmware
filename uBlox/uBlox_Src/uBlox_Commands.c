/*
 * uBlox_Commands.c
 *
 *  Created on: Apr 20, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_Commands.h"
#include "uBlox_Acknowledge.h"
#include "uBlox_Interface.h"
#include "uBlox_General.h"
#include "uBlox_Config.h"

uint8_t UBX_Reset_Odometer(void)
{
	// ---- GPS GLONASS has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-NAV-RESETODO" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_NAV;
	uBlox_TX.uBlox_Structure.ID = ID_NAV_RESETODO;
	uBlox_TX.uBlox_Structure.Length[0] = 0x00 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}
