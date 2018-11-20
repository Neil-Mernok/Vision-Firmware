/*
 * uBlox_TX.c
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_Config.h"
#include "uBlox_Interface.h"
#include "uBlox_General.h"

uint8_t UBX_CLASS_ID_Poll(uint8_t CLASS_Poll, uint8_t ID_Poll)
{
	// ---- Software Reset has no UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_FALSE ;
	// ----------------- "UBX-CLASS-ID" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = CLASS_Poll;
	uBlox_TX.uBlox_Structure.ID = ID_Poll;
	uBlox_TX.uBlox_Structure.Length[0] = 0x00 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_MON_VER_Poll(void)
{
	// ---- Reset UBX-ACK-ACK ----
	uBlox_TX.Acknowledge_State = ID_ACK_NAK;
	// ---- Reset Counter ----
	uBlox_TX.Counter = 0 ;

	while ((uBlox_TX.Acknowledge_State == ID_ACK_NAK) && (uBlox_TX.Counter++ < Max_Config_Retry))
	{
		// ---- Transmit Message via UART ----
		UBX_CLASS_ID_Poll(Class_MON,ID_MON_VER);

		if (!uBlox_TX.Acknowlegde_Required)
		{
			return UBX_TRUE;
		}
	}

	if (uBlox_TX.Acknowledge_State == ID_ACK_NAK)
	{
		return UBX_FALSE;
	}

	return UBX_TRUE;
}

uint8_t UBX_NAV_PVT_Poll(void)
{
	// ---- Software Reset has no UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_FALSE ;
	// ----------------- "UBX-CFG-RST" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_NAV;
	uBlox_TX.uBlox_Structure.ID = ID_NAV_PVT;
	uBlox_TX.uBlox_Structure.Length[0] = 0x00 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_NAV_POSECEF_Poll(void)
{
	// ---- Software Reset has no UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_FALSE ;
	// ----------------- "UBX-CFG-RST" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_NAV;
	uBlox_TX.uBlox_Structure.ID = ID_NAV_POSECEF;
	uBlox_TX.uBlox_Structure.Length[0] = 0x00 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_NAV_ODO_Poll(void)
{
	// ---- Software Reset has no UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_FALSE ;
	// ----------------- "UBX-CFG-RST" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_NAV;
	uBlox_TX.uBlox_Structure.ID = ID_NAV_ODO;
	uBlox_TX.uBlox_Structure.Length[0] = 0x00 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}
