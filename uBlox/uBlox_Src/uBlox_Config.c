/*
 * uBlox_Config.c
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_Config.h"
#include "uBlox_Interface.h"
#include "uBlox_General.h"
#include "uBlox_Acknowledge.h"
#include "Delay.h"
#include "Vision_Parameters.h"

/**
 * @brief  Send UBX message over UART
 * @param  Last updated value
 * @retval Size of the uBlox message including the checksum size
 *
 */

uint8_t UBX_Transmit(void)
{
	Reset_ACK_Message_List();
	// ---- Reset UBX-ACK-ACK ----
	uBlox_TX.Acknowledge_State = ID_ACK_NAK;
	// ---- Reset Counter ----
	uBlox_TX.Counter = 0 ;

	//while ((uBlox_TX.Acknowledge_State == ID_ACK_NAK) && (uBlox_TX.Counter++ < Max_Config_Retry))
	while ((uBlox_TX.Counter++ < Max_Config_Retry))
	{
		// ---- Transmit Message via UART ----
		UBX_TX(&uBlox_TX);
		if (!uBlox_TX.Acknowlegde_Required)
		{
			return UBX_TRUE;
		}
		else
		{
			Delay(10);
			if (Find_ACK_Message(uBlox_TX.uBlox_Structure.Class,uBlox_TX.uBlox_Structure.ID) != 0xFF)
			{
				return UBX_TRUE;
			}
		}
	}
	return UBX_FALSE;
}

/**
 * @brief  UBX_Soft_Reset, send reset message
 * @param  Reset Type - enumeration _uBlox_Reset
 * @retval Size of the uBlox message including the checksum size
 */
uint8_t UBX_Soft_Reset(uint8_t _uBlox_Reset_Type)
{
	// ---- Software Reset has no UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_FALSE ;

	// ----------------- "UBX-CFG-RST" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_RST;

	uBlox_TX.uBlox_Structure.Length[0] = 0x04 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;

	uBlox_TX.uBlox_Structure.Data[0] = 0x00;	// Hot Start
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;	// Warm Start
	uBlox_TX.uBlox_Structure.Data[2] = _uBlox_Reset_Type; // enum _uBlox_Reset
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;	// Reserved

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Save_Default(void)
{
	// ---- Default Configuration has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;

	// ----------------- "UBX-CFG-CFG" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_CFG;

	uBlox_TX.uBlox_Structure.Length[0] = 0x0D;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00;

	uBlox_TX.uBlox_Structure.Data[0] = 0xFF;
	uBlox_TX.uBlox_Structure.Data[1] = 0xFF;
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0x00;
	uBlox_TX.uBlox_Structure.Data[9] = 0xFF;
	uBlox_TX.uBlox_Structure.Data[10] = 0xFF;
	uBlox_TX.uBlox_Structure.Data[11] = 0x00;
	uBlox_TX.uBlox_Structure.Data[12] = 0x00;
	uBlox_TX.uBlox_Structure.Data[13] = 0x01;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Save_Config(void)
{
	// ---- Save Configuration has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-CFG" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_CFG;

	uBlox_TX.uBlox_Structure.Length[0] = 0x0D;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00;

	uBlox_TX.uBlox_Structure.Data[0] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[1] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;
	uBlox_TX.uBlox_Structure.Data[4] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[5] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[9] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[10] = 0x00;
	uBlox_TX.uBlox_Structure.Data[11] = 0x00;
	uBlox_TX.uBlox_Structure.Data[12] = 0x01;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Load_Config(void)
{
	// ---- Load Configuration has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-CFG" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_CFG;
	uBlox_TX.uBlox_Structure.Length[0] = 0x0D;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0xFF;
	uBlox_TX.uBlox_Structure.Data[9] = 0xFF;
	uBlox_TX.uBlox_Structure.Data[10] = 0x00;
	uBlox_TX.uBlox_Structure.Data[11] = 0x00;
	uBlox_TX.uBlox_Structure.Data[12] = 0x01;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Antenna_Config(void)
{
	// ---- Antenna Configuration has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-ANT" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_ANT;
	uBlox_TX.uBlox_Structure.Length[0] = 0x04 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x1F;
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[2] = 0x0F;
	uBlox_TX.uBlox_Structure.Data[3] = 0x64;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_GPS_GLONASS_Config(void)
{
	// ---- GPS GLONASS has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-GNNS" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_GNSS;
	uBlox_TX.uBlox_Structure.Length[0] = 0x14 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[2] = 0x10;
	uBlox_TX.uBlox_Structure.Data[3] = 0x02;
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;
	uBlox_TX.uBlox_Structure.Data[5] = 0x08;
	uBlox_TX.uBlox_Structure.Data[6] = 0x10;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0x01;
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;
	uBlox_TX.uBlox_Structure.Data[10] = 0x00;
	uBlox_TX.uBlox_Structure.Data[11] = 0x01;
	uBlox_TX.uBlox_Structure.Data[12] = 0x06;
	uBlox_TX.uBlox_Structure.Data[13] = 0x08;
	uBlox_TX.uBlox_Structure.Data[14] = 0x0E;
	uBlox_TX.uBlox_Structure.Data[15] = 0x00;
	uBlox_TX.uBlox_Structure.Data[16] = 0x01;
	uBlox_TX.uBlox_Structure.Data[17] = 0x00;
	uBlox_TX.uBlox_Structure.Data[18] = 0x00;
	uBlox_TX.uBlox_Structure.Data[19] = 0x01;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}


uint8_t UBX_Navigation_Config(void)
{
	// ---- Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-NAV5" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_NAV5; 	//24 24 00 47 04 04 03 00 00 00 00 10 27 00 00 05 00
	uBlox_TX.uBlox_Structure.Length[0] = 0x24 ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x43; 	// staticHoldMask=1; dgpsMask=0;cnoThreshold=0; utc=1;
	uBlox_TX.uBlox_Structure.Data[1] = 0x04; 	// dyn = 1; minEl=1; posFixMode=0; drLm = 0; posMask=0; timeMask=0;
//	if(Vision_Status.Group_status == Pedestrians)
//		uBlox_TX.uBlox_Structure.Data[2] = 0x03; 	//dynModel = 4 (automotive)
//	else if(Vision_Status.Group_status == System_Specific)
//		uBlox_TX.uBlox_Structure.Data[2] = 0x02;
//	else
		uBlox_TX.uBlox_Structure.Data[2] = 0x04;
	uBlox_TX.uBlox_Structure.Data[3] = 0x03; 	//fixMode = 3 (auto 2D/3D)
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;  	//fixedAlt = disabled
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0x10; 	//fixedAltVar = disabled
	uBlox_TX.uBlox_Structure.Data[9] = 0x27;
	uBlox_TX.uBlox_Structure.Data[10] = 0x00;
	uBlox_TX.uBlox_Structure.Data[11] = 0x00;
	uBlox_TX.uBlox_Structure.Data[12] = 0x05; 	// minElev = 5deg
	uBlox_TX.uBlox_Structure.Data[13] = 0x00; 	// reserverd
	uBlox_TX.uBlox_Structure.Data[14] = 0xFA; 	// pDop = disabled (default: 0xFA)
	uBlox_TX.uBlox_Structure.Data[15] = 0x00;
	uBlox_TX.uBlox_Structure.Data[16] = 0xFA; 	// tDop = disabled (default: 0xFA)
	uBlox_TX.uBlox_Structure.Data[17] = 0x00;
	uBlox_TX.uBlox_Structure.Data[18] = 0x64; 	// pAcc = disabled (default: 0x64) -> 100m
	uBlox_TX.uBlox_Structure.Data[19] = 0x00;
	uBlox_TX.uBlox_Structure.Data[20] = 0x2C; 	// tAcc = disabled (default: 0x012C) -> 300m
	uBlox_TX.uBlox_Structure.Data[21] = 0x01;
	uBlox_TX.uBlox_Structure.Data[22] = 0x32; 	// staticHoldThresh = 0.5 m/s -> 2 km/h
	uBlox_TX.uBlox_Structure.Data[23] = 0x00; 	// DGPS= disabled
	uBlox_TX.uBlox_Structure.Data[24] = 0x10; 	// threshold disabled
	uBlox_TX.uBlox_Structure.Data[25] = 0x27; 	// threshold disabled
	uBlox_TX.uBlox_Structure.Data[26] = 0x00; 	// reserved
	uBlox_TX.uBlox_Structure.Data[27] = 0x00;
	uBlox_TX.uBlox_Structure.Data[28] = 0x02; 	// staticHoldMaxDist = 5m //test this at 2m.. TODO: NEIl
	uBlox_TX.uBlox_Structure.Data[29] = 0x00;
	uBlox_TX.uBlox_Structure.Data[30] = 0x00; 	// UTC=0 (Automatic)
	uBlox_TX.uBlox_Structure.Data[31] = 0x00; 	// Reserved
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_High_Navigation_Rate_Config(void)
{
	// ---- High Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-HNR" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_HNR; 		// 5C 04 00 02 00 00 00 7A 1C
	uBlox_TX.uBlox_Structure.Length[0] = 0x04 ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x05; 	// High Nav Rate = 5Hz (Max=20Hz; messages should be adjusted to this rate as well)
	uBlox_TX.uBlox_Structure.Data[1] = 0x00; 	// Reserved
	uBlox_TX.uBlox_Structure.Data[2] = 0x00; 	// Reserved
	uBlox_TX.uBlox_Structure.Data[3] = 0x00; 	// Reserved

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}


uint8_t UBX_Datum_Config(void)
{
	// no need to configure, uses default of WGS84 for Datum
	// might need to set up blank message for polling

	return UBX_TRUE;
}


uint8_t UBX_Navigation_Expert_Config(void)
{
	// ---- Expert Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-NAVX5" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_NAVX5;
	uBlox_TX.uBlox_Structure.Length[0] = 0x28;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[0] = 0x02; 	// Version = 2;
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[2] = 0x0C; 	// mask1 (aop=0;ppp=0;ackAid=0;wknRoll=1;initial3dfix=0;minCno=1;minMax=1;)
	uBlox_TX.uBlox_Structure.Data[3] = 0x04;
	uBlox_TX.uBlox_Structure.Data[4] = 0x80; 	// mask2 (sigAttenComp=1; adr=0;)
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0x00; 	// reserved [2]
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;
	uBlox_TX.uBlox_Structure.Data[10] = 0x03; 	// minSVs = 3
	uBlox_TX.uBlox_Structure.Data[11] = 0x10; 	// maxSVs = 16
	uBlox_TX.uBlox_Structure.Data[12] = 0x00; 	// minCNO = 0 (dBHz); unsure about minimum signal level for navigation
	uBlox_TX.uBlox_Structure.Data[13] = 0x00; 	// reserved
	uBlox_TX.uBlox_Structure.Data[14] = 0x00; 	// iniFix3D=0;
	uBlox_TX.uBlox_Structure.Data[15] = 0x00; 	// reserved[2]
	uBlox_TX.uBlox_Structure.Data[16] = 0x00; 	//
	uBlox_TX.uBlox_Structure.Data[17] = 0x00;	// ackAiding = 0; not available in SA
	uBlox_TX.uBlox_Structure.Data[18] = 0x00; 	// wknRollover=1024
	uBlox_TX.uBlox_Structure.Data[19] = 0x04;
	uBlox_TX.uBlox_Structure.Data[20] = 0xFF; 	// sigAttenCompMode = 255 (Automatic)
	uBlox_TX.uBlox_Structure.Data[21] = 0x00;
	uBlox_TX.uBlox_Structure.Data[22] = 0x00; 	// reserved[2]
	uBlox_TX.uBlox_Structure.Data[23] = 0x00;
	uBlox_TX.uBlox_Structure.Data[24] = 0x00; 	// reserved[2]
	uBlox_TX.uBlox_Structure.Data[25] = 0x00;
	uBlox_TX.uBlox_Structure.Data[26] = 0x00; 	// usePPP = 0; disabled
	uBlox_TX.uBlox_Structure.Data[27] = 0x00; 	// aopCfg = 0; disabled
	uBlox_TX.uBlox_Structure.Data[28] = 0x00; 	// reserved[2]
	uBlox_TX.uBlox_Structure.Data[29] = 0x00;
	uBlox_TX.uBlox_Structure.Data[30] = 0x00; 	// Maximum acceptable AssistNowAutonomous orbit error= 0 (firmware default);
	uBlox_TX.uBlox_Structure.Data[31] = 0x00;
	uBlox_TX.uBlox_Structure.Data[32] = 0x00; 	// reserved[4]
	uBlox_TX.uBlox_Structure.Data[33] = 0x00;
	uBlox_TX.uBlox_Structure.Data[34] = 0x00;
	uBlox_TX.uBlox_Structure.Data[35] = 0x00;
	uBlox_TX.uBlox_Structure.Data[36] = 0x00; 	// reserved[3]
	uBlox_TX.uBlox_Structure.Data[37] = 0x00;
	uBlox_TX.uBlox_Structure.Data[38] = 0x00;
	uBlox_TX.uBlox_Structure.Data[39] = 0x00; 	// useAdr = disabled
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Measurement_NAV_Rate_Config(void)
{
	// ---- Expert Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-RATE" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //B5 62 06 08
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_RATE;
	uBlox_TX.uBlox_Structure.Length[0] = 0x06 ; //06 00 F4 01 01 00 01 00
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0xF4; 	// measRate[2] = 500ms -> 2Hz;
	uBlox_TX.uBlox_Structure.Data[1] = 0x01;
	uBlox_TX.uBlox_Structure.Data[2] = 0x02; 	// navRate[2] = 2;
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;
	uBlox_TX.uBlox_Structure.Data[4] = 0x01; 	// timeRef = 1 (GPS time)
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_SBAS_Config(void)
{
	// ---- Expert Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-SBAS" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //B5 62 06 08
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_SBAS;
	uBlox_TX.uBlox_Structure.Length[0] = 0x08 ; //08 00 00 01 01 00 00 00 00 00
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00; 	// mode=0; disabled
	uBlox_TX.uBlox_Structure.Data[1] = 0x01; 	// usage = 1; range N/A
	uBlox_TX.uBlox_Structure.Data[2] = 0x01; 	// maxSBAS = 1;
	uBlox_TX.uBlox_Structure.Data[3] = 0x00; 	// scanmode2 = 0
	uBlox_TX.uBlox_Structure.Data[4] = 0x00; 	// scanmode1[4] =0
	uBlox_TX.uBlox_Structure.Data[5] = 0x00; 	//
	uBlox_TX.uBlox_Structure.Data[6] = 0x00; 	//
	uBlox_TX.uBlox_Structure.Data[7] = 0x00; 	//

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}


uint8_t UBX_NMEA_Protocol_Config(void)
{
	// ---- Expert Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-NMEA" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //B5 62 06 17
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_NMEA;
	uBlox_TX.uBlox_Structure.Length[0] = 0x04 ; //04 00 00 41 00 02
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00; 	// filter=0 (trackFilt=0,gpsOnlyFilter=0;dateFilt=0;timeFilt=0;mskPosFilt=0;posFilt=0;)
	uBlox_TX.uBlox_Structure.Data[1] = 0x41; 	// NMEA version 4.1
	uBlox_TX.uBlox_Structure.Data[2] = 0x00; 	// numSV = 0; (Unlimited)
	uBlox_TX.uBlox_Structure.Data[3] = 0x02; 	// flags = 0; (consider=1; compat=0;)

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Power_Management_Config(void)
{
	// ---- Expert Navigation has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-RXM" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //B5 62 06 11
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_RXM;
	uBlox_TX.uBlox_Structure.Length[0] = 0x02 ; //02 00 08 00
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x08; 	// reserved (0x08 default value)
	uBlox_TX.uBlox_Structure.Data[1] = 0x00; 	// lpMode = 0: (Continues mode)

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}


uint8_t UBX_Extened_Power_Management_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-PM2" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_PM2;
	uBlox_TX.uBlox_Structure.Length[0] = 0x2C ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x01;  // Message version (0x01 for this version)
	uBlox_TX.uBlox_Structure.Data[1] = 0x06;  // Reserved
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;  // maxStartupStateDur
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  // Reserved
	uBlox_TX.uBlox_Structure.Data[4] = 0x60;  // flags[4] (EXTINTpin0 (0); extintWake = enabled (1); extIntBackup= enabled (1); limitPeakCurr = enabled(01);
	uBlox_TX.uBlox_Structure.Data[5] = 0x91;  // waitTimeFix=0; updateRTC=0; updateEPH=1; doNotEnterOff = 1; mode=01;
	uBlox_TX.uBlox_Structure.Data[6] = 0x42;  //
	uBlox_TX.uBlox_Structure.Data[7] = 0x01;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0xE8;  // updatePeriod[4] (1000ms)
	uBlox_TX.uBlox_Structure.Data[9] = 0x03;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[12] = 0x10; // searchPeriod[4] (10 000ms)
	uBlox_TX.uBlox_Structure.Data[13] = 0x27; //
	uBlox_TX.uBlox_Structure.Data[14] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[15] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[16] = 0x00; // gridOffset [4] (disabled)
	uBlox_TX.uBlox_Structure.Data[17] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[18] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[19] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[20] = 0x02; // onTime [2] (2s)
	uBlox_TX.uBlox_Structure.Data[21] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[22] = 0x00; // minAcqTime [2] (0s)
	uBlox_TX.uBlox_Structure.Data[23] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[24] = 0x00; // reserved3 [20]
	uBlox_TX.uBlox_Structure.Data[25] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[26] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[27] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[28] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[29] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[30] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[31] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[32] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[33] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[34] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[35] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[36] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[37] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[38] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[39] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[40] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[41] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[42] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[43] = 0x00; //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Power_Mode_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-PMS" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_PMS;
	uBlox_TX.uBlox_Structure.Length[0] = 0x08 ; // Length=8
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00; 	// version (0x00)
	uBlox_TX.uBlox_Structure.Data[1] = 0x00; 	// powerSetupValue = 0 (Full Power)
	uBlox_TX.uBlox_Structure.Data[2] = 0x00; 	// period[2] = 0;
	uBlox_TX.uBlox_Structure.Data[3] = 0x00; 	//
	uBlox_TX.uBlox_Structure.Data[4] = 0x00; 	// onTime[2] = 0;
	uBlox_TX.uBlox_Structure.Data[5] = 0x00; 	//
	uBlox_TX.uBlox_Structure.Data[6] = 0x00; 	// reserved[2]
	uBlox_TX.uBlox_Structure.Data[7] = 0x00; 	//

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Jamming_Monitor_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-ITFM" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_ITFM;
	uBlox_TX.uBlox_Structure.Length[0] = 0x08 ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0xF3; // config (jamming monitor= disabled; reserved algorithm settings (0x16B156); CW jamming detection (dB); Broadband Jamming detection (dB))
	uBlox_TX.uBlox_Structure.Data[1] = 0xAC; //
	uBlox_TX.uBlox_Structure.Data[2] = 0x62; //
	uBlox_TX.uBlox_Structure.Data[3] = 0x2D; //
	uBlox_TX.uBlox_Structure.Data[4] = 0x1E; // config2 (enable2; antSetting; generalBites)
	uBlox_TX.uBlox_Structure.Data[5] = 0x03; //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00; //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); // Calculate Checksum

	// ---- Transmit Message via UART ----
	return UBX_Transmit();
}

uint8_t UBX_UART_Port_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-PRT" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_PRT;
	uBlox_TX.uBlox_Structure.Length[0] = 0x14 ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x01;  // PortID = 1 (UART1)
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;  // Reserved
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;  // TxReady[2] = disabled
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[4] = 0xC0;  // mode[4]  UART all 8bit with no parity and one stop bit
	uBlox_TX.uBlox_Structure.Data[5] = 0x08;  //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0x80;  // baudRate[4] ( 9600 Bits/s {0x25, 0x80}) ( 115200 Bits/s {0x01 0xC2 0x00})
	uBlox_TX.uBlox_Structure.Data[9] = 0x25;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[12] = 0x01; // inProtoMask[2]
	uBlox_TX.uBlox_Structure.Data[13] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[14] = 0x01; // outProtoMask[2]
	uBlox_TX.uBlox_Structure.Data[15] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[16] = 0x00; // flags[2]
	uBlox_TX.uBlox_Structure.Data[17] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[18] = 0x00; // reserved[2]
	uBlox_TX.uBlox_Structure.Data[19] = 0x00; //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Info_Messages_Poll_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-INF" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_INF;
	uBlox_TX.uBlox_Structure.Length[0] = 0x01 ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;    // protocolID = 0 (UBX Protocol)

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  // Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();

}

uint8_t UBX_Information_Messages_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-INF" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_INF;
	uBlox_TX.uBlox_Structure.Length[0] = 0x0A ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;    // protocolID = 0 (UBX Protocol)
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;    // reserved [3]
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;    // infMsgMask [6]
	uBlox_TX.uBlox_Structure.Data[5] = 0xFF;    // Enable all
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[8] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;    //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Messages_Config(uint8_t UBX_Class, uint8_t UBX_ID)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-MSG" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ; //
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_CON_MSG;
	uBlox_TX.uBlox_Structure.Length[0] = 0x08 ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = UBX_Class; //
	uBlox_TX.uBlox_Structure.Data[1] = UBX_ID;    //
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[3] = 0x01;    //
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;    //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;    //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); 	// Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

//void UBX_Message_Rate_Config(enum msgClass,  enum msgID, int rate)
//{
//// ---- Configuration has UBX-ACK-ACK acknowledge return ----
//		uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
//	// ----------------- "UBX-CFG-MSG" ------------------
//	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
//
//	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
//	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
//	uBlox_TX.uBlox_Structure.Class = Class_CFG;
//	uBlox_TX.uBlox_Structure.ID = ID_MSG;
//	uBlox_TX.uBlox_Structure.Length[0] = 0x03 ; 		//
//	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
//	uBlox_TX.uBlox_Structure.Data[0] = msgClass;    	// _UBX_Class
//	uBlox_TX.uBlox_Structure.Data[1] = msgID;    	// _UBX_ID
//	uBlox_TX.uBlox_Structure.Data[2] = rate;    		// rate = 1
//
//
//	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); // Calculate Checksum
//
//	// ---- Transmit Message via UART ----
//	UBX_TX(&uBlox_TX);
//
//}

uint8_t UBX_Remote_Inventory_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-RINV" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_RINV;
	uBlox_TX.uBlox_Structure.Length[0] = 0x01 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00; // flags dump=0; disabled if enabled set binary to 0 to make string dump after bootload;

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX); // Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Data_Logger_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-LOGFILTER" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_LOGFILTER;
	uBlox_TX.uBlox_Structure.Length[0] = 0x0C ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x01;  // version = 1
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;  // flags (recordEnabled = 0; psmOncePerWakupEnabled = 0; applyAllFilterSettings = 0;)
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;  // minInterval[2] = 0 (not set)
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;  // timeThreshold[2] = 0 (not set)
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  // speedThreshold[2] = 0 (not set)
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0x00;  // positionThreshold[4] = 0 (not set)
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  // Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Time_Pulse_Parameter_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-TP5" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_TP5;
	uBlox_TX.uBlox_Structure.Length[0] = 0x20 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x01;  // tpIdx = 1 TIMEPULSE2
	uBlox_TX.uBlox_Structure.Data[1] = 0x01;  // version = 0x01
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;  // reserved[2]
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[4] = 0x32;  // antCableDelay[2] = 50ns (default)
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  // rfGroupDelay[2] = 0ns
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0x04;  // freqPeriod[4] = 4Hz (default value)
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[12] = 0x01; // freqPeriodLock[4] = 1Hz
	uBlox_TX.uBlox_Structure.Data[13] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[14] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[15] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[16] = 0x00; // pulseLenRatio[4] = 50% dutycycle
	uBlox_TX.uBlox_Structure.Data[17] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[18] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[19] = 0x80; //
	uBlox_TX.uBlox_Structure.Data[20] = 0x99; // pulseLenRatioLock[4] = 10% dutycycle
	uBlox_TX.uBlox_Structure.Data[21] = 0x99; //
	uBlox_TX.uBlox_Structure.Data[22] = 0x99; //
	uBlox_TX.uBlox_Structure.Data[23] = 0x19; //
	uBlox_TX.uBlox_Structure.Data[24] = 0x00; // userConfigDelay[4] = 0ns
	uBlox_TX.uBlox_Structure.Data[25] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[26] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[27] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[28] = 0xEF; // flags[4] (active = 1; lockGnssFreq = 1; lockedOtherSet = 1; isFreq = 1; alignToTow =1; polarity = 1; gridUctGnss = 0001; syncMode = 000;)
	uBlox_TX.uBlox_Structure.Data[29] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[30] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[31] = 0x00; //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  // Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Odometer_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-ODO" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_ODO;
	uBlox_TX.uBlox_Structure.Length[0] = 0x14 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[2] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[4] = 0x01;  //
	uBlox_TX.uBlox_Structure.Data[5] = 0x03;  //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[12] = 0x0A; //
	uBlox_TX.uBlox_Structure.Data[13] = 0x32; //
	uBlox_TX.uBlox_Structure.Data[14] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[15] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[16] = 0x99; // pulseLenRatio[4] = 50% dutycycle
	uBlox_TX.uBlox_Structure.Data[17] = 0x4C; //
	uBlox_TX.uBlox_Structure.Data[18] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[19] = 0x00; //

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  // Calculate Checksum

	// ---- Transmit Message via UART ----
	return UBX_Transmit();
}

uint8_t UBX_Geofencing_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-GEOFENCE" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_GEOFENCE;
	uBlox_TX.uBlox_Structure.Length[0] = 0x08 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;  // version = 0
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;  // numFences = 0
	uBlox_TX.uBlox_Structure.Data[2] = 0x02;  // confLvl = 2 (95%)
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  // reserved
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;  // pioEnabled
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;  // pinPolarity
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  // pin
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  // reserved

	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  // Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_Dynamic_Seed_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-DYNSEED" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_DYNSEED;
	uBlox_TX.uBlox_Structure.Length[0] = 0x0C ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x01;  // version = 1
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;  // reserved[3]
	uBlox_TX.uBlox_Structure.Data[2] = 0x02;  //
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;  // seedHi[4] not yet configured (high word of dynamic seed)
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0x00;  // seedLo[4] not yet configured (low word of dynamic seed)
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  // Calculate Checksum

	// ---- Transmit Message ----
	return UBX_Transmit();

}


uint8_t UBX_Fixed_Seed_Config(void)
{
	// ---- Configuration has UBX-ACK-ACK acknowledge return ----
	uBlox_TX.Acknowlegde_Required = UBX_TRUE ;
	// ----------------- "UBX-CFG-FIXSEED" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_FIXSEED;
	uBlox_TX.uBlox_Structure.Length[0] = 0x0E ; //
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x01;  // version = 1
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;  // reserved[3]
	uBlox_TX.uBlox_Structure.Data[2] = 0x02;  //
	uBlox_TX.uBlox_Structure.Data[3] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;  // seedHi[4] not yet configured (high word of fixed seed)
	uBlox_TX.uBlox_Structure.Data[5] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[6] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[8] = 0x00;  // seedLo[4] not yet configured (low word of fixed seed)
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;  //
	uBlox_TX.uBlox_Structure.Data[10] = 0x00; //
	uBlox_TX.uBlox_Structure.Data[11] = 0x00; //
	//uBlox_TX.uBlox_Structure.Data[12] = 0x00;  // use for specific messages checked when fixed seed is used
	//uBlox_TX.uBlox_Structure.Data[13] = 0x00;  // use for specific messages checked when fixed seed is used
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);  //checksum

	// ---- Transmit Message ----
	return UBX_Transmit();
}

uint8_t UBX_$_Config(void)
{
	// ---- GPS GLONASS has UBX-ACK-ACK return ----
	uBlox_TX.Acknowlegde_Required = UBX_FALSE ;
	// ----------------- "UBX-CFG-GNNS" ------------------
	uBlox_TX.uBlox_Structure.Header[0] = '$' ;
	uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
	uBlox_TX.uBlox_Structure.Class = Class_CFG;
	uBlox_TX.uBlox_Structure.ID = ID_GNSS;
	uBlox_TX.uBlox_Structure.Length[0] = 0x14 ;
	uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
	uBlox_TX.uBlox_Structure.Data[0] = 0x00;
	uBlox_TX.uBlox_Structure.Data[1] = 0x00;
	uBlox_TX.uBlox_Structure.Data[2] = 0x10;
	uBlox_TX.uBlox_Structure.Data[3] = 0x02;
	uBlox_TX.uBlox_Structure.Data[4] = 0x00;
	uBlox_TX.uBlox_Structure.Data[5] = 0x08;
	uBlox_TX.uBlox_Structure.Data[6] = 0x10;
	uBlox_TX.uBlox_Structure.Data[7] = 0x00;
	uBlox_TX.uBlox_Structure.Data[8] = 0x01;
	uBlox_TX.uBlox_Structure.Data[9] = 0x00;
	uBlox_TX.uBlox_Structure.Data[10] = 0x00;
	uBlox_TX.uBlox_Structure.Data[11] = 0x01;
	uBlox_TX.uBlox_Structure.Data[12] = 0x06;
	uBlox_TX.uBlox_Structure.Data[13] = 0x08;
	uBlox_TX.uBlox_Structure.Data[14] = 0x0E;
	uBlox_TX.uBlox_Structure.Data[15] = 0x00;
	uBlox_TX.uBlox_Structure.Data[16] = 0x01;
	uBlox_TX.uBlox_Structure.Data[17] = 0x00;
	uBlox_TX.uBlox_Structure.Data[18] = 0x00;
	uBlox_TX.uBlox_Structure.Data[19] = 0x01;
	uBlox_TX.Size = UBX_Checksum(&uBlox_TX);

	// ---- Transmit Message ----
	return UBX_Transmit();
}

//uint8_t UBX_Reset_Odometer(void)
//{
//	// ---- GPS GLONASS has UBX-ACK-ACK return ----
//		uBlox_TX.Acknowlegde_Required = UBX_FALSE ;
//		// ----------------- "UBX-CFG-GNNS" ------------------
//		uBlox_TX.uBlox_Structure.Header[0] = UBX_1 ;
//		uBlox_TX.uBlox_Structure.Header[1] = UBX_2 ;
//		uBlox_TX.uBlox_Structure.Class = Class_NAV;
//		uBlox_TX.uBlox_Structure.ID = ID_NAV_RESETODO;
//		uBlox_TX.uBlox_Structure.Length[0] = 0x00 ;
//		uBlox_TX.uBlox_Structure.Length[1] = 0x00 ;
//		uBlox_TX.Size = UBX_Checksum(&uBlox_TX);
//
//		// ---- Transmit Message ----
//		return UBX_Transmit();
//}
