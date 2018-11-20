/*
 * uBlox_Parse_Message.c
 *
 *  Created on: Mar 9, 2017
 *      Author: FrancoisHattingh
 */
#include "uBlox_Parse_Message.h"
#include "uBlox_General.h"
#include "uBlox_Acknowledge.h"
#include "GPS_APL.h"
#include "Global_Variables.h"
#include "Vision_Parameters.h"

uint16_t Year;
/**
 * @brief  Parse message received from uBlox module
 * @param  MIF - Master Information Frame, a general information frame to store information in
 * @retval Successfully executed or failed
 */
uint16_t uBlox_parse_message(_Q_MasterIF uBlox_MIF, uint16_t start_pos)
{
	// ---- Declare a general uBlox Message structure ----
	_uBlox_Message Parse_Messsage;

	// ---- Check if a valid message is received ----
	if (uBlox_MIF.len)
	{
		// ---- Copy first six bytes into temporary structure ----
		// ---- UBX-Class-ID-Length ----
		memcpy(&Parse_Messsage.uBlox_Structure, uBlox_MIF.data + start_pos, Identifier_Offset);

		// ---- Conjugate Length ----
		Parse_Messsage.Size = UBX_Conjugate_uint16(Parse_Messsage.uBlox_Structure.Length[0], Parse_Messsage.uBlox_Structure.Length[1]);

		// ---- Copy data bytes into temporary structure ----
		memcpy(&Parse_Messsage.uBlox_Structure.Data, uBlox_MIF.data + start_pos + Identifier_Offset, Parse_Messsage.Size);

		// ---- Add Header to Length ----
		Parse_Messsage.Size += Identifier_Offset;

		// ---- Copy data bytes into temporary structure ----
		memcpy(&Parse_Messsage.uBlox_Structure.Checksum, uBlox_MIF.data + start_pos + Parse_Messsage.Size, Checksum_Offset);

		// ---- Compare the checksum values to ensure full message has been received ----
		if(UBX_Checksum_Compare(&Parse_Messsage))
		{
			// ---- Switch between different message classes ----
			switch (Parse_Messsage.uBlox_Structure.Class)
			{
				case Class_ACK:		// ---- Message Acknowledge ----
					Parse_ACK_Message(&Parse_Messsage.uBlox_Structure);
					break;
				case Class_NAV:		// ---- Navigation Position Velocity ----
					Parse_NAV_Message(&Parse_Messsage.uBlox_Structure);
					break;
				case Class_HNR: 	// ---- High Rate Output of PVT Solution ----
					Parse_HNR_Message(&Parse_Messsage.uBlox_Structure);
					break;
				case Class_MON:		// ---- Monitoring information ----
					Parse_MON_Message(&Parse_Messsage.uBlox_Structure);
					break;
				case Class_ESF: 	// ---- Vehicle dynamics information ----
					Parse_ESF_Message(&Parse_Messsage.uBlox_Structure);
					break;
				case Class_SEC:		// ---- Security Feature Messages ----
					Parse_SEC_Message(&Parse_Messsage.uBlox_Structure);
					break;
				default:
					return UBX_FAIL;
			}
			// ---- Return Length - Message successfully parsed ----
			return (Parse_Messsage.Size + Checksum_Offset);
		}
	}
	// ---- Return FAIL - Empty message received, Checksum did not match ----
	return UBX_FAIL;
}

/**
 * @brief  Parse Navigation Position Velocity message received from uBlox module
 * @param  Pointer to general uBlox frame where information is stored
 * @retval Successfully executed or failed
 */
uint8_t Parse_NAV_Message(_uBlox_Structure* Parse_NAV_Message)
{
	// ---- Switch between different message identifier ----
	switch (Parse_NAV_Message->ID)
	{
		case ID_NAV_POSECEF:	// ---- Position Solution in ECEF ----
//			Vision_Status.Xcoordinate_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[4],Parse_NAV_Message->Data[5],Parse_NAV_Message->Data[6],Parse_NAV_Message->Data[7]);
//			Vision_Status.Ycoordinate_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[8],Parse_NAV_Message->Data[9],Parse_NAV_Message->Data[10],Parse_NAV_Message->Data[11]);
//			Vision_Status.Zcoordinate_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[12],Parse_NAV_Message->Data[13],Parse_NAV_Message->Data[14],Parse_NAV_Message->Data[15]);
//			Vision_Status.XYZAccuracy_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[16],Parse_NAV_Message->Data[17],Parse_NAV_Message->Data[18],Parse_NAV_Message->Data[19]);
			break;
		case ID_NAV_POSLLH: 	// ---- Geodetic Position Solution ----
			Vision_Status.GPS_Data.Longitude = UBX_Conjugate_uint32(Parse_NAV_Message->Data[4],Parse_NAV_Message->Data[5],Parse_NAV_Message->Data[6],Parse_NAV_Message->Data[7]);
			Vision_Status.GPS_Data.Latitude = UBX_Conjugate_uint32(Parse_NAV_Message->Data[8],Parse_NAV_Message->Data[9],Parse_NAV_Message->Data[10],Parse_NAV_Message->Data[11]);
			Vision_Status.GPS_Data.SeaLevel = UBX_Conjugate_uint32(Parse_NAV_Message->Data[16],Parse_NAV_Message->Data[17],Parse_NAV_Message->Data[18],Parse_NAV_Message->Data[19]);
			break;
		case ID_NAV_STATUS:		// ---- Receiver Navigation Status ----
			Vision_Status.GPS_Data.FixType = Parse_NAV_Message->Data[4];	// ---- Refer to enum _uBlox_Fix_Type ----
			break;
		case ID_NAV_SOL:		// ---- Navigation Solution Information - users are recommended to use the UBX-NAV-PVT message in preference
			Vision_Status.GPS_Data.FixType = Parse_NAV_Message->Data[10];	// ---- Refer to enum _uBlox_Fix_Type ----
//			Vision_Status.Xcoordinate_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[12],Parse_NAV_Message->Data[13],Parse_NAV_Message->Data[14],Parse_NAV_Message->Data[15]);
//			Vision_Status.Ycoordinate_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[16],Parse_NAV_Message->Data[17],Parse_NAV_Message->Data[18],Parse_NAV_Message->Data[19]);
//			Vision_Status.Zcoordinate_cm = UBX_Conjugate_uint32(Parse_NAV_Message->Data[20],Parse_NAV_Message->Data[21],Parse_NAV_Message->Data[22],Parse_NAV_Message->Data[23]);
			Vision_Status.GPS_Data.NumberOfSat= Parse_NAV_Message->Data[5]; // Number of satellites used in Nav Solution
			break;
		case ID_NAV_PVT:		// ---- Navigation Position Velocity Time Solution ----
			Year = UBX_Conjugate_uint16(Parse_NAV_Message->Data[4],Parse_NAV_Message->Data[5]);
			Vision_Status.GPS_Data._Date[3] = (uint8_t)(Year/100);	// Date - Century - UTC
			Vision_Status.GPS_Data._Date[2] = (uint8_t)(Year - ((Year/1000)*1000));	// Date - Year - UTC
//			Vision_Status.GPS_Data._Date[3] = Parse_NAV_Message->Data[4];	// Date - Century - UTC
//			Vision_Status.GPS_Data._Date[2] = Parse_NAV_Message->Data[5];	// Date - Year - UTC
			Vision_Status.GPS_Data._Date[1] = Parse_NAV_Message->Data[6];	// Date - Month - UTC 1..12
			Vision_Status.GPS_Data._Date[0] = Parse_NAV_Message->Data[7];	// Date - Day - UTC 1..31
			Vision_Status.GPS_Data._Time[3] = 0x00;							// Unused
			Vision_Status.GPS_Data._Time[2] = Parse_NAV_Message->Data[8];	// Time - Hour - UTC 0..23
			Vision_Status.GPS_Data._Time[1] = Parse_NAV_Message->Data[9];	// Time - Minutes - UTC 0..59
			Vision_Status.GPS_Data._Time[0] = Parse_NAV_Message->Data[10];	// Time - Seconds - UTC 0..60
			Vision_Status.GPS_Data._Validity = Parse_NAV_Message->Data[11];  // Date and Time Validity Flags
			Vision_Status.GPS_Data.FixType = Parse_NAV_Message->Data[20];	// Refer to enum _uBlox_Fix_Type
			Vision_Status.sts.Module_RF_working = (bool)(Parse_NAV_Message->Data[21]&0x01);
			Vision_Status.GPS_Data.flags = UBX_Conjugate_uint16(Parse_NAV_Message->Data[21],Parse_NAV_Message->Data[22]);
			Vision_Status.GPS_Data.NumberOfSat= Parse_NAV_Message->Data[23]; // Number of satellites used in Nav Solution
			Vision_Status.GPS_Data.Longitude = UBX_Conjugate_uint32(Parse_NAV_Message->Data[24],Parse_NAV_Message->Data[25],Parse_NAV_Message->Data[26],Parse_NAV_Message->Data[27]);
			Vision_Status.GPS_Data.Latitude = UBX_Conjugate_uint32(Parse_NAV_Message->Data[28],Parse_NAV_Message->Data[29],Parse_NAV_Message->Data[30],Parse_NAV_Message->Data[31]);
			Vision_Status.GPS_Data.SeaLevel = UBX_Conjugate_uint32(Parse_NAV_Message->Data[36],Parse_NAV_Message->Data[37],Parse_NAV_Message->Data[38],Parse_NAV_Message->Data[39]);
			Vision_Status.GPS_Data.HorizontalAccuracy = UBX_Conjugate_uint32(Parse_NAV_Message->Data[40],Parse_NAV_Message->Data[41],Parse_NAV_Message->Data[42],Parse_NAV_Message->Data[43]);
			Vision_Status.GPS_Data.VerticalAccuracy = UBX_Conjugate_uint32(Parse_NAV_Message->Data[44],Parse_NAV_Message->Data[45],Parse_NAV_Message->Data[46],Parse_NAV_Message->Data[47]);
			Vision_Status.GPS_Data.Speed = UBX_Conjugate_uint32(Parse_NAV_Message->Data[60],Parse_NAV_Message->Data[61],Parse_NAV_Message->Data[62],Parse_NAV_Message->Data[63]);
			Vision_Status.GPS_Data.SpeedAccuracy = (UBX_Conjugate_uint32(Parse_NAV_Message->Data[68],Parse_NAV_Message->Data[69],Parse_NAV_Message->Data[70],Parse_NAV_Message->Data[71]));
			Vision_Status.GPS_Data.HeadingMotion = UBX_Conjugate_uint32(Parse_NAV_Message->Data[72],Parse_NAV_Message->Data[73],Parse_NAV_Message->Data[74],Parse_NAV_Message->Data[75]);
			Vision_Status.GPS_Data.HeadingVehicle = UBX_Conjugate_uint32(Parse_NAV_Message->Data[84],Parse_NAV_Message->Data[85],Parse_NAV_Message->Data[86],Parse_NAV_Message->Data[87]);
			Vision_Status.GPS_Data.HeadingAccuracy = UBX_Conjugate_uint32(Parse_NAV_Message->Data[72],Parse_NAV_Message->Data[65],Parse_NAV_Message->Data[66],Parse_NAV_Message->Data[67]);
			// ---- Calculate fix age according to fix ----
			Vision_Status.GPS_Data.FixAge = UBX_Calculate_Age(Vision_Status.GPS_Data.FixType);
			// ---- Push GPS information to Master ----
			if (vision_settings.getActivities().forward_RF)
			{
				SetLed(&LED1, Blue, 0);
				uAPL_send_master_message(Vision_Status.GPS_Data.FixAge);
				SetLed(&LED1, Off, 0);
			}
			break;
		case ID_NAV_ODO: 		// ---- Odometer Solution ----
			Vision_Status.GPS_Data.Distance = UBX_Conjugate_uint32(Parse_NAV_Message->Data[8],Parse_NAV_Message->Data[9],Parse_NAV_Message->Data[10],Parse_NAV_Message->Data[11]);
			Vision_Status.GPS_Data.TotalDistance = UBX_Conjugate_uint32(Parse_NAV_Message->Data[12],Parse_NAV_Message->Data[13],Parse_NAV_Message->Data[14],Parse_NAV_Message->Data[15]);
			// ---- Push Distance information to Master ----
			if (vision_settings.getActivities().forward_RF)
			{
				SetLed(&LED1, Blue, 0);
				uAPL_overall_distance_message();
				SetLed(&LED1, Off, 0);
			}
			break;
		case ID_NAV_TIMEUTC:	// ---- UTC Time Solution ----
			Year = UBX_Conjugate_uint16(Parse_NAV_Message->Data[12],Parse_NAV_Message->Data[13]);
			Vision_Status.GPS_Data._Date[3] = (uint8_t)(Year/100);	// Date - Century - UTC
			Vision_Status.GPS_Data._Date[2] = (uint8_t)(Year - ((Year/1000)*1000));	// Date - Year - UTC
//			Vision_Status.GPS_Data._Date[3] = Parse_NAV_Message->Data[12];	// Date - Century - UTC
//			Vision_Status.GPS_Data._Date[2] = Parse_NAV_Message->Data[13];	// Date - Year - UTC
			Vision_Status.GPS_Data._Date[1] = Parse_NAV_Message->Data[14];	// Date - Month - UTC 1..12
			Vision_Status.GPS_Data._Date[0] = Parse_NAV_Message->Data[15];	// Date - Day - UTC 1..31
			Vision_Status.GPS_Data._Time[3] = 0x00;							// Unused
			Vision_Status.GPS_Data._Time[2] = Parse_NAV_Message->Data[16];	// Time - Hour - UTC 0..23
			Vision_Status.GPS_Data._Time[1] = Parse_NAV_Message->Data[17];	// Time - Minutes - UTC 0..59
			Vision_Status.GPS_Data._Time[0] = Parse_NAV_Message->Data[18];	// Time - Seconds - UTC 0..60
			Vision_Status.GPS_Data._Validity = Parse_NAV_Message->Data[19];  // Date and Time Validity Flags
			break;
		case ID_NAV_SAT:		// ---- Satellite Information ----
			Vision_Status.GPS_Data.NumberOfSat = Parse_NAV_Message->Data[5]; // Number of satellites used in Nav Solution
			break;
		default:
			return UBX_FAIL;
	}
	return UBX_PASS;
}

/**
 * @brief  Parse Acknowledge message received from uBlox module
 * @param  Pointer to general uBlox frame where information is stored
 * @retval Successfully executed or failed
 */
uint8_t Parse_ACK_Message(_uBlox_Structure* Parse_ACK_Message)
{
	// ---- Switch between different message identifier ----
//	switch (Parse_ACK_Message->ID)
//	{
//		case ID_ACK_ACK:		// ---- Message Acknowledge ----
//			// ---- Acknowledge message received ----
//			if (Parse_ACK_Message->Data[0] == uBlox_TX.uBlox_Structure.Class && Parse_ACK_Message->Data[1] == uBlox_TX.uBlox_Structure.ID)
//			{
//				uBlox_TX.Acknowledge_State = ID_ACK_ACK;
//			}
//			break;
//
//		case ID_ACK_NAK:		// ---- Message Not Acknowledge ----
//			// ---- Acknowledge failed message received ----
//			if (Parse_ACK_Message->Data[0] == uBlox_TX.uBlox_Structure.Class && Parse_ACK_Message->Data[1] == uBlox_TX.uBlox_Structure.ID)
//			{
//				uBlox_TX.Acknowledge_State = ID_ACK_NAK;
//			}
//			break;
//		default:
//			// ---- Acknowledge failed message received ----
//			uBlox_TX.Acknowledge_State = ID_ACK_NAK;
//			return UBX_FAIL;
//	}
	Add_ACK_Message(Parse_ACK_Message->Data[0], Parse_ACK_Message->Data[1], Parse_ACK_Message->ID);
	return UBX_PASS;
}

uint8_t Parse_HNR_Message(_uBlox_Structure* Parse_HNR_Message)
{

	// ---- Switch between different message identifier ----
	switch (Parse_HNR_Message->ID)
	{
		case ID_HNR_PVT:		// ---- Navigation Position Velocity Time Solution ----
			Year = UBX_Conjugate_uint16(Parse_HNR_Message->Data[4],Parse_HNR_Message->Data[5]);
			Vision_Status.GPS_Data._Date[3] = (uint8_t)(Year/100);	// Date - Century - UTC
			Vision_Status.GPS_Data._Date[2] = (uint8_t)(Year - ((Year/1000)*1000));	// Date - Year - UTC
//			Vision_Status.GPS_Data._Date[3] = Parse_NAV_Message->Data[4];	// Date - Century - UTC
//			Vision_Status.GPS_Data._Date[2] = Parse_NAV_Message->Data[5];	// Date - Year - UTC
			Vision_Status.GPS_Data._Date[1] = Parse_HNR_Message->Data[6];	// Date - Month - UTC 1..12
			Vision_Status.GPS_Data._Date[0] = Parse_HNR_Message->Data[7];	// Date - Day - UTC 1..31
			Vision_Status.GPS_Data._Time[3] = 0x00;							// Unused
			Vision_Status.GPS_Data._Time[2] = Parse_HNR_Message->Data[8];	// Time - Hour - UTC 0..23
			Vision_Status.GPS_Data._Time[1] = Parse_HNR_Message->Data[9];	// Time - Minutes - UTC 0..59
			Vision_Status.GPS_Data._Time[0] = Parse_HNR_Message->Data[10];	// Time - Seconds - UTC 0..60
			Vision_Status.GPS_Data._Validity = Parse_HNR_Message->Data[11]; // Date and Time Validity Flags
			Vision_Status.GPS_Data.FixType = Parse_HNR_Message->Data[16];	// Refer to enum _uBlox_Fix_Type
			Vision_Status.GPS_Data.Longitude = UBX_Conjugate_uint32(Parse_HNR_Message->Data[20],Parse_HNR_Message->Data[21],Parse_HNR_Message->Data[22],Parse_HNR_Message->Data[23]);
			Vision_Status.GPS_Data.Latitude = UBX_Conjugate_uint32(Parse_HNR_Message->Data[24],Parse_HNR_Message->Data[25],Parse_HNR_Message->Data[26],Parse_HNR_Message->Data[27]);
			Vision_Status.GPS_Data.SeaLevel = (UBX_Conjugate_uint32(Parse_HNR_Message->Data[32],Parse_HNR_Message->Data[33],Parse_HNR_Message->Data[34],Parse_HNR_Message->Data[35]));
			Vision_Status.GPS_Data.HorizontalAccuracy = UBX_Conjugate_uint32(Parse_HNR_Message->Data[52],Parse_HNR_Message->Data[53],Parse_HNR_Message->Data[54],Parse_HNR_Message->Data[55]);
			Vision_Status.GPS_Data.VerticalAccuracy = UBX_Conjugate_uint32(Parse_HNR_Message->Data[56],Parse_HNR_Message->Data[57],Parse_HNR_Message->Data[58],Parse_HNR_Message->Data[59]);
			Vision_Status.GPS_Data.Speed = UBX_Conjugate_uint32(Parse_HNR_Message->Data[36],Parse_HNR_Message->Data[37],Parse_HNR_Message->Data[38],Parse_HNR_Message->Data[39]);
			Vision_Status.GPS_Data.SpeedAccuracy = UBX_Conjugate_uint32(Parse_HNR_Message->Data[60],Parse_HNR_Message->Data[61],Parse_HNR_Message->Data[62],Parse_HNR_Message->Data[63]);
			Vision_Status.GPS_Data.HeadingMotion = UBX_Conjugate_uint32(Parse_HNR_Message->Data[44],Parse_HNR_Message->Data[45],Parse_HNR_Message->Data[46],Parse_HNR_Message->Data[47]);
			Vision_Status.GPS_Data.HeadingVehicle = UBX_Conjugate_uint32(Parse_HNR_Message->Data[48],Parse_HNR_Message->Data[49],Parse_HNR_Message->Data[50],Parse_HNR_Message->Data[51]);
			Vision_Status.GPS_Data.HeadingAccuracy = UBX_Conjugate_uint32(Parse_HNR_Message->Data[64],Parse_HNR_Message->Data[65],Parse_HNR_Message->Data[66],Parse_HNR_Message->Data[67]);
			// ---- Calculate fix age according to fix ----
			Vision_Status.GPS_Data.FixAge = UBX_Calculate_Age(Vision_Status.GPS_Data.FixType);
			// ---- Push GPS information to Master ----
			if (vision_settings.getActivities().forward_RF)
			{
				SetLed(&LED1, Blue, 0);
				uAPL_send_master_message(Vision_Status.GPS_Data.FixAge);
				SetLed(&LED1, Off, 0);
			}
		default:
			return UBX_FAIL;
	}
	return UBX_PASS;
}

uint8_t Parse_STA_Message(_uBlox_Structure* Parse_STA_Message)
{
	return 0;
}

uint8_t Parse_ESF_Message(_uBlox_Structure* Parse_ESF_Message)
{
	return 0;
}
/**
 * @brief  Parse Security message received from uBlox module
 * @param  Pointer to general uBlox frame where information is stored
 * @retval Successfully executed or failed
 */
uint8_t Parse_SEC_Message(_uBlox_Structure* Parse_SEC_Message)
{
	// ---- Switch between different message identifier ----
	switch (Parse_SEC_Message->ID)
	{
		case ID_SEC_UNIQID:		// ---- Unique Chip ID ----
//			memcpy(&Vision_Status.UNIQID,&Parse_SEC_Message->Data[5],5); //Unique chip ID
			break;
		default:
			return UBX_FAIL;
	}
	return UBX_PASS;
}
/**
 * @brief  Parse Security message received from uBlox module
 * @param  Pointer to general uBlox frame where information is stored
 * @retval Successfully executed or failed
 */
uint8_t Parse_MON_Message(_uBlox_Structure* Parse_MON_Message)
{
	// ---- Switch between different message identifier ----
	switch (Parse_MON_Message->ID)
	{
		case ID_MON_VER:		// ---- Receiver / Software Version ----
//			memcpy(&Vision_Status.swVersion,&Parse_MON_Message->Data[0],30); // uBlox Software version
//			memcpy(&Vision_Status.hwVersion,&Parse_MON_Message->Data[30],10); // uBlox Hardware version
			break;
		case ID_MON_HW:			// ---- Hardware Status ----
			Vision_Status.GPS_Data.antenna_status = Parse_MON_Message->Data[20];
			// ---- Push antenna information to Master ----
			if (vision_settings.getActivities().forward_RF)
			{
				SetLed(&LED1, Blue, 0);
				uAPL_antenna_message();
				SetLed(&LED1, Off, 0);
			}
			break;
		default:
			return UBX_FAIL;
	}
	return UBX_PASS;
}
