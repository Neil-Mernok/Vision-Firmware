/*
 * uBlox_Parse_Message.c
 *
 *  Created on: Mar 9, 2017
 *      Author: FrancoisHattingh
 */
#include "NMEA_Parse_Message.h"
#include "NMEA_General.h"
#include "string.h"

/**
 * @brief  Parse message received from uBlox module
 * @param  MIF - Master Information Frame, a general information frame to store information in
 * @retval Successfully executed or failed
 */
uint16_t NMEA_parse_message(_Q_MasterIF* NMEA_MIF, uint8_t NMEA_start)
{
		// ---- Switch between different message classes ----
		switch (NMEA_MIF->data[NMEA_start + 2])
		{
			case NMEA_Galileo:		// ---- NMEA Galileo ----
				Parse_NMEA_GNSS_Message(NMEA_MIF, NMEA_start);
				break;
			case NMEA_BeiDou:		// ---- NMEA BeiDou ----
				Parse_NMEA_GNSS_Message(NMEA_MIF, NMEA_start);
				break;
			case NMEA_GLONASS: 		// ---- NMEA GLONASS ----
				Parse_NMEA_GNSS_Message(NMEA_MIF, NMEA_start);
				break;
			case NMEA_GNSS:			// ---- NMEA GNSS ----
				Parse_NMEA_GNSS_Message(NMEA_MIF, NMEA_start);
				break;
			case NMEA_GPS: 			// ---- NMEA GPS ----
				Parse_NMEA_GNSS_Message(NMEA_MIF, NMEA_start);
				break;
			default:
				break;
		}
			// ---- Return length - Message successfully parsed ----
			return NMEA_PASS;
//	}
//	// ---- Return FAIL - Empty message received, Checksum did not match ----
//	return UBX_FAIL;
}

uint8_t Parse_NMEA_GLONASS_Message(_Q_MasterIF* NMEA_GLONASS_MIF, uint8_t GLONASS_start)
{
	if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_B) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_S))
	{

	}
	else if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_A))
	{

	}
	else if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_N) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_S))
	{

	}
	else if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_L) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_L))
	{

	}
	else if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_S) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_T))
	{

	}
	else if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_G) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_S) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_V))
	{

	}
	else if ((NMEA_GLONASS_MIF->data[GLONASS_start + 3] == ASCII_R) && (NMEA_GLONASS_MIF->data[GLONASS_start + 4] == ASCII_M) && (NMEA_GLONASS_MIF->data[GLONASS_start + 5] == ASCII_C))
	{

	}
	return NMEA_FAIL;
}

uint8_t Parse_NMEA_GNSS_Message(_Q_MasterIF* NMEA_GNSS_MIF, uint8_t GNSS_start)
{
	//uint16_t length = NMEA_Checksum(NMEA_GNSS_MIF, ASCII_$, GNSS_start);

//	uint8_t RMC_Checksum[3] = {ASCII_star, ASCII_5, ASCII_7};
	uint8_t Data[29] = {0} ;

	if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_B) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_S))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x52;
		Data[10] = 0x4D;
		Data[11] = 0x43;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x34;
		Data[26] = 0x37;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_A))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x47;
		Data[10] = 0x47;
		Data[11] = 0x41;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x35;
		Data[26] = 0x41;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);

	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_N) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_S))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x47;
		Data[10] = 0x4E;
		Data[11] = 0x53;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x34;
		Data[26] = 0x31;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_L) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_L))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x47;
		Data[10] = 0x4C;
		Data[11] = 0x4C;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x35;
		Data[26] = 0x43;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);

	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_S) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_T))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x47;
		Data[10] = 0x53;
		Data[11] = 0x54;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x35;
		Data[26] = 0x42;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_S) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_A))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x47;
		Data[10] = 0x53;
		Data[11] = 0x41;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x34;
		Data[26] = 0x45;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_G) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_S) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_V))
	{

		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x47;
		Data[10] = 0x53;
		Data[11] = 0x56;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x35;
		Data[26] = 0x39;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_R) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_M) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_C))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x52;
		Data[10] = 0x4D;
		Data[11] = 0x43;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x34;
		Data[26] = 0x37;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	else if ((NMEA_GNSS_MIF->data[GNSS_start + 3] == ASCII_V) && (NMEA_GNSS_MIF->data[GNSS_start + 4] == ASCII_T) && (NMEA_GNSS_MIF->data[GNSS_start + 5] == ASCII_G))
	{
		Data[0] = 0x24;
		Data[1] = 0x50;
		Data[2] = 0x55;
		Data[3] = 0x42;
		Data[4] = 0x58;
		Data[5] = 0x2C;
		Data[6] = 0x34;
		Data[7] = 0x30;
		Data[8] = 0x2C;
		Data[9] = 0x56;
		Data[10] = 0x54;
		Data[11] = 0x47;
		Data[12] = 0x2C;
		Data[13] = 0x30;
		Data[14] = 0x2C;
		Data[15] = 0x30;
		Data[16] = 0x2C;
		Data[17] = 0x30;
		Data[18] = 0x2C;
		Data[19] = 0x30;
		Data[20] = 0x2C;
		Data[21] = 0x30;
		Data[22] = 0x2C;
		Data[23] = 0x30;
		Data[24] = 0x2A;
		Data[25] = 0x35;
		Data[26] = 0x45;
		Data[27] = 0x0D;
		Data[28] = 0x0A;

		HAL_UART_Transmit(uBlox_COM, Data, 29,10);
	}
	return NMEA_FAIL;
}
