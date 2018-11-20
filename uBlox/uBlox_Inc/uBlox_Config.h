/*
 * uBlox_Config.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef UBLOX_CONFIG_H_
#define UBLOX_CONFIG_H_

#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#define Max_Config_Retry 3

uint8_t UBX_Transmit(void);
uint8_t UBX_Soft_Reset(uint8_t _uBlox_Reset_Type);
void UBX_Hard_Reset(void);

uint8_t UBX_Save_Default(void);
uint8_t UBX_Save_Config(void);
uint8_t UBX_Load_Config(void);

uint8_t UBX_Antenna_Config(void);
uint8_t UBX_GPS_GLONASS_Config(void);
uint8_t UBX_Navigation_Config(void);
uint8_t UBX_High_Navigation_Rate_Config(void);
uint8_t UBX_Time_Pulse_Parameter_Config(void);
uint8_t UBX_Navigation_Expert_Config(void);
uint8_t UBX_Datum_Config(void);
uint8_t UBX_Measurement_NAV_Rate_Config(void);
uint8_t UBX_SBAS_Config(void);
uint8_t UBX_NMEA_Protocol_Config(void);
uint8_t UBX_Power_Management_Config(void);
uint8_t UBX_Extened_Power_Management_Config(void);
uint8_t UBX_Power_Mode_Config(void);
uint8_t UBX_Jamming_Monitor_Config(void);
uint8_t UBX_UART_Port_Config(void);
uint8_t UBX_Poll_Config(void);
uint8_t UBX_Information_Messages_Config(void);
uint8_t UBX_Messages_Config(uint8_t UBX_Class, uint8_t UBX_ID);
//uint8_t UBX_Message_Rate_Config(enum _uBlox_Class,  enum _uBlox_ID, int rate);
uint8_t UBX_Antenna_Control_Config(void);
uint8_t UBX_Remote_Inventory_Config(void);
uint8_t UBX_Data_Logger_Config(void);
uint8_t UBX_Time_Pulse_Parmater_Config(void);
uint8_t UBX_Odometer_Config(void);
uint8_t UBX_Geofencing_Config(void);
uint8_t UBX_Dynamic_Seed_Config(void);
uint8_t UBX_Reset_Odometer(void);

#ifdef __cplusplus
}
#endif

#endif /* UBLOX_CONFIG_H_ */

