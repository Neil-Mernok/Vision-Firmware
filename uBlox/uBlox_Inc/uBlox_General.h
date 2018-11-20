/*
 * uBlox_General.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef UBLOX_GENERAL_H_
#define UBLOX_GENERAL_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_Offset			2
#define Checksum_Offset 	2
#define Identifier_Offset 	6
#define Max_Message_Size 	300

typedef enum
{
	UBX_1 = 0xB5,
	UBX_2 = 0x62,
	Class_NAV = 0x01, // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
	Class_RXM = 0x02, // Receiver Manager Messages: Satellite Status, RTC Status
	Class_INF = 0x04, // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
	Class_ACK = 0x05, // Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
	Class_CFG = 0x06, // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
	Class_UPD = 0x09, // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
	Class_MON = 0x0A, // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
	Class_AID = 0x0B, // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
	Class_TIM = 0x0D, // Timing Messages: Time Pulse Output, Time Mark Results
	Class_ESF = 0x10, // External Sensor Fusion Messages: External Sensor Measurements and Status Information
	Class_MGA = 0x13, // Multiple GNSS Assistance Messages: Assistance data for various GNSS
	Class_LOG = 0x21, // Logging Messages: Log creation, deletion, info and retrieval
	Class_SEC = 0x27, // Security Feature Messages
	Class_HNR = 0x28, // High Rate Navigation Results Messages: High rate time, position, speed, heading
}_uBlox_Class;

typedef enum
{
	ID_PRT = 0x00,     	// Port Configuration for UART
	ID_CON_MSG = 0x01,     	// Message configuration, setting message rate, and polling messages
	ID_INF = 0x02,     	// Information message
	ID_INI = 0x01, // Aiding position, time, frequency, clock drift
	ID_HUI = 0x02, // GPS Health, UTC, ionosphere parameters
	ID_RST = 0x04, // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
	ID_DAT = 0x06, // Datum
	ID_TP = 0x07,  // Time Pulse Parameters for Time Pulse
	ID_RATE = 0x08, // Navigation/Measurement Rate Settings
	ID_CFG = 0x09, // Clear, Save and Load configurations
	ID_RXM = 0x11, //Power management
	ID_ANT = 0x13, // Antenna Control Settings
	ID_SBAS = 0x16, // SBAS receiver subsystem (i.e. WAAS, EGNOS, MSAS).
	ID_NMEA = 0x17, //NMEA protocol
	ID_ODO = 0x1E,     	// Odometer, Low-speed COG Engine Settings
	ID_NAVX5 = 0x23, // Navigation Engine Expert Settings
	ID_NAV5 = 0x24, // Navigation Engine Settings
	ID_ALM = 0x30, // Aiding Almanac Data
	ID_TP5 = 0x31,  	// Time Pulse Parameters for Time Pulse
	ID_EPH = 0x31, // GPS Aiding Ephemeris Data
	ID_AOP = 0x33, // AssistNow Autonomous data
	ID_RINV = 0x34,    	// Contents of remote inventory
	ID_ITFM = 0x39,    	// Jamming/Interference Monitor configuration
	ID_PM2 = 0x3B,     	// extended Power Management
	ID_GNSS = 0x3E,
	ID_LOGFILTER = 0x47,// This message can be used to configure the data logger, i.e. to enable/disable the log recording and to get/set the position entry filter settings.
	ID_HNR = 0x5C, // High Navigation Rate Settings
	ID_DOSC = 0x61, // Disciplined oscillator configuration
	ID_GEOFENCE = 0x69, // Gets or sets the geofencing configuration
	ID_FIXSEED = 0x84, 	// Programming the fixed seed for the host
	ID_DYNSEED = 0x85, // Programming the dynamic seed for the host
	ID_PMS = 0x86,     	//Power Mode Setup (Predefined power settings)

}_uBlox_ID;

typedef enum
{
	ID_ACK_NAK = 0x00, // Message Not-Acknowledged
	ID_ACK_ACK = 0x01, // Message Acknowledged
}_uBlox_ACK_ID;

typedef enum
{
	ID_NAV_POSECEF = 0x01, 	// Position Solution in ECEF
	ID_NAV_POSLLH = 0x02,	// Geodetic Position Solution
	ID_NAV_STATUS = 0x03, 	// Position Solution in ECEF
	ID_NAV_SOL = 0x06,		// Navigation Solution Information
	ID_NAV_PVT = 0x07,  	// Navigation Position Velocity Time Solution
	ID_NAV_ODO = 0x09, 		// Odometer Solution
	ID_NAV_RESETODO = 0x10, // Reset Odometer Solution
	ID_NAV_TIMEUTC = 0x21,  // UTC Time Solution
	ID_NAV_SAT = 0x35		// Satellite Information
}_uBlox_NAV_ID;

typedef enum
{
	ID_SEC_SIGN = 0x00, 	// Signature of a previous message
	ID_SEC_UNIQID = 0x01, 	// Unique Chip ID
}_uBlox_SEC_ID;

typedef enum
{
	ID_MON_VER = 0x04, 	// Unique Chip ID
	ID_MON_HW = 0x09, 	// Signature of a previous message
}_uBlox_MON_ID;

typedef enum
{
	ID_HNR_PVT = 0x00, 	// Navigation Position Velocity Time Solution
}_uBlox_HNR_ID;

typedef enum
{
	Hardware_Immediately = 0x00,	 // Hardware reset (Watchdog) immediately
	Controlled_Sotware_Reset = 0x01, // Controlled Software reset
	GNSS_Only_Reset = 0x02, 		 // Controlled Software reset (GNSS only)
	Hardware_After = 0x04,			 // Hardware reset (Watchdog) after shutdown
	Controlled_GNSS_Stop = 0x08,	 // Controlled GNSS stop
	Controlled_GNSS_Start = 0x09,	 // Controlled GNSS start
}_uBlox_Reset;

typedef enum
{
	UBX_FALSE = 0x00, // Bool false
	UBX_TRUE = 0x01   // Bool true
}_uBlox_Bool;

typedef enum
{
	UBX_FAIL = 0,
	UBX_PASS = 1
}_uBlox_General;

typedef struct
{
	uint8_t Header[2];
	uint8_t Class;
	uint8_t ID;
	uint8_t Length[2];
	uint8_t Data[Max_Message_Size];
	uint8_t Checksum[2];

	// reserved - The contents of these elements
	// should be ignored in output messages and must be set to zero in input messages.
}_uBlox_Structure;

typedef struct
{
	_uBlox_Structure uBlox_Structure;
	uint16_t Size;
	uint8_t Acknowlegde_Required;
	uint8_t Acknowledge_State;
	uint8_t Counter;
	uint32_t Time_Sent;

}_uBlox_Message;

typedef struct
{
	uint8_t Date[4];
	uint8_t Time[4];

}_Module_Information;

_uBlox_Message uBlox_TX;
_uBlox_Message uBlox_Poll;

uint16_t UBX_Checksum(_uBlox_Message* Check_Message);
uint8_t UBX_Checksum_Compare(_uBlox_Message* Compare_Message);
uint16_t UBX_Conjugate_uint16(uint8_t First, uint8_t Second);
uint32_t UBX_Conjugate_uint32(uint8_t First, uint8_t Second, uint8_t Third, uint8_t Fourth);
uint8_t UBX_Calculate_Age(uint8_t Current_Fix);
uint8_t UBX_Compress_uint32(uint32_t Value, uint8_t Max_Value);

#ifdef __cplusplus
}
#endif

#endif /* UBLOX_GENERAL_H_ */

