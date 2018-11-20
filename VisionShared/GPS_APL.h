/*
 * GPS_APL.h
 *
 *  Created on: Mar 8, 2017
 *      Author: FrancoisHattingh
 */

#ifndef GPS_APL_H_
#define GPS_APL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _GPS_Data_Type
{
	int32_t Longitude;				// Unit : decimal degrees, Scaling: 10^(-7)
	int32_t Latitude;				// Unit : decimal degrees, Scaling: 10^(-7)

	uint32_t HorizontalAccuracy;	// Unit : millimeters, Scaling: -
	uint32_t VerticalAccuracy;		// Unit : millimeters, Scaling: -

	uint8_t FixType;				// Unit : enum _uBlox_Fix_Type, Scaling: -

	int32_t Speed;					// Unit : millimeters/second, Scaling: -
	uint32_t SpeedAccuracy;			// Unit : millimeters/second, Scaling: -

	int32_t HeadingMotion;			// Unit : degrees, Scaling: 10^(-5)
	int32_t HeadingVehicle;			// Unit : degrees, Scaling: 10^(-5)
	uint32_t HeadingAccuracy;		// Unit : degrees, Scaling: 10^(-5)

	int32_t SeaLevel;				// Unit : millimeters, Scaling: -

	uint8_t NumberOfSat;			// Unit : -, Scaling: -
	uint8_t antenna_status;			// Unit : enum _uBlox_Antenna_State, Scaling: -
	uint16_t flags;					// Unit : function uAPL_Get_Flag(uint16_t, uint8_t), Scaling: -

	uint8_t _Date[4];				// _Date[3] = Century, _Date[2] = Year, _Date[1] = Month, _Date[0] = Day,
	uint8_t _Time[4];				// _Time[3] = NotUsed, _Time[2] = Hour, _Time[1] = Minutes, _Time[0] = Seconds,
	uint8_t _Validity;

	uint32_t Distance;				// Unit : meters, Scaling: -
	uint32_t TotalDistance;			// Unit : meters, Scaling: -

	uint8_t FixAge;					// Unit : seconds, Scaling: -
} GPS_Data_Type;

typedef struct _GPS_Transponder_Type
{
	int32_t Longitude;				// Unit : decimal degrees, Scaling: 10^(-7)
	int32_t Latitude;				// Unit : decimal degrees, Scaling: 10^(-7)

	int32_t SeaLevel;				// Unit : millimeters, Scaling: -

	uint32_t HorizontalAccuracy;	// Unit : millimeters, Scaling: -
	uint32_t VerticalAccuracy;		// Unit : millimeters, Scaling: -

	int32_t Speed;					// Unit : millimeters/second, Scaling: -
	int32_t HeadingVehicle;			// Unit : degrees, Scaling: 10^(-5)

	uint8_t FixType;				// Unit : enum _uBlox_Fix_Type, Scaling: -
	uint8_t FixAge;					// Unit : seconds, Scaling: -

} GPS_Transponder_Type;

typedef enum
{
	No_Fix = 0x00,
	Dead_Reckoning = 0x01,
	Fix_2D = 0x02,
	Fix_3D = 0x03,
	GPS_Dead_Reckoning = 0x04,
	Time_Only = 0x05
}_uBlox_Fix_Type;

typedef enum
{
	No_State = 0x00,
	Dont_Know = 0x01,
	OK = 0x02,
	Short = 0x03,
	Open = 0x04,
}_uBlox_Antenna_State;

typedef enum
{
	GNSS_Fix_OK = 0x00,
	Differential_Corrections = 0x01,
	Power_Management_State = 0x02,
	Heading_Valid = 0x03,
	Carrier_Phase = 0x04
}_uBlox_Flags;

uint8_t uAPL_send_master_message(uint8_t Last_Fix);
void uAPL_antenna_message(void);
void uAPL_overall_distance_message(void);
extern uint8_t UBX_Reset_Odometer(void);
uint8_t uAPL_Get_Flag(uint16_t uBlox_Flag, _uBlox_Flags Flag);

#ifdef __cplusplus
}
#endif

#endif /* GPS_APL_H_ */

