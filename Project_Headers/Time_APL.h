/*
 * Time_APL.h
 *
 *  Created on: 11 Dec 2018
 *      Author: NeilPretorius
 */

#ifndef TIME_APL_H_
#define TIME_APL_H_

#include "Global_Variables.h"

extern RTC_HandleTypeDef hrtc;

#ifdef __cplusplus
extern "C" {
#endif



void Get_RTCTime(void);
void Get_RTCDate(void);
void Set_RTCDateTime(uint8_t parse_data[]);

typedef struct
{
	  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
	                                 This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected.
	                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected */

	  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
	                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

	  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
	                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

	  uint8_t WeekDay;  		/*!< Specifies the RTC Date WeekDay.
	                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

	  uint8_t Month;    		/*!< Specifies the RTC Date Month (in BCD format).
	                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

	  uint8_t Date;     		/*!< Specifies the RTC Date.
	                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */

	  uint8_t Year;     		/*!< Specifies the RTC Date Year.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 99 */

} TimeDate_Data_Type;

#ifdef __cplusplus
}
#endif

#endif /* TIME_APL_H_ */
