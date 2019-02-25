/*
 * Time_APL.c
 *
 *  Created on: 11 Dec 2018
 *      Author: NeilPretorius
 */

#include "Time_APL.h"
#include "rtc.h"
#include "Vision_Parameters.h"
extern RTC_HandleTypeDef hrtc;

void Get_RTCTime(void)
{
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	Vision_Status.DateTime.Hours = RTC_Bcd2ToByte(sTime.Hours);
	Vision_Status.DateTime.Minutes = RTC_Bcd2ToByte(sTime.Minutes);
	Vision_Status.DateTime.Seconds = RTC_Bcd2ToByte(sTime.Seconds);
}


void Get_RTCDate(void)
{
	RTC_DateTypeDef sDate;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	Vision_Status.DateTime.Date = RTC_Bcd2ToByte(sDate.Date);
	Vision_Status.DateTime.Month = RTC_Bcd2ToByte(sDate.Month);
	Vision_Status.DateTime.Year = RTC_Bcd2ToByte(sDate.Year);
	Vision_Status.DateTime.WeekDay = RTC_Bcd2ToByte(sDate.WeekDay);
}


void Set_RTCDateTime(uint8_t parse_data[])
{
	RTC_DateTypeDef setDate;
	RTC_TimeTypeDef setTime;

	setTime.Seconds = RTC_ByteToBcd2(parse_data[0]);
	setTime.Minutes = RTC_ByteToBcd2(parse_data[1]);
	setTime.Hours = RTC_ByteToBcd2(parse_data[2]);
	setDate.Date = RTC_ByteToBcd2(parse_data[3]);
	setDate.Month = RTC_ByteToBcd2(parse_data[4]);
	setDate.Year = RTC_ByteToBcd2(parse_data[5]);

  if(HAL_RTC_SetDate(&hrtc,&setDate,FORMAT_BCD) != HAL_OK)
  {
	/* Initialization Error */
//	Error_Handler();
  }

  if(HAL_RTC_SetTime(&hrtc,&setTime,FORMAT_BCD) != HAL_OK)
    {
      /* Initialization Error */
      //Error_Handler();
    }

  /*##-3- Writes a data in a RTC Backup data Register0 #######################*/
//    HAL_RTCEx_BKUPWrite(&RtcHandle,RTC_BKP_DR0,0x32F2);
}
