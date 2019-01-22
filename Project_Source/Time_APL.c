/*
 * Time_APL.c
 *
 *  Created on: 11 Dec 2018
 *      Author: NeilPretorius
 */

#include "Time_APL.h"
#include "Vision_Parameters.h"

void Get_RTCTime(void)
{
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	Vision_Status.DateTime.Hours = sTime.Hours;
	Vision_Status.DateTime.Minutes = sTime.Minutes;
	Vision_Status.DateTime.Seconds = sTime.Seconds;
}


void Get_RTCDate(void)
{
	RTC_DateTypeDef sDate;
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	Vision_Status.DateTime.Date = sDate.Date;
	Vision_Status.DateTime.Month = sDate.Month;
	Vision_Status.DateTime.Year = sDate.Year;
	Vision_Status.DateTime.WeekDay = sDate.WeekDay;
}


void Set_RTCDateTime(uint8_t parse_data[])
{
	RTC_DateTypeDef setDate;
	RTC_TimeTypeDef setTime;

	setTime.Seconds = parse_data[0];
	setTime.Minutes = parse_data[1];
	setTime.Hours = parse_data[2];
	setDate.Date = parse_data[3];
	setDate.Month = parse_data[4];
	setDate.Year = parse_data[5];

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
