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

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

	Vision_Status.DateTime.Hours = sTime.Hours;
	Vision_Status.DateTime.Minutes = sTime.Minutes;
	Vision_Status.DateTime.Seconds = sTime.Seconds;
}


void Get_RTCDate(void)
{

	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	Vision_Status.DateTime.Date = sDate.Date;
	Vision_Status.DateTime.Month = sDate.Month;
	Vision_Status.DateTime.Year = sDate.Year;
	Vision_Status.DateTime.WeekDay = sDate.WeekDay;
}
