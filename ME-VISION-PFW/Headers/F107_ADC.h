/*
 * F107_ADC.h
 *
 *  Created on: Jul 16, 2015
 *      Author: Kobus
 */

#ifndef F107_ADC_H_
#define F107_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

void ADC_ConvCpltCallback(void);	
void RTC_Alarm_callback(void);	 

void ADC_Setup(uint32_t* data_address);
void HAL_ADC_Start_DMA(void);


#ifdef __cplusplus
}
#endif


#endif /* F107_ADC_H_ */
