#ifndef MY_RF_SETTINGS_H
#define MY_RF_SETTINGS_H

#include "hal_rf.h"

extern HAL_RF_CONFIG myRfConfig_250;
extern const HAL_RF_CONFIG myRfConfig_40;

extern uint8_t myPaTable[];	
extern uint8_t myPaTableLen;

extern uint8_t PA_setting;	//-30dB -20dB -15dB -10dB  0dB   5dB   7dB   10dB
extern const uint8_t PAsettings[];

#endif //MY_RF_SETTINGS_H
