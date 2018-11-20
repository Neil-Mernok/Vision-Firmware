/*
 * F207_Init.h
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */

#ifndef F207_INIT_H_
#define F207_INIT_H_
//includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

//defines
//#define IWDG_reload_val 2000
#define IWDG_reload_val 0xFFF

//Variables made public


//Functions made public
void Init_103(int lf_freq, int can_baud, int uart_baud);
void stm32f10x_sysclk_conf(void);
void Watchdog_reset_and_reload(uint16_t watchdog_ms);

#ifdef __cplusplus
}
#endif

#endif /* F207_INIT_H_ */
