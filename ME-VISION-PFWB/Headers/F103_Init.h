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

//defines
#define IWDG_reload_val 0xFFF

//Variables made public


//Functions made public
void Init_103(void);
void stm32f10x_sysclk_conf(void);

#endif /* F207_INIT_H_ */
