/*
 * Startup.c
 * Created on: Aug 2014
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#include "Startup.h"

/**
 * @brief  Configures the LCU System, including startup testing etc.
 * @param  None
 * @retval None
 */
bool Init_board(void)
{
	// Configure GPIOs
	Config_Ports();
	
	/****************************************************************/
	/*Can be used to check main clock speed and bus speeds*/
	/****************************************************************/
	RCC_GetClocksFreq(&RCC_Clocks);
	if(RCC_Clocks.HCLK_Frequency == 72000000)
		return true;
	else
		return false;
	/****************************************************************/
}
