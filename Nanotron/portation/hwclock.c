/**
 * @file hwclock.c
 * @date 2007-Dez-11
 * @author S. Rohdemann, S. Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Timer support for AVR.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the source code for the utility functions
 *    of the Hardware Timer Function.
 *
 */
#include	"config.h"
#ifndef __OPTIMIZE__
#	define __OPTIMIZE__ 0
#endif
#include    "ntrxtypes.h"
#include	"hwclock.h"
//#include 	"keys.h"
#include	"STMport.h"
#include 	"portation.h"

#define CONFIG_HWCLOCK_USE_CRYSTAL
#ifdef CONFIG_SPI_TRACE
extern int traceOn;
#endif

volatile bool_t key_flags[] = { FALSE, FALSE, FALSE };
uint32_t	jiffies = 0;

/***************************************************************************/
/**
 * @brief Delay processing for n microseconds.
 * @param us this is the delay in microseconds
 *
 * This function is used for waiting before continue with
 * program execution. Interrupts are still processed.
 * Because of the high inaccuracy of the delay function
 * this function tries to compensate the delay error
 * by adding an offset.
 */
void HWDelayus( uint16_t us )
{
	if (us)
		DelayUs(us-1);		
}



/***************************************************************************/
/**
 * @brief Delay processing for n milliseconds.
 * @param ms this is the delay in milliseconds
 *
 * This function is used for waiting ms before continue with
 * program execution. Interrupts are still processed.
 */
void HWDelayms( uint16_t ms )
{
	Delay(ms);
}


/***************************************************************************/
/**
 * @brief return system clock in milliseconds
 * @returns time in milliseconds
 *
 * This function returns the elapsed time since
 * program start in milliseconds.
 */
uint32_t	hwclock(void)
{
	return time_now();
}



/***************************************************************************/
/**
 * @brief Modifies the time interval between any two subsequent timer ticks.
 * @param tuningDirection +1 to speed up the hwclock, -1 to speed down,
 * 0 to reset to default.
 */
void hwclock_tune(int8_t tuningDirection)
{
#	ifndef CONFIG_HWCLOCK_USE_CRYSTAL
	uint8_t reloadVal = hwclTimerReloadVal;

	switch (tuningDirection)
	{
		case -1:
			if (reloadVal > 0) reloadVal--;
			break;

		case 0:
			reloadVal = TIMER_RELOAD_VALUE;
			break;

		case 1:
			if (reloadVal < 0xFF) reloadVal++;
			break;

		default:
			break;
	}

	ENTER_TASK;
	hwclTimerReloadVal = reloadVal;
	LEAVE_TASK;
#	endif /* !CONFIG_HWCLOCK_USE_CRYSTAL */
}

