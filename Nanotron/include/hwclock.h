/**
 * @file hwclock.h
 * @date 2007-Dez-11
 * @author S. Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Timer support for AVR.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the function definitions for the utility functions
 *    of the AVR hardware timer.
 */
#ifndef	_HWCLOCK_H
#define	_HWCLOCK_H

#include "config.h"
#include "ntrxtypes.h"
#define NKEYS 3

extern uint32_t jiffies;

extern	uint32_t	hwclock(void);
extern  void 		HWDelayms( uint16_t ms );
extern  void 		HWDelayus( uint16_t us );
extern	void		hwclock_tune(int8_t tuningDirection);

#endif	/* _HWCLOCK_H */
