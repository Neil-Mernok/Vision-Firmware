/* $Id$ */

/*****************************************************************************
 *
 * Copyright 2002
 * Nanotron Technologies
 *
 * Author: S. Rohdemann
 *
 * BuildNumber = "BuildNumber : 7951";
 *
 * Description :
 *    This file contains the type- / data- and function definitions
 *    for the skeleton portation
 *
 * $Revision: 7207 $
 * $Date: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 *
 ****************************************************************************/

/*
 * $Log$
 */

#ifndef	_PORTATION_H
#define	_PORTATION_H

#include "ntrxtypes.h"
#include "hwclock.h"
#include <stdio.h>

#ifndef ENTER_TASK
#define ENTER_TASK HAL_NVIC_DisableIRQ(N_uIRQ_Channel);
#endif
#ifndef LEAVE_TASK
#define LEAVE_TASK HAL_NVIC_EnableIRQ(N_uIRQ_Channel);
#endif


/*This macro should be used for debugging output.*/
#define PRINTF(args, ...)	printf_P(PSTR(args), ##__VA_ARGS__)

/*This macro should be used to supress debugging output.*/
#define NoDebugging(args, ...)



#endif	/* _PORTATION_H */
