/*
 * Vision_HAL.h
 *
 *  Created on: Jul 27, 2015
 *      Author: Kobus
 */

#ifndef VISION_HAL_H_
#define VISION_HAL_H_

#ifdef STM32F10X_CL

#endif
#ifdef STM32F4XX

#endif

#ifndef MAX
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#define MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

//#define LF_TX_Capable

//#define FILTER_LF				// experimental

#define num_transp_total 16

// define how long a microsecond is
#define UsBase	16

#endif /* VISION_HAL_H_ */
