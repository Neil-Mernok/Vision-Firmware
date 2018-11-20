/* $Id$ */

/*****************************************************************************
 *
 * Copyright 2007
 * Nanotron Technologies
 *
 * Author: S. Radtke
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * Description :
 *    This file contains the project defines for the standalone atmega
 *    ntrx driver.
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

#ifndef _CONFIG_H
#define _CONFIG_H

/////////////////////////
//#define SIMULATE_USER_DATA
/////////////////////////
//#define CSMA NA_TxPhCarrSensModeOff_VC_C        
//#define CSMA NA_TxPhCarrSensModeSymbols_VC_C    
//#define CSMA NA_TxPhCarrSensModeRssi_VC_C       
#define CSMA NA_TxPhCarrSensModeSymbolsRssi_VC_C
/////////////////////////

/* used for libtimer, maximum available timers which can run simutanusly*/
#define CONFIG_NUM_TIMERS 4

/* use the console interface for communication with the application */
//#define CONFIG_CONSOLE 1

/* allow some printf output during program execution */
//#define CONFIG_PRINTF 1

/* size in char of the USART's input queue */
#define CONFIG_CONSOLE_QUEUE_SIZE (16)

/* size in char of the maximal printf line */
#define CONFIG_PRINTF_LINE_SIZE (64)

/* size in char of the maximal console line */
#define CONFIG_CONSOLE_LINE_SIZE (64)

/* select the time between to nanoNET TRX chip recalibrations in ms */
#define CONFIG_NTRX_RECAL_DELAY (6000)

/* select the time between last packet and timeout indication in ms */
#define CONFIG_NTRX_PHY_RX_TIMEOUT (5000)

/* version.revision number of the firmware */
#define CONFIG_REG_MAP_NR (501)
/* use only necessary shadow registers */
// #define CONFIG_NTRX_NO_SREG 1

/* use leds for tx/rx indication */
//#define CONFIG_TRAFFIC_LED 1
//
///* use led for recalibration indication */
//#define CONFIG_RECALIB_LED 1
//// #define CONFIG_USE_TX_LED 1
//#define CONFIG_USE_RX_LED 1
//#define CONFIG_USE_ERR_LED 1
//#define CONFIG_USE_CAL_LED 1

/* use auto recalibration */
#define CONFIG_NTRX_AUTO_RECALIB 1

/* use when external power amplifier is present */
#define CONFIG_NTRX_RF_TX_EXT_PAMP_OUT 1

/* use syncword */
#define CONFIG_DEFAULT_SYNCWORD {0xab,0x69,0xca,0x94,0x92,0xd5,0x2c,0xab}


/* All available trx modes */
#define CONFIG_NTRX_22MHZ_1000NS  1
#define CONFIG_NTRX_22MHZ_2000NS  1
#define CONFIG_NTRX_22MHZ_4000NS  1
#define CONFIG_NTRX_80MHZ_500NS   1
#define CONFIG_NTRX_80MHZ_1000NS  1
#define CONFIG_NTRX_80MHZ_2000NS  1
#define CONFIG_NTRX_80MHZ_4000NS  1
// #define CONFIG_DEFAULT_TRX_22MHZ_1000NS 1
// #define CONFIG_DEFAULT_TRX_22MHZ_2000NS 1
// #define CONFIG_DEFAULT_TRX_22MHZ_4000NS 1
// #define CONFIG_DEFAULT_TRX_80MHZ_500NS  1
//#define CONFIG_DEFAULT_TRX_80MHZ_1000NS 1
// #define CONFIG_DEFAULT_TRX_80MHZ_2000NS 1
#define CONFIG_DEFAULT_TRX_80MHZ_4000NS 1
#if 	  (!defined (CONFIG_DEFAULT_TRX_22MHZ_1000NS) \
		&& !defined (CONFIG_DEFAULT_TRX_22MHZ_2000NS) \
		&& !defined (CONFIG_DEFAULT_TRX_22MHZ_4000NS) \
		&& !defined (CONFIG_DEFAULT_TRX_80MHZ_500NS) \
		&& !defined (CONFIG_DEFAULT_TRX_80MHZ_1000NS) \
		&& !defined (CONFIG_DEFAULT_TRX_80MHZ_2000NS) \
		&& !defined (CONFIG_DEFAULT_TRX_80MHZ_4000NS))
#ifdef CONFIG_NTRX_80MHZ_1000NS
#define CONFIG_DEFAULT_TRX_80MHZ_1000NS 1
#warning No default mode set. Set to 80Mhz 1us!
#else
#error Please select a default mode in config.h
#endif
#endif

//#define CONFIG_ALIVE_LED	1

#define CONFIG_MAX_ARQ 		3
#define CONFIG_TX_PWR 		0x3F					// max value is 63 = 0x3F
#define CONFIG_MAX_LOG_CHANNEL	4

/* use internal default chirp matrix */
//#define CONFIG_IQ_MATRIX_ROM 1

#define CONFIG_MAX_PACKET_SIZE 128
#define CONFIG_NTRX_IRQ 1
// #define CONFIG_SIMPLE_DRV 1
#endif /* _CONFIG_H */
