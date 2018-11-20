/* $Id$ */
/**
 * @file ntrxutil.h
 * @date 2007-Dez-11
 * @author S.Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Utility functions to operate the nanoLOC chip.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This module contains utility functions to operate the nanoLOC chip.
 *
 * $Revision: 7207 $
 * $Date: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 */
/*
 * $Log$
 */

#ifndef NTRXUTIL_H
#define NTRXUTIL_H

#include 	"config.h"
#include 	"ntrxtypes.h"
#include    "registermap.h"

/** @brief Structure for configuration settings
 *
 * To initialize the nanoLOC chip to a specific transmission mode
 * all necessary parameters are combined in this structure.
 */
typedef struct
{
    uint8_t    bw;			/**< Bandwidth 					*/
    uint8_t    sd;			/**< Symbolduration 			*/
    uint8_t    sr;			/**< Symbolrate 				*/
    uint8_t    fdma;			/**< Fdma (22MHz) or 80MHz mode */
    uint8_t    fixnmap;		/**< Fixed or map mode 			*/
	uint8_t	 fec;
	double rangingConst_FECon;	/**< Ranging constant fec on		*/
	double rangingConst_FECoff;	/**< Ranging constant fec off	*/
	uint16_t	 rangingTimeout;/**< cbo add comment 			*/
	uint16_t	 rangingFastTimeout; /**< cbs add comment		*/
}PhyModeSetT;

extern PhyModeSetT modeSet;

/** @brief structure type for all layer configuration settings
 *
 * To be IEEE layer complient, all configuration data is stored
 * in one data structure that can be set or read by an upper layer.
 */
typedef struct
{
	uint8_t	CAMode;			/**< cca/csma mode */
	uint32_t titxstart;		/**< hwclock of last tx start cmd, used for CA timeout */
    uint8_t	currentChannel;	/**< selected channel. */
	uint8_t	logChannel;		/**< logical channel id. */

 	uint8_t	txPower;		/**< tx power setting for the transmitter. */
	uint8_t	rxState;		/**< receiver state. */
	uint8_t	txState;		/**< transmitter state. */

	uint8_t	macAddr0[6];	/**< 1st MAC address of nanoLOC chip. */
	uint8_t	macAddr1[6];	/**< 2nd MAC address of nanoLOC chip. */
	uint8_t	txAddrSel;		/**< selection for MAC addr 0 / 1. */
	uint8_t	arqMax;			/**< max number of retransmissions before no ack 0 - 14. */
	uint8_t	arqMode;		/**< hardware acks enabled or disabled */
	uint8_t	addrMatch; 		/**< promisc / auto mode. */
	uint8_t	rawMode;		/**< raw / auto mode. */
	uint8_t	trxMode;		/**< trx mode. */
	uint32_t recalInterval;  /**< time interval in ms for recalibration */
	uint32_t lastRecalibration;  /**< last recalibration */
	uint32_t phyRxTimeout;	/**< timeout if no packet received [ms] */
	uint8_t	pwrDownMode;	/**< 0 full 1 pad 2 up */
	uint8_t	frameType;      /**< supported frame types */
	uint8_t	testmode;		/**< set cont. chirp or carrier mode */
	uint8_t	syncword[8];	/**< syncword for communication */
	uint8_t 	rxOn;			/**< is rx already started */
	uint8_t	fec;			/**< fec on or off */
	uint8_t	pwrUp;			/**< mark power up condition */
	uint8_t	pwrDown;		/**< mark ntrx state */
	uint8_t		ant;			/**< mark last selected antenna */
} PhyPIB;

extern bool_t  ntrxRun;
extern uint8_t ntrxIrqStatus[2];
extern uint8_t ntrxBBStatus;
extern uint8_t ntrxState;
extern uint8_t ntrxCal;


#define SHADOWREGCOUNT        128
/* COldfire timer for 66Mhz */
//#define DELAY_20us      hwdelay(40) /* 20 */
//#define DELAY_100us     hwdelay(130)
//#define DELAY_800us     hwdelay(3200) /* 1100 */
//#define DELAY_20ms  { hwdelay(8000); hwdelay(8000); hwdelay (8000); hwdelay (3000); }
#define RfTxOutputPowerData_DEFAULT         0x3f
#define RfTxOutputPowerReq_DEFAULT          0x3f
#define NTRX_MAX_CHANNEL_ID	17

#define TxIDLE      0x00
#define TxSEND      0x01
#define TxWAIT      0x02

#define NoCAL       0X00
#define TxCAL       0x01
#define RxCAL       0x02
#define AllCAL      0x03


#define txIrqStatus (ntrxIrqStatus[0])
#define rxIrqStatus (ntrxIrqStatus[1])


enum PWROUTTYPE
{
    pwrOutData = 0,
    pwrOutReqst = 1
};

extern uint8_t ntrxShadowReg[SHADOWREGCOUNT];

void NTRXSetIndexReg (uint8_t page);
void NTRXSetChannel (uint8_t value);
void NTRXPowerdownMode (uint8_t mode, uint32_t ms);
void NTRXResetSettings (void);
void NTRXStartBbTimer (int16_t startvalue);
void NTRXStopBbTimer (void);
void NTRXSetupTrxMode (uint8_t fdma, uint8_t sd, uint8_t sr);
bool_t NTRXAllCalibration (void);
void NTRXRxLoCalibration (void);
void NTRXTxLoCalibration (void);
bool_t NTRXAllFakeCalibration (void);
void NTRXSetRxIqMatrix (uint8_t bandwidth, uint8_t symbolDur);
void NTRXSetTxIqMatrix (uint8_t bandwidth, uint8_t symbolDur);
void NTRXInitShadowRegister (void);
bool_t NTRXCheckVerRev (void);
void NTRXGetPayload(uint8_t *payload, uint8_t len);
void NTRXSetTestChirpMode (bool_t value);
void NTRXSetTestCarrierMode (bool_t value);
void NTRXSetSyncWord (uint8_t *value);
void NTRXPowerdownModeDIO( uint8_t mode , uint8_t dio );
void NTRXPowerupFromPadMode( void );
void NTRXDioOutput32kHz( /* Dio[3:0] */ uint8_t bits);
void NTRXRXEnable(bool_t enable);
uint8_t NTRXGetRssi( void );


#define NA_80MHz            0
#define NA_22MHz            1

#define NA_125k_S  NA_SymbolRate125kSymbols_VC_C
#define NA_250k_S  NA_SymbolRate250kSymbols_VC_C
#define NA_500k_S  NA_SymbolRate500kSymbols_VC_C
#define NA_1M_S    NA_SymbolRate1MSymbols_VC_C
#define NA_2M_S    NA_SymbolRate2MSymbols_VC_C

#define NA_500ns    NA_SymbolDur500ns_C
#define NA_1us      NA_SymbolDur1000ns_C
#define NA_2us      NA_SymbolDur2000ns_C
#define NA_4us      NA_SymbolDur4000ns_C
#define NA_8us      NA_SymbolDur8000ns_C
#define NA_16us     NA_SymbolDur16000ns_C

#define NA_MAP_MODE 0
#define NA_FIX_MODE 1

/* timeout for ranging in [ms] */
#define RANGING_TIMEOUT_13MS (uint16_t)13000
#define RANGING_TIMEOUT_9MS  (uint16_t)9000
#define RANGING_TIMEOUT_7MS  (uint16_t)7000
#define RANGING_TIMEOUT_6MS  (uint16_t)6000
#define RANGING_TIMEOUT_5MS  (uint16_t)5000
#define RANGING_TIMEOUT_4MS  (uint16_t)4000

/* recalibrationtime [ms] */
#define RECAL_TIME	(uint16_t)20000




/**
 * @def NTRX_RSSI_BUSY_THRESHOLD
 * @brief If the measured RSSI value is @em below this value the radio channel
 * should be assumed busy.
 * @see NTRXInitRssi
 */
#define NTRX_RSSI_BUSY_THRESHOLD	(0x10)
// #define NTRX_RSSI_BUSY_THRESHOLD	(0x1A)


#endif /* NTRXUTIL_H */
