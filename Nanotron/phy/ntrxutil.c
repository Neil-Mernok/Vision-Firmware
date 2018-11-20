/* $Id$ */

/**
 * @file ntrxutil.c
 * @date 2007-Dez-4
 * @author S.Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Functions for the physical layer helper functions.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the source code for the implementation of the
 *    NTRX helper functions
 *
 * $Revision: 7207 $
 * $Date: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 */
/*
 * $Log$
 */

#include	"portation.h"
#include    <stdlib.h>
#include    <string.h>
#include    "config.h"
#include    "ntrxtypes.h"
#include    "ntrxutil.h"
#include    "hwclock.h"
#include    "nnspi.h"
#include    "ntrxiqpar.h"
#include 	"phy.h"

#define CONST

extern bool_t NTRXRestart (void);
extern PhyPIB phyPIB;

uint8_t loTarVal[5][2] = {{0x59, 0x68},			/* current value */
							{0x59, 0x68}, 			/* Channel 0: 2.44175 GHz, center of the band,
																		shall be used for 80 MHz mode */
#ifdef COUNTRY_CODE_1
/* USA */
							{0x66, 0x62},			/* Channel 0: 2.412 GHz non overlapping chanel no. ( USA ) */
							{0x66, 0x67},			/* Channel 5: 2.437 GHz non overlapping chanel no. ( USA ) */
							{0x66, 0x6c}			/* Channel 10: 2.462 GHz non overlapping chanel no. ( USA ) */
#else
/* COUNTRY_CODE_2 */
/* EUROPE */
							{0x66, 0x62},			/* Channel 0: 2.412 GHz non overlapping chanel no. ( Europe ) */
							{0x66, 0x68},			/* Channel 6: 2.442 GHz non overlapping chanel no. ( Europe ) */
							{0x66, 0x6e}			/* Channel 12: 2.472 GHz non overlapping chanel no. ( Europe ) */
#endif
};


uint8_t loTarValTx[2] = {0x59, 0x68};			/* current value */
uint8_t loTarValRx[2] = {0x6f, 0x68};			/* current value */

#ifdef CONFIG_DEFAULT_TRX_80MHZ_1000NS
uint8_t loTarValRx_418kHz[] = {0x6f, 0x68};
#endif
#ifndef CONFIG_DEFAULT_TRX_80MHZ_1000NS
uint8_t loTarValRx_418kHz[] = {0x5c, 0x68};
#endif

uint8_t	ntrxShadowReg[SHADOWREGCOUNT];	/**< shadow register for the nanoLOC chip. Needed for write-only register */
PhyModeSetT modeSet;						/**< group of parameters that define a transmission mode. */
uint8_t 	ntrxIrqStatus[2];				/**< interrupt status register */
uint8_t   ntrxBBStatus;	/**< baseband status register */
uint8_t   ntrxState;						/**< state of the nanoLOC chip */
uint8_t 	lfdma = 0, lsd = 0, lsr = 0;	/**< ranging variables */

uint8_t tx_caps[3]={0,0,0}, rx_caps[3]={0,0,0}; /**< used for fake calibration */

uint8_t 	ntrxInitShadowReg[]  =
{
/* 0x00 */   0,   0,   0,   0,   0,   0,   0,   6,
/* 0x08 */   0,   0,   0,   0,   0,   0,   0,   0,
/* 0x10 */   0,   0,   0,   0,   0,   0,  64,   0,
/* 0x18 */  32,  64,   0,  32,   0,   0,   0,   3,
/* 0x20 */   6, 152,  12,   0,   2,  63,  30,   6,
/* 0x28 */   0,   0, 171,  44, 213, 146, 148, 202,
/* 0x30 */ 105, 171,  48,   0,   0,   0,   0,   0,
/* 0x38 */   0,   0,   0,   0, 224,   4,   0,   1,
/* 0x40 */   3,   7,   0,   3,  63,  63,  15,  15,
/* 0x48 */ 115,   0,  16,  16,  67,  20,  16,   0,
/* 0x50 */   0, 127,   0,   0,   0,   0,   0,   0,
/* 0x58 */   0,   0,  11,  95,   5,   7, 213,  98,
/* 0x60 */   0,   0,   0,  12,  10,   0,   0,   0,
/* 0x68 */   0,   0,   0,   0,   0,   0,   0,   0,
/* 0x70 */   0,   0,   0,   0,   0,   0,   0,   0,
/* 0x78 */   0,   0,   0,   0,   0,  80,   0,   0
};

/**
 * @brief resets the transmission mode parameters.
 *
 * This function sets the three parameters that define a transmission mode.
 *
 */
/**************************************************************************/
void NTRXResetSettings (void)
/**************************************************************************/
{
	lfdma 	= 0;
	lsd 	= 0;
	lsr 	= 0;
}

/**
 * @brief initialize shadow registers.
 *
 * This function initializes the shadow register array to the initial
 * values in the transceiver when it is powered up or reset.
 *
 */
/**************************************************************************/
void NTRXInitShadowRegister (void)
/**************************************************************************/
{
    // set shadow registers to the initial value of the nanoNET TRX chip
    memcpy(ntrxShadowReg, ntrxInitShadowReg, SHADOWREGCOUNT);
}

/**
 * @brief set the logical channel.
 * @param value the logical channel id
 *
 * This function sets the logical channel for transmission and reception
 *
 */
/**************************************************************************/
void NTRXSetChannel (uint8_t value)
/**************************************************************************/
{
	/*
	 * first check for a valid parameter. If the parameter is out of range
	 * do nothing.
	 */
	if (value > NTRX_MAX_CHANNEL_ID)
		return;

	/*
	 * in the first slot of the loTarVal array the currently used channel
	 * (center frequency) is stored. The other 4 are used to hold the
	 * predefined channels. When a new channel is selected, the value
	 * is copied form the list to the first slot. For fdma only the first
	 * channel is stored. The other 14 channels are calculated with the
	 * first channel as the basis.
	 */
	switch (value)
	{
		case 0:	loTarValTx[0] = loTarVal[1][0];
				loTarValTx[1] = loTarVal[1][1];
				loTarValRx[0] = loTarValRx_418kHz[0];
				loTarValRx[1] = loTarValRx_418kHz[1];
				break;

		default:
				loTarValTx[0] = loTarVal[1 + value][0];
				loTarValTx[1] = loTarVal[1 + value][1];
				loTarValRx[0] = loTarVal[1 + value][0];
				loTarValRx[1] = loTarVal[1 + value][1];
				break;


	}
}

/**
 * @brief set the index register.
 * @param page address index
 *
 * This function sets the index registers for the transceiver chip.
 * The chip address space is divided into 4 sections a 1kbyte.
 *
 */
/**************************************************************************/
void NTRXSetIndexReg (uint8_t page)
/**************************************************************************/
{
    if (page != ntrxShadowReg[NA_RamIndex_O])
    {
        ntrxShadowReg[NA_RamIndex_O] = page;
        NTRXSPIWriteByte (NA_RamIndex_O, page);
    }
}

/**
 * @brief set the index register.
 * @param page address index
 *
 * This function sets the index registers for the transceiver chip
 * The chip address space is divided into 4 sections a 1kbyte.
 *
 */
/**************************************************************************/
void NTRXSetRamIndex (uint8_t page)
/**************************************************************************/
{
    page &= 0x03;
    if (page != (ntrxShadowReg[NA_RamIndex_O] & 0x03))
    {
        ntrxShadowReg[NA_RamIndex_O] &= 0xf0;
        ntrxShadowReg[NA_RamIndex_O] |= page;
        page = ntrxShadowReg[NA_RamIndex_O];
        NTRXSPIWriteByte (NA_RamIndex_O, page);
    }
}

/**
 * @brief calibrates the local ocsillator.
 *
 * This function callibrates the local oscillator of the receiver part.
 * It is not called directly but from within the NTRXAllCalibration
 * function.
 *
 */
/**************************************************************************/
void NTRXRxLoCalibration (void)
/**************************************************************************/
{
    uint8_t data[3];

	NTRXRXEnable(FALSE);

	NTRXSPIWriteByte (NA_EnableLO_O, (uint8_t)(ntrxShadowReg[NA_EnableLO_O] | (1 << NA_EnableLO_B) | (1 << NA_EnableLOdiv10_B)));


	NTRXSPIWriteByte (NA_UseLoRxCaps_O, (uint8_t)(ntrxShadowReg[NA_UseLoRxCaps_O] | (1 << NA_UseLoRxCaps_B)));

    data[0] = 0x03;
	NTRXSPIWriteByte (NA_LoIntsReset_O, data[0]);
    NTRXSPIWrite (NA_LoTargetValue_O, loTarValRx, 2);

    do
    {
        NTRXSPIReadByte (NA_LoIntsRawStat_O, &data[0]);
    } while ((data[0] & (1 << NA_LoTuningReady_B)) != (1 << NA_LoTuningReady_B));

	/* Read out caps values if needed */
    //NTRXSPIRead (0x16, data, 3);
	NTRXSPIRead (0x16, rx_caps, 3);

	NTRXSPIWriteByte (NA_UseLoRxCaps_O, ntrxShadowReg[NA_UseLoRxCaps_O]);
	NTRXSPIWriteByte (NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O]);

	/*
	 * start receiver if it was started before calibration
	 */
	if (phyPIB.rxState == PHY_RX_ON) NTRXRXEnable(TRUE);
}

/**
 * @brief calibrates the local ocsillator.
 *
 * This function callibrates the local oscillator of the transmitter part.
 * It is not called directly but from within the NTRXAllCalibration
 * function.
 *
 */
/**************************************************************************/
void NTRXTxLoCalibration (void)
/**************************************************************************/
{
    uint8_t data[3];

	NTRXRXEnable(FALSE);

	NTRXSPIWriteByte (NA_EnableLO_O, (uint8_t)(ntrxShadowReg[NA_EnableLO_O] | (1 << NA_EnableLO_B) | (1 << NA_EnableLOdiv10_B)));


    data[0] = 0x03;
    NTRXSPIWriteByte (NA_LoIntsReset_O, data[0]);

    NTRXSPIWrite (NA_LoTargetValue_O, loTarValTx, 2);

    do
    {
        NTRXSPIReadByte (NA_LoIntsRawStat_O, &(data[0]));
    } while ((data[0] & (1 << NA_LoTuningReady_B)) != (1 << NA_LoTuningReady_B));

	/* Read out caps values if needed */
    //NTRXSPIRead (0x19, data, 3);
	NTRXSPIRead (0x19, tx_caps, 3);

	NTRXSPIWriteByte (NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O]);

	/*
	 * start receiver if it was started before calibration
	 */
	if (phyPIB.rxState == PHY_RX_ON) NTRXRXEnable(TRUE);
}

/**
 * @brief copy values from last calibration into chip.
 */
/**************************************************************************/
bool_t NTRXAllFakeCalibration (void)
/**************************************************************************/
{
	NTRXSPIWrite (0x16, rx_caps, 3);
	NTRXSPIWrite (0x19, tx_caps, 3);
	return TRUE;
}

/**
 * @brief calibrates the local ocsillator.
 *
 * This function callibrates the local oscillator for both the transmitter
 * and the receiver part. This function should be called regularly to maintain
 * a stable and in spec transmission.
 *
 */
/**************************************************************************/
bool_t NTRXAllCalibration (void)
/**************************************************************************/
{
	/*
	 * make sure the we are not currently transmitting. This would destroy
	 * the partialy send message.
	 */
	if (ntrxState == TxSEND) return FALSE;

	/*
	 * stop the receiver and clear the buffers
	 */
	NTRXRXEnable(FALSE);

	/*
	 * calibrate transmitter and receiver
	 */
    NTRXRxLoCalibration ();
    NTRXTxLoCalibration ();

	/*
	 * start receiver if it was started before calibration
	 */
	if (phyPIB.rxState == PHY_RX_ON)
	{
		NTRXRXEnable(TRUE);
	}
	return TRUE;
}

/**
 * @brief read out version an revision of the transceiver chip.
 * @returns TRUE if equal, False if not equal or SPI access failed.
 *
 * This function compares the version and revision register with
 * the expected Version and Revision number from the software.
 *
 */
/**************************************************************************/
bool_t NTRXCheckVerRev (void)
/**************************************************************************/
{
    uint8_t buff[2];

    NTRXSPIRead (NA_Version_O, buff, 1);
    NTRXSPIRead (NA_Revision_O, &(buff[1]), 1);

#   if ((CONFIG_PRINTF) && (CONFIG_LOGO))
    DEBUG_PHY("ver:%d, rev:%d\n", buff[0], buff[1]);
#   endif

    if (buff[0] == NA_Version_I && buff[1] == NA_Revision_I)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * @brief read out number of retransmission attempts.
 * @returns tx arq maximum value in the range of (0x0...0xE).
 *
 * This function returns the number of packet retransmissions used by
 * the nanoLOC chip.
 * The value is taken from the shadow ram.
 *
 */
/**************************************************************************/
uint8_t NTRXGetTxArqMax (void)
/**************************************************************************/
{
	return (uint8_t)((ntrxShadowReg[NA_TxArqMax_O] >> 4) & 0x0f);
}

/**
 * @brief returns the setting vor the CRC-2 mode of the transceiver chip.
 * @returns On (TRUE) or Off (FALSE).
 *
 * This function returns the CRC-2 mode settings in the transceiver chip.
 * The value is taken from the shadow ram.
 */
/**************************************************************************/
bool_t NTRXGetRxCrc2Mode (void)
/**************************************************************************/
{
    return ((bool_t)((ntrxShadowReg[NA_RxCrc2Mode_O] & (1<< NA_RxCrc2Mode_B)) >> NA_RxCrc2Mode_B));
}



/**
 * @brief sets the capacitors to tune the frequency.
 *
 * This function executes the recommended procedure to tune the frequency
 * by en- and disableing capacitors.
 */
/**************************************************************************/
void NTRXFctCal (void)
/**************************************************************************/
{
    int capsVal = 7;
    int targetValMax = 174;
    int targetValMin = 152;
    int fctSum = 0;
    int ctref;
    unsigned char payload;

    payload = ( 1 << NA_FctClockEn_B ) | capsVal;
    NTRXSPIWriteByte (NA_ChirpFilterCaps_O, payload);
	HWDelayus( 10 ); /* 10 us */

    while ((capsVal < 16) && (capsVal >= 0))
    {
        fctSum = 0;
        for (ctref = 0; ctref < 4; ctref++)
        {
    		payload = (uint8_t)(( 1 << NA_FctClockEn_B ) | ( 1 << NA_StartFctMeasure_B ) | capsVal );
		    NTRXSPIWriteByte (NA_ChirpFilterCaps_O, payload);
			HWDelayus( 15 ); /* 10 us */

            NTRXSPIReadByte (NA_FctPeriod_O, &payload);
            fctSum += payload;
        }
        if (fctSum >= targetValMax)
        {
            capsVal--;
        }
        else if (fctSum <= targetValMin)
        {
            capsVal++;
        }
        else
        {
            payload = (uint8_t)( capsVal );
			ntrxShadowReg[NA_ChirpFilterCaps_O] = payload;
		    NTRXSPIWriteByte (NA_ChirpFilterCaps_O, payload);
			return;
        }
    }
    payload = 6;
	ntrxShadowReg[NA_ChirpFilterCaps_O] = payload;
    NTRXSPIWriteByte (NA_ChirpFilterCaps_O, payload);
}

/**
 * @brief sets the sync word in the transceiver.
 * @param value a pointer to the 8 byte syncword.
 *
 * This function writes the syncword in the syncword register.
 * The syncword is used for add a transceivers to a logical
 * group. Only messages with the right syncword are detected
 * (received) and processed.
 */
/**************************************************************************/
void NTRXSetSyncWord (uint8_t *value)
/**************************************************************************/
{
	NTRXSPIWrite (NA_SyncWord_O, value, 8);
}

/**
 * @brief sets transmission mode in the transceiver.
 * @param fdma this parameter determines the the mode (fdma or 80 MHz)
 * @param sd this parameter determines the symbol duration between 500ns and 8us)
 * @param sr this parameter determines the symbol rate between 125kbit and 2Mbit)
 *
 * Based on the three parameters the transceiver is initialized.
 * The iq parameter are set and the nanoLOC chip is calibrated.
 *
 */
/**************************************************************************/
void NTRXSetupTrxMode (uint8_t fdma, uint8_t sd, uint8_t sr)
/**************************************************************************/
{
	modeSet.bw = fdma;
	modeSet.sd = sd;
	modeSet.sr = sr;

#	ifndef CONFIG_IQ_MATRIX_ROM
	if (fdma == NA_22MHz)
	{
    	modeSet.fdma= TRUE;
    	modeSet.fixnmap=NA_FIX_MODE;

		ntrxShadowReg[NA_RfRxCompValueI_O] |= (0x0f << NA_RfRxCompValueI_LSB);
		ntrxShadowReg[NA_RfRxCompValueQ_O] |= (0x0f << NA_RfRxCompValueQ_LSB);
	}
	else
	{
    	modeSet.fdma= FALSE;
    	modeSet.fixnmap=NA_FIX_MODE;

		ntrxShadowReg[NA_RfRxCompValueI_O] &= (uint8_t)( ~(0x0f << NA_RfRxCompValueI_LSB ));
		ntrxShadowReg[NA_RfRxCompValueI_O] |= (uint8_t)( (0x0e << NA_RfRxCompValueI_LSB ));
		ntrxShadowReg[NA_RfRxCompValueQ_O] &= (uint8_t)( ~(0x0f << NA_RfRxCompValueQ_LSB ));
		ntrxShadowReg[NA_RfRxCompValueQ_O] |= (uint8_t)( (0x0e << NA_RfRxCompValueQ_LSB ));
	}
	NTRXSPIWriteByte (NA_RfRxCompValueI_O, ntrxShadowReg[NA_RfRxCompValueI_O] );
	NTRXSPIWriteByte (NA_RfRxCompValueQ_O, ntrxShadowReg[NA_RfRxCompValueQ_O] );
#	else
   	modeSet.fdma = TRUE;
   	modeSet.fixnmap = NA_FIX_MODE;
#	endif /* CONFIG_IQ_MATRIX_ROM */

	if( phyPIB.pwrUp == FALSE )
	{
		if (ntrxShadowReg[NA_RamIndex_O] != 0)
		{
			NTRXSPIWriteByte (NA_RamIndex_O, 0);
		}
		NTRXRXEnable(FALSE);

		/* switch off diio functions to save more power*/
  		ntrxShadowReg[NA_DioDirection_O] = 0x01;
		NTRXSPIWriteByte( NA_DioDirection_O, ntrxShadowReg[NA_DioDirection_O]);
		NTRXSPIWriteByte( NA_DioPortWe_O, 0xf );
		NTRXSPIWriteByte( NA_DioPortWe_O, 0);
	}
	else
	{
	 	//phyPIB.rxState = PHY_RX_ON;
		phyPIB.rxState = PHY_TRX_OFF;
		phyPIB.pwrUp = FALSE;
	}

#	ifndef CONFIG_IQ_MATRIX_ROM
    /* first part of initial register setting */
	ntrxShadowReg[NA_ResetBbClockGate_O] |= (1 << NA_ResetBbClockGate_B);
	ntrxShadowReg[NA_EnableBbCrystal_O] |= (1 << NA_EnableBbCrystal_B);
	NTRXSPIWrite (NA_ResetBbClockGate_O, ntrxShadowReg + NA_ResetBbClockGate_O, 2);
	HWDelayms( 5 ); /* 5 ms */ /* TODO: is this needed?? */
	ntrxShadowReg[NA_ResetBbClockGate_O] &= (uint8_t)( ~(1 << NA_ResetBbClockGate_B));
	ntrxShadowReg[NA_EnableBbClock_O] |= (1 << NA_EnableBbClock_B);
	NTRXSPIWrite (NA_ResetBbClockGate_O, ntrxShadowReg + NA_ResetBbClockGate_O, 2);
	NTRXSPIWriteByte (NA_ResetBbRadioCtrl_O, (1 << NA_ResetBbRadioCtrl_B));
	NTRXSPIWriteByte (NA_ResetBbClockGate_O, 0);
#	else
	ntrxShadowReg[NA_EnableBbCrystal_O] |= (1 << NA_EnableBbCrystal_B);
	ntrxShadowReg[NA_EnableBbClock_O] |= (1 << NA_EnableBbClock_B);

	ntrxShadowReg[NA_ResetBbClockGate_O] &= (uint8_t)( ~(1 << NA_ResetBbClockGate_B));
	ntrxShadowReg[NA_ResetBbRadioCtrl_O] &= (uint8_t)( ~(1 << NA_ResetBbRadioCtrl_B));
	NTRXSPIWrite (NA_ResetBbClockGate_O, ntrxShadowReg + NA_ResetBbClockGate_O, 2);
#	endif /* CONFIG_IQ_MATRIX_ROM */


#	ifndef CONFIG_IQ_MATRIX_ROM
    /* Tx Part */
	ntrxShadowReg[NA_TxArqMax_O] = (CONFIG_MAX_ARQ << NA_TxArqMax_LSB);
	ntrxShadowReg[NA_CsqUsePhaseShift_O] &= (uint8_t)( ~((1 << NA_CsqUsePhaseShift_B)
							| (1 << NA_CsqAsyMode_B)));
	ntrxShadowReg[NA_CsqUseRam_O] |= (1 << NA_CsqUseRam_B);
	NTRXSPIWrite (NA_TxArqMax_O, ntrxShadowReg + NA_TxArqMax_O, 2);

	ntrxShadowReg[NA_EnableLO_O] &= (uint8_t)( ~((1 << NA_EnableLO_B) | (1 << NA_EnableLOdiv10_B)));
	ntrxShadowReg[NA_EnableExtPA_O] |= ((1 << NA_EnableExtPA_B)
							| (1 << NA_EnableCsqClock_B));
	NTRXSPIWrite (NA_EnableExtPA_O, ntrxShadowReg + NA_EnableExtPA_O, 7);
	ntrxShadowReg[NA_TxScrambEn_O] |= (1 << NA_TxScrambEn_B);
	ntrxShadowReg[NA_TxAddrSlct_O] &= (uint8_t)( ~(1 << NA_TxAddrSlct_B));
	NTRXSPIWrite (NA_TxArq_O, ntrxShadowReg + NA_TxArq_O , 7);
#	else
	ntrxShadowReg[NA_EnableExtPA_O] |= (1 << NA_EnableExtPA_B);
	NTRXSPIWriteByte (NA_EnableExtPA_O, ntrxShadowReg[NA_TxArqMax_O] );
#	endif

	ntrxShadowReg[NA_RxCrc2Mode_O] |= (NA_RxCrc2ModeTrigOn_BC_C << NA_RxCrc2Mode_B);
	ntrxShadowReg[NA_RxTimeBCrc1Mode_O] &= (uint8_t)( ~(NA_RxTimeBCrc1ModeOn_BC_C << NA_RxTimeBCrc1Mode_B));
	NTRXSPIWriteByte( NA_RxArqMode_O, ntrxShadowReg[NA_RxArqMode_O] );

#	ifdef CONFIG_NTRX_SNIFFER
	ntrxShadowReg[NA_RxAddrMode_O] &= (uint8_t)( ~(1 << NA_RxAddrMode_B));
	NTRXSPIWriteByte( NA_RxAddrMode_O, ntrxShadowReg[NA_RxAddrMode_O] );
#	endif

	/* enable fast calibration tuning */
	ntrxShadowReg[NA_LoEnableFastTuning_O] =
		(1 << NA_LoEnableFastTuning_B) |
		(1 << NA_LoEnableLsbNeg_B) |
		(4 << NA_LoFastTuningLevel_LSB);
    NTRXSPIWriteByte (NA_LoEnableFastTuning_O, ntrxShadowReg[NA_LoEnableFastTuning_O]);



	NTRXFctCal ();

#	ifndef CONFIG_IQ_MATRIX_ROM
    NTRXSetRxIqMatrix (fdma, sd);
#	endif
	NTRXSetCorrThreshold (fdma, sd);
#	ifndef CONFIG_IQ_MATRIX_ROM
   	NTRXSetTxIqMatrix (fdma, sd);
#	endif
	NTRXSetSyncWord((uint8_t *)phyPIB.syncword);

#	ifdef CONFIG_NTRX_AUTO_RECALIB
    NTRXAllCalibration ();
#	endif /* CONFIG_NTRX_AUTO_RECALIB */

   	NTRXSetAgcValues (fdma, sd, sr);

	ntrxShadowReg[NA_GateSizeFramesync_O]  = ((NA_GateSize9Slots_VC_C << NA_GateSizeFramesync_LSB)
							| (NA_GateSize9Slots_VC_C << NA_GateSizeUnsync_LSB)
							| (NA_GateSize9Slots_VC_C << NA_GateSizeBitsync_LSB)
							| ( 1 << NA_GateAdjBitsyncEn_B )
							);

	NTRXSPIWriteByte( NA_GateSizeFramesync_O, ntrxShadowReg[NA_GateSizeFramesync_O] );

	PHYInit ();

	lfdma 	= fdma;
	lsd   	= sd;
	lsr		= sr;
}


/**
 * @brief read out the real time clock of the transceiver.
 * @param force == TRUE lets the function read the RTC anyway
 * @returns the time in 1/32768 seconds (ca. 30 us)
 *
 * This function reads out the real time clock of the transceiver
 *
 */
/**************************************************************************/
uint32_t NTRXGetRtc (bool_t force)
/**************************************************************************/
{
	/*FIXME disabled for debugging!*/
	static uint32_t currentRTC;					/**< stores the last RTC value*/
	static uint32_t currentHwclock = 0xFFFFFFFF;	/**< stores the hwclock value when RTC was read*/
	uint32_t now = hwclock();

	/*
	Avoid redundant access to RTC to save time.
	force == TRUE lets the function read the RTC anyway.
	*/
	if ((now != currentHwclock) || (force))
	{
		uint8_t rtc[5];
		uint8_t reg;


		/*read current RTC*/
		//ENTER_TASK;
		reg = (uint8_t)( ntrxShadowReg[NA_RtcCmdRd_O] | ( 1 << NA_RtcCmdRd_B ));
		NTRXSPIWriteByte(NA_RtcCmdRd_O, reg);
		HWDelayus( 130 );
		NTRXSPIRead(NA_RamRtcReg_O + 1, rtc, 4);
		reg = 0;
		NTRXSPIWriteByte(NA_RtcCmdRd_O, reg);

		/*update current RTC and hwclock entries*/
		currentRTC = (
				(((uint32_t)rtc[0])   <<  0)
				| (((uint32_t)rtc[1]) <<  8)
				| (((uint32_t)rtc[2]) << 16)
				| (((uint32_t)rtc[3]) << 24)
		);
		currentHwclock = now;
		LEAVE_TASK;
	}
	return currentRTC;
}

/**
 * @brief read out the real time clock of the transceiver.
 * @param mode 0 is full and 1 is for pad mode
 * @param seconds wake up time from now
 *
 * This function starts the alarm clock if the parameter seconds is
 * greater than 0 and brings the transceiver into the requested powerdown mode.
 * There are two powerdown modes supported: powerdown mode full and
 * powerdown mode pad. The second mode can be used to supervise the dig-IO
 * pins of the nanoLOC chip.
 *
 */
/**************************************************************************/
void NTRXPowerdownMode (uint8_t mode, uint32_t ms)
/**************************************************************************/
{
	uint8_t 	value, valNull = 0;
	uint8_t 	i;
	uint8_t 	wakeupTime8;
	uint32_t wakeupTime32;

	/*Stop receiver*/
	NTRXRXEnable(FALSE);

	if (ms > 0)
	{
		ms = (uint32_t)( ms / (float) 7.8125 ); /* 7.8125ms steps */
		/* Round errors leads to always to short measurements.
		 * add +1 to be sure not to sleep shorter than expected */
		ms += 1;

		wakeupTime32 = ms + NTRXGetRtc(TRUE);

		/*write down wake up time*/
		for (i = NA_WakeUpTimeWe_LSB; i <= NA_WakeUpTimeWe_MSB; i++)
		{

			/*convert 32bit integer wakeup time -> RTC value*/
			wakeupTime8 = (uint8_t)(wakeupTime32 & 0xFF);
			wakeupTime32 >>= 8;

			/*write one of the 3 bytes into the RtcWakeUpTime Register*/
			NTRXSPIWriteByte( NA_WakeUpTimeByte_O, wakeupTime8 );

			/*generate Strobe to store value in the right place*/
			value = (uint8_t)( 1 << i);
			NTRXSPIWriteByte( NA_WakeUpTimeWe_O, value );
			NTRXSPIWriteByte( NA_WakeUpTimeWe_O, valNull );
		}
	}

	value = ( 0x00 | ( 1 << NA_ResetBbClockGate_B ));
	NTRXSPIWriteByte( NA_PowerDown_O, value );
	value = 0;
	NTRXSPIWriteByte( 0x8, value );
	/*set wakeup parameter*/
	if (ms > 0)
	{
		value = (uint8_t)((0x00
			| ( 1 << NA_EnableWakeUpRtc_B )
			| ( 1 << NA_PowerUpTime_LSB )
			| (( 0x01 & mode ) << NA_PowerDownMode_B )));
	}
	else
	{
		value = (uint8_t)((( 0x01 & mode ) << NA_PowerDownMode_B ));
	}
	NTRXSPIWriteByte( NA_EnableWakeUpRtc_O, value );

	value =(0x00
  			| ( 1 << NA_PowerDown_B )
			| ( 1 << NA_ResetBbClockGate_B ));
	NTRXSPIWriteByte( NA_PowerDown_O, value );
	phyPIB.pwrUp = TRUE;
	phyPIB.pwrDown = TRUE;
}

/**
 * @brief starts the baseband timer.
 * @param startvalue countdown value in us
 *
 * Starts a countdown from startvalue to zero ( in us )
 *
 */
/**************************************************************************/
void NTRXStartBbTimer (int16_t startvalue)
/**************************************************************************/
{
    NTRXSPIWrite (NA_BasebandTimerStartValue_O, (uint8_t *)(&startvalue), 2);
}

/**
 * @brief stops the baseband timer.
 *
 * Stops the baseband timer. The baseband timer is recommended for very
 * short timer intervals < 1 ms
 *
 */
/**************************************************************************/
void NTRXStopBbTimer (void)
/**************************************************************************/
{
	NTRXSPIWriteByte (NA_ClearBasebandTimerInt_O, (uint8_t)(ntrxShadowReg[NA_ClearBasebandTimerInt_O] | (1 << NA_ClearBasebandTimerInt_B)));
}

/**
 * @brief Testmode for chip measurement.
 *
 * This function enables or disables the chip test mode. In this mode
 * a continuous chirp is transmitted. This requires to swich off the receiver.
 * Note: This function should be interrupted periodically and the transmitter
 * recalibrated. Otherwise the frequency might be out of range.
 */
/**************************************************************************/
void NTRXSetTestChirpMode (bool_t value)
/**************************************************************************/
{
	NTRXRXEnable(FALSE);

	if (value == TRUE)
	{
		ntrxShadowReg[NA_CsqUsePhaseShift_O] |= (1 << NA_CsqUsePhaseShift_B);
		NTRXSPIWriteByte (NA_CsqUsePhaseShift_O, ntrxShadowReg[NA_CsqUsePhaseShift_O]);

		ntrxShadowReg[NA_EnableLO_O] |= ((1 << NA_EnableLO_B) | (1 << NA_EnableLOdiv10_B) | (1 << NA_EnableCsqClock_B));
		NTRXSPIWriteByte (NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O]);

		ntrxShadowReg[NA_EnableTx_O] |= (1 << NA_EnableTx_B);
		NTRXSPIWriteByte (NA_EnableTx_O, ntrxShadowReg[NA_EnableTx_O]);

		ntrxShadowReg[NA_EnableExtPA_O] |= 0x20;
		NTRXSPIWriteByte (NA_EnableExtPA_O, ntrxShadowReg[NA_EnableExtPA_O]);

		ntrxShadowReg[NA_CsqUseRam_O] |= 0x80;
		NTRXSPIWriteByte (NA_CsqUseRam_O, ntrxShadowReg[NA_CsqUseRam_O]);
	}
	else
	{
		ntrxShadowReg[NA_CsqUseRam_O] &= (uint8_t)( ~(0x80));
		NTRXSPIWriteByte (NA_CsqUseRam_O, ntrxShadowReg[NA_CsqUseRam_O]);

		ntrxShadowReg[NA_EnableExtPA_O] &= (uint8_t)( ~(0x20));
		NTRXSPIWriteByte (NA_EnableExtPA_O, ntrxShadowReg[NA_EnableExtPA_O]);

		ntrxShadowReg[NA_EnableTx_O] &= (uint8_t)( ~(1 << NA_EnableTx_B));
		NTRXSPIWriteByte (NA_EnableTx_O, ntrxShadowReg[NA_EnableTx_O]);

		ntrxShadowReg[NA_EnableCsqClock_O] &= (uint8_t)( ~(1 << NA_EnableCsqClock_B));
		ntrxShadowReg[NA_EnableLOdiv10_O] &= (uint8_t)( ~(1 << NA_EnableLOdiv10_B));
		ntrxShadowReg[NA_EnableLO_O] &= (uint8_t)( ~(1 << NA_EnableLO_B));
		NTRXSPIWriteByte (NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O]);

		ntrxShadowReg[NA_CsqUsePhaseShift_O] &= (uint8_t)( ~(1 << NA_CsqUsePhaseShift_B));
		NTRXSPIWriteByte (NA_CsqUsePhaseShift_O, ntrxShadowReg[NA_CsqUsePhaseShift_O]);
	}
}

/**
 * @brief Carriermode for chip measurement.
 *
 *
 *
 * This function enables or disables the unmodulated carrier signal test mode by
 * sending a continuous carrier. This requires to swich off the receiver.
 * Note: This function should be periodically interrupted and the transmitter
 * recalibrated. Otherwise the frequency might be out of range.
 */
/**************************************************************************/
void NTRXSetTestCarrierMode (bool_t value)
/**************************************************************************/
{
	static uint8_t fdma;

	NTRXRXEnable(FALSE);

	if (value == TRUE)
	{
		fdma = ntrxShadowReg[NA_FdmaEnable_O];
		ntrxShadowReg[NA_FdmaEnable_O] &= (uint8_t)( ~(1 << NA_FdmaEnable_B));
		NTRXSPIWriteByte (NA_FdmaEnable_O, ntrxShadowReg[NA_FdmaEnable_O]);

		ntrxShadowReg[NA_EnableLO_O] |= ((1 << NA_EnableLO_B) | (1 << NA_EnableLOdiv10_B));
		NTRXSPIWriteByte (NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O]);

		ntrxShadowReg[NA_CsqSetValue_O] = 0x1F;
		NTRXSPIWriteByte (NA_CsqSetValue_O, ntrxShadowReg[NA_CsqSetValue_O]);

		ntrxShadowReg[NA_CsqSetValue_O] |= (1 << 6);
		NTRXSPIWriteByte (NA_CsqSetValue_O, ntrxShadowReg[NA_CsqSetValue_O]);

		ntrxShadowReg[NA_CsqSetValue_O] = 0x3F;
		NTRXSPIWriteByte (NA_CsqSetValue_O, ntrxShadowReg[NA_CsqSetValue_O]);

		ntrxShadowReg[NA_CsqSetValue_O] |= (1 << 7);
		NTRXSPIWriteByte (NA_CsqSetValue_O, ntrxShadowReg[NA_CsqSetValue_O]);

		ntrxShadowReg[NA_EnableTx_O] |= (1 << NA_EnableTx_B);
		NTRXSPIWriteByte (NA_EnableTx_O, ntrxShadowReg[NA_EnableTx_O]);

		ntrxShadowReg[NA_EnableExtPA_O] |= 0x20;
		NTRXSPIWriteByte (NA_EnableExtPA_O, ntrxShadowReg[NA_EnableExtPA_O]);
	}
	else
	{
		ntrxShadowReg[NA_EnableExtPA_O] &= (uint8_t)( ~(0x20));
		NTRXSPIWriteByte (NA_EnableExtPA_O, ntrxShadowReg[NA_EnableExtPA_O]);

		ntrxShadowReg[NA_EnableTx_O] &= (uint8_t)( ~(1 << NA_EnableTx_B));
		NTRXSPIWriteByte (NA_EnableTx_O, ntrxShadowReg[NA_EnableTx_O]);

		ntrxShadowReg[NA_EnableLO_O] &= (uint8_t)( ~((1 << NA_EnableLO_B)
											| (1 << NA_EnableLOdiv10_B)
											| (1 << NA_EnableCsqClock_B)));
		NTRXSPIWriteByte (NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O]);

		ntrxShadowReg[NA_FdmaEnable_O] = fdma;
		NTRXSPIWriteByte (NA_FdmaEnable_O, ntrxShadowReg[NA_FdmaEnable_O]);
	}
}


/**************************************************************************/
void NTRXPowerupFromPadMode( void )
/**************************************************************************/
{
	ntrxShadowReg[NA_EnableBbCrystal_O] |= (1 << NA_EnableBbCrystal_B);
	NTRXSPIWriteByte(NA_EnableBbCrystal_O, ntrxShadowReg[NA_EnableBbCrystal_O]);

	HWDelayms( 5 ); /* 5 ms */

	NTRXSPIWriteByte (NA_ResetBbClockGate_O, 0);

	ntrxShadowReg[NA_EnableBbClock_O] |= (1 << NA_EnableBbClock_B);
	NTRXSPIWriteByte(NA_EnableBbCrystal_O, ntrxShadowReg[NA_EnableBbCrystal_O]);
}

/**************************************************************************/
void NTRXPowerdownModeDIO( uint8_t mode , uint8_t dio )
/**************************************************************************/
{
	/*Stop receiver*/
	NTRXRXEnable(FALSE);


	// enable DIGIO for wake-up alarm, start the alarm function,
	// rising edge causes the alarm, configure as an input pin
	ntrxShadowReg[NA_DioDirection_O] =  ( 1 << NA_DioOutValueAlarmEnable_B )
									  | ( 1 << NA_DioAlarmStart_B )
									  | ( 1 << NA_DioAlarmPolarity_B );
	NTRXSPIWriteByte ( NA_DioDirection_O, ntrxShadowReg[NA_DioDirection_O] );

	// apply settings to DIGIO port
	ntrxShadowReg[NA_DioPortWe_O] = (uint8_t)(1 << dio);
	NTRXSPIWriteByte (NA_DioPortWe_O, ntrxShadowReg[NA_DioPortWe_O]);
	ntrxShadowReg[NA_DioPortWe_O] = 0;
	NTRXSPIWriteByte (NA_DioPortWe_O, ntrxShadowReg[NA_DioPortWe_O]); // clear the write enable register

	// alarm source for powering up is DIGIO, power down mode as selected with parameter mode,
	// power up time is set (ticks)
	// 1 tick = 1/4096s ~= 244us
	// reg. 0x06 is R/W
	ntrxShadowReg[NA_EnableWakeUpDio_O] = (uint8_t)(( 1 << NA_EnableWakeUpDio_B )
										| ( NA_PowerUpTime1Ticks_C << NA_PowerUpTime_LSB )
										| ((mode & 0x01) << NA_PowerDownMode_B )); // register value is rewritten completely
	NTRXSPIWriteByte (NA_EnableWakeUpDio_O, ntrxShadowReg[NA_EnableWakeUpDio_O]);

	// turn off BB clock
	ntrxShadowReg[NA_EnableBbClock_O] &= (uint8_t)( ~(1 << NA_EnableBbClock_B));
	NTRXSPIWriteByte(NA_EnableBbClock_O, ntrxShadowReg[NA_EnableBbClock_O]);
	ntrxShadowReg[NA_ResetBbClockGate_O] = (1 << NA_ResetBbClockGate_B) | (1 << NA_ResetBbRadioCtrl_B);
	NTRXSPIWriteByte(NA_ResetBbClockGate_O, ntrxShadowReg[NA_ResetBbClockGate_O]);
	ntrxShadowReg[NA_EnableBbCrystal_O] &= (uint8_t)( ~(1 << NA_EnableBbCrystal_B));
	NTRXSPIWriteByte(NA_EnableBbCrystal_O, ntrxShadowReg[NA_EnableBbCrystal_O]);

	// go to power down mode
	ntrxShadowReg[NA_PowerDown_O] = (1 << NA_ResetBbRadioCtrl_B)
									| ( 1 << NA_ResetBbClockGate_B )
									| (1 << NA_PowerDown_B);
	NTRXSPIWriteByte (NA_PowerDown_O, ntrxShadowReg[NA_PowerDown_O]);
}

/**
 * @brief enable diio[3:0] as output with 32khz
 */
/**************************************************************************/
void NTRXDioOutput32kHz( /* Dio[3:0] */ uint8_t bits)
/**************************************************************************/
{

/* merged into one operation ...
	ntrxShadowReg[NA_DioDirection_O]     |=  (1<< NA_DioDirection_B);
	ntrxShadowReg[NA_DioAlarmPolarity_O] &= ~(1 << NA_DioAlarmPolarity_B);
	ntrxShadowReg[NA_DioAlarmPolarity_O] |=  (1 << NA_DioAlarmPolarity_B);
	ntrxShadowReg[NA_DioUsePulldown_O]   |=  (1 << NA_DioUsePulldown_B);
*/
    ntrxShadowReg[NA_DioDirection_O] = 0x21;

	NTRXSPIWriteByte( NA_DioPortWe_O, 0xf );
	NTRXSPIWriteByte( NA_DioPortWe_O, 0);

	/* both registers are contained at same offset */
	ntrxShadowReg[NA_DioAlarmPolarity_O] |=  (1 << NA_DioAlarmPolarity_B);


	NTRXSPIWriteByte( NA_DioDirection_O, ntrxShadowReg[NA_DioDirection_O]);
	NTRXSPIWriteByte( NA_DioPortWe_O, bits );
	NTRXSPIWriteByte( NA_DioPortWe_O, 0);
}

/**************************************************************************/
void NTRXRXEnable(bool_t enable)
/**************************************************************************/
{
    /*
    * start the receiver of the TRX chip
    */
	if (enable == TRUE && phyPIB.rxOn == FALSE)
	{
		NTRXSPIWriteByte (NA_RxIntsReset_O, 0x7F);
		rxIrqStatus = 0;

		NTRXSPIWriteByte (NA_RxCmdStart_O, (uint8_t)(ntrxShadowReg[NA_RxCmdStart_O]
												| (1 << NA_RxCmdStart_B)
												| (1 << NA_RxBufferCmd_LSB)
												| (1 << NA_RxBufferCmd_MSB)));

		phyPIB.rxOn = TRUE;

	}
	else if (enable == FALSE && phyPIB.rxOn == TRUE)
	{
		NTRXSPIWriteByte (NA_RxCmdStop_O, (uint8_t)(ntrxShadowReg[NA_RxCmdStop_O]
											| (1 << NA_RxCmdStop_B)
											| (1 << NA_RxBufferCmd_LSB)
											| (1 << NA_RxBufferCmd_MSB)));

		NTRXSPIWriteByte (NA_RxIntsReset_O, 0x7F);
		phyPIB.rxOn = FALSE;
		rxIrqStatus = 0;
	}
}


/************************************************************************/
uint8_t NTRXGetRssi( void )
/************************************************************************/
{
	CONST uint8_t threshold[] = {0x50, 0xE8, 0xE8, 0x00, 0x00};
	CONST uint8_t threshold2[] = {0x00, 0x8D, 0x8D, 0x8D, 0x8D};
	CONST uint8_t tmpBuffer[] = {0,0,0,0,0,0,0,0};
	uint8_t rssi;

	NTRXRXEnable( FALSE );

	// Sets SyncWord to 0 **
	NTRXSetSyncWord ( tmpBuffer );

	// CRC2 Type 1 / !FEC / TxRxCryptCrc2ModeUncrypted / TxRxCryptClkModeCryptClock
	NTRXSPIWriteByte(NA_Crc2Type_O, tmpBuffer[0] );

	// PulseDetDelay 7 **
	NTRXSPIWriteByte(NA_PulseDetDelay_O, (0x7 << NA_PulseDetDelay_LSB));

	NTRXSPIWriteByte(NA_AgcValue_O, 0x3f);

	// Do not Hold AGC / HoldAgcInBitSync = 0x7
	NTRXSPIWriteByte(NA_HoldAgcInBitSync_O, 0x07 );

	// Use Alternative AGC **
	NTRXSPIWriteByte(NA_UseAlternativeAgc_O, 0x05 );

	// Sets correlation threshold register to 0
    NTRXSetIndexReg (0x32);
    NTRXSPIWrite (0x80, threshold, 5);
	NTRXSetIndexReg (0x00);

	// Enable LO
	NTRXSPIWriteByte(NA_EnableLO_O, 0x1);
	NTRXRXEnable( TRUE );

	HWDelayms( 1 ); /* measurement time */
	NTRXSPIReadByte(NA_AgcGain_O, &rssi);
	HWDelayus( 250 ); /* otherwise all hell breaks loose ... */

	/*restore previous configuration*/
	NTRXRXEnable( FALSE );
	NTRXSetSyncWord ( phyPIB.syncword );
	NTRXSPIWriteByte(NA_Crc2Type_O, ntrxShadowReg[NA_Crc2Type_O] );
	NTRXSPIWriteByte(NA_PulseDetDelay_O, ntrxShadowReg[NA_PulseDetDelay_O] );
	NTRXSPIWriteByte(NA_AgcValue_O, ntrxShadowReg[NA_AgcValue_O] );
	NTRXSPIWriteByte(NA_HoldAgcInBitSync_O, ntrxShadowReg[NA_HoldAgcInBitSync_O] );
	NTRXSPIWriteByte(NA_UseAlternativeAgc_O, ntrxShadowReg[NA_UseAlternativeAgc_O] );
    NTRXSetIndexReg (0x32);
    NTRXSPIWrite (0x80, threshold2, 5);
	NTRXSetIndexReg (0x00);
	NTRXSPIWriteByte(NA_EnableLO_O, ntrxShadowReg[NA_EnableLO_O] );

	if (phyPIB.rxState == PHY_RX_ON) NTRXRXEnable(TRUE);

	return( rssi );
}
