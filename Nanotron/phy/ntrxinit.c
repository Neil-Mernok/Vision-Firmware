/* $Id$ */
/**
 * @file ntrxinit.c
 * @date 2007-Dez-4
 * @author S.Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief Functions for the initialization of the nanoLOC transceiver.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the source code for the implementation of the
 * NA5TR1 initialisation.
 *
 * $Revision: 7207 $
 * $Date: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-11-25 10:58:46 +0100 (Mi, 25 Nov 2009) $
 */
/*
 * $Log$
 */

#include	"config.h"
#include	"ntrxtypes.h"
#include    "ntrxutil.h"
#include    "hwclock.h"
#include    "nnspi.h"
#include	<string.h>
#include 	<stdio.h>

extern PhyPIB phyPIB;
extern void error_handler (int16_t err);

/*
 * do not change this syncword, since not every 64 bit sequence is valid
 */
const uint8_t TRX_SYNC_WORD[] =  	/**< Default sync word for transceiver. */
	CONFIG_DEFAULT_SYNCWORD;

/**************************************************************************/
void NTRXSetDefaultMode( void )
/**************************************************************************/
{

#	ifdef CONFIG_DEFAULT_TRX_22MHZ_1000NS
	modeSet.bw = NA_22MHz;
	modeSet.sd = NA_1us;
	modeSet.sr = NA_1M_S;
	modeSet.fdma= TRUE;
	modeSet.fec = FALSE;
	modeSet.fixnmap=NA_FIX_MODE;
	modeSet.rangingConst_FECon=0.0; /* no ranging allowed */
	modeSet.rangingTimeout=0; /* no ranging allowed */
	modeSet.rangingFastTimeout=0;   /* no ranging allowed */
#	endif /* CONFIG_DEFAULT_TRX_22MHZ_1000NS */

#	ifdef CONFIG_DEFAULT_TRX_22MHZ_2000NS
	modeSet.bw = NA_22MHz;
	modeSet.sd = NA_2us;
	modeSet.sr = NA_500k_S;
	modeSet.fdma= TRUE;
	modeSet.fec = FALSE;
	modeSet.fixnmap=NA_FIX_MODE;
	modeSet.rangingConst_FECon=0.0; /* no ranging allowed */
	modeSet.rangingTimeout=0; /* no ranging allowed */
	modeSet.rangingFastTimeout=0;   /* no ranging allowed */
#	endif /* CONFIG_DEFAULT_TRX_22MHZ_2000NS */

#	ifdef CONFIG_DEFAULT_TRX_22MHZ_4000NS
	modeSet.bw = NA_22MHz;
	modeSet.sd = NA_4us;
	modeSet.sr = NA_250k_S;
	modeSet.fdma= TRUE;
	modeSet.fec = FALSE;
	modeSet.fixnmap = NA_FIX_MODE;
	modeSet.rangingConst_FECon=0.0; /* no ranging allowed */
	modeSet.rangingTimeout=0; /* no ranging allowed */
	modeSet.rangingFastTimeout=0;   /* no ranging allowed */
#	endif /* CONFIG_DEFAULT_TRX_22MHZ_4000NS */

#	ifdef CONFIG_DEFAULT_TRX_80MHZ_500NS
	modeSet.bw = NA_80MHz;
	modeSet.sd = NA_500ns;
	modeSet.sr = NA_2M_S;
	modeSet.fdma= FALSE;
	modeSet.fec = FALSE;
	modeSet.fixnmap=NA_FIX_MODE;
	modeSet.rangingConst_FECon=68.929336; /**/
	modeSet.rangingConst_FECoff=53.991800; /**/
	modeSet.rangingTimeout		=	RANGING_TIMEOUT_6MS;
	modeSet.rangingFastTimeout	=	RANGING_TIMEOUT_4MS;
#	endif /* CONFIG_DEFAULT_TRX_80MHZ_500NS */

#	ifdef CONFIG_DEFAULT_TRX_80MHZ_1000NS
	modeSet.bw = NA_80MHz;
	modeSet.sd = NA_1us;
	modeSet.sr = NA_1M_S;
	modeSet.fdma= FALSE;
	modeSet.fec = FALSE;
	modeSet.fixnmap=NA_FIX_MODE;
 	modeSet.rangingConst_FECon=122.55664361;
	modeSet.rangingConst_FECoff=92.494781680; /* after gate size9 measurements */
	modeSet.rangingTimeout		=	RANGING_TIMEOUT_7MS;
	modeSet.rangingFastTimeout	=	RANGING_TIMEOUT_5MS;
#	endif /* CONFIG_DEFAULT_TRX_80MHZ_1000NS */

#	ifdef CONFIG_DEFAULT_TRX_80MHZ_2000NS
	modeSet.bw = NA_80MHz;
	modeSet.sd = NA_2us;
	modeSet.sr = NA_500k_S;
	modeSet.fdma= FALSE;
	modeSet.fec = FALSE;
	modeSet.fixnmap=NA_FIX_MODE;
 	modeSet.rangingConst_FECon=229.554470462;
 	modeSet.rangingConst_FECoff=169.55441720;
	modeSet.rangingTimeout		=	RANGING_TIMEOUT_9MS;
	modeSet.rangingFastTimeout	=	RANGING_TIMEOUT_7MS;
#	endif /* CONFIG_DEFAULT_TRX_80MHZ_2000NS */

#	ifdef CONFIG_DEFAULT_TRX_80MHZ_4000NS
	modeSet.bw = NA_80MHz;
	modeSet.sd = NA_4us;
	modeSet.sr = NA_250k_S;
	modeSet.fdma= FALSE;
	modeSet.fec = FALSE;
	modeSet.fixnmap=NA_MAP_MODE;	//NA_FIX_MODE;
	//modeSet.rangingConst_FECon=445.648702415;			-- what it used to be...
	modeSet.rangingConst_FECon=445.593916660618;		// new value found
	modeSet.rangingConst_FECoff=325.587060636; /* [s] */

	modeSet.rangingTimeout		=	RANGING_TIMEOUT_13MS;
	modeSet.rangingFastTimeout	=	RANGING_TIMEOUT_9MS;
#	endif /* CONFIG_DEFAULT_TRX_80MHZ_4000NS */


	phyPIB.pwrUp = TRUE;
}

/**
 * @brief Initializing of the transceiver chip.
 *
 * This function initializes all registers of the nanoLOC transceiver chip.
 * This function should only be called once.
 *
 * If this function is called for the first time, the structure settingVal is
 * set to the predefined default mode values. After setting up the SPI interface
 * this function compares the version and revision register of the chip with
 * the values expected by the driver. If these don't match, the software will
 * call an error function and halts. The reason for this drastic reaction
 * is that the driver can not garantie a non interfering behaviour.
 *
 */
/**************************************************************************/
bool_t NTRXInit(void)
/**************************************************************************/
{
	/* init SPI HW&SW */
	InitSPI();
	NTRXPowerOnReset();

	/* initialize shadow registers */
	NTRXInitShadowRegister ();
	/* configure SPI output of chip MSB first / push pull */
	NTRXSetupSpiInterface ();

	HWDelayus( 1 ); //!! Some short delay seems necessary here??
	/* check connection and firmware version and revision */
	if (!NTRXCheckVerRev())
	{
		return( FALSE );
	}
	
	memcpy (phyPIB.syncword, TRX_SYNC_WORD, 8 );
    if (modeSet.bw != NA_22MHz)
	{
		NTRXSetChannel( 0 );
	}
	else
	{
		NTRXSetChannel( 2 );
	}
    NTRXSetupTrxMode (modeSet.bw, modeSet.sd, modeSet.sr);
    
    ///////////////////////////////////////////////////////////////////////////////
    ////  Code for more robust communication (“ANnLOC0802-Modifying-NtrxInit”)
//    NTRXSetRegister (NA_TxScrambInit, NA_TxScrambInit_I);
//    NTRXSetRegister (NA_TxScrambEn, TRUE);
//    NTRXSetRegister (NA_GateAdjFramesyncEn, FALSE);
//    NTRXSetRegister (NA_GateAdjBitsyncEn, TRUE);
//    NTRXSetRegister (NA_GateSizeFramesync, NA_GateSize5Slots_VC_C);
//    NTRXSetRegister (NA_GateSizeBitsync, NA_GateSize5Slots_VC_C);
//    NTRXSetRegister (NA_GateSizeUnsync, NA_GateSize5Slots_VC_C);
//    NTRXSetRegister (NA_AgcHold, TRUE);
//    NTRXSetRegister (NA_AgcDefaultEn, TRUE);
//    NTRXSetRegister (NA_AgcValue, 35);
//    NTRXSetRegister (NA_AgcRangeOffset, 12);
//    NTRXSetRegister (NA_UseAlternativeAgc, FALSE);
    ///////////////////////////////////////////////////////////////////////////////
    
	phyPIB.pwrDown = FALSE;
	return( TRUE );
}

