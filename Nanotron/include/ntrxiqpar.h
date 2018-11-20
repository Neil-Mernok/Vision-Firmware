/* $Id$ */
/**
 * @file ntrxiqpar.h
 * @date 2007-Dez-11
 * @author S. Radtke, O. Tekyar
 * @c (C) 2007 Nanotron Technologies
 * @brief Utility functions for nanoLOC transmit mode setting.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the prototypes for setting the
 *       transmission modes of the nanoLOC chip.
 *
 * $Revision: 6839 $
 * $Date: 2009-08-13 09:00:45 +0200 (Do, 13 Aug 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-08-13 09:00:45 +0200 (Do, 13 Aug 2009) $
 */
/*
 * $Log$
 */

#ifndef NTRXIQPAR_H
#define NTRXIQPAR_H

#include    "config.h"
#include    "ntrxtypes.h"

void NTRXSetAgcValues (uint8_t bandwidth, uint8_t symbolDur, uint8_t symbolRate);
void NTRXSetTxIqMatrix (uint8_t bandwidth, uint8_t symbolDur);
void NTRXSetRxIqMatrix (uint8_t bandwidth, uint8_t symbolDur);
void NTRXSetCorrThreshold (uint8_t bandwidth, uint8_t symbolDur);

#endif
