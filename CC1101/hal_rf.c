/******************************************************************************
 Filename: hal_rf.c

 This file contains functions for accessing the CC1100/CC2500 family
 of RF ICs from Texas Instruments.

 Copyright 2007 Texas Instruments, Inc.
 ******************************************************************************/

//#include "hal_types.h"
//#include "hal_board.h"
#include "hal_spi.h"
//#include "hal_spi_config.h"
//#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_rf.h"
#include "cc1100.h"

//----------------------------------------------------------------------------------
//  void halRfResetChip(void)
//
//  DESCRIPTION:
//    Resets the chip using the procedure described in the datasheet.
//----------------------------------------------------------------------------------
void halRfResetChip(void)
{
	// Toggle chip select signal
	HAL_SPI_CS_DEASSERT;
	halMcuWaitUs(30);
	HAL_SPI_CS_ASSERT;
	halMcuWaitUs(30);
	HAL_SPI_CS_DEASSERT;
	halMcuWaitUs(45);

	// reset command
	halSpiStrobe(CC1100_SRES);
}

//----------------------------------------------------------------------------------
//  void halRfConfig(const HAL_RF_CONFIG* rfConfig, const uint8* rfPaTable, uint8 rfPaTableLen)
//
//  DESCRIPTION:
//    Used to configure the CC1100/CC2500 registers with exported register
//    settings from SmartRF Studio.
//
//  ARGUMENTS:
//    rfConfig     - register settings (as exported from SmartRF Studio)
//    rfPaTable    - array of PA table values (from SmartRF Studio)
//    rfPaTableLen - length of PA table
//
//----------------------------------------------------------------------------------
void halRfConfig(const HAL_RF_CONFIG* rfConfig, uint8_t* rfPaTable, uint8_t rfPaTableLen)
{
	halRfWriteReg(CC1100_FSCTRL1, rfConfig->fsctrl1);    // Frequency synthesizer control.
	halRfWriteReg(CC1100_FSCTRL0, rfConfig->fsctrl0);    // Frequency synthesizer control.
	halRfWriteReg(CC1100_FREQ2, rfConfig->freq2);      // Frequency control word, high byte.
	halRfWriteReg(CC1100_FREQ1, rfConfig->freq1);      // Frequency control word, middle byte.
	halRfWriteReg(CC1100_FREQ0, rfConfig->freq0);      // Frequency control word, low byte.
	halRfWriteReg(CC1100_MDMCFG4, rfConfig->mdmcfg4);    // Modem configuration.
	halRfWriteReg(CC1100_MDMCFG3, rfConfig->mdmcfg3);    // Modem configuration.
	halRfWriteReg(CC1100_MDMCFG2, rfConfig->mdmcfg2);    // Modem configuration.
	halRfWriteReg(CC1100_MDMCFG1, rfConfig->mdmcfg1);    // Modem configuration.
	halRfWriteReg(CC1100_MDMCFG0, rfConfig->mdmcfg0);    // Modem configuration.
	halRfWriteReg(CC1100_CHANNR, rfConfig->channr);     // Channel number.
	halRfWriteReg(CC1100_DEVIATN, rfConfig->deviatn);    // Modem deviation setting (when FSK modulation is enabled).
	halRfWriteReg(CC1100_FREND1, rfConfig->frend1);     // Front end RX configuration.
	halRfWriteReg(CC1100_FREND0, rfConfig->frend0);     // Front end RX configuration.
	halRfWriteReg(CC1100_MCSM0, rfConfig->mcsm0);      // Main Radio Control State Machine configuration.
	halRfWriteReg(CC1100_FOCCFG, rfConfig->foccfg);     // Frequency Offset Compensation Configuration.
	halRfWriteReg(CC1100_BSCFG, rfConfig->bscfg);      // Bit synchronization Configuration.
	halRfWriteReg(CC1100_AGCCTRL2, rfConfig->agcctrl2);   // AGC control.
	halRfWriteReg(CC1100_AGCCTRL1, rfConfig->agcctrl1);   // AGC control.
	halRfWriteReg(CC1100_AGCCTRL0, rfConfig->agcctrl0);   // AGC control.
	halRfWriteReg(CC1100_FSCAL3, rfConfig->fscal3);     // Frequency synthesizer calibration.
	halRfWriteReg(CC1100_FSCAL2, rfConfig->fscal2);     // Frequency synthesizer calibration.
	halRfWriteReg(CC1100_FSCAL1, rfConfig->fscal1);     // Frequency synthesizer calibration.
	halRfWriteReg(CC1100_FSCAL0, rfConfig->fscal0);     // Frequency synthesizer calibration.
	halRfWriteReg(CC1100_FSTEST, rfConfig->fstest);     // Frequency synthesizer calibration.
	halRfWriteReg(CC1100_TEST2, rfConfig->test2);      // Various test settings.
	halRfWriteReg(CC1100_TEST1, rfConfig->test1);      // Various test settings.
	halRfWriteReg(CC1100_TEST0, rfConfig->test0);      // Various test settings.
	halRfWriteReg(CC1100_IOCFG2, rfConfig->iocfg2);     // GDO2 output pin configuration.
	halRfWriteReg(CC1100_IOCFG0, rfConfig->iocfg0);     // GDO0 output pin configuration.
	halRfWriteReg(CC1100_PKTCTRL1, rfConfig->pktctrl1);   // Packet automation control.
	halRfWriteReg(CC1100_PKTCTRL0, rfConfig->pktctrl0);   // Packet automation control.
	halRfWriteReg(CC1100_ADDR, rfConfig->addr);       // Device address.
	halRfWriteReg(CC1100_PKTLEN, rfConfig->pktlen);     // Packet length.

	halSpiWrite(CC1100_PATABLE | CC1100_WRITE_BURST, rfPaTable, rfPaTableLen);
}

//----------------------------------------------------------------------------------
//  void halRfReadConfig(const HAL_RF_CONFIG* rfConfig, const uint8* rfPaTable, uint8 rfPaTableLen)
//
//  DESCRIPTION:
//    Used to check all the bytes written to the device
//  ARGUMENTS:
//    rfConfig     - register settings (as exported from SmartRF Studio)
//    rfPaTable    - array of PA table values (from SmartRF Studio)
//    rfPaTableLen - length of PA table
//
//----------------------------------------------------------------------------------
void halRfReadConfig(HAL_RF_CONFIG* rfConfig)
{
	rfConfig->fsctrl1 = halRfReadReg(CC1100_FSCTRL1);	// Frequency synthesizer control.
	rfConfig->fsctrl1 = halRfReadReg(CC1100_FSCTRL1);   // Frequency synthesizer control.
	rfConfig->fsctrl0 = halRfReadReg(CC1100_FSCTRL0);  // Frequency synthesizer control.
	rfConfig->freq2 = halRfReadReg(CC1100_FREQ2);  // Frequency control word, high byte.
	rfConfig->freq1 = halRfReadReg(CC1100_FREQ1);  // Frequency control word, middle byte.
	rfConfig->freq0 = halRfReadReg(CC1100_FREQ0);  // Frequency control word, low byte.
	rfConfig->mdmcfg4 = halRfReadReg(CC1100_MDMCFG4);  // Modem configuration.
	rfConfig->mdmcfg3 = halRfReadReg(CC1100_MDMCFG3);  // Modem configuration.
	rfConfig->mdmcfg2 = halRfReadReg(CC1100_MDMCFG2);  // Modem configuration.
	rfConfig->mdmcfg1 = halRfReadReg(CC1100_MDMCFG1);  // Modem configuration.
	rfConfig->mdmcfg0 = halRfReadReg(CC1100_MDMCFG0);  // Modem configuration.
	rfConfig->channr = halRfReadReg(CC1100_CHANNR);  // Channel number.
	rfConfig->deviatn = halRfReadReg(CC1100_DEVIATN);  // Modem deviation setting (when FSK modulation is enabled).
	rfConfig->frend1 = halRfReadReg(CC1100_FREND1);  // Front end RX configuration.
	rfConfig->frend0 = halRfReadReg(CC1100_FREND0);  // Front end RX configuration.
	rfConfig->mcsm0 = halRfReadReg(CC1100_MCSM0);  // Main Radio Control State Machine configuration.
	rfConfig->foccfg = halRfReadReg(CC1100_FOCCFG);  // Frequency Offset Compensation Configuration.
	rfConfig->bscfg = halRfReadReg(CC1100_BSCFG);  // Bit synchronization Configuration.
	rfConfig->agcctrl2 = halRfReadReg(CC1100_AGCCTRL2);  // AGC control.
	rfConfig->agcctrl1 = halRfReadReg(CC1100_AGCCTRL1);  // AGC control.
	rfConfig->agcctrl0 = halRfReadReg(CC1100_AGCCTRL0);  // AGC control.
	rfConfig->fscal3 = halRfReadReg(CC1100_FSCAL3);  // Frequency synthesizer calibration.
	rfConfig->fscal2 = halRfReadReg(CC1100_FSCAL2);  // Frequency synthesizer calibration.
	rfConfig->fscal1 = halRfReadReg(CC1100_FSCAL1);  // Frequency synthesizer calibration.
	rfConfig->fscal0 = halRfReadReg(CC1100_FSCAL0);  // Frequency synthesizer calibration.
	rfConfig->fstest = halRfReadReg(CC1100_FSTEST);  // Frequency synthesizer calibration.
	rfConfig->test2 = halRfReadReg(CC1100_TEST2);  // Various test settings.
	rfConfig->test1 = halRfReadReg(CC1100_TEST1);  // Various test settings.
	rfConfig->test0 = halRfReadReg(CC1100_TEST0);  // Various test settings.
	rfConfig->iocfg2 = halRfReadReg(CC1100_IOCFG2);  // GDO2 output pin configuration.
	rfConfig->iocfg0 = halRfReadReg(CC1100_IOCFG0);  // GDO0 output pin configuration.
	rfConfig->pktctrl1 = halRfReadReg(CC1100_PKTCTRL1);  // Packet automation control.
	rfConfig->pktctrl0 = halRfReadReg(CC1100_PKTCTRL0);  // Packet automation control.
	rfConfig->addr = halRfReadReg(CC1100_ADDR);  // Device address.
	rfConfig->pktlen = halRfReadReg(CC1100_PKTLEN);  // Packet length.
}

//----------------------------------------------------------------------------------
//  void  halRfBurstConfig(const HAL_RF_BURST_CONFIG rfConfig, const uint8* rfPaTable, uint8 rfPaTableLen)
//
//  DESCRIPTION:
//    Used to configure all of the CC1100/CC2500 registers in one burst write.
//
//  ARGUMENTS:
//    rfConfig     - register settings
//    rfPaTable    - array of PA table values (from SmartRF Studio)
//    rfPaTableLen - length of PA table
//
//----------------------------------------------------------------------------------
void halRfBurstConfig(const HAL_RF_BURST_CONFIG rfConfig, const uint8_t* rfPaTable, uint8_t rfPaTableLen)
{
	halSpiWrite(CC1100_IOCFG2 | CC1100_WRITE_BURST, (uint8_t*)rfConfig, sizeof(*rfConfig));
	halSpiWrite(CC1100_PATABLE | CC1100_WRITE_BURST, (uint8_t*)rfPaTable, rfPaTableLen);
}

//----------------------------------------------------------------------------------
//  uint8 halRfGetChipId(void)
//----------------------------------------------------------------------------------
uint8_t halRfGetChipId(void)
{
	return (halRfReadStatusReg(CC1100_PARTNUM));
}

//----------------------------------------------------------------------------------
//  uint8 halRfGetChipVer(void)
//----------------------------------------------------------------------------------
uint8_t halRfGetChipVer(void)
{
	return (halRfReadStatusReg(CC1100_VERSION));
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfStrobe(uint8 cmd)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfStrobe(uint8_t cmd)
{
	return (halSpiStrobe(cmd));
}

//----------------------------------------------------------------------------------
//  uint8 halRfReadStatusReg(uint8 addr)
//
//  NOTE:
//      When reading a status register over the SPI interface while the register
//      is updated by the radio hardware, there is a small, but finite, probability
//      that the result is corrupt. The CC1100 and CC2500 errata notes explain the
//      problem and propose several workarounds.
//
//----------------------------------------------------------------------------------
uint8_t halRfReadStatusReg(uint8_t addr)
{
	uint8_t reg;
	halSpiRead(addr | CC1100_READ_BURST, &reg, 1);
	return (reg);
}

//----------------------------------------------------------------------------------
//  uint8 halRfReadReg(uint8 addr)
//----------------------------------------------------------------------------------
uint8_t halRfReadReg(uint8_t addr)
{
	uint8_t reg;
	halSpiRead(addr | CC1100_READ_SINGLE, &reg, 1);
	return (reg);
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfWriteReg(uint8 addr, uint8 data)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfWriteReg(uint8_t addr, uint8_t data)
{
	uint8_t rc;
	rc = halSpiWrite(addr, &data, 1);
	return (rc);
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfWriteFifo(uint8* data, uint8 length)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfWriteFifo(const uint8_t* data, uint8_t length)
{
	return (halSpiWrite(CC1100_TXFIFO | CC1100_WRITE_BURST, (uint8_t*)data, length));
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfReadFifo(uint8* data, uint8 length)
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfReadFifo(uint8_t* data, uint8_t length)
{
	return (halSpiRead(CC1100_RXFIFO | CC1100_READ_BURST, data, length));
}

//----------------------------------------------------------------------------------
//  uint8 halRfGetTxStatus(void)
//
//  DESCRIPTION:
//      This function transmits a No Operation Strobe (SNOP) to get the status of
//      the radio and the number of free bytes in the TX FIFO
//
//      Status byte:
//
//      ---------------------------------------------------------------------------
//      |          |            |                                                 |
//      | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
//      |          |            |                                                 |
//      ---------------------------------------------------------------------------
//
//  NOTE:
//      When reading a status register over the SPI interface while the register
//      is updated by the radio hardware, there is a small, but finite, probability
//      that the result is corrupt. This also applies to the chip status byte. The
//      CC1100 and CC2500 errata notes explain the problem and propose several
//      workarounds.
//
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfGetTxStatus(void)
{
	return (halSpiStrobe(CC1100_SNOP));
}

//----------------------------------------------------------------------------------
//  uint8 halRfGetRxStatus(void)
//
//  DESCRIPTION:
//      This function transmits a No Operation Strobe (SNOP) with the read bit set
//      to get the status of the radio and the number of available bytes in the RX
//      FIFO.
//
//      Status byte:
//
//      --------------------------------------------------------------------------------
//      |          |            |                                                      |
//      | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
//      |          |            |                                                      |
//      --------------------------------------------------------------------------------
//
//  NOTE:
//      When reading a status register over the SPI interface while the register
//      is updated by the radio hardware, there is a small, but finite, probability
//      that the result is corrupt. This also applies to the chip status byte. The
//      CC1100 and CC2500 errata notes explain the problem and propose several
//      workarounds.
//
//----------------------------------------------------------------------------------
HAL_RF_STATUS halRfGetRxStatus(void)
{
	return (halSpiStrobe(CC1100_SNOP | CC1100_READ_SINGLE));
}

