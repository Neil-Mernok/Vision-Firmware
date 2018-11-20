#include "my_rf_settings.h"

/* Chipcon */
/* Product = CC1100 */
/* Chip version = E */
/* Crystal accuracy = 10 ppm */
/* X-tal frequency = 26 MHz */
/* RF output power = 0 dBm */
/* RX filterbandwidth = 541.666667 kHz */
/* Phase = 1 */
/* Datarate = 249.938965 kbps */
/* Modulation = (7) MSK */
/* Manchester enable = (0) Manchester disabled */
/* RF Frequency = 432.999908 MHz */
/* Channel spacing = 199.951172 kHz */
/* Channel number = 0 */
/* Optimization = Sensitivity */
/* Sync mode = (3) 30/32 sync word bits detected */
/* Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX */
/* CRC operation = (1) CRC calculation in TX and CRC check in RX enabled */
/* Forward Error Correction = (0) FEC disabled */
/* Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word. */
/* Packetlength = 255 */
/* Preamble count = (2)  4 bytes */
/* Append status = 1 */
/* Address check = (0) No address check */
/* FIFO autoflush = 0 */
/* Device address = 0 */
/* GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet */
/* GDO2 signal selection = (11) Serial Clock */

//for fast FSK
HAL_RF_CONFIG myRfConfig_250 = {
	0x0C,	// FSCTRL1   Frequency synthesizer control.
	0x00,   // FSCTRL0   Frequency synthesizer control.
	// 433 MHz
//	0x10,   // FREQ2     Frequency control word, high byte.
//	0xA7,   // FREQ1     Frequency control word, middle byte.
//	0x62,   // FREQ0     Frequency control word, low byte.
	// 433.92 MHz
	0x10,   // FREQ2     Frequency control word, high byte.
	0xB0,   // FREQ1     Frequency control word, middle byte.
	0x71,   // FREQ0     Frequency control word, low byte.

//Test Parms for 432 MHz
//	0x10,	// FREQ2     Frequency control word, high byte.
//	0x9D,	// FREQ1     Frequency control word, middle byte.
//	0x8A,	// FREQ0     Frequency control word, low byte.

	0x2D,   // MDMCFG4   Modem configuration.
	0x3B,   // MDMCFG3   Modem configuration.
	0x13,   // MDMCFG2   Modem configuration.
	0x22,   // MDMCFG1   Modem configuration.
	0xF8,   // MDMCFG0   Modem configuration.
	0x00,   // CHANNR    Channel number.
	0x62,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
	0xB6,   // FREND1    Front end RX configuration.
	0x10,   // FREND0    Front end TX configuration.
	0x18,   // MCSM0     Main Radio Control State Machine configuration.
	0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
	0x1C,   // BSCFG     Bit synchronization Configuration.
	0xC7,   // AGCCTRL2  AGC control.
	0x00,   // AGCCTRL1  AGC control.
	0xB0,   // AGCCTRL0  AGC control.
	0xEA,   // FSCAL3    Frequency synthesizer calibration.
	0x2A,   // FSCAL2    Frequency synthesizer calibration.
	0x00,   // FSCAL1    Frequency synthesizer calibration.
	0x1F,   // FSCAL0    Frequency synthesizer calibration.
	0x59,   // FSTEST    Frequency synthesizer calibration.
	0x88,   // TEST2     Various test settings.
	0x31,   // TEST1     Various test settings.
	0x09,   // TEST0     Various test settings.
	0x0B,   // IOCFG2    GDO2 output pin configuration.
	0x06,   // IOCFG0	 GDO0 output pin configuration.
	0x04,   // PKTCTRL1  Packet automation control.
	0x45,   // PKTCTRL0  Packet automation control.
	0x00,   // ADDR      Device address.
	0x61    // PKTLEN    Packet length.
};
//
// const uint8_t myPaTable[] = {0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
// const uint8_t myPaTableLen = 8;
// 
// 





// Sync word qualifier mode = 30/32 sync word bits detected
// CRC autoflush = false
// Channel spacing = 199.951172
// Data format = Normal mode
// Data rate = 49.9878
// RX filter BW = 116.071429
// PA ramping = false
// Preamble count = 4
// Whitening = false
// Address config = No address check
// Carrier frequency = 433.999969
// Device address = 0
// TX power = 10
// Manchester enable = false
// CRC enable = true
// Deviation = 50.781250
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word
// Packet length = 255
// Modulation format = GFSK
// Base frequency = 433.999969
// Modulated = true
// Channel number = 0


const HAL_RF_CONFIG myRfConfig_40 = {
	0x0C,  // FSCTRL1   Frequency synthesizer control.
	0x00,  // FSCTRL0   Frequency synthesizer control.
	0x10,  // FREQ2     Frequency control word, high byte.
	0xB6,  // FREQ1     Frequency control word, middle byte.
	0x3B,  // FREQ0     Frequency control word, low byte.
	0xBA,  // MDMCFG4       Modem Configuration
	0xF8,  // MDMCFG3       Modem Configuration
	0x13,  // MDMCFG2       Modem Configuration
	0x22,   // MDMCFG1   Modem configuration.
	0xF8,   // MDMCFG0   Modem configuration.
	0x00,   // CHANNR    Channel number.
	0x50,  // DEVIATN       Modem Deviation Setting 
	0xB6,  // FREND1    Front end RX configuration.
	0x10,   // FREND0    Front end TX configuration.
	0x18,  // MCSM0         Main Radio Control State Machine Configuration
	0x1D,  // FOCCFG    Frequency Offset Compensation Configuration.
	0x1C,  // BSCFG         Bit Synchronization Configuration
	0xC7,  // AGCCTRL2  AGC control.
	0x00,  // AGCCTRL1  AGC control.
	0xB0,  // AGCCTRL0  AGC control.
	0xE9,  // FSCAL3        Frequency Synthesizer Calibration
	0x2A,  // FSCAL2        Frequency Synthesizer Calibration
	0x00,  // FSCAL1        Frequency Synthesizer Calibration
	0x1F,  // FSCAL0        Frequency Synthesizer Calibration
	0x59,   // FSTEST    Frequency synthesizer calibration.
	0x81,  // TEST2     Various test settings.
	0x35,  // TEST1     Various test settings.
	0x09,  // TEST0     Various test settings.
	0x0B,  // IOCFG2    GDO2 output pin configuration.
	0x06,  // IOCFG0	 GDO0 output pin configuration.
	0x04,   // PKTCTRL1  Packet automation control.
	0x45,  // PKTCTRL0      Packet Automation Control
	0x00,   // ADDR      Device address.
	0x61    // PKTLEN    Packet length.
};


uint8_t myPaTable[] = {0xC0, 0};	
uint8_t myPaTableLen = 2;

const uint8_t PAsettings[]	= {0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0};
