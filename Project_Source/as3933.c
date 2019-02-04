/*
 *****************************************************************************
 * Copyright @ 2011 by austriamicrosystems AG                                *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS3940 ActiveTag firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file as3933.c
 *
 *  \author Wolfgang Reichart
 *
 *  \brief as3933 driver implementation file
 *
 *  This is the implementation file for the as3933 driver.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "as3933.h"
#include "LF_TX.h"
#include "errno.h"

#include "Global_Variables.h"

// this contains info about the LF_tX system. kept here so it does not compile away when LF is disabled
struct _LF_Params LF_Params;

uint32_t LF_Last_Clock;

uint32_t AS3933_clksamples = 128;

#ifdef USE_HAL_DRIVER

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
////////////////////////////////////////////////////////////////////////////////
#define ASSERT_AS3933_CS			GPIO_SetBits(LF_CS_PORT, LF_CS_PIN)
#define DEASSERT_AS3933_CS			GPIO_ResetBits(LF_CS_PORT, LF_CS_PIN)

#define SET_AS3933_SCLK				GPIO_SetBits(LF_SCLK_PORT, LF_SCLK_PIN)
#define CLR_AS3933_SCLK				GPIO_ResetBits(LF_SCLK_PORT, LF_SCLK_PIN)
//#define SET_AS3933_SCLK				LF_SCLK_PORT->BSRR = LF_SCLK_PIN;
//#define CLR_AS3933_SCLK				LF_SCLK_PORT->BSRR = LF_SCLK_PIN<<16;

//static GPIO_InitTypeDef OUT =
//{ .Pin = LF_SCLK_PIN, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_HIGH };
//
//static GPIO_InitTypeDef SPI =
//{ .Pin = LF_SCLK_PIN, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF5_SPI2 };

#define OUT_AS3933_SCLK HAL_GPIO_Init(LF_SCLK_PORT, &OUT)
#define SPI_AS3933_SCLK HAL_GPIO_Init(LF_SCLK_PORT, &SPI)

// clock for sampled data
#define AS3933_CL_DAT_PIN         	GPIO_ReadInputDataBit(LF_CLK_IRQ_PORT, LF_CLK_IRQ_PIN)
// sampled data is displayed on this pin
#define AS3933_DAT_PIN              GPIO_ReadInputDataBit(LF_DAT_PORT, LF_DAT_PIN)
// IRQ pin.
#define AS3933_WAKE_PIN				GPIO_ReadInputDataBit(LF_IRQ_PORT, LF_IRQ_PIN) 
//#define AS3933_SPI(x)				send_spi(AS_SPI, x)

#define	AS3933_ENABLE_INT			__enable_irq()		
#define	AS3933_DISABL_INT			__disable_irq()

#define AS3933_start				AS3933_SPI_start()
#define AS3933_stop					AS3933_SPI_end()

#define AS3933_enable_CLK_ISR		HAL_NVIC_EnableIRQ(LF_CLK_IRQ_Channel)
#define AS3933_disable_CLK_ISR		HAL_NVIC_DisableIRQ(LF_CLK_IRQ_Channel)
////////////////////////////////////////////////////////////////////////////////

#define AS3933_USE_DEFAULTS
#define AS3933_DEBUG_IO

/*
 ******************************************************************************
 * LOCAL FUNCTION
 ******************************************************************************
 */

void AS3933_SPI_start(void)
{
//	SPI_change_mode(AS_SPI, FALSE);

	AS3933_DISABL_INT;
	ASSERT_AS3933_CS;   // set CS
}

void AS3933_SPI_end(void)
{
	DEASSERT_AS3933_CS;   // clear
	AS3933_ENABLE_INT;
}

/*!
 *****************************************************************************
 *  \brief  Write a single byte of data to register
 *
 *  \param  address: The address for writing
 *  \param  value: Data value to be written to AS3933
 *
 *  \return ERR_NONE
 *****************************************************************************
 */
int8_t as3933WriteRegister(uint8_t address, uint8_t value)
{
	uint8_t outbuf[2];

	outbuf[0] = address & 0x3F;   // write mode
	outbuf[1] = value;

	AS3933_start;

	spi_send(&AS_SPI, outbuf[0]);
	spi_send(&AS_SPI, outbuf[1]);

	AS3933_stop;

	return ERR_NONE;
}

/*!
 *****************************************************************************
 *  \brief  Read a single byte of data from register
 *
 *  \param  address: The address for reading
 *  \param  value: Data value is read into this variable from AS3933
 *
 *  \return ERR_NONE
 *****************************************************************************
 */
int8_t as3933ReadRegister(uint8_t address, uint8_t *value)
{
	uint8_t outbuf[2];

	outbuf[0] = 0x40 | (address & 0x3F);   // read mode
	outbuf[1] = 0x00;

	AS3933_start;

	spi_send(&AS_SPI, outbuf[0]);
	*value = spi_send(&AS_SPI, outbuf[1]);

	AS3933_stop;

	return ERR_NONE;
}

/*!
 *****************************************************************************
 *  \brief  Modify bits of data of an address
 *
 *  \param  address: The  address for modifying
 *  \param  mask: Bit mask for modification
 *  \param  value: Data to set on address
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
int8_t as3933ModifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
	uint8_t registerValue;

	as3933ReadRegister(address, &registerValue);
	registerValue &= ~mask;   // clear all bits in masked area
	registerValue |= (value & mask);   // set all desired bits in masked area
	as3933WriteRegister(address, registerValue);

	return ERR_NONE;
}

/*!
 *****************************************************************************
 *  \brief  Send Command to AS3933
 *
 *  \param  command: The command byte
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
int8_t as3933SendCommand(as3933Commands_t command)
{
	uint8_t outbuf;

	outbuf = 0xC0 | (command & 0x3F);   // command mode

	AS3933_start;

	spi_send(&AS_SPI, outbuf);

	AS3933_stop;

	return ERR_NONE;
}


#else    // not using hal driver 

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
////////////////////////////////////////////////////////////////////////////////
#define ASSERT_AS3933_CS			GPIO_SetBits(LF_CS_PORT, LF_CS_PIN)
#define DEASSERT_AS3933_CS			GPIO_ResetBits(LF_CS_PORT, LF_CS_PIN)

//#define SET_AS3933_SCLK				SPI3_GPIO->BSRR = SPI3_PIN_SCK
//#define CLR_AS3933_SCLK				SPI3_GPIO->BRR = SPI3_PIN_SCK
#define SET_AS3933_SCLK				SPI1_GPIO->BSRR = SPI1_PIN_SCK
#define CLR_AS3933_SCLK				SPI1_GPIO->BRR = SPI1_PIN_SCK

#define OUT_AS3933_SCLK 	GPIO_Init(SPI1_GPIO, &OUT)
#define SPI_AS3933_SCLK 	GPIO_Init(SPI1_GPIO, &SPI)

#define AS3933_CL_DAT_PIN         	GPIO_ReadInputDataBit(LF_CLK_IRQ_PORT, LF_CLK_IRQ_PIN) 	// clock for sampled data
#define AS3933_DAT_PIN              GPIO_ReadInputDataBit(LF_DAT_PORT, LF_DAT_PIN) 		// sampled data is displayed on this pin
#define AS3933_WAKE_PIN				GPIO_ReadInputDataBit(LF_IRQ_PORT, LF_IRQ_PIN) 		// IRQ pin. 

#define AS3933_SPI(x)				send_spi(AS_SPI, x)

//#define	AS3933_ENABLE_INT			vPortExitCritical()		
//#define	AS3933_DISABL_INT			vPortEnterCritical()

#define	AS3933_ENABLE_INT			__enable_irq()		
#define	AS3933_DISABL_INT			__disable_irq()

#define AS3933_start				AS3933_SPI_start()
#define AS3933_stop					AS3933_SPI_end()

#define AS3933_enable_CLK_ISR		NVIC_EnableIRQ(LF_CLK_IRQ_Channel)
#define AS3933_disable_CLK_ISR		NVIC_DisableIRQ(LF_CLK_IRQ_Channel)
////////////////////////////////////////////////////////////////////////////////
#define AS3933_USE_DEFAULTS
#define AS3933_DEBUG_IO


/*
 ******************************************************************************
 * LOCAL FUNCTION
 ******************************************************************************
 */

void AS3933_SPI_start(void)
{
	SPI_change_mode(AS_SPI, FALSE);

	AS3933_DISABL_INT;
	ASSERT_AS3933_CS;   // set CS
}

void AS3933_SPI_end(void)
{
	DEASSERT_AS3933_CS;   // clear
	AS3933_ENABLE_INT;
}

/*!
 *****************************************************************************
 *  \brief  Write a single byte of data to register
 *
 *  \param  address: The address for writing
 *  \param  value: Data value to be written to AS3933
 *
 *  \return ERR_NONE
 *****************************************************************************
 */
int8_t as3933WriteRegister(uint8_t address, uint8_t value)
{
	uint8_t outbuf[2];

	outbuf[0] = address & 0x3F;// write mode
	outbuf[1] = value;

	AS3933_start;

	AS3933_SPI(outbuf[0]);
	AS3933_SPI(outbuf[1]);

	AS3933_stop;

	return ERR_NONE;
}

/*!
 *****************************************************************************
 *  \brief  Read a single byte of data from register
 *
 *  \param  address: The address for reading
 *  \param  value: Data value is read into this variable from AS3933
 *
 *  \return ERR_NONE
 *****************************************************************************
 */
int8_t as3933ReadRegister(uint8_t address, uint8_t *value)
{
	uint8_t outbuf[2];

	outbuf[0] = 0x40 | (address & 0x3F);// read mode
	outbuf[1] = 0x00;

	AS3933_start;

	AS3933_SPI(outbuf[0]);
	*value = AS3933_SPI(outbuf[1]);

	AS3933_stop;

	return ERR_NONE;
}

/*!
 *****************************************************************************
 *  \brief  Modify bits of data of an address
 *
 *  \param  address: The  address for modifying
 *  \param  mask: Bit mask for modification
 *  \param  value: Data to set on address
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
int8_t as3933ModifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
	uint8_t registerValue;

	as3933ReadRegister(address, &registerValue);
	registerValue &= ~mask;   // clear all bits in masked area
	registerValue |= (value & mask);// set all desired bits in masked area
	as3933WriteRegister(address, registerValue);

	return ERR_NONE;
}

/*!
 *****************************************************************************
 *  \brief  Send Command to AS3933
 *
 *  \param  command: The command byte
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
int8_t as3933SendCommand(as3933Commands_t command)
{
	uint8_t outbuf;

	outbuf = 0xC0 | (command & 0x3F);// command mode

	AS3933_start;

	AS3933_SPI(outbuf);

	AS3933_stop;

	return ERR_NONE;
}


#endif

/*
 ******************************************************************************
 * MACROS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * CONSTANTS
 ******************************************************************************
 */

uint8_t as3933RegisterDefaults[][2] =
{
// AS3933 default settings for approx. 9m LF range
		{ 0x00, 0x6E },
		{ 0x01, 0x6A },	// AGC_T-LIM = 0 Had improved stability (RSSI value constant), but was causing lost LF. Changed back
		{ 0x02, 0x20 },
		{ 0x03, 0x3F },
		{ 0x04, 0x30 },	// reduce Off time.
		{ 0x05, 0x69 },
		{ 0x06, 0x96 },
		{ 0x07, 0x3F },
		{ 0x08, 0x00 },
		{ 0x09, 0x00 },
		{ 0x10, 0x00 },
		{ 0x11, 0x00 },
		{ 0x12, 0x00 },
		{ 0x13, 0x00 } };

//		|	7			6			5			4			3			2			1			0	      
//R00	|	PATT32	|	DAT_MASK	ON_OFF	|	MUX_123	|	EN_A2	|	EN_A3	|	EN_A1	|			| 
//R01	|	ABS_HY	|	AGC_TLIM	AGC_UD	|	ATT_ON	|	EN_MANCH|	EN_PAT2	|	EN_WPAT	|	EN_XTAL	| 
//R02	|	S_ABS	|	EN_EXT_CLK	G_BOOST	|	Reserved|------DISPLAY_CLK------|---------S_WU1---------| 
//R03	|	HY_20m	|	HY_POS	|--min preamble length(0.8-3.5ms)---|	----manchester symbol rate------| 
//R04	|	----- T_OFF -------	|	----- R_VAL --------|	------------------ GR ----------------------| 
//R05	|	---------------------------------------- PATT2B --------------------------------------------| 
//R06	|	---------------------------------------- PATT1B --------------------------------------------| 
//R07	|	------------T_OUT---------------|----------------------- T_HBIT ----------------------------| 
//R08	|	------------BAND_SEL -----------|						|	----------- T_AUTO -------------| 
//R09	|  BLOCK_AGC|																					| 
//R10	|									|----------------------- 	RSSI1	------------------------| 
//R11	|									|----------------------- 	RSSI2	------------------------| 
//R12	|									|----------------------- 	RSSI3	------------------------| 
//R13	|	---------------------------------------- F_WAKE	--------------------------------------------| 
//R14	|	RC_CAL_OK	RC_CAL_KO|	--------------------------------	RC_OSC_TAPS	--------------------| 
//R15	|									|	LC_OSC_OK	LC_OSC_KO|									| 
//R16	| CLOCK_GEN_DIS|		|	RC_OSC_MIN	RC_OSC_MAX|			|	---------	LC_OSC_MUX	--------| 
//R17	|									|----------------------- 	CAP__CH1	--------------------| 
//R18	|									|----------------------- 	CAP__CH2	--------------------| 
//R19	|									|----------------------- 	CAP__CH3	--------------------| 

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/* for debugging, these values are used in function as3933TuneCapacitors() */
uint16_t rawValue;
as3933AntennaTuningResults as3933TuneResults[AS3933_NR_OF_ANTENNAS];
volatile uint32_t as3933LfSampleData;
volatile uint16_t as3933LfSampleBitCount;
volatile bool as3933LfSampleActive;

bool wakePinWasHigh = FALSE;
bool pairingIsDone = FALSE;
uint16_t wakeupCount = 0;

uint8_t rssiX, rssiY, rssiZ;   // rssi is only 5 bit, so signed data type is ok

int LF_freq_counter;

#ifdef AS3933_DEBUG_IO
int8_t as3933DebugRegs(void);
#endif

/*
 ******************************************************************************
 * GLOBAL FUNCTION
 ******************************************************************************
 */
/*!
 *****************************************************************************
 *  \brief  Initialises AS3933
 *
 *  Initializes the AS3933 registers. *
 *
 *  \param  n/a
 *
 *  \return ERR_NONE ... success, ERR_IO ... input/output error
 *****************************************************************************
 */
int8_t as3933Initialize(int frequency)
{
	int8_t retVal = ERR_NONE;
	int i;


	// correct the settings for use with a crystal at the LF detector.
#ifdef lf_crystal
	as3933RegisterDefaults[1][1] |= 1;
#endif

	if(frequency >= 95000)
	as3933RegisterDefaults[8][1] = 0;
	else if(frequency >= 65000)
	as3933RegisterDefaults[8][1] |= 0x20;
	else /* freq > 40000. lower frequency bands ignored*/
	as3933RegisterDefaults[8][1] |= 0x40;

	// get the register defaults on AS3933
	as3933SendCommand(preset_default);
	// clear any pending wake IRQ
	as3933SendCommand(clear_wake);
	// 0x07 == 0x0B at start up
	as3933WriteRegister(0x07, 0xEB);
	// write the frequency select before the rest. it seems to reset regs 2 and 3. 
	as3933WriteRegister(0x08, as3933RegisterDefaults[8][1]);

	// write register defaults to relevant registers
	for (i = 0; i < sizeof(as3933RegisterDefaults) / sizeof(as3933RegisterDefaults[0]); i++)
	{
		as3933WriteRegister(as3933RegisterDefaults[i][0], as3933RegisterDefaults[i][1]);
	}
#ifdef AS3933_DEBUG_IO
	if (as3933DebugRegs() < 0)
	{
		retVal = ERR_IO;
	}
#endif

	Config_EXTI_LFCLK(AS3933_CLK_ISR);
	return retVal;
}

#ifdef AS3933_DEBUG_IO
/*!
 *****************************************************************************
 *  \brief  Checks if register defaults have been overwritten
 *
 *  \param  n/a
 *
 *  \return 0 ... registers unchanged, -1 ... registers were changed
 *****************************************************************************
 */
int8_t as3933DebugRegs(void)
{
	int i;
	uint8_t regVal = 0;
	int8_t retVal = 0;

	for (i = 0; i < sizeof(as3933RegisterDefaults) / sizeof(as3933RegisterDefaults[0]); i++)
	{
		as3933ReadRegister(as3933RegisterDefaults[i][0], &regVal);
		if (regVal != as3933RegisterDefaults[i][1])
		{
			retVal = -1;
		}
	}

	return retVal;
}
#endif


#if 1
/*!
 *****************************************************************************
 *  \brief  Calibrate the internal RCO of AS3933 using the LCO
 *
 *  \param  n/a
 *
 *  \return 0 ... success, -1 ... error
 *****************************************************************************
 */
int8_t as3933CalibrateRCOViaLCO (void)
{
	uint8_t as3933DebugBuffer[20] = {};
	uint8_t regVal = 0;
	uint8_t registerFour, registerOne;
	int8_t retVal = -1;

	/*
	 * before we start the calibration of the RCO with the LC antenna
	 * make sure that the antenna damper is enabled on 27kOhm
	 */
	as3933ReadRegister(0x04, &registerFour);
	as3933DebugBuffer[0] = registerFour;
	as3933ModifyRegister(0x04, 0x30, 0x30);
	as3933ReadRegister(0x04, &as3933DebugBuffer[1]);

	as3933ReadRegister(0x01, &registerOne);
	as3933DebugBuffer[2] = registerOne;
	as3933ModifyRegister(0x01, 0x10, 0x10);
	as3933ReadRegister(0x01, &as3933DebugBuffer[3]);

	/* wait until the damper is effective, how long do we have to wait? */
	Delay(5);
	/* send direct command calib_rco_lc */
	as3933SendCommand(Calib_RCO_LC);
	/* wait until the damper is effective, how long do we have to wait? */
	Delay(15);

	/* revert registers 0x01 and 0x04 to their previous values */
	as3933WriteRegister(0x01, registerOne);
	as3933WriteRegister(0x04, registerFour);

	/* check status of calibration procedure in R14[6] (NOK) and R14[7] (OK) */
	as3933ReadRegister(0x0e, &regVal);
	as3933DebugBuffer[4] = regVal;

	if ((regVal & 0x80) && !(regVal & 0x40))
	{
		retVal = 0;
	}
	as3933DebugBuffer[5] = retVal;
	
	////////////////////////////////////////////////////////
	// test code: get RCO frequency
	as3933WriteRegister(8, 0x04);
	as3933WriteRegister(9, 0x14);
		
	Delay(2);
	
	////////////////////////////////////////////////////////
	
	
	return retVal;
}
#endif

#ifndef lf_crystal
/*!
 *****************************************************************************
 *  \brief  Calibrate the internal RCO of AS3933 using the SPI signal from MCU
 *
 *  \param  n/a
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
int8_t as3933CalibrateRCOViaSPI(void)
{
	uint8_t regVal = 0;
	int8_t retVal = -1;
	
	uint8_t outbuf;
	int counter;
	uint32_t pulses = 0;

	outbuf = 0xC0 | (Calib_RCosc & 0x3F);   // command mode

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* disable interrupts here. we must not get interrupted */
	AS3933_start;

#ifdef USE_HAL_DRIVER
#define counter_val	36								///// NOTE! this needs to be tuned for the specific micro and compiler optimisation
	/// send the calibrate command
	HAL_SPI_Transmit(&AS_SPI, outbuf, 1, 2);
	OUT_AS3933_SCLK;

#else
#define counter_val	90								///// NOTE! this needs to be tuned for the specific micro and compiler optimisation
	/// send the calibrate command
	AS3933_SPI(outbuf);


	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
#endif

	/// send 31.250 kHz (F_LF/4 = 150k/4) reference clock on SCL, for 65 pulses to tune the internal oscillator 
	pulses = 0;

	while (pulses < 72)	// note: make this number very large to tune 
	{
		CLR_AS3933_SCLK;						// clear clk pin
		for (counter = 0; counter < counter_val; counter++)
			;
		SET_AS3933_SCLK;						// set clk pin
		for (counter = 0; counter < counter_val; counter++)
			;
		pulses++;
	}
	CLR_AS3933_SCLK;						// set clk pin
			
	AS3933_stop;

	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// return control of the SCLK pin to SPI bus. 
#ifdef USE_HAL_DRIVER
	
	SPI_AS3933_SCLK;
	
#else
	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_SCK;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
#endif

	Delay(2);
	
	/* check status of calibration procedure in R14[6] (NOK) and R14[7] (OK) */
	as3933ReadRegister(0x0e, &regVal);
//	as3933DebugBuffer[4] = regVal;

	if ((regVal & 0x80) && !(regVal & 0x40))
	{
		retVal = 0;
	}
	
//	////////////////////////////////////////////////////////
//	// test code: get RCO frequency
//	as3933WriteRegister(8, 0x04);
//	as3933WriteRegister(9, 0x14);
//	Delay(2);
//	////////////////////////////////////////////////////////

	return retVal;
}

#endif

uint8_t last_rssi = 0;

/*!
 *****************************************************************************
 *  \brief  Returns the strongest RSSI measured by AS3933
 *
 *  \param  n/a
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
uint8_t as3933GetStrongestRssi(void)
{
	uint8_t rssiMax = 0;
	as3933ReadRegister(10, &rssiX);
	rssiX &= 0x1F;   // 5 bit value in register
	//SET_DBG_BUF(rssiX);

	as3933ReadRegister(11, &rssiY);
	rssiY &= 0x1F;   // 5 bit value in register
	//SET_DBG_BUF(rssiY);

	as3933ReadRegister(12, &rssiZ);
	rssiZ &= 0x1F;   // 5 bit value in register
	//SET_DBG_BUF(rssiZ);

//	as3933SendCommand(clear_wake);
	// calculate strongest RSSI
	rssiMax = MAX(rssiX, rssiY);
	rssiMax = MAX(rssiZ, rssiMax);
	//SET_DBG_BUF(rssiMax);

	last_rssi = rssiMax;
	return rssiMax;
}

/*
 
 */
/*!
 *****************************************************************************
 *  \brief  Tunes LF-Antennas for X,Y or Z-direction using the internal caps
 *  of the AS3933. The desired value is 125 kHz.
 *  The capacitance value is written to register 17 + capacitor.
 * 
 *  Connect LFxP to the LCO (R16<2:0>), after ~50 pulses
 *  (oscillator stabilizes) correct frequency at CL_DAT pin
 *  R17[4:0] - R19[4:0] adds capacitors to decrease frequency until 125 kHz
 *  return RSSI on LEDs
 *
 *  \param[IN]  capacitor ... cap 0 (=X), 1 (=Y) or 2 (=Z)
 *  \param[out] capacity ... calculated capacitance value
 *
 *  \return ERR_NONE ... success
 *****************************************************************************
 */
int8_t as3933TuneCapacitors(uint8_t capacitor, uint8_t *capacity, uint32_t frequency)
{
	int8_t retVal, origFreqDone;
	uint8_t capacitorValue, shadowRegisterOne, shadowRegisterFour;
	uint32_t cIC2TmrOverflowValue, currSysFreq;
	uint32_t cRisingEdgeCount;
	int32_t noTimeout;
	origFreqDone = 0;
	/* we need to enable antenna damper at 27 kOhm to avoid cross coupling */
	as3933ReadRegister(4, &shadowRegisterFour);
	as3933WriteRegister(4, shadowRegisterFour | 0x30);
	as3933ReadRegister(1, &shadowRegisterOne);
	as3933WriteRegister(1, shadowRegisterOne | 0x10);

#ifdef USE_HAL_DRIVER
	currSysFreq = HAL_RCC_GetPCLK1Freq();
#else
	currSysFreq = RCC_Clocks.SYSCLK_Frequency;
#endif
	cIC2TmrOverflowValue = currSysFreq >> 7;

	/// make sure we're not using an insane value for the clock samples. if the timer overflows we'll never get good results.
	AS3933_clksamples = 45000 * (uint64_t) frequency / (uint64_t) currSysFreq;

#ifdef lf_freq_timer

	cRisingEdgeCount = AS3933_desired_freq / 10;   // 125000 Hz is the desired frequency

	capacitorValue = 0x00;
	as3933WriteRegister(17 + capacitor, capacitorValue);

	as3933WriteRegister(16, 1 << capacitor);// Connects LFxP to the LCO (display on DAT pin)

	retVal = ERR_BUSY;
	while (retVal == ERR_BUSY)// we start the calibration
	{

		// wait 64 pulses until the oscillator of the AS3933 is stable
		Delay(1);

		LF_freq_counter = 0;
		rawValue = 0;

		// reset the timer that captures the input frequency. 
		htim3.Instance->CNT = 0;

		// wait a while 
		Delay(100);

		rawValue = htim3.Instance->CNT;

		if (rawValue > 0)// timer worked
		{
			as3933TuneResults[capacitor].rawValue = rawValue;

			if ((origFreqDone == 0) && (rawValue > 0))
			{
				origFreqDone = 1;
				as3933TuneResults[capacitor].resonanceFrequencyOrig = (uint32_t) 10 * rawValue;
			}

			if (rawValue < cRisingEdgeCount)
			{
				capacitorValue++;
				if (capacitorValue > 0x1F)
				{
					retVal = ERR_START_FREQ_TOO_HIGH;   // we were not able to tune, antenna trimming out of range
					capacitorValue = 0x1F;
				}
				as3933WriteRegister(17 + capacitor, capacitorValue);
			}
			else if (capacitorValue == 0)
			retVal = ERR_START_FREQ_TOO_LOW;   // we were not able to tune, antenna trimming out of range
			else
			retVal = ERR_NONE;
		}
		else   // we detected an overflow
		{
			retVal = ERR_NO_FREQ_DETECTED;   // there is no frequency at CL_DAT pin -> hardware problem? (antenna not connected?)
		}
	}

	*capacity = capacitorValue;

	// make sure to remove LFxP from the LCO, otherwise wake up won't work anymore
	as3933WriteRegister(16, 0x00);

	/* set antenna damper back to user defined value */
	as3933WriteRegister(1, shadowRegisterOne);
	as3933WriteRegister(4, shadowRegisterFour);

	/* store results in result struct */
	if (rawValue > 0)
	{
		as3933TuneResults[capacitor].resonanceFrequencyTuned = (uint32_t) 10 * rawValue;
	}

#else

	cRisingEdgeCount = ((uint64_t) currSysFreq * (uint64_t) AS3933_clksamples) / (uint64_t) frequency;   // 125000 Hz is the desired frequency, usually

	capacitorValue = 0x00;
	as3933WriteRegister(17 + capacitor, capacitorValue);

	as3933WriteRegister(16, 1 << capacitor);   // Connects LFxP to the LCO (display on DAT pin)

	retVal = ERR_BUSY;
	while (retVal == ERR_BUSY)   // we start the calibration
	{

		// wait 64 pulses until the oscillator of the AS3933 is stable
		Delay(1);

		LF_freq_counter = 0;
		rawValue = 0;

		Start_EXTI_LFDAT(AS3933_FREQ_ISR);

		noTimeout = cIC2TmrOverflowValue;
		while ((rawValue == 0) && (noTimeout-- > 0))
			;   // wait for first few pulses so we are sure the oscillator is up and running

		LF_freq_counter = 0;
		rawValue = 0;

		Start_EXTI_LFDAT(AS3933_FREQ_ISR);

		while ((rawValue == 0) && (noTimeout-- > 0))
			;   // wait for the 16 pulses or timeout

		Stop_EXTI_LFDAT();   // disbale interrupt

		if (noTimeout > 0)   // no overflow, all ok
		{
			as3933TuneResults[capacitor].rawValue = rawValue;

			if ((origFreqDone == 0) && (rawValue > 0))
			{
				origFreqDone = 1;
				as3933TuneResults[capacitor].resonanceFrequencyOrig = currSysFreq / rawValue * AS3933_clksamples;
			}

			if (rawValue < cRisingEdgeCount)
			{
				capacitorValue++;
				if (capacitorValue > 0x1F)
				{
					retVal = ERR_START_FREQ_TOO_HIGH;   // we were not able to tune, antenna trimming out of range
					capacitorValue = 0x1F;
				}
				as3933WriteRegister(17 + capacitor, capacitorValue);
			}
			else if (capacitorValue == 0)
				retVal = ERR_START_FREQ_TOO_LOW;   // we were not able to tune, antenna trimming out of range
			else
				retVal = ERR_NONE;
		}
		else   // we detected an overflow
		{
			retVal = ERR_NO_FREQ_DETECTED;   // there is no frequency at CL_DAT pin -> hardware problem? (antenna not connected?)
		}
	}

	*capacity = capacitorValue;

	// make sure to remove LFxP from the LCO, otherwise wake up won't work anymore
	as3933WriteRegister(16, 0x00);

	/* set antenna damper back to user defined value */
	as3933WriteRegister(1, shadowRegisterOne);
	as3933WriteRegister(4, shadowRegisterFour);

	/* store results in result struct */
	if (rawValue > 0)
	{
		as3933TuneResults[capacitor].resonanceFrequencyTuned = currSysFreq / rawValue * AS3933_clksamples;
	}

#endif

	//as3933TuneResults[capacitor].capacitance = capacitorValue;
	as3933ReadRegister(17 + capacitor, &capacitorValue);
	as3933TuneResults[capacitor].capacitance = capacitorValue;
	as3933TuneResults[capacitor].returnValue = retVal;

	return retVal;
}

/*!
 *****************************************************************************
 *  \brief  Performs the antenna tuning loop over X, Y and Z LF-antennas.
 *
 *  \param n/a
 *  \param[out] capacity ... calculated capacitance value
 *
 *  \return number of working channels
 *****************************************************************************
 */
int8_t as3933AntennaTuning(uint32_t frequency)
{
	uint8_t capacitor, capacity, i, working = 0;

	for (capacitor = 0; capacitor < 3; capacitor++)
	{
		as3933TuneCapacitors(capacitor, &capacity, frequency);
	}

	/* check tuning results and display error if any antenna is out of range */
	for (i = 0; i < AS3933_NR_OF_ANTENNAS; i++)
	{
		if ((as3933TuneResults[i].resonanceFrequencyTuned < (frequency + 5000)) && (as3933TuneResults[i].resonanceFrequencyTuned > (frequency - 5000)))
			working++;
	}
	return working;
}

/*!
 *****************************************************************************
 *  \brief  test code to get the oscillation frequency of one of the channels.
 *  used to generate a LF coil tester device.
 *  returns frequency
 *****************************************************************************
 */
uint32_t as3933GetFrequency(uint8_t capacitor)
{
	uint32_t freq_out = 0;
	uint32_t cIC2TmrOverflowValue, currSysFreq;
	int32_t noTimeout;

	/* we need to enable antenna damper at 27 kOhm to avoid cross coupling */
//	as3933ReadRegister(4, &shadowRegisterFour);
//	as3933WriteRegister(4, shadowRegisterFour | 0x30);
//	as3933ReadRegister(1, &shadowRegisterOne);
//	as3933WriteRegister(1, shadowRegisterOne | 0x10);
#ifdef USE_HAL_DRIVER
	currSysFreq = HAL_RCC_GetPCLK1Freq();
#else
	currSysFreq = RCC_Clocks.SYSCLK_Frequency;
#endif
	cIC2TmrOverflowValue = currSysFreq >> 4;

	// set capacitance to none
	as3933WriteRegister(17 + capacitor, 0);

	as3933WriteRegister(16, 1 << capacitor);   // Connects LF channel x to the LCO (display on DAT pin)

	// wait 64 pulses until the oscillator of the AS3933 is stable
	Delay(1);

	LF_freq_counter = 0;
	rawValue = 0;

	Start_EXTI_LFDAT(AS3933_FREQ_ISR);

	noTimeout = cIC2TmrOverflowValue;
	while ((rawValue == 0) && (noTimeout-- > 0))
		;   // wait for first few pulses so we are sure the oscillator is up and running

	LF_freq_counter = 0;
	rawValue = 0;

	Start_EXTI_LFDAT(AS3933_FREQ_ISR);

	while ((rawValue == 0) && (noTimeout-- > 0))
		;   // wait for the 16 pulses or timeout

	Stop_EXTI_LFDAT();   // disbale interrupt

	if (noTimeout > 0)   // no overflow, all ok
	{
		freq_out = currSysFreq / rawValue * AS3933_clksamples;
	}
	else
	{
		return 0;   // there is no frequency at CL_DAT pin -> hardware problem? (antenna not connected?)
	}
	return freq_out;
}

/*!
 *****************************************************************************
 *  \brief  Samples LF-data from DAT pin of the AS3933.
 *
 *  The manchester-decoded LF-data is output on DAT (clock on CL_DAT) pin and
 *  sampled for further use. A timer prevents from deadlocks when
 *  less  than expected bits are output on DAT pin.
 *
 *  \param[out] sampleData ... uint32_t value to store sampled data.
 *
 *  \return ERR_NONE ... success, ERR_IO on timeout
 *****************************************************************************
 */
int8_t as3933SampleData(uint32_t * sampleData)
{
	int8_t retVal = ERR_NONE;

	as3933LfSampleBitCount = 0;
	as3933LfSampleData = 0;
	as3933LfSampleActive = 1;

	*sampleData = as3933LfSampleData;

	return retVal;
}

/*
 *****************************************************************************
 *  \brief STM32 input change ISR for data sampling on DAT and CL_DAT pins of
 *  AS3933.
 *
 *  Waits for AS3933_NUM_BITS_TO_SAMPLE to be received.
 *
 *  \param n/a
 *
 *  \return n/a
 *****************************************************************************
 */
void AS3933_CLK_ISR(void)
{
	uint8_t as3933LfCurrentBit;
	uint32_t now;
	int message = 0;

	now = time_now();
	if (LF_Params.state == 0)					// dont accept LF in while sending.
	{
		if (now - LF_Last_Clock > 5)			// this is the start of a new message
		{
			as3933LfSampleBitCount = 0;
			as3933LfSampleData = 0;
			message = 1;

			pipe_put(&LF_RX.p, &message);
			// the AS3933_DAT_PIN and "OR" it to the as3933LfSampleData
			as3933LfSampleBitCount++;
			as3933LfSampleData <<= 1;
			as3933LfCurrentBit = (AS3933_DAT_PIN ? 1 : 0);
			as3933LfSampleData |= as3933LfCurrentBit;

		}
		else
		{
			// data can be sampled on rising edge -> when pin is high take the data from
			// the AS3933_DAT_PIN and "OR" it to the as3933LfSampleData
			as3933LfSampleBitCount++;
			as3933LfSampleData <<= 1;
			as3933LfCurrentBit = (AS3933_DAT_PIN ? 1 : 0);
			as3933LfSampleData |= as3933LfCurrentBit;
		}
	}

	LF_Last_Clock = now;
#ifdef USE_HAL_DRIVER
	htim4.Instance->CNT = 0;
#else
	Timer4_Restart();
#endif
}

/*
 *****************************************************************************
 *  \brief STM32 input change ISR for measuring clock frequency on DAT pin.
 *
 *  \param n/a
 *
 *  \return n/a
 *****************************************************************************
 */
void AS3933_FREQ_ISR(void)
{
	static uint16_t timenow, timelast;

	// first time we've reached the frequency interrupt after it was enabled. so grab the timer value 
	if (LF_freq_counter == 0)
	{
#ifdef USE_HAL_DRIVER
		timelast = htim7.Instance->CNT;
#else
		timelast = TIM_GetCounter(TIM7);
#endif
	}

	if (LF_freq_counter >= AS3933_clksamples)
	{
#ifdef USE_HAL_DRIVER
		timenow = htim7.Instance->CNT;
#else
		timenow = TIM_GetCounter(TIM7);
#endif

		rawValue = timenow - timelast;
		Stop_EXTI_LFDAT();
	}
	LF_freq_counter++; 		// count the time pulses passed
}

/*
 *****************************************************************************
 *  \brief: LF receiver timout interrupt function.
 *  his gets called by some timer interrupt when no LF clks have been detected for a while.
 *  
 *  \param n/a
 *
 *  \return n/a
 *****************************************************************************
 */
//void AS3933_LF_timout_ISR(void)
//{
//	int message;
//
//	LF_bits = as3933LfSampleBitCount;
////	wakePinWasHigh = TRUE;
//	LF_Data = as3933LfSampleData;
//	if (as3933LfSampleBitCount > 10)
//	{
//		message = 2;
//		pipe_put(&LF_RX.p, &message);
//	}
//}
/*
 *****************************************************************************
 *  \brief: this is just a check to see if LF has occurred
 *****************************************************************************
 */
int AS3933_LF_indication(void)
{

	if (LF_Last_Clock == 0)
		return 0;
	else
		return 1;
}

/*
 *****************************************************************************
 *  \brief: this is used as a replacement for the LF data timeout.
 *  Check how long its been since the last LF bit, if > 5ms, timeout and if applicable generate flag
 *  \return 0 for none, 1 for complete packet, 2 for incomplete packet. 
 *****************************************************************************
 */
int AS3933_LF_check_timout(void)
{

	if (LF_Last_Clock == 0)
		return 0;
	else if (time_since(LF_Last_Clock) > 5)
		return 1;
//	else if (as3933LfSampleBitCount >= 25)
//		return 1;
	return 0;
}

/*
 *****************************************************************************
 *  \brief: this is used as a replacement for the LF data timeout.
 *  Check how long its been since the last LF bit, if > 5ms, timeout and if applicable generate flag
 *  \return 0 for none, 1 for complete packet, 2 for incomplete packet. 
 *****************************************************************************
 */
int AS3933_LF_get_data(LF_message_type* LF, bool disable_CRC)
{
	uint8_t crc_val = 0;
	
	if (as3933LfSampleBitCount >= 24)
	{
		LF_Last_Clock = 0;
		
		// send the LF RX message to the RF task. 
		LF->RSSI = last_rssi;
		LF->VehicleID = (as3933LfSampleData >> (as3933LfSampleBitCount - 16)) & 0xFFFF;
		LF->SlaveID = (as3933LfSampleData >> (as3933LfSampleBitCount - 20)) & 0xF;

		if (disable_CRC == false)
		{
			crc_val = (as3933LfSampleData >> (as3933LfSampleBitCount - 24)) & 0xF;
			if (makeCrc4Check((as3933LfSampleData >> (as3933LfSampleBitCount - 20)), crc_val))
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if (LF->SlaveID == 0 && LF->VehicleID == 0)		// disregard blank LF messages. could be noise.
			return 0;
		else
			return 1;
	}
	return 0;
}

#define WAKE_ISR
#ifdef WAKE_ISR
/*
 *****************************************************************************
 *  \brief STM32 input change ISR for AS3933 wake pin.
 *
 *  \param n/a
 *
 *  \return n/a
 *****************************************************************************
 */
void AS3933_Wake_ISR(void)
{
	int message = 1;

	if (AS3933_WAKE_PIN) // if wake pin rising edge isr, start LF data reception
	{
		//as3933LfSampleBitCount = 0;
		//as3933LfSampleData = 0;
		//as3933LfSampleActive = TRUE;
		pipe_put(&LF_RX.p, &message);
		//AS3933_enable_CLK_ISR;
	}
	else
	{
		//as3933LfSampleActive = FALSE;
		wakePinWasHigh = 1;
		pipe_put(&LF_RX.p, &message);
		//AS3933_disable_CLK_ISR;
		//LF_Data = as3933LfSampleData;
	}
}
#endif

