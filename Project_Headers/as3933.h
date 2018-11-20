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

/*! \file as3933.h
 *
 *  \author Wolfgang Reichart
 *
 *  \brief as3933 driver definition file
 *
 *  This is the definition file for the as3933 driver.
 *  
 */

/*!
 * \defgroup xxxx as3933 driver module
 * some words to describe the module
 */

#ifndef AS3933_H
#define AS3933_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <stdint.h>
#include <stdbool.h>
#include "Transponders.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
******************************************************************************
* DEFINES
******************************************************************************
*/

#define USE_LF_CRC_CHECK            1
#if (USE_LF_CRC_CHECK)
    #define USE_LF_CRC_CHECK_STRING     "CRC=YES"
#else
    #define USE_LF_CRC_CHECK_STRING     "CRC=NO"
#endif


#define RAW_VALUE_ARRAY_SIZE        4   // number of samples of input capture unit
#define MINIMUM_FREQUENCY_DEVIATION 0x20
#define MAXIMUM_FREQUENCY_DEVIATION 0x5F
#define AS393X_NR_OF_REGISTERS      20
#define AS3933_NR_OF_ANTENNAS       3



/*
******************************************************************************
* ERRORS
******************************************************************************
*/
/*!\defgroup errorcodes application specific errorcodes start at -50 */
#define ERR_START_FREQ_TOO_HIGH -50 /*!< \ingroup errorcodes
                      the desired frequency can not be reached with the internal capacitors error */
#define ERR_START_FREQ_TOO_LOW -51 /*!< \ingroup errorcodes
                        even without internal capacitors we are already below the desired frequency error */
#define ERR_NO_FREQ_DETECTED -52 /*!< \ingroup errorcodes
                      no frequency was seen on the CL_DAT pin, this can be because of hardware problem
                      (no connection of the antenna) and by this the oscillator does not start */

#define ERR_NOTFOUND -60 /*!< \ingroup errorcodes
                       transponder not found */
#define ERR_NOTUNIQUE -61 /*!< \ingroup errorcodes
                       transponder not unique - more than one transponder in field */
#define ERR_NOTSUPP -62 /*!< \ingroup errorcodes
                          requested operation not supported */

#define ERR_NONE     0                      // no error occured 
#define ERR_NOMEM   -1                      // not enough memory to perform the requested operation 
#define ERR_BUSY    -2                      // device or resource busy 
#define ERR_IO      -3                      // generic IO error 
#define ERR_TIMEOUT -4                      // error due to timeout 
#define ERR_REQUEST -5                      // invalid request or requested function can't be executed at the moment 
#define ERR_NOMSG   -6                      // No message of desired type 
#define ERR_PARAM   -7                      // Parameter error 

#define ERR_LAST_ERROR -32

/*
******************************************************************************
* GLOBAL DATATYPES
******************************************************************************
*/
/*! \ingroup as3933
 * possible direct commands
 */
typedef enum
{
    clear_wake      = 0x0,  /*!< clears the wake state of the chip. In case the chip has woken up (WAKE pin is high) the chip is set back to listening mode */
    reset_RSSI      = 0x1,  /*!< resets the RSSI measurement */
    Calib_RCosc     = 0x2, /*!< starts the trimming procedure of the internal RC oscillator */
    clear_false     = 0x3, /*!< resets the false wake up register (R13=00) */
    preset_default  = 0x4, /*!< sets all register in the default mode */
    Calib_RCO_LC    = 0x5, /*!< calibration of the RC-oscillator with the external LC tank */
}as3933Commands_t;

/*! \ingroup as3933
 * data structure to hold antenna tuning results.
 */
typedef struct
{
    uint32_t resonanceFrequencyOrig;
    uint32_t resonanceFrequencyTuned;
    uint16_t rawValue;
    uint8_t  capacitance;
    int8_t  returnValue;
} as3933AntennaTuningResults;




/*
******************************************************************************
* GLOBAL Variables
******************************************************************************
*/

extern as3933AntennaTuningResults as3933TuneResults[AS3933_NR_OF_ANTENNAS];
extern volatile uint32_t as3933LfSampleData;
extern volatile bool as3933LfSampleActive;
extern bool wakePinWasHigh;

//////////////////////////////////////////////////////////////////////////////
extern uint8_t rssiX, rssiY, rssiZ;   // rssi is only 5 bit, so signed data type is ok
//////////////////////////////////////////////////////////////////////////////

//extern uint32_t LF_Data;
//extern int LF_bits;

/*
******************************************************************************
* GLOBAL FUNCTION PROTOTYPES
******************************************************************************
*/
/*! \ingroup as3933
 *****************************************************************************
 *  \brief  Initializes the as3933 driver module
 *
 *  Initializes the as3933 driver.
 *
 *  \return #ERR_NONE : No error
 *****************************************************************************
 */
int8_t as3933Initialize(int frequency);
int8_t as3933Deinitialize (void);
int8_t as3933Reinitialize (void);
int8_t as3933WriteRegister (uint8_t address, uint8_t value);
int8_t as3933ReadRegister (uint8_t address, uint8_t *value);
int8_t as3933ModifyRegister (uint8_t address, uint8_t mask, uint8_t value);
int8_t as3933SendCommand (as3933Commands_t command);
int8_t as3933CalibrateRCOViaSPI (void);
int8_t as3933CalibrateRCOViaLCO (void);
uint8_t as3933GetStrongestRssi (void);
void AS3933_CLK_ISR(void);
void AS3933_FREQ_ISR(void);
void AS3933_LF_timout_ISR(void);
//void AS3933_Wake_ISR(void);

uint8_t AS3933_RSSI_to_LED(uint8_t RSSI);

extern int8_t as3933DebugRegs (void);

/*!
 * capacitor... which bank should be tuned (0... channel 1, 1... channel 2, 2... channel 3)
 * capacity... returns the value of R17, R18 or R19 depending on value of capacitor
 *             the used value for the tuned antenna
 * returns if tuning was ok, or if we were not able to tune the antenna
 */
extern int8_t as3933TuneCapacitors (uint8_t capacitor, uint8_t *capacity, uint32_t frequency);
extern int8_t measureResonanceFrequency (uint8_t channel, uint16_t *resonanceFrequency);
extern int8_t as3933AntennaTuning(uint32_t frequency);

uint32_t as3933GetFrequency(uint8_t capacitor);

int AS3933_LF_check_timout(void);
int AS3933_LF_indication(void);
int AS3933_LF_get_data(LF_message_type* LF, bool disable_CRC);

extern int8_t as3933SampleData (uint32_t * sampleData);

#ifdef __cplusplus
}
#endif


#endif /* AS3933_H */
