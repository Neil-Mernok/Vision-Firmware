/***********************************************************************************
    Filename: hal_spi.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_SPI_H
#define HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <hal_types.h>
#ifdef USE_HAL_DRIVER
#include "spi.h"
#include "stdint.h"

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

// these should be defined to use the CC1101 CS pin.
// #define select_CC()			PORTB &= ~4;		//set SPI SS pin low to enable
// #define deselect_CC()		PORTB |=  4;		//set SPI SS pin high to disable

//----------------------------------------------------------------------------------
//   Common Macros
//----------------------------------------------------------------------------------
#define HAL_SPI_CS_DEASSERT	SET_CC_CS(1)
#define HAL_SPI_CS_ASSERT	SET_CC_CS(0)
#define HAL_SPI_SOMI_VAL	HAL_GPIO_ReadPin(CC_MISO_PORT, CC_MISO_PIN)
// replaced with more friendly version that doesn't block program flow
//#define HAL_SPI_BEGIN		st( HAL_SPI_CS_ASSERT; long count = 0; while(HAL_SPI_SOMI_VAL) {if(count++ >1000000) break;} )
//#define HAL_SPI_END			st( HAL_SPI_CS_DEASSERT; )
#define HAL_SPI_BEGIN		HAL_SPI_start()
#define HAL_SPI_END			HAL_SPI_end()
#define HAL_SPI_TXBUF_SET(x) st( HAL_SPI_Transmit(CC_SPI, x, 1, 1);)

//void HAL_SPI_BEGIN(void);

uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint16_t len);
uint8_t halSpiWrite(uint8_t addr, uint8_t* data, uint16_t len);
uint8_t halSpiStrobe(uint8_t cmd);


#else
#include "F103_SPI.h"
#include "Global_Variables.h"

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

// these should be defined to use the CC1101 CS pin.
// #define select_CC()			PORTB &= ~4;		//set SPI SS pin low to enable
// #define deselect_CC()		PORTB |=  4;		//set SPI SS pin high to disable

//----------------------------------------------------------------------------------
//   Common Macros
//----------------------------------------------------------------------------------
#define HAL_SPI_CS_DEASSERT	SET_CC_CS(1)
#define HAL_SPI_CS_ASSERT	SET_CC_CS(0)
#define HAL_SPI_SOMI_VAL	GPIO_ReadInputDataBit(CC_MISO_PORT, CC_MISO_PIN)
// replaced with more friendly version that doesn't block program flow
//#define HAL_SPI_BEGIN		st( HAL_SPI_CS_ASSERT; long count = 0; while(HAL_SPI_SOMI_VAL) {if(count++ >1000000) break;} )
//#define HAL_SPI_END			st( HAL_SPI_CS_DEASSERT; )
#define HAL_SPI_BEGIN		HAL_SPI_start()
#define HAL_SPI_END			HAL_SPI_end()
#define HAL_SPI_TXBUF_SET(x) st( send_spi(CC_SPI, x); )

//void HAL_SPI_BEGIN(void);

uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint16_t len);
uint8_t halSpiWrite(uint8_t addr, const uint8_t* data, uint16_t len);
uint8_t halSpiStrobe(uint8_t cmd);

#endif


#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif
