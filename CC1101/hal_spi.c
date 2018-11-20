/******************************************************************************
 Filename: hal_spi.c

 Copyright 2007 Texas Instruments, Inc.
 ******************************************************************************/

#include "hal_defs.h"
#include "hal_spi.h"

#ifdef USE_HAL_DRIVER

//----------------------------------------------------------------------------------
//   Target specific initialization of SPI interface in hal_spi_config.c
//----------------------------------------------------------------------------------

void HAL_SPI_start(void)
{
	long delay = 0;

//	SPI_change_mode(CC_SPI, TRUE);
	
	HAL_SPI_CS_ASSERT;
	while(HAL_SPI_SOMI_VAL)
	{
		if(delay++ > 10000) break;
	}
}

void HAL_SPI_end(void)
{
	HAL_SPI_CS_DEASSERT;
	
}

//----------------------------------------------------------------------------------
//  void halSpiWrite(uint8 addr, const uint8 *buffer, uint16 length)
//
//  DESCRIPTION:
//    Write data to device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is written. For single byte write, set length to 1.
//----------------------------------------------------------------------------------
uint8_t halSpiWrite(uint8_t addr, uint8_t* data, uint16_t length)
{
	uint16_t i;
	uint8_t rc;

	HAL_SPI_BEGIN;
	rc = spi_send(&CC_SPI, addr);
	for (i = 0; i < length; i++)
		spi_send(&CC_SPI, *data++);
	HAL_SPI_END;
	return (rc);
}

//----------------------------------------------------------------------------------
//  uint8 halSpiRead(uint8 addr, uint8* data, uint16 length)
//
//  DESCRIPTION:
//    Read data from device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is read. Note that the master device needs to write a dummy byte
//    (in this case 0) for every new byte in order to generate the clock to
//    clock out the data. For single byte read, set length to 1.
//----------------------------------------------------------------------------------
uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint16_t length)
{
	uint16_t i;
	uint8_t rc;

	HAL_SPI_BEGIN;
	rc = spi_send(&CC_SPI, addr);
	for (i = 0; i < length; i++)
	{
		data[i] = spi_send(&CC_SPI, 0xFF);
	}
	HAL_SPI_END;
	return (rc);
}

//----------------------------------------------------------------------------------
//  uint8 halSpiStrobe(uint8 cmd)
//
//  DESCRIPTION:
//    Special write function, writing only one byte (cmd) to the device.
//----------------------------------------------------------------------------------
uint8_t halSpiStrobe(uint8_t cmd)
{
	uint8_t rc;

	HAL_SPI_BEGIN;

	rc = spi_send(&CC_SPI, cmd);

	HAL_SPI_END;
	return (rc);
}

#else


//----------------------------------------------------------------------------------
//   Target specific initialization of SPI interface in hal_spi_config.c
//----------------------------------------------------------------------------------

void HAL_SPI_start(void)
{
	long delay = 0;

	SPI_change_mode(CC_SPI, TRUE);
	
	HAL_SPI_CS_ASSERT;
	while(HAL_SPI_SOMI_VAL)
	{
		if(delay++ > 10000) break;
	}
}

void HAL_SPI_end(void)
{
	HAL_SPI_CS_DEASSERT;
}


//----------------------------------------------------------------------------------
//  void halSpiWrite(uint8 addr, const uint8 *buffer, uint16 length)
//
//  DESCRIPTION:
//    Write data to device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is written. For single byte write, set length to 1.
//----------------------------------------------------------------------------------
uint8_t halSpiWrite(uint8_t addr, const uint8_t* data, uint16_t length)
{
	uint16_t i;
	uint8_t rc;

	HAL_SPI_BEGIN;
	rc = send_spi(CC_SPI, addr);
	for (i = 0; i < length; i++)
	{
		send_spi(CC_SPI, *data++);
	}
	HAL_SPI_END;
	return (rc);
}

//----------------------------------------------------------------------------------
//  uint8 halSpiRead(uint8 addr, uint8* data, uint16 length)
//
//  DESCRIPTION:
//    Read data from device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is read. Note that the master device needs to write a dummy byte
//    (in this case 0) for every new byte in order to generate the clock to
//    clock out the data. For single byte read, set length to 1.
//----------------------------------------------------------------------------------
uint8_t halSpiRead(uint8_t addr, uint8_t* data, uint16_t length)
{
	uint16_t i;
	uint8_t rc;

	HAL_SPI_BEGIN;
	rc = send_spi(CC_SPI, addr);
	for (i = 0; i < length; i++)
	{
		data[i] = send_spi(CC_SPI, 0xFF);
	}
	HAL_SPI_END;
	return (rc);
}

//----------------------------------------------------------------------------------
//  uint8 halSpiStrobe(uint8 cmd)
//
//  DESCRIPTION:
//    Special write function, writing only one byte (cmd) to the device.
//----------------------------------------------------------------------------------
uint8_t halSpiStrobe(uint8_t cmd)
{
	uint8_t rc;

	HAL_SPI_BEGIN;

	rc = send_spi(CC_SPI, cmd);

	HAL_SPI_END;
	return (rc);
}



#endif
