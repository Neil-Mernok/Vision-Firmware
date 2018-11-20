/**
 * @file nnspi.c
 * @date 2007-Dez-11
 * @author B. Jozefini, S. Radtke
 * @c (C) 2007 Nanotron Technologies
 * @brief SPI bus access functions.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the source code for the SPI bus
 *       control functions of the AVR controller.
 */
#include "config.h"
#include "ntrxtypes.h"
#include "nnspi.h"
#include "hwclock.h"
#include "portation.h"
#include "ME-PCB-138-03-Ports.h"

#ifdef CONFIG_NTRX_IRQ
//volatile uint8_t nnIrq = FALSE;
volatile uint8_t sreg = 0;
#endif

#define CHECK_IRQ if( sreg == 0 )

// #define CONFIG_SPI_TRACE 1
#ifdef CONFIG_SPI_TRACE
#include <stdio.h>

static int traceId = 0;
int traceOn = FALSE;

void SpiTraceOn (bool_t flag)
{
	traceOn = flag;
}

void PrintSpi(CMDT command, uint8_t address,
		uint8_t *buffer, uint8_t len)
{
	char c;
	int n;

	printf ("%4d:", traceId++);
	c = (command == WRITE_CMD) ? 'W' : 'R';
	printf (" %c", c);
	if (address < 16)
	{
		printf (" 0x0");
	}
	else
	{
		printf (" 0x");
	}
	printf ("%x", address);
	printf (" %3d: ", len);

	for (n = 0; n < len; n++)
	{
		if (buffer[n] < 16)
		{
			printf ("0");
		}
		printf ("%x ", buffer[n]);
		if(( (n + 1) % 16 ) == 0 )
		{
			printf ("\n                  ");
		}
	}
	printf ("\n");
}

void traceSpiReset (void)
{
	traceId = 0;
}
#endif /* CONFIG_SPI_TRACE */
/**
 * NTRXPowerOnReset:
 *
 * NanoReset() resets the nanoNET chip and adjusts the pin level.
 *
 * Returns: none
 */

void NTRXPowerOnReset(void)
{
	Reset_PON_RST; /* PON_RST low */

	/*
	 * The reset line has to stay low for at least 360 us
	 * before you can access the transceiver through the
	 * SPI interface.
	 * If you need to optimize the startup time you can
	 * can do other stuff in between and either remove
	 * or shorten this active waiting period.
	 */
	HWDelayus(100);
	Set_PON_RST; /* PON_RST high */
	HWDelayus(400);
}

/**
 * InitSPI:
 *
 * InitSPI() initializes the spi interface on the microcontroller
 *
 * Returns: none
 */
void InitSPI(void)
{
	//SPI3_Config();
	Config_NN_pins();
	HAL_NN_SPI_END;
}

/**
 * NTRXSetupSpiInterface:
 *
 * NTRXSetupSpiInterface() initializes the SPI interface on nanoLOC
 *
 * Returns: none
 */
void NTRXSetupSpiInterface(void)
{
	//NTRXSPIWriteByte(0x00, 0b01011010);		//this sets SPI to LSB first, output MISO output push-pull, irq active low push-pull
	//NTRXSPIWriteByte(0x00, 0b01000010);	//this sets SPI to LSB first, output MISO output push-pull, irq active low OD
	//NTRXSPIWriteByte(0x00, 0b01100110);	//this sets SPI to LSB first, output MISO output push-pull, irq active high.
	
	NTRXSPIWriteByte(0x00, 0b11011011);		//this sets SPI to MSB first, output MISO output push-pull, irq active low push-pull
	//NTRXSPIWriteByte(0x00, 0b11000011);	//this sets SPI to MSB first, output MISO output push-pull, irq active low OD
	//NTRXSPIWriteByte(0x00, 0b11100111);	//this sets SPI to MSB first, output MISO output push-pull, irq active high. 
}

void NTRXSPIRead(uint8_t address, uint8_t *buffer, uint8_t len)
{
	uint16_t i;

#	ifdef CONFIG_SPI_TRACE
	uint8_t *tb;
	uint8_t tl;
	tb = buffer;
	tl = len;
#	endif

	if (len > 0x80 || len == 0)
		return;

#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		ENTER_TASK;
#	endif /* CONFIG_NTRX_IRQ */

	HAL_NN_SPI_BEGIN;
	spi_send(&NN_SPI, len & 0x7F);
	spi_send(&NN_SPI, address);
	for (i = 0; i < len; i++)
	{
		*buffer++ = spi_send(&NN_SPI, 0xFF);
	}
	HAL_NN_SPI_END;

#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		LEAVE_TASK;
#	endif /* CONFIG_NTRX_IRQ */

#	ifdef CONFIG_SPI_TRACE
	if (traceOn == TRUE)
	{
		// traceSpi(READ_CMD, address, tb, tl);
		PrintSpi (READ_CMD, address, tb, tl);
	}
#	endif /* CONFIG_SPI_TRACE */
}

void NTRXSPIWrite(uint8_t address, uint8_t *buffer, uint8_t len)
{
	uint16_t i;
//	uint8_t rc;

#	ifdef CONFIG_SPI_TRACE
	if (traceOn == TRUE)
	{
		PrintSpi(WRITE_CMD, address, buffer, len);
	}
#	endif /* CONFIG_SPI_TRACE */
#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		ENTER_TASK;
#	endif /* CONFIG_NTRX_IRQ */

	HAL_NN_SPI_BEGIN;
	//rc =
			spi_send(&NN_SPI, 0x80 | (len & 0x7F));
	//rc =
			spi_send(&NN_SPI, address);
	for (i = 0; i < len; i++)
	{
		spi_send(&NN_SPI, *buffer++);
	}
	HAL_NN_SPI_END;

#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		LEAVE_TASK;
#	endif /* CONFIG_NTRX_IRQ */

}

void NTRXSPIReadByte(uint8_t address, uint8_t *buffer)
{
#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		ENTER_TASK;
#	endif /* CONFIG_NTRX_IRQ */

	/* chip select */HAL_NN_SPI_BEGIN;

	spi_send(&NN_SPI, 1);			// length
	spi_send(&NN_SPI, address);	//
	*buffer = spi_send(&NN_SPI, 0xff);

	/* chip select */HAL_NN_SPI_END;

#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		LEAVE_TASK;
#	endif /* CONFIG_NTRX_IRQ */

#	ifdef CONFIG_SPI_TRACE
	if (traceOn == TRUE)
	{
		PrintSpi (READ_CMD, address, buffer, 1);
	}
#	endif /* CONFIG_SPI_TRACE */
}

void NTRXSPIWriteByte(uint8_t address, uint8_t buffer)
{
#	ifdef CONFIG_SPI_TRACE
	if (traceOn == TRUE)
	{
		PrintSpi(WRITE_CMD, address, &buffer, 1);
	}
#	endif /* CONFIG_SPI_TRACE */
#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		ENTER_TASK;
#	endif /* CONFIG_NTRX_IRQ */

	/* chip select */HAL_NN_SPI_BEGIN;

	spi_send(&NN_SPI, 0x81);
	spi_send(&NN_SPI, address);
	spi_send(&NN_SPI, buffer);

	/* chip select */HAL_NN_SPI_END;

#	ifdef CONFIG_NTRX_IRQ
	CHECK_IRQ
		LEAVE_TASK;
#	endif /* CONFIG_NTRX_IRQ */

}

/**
 * NTRXIrqEnable:
 * value: -input- boolean enables (TRUE) or disables (FALSE) the external
 *                 interrupt request (interrupts from the nanochip)
 *
 * Returns: none
 */

#ifdef CONFIG_NTRX_IRQ

void NTRXIrqEnable(bool_t value)
{
	Config_EXTI_NN_uIRQ();
}

extern void NTRXInterrupt(void);

/**
 * INTERRUPT EXTI external interrupt:
 *
 * This is an interrupt service routine for the nanochip.
 * It calls the nanoInterrupt service routine.
 */
/**
 * @brief  This function handles External line x interrupt request.
 * @param  None
 * @retval None
 */
void N_uIRQ_Handler(void)
{
	/////////////////////////////
	sreg = 1;				// tell the underlying stack that the process is happening in interrupt.
	NTRXInterrupt();
	sreg = 0;
	///////////////////////////////
}

#endif

