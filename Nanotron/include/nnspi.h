
#ifndef  _NNSPI_H
#define  _NNSPI_H

#include "config.h"
#include "ntrxtypes.h"
#include "STMport.h"

#define HAL_NN_SPI_BEGIN		Reset_NN_CS
#define HAL_NN_SPI_END			Set_NN_CS

typedef enum
{
    READ_CMD = 0x00,                          /* SPI read command */
    WRITE_CMD = 0x80                          /* SPI write command */
} CMDT;

#define NANONETRESETDELAY   1000             /* reset of TRX in us */

void NTRXPowerOnReset( void );
void InitSPI	(void);
void NTRXSetupSpiInterface 	(void);
void NTRXSPIRead(uint8_t address, uint8_t *buffer, uint8_t len);
void NTRXSPIWrite(uint8_t address, uint8_t *buffer, uint8_t len);
void NTRXSPIReadByte(uint8_t address, uint8_t *buffer);
void NTRXSPIWriteByte(uint8_t address, uint8_t buffer);
void SpiTraceOn (bool_t flag);
#ifdef CONFIG_NTRX_IRQ
void NTRXIrqEnable( bool_t value );
//extern volatile uint8_t nnIrq;
extern volatile uint8_t sreg;
#endif /* CONFIG_NTRX_IRQ */

#ifdef CONFIG_SPI_TRACE
#define TRACE_DATA_LEN 2
#define TRACE_BUFFER_SIZE 200
typedef struct traceS
{
	unsigned char cmd;
	unsigned char len;
	unsigned char addr;
	unsigned char data[TRACE_DATA_LEN];
	int	id;
} traceT;

void printSpiTrace (int min, int max);
void traceSpiReset (void);
#endif /* CONFIG_SPI_TRACE */

#endif   /* _NNSPI_H */
