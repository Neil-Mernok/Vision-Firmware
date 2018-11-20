/***********************************************************************************
    Filename: hal_uart.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_UART_H
#define HAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Serial Port Baudrate Settings */
#define HAL_UART_BAUDRATE_4800        0x01
#define HAL_UART_BAUDRATE_9600        0x02
#define HAL_UART_BAUDRATE_19200       0x03
#define HAL_UART_BAUDRATE_38400       0x04
#define HAL_UART_BAUDRATE_57600       0x05
#define HAL_UART_BAUDRATE_115200      0x06

/* Stop Bits */
#define HAL_UART_ONE_STOP_BIT         0x01
#define HAL_UART_TWO_STOP_BITS        0x02

/* Parity settings */
#define HAL_UART_NO_PARITY            0x04
#define HAL_UART_EVEN_PARITY          0x08
#define HAL_UART_ODD_PARITY           0x10

/* Number of bits in data field */
#define HAL_UART_7_BIT_DATA           0x20
#define HAL_UART_8_BIT_DATA           0x40

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void halUartInit(uint8 baudrate, uint8 options);
void halUartWrite(const uint8* buf, uint16 length);
void halUartRead(uint8* buf, uint16 length);


#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif
