/*
 * F207_Uart.h
 * Created on: Mar 10, 2012
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
#ifndef F103_UART_H_
#define F103_UART_H_
//includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

//defines
#define COMn		4
#define USE_COM_1
//#define USE_COM_2
#define USE_COM_3
//#define USE_COM_4

#define COM_1_BAUD 		115200
//#define COM_1_BAUD 		9600
//#define COM_2_BAUD 		115200
#define COM_3_BAUD		115200
//#define COM_4_BAUD 		115200

typedef enum
{
	COM_1 = 0,			// usart interface
	COM_2 = 1,			// unused
	COM_3 = 2,			// new usart interface rev 4 boards
	COM_4 = 3,			// unused
}COM_TypeDef;

extern COM_TypeDef Master_COM;

/**
 * @brief Hardware Definition for COM Ports
 */
#define USART1_TX_PIN                 	GPIO_Pin_9
#define USART1_TX_GPIO_PORT           	GPIOA
#define USART1_TX_GPIO_CLK            	RCC_APB2Periph_GPIOA
#define USART1_TX_SOURCE              	GPIO_PinSource9
#define USART1_RX_PIN                 	GPIO_Pin_10
#define USART1_RX_GPIO_PORT           	GPIOA
#define USART1_RX_GPIO_CLK            	RCC_APB2Periph_GPIOA
#define USART1_RX_SOURCE              	GPIO_PinSource10

#define USART2_TX_PIN                 	GPIO_Pin_2
#define USART2_TX_GPIO_PORT           	GPIOA
#define USART2_TX_GPIO_CLK            	RCC_APB2Periph_GPIOA
#define USART2_TX_SOURCE              	GPIO_PinSource2
#define USART2_RX_PIN                 	GPIO_Pin_3
#define USART2_RX_GPIO_PORT           	GPIOA
#define USART2_RX_GPIO_CLK            	RCC_APB2Periph_GPIOA
#define USART2_RX_SOURCE              	GPIO_PinSource3

#define USART3_TX_PIN                 	GPIO_Pin_10
#define USART3_TX_GPIO_PORT           	GPIOB
#define USART3_TX_GPIO_CLK            	RCC_APB2Periph_GPIOB
#define USART3_TX_SOURCE              	GPIO_PinSource10
#define USART3_RX_PIN                 	GPIO_Pin_11
#define USART3_RX_GPIO_PORT           	GPIOB
#define USART3_RX_GPIO_CLK            	RCC_APB2Periph_GPIOB
#define USART3_RX_SOURCE              	GPIO_PinSource11

#define UART4_TX_PIN                 	GPIO_Pin_10
#define UART4_TX_GPIO_PORT           	GPIOC
#define UART4_TX_GPIO_CLK            	RCC_APB2Periph_GPIOC
#define UART4_TX_SOURCE              	GPIO_PinSource10
#define UART4_RX_PIN                 	GPIO_Pin_11
#define UART4_RX_GPIO_PORT           	GPIOC
#define UART4_RX_GPIO_CLK            	RCC_APB2Periph_GPIOC
#define UART4_RX_SOURCE              	GPIO_PinSource11


//Functions made public
void COMInit_INT(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);
void USART_Configure(COM_TypeDef COM, uint32_t BaudRate);
int USART_SendDMA(COM_TypeDef COM, uint8_t* buf, uint32_t len);
void USART_SendString(COM_TypeDef COM, char[]);
void USART_Sendarray(COM_TypeDef COM, uint8_t* Data,uint8_t length);
void USART_Sendbyte(COM_TypeDef COM, uint8_t Data);
void USART_SendDataCustom(USART_TypeDef* USARTx, uint16_t Data);
//int write(int fd, uint8_t *str, int len);

#ifdef __cplusplus
}
#endif
#endif /* F103_UART_H_ */
