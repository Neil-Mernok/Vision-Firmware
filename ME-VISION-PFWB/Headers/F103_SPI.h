/*
 * F207_SPI.h
 * Created on: Mar 11, 2012
 * Company: Mernok Elektronik 
 * Author: S.D. Janse van Rensburg
 */

#ifndef F103_SPI_H_
#define F103_SPI_H_
//includes
#include "Global_Variables.h"

//defines
//pins used on SPI

#define SPI1_CLK             		   	RCC_APB2Periph_SPI1
#define SPI1_GPIO             		  	GPIOA
#define SPI1_GPIO_CLK         		  	RCC_APB2Periph_GPIOA
#define SPI1_PIN_SCK          		  	GPIO_Pin_5
#define SPI1_PIN_MISO          		 	GPIO_Pin_6
#define SPI1_PIN_MOSI          		 	GPIO_Pin_7

#define SPI2_CLK             		   	RCC_APB1Periph_SPI2
#define SPI2_GPIO             		  	GPIOB
#define SPI2_GPIO_CLK         		  	RCC_APB2Periph_GPIOB
#define SPI2_PIN_SCK          		  	GPIO_Pin_13
#define SPI2_PIN_MISO          		 	GPIO_Pin_14
#define SPI2_PIN_MOSI          		 	GPIO_Pin_15

#define SPI3_CLK             		   	RCC_APB1Periph_SPI3
#define SPI3_GPIO             		  	GPIOB
#define SPI3_GPIO_CLK         		  	RCC_APB2Periph_GPIOB
#define SPI3_PIN_SCK          		  	GPIO_Pin_3
#define SPI3_PIN_MISO          		 	GPIO_Pin_4
#define SPI3_PIN_MOSI          		 	GPIO_Pin_5

//Functions made public
void SPI1_Config(void);
void SPI2_Config(void);
void SPI3_Config(void);

uint8_t send_spi(SPI_TypeDef* SPIx, uint8_t data);
void SPI_change_mode(SPI_TypeDef* SPIx, int rising);

#endif /* F103_SPI_H_ */
