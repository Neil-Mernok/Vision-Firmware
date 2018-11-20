/*F103_i2c.h
 *Created on: Oct 29, 2013
 *Company: Mernok Elektronik
 *Author: J.L. goosen
 */

#ifndef F103_i2c_H_
#define F103_i2c_H_
//Includes
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

//Defines

#define I2C_SPEED                       400000
// timeout for not receiving start-bits etc
#define I2C_TIMEOUT_MAX					100000

#define ADDR_16_bit	



//I2C PINS
#define I2C                         I2C1
#define I2C_CLK                     RCC_APB1Periph_I2C1
#define I2C_SCL_PIN                 GPIO_Pin_6                  /* PB.06 */
#define I2C_SDA_PIN                 GPIO_Pin_7                  /* PB.07 */
#define I2C_GPIO_PORT	           	GPIOB                       /* GPIOB */
#define I2C_GPIO_CLK	            RCC_APB2Periph_GPIOB

#define I2C_SDA_LOW()       GPIO_ResetBits(I2C_GPIO_PORT, I2C_SDA_PIN)
#define I2C_SDA_HIGH()      GPIO_SetBits(I2C_GPIO_PORT, I2C_SDA_PIN)

//Variables made public

//Functions made public
void F103_i2c_init(void);
int I2C_BufferRead(uint8_t* data, uint8_t DeviceAddr, uint16_t ReadAddr, uint8_t len);
int I2C_ByteWrite(uint8_t* data, uint8_t DeviceAddr, uint16_t WriteAddr);
int I2C_BufferWrite(uint8_t* data, uint8_t DeviceAddr, uint16_t WriteAddr, uint8_t len);
void F103_i2c_DeInit(void);
int I2C_timeout(void);

ErrorStatus I2C_GetStatus(uint8_t DeviceAddr);

#ifdef __cplusplus
}
#endif

#endif /* LSM303_H_ */
