/*F103_i2c.c
 *Created on: Oct 29, 2013
 *Company: Mernok Elektronik
 *Author: J.L. Goosen
 */

#include "F103_i2c.h"

/***********************************************/
//Local Variables
uint32_t IOE_TimeOut = 0;

//Global Variables

//#define i2c_timeout return 0
//#define i2c_timeout while(1)
#define i2c_timeout return I2C_timeout()

/****************************************************************/
/**
 * @brief initialises I2C and pins for use as defined in header
 * 	@param	None
 *  @arg	None
 * @retval 	None
 */
/****************************************************************/
void F103_i2c_init(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	
	/* Reset I2C IP */
	F103_i2c_DeInit();
	
	
	/*clock enable */
	RCC_APB1PeriphClockCmd(I2C_CLK, ENABLE);
	/*I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
	RCC_APB2PeriphClockCmd(I2C_GPIO_CLK, ENABLE);

	/* Connect i2c pins*/

	// init pins
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

	I2C_Cmd(I2C, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

	I2C_Init(I2C, &I2C_InitStructure);
}

/**
 * @brief: Reads any number of bytes from the device at the given address.
 * 			Ensure that your device supports reading large contingent spaces
 * @param data:			Buffer to read into     
 * @param DeviceAddr:	Device I2C slave address
 * @param ReadAddr:		Address in memory to read
 * @param len:			Number of bytes to read
 * @return				1 if success, 0 if failed
 */
int I2C_BufferRead(uint8_t* data, uint8_t DeviceAddr, uint16_t ReadAddr, uint8_t len)
{
	int32_t I2C_TimeOut = I2C_TIMEOUT_MAX;

	while (I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}
	I2C_TimeOut = I2C_TIMEOUT_MAX;

	I2C_AcknowledgeConfig(I2C, ENABLE);			// Enable I2C acknowledgement if it is already disabled by other function 

	I2C_GenerateSTART(I2C, ENABLE);				// Send I2C START condition
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT )) /* EV5 */
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}
	I2C_TimeOut = I2C_TIMEOUT_MAX;

	I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Transmitter );	// Send M24LR04E-R slave address for write 
	while ((!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )))/* EV6 */
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	if (len == 1)
		I2C_AcknowledgeConfig(I2C, DISABLE);		// If single byte disable ACK before reading the data //
	else
		I2C_AcknowledgeConfig(I2C, ENABLE);

#ifdef ADDR_16_bit
	I2C_SendData(I2C, (uint8_t) (ReadAddr >> 8)); 								// Send the reg_address to read from
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

#endif

	I2C_SendData(I2C, (uint8_t) (ReadAddr)); 								// Send the reg_address to read from

	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	I2C_GenerateSTART(I2C, ENABLE); 							// Send STRAT condition again //
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT ))	// Test on EV5 and clear it start bit has been sent
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Receiver );
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	I2C_TimeOut = I2C_TIMEOUT_MAX;
	while (len)
	{
		if (I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_RECEIVED ))
		{
			*data = I2C_ReceiveData(I2C );
			if (len == 2) 										// This is the reciprocal of two data
			{
				I2C_AcknowledgeConfig(I2C, DISABLE);
			} 		// Disable Acknowledgment
			if (len == 1) 										// This is the last data
			{
				I2C_GenerateSTOP(I2C, ENABLE);
			} 				// Send STOP Condition
			data++;
			len--;
		}
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	return 1;
}

/**
 * @brief:				Writes one byte to the specified address in memory
 * @param data:			pointer to Byte to be written.
 * @param DeviceAddr:	Slave address of I2C device. 
 * @param WriteAddr:	Address in memory to be written.
 * @return
 */
int I2C_ByteWrite(uint8_t* data, uint8_t DeviceAddr, uint16_t WriteAddr)
{
	int32_t I2C_TimeOut = I2C_TIMEOUT_MAX;

	if (I2C_GetStatus(DeviceAddr) == ERROR)					// waits for the I2C to be ready for next transfer after writes
	{
		return 0;
	}

#ifdef ADDR_16_bit
	I2C_SendData(I2C, (uint8_t) (WriteAddr >> 8)); 								// Send the reg_address to read from
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

#endif

	I2C_SendData(I2C, (uint8_t) (WriteAddr)); 								// Send the reg_address to read from
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	I2C_SendData(I2C, *data);
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))			// Test on EV8 and clear it
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}
	I2C_GenerateSTOP(I2C, ENABLE);												// Send STOP condition
	return 1;
}

/**
 * @brief:				Writes multiple bytes to the specified address in memory
 * @warning!!			Most devices have a limit to the number of bytes that can be written sequentially (usually confined to a block). check datasheet.
 * @param data:			pointer to data to be written.
 * @param DeviceAddr:	Slave address of I2C device. 
 * @param WriteAddr:	Address in memory to be written.
 * @return
 */
int I2C_BufferWrite(uint8_t* data, uint8_t DeviceAddr, uint16_t WriteAddr, uint8_t len)
{
	int i;
	int32_t I2C_TimeOut = I2C_TIMEOUT_MAX;

	if (I2C_GetStatus(DeviceAddr) == ERROR)					// waits for the I2C to be ready for next transfer after writes
	{
		return 0;
	}

#ifdef ADDR_16_bit
	I2C_SendData(I2C, (uint8_t) (WriteAddr >> 8)); 								// Send the reg_address to read from
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}
#endif

	I2C_SendData(I2C, (uint8_t) (WriteAddr)); 								// Send the reg_address to read from
	while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))
	{
		if (I2C_TimeOut-- <= 0)
			i2c_timeout;
	}

	I2C_TimeOut = I2C_TIMEOUT_MAX;
	for (i = 0; i < len; i++)
	{
		I2C_SendData(I2C, *data++);
		while (!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED ))			// Test on EV8 and clear it
		{
			if (I2C_TimeOut-- <= 0)
				i2c_timeout;
		}
	}
	I2C_GenerateSTOP(I2C, ENABLE);												// Send STOP condition

	return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  Checks the Chip status. (some devices indicate whether or not they are still busy by not sending an ACK...)
 * @param  None
 * @retval ErrorStatus: status (ERROR if the device timed out, otherwise SUCCESS).
 */
ErrorStatus I2C_GetStatus(uint8_t DeviceAddr)
{
	uint32_t I2C_TimeOut = I2C_TIMEOUT_MAX;

	while (1)
	{
		I2C_ClearFlag(I2C, I2C_FLAG_AF );

		I2C_AcknowledgeConfig(I2C, ENABLE);			// Enable I2C acknowledgement if it is already disabled by other function 

		I2C_GenerateSTART(I2C, ENABLE);				// Send I2C START condition
		while ((!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT )))
		{
			if (I2C_TimeOut-- <= 0)
				i2c_timeout;
		}

		I2C_TimeOut = I2C_TIMEOUT_MAX;

		I2C_Send7bitAddress(I2C, DeviceAddr, I2C_Direction_Transmitter );	// Send M24LR04E-R slave address for write 
		while ((!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )))/* EV6 */
		{
			if (I2C_TimeOut-- <= 0)
				i2c_timeout;
			if (I2C_GetFlagStatus(I2C, I2C_FLAG_AF ))
				break;
		}

		if (I2C_GetFlagStatus(I2C, I2C_FLAG_AF ) != 0x00)
			continue;
		else
			return SUCCESS;
	}
}

int I2C_timeout(void)
{
	F103_i2c_init();

//	while (1)
//	{}

	return 0;
}

/**
 * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
 * @param  None
 * @retval None
 */
void F103_i2c_DeInit(void)
{
	/* sEE_I2C Peripheral Disable */
	I2C_Cmd(I2C, DISABLE);

	/* sEE_I2C DeInit */
	I2C_DeInit(I2C );

	/*!< sEE_I2C Periph clock disable */
	RCC_APB1PeriphClockCmd(I2C_CLK, DISABLE);
}

