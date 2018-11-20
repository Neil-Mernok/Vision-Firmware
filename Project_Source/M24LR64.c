/*
 * M24LR64.c
 *
 * Created: 2012/10/09 03:25:09 PM
 *  Author: Kobus Goosen
 */

#include "M24LR64.h"
uint8_t device_found = 0;

#ifdef USE_HAL_DRIVER
#include "i2c.h"

//----------------------------------------------------------------------------------
//  uint8_t RFID_Write_byte(uint16_t addr, uint8_t data)
//
//  DESCRIPTION:
//    Write a byte to device, starting at internal device address "addr".
//----------------------------------------------------------------------------------
uint8_t RFID_Write_byte(uint16_t addr, uint8_t data)
{
	HAL_StatusTypeDef r;
	r = HAL_I2C_Mem_Write(&M24_i2c, DevM24LR64, addr, I2C_MEMADD_SIZE_16BIT, &data, 1, 10);
	if (r == HAL_OK)
	{
		r = HAL_I2C_IsDeviceReady(&M24_i2c, DevM24LR64, 1000, 10);
		if (r != HAL_OK)
			return 0;
		return 4;
	}
	r = HAL_I2C_IsDeviceReady(&M24_i2c, DevM24LR64, 1000, 10);
	if (r != HAL_OK)
		return 0;
	return 0;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Write_4bytes(uint16_t addr, uint8_t* data)
//
//  DESCRIPTION:
//  Writes 4 bytes consecutively (the most the chip can) to device, starting at internal device address "addr".
// make sure addr is in the start of a block, ie addr bit1 and addr bit0 = 0
//----------------------------------------------------------------------------------
uint8_t RFID_Write_4bytes(uint16_t addr, uint8_t* data)
{
	HAL_StatusTypeDef r;
	r = HAL_I2C_Mem_Write(&M24_i2c, DevM24LR64, addr, I2C_MEMADD_SIZE_16BIT, data, 4, 10);
	if (r == HAL_OK)
	{
		r = HAL_I2C_IsDeviceReady(&M24_i2c, DevM24LR64, 1000, 10);
		if (r != HAL_OK)
			return 0;
		return 4;
	}

	else
		return 0;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Write_bytes(uint16_t addr, uint8_t* data)
//
//  DESCRIPTION:
//  Writes many bytes consecutively (in a bunch of 4 bytes packets) to device, starting at internal device address "addr".
// make sure addr is in the start of a block, ie addr bit1 and addr bit0 = 0
//----------------------------------------------------------------------------------
uint8_t RFID_Write_bytes(uint16_t addr, uint8_t* data, uint8_t Len)
{
	uint8_t status, i = 0;

	while ((addr & 0b00000011) != 0)
	{
		status = RFID_Write_byte(addr, data[i++]);
		Len--;
		addr++;
		if (Len == 0)
			return status;
	}

	while (Len > 3)
	{
		status = RFID_Write_4bytes(addr, &data[i]);
		Len = Len - 4;
		i = i + 4;
		addr = addr + 4;
//		if(Len<4 && Len != 0)
//			DelayUs(5000);
	}

	while (Len > 0)
	{
		status = RFID_Write_byte(addr, data[i++]);
		Len--;
		addr++;
	}

	return status;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_byte(uint16_t addr)
//
//  DESCRIPTION:
//    Read a byte from device, starting at internal device address "addr".
//----------------------------------------------------------------------------------
uint8_t RFID_Read_byte(uint16_t addr)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&M24_i2c, DevM24LR64, addr, I2C_MEMADD_SIZE_16BIT, &data, 1, 20);
	return data;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_bytes(uint16_t addr, uint8_t* data, uint16_t Len)
//
//  DESCRIPTION:
//  Read continuously from device, starting at internal device address "addr".
//  unitl Len bytes have been read.
//----------------------------------------------------------------------------------
uint8_t RFID_Read_bytes(uint16_t addr, uint8_t* data, uint16_t Len)
{
	HAL_StatusTypeDef r;
	r = HAL_I2C_Mem_Read(&M24_i2c, DevM24LR64, addr, I2C_MEMADD_SIZE_16BIT, data, Len, 20);
	if (r == HAL_OK)
		return Len;
	else
		return 0;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_UID(uint8_t* UID_data)
//
//  DESCRIPTION:
//  Reads the 8 bytes for the system uid
//----------------------------------------------------------------------------------
uint8_t RFID_Read_UID(uint8_t* UID_data)
{
	HAL_StatusTypeDef r;
	r = HAL_I2C_Mem_Read(&M24_i2c, DevM24LR64_system, 2324, I2C_MEMADD_SIZE_16BIT, UID_data, 8, 20);
	if (r == HAL_OK)
		return 8;
	else
		return 0;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_UID(uint8_t* UID_data)
//
//  DESCRIPTION:
//  Reads the 8 bytes for the system uid
//----------------------------------------------------------------------------------
uint8_t RFID_Check_FieldOn(void)
{
	HAL_StatusTypeDef r;
	uint8_t Control_byte = 0;
	r = HAL_I2C_Mem_Read(&M24_i2c, DevM24LR64_system, 2336, I2C_MEMADD_SIZE_16BIT, &Control_byte, 1, 20);
	if (r == HAL_OK)
		return Control_byte & 0x02;			// only interested in the 2nd LSB of the control byte.;
	else
		return 0;
}


int EEPROM_data_calc;
static int EEPROMadrBlock= 0;
int usedindex = 0;
uint16_t counterEEPROM = 0;
int percentage = 0;
bool sizeCalculated_flag = false;

void EEPROMsizeCalc(void)
{

	if(!sizeCalculated_flag)
		for(int i=0; i<128;i++)
		{
			if((EEPROMadrBlock>=0)&&(EEPROMadrBlock<8192)&&(!sizeCalculated_flag))
				{
					EEPROM_data_calc = RFID_Read_byte(EEPROMadrBlock);
					if(EEPROM_data_calc!=0xff)
					{
						usedindex++;
					}
					EEPROMadrBlock = EEPROMadrBlock+1;
					counterEEPROM++;
				}
				else if(EEPROMadrBlock>=8192)
				{
					sizeCalculated_flag = true;
					usedindex = usedindex/8;
					percentage = (int)((1- (float)usedindex/8192)*100);
					i=128;
					//EEPROMadrBlock= 8188;
				}
		}

}



#else
#include "F103_i2c.h"
#include "Global_Variables.h"

//----------------------------------------------------------------------------------
//  uint8_t RFID_Write_byte(uint16_t addr, uint8_t data)
//
//  DESCRIPTION:
//    Write a byte to device, starting at internal device address "addr".
//----------------------------------------------------------------------------------
uint8_t RFID_Write_byte(uint16_t addr, uint8_t data)
{
	return I2C_ByteWrite(&data, DevM24LR64, addr);
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Write_4bytes(uint16_t addr, uint8_t* data)
//
//  DESCRIPTION:
//  Writes 4 bytes consecutively (the most the chip can) to device, starting at internal device address "addr".
// make sure addr is in the start of a block, ie addr bit1 and addr bit0 = 0
//----------------------------------------------------------------------------------
uint8_t RFID_Write_4bytes(uint16_t addr, uint8_t* data)
{
	return I2C_BufferWrite(data, DevM24LR64, addr, 4);
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Write_bytes(uint16_t addr, uint8_t* data)
//
//  DESCRIPTION:
//  Writes many bytes consecutively (in a bunch of 4 bytes packets) to device, starting at internal device address "addr".
// make sure addr is in the start of a block, ie addr bit1 and addr bit0 = 0
//----------------------------------------------------------------------------------
uint8_t RFID_Write_bytes(uint16_t addr, uint8_t* data, uint8_t Len)
{
	uint8_t status, i=0;

	while ((addr & 0b00000011) != 0)
	{
		status = RFID_Write_byte(addr, data[i++]);
		Len--;
		addr++;
		if (Len == 0) return status;
	}

	while (Len > 3)
	{
		status = RFID_Write_4bytes(addr, &data[i]);
		Len = Len-4;
		i = i+4;
		addr = addr+4;
	}

	while (Len > 0)
	{
		status = RFID_Write_byte(addr, data[i++]);
		Len--;
		addr++;
	}

	return status;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_byte(uint16_t addr)
//
//  DESCRIPTION:
//    Read a byte from device, starting at internal device address "addr".
//----------------------------------------------------------------------------------
uint8_t RFID_Read_byte(uint16_t addr)
{
	uint8_t data;
	I2C_BufferRead(&data, DevM24LR64, addr, 1);
	return data;
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_bytes(uint16_t addr, uint8_t* data, uint16_t Len)
//
//  DESCRIPTION:
//  Read continuously from device, starting at internal device address "addr".
//  unitl Len bytes have been read.
//----------------------------------------------------------------------------------
uint8_t RFID_Read_bytes(uint16_t addr, uint8_t* data, uint16_t Len)
{
	return I2C_BufferRead(data, DevM24LR64, addr, Len);
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_UID(uint8_t* UID_data)
//
//  DESCRIPTION:
//  Reads the 8 bytes for the system uid
//----------------------------------------------------------------------------------
uint8_t RFID_Read_UID(uint8_t* UID_data)
{
	return I2C_BufferRead(UID_data, DevM24LR64_system, 2324, 8);
}

//----------------------------------------------------------------------------------
//  uint8_t RFID_Read_UID(uint8_t* UID_data)
//
//  DESCRIPTION:
//  Reads the 8 bytes for the system uid
//----------------------------------------------------------------------------------
uint8_t RFID_Check_FieldOn(void)
{
	uint8_t Control_Byte = 0;
	if(I2C_BufferRead(&Control_Byte, DevM24LR64_system, 2336, 1))
		return Control_Byte & 0x02;			// only interested in the 2nd LSB of the control byte.
	else
		return 1;							// assume LF coil busy if we timed out. 
}


#endif
