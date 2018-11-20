/*
 * M24LR64.c
 *
 * Created: 2012/10/09 03:25:09 PM
 *  Author: Kobus Goosen
 */ 

#include "M24LR64.h"
#include "F103_i2c.h"

#include "Global_Variables.h"


uint8_t device_found = 0;

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

