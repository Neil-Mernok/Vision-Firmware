/*
 * Created on: Mar 10, 2013
 * Company: Mernok Elektronik 
 * Author: J.L. Goosen
 */
//#include "Update.h"
#include "Global_Variables.h"

/****************************************************************/
/****************************************************************/

uint8_t F_FlashErase = 0;

uint8_t	Run_Application = 0;
uint8_t	Run_Bootloader = 0;

uint32_t deserialize_uint32(unsigned char *buffer) {
    uint32_t res = *((uint32_t *) buffer);
    return res;
}

#ifdef USE_HAL_DRIVER
/****************************************************************/
/* receives up to 4096 bytes from the boot master and stores in
 * flash If not, clear all those
 */
/****************************************************************/
uint32_t Flash_Packet(uint8_t* data, uint32_t Len)
{
	static uint32_t LastPGAddress = APPLICATION_ADDRESS;
	uint32_t i=0;
	uint32_t errcount = 0;
	
	// While file still contain data or while the total counter has not reach the total file size
	if ((UPDATE_Flags.Update_Pointer <  UPDATE_Flags.Update_FileSize))
	{
		//This function programs the required flash memory a word at a time
#ifdef STM32L1
		for (i=0; i< Len; i+=128)
		{
			if(HAL_FLASHEx_HalfPageProgram(LastPGAddress + i, (uint32_t*)&data[i]) != HAL_OK)
			{
				errcount++;
				return i;
			}
		}
#else
		for (i=0; i< Len; i+=4)
		{
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, LastPGAddress + i, (uint64_t)deserialize_uint32(&data[i])) != HAL_OK)
			{
				errcount++;
				return i;
			}
		}
#endif
		//Update last programmed address value
		LastPGAddress += Len;
		UPDATE_Flags.Update_Pointer += Len;

		//trigger update end if the total counter reaches file size
		if (UPDATE_Flags.Update_Pointer >=  UPDATE_Flags.Update_FileSize)
		{
			UPDATE_Flags.F_UPDATE_END = time_now() + 100;
		}
	}
	return i;
}

/****************************************************************/
/* receives up to 4096 bytes from the boot master and stores in
 * flash at the specified address
 */
/****************************************************************/
uint32_t Flash_Packet_at_adr(uint8_t* data, uint32_t Len, uint32_t start_adr)
{
	static uint32_t LastPGAddress = APPLICATION_ADDRESS;
	uint32_t i = 0;
	uint32_t errcount = 0;

	LastPGAddress = APPLICATION_ADDRESS + start_adr;
	UPDATE_Flags.Update_Pointer = start_adr;
	
	// While file still contain data or while the total counter has not reach the total file size
	if ((UPDATE_Flags.Update_Pointer <  UPDATE_Flags.Update_FileSize))
	{
		//This function programs the required flash memory a word at a time
#ifdef STM32L4
		for (i=0; i< Len; i+=8)
		{
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, LastPGAddress + i, *(uint64_t*)(&data[i])) != HAL_OK)
			{
				errcount++;
				return i;
			}
		}
#else
		for (i=0; i< Len; i+=128)
		{
			if(HAL_FLASHEx_HalfPageProgram(LastPGAddress + i, (uint32_t*)&data[i]) != HAL_OK)
			{
				errcount++;
				return i;
			}
		}
#endif	

		//Update last programmed address value
		LastPGAddress += Len;
		UPDATE_Flags.Update_Pointer += Len;


		//trigger update end if the total counter reaches file size
		if (UPDATE_Flags.Update_Pointer >=  UPDATE_Flags.Update_FileSize)
		{
			UPDATE_Flags.F_UPDATE_END = time_now() + 100;
		}
	}
	return i;
}
#else
/****************************************************************/
/* receives up to 4096 bytes from the boot master and stores in
 * flash If not, clear all those
 */
/****************************************************************/
uint32_t Flash_Packet(uint8_t* data, uint32_t Len)
{
	static uint32_t LastPGAddress = APPLICATION_ADDRESS;
	FLASH_Status stat;
	uint32_t i;
	uint32_t errcount = 0;

	// While file still contain data or while the total counter has not reach the total file size
	if ((UPDATE_Flags.Update_Pointer <  UPDATE_Flags.Update_FileSize))
	{
		//This function programs the required flash memory a word at a time
		for (i=0; i< Len; i+=4)
		{
			stat = FLASH_ProgramWord(LastPGAddress + i, deserialize_uint32(&data[i]));
			if (stat != FLASH_COMPLETE)
			{
				errcount++;
				return i;
			}
		}

		//Update last programmed address value
		LastPGAddress += Len;
		UPDATE_Flags.Update_Pointer += Len;

		//USART_Sendbyte(COM_1,0x07);


		//trigger update end if the total counter reaches file size
		if (UPDATE_Flags.Update_Pointer >=  UPDATE_Flags.Update_FileSize)
		{
			UPDATE_Flags.F_UPDATE_END = time_now() + 100;
		}
	}
	return i;
}

/****************************************************************/
/* receives up to 4096 bytes from the boot master and stores in
 * flash at the specified address
 */
/****************************************************************/
uint32_t Flash_Packet_at_adr(uint8_t* data, uint32_t Len, uint32_t start_adr)
{
	static uint32_t LastPGAddress = APPLICATION_ADDRESS;
	FLASH_Status stat;
	uint32_t i;
	uint32_t errcount = 0;

	LastPGAddress = APPLICATION_ADDRESS + start_adr;
	UPDATE_Flags.Update_Pointer = start_adr;
	
	// While file still contain data or while the total counter has not reach the total file size
	if ((UPDATE_Flags.Update_Pointer <  UPDATE_Flags.Update_FileSize))
	{
		//This function programs the required flash memory a word at a time
		for (i=0; i< Len; i+=4)
		{
			stat = FLASH_ProgramWord(LastPGAddress + i, deserialize_uint32(&data[i]));
			if (stat != FLASH_COMPLETE)
			{
				errcount++;
				return i;
			}
		}

		//Update last programmed address value
		LastPGAddress += Len;
		UPDATE_Flags.Update_Pointer += Len;

		//USART_Sendbyte(COM_1,0x07);

		//trigger update end if the total counter reaches file size
		if (UPDATE_Flags.Update_Pointer >=  UPDATE_Flags.Update_FileSize)
		{
			UPDATE_Flags.F_UPDATE_END = time_now() + 100;
		}
	}
	return i;
}
#endif



