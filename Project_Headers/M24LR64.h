/*
 * M24LR64.h
 *
 * Created: 2012/10/09 03:25:33 PM
 *  Author: user
 */ 

#include <inttypes.h>
#include "Global_Variables.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M24LR64_H_
#define M24LR64_H_


#define DevM24LR64			0xA6
#define DevM24LR64_system	0xAE

uint8_t RFID_Write_byte(uint16_t addr, uint8_t data);
uint8_t RFID_Write_4bytes(uint16_t addr, uint8_t* data);
uint8_t RFID_Write_bytes(uint16_t addr, uint8_t* data, uint8_t Len);
uint8_t RFID_Read_byte(uint16_t addr);
uint8_t RFID_Read_bytes(uint16_t addr, uint8_t* data, uint16_t Len);
uint8_t RFID_Read_UID(uint8_t* UID_data);
uint8_t RFID_Check_FieldOn(void);


//TODO:Neil testing EEPROM size calculation
void EEPROMsizeCalc(void);

#ifdef __cplusplus
}
#endif

#endif /* M24LR64_H_ */
