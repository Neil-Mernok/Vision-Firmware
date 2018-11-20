/*
 * framing.h
 *
 *  Created on: Jun 23, 2015
 *      Author: Kobus
 */

#ifndef FRAMING_H_
#define FRAMING_H_

#include <stdint.h>
#include <stdbool.h>

#define SOF1 0x7E
#define SOF2 0xE7
#define EOF1 0x0A

#define SOF_RF_1 0x3A
#define SOF_RF_2 0xA3

#ifdef __cplusplus
extern "C" {
#endif

uint8_t* get_frame(uint8_t* data, uint8_t len, uint8_t* lenout);
bool check_frame_crc(uint8_t* data, uint8_t len);
bool check_frame(uint8_t* data);

uint8_t* get_RF_frame(uint8_t* data, uint8_t len, uint8_t* lenout);
bool check_RF_frame_crc(uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* FRAMING_H_ */
