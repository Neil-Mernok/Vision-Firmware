/*
 * LF_TX.h
 *
 *  Created on: Sep 9, 2014
 *      Author: KobusGoosen
 */

#ifndef LF_TX_H_
#define LF_TX_H_

#include <stdint.h>
#include <stdbool.h>

//#include "Global_Variables.h"

#define LF_block_time 300			// how long it takes to send LF. Wait this long after another tag before sending your own LF.

#ifdef __cplusplus
extern "C" {
#endif

struct _LF_Params
{
	int bitcount;	// counts down data bits sent, i.e. manchester symbols   
	int state;		// 0= off, 1= carrier burst, 2= data
	bool manch;		// toggled to indicate which manchester cycle we're in
	bool data_ready;// set this once the BSID has been encoded into data
	uint64_t data;	// TX data, including pattern and sync bits. 
};

extern struct _LF_Params LF_Params;

void LF_send(void);
int LF_send_bits(void);
void LF_form_packet(uint16_t vehicle_ID, uint8_t slave_ID, bool crc);
void PWM_pins_change(bool PWM);						// this function is implemented in the appropriate GPIO c file.
int8_t makeCrc4Check(uint64_t theData, uint16_t theCrc4);

#ifdef __cplusplus
}
#endif

#endif /* LF_TX_H_ */
