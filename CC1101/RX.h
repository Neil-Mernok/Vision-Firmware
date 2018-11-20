/*
 * RX.h
 *
 * Created: 2012/10/11 10:46:28 AM
 *  Author: user
 */ 


#ifndef RX_H_
#define RX_H_

//---------- Defines and includes ---------------

#include "hal_defs.h"
#include "hal_spi.h"
#include "hal_rf.h"
#include "cc1100.h"

#ifdef __cplusplus
extern "C" {
#endif

#define highest_RSSI 	96
#define lowest_RSSI		-32

//---------- local function prototypes ----------

uint8_t rxRecvPacket(uint8_t* data, uint8_t* length, uint8_t* RSSI, int timeout);
uint8_t rxRecvReentrant(uint8_t* data, uint8_t* length, uint8_t* RSSI, int timeout);


extern uint8_t packetReceived;

#ifdef __cplusplus
}
#endif


#endif /* RX_H_ */
