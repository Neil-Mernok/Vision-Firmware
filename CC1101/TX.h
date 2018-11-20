/*
 * TX.h
 *
 * Created: 2012/10/11 10:45:36 AM
 *  Author: user
 */ 


#ifndef TX_H_
#define TX_H_

//---------- Defines and includes ---------------

#include "hal_defs.h"
#include "hal_spi.h"
#include "hal_rf.h"
#include "cc1100.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t packetSent;
//extern uint8_t packetReceived;


//---------- local function prototypes ----------


uint8_t txSendPacket(uint8_t* data, uint8_t length);


#ifdef __cplusplus
}
#endif
#endif /* TX_H_ */
