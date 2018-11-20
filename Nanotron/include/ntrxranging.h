/* $Id$ */

/**
 * @file ntrxranging.h
 * @date 2009-07-07
 * @author Christian Bock
 * @c (C) 2009 Nanotron Technologies
 * @brief Ranging support functions.
 *
 * This file contains the definitions of the ranging support
 * and calculation of a ranging cycle with the nanoLOC chip.
 *
 * $Revision: 6844 $
 * $Date: 2009-08-13 09:06:26 +0200 (Do, 13 Aug 2009) $
 * $LastChangedBy: sra $
 * $LastChangedDate: 2009-08-13 09:06:26 +0200 (Do, 13 Aug 2009) $
 */

#ifndef NTRXRANGING_H
#define NTRXRANGING_H

/**
 * @def RANGE_DEBUG
 * @brief Enables debugging mode.
 */
//#define RANGE_DEBUG	1

/**
 * @def DEBUG_P
 * @brief Debugging output using printf. Can be globally disabled with
 * @ref RANGE_DEBUG .
 */
#include <stdio.h>
#ifdef RANGE_DEBUG
#	define DEBUG_P(args, ...) printf_P(PSTR(args), ##__VA_ARGS__)
#else  /*RANGE_DEBUG*/
#	define DEBUG_P(args, ...)
#endif /*RANGE_DEBUG*/

#include "ntrxtypes.h"

/** @brief Structure for the ranging result.
 *
 * When a ranging cycle is finished a ranging result message will
 * be send to the upper layer. The message will have the following
 * format.
 */
typedef struct
{
	uint8_t error; /**< status value of the ranging cycle */
	double distance; /**< result of measurement */
	double prev_distance; /// Francois: store previous result of measurement.
	uint8_t distance_error; /// Francois: keep track of previous distance error
	int antenna_offset;	/// Kobus: place to store antenna offset of remote tag. 
	AddrT addr; /**< address of remote ranging station */
} RangingMsgT;

#define MSG_TYPE_DATA			0
#define MSG_TYPE_RANGING		1

/* timeout in ms */
#define RANGING_TIMEOUT 250

/**
 * @def RANGING_PROTOCOL_LEN
 * @brief Defines the length of the ranging protocol.
 */
#define RANGING_PROTOCOL_LEN 1

/**
 * @def RANGING_TYPE_3W_A
 * @brief Defines the ranging type.
 * This type of ranging consists of one request packet, and two answer packets
 * with following behavior: ==>, <==, <== (distance is local generated).
 * The station which requests the ranging, gets the distance as indication in
 * the registered callback function.
 */
#define RANGING_TYPE_3W_A		0

/**
 * @def RANGING_TYPE_3W_B
 * @brief Defines the ranging type.
 * This type of ranging consists of one request packet, and two answer packets
 * with following behavior: ==>, <==, ==> (distance is remote generated).
 * The station which receives the request gets the distance as indication in
 * the registered callback function.
 */
#define RANGING_TYPE_3W_B		1

/**
 * @def RANGING_TYPE_2W_PP
 * @brief Defines the ranging type.
 * This type of ranging consists of one request packet, and one answer packet
 * with following behavior: ==>, <== (distance is remote generated).
 * The station which receives the request gets the distance as indication in
 * the registered callback function.
 * Advantage: Only 2 packets generate a distance (low power consumption)
 * Disadvantage: The first ranging dont generate a distance. (delay)
 */
#define RANGING_TYPE_2W_PP	2


/**
 * @brief This function try to start a ranging process.
 * @param *msg this is the pointer to send message
 * @note This function is called ever on a PD_RANGING_REQUEST.
 * If transmitter is free, and no other ranging is running, this
 * function starts a new ranging process.
 *
 * RANGING_TYPE_3W_A = --->,<---,<--- (single measurement, distance local known)
 * RANGING_TYPE_3W_B = --->,<---,---> (single measurement, distance remote known)
 * RANGING_TYPE_2W_PP= --->,<--- (continue, distance remote known)
 *
 */
int NTRXRangingRequest(MsgT *msg);


/**
 * @brief This function initialize the local ranging memory.
 * @note This function must called once during initialisation to
 * ensure that the local ranging memory is in initial position.
 *
 */
void NTRXRangingInit(void);


/**
 * @brief This function collects data from a rx packet.
 * @param *msg this is the pointer to received message
 * @note This function is called on rx interrupt, after an ranging
 * packet is received. Depending on the RANGING_TYPE next steps will
 * processed.
 *
 */
void NTRXRangingRX(MsgT *msg);


/**
 * @brief This function collects data from a hardware acknowledge.
 * @param *msg this is the message pointer to the sended message
 * @note This function is called on hardware ack, after an ranging
 * packet is send. Depending on the RANGING_TYPE next steps will
 * proccessed.
 *
 */
void NTRXRangingACK(void);


/**
 * @brief This function is called on baseband timer interrupt (ranging timeouts)
 * @note baseband timer starts on every successful ranging request and
 * if no answer is received the interrupt occure and ranging will abort.
 * This prevents blocking the phy for new remote ranging requests.
 */
void NTRXRangingInterrupt(void);

/**
 * @brief This function returns the internal status of ranging.
 * @note This function should be called before setting the driver
 * into power down, to ensure that ranging is finished.
 * @return TRUE if no ranging is pending, otherweise FALSE
 *
 */
bool_t NTRXRangingIsIDLE(void);

/**
 * @brief Function pointer type which the PHY uses to get permission for
 * automatic ranging measurements with the given MAC address.
 * @param macAddr MAC address of the remote ranging device.
 * @param payload user data
 * @param len length of data
 * @returns TRUE if ranging with the remote device will be permitted; FALSE
 * otherwise.
 * @see RangingRegisterCallback
 *
 * Implement a function with this interface above the PHY layer that decides
 * whether or not the PHY may perform ranging with the remote station. Register
 * your function to the PHY by calling @ref RangingRegisterCallback .
 */
typedef bool_t (*permissionfn_t)(uint8_t *macAddr, uint8_t *data, uint8_t *len);



/**
 * @brief Function pointer which the PHY uses to request memory for its
 * internal ranging states regarding the given remote MAC address.
 * @param memSize Size of memory which the PHY needs to perform ranging with
 * the given remote station.
 * @param macAddr MAC address of the remote ranging partner.
 * @returns Pointer to a piece of memory of the requested size that the PHY
 * may use. NULL if there is no memory available for the given remote station.
 * @see RangingRegisterCallback
 *
 * Implement a function with this interface above the PHY layer if you wish to
 * perform ping pong ranging. The PHY needs a seperate piece of memory to save
 * its internal ranging state for @em each remote station. The user is
 * responsible for the memory management. i.e. memory allocation and assignment
 * corresponding to the given MAC address. Register your function to the PHY
 * by calling @ref RangingRegisterCallback .
 */
typedef void* (*mem_assignfn_t)(uint16_t memSize, uint8_t *macAddr);



/**
 * @brief Registers function pointers to ranging specific functions.
 * @param rgPermission If set the PHY will ask the user by this function for
 * permission to perform ranging with a given remote MAC address. Default
 * on startup for this parameter is NULL, i.e. ranging will always be performed.
 * @param rgGetMemory If set the PHY will call this function in order to
 * obtain a valid piece of memory to store internal ranging states while
 * ranging with a certain remote station. Default for this parameter is NULL,
 * i.e. the PHY will be capable to perform ranging with @em only @em one
 * ranging partner.
 * @note Ping-Pong ranging will be rejected if @ref rgGetMemory is not assigned.
 */
void RangingRegisterCallback(permissionfn_t rgPermission, mem_assignfn_t rgGetMemory);



//// Kobus: add antenna offset 
extern uint16_t local_antenna_offset;

#endif /* NTRXRANGING_H */
