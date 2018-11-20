
/**
 * @file ntrxtypes.h
 * @date 2007-Dez-11
 * @author S. Rohdemann
 * @c (C) 2007 Nanotron Technologies
 * @brief Type definitions for data types.
 *
 * @note BuildNumber = "BuildNumber : 7951";
 *
 * @note This file contains the generic datatype definitions of the nTRX driver.
 *    These type definitions isolate the nTRX code from dependencies on the
 *    hardware environment for which the nTRX is compiled.
 *
 * $Revision: 7037 $
 * $Date: 2009-10-02 16:40:28 +0200 (Fr, 02 Okt 2009) $
 * $LastChangedBy: ak $
 * $LastChangedDate: 2009-10-02 16:40:28 +0200 (Fr, 02 Okt 2009) $
 */
/*
 * $Log$
 */
#include <config.h>

#ifndef	_NTRXTYPES_H
#define	_NTRXTYPES_H
#include <stdint.h>
#define	BITS2BYTES(n)	(n/8)		/* calc. bits into bytes */

#ifndef	NULL
#define	NULL	(0)
#endif	/* NULL */

#ifndef	NIL
#define	NIL	(0)
#endif	/* NIL */

#ifdef __cplusplus
extern "C" {
#endif


#ifdef	TRUE
#	undef	TRUE
#endif	/* TRUE */
#ifdef	FALSE
#	undef	FALSE
#endif	/* FALSE */

/** @brief Bool type. */
typedef	enum	{
	FALSE = 0,
	TRUE = 1
}	bool_t;
/** @brief 48 bit address type. */
typedef	uint8_t AddrT[6];


/**
 * @brief General purpose message structure.
 *
 * This general purpose message structure is used for all
 * layer to layer communication.
 */
typedef struct
{
	uint8_t prim;			/**< The primitive of the message. */
	AddrT  addr;			/**< MAC address of the message. */
	uint8_t len;			/**< Payload length. */
	uint8_t *pdu;			/**< Auxiliary pointer into payload (use for protocol interleaving).*/
	uint8_t data[CONFIG_MAX_PACKET_SIZE];/**< Payload of the message. */

	uint8_t status;			/**< Status value for confirmations */
	uint16_t value;			/**< Configuration value of a layer setting */
	uint8_t attribute;
#	ifdef CONFIG_NTRX_SNIFFER
	uint32_t count;		/**< Sniffer stuff */
	AddrT  rxAddr;			/**< MAC address of the recipient */
	uint8_t frameType;		/**< Frame type of the message */
	uint8_t extBits;		/**< Extended bits in the header for ranging */
#	endif /* CONFIG_NTRX_SNIFFER */
} MsgT;


/**
 * @brief Function pointer type for upstream callback functions.
 * @param msg The received message that will be delivered upstream.
*/
typedef void (*callbackfn_t)(MsgT *msg);


/**
 * @brief Timer structure used by the timerlib.
 *
 * This structure is equal to the clib timer structure.
 */
typedef	struct
{
	int16_t	tm_sec;
	int16_t	tm_min;
	int16_t	tm_hour;
	int16_t	tm_mday;
	int16_t	tm_mon;
	int16_t	tm_year;
	int16_t	tm_wday;
	int16_t	tm_yday;
	int16_t	tm_isdst;
}	MyTmT;


#ifdef __cplusplus
}
#endif

#endif	/* _NTRXTYPES_H */
