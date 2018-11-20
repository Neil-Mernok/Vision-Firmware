/*
 * TX.c
 *
 * Created: 2012/10/11 10:45:22 AM
 *  Author: J.L. Goosen
 * holds functions used to send packages with the CC1101
 */

#include "TX.h"
#include "framing.h"

uint8_t packetSent = 0;
//----------------------------------------------------------------------------------
//  uint8 txSendPacket(uint8* data, uint8 length)
//
//  DESCRIPTION:
//    Send a packet that is smaller than the size of the FIFO, making it possible
//    to write the whole packet at once. Wait for the radio to signal that the packet
//    has been transmitted.
//
//  ARGUMENTS:
//    data   - Data to send. First byte contains length byte
//    length - Total length of packet to send
//
//  RETURNS:
//    This function always returns 0.
//----------------------------------------------------------------------------------
uint8_t txSendPacket(uint8_t* data, uint8_t length)
{
	int counter = 0;
	uint8_t i, status, packet_length;
	packetSent = 0;

	data = get_RF_frame((uint8_t*)data, length, &packet_length);
	length = packet_length;

	for (i = length; i > 0; i--)
		data[i] = data[i - 1];
	data[0] = length;

	////	in case the radio is in sleep mode,	////////
	////	try put in idle until its responsive	////
	do // wake radio up, if sleeping. 
	{
		status = halRfStrobe(CC1100_SIDLE);			// must be in idle to flush fifo	
	} while( (status & 0xF0) != CC1100_STATE_IDLE); // wait for device to wake up from sleep (~600us)
	////////////////////////////////////////////////////
		
	////	CCA before TX	////////////////////////////
	////	set the radio in RX, then send. this way it will wait for clear channel first. 
	halRfStrobe(CC1100_SIDLE);
	// Set radio in RX mode
	halRfStrobe(CC1100_SRX);
	////////////////////////////////////////////////////
	// Write data to FIFO
//	halRfWriteFifo(&length, 1);
//	halRfWriteFifo(data, length);
	halRfWriteFifo(data, length + 1);

	// Set radio in transmit mode
	halRfStrobe(CC1100_SIDLE);
	halRfStrobe(CC1100_STX);

	/// TODO: enter a wait here for the TX to complete. seems to be unstable without this. 
	while (packetSent == 0)
	{
		if (counter++ > 20000)
			return 1;
	}
	halRfStrobe(CC1100_SIDLE);
	halRfStrobe(CC1100_SFTX);

	return (0);
}

