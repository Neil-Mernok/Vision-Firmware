/*
 * RX.c
 *
 * Created: 2012/10/11 10:46:37 AM
 * Author: J.L. Goosen
 * Holds functions used to receive packets from th RF. 
 */

#include "RX.h"
#include "Delay.h"

int awaiting_packet = 0;
uint8_t packetReceived = 0;

//----------------------------------------------------------------------------------
//  uint8_t rxRecvPacket(uint8_t* data, uint8_t* length, int8_t* RSSI)
//
//  DESCRIPTION:
//    Receive a packet that is smaller than the size of the FIFO, i.e. wait for the
//    complete packet to be received before reading from the FIFO. This function sets
//    the CC1100/CC2500 in RX and waits for the chip to signal that a packet is received.
//
//  ARGUMENTS:
//    data   - Where to write incoming data.
//    length - Length of payload.
//	  RSSI	 - Returns a 8 bit signed value showing the packets' receive strength.
//	  timeout- max number of milliseconds to wait before timing out.  
//  RETURNS:
//    0 if a packet was received successfully.
//    1 if chip is in overflow state (packet longer than FIFO).
//    2 if the length of the packet is illegal (0 or > 61).
//    3 if the CRC of the packet is not OK.
//    4 if the function timed out.
//----------------------------------------------------------------------------------

#ifdef FREERTOS_CONFIG_H
uint8_t rxRecvPacket(uint8_t* data, uint8_t* length, uint8_t* RSSI, int timeout)
{
	uint8_t packet_status[2];
	uint8_t status;
	int8_t RSSI_signed;
	uint8_t RSSI_holder;
//	packetReceived = FALSE;
	status = RX_OK;

// wait for access to SPI
	xSemaphoreTake(SPI_keeper, portMAX_DELAY);

// clear the ISR flag in case it was already set upon entry. 
	xSemaphoreTake(ccIrq, 0);

	//flush RX fifo
	halRfStrobe(CC1100_SFRX);
	// Set radio in RX mode
	halRfStrobe(CC1100_SRX);
	xSemaphoreGive(SPI_keeper);

//	awaiting_packet = 1;		// tell the interrupt we're expecting RX data

// Wait for incoming packet
/// TODO: fix this. Use a queue item to tell the IC that data has been received
	if(xSemaphoreTake(ccIrq, timeout) != pdTRUE)
	status = RX_NO_PACKET;

// wait for access to SPI
	xSemaphoreTake(SPI_keeper, portMAX_DELAY);
// Read first element of packet from the RX FIFO
	if (status != RX_NO_PACKET)
	{
		status = halRfReadFifo(length, 1);

		if ((status & CC1100_STATUS_STATE_BM) == CC1100_STATE_RX_OVERFLOW)
		{
			halRfStrobe(CC1100_SIDLE);
			halRfStrobe(CC1100_SFRX);
			status = RX_FIFO_OVERFLOW;
		}
		else if (*length == 0 || *length > 61)
		{
			halRfStrobe(CC1100_SIDLE);
			halRfStrobe(CC1100_SFRX);
			status = RX_LENGTH_VIOLATION;
		}
		else
		{
			// update status to reflect that we've seen a good RF packet
//			////////////////////Reader_Status.Com_status |= RF_Coms;

// Get payload
			halRfReadFifo(data, *length);

			// Get the packet status bytes [RSSI, LQI]
			halRfReadFifo(packet_status, 2);

			// Check CRC
			if ((packet_status[1] & CC1100_LQI_CRC_OK_BM) != CC1100_LQI_CRC_OK_BM)
			{
				status = RX_CRC_MISMATCH;
			}
			else
			{
				RSSI_signed = packet_status[0];
				RSSI_holder = RSSI_signed;

				if( RSSI_holder >= highest_RSSI )
				{
					*RSSI = 255;
				}
				else if ( RSSI_holder <= lowest_RSSI)
				{
					*RSSI= 0;
				}
				else
				{
					RSSI_holder = RSSI_holder * 2 + 64;
					*RSSI = RSSI_holder;
				}
//				*RSSI = *RSSI/2;
//				*RSSI = *RSSI-74;
				status = RX_OK;
			}
			//////////////////////////////////////
			halRfStrobe(CC1100_SIDLE);
			halRfStrobe(CC1100_SFRX);
			//////////////////////////////////////
		}
	}
	else
	{
		halRfStrobe(CC1100_SIDLE);
		halRfStrobe(CC1100_SFRX);
	}
	xSemaphoreGive(SPI_keeper);
	return(status);
}
#else
uint8_t rxRecvPacket(uint8_t* data, uint8_t* length, uint8_t* RSSI, int timeout)
{
	uint8_t packet_status[2];
	uint8_t status;
	int8_t RSSI_signed;
	int counter = 0, RSSI_holder;

	packetReceived = FALSE;
	status = RX_OK;

	do // wake radio up, if sleeping. 
	{
		status = halRfStrobe(CC1100_SIDLE);			// must be in idle to flush fifo	
	} while( (status & 0xF0) != CC1100_STATE_IDLE); // wait for device to wake up from sleep (~600us)
	//flush RX fifo		
	halRfStrobe(CC1100_SFRX);
	// Set radio in RX mode
	halRfStrobe(CC1100_SRX);

	// Wait for incoming packet
	/// TODO: fix this. Use a queue item to tell the IC that data has ben received
	while (!packetReceived)
	{
		Delay(1);
		counter++;
		if (counter > timeout)
		{
			status = RX_NO_PACKET;
			break;
		}
	}

	// Read first element of packet from the RX FIFO
	if (status != RX_NO_PACKET)
	{
		status = halRfReadFifo(length, 1);

		if ((status & CC1100_STATUS_STATE_BM) == CC1100_STATE_RX_OVERFLOW)
		{
			halRfStrobe(CC1100_SIDLE);
			halRfStrobe(CC1100_SFRX);
			status = RX_FIFO_OVERFLOW;
		}
		else if (*length == 0 || *length > 61)
		{
			halRfStrobe(CC1100_SIDLE);
			halRfStrobe(CC1100_SFRX);
			status = RX_LENGTH_VIOLATION;
		}
		else
		{
			// update status to reflect that we've seen a good RF packet
//			////////////////////Reader_Status.Com_status |= RF_Coms;

			// Get payload
			halRfReadFifo(data, *length);

			// Get the packet status bytes [RSSI, LQI]
			halRfReadFifo(packet_status, 2);

			// Check CRC
			if ((packet_status[1] & CC1100_LQI_CRC_OK_BM) != CC1100_LQI_CRC_OK_BM)
			{
				status = RX_CRC_MISMATCH;
			}
			else
			{
				RSSI_signed = packet_status[0];
				RSSI_holder = RSSI_signed;

				if (RSSI_holder >= highest_RSSI)
				{
					*RSSI = 255;
				}
				else if (RSSI_holder <= lowest_RSSI)
				{
					*RSSI = 0;
				}
				else
				{
					RSSI_holder = RSSI_holder * 2 + 64;
					*RSSI = RSSI_holder;
				}
//				*RSSI = *RSSI/2;
//				*RSSI = *RSSI-74;
				status = RX_OK;
			}
		}
	}
	else
	{
		halRfStrobe(CC1100_SIDLE);
		halRfStrobe(CC1100_SFRX);
	}
	return (status);
}

uint8_t rxRecvReentrant(uint8_t* data, uint8_t* length, uint8_t* RSSI, int timeout)
{
	uint8_t packet_status[2];
	uint8_t status = RX_OK;
	int8_t RSSI_signed;
	int RSSI_holder;
	static uint32_t start_time, increment;
	static uint8_t waiting = 0;

	// if we're not expecting a packet, start receiving again
	if (waiting == 0)
	{
		packetReceived = FALSE;

		do // wake radio up, if sleeping. 
		{
			status = halRfStrobe(CC1100_SIDLE);			// must be in idle to flush fifo	
		} while( (status & 0xF0) != CC1100_STATE_IDLE); // wait for device to wake up from sleep (~600us)

		// ---- Flush RX FIFO ----
		halRfStrobe(CC1100_SFRX);

		halRfStrobe(CC1100_SCAL);

		// ---- Set radio in RX mode ----
		halRfStrobe(CC1100_SRX);
		waiting = 1;
		
		start_time = time_now();
		increment = start_time + 50;

		// leave it in RX, just jump out. 
		status = RX_WAITING;
		return status;
	}
	else
	{
		// we're expecting a packet, so check if its done, or timed out
		// Wait for incoming packet
		if (packetReceived == FALSE)
		{
			if (time_now() > increment)							// check the status every few ms.
			{
				increment +=50;
				status = halRfStrobe(CC1100_SNOP);
				if (status != 0x1F)
				{												// the chip is no longer in RX mode, so leave and try again.					 
					status = RX_NO_PACKET;
					halRfStrobe(CC1100_SIDLE);					// turn off RX mode and flush fifo
					halRfStrobe(CC1100_SFRX);
					waiting = 0;								// reset state machine
					return (status);
				}
			}
			
			// 2017/11/01 Debug
			if (time_now() > (start_time + timeout))
			{													// timed out, so start over
				status = RX_NO_PACKET;
				halRfStrobe(CC1100_SIDLE);						// turn off RX mode and flush fifo
				halRfStrobe(CC1100_SFRX);
				waiting = 0;									// reset the state machine the next time.
				return (status);
			}

			status = RX_WAITING;
			return (status);
		}

		waiting = 0;

		// Read first element of packet from the RX FIFO
		if (status != RX_NO_PACKET)
		{
			status = halRfReadFifo(length, 1);

//			// Debug 11/10/2017
//			uint8_t length_Status, prev_length_Status;
//			length_Status = halRfReadStatusReg(CC1100_RXBYTES);
//			do {
//				prev_length_Status = length_Status;
//				length_Status = halRfReadStatusReg(CC1100_RXBYTES);
//				} while (length_Status<2 && length_Status!= prev_length_Status);

			if ((status & CC1100_STATUS_STATE_BM) == CC1100_STATE_RX_OVERFLOW)
			{
				halRfStrobe(CC1100_SIDLE);
				halRfStrobe(CC1100_SFRX);
				status = RX_FIFO_OVERFLOW;
			}
			else if (*length == 0 || *length > 61)
			{
				halRfStrobe(CC1100_SIDLE);
				halRfStrobe(CC1100_SFRX);
				status = RX_LENGTH_VIOLATION;
			}
			else
			{
				// update status to reflect that we've seen a good RF packet
//			////////////////////Reader_Status.Com_status |= RF_Coms;

				// Get payload
				halRfReadFifo(data, *length);

				// Get the packet status bytes [RSSI, LQI]
				halRfReadFifo(packet_status, 2);

				// Check CRC
				if ((packet_status[1] & CC1100_LQI_CRC_OK_BM) != CC1100_LQI_CRC_OK_BM)
				{
					status = RX_CRC_MISMATCH;
				}
				else
				{
					RSSI_signed = packet_status[0];
					RSSI_holder = RSSI_signed;

					if (RSSI_holder >= highest_RSSI)
					{
						*RSSI = 255;
					}
					else if (RSSI_holder <= lowest_RSSI)
					{
						*RSSI = 0;
					}
					else
					{
						RSSI_holder = RSSI_holder * 2 + 64;
						*RSSI = RSSI_holder;
					}
//				*RSSI = *RSSI/2;
//				*RSSI = *RSSI-74;
					status = RX_OK;

					halRfStrobe(CC1100_SIDLE);
					halRfStrobe(CC1100_SFRX);

				}
			}
		}
		else
		{
			halRfStrobe(CC1100_SIDLE);
			halRfStrobe(CC1100_SFRX);
		}
	}
	return (status);
}
#endif
