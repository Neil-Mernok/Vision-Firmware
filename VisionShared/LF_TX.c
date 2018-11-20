/*
 * LF_TX.c
 *
 *  Created on: Sep 9, 2014
 *      Author: KobusGoosen
 */

#include "LF_TX.h"

#define GENPOLY 0x0017 /* x^4 + x^2 + x + 1 */

#define DEFAULT_BSID            (uint16_t)0xABCD
#define DEFAULT_SNR             (uint32_t)0x00BEEF
#define DELTA_T_COUNT                   1

#define LF_TX_Capable
#ifdef LF_TX_Capable
extern void PWM_reset(void);

void send_carrier(void)
{
	PWM_pins_change(true);
}

void pause_carrier(void)
{
	PWM_pins_change(false);
}

static uint16_t getbitint(uint64_t b, int16_t n)
{
	return (b >> (n)) & 1;
}

/*
 * Takes b as input, which should be the information vector
 * already multiplied by x^4 (ie. shifted over 4 bits), and
 * returns the crc for this input based on the defined generator
 * polynomial GENPOLY
 */
uint16_t makeCrc4(uint64_t b)
{
	int i;
	
	i = 63;
	while (b >= 16 && i>3) /* >= 2^4, so degree(b) >= degree(genpoly) */
	{
		if (getbitint(b, i) == 1)
			b ^= ((uint64_t)GENPOLY) << (i-4); /* reduce with GENPOLY */
		i--;
	}
	return b;
}

int8_t makeCrc4Check(uint64_t theData, uint16_t theCrc4)
{
	uint16_t myCrc4 = makeCrc4(theData);
	return (theCrc4 == myCrc4);
}

/**
 * @brief: this function gets called periodically, typically from an interrupt or\
 * 			systic, roughly every millisecond
 */
#define carrier_burst_periods	4
int LF_send_bits(void)
{
	static int carrier_count = 0, bitcount;
	uint64_t bit_window = 0;

	if (LF_Params.state == 1)
	{
		send_carrier();
		if (++carrier_count >= carrier_burst_periods-1)
		{
			carrier_count = 0;
			LF_Params.state = 2;
			bitcount = LF_Params.bitcount;
		}
	}
	else if (LF_Params.state == 2)
	{
		if (bitcount)
		{
			bit_window = (uint64_t) 0x01 << (bitcount - 1);
			if (bit_window & LF_Params.data)			// this bit is a one, so send '10'
			{
				if (LF_Params.manch == 0)			// first bit of symbol
					send_carrier();
				else
					// second bit of symbol
					pause_carrier();
			}
			else									// this bit is a zero, so send '01'
			{
				if (LF_Params.manch == 0)			// first bit of symbol
					pause_carrier();
				else
					// second bit of symbol
					send_carrier();
			}
			if (LF_Params.manch)
				bitcount--;
			LF_Params.manch = !LF_Params.manch;
		}
		else
		{
			pause_carrier();
			LF_Params.state = 0;
			LF_Params.manch = false;
		}
		//if(bitcount%7 == 0) PWM_reset();
	}
	return LF_Params.state;
}

#define pattern 				0x96

void LF_send(void)
{
	if (LF_Params.data_ready)
		LF_Params.state = 1;					// tell the ticker to start the LF transmission
}

void LF_form_packet(uint16_t vehicle_ID, uint8_t slave_ID, bool crc)
{
	uint8_t crc_val = 0;
	LF_Params.manch = 0;

	LF_Params.data = 0b1111;				//four 1's to create sync word.
	LF_Params.bitcount = 4;

	LF_Params.data <<= 8;					// shift up to make space for pattern
	LF_Params.bitcount += 8;
	LF_Params.data |= (uint64_t) pattern;	// place pattern

	LF_Params.data <<= 16;					// shift up to make space for vehicle_ID
	LF_Params.bitcount += 16;
	LF_Params.data |= (uint16_t)vehicle_ID;			// place vehicle ID

	LF_Params.data <<= 4;					// shift up to make space for slave_ID
	LF_Params.bitcount += 4;
	LF_Params.data |= (slave_ID & 0x0F);	// place data

	if (crc)
	{
		crc_val = (makeCrc4(LF_Params.data & 0xFFFFF) & 0x0F);// calc 4-bit CRC
		LF_Params.data <<= 4;						// shift up to make space for crc
		LF_Params.bitcount += 4;
		LF_Params.data |= crc_val;
	}

	LF_Params.data_ready = true;
//	LF_Params.state = 1;					// tell the ticker to start the LF transmission
}
#endif
