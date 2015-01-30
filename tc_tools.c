/*
 * tc_tools.c
 *
 *  Created on: Jan 28, 2015
 *      Author: Nco
 */

#include <stdint.h>

#include "tc_tools.h"


/* PUBLIC HELPER FUNCTIONS */

void tc_init_ddr(volatile uint8_t* DDR_ptr,
		         volatile uint8_t* PORT_ptr,
				 uint8_t pin,
				 volatile uint8_t port_value,
				 volatile uint8_t ddr_value)
{
	if (ddr_value == 1)
	{
		*DDR_ptr |= (1<<pin);	/* write '1' (OUTPUT) */
	}
	else
	{
		*DDR_ptr &= ~(1<<pin);	/* write '0' (INPUT) */
	}

	if (port_value == 0)
	{
		*PORT_ptr &= ~(1<<pin);	/* write '0' */
	}
	else
	{
		*PORT_ptr |= 1<<pin;	/* write '1' */
	}
}

/* set the Compare Output Mode for channel A, B, or C */
void tc_set_com(volatile uint8_t* TCCR_A_ptr,
		        uint8_t channel,
				uint8_t co_mode)
{
	uint8_t	tccra;
	uint8_t	co_mode_mask = 0b00000011;

	/* read the value of TCCRnA */

	tccra = *TCCR_A_ptr;

	switch(channel)
	{
	case TC_CHANNEL_A:
		co_mode = co_mode << 6;
		co_mode_mask = co_mode_mask << 6;
		break;

	case TC_CHANNEL_B:
		co_mode = co_mode << 4;
		co_mode_mask = co_mode_mask << 4;
		break;

	case TC_CHANNEL_C:
		co_mode = co_mode << 2;
		co_mode_mask = co_mode_mask << 2;
		break;

	default:
		return;
		break;
	}

	co_mode &= co_mode_mask;

	tccra |= co_mode;

	co_mode |= ~co_mode_mask;

	tccra &= co_mode;

	/* write the value to TCCRnB */

	*TCCR_A_ptr = tccra;
}

/* set the waveform generation mode */
void tc_set_wgm(volatile uint8_t* TCCR_A_ptr,
		        volatile uint8_t* TCCR_B_ptr,
				uint8_t wgm)
{
	uint8_t wgm_bit3_bit2 = wgm & 0b00001100;
	uint8_t wgm_bit1_bit0 = wgm & 0b00000011;

	wgm_bit3_bit2 = wgm_bit3_bit2 << 1;

	uint8_t tccra = *TCCR_A_ptr;
	uint8_t tccrb = *TCCR_B_ptr;

	/* set WGMn1 and WGMn0 in TCCRnA*/
	tccra |= wgm_bit1_bit0;
	tccra &= (0b11111100 | wgm_bit1_bit0);

	/* set WGMn3 and WGMn2 in TCCRnB */
	tccrb |= wgm_bit3_bit2;
	tccrb &= (0b11100111 | wgm_bit3_bit2);

	*TCCR_A_ptr = tccra;
	*TCCR_B_ptr = tccrb;
}

void tc_set_prescaler(volatile uint8_t* TCCR_B_ptr,
		              int prescaler)
{
	/* copy the value contained in the register */

	uint8_t tccrb = *TCCR_B_ptr;

	switch(prescaler)
	{
	case 1:		// 0b001
		tccrb |= 0b00000001;
		tccrb &= 0b11111001;
		break;

	case 8:		// 0b010

default_case: /* DEFAULT */

		tccrb |= 0b00000010;
		tccrb &= 0b11111010;
		break;

	case 64:	// 0b011
		tccrb |= 0b00000011;
		tccrb &= 0b11111011;
		break;

	case 256:	// 0b100
		tccrb |= 0b00000100;
		tccrb &= 0b11111100;
		break;

	case 1024:	// 0b101
		tccrb |= 0b00000101;
		tccrb &= 0b11111101;
		break;

	default:
		goto default_case; /* GOTO DEFAULT */
		break;
	}

	/* write the new value to the register */

	*TCCR_B_ptr = tccrb;
}
