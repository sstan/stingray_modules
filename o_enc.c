/*
 * o_enc.c
 *
 *  Created on: Jan 28, 2015
 *      Author:
 */

#include <stdint.h>

#include <avr/io.h>

#include "o_enc.h"
#include "tc_tools.h"

#define TIFR_ICF_MASK ((uint8_t) 0b00100000)

static inline uint16_t difference_16bits(uint16_t a, uint16_t b);

struct encoder_registers_s {
	volatile uint8_t*	TIFR_ptr;
	volatile uint16_t*	TCNT_ptr;
	volatile uint8_t*	PORT_ptr;
	volatile uint8_t*	DDR_ptr;
	volatile uint16_t*	ICR_ptr;
	uint8_t				pin;
	volatile uint8_t*	TCCR_A_ptr;
	volatile uint8_t*	TCCR_B_ptr;
	int					prescale;
};

static struct encoder_state enc_state[2];
static struct encoder_state enc_state_shadow[2];

static const struct encoder_registers_s encoders[2] =
{
	{	/* OPTICAL_ENCODER_LEFT */
		TIFR_ptr:	&TIFR4,
		TCNT_ptr:	&TCNT4,
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		ICR_ptr:	&ICR4,
		pin:		0,			/* Port L 0 (pin PL0) Digital pin 49 */
		TCCR_A_ptr:	&TCCR4A,
		TCCR_B_ptr:	&TCCR4B,
		prescale:	256
	},
	{
		TIFR_ptr:	&TIFR5,
		TCNT_ptr:	&TCNT5,
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		ICR_ptr:	&ICR5,
		pin:		1,			/* Port L 1 (pin PL1) */
		TCCR_A_ptr:	&TCCR5A,
		TCCR_B_ptr:	&TCCR5B,
		prescale:	256
	}
};

struct encoder_state* enc_init(int arg)
{
	/* Waveform Generation Mode normal for TC modules */

	tc_set_wgm(encoders[arg].TCCR_A_ptr, 	/* TC Configuration Register A */
			   encoders[arg].TCCR_B_ptr, 	/* TC Configuration Register B */
			   0);							/* WGM mode: 0 (normal). */

	/* Set the Timer/Counter frequency divider */

	tc_set_prescaler(encoders[arg].TCCR_B_ptr,
			         encoders[arg].prescale);


	/* The Input Capture Pins are INPUTS. This call will
	 * set the Data Direction Registers accordingly.
	 */

	tc_init_ddr(encoders[arg].DDR_ptr,
			    encoders[arg].PORT_ptr,
				encoders[arg].pin,
				0,		/* no pull-up resistor */
				0);		/* INPUT */

	/* initialize encoder structure */

	enc_state[arg].counter		= 0;
	enc_state[arg].encoder_id	= arg;
	enc_state[arg].last_icr_val	= 0;
	enc_state[arg].period		= 0;
	enc_state[arg].state		= OENC_STATE_STOPPED;

	enc_state_shadow[arg] = enc_state[arg];

	/* return a pointer to the shadow state structure */

	return &enc_state_shadow[arg];
}

struct encoder_state* enc_get_handle(int arg)
{
	return &enc_state_shadow[arg];
}

static inline void enc_toggle_trigger_edge(int arg)
{
	if ((*encoders[arg].TCCR_B_ptr & (1 << 6)) == 0)
	{
		/* A falling edge has been used as trigger. Now use a rising edge. */
		*encoders[arg].TCCR_B_ptr |= 1 << 6;
	}
	else
	{
		/* A rising edge has been used as trigger. Now use a falling edge. */
		*encoders[arg].TCCR_B_ptr &= ~(1 << 6);
	}
}

void enc_compute_tr_period(int arg)
{
	/* VARIABLES */

	/* Holds the value such as the new value is overwritten
	 * immediately in the state variable structure.
	 */

	uint16_t last_icr_val_copy;

	uint16_t difference; /* temporary variable */

	/* ALGORITHM */

	if (((*encoders[arg].TIFR_ptr) & TIFR_ICF_MASK) != 0)
	{
		enc_state[arg].counter++;

		enc_toggle_trigger_edge(arg);

		/* The flag is cleared by writing a logical 1. */

		*encoders[arg].TIFR_ptr |= TIFR_ICF_MASK;

		last_icr_val_copy = enc_state[arg].last_icr_val;

		enc_state[arg].last_icr_val = *encoders[arg].ICR_ptr;


		if (enc_state[arg].state == OENC_STATE_NEED_ONE_MORE)
		{
			enc_state[arg].state = OENC_STATE_VALID;
		}
		else if (enc_state[arg].state == OENC_STATE_STOPPED)
		{
			enc_state[arg].state = OENC_STATE_NEED_ONE_MORE;

			goto algo_end;
		}

		/* compute period */

		enc_state[arg].period = difference_16bits(enc_state[arg].last_icr_val,
				                                  last_icr_val_copy);

	}
	else if (enc_state[arg].state != OENC_STATE_STOPPED)
	{
		/* compute difference: compare TCNT to last captured ICR */

		difference = difference_16bits(*encoders[arg].TCNT_ptr,
				                       enc_state[arg].last_icr_val);

		/* If no signal transition coming from the optical encoder
		 * was detected for an arbitrarily long time, we assume
		 * the wheel is not moving.
		 */

		if (difference > OENC_SAMPLE_EXPIRES_CYCLES)
		{
			enc_state[arg].state	= OENC_STATE_STOPPED;
			enc_state[arg].period	= 0;
		}
	}
	else
	{
		return; /* ALGORITHM EXIT POINT */
	}

algo_end: /* ALGORITHM EXIT POINT */

	/* update shadow encoder state variables */

	enc_state_shadow[arg] = enc_state[arg];

	return;


}

void enc_clear_counter(int arg)
{
	enc_state_shadow[arg].counter = 0;
}

/* difference_16bits
 *
 * what we assume to be the bigger value is the first parameter.
 * The second parameter is assumed to be the smaller value.
 *
 * Assumption: the overflow case is taken into account.
 */
static inline uint16_t difference_16bits(uint16_t a, uint16_t b)
{
	if (b > a)
	{
		return (uint16_t) 0xffff - (b - a);
	}
	return a - b;
}
