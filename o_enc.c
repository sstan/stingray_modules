/*
 * o_enc.c
 *
 *  Created on: Jan 28, 2015
 *      Author: sstan
 */

#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "o_enc.h"
#include "tc_tools.h"
#include "cbuf.h"

#define TIFR_ICF_MASK ((uint8_t) 0b00100000)
#define BUF_ENTRIES 20

static inline void increment_tov_cntr(uint8_t enc_id);
static inline void handle_transition(uint8_t enc_id);
static inline void enc_toggle_trigger_edge(uint8_t enc_id);

struct encoder_registers_s {
	volatile uint8_t*			PORT_ptr;
	volatile uint8_t*			DDR_ptr;
	uint8_t						pin;
	struct tc_module_registers*	tc_p;
};

/* Initialization of the array of structures that holds all the registers
 * that are needed to use the optical encoders.
 *
 */
static const struct encoder_registers_s encoders[ENC_NUMBER_OF_ENCODERS] =
{
	{	/* ENC_ID_ENCODER_LEFT */
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		pin:		0,			/* Port L 0 (pin PL0) Digital pin 49  PL0 ( ICP4 ) */
		tc_p:		&tc_module_s[TC_MODULE_4]
	},
	{   /* ENC_ID_ENCODER_RIGHT */
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		pin:		1,			/* Port L 1 (pin PL1)  ( ICP5 )  Digital pin 48*/
		tc_p:		&tc_module_s[TC_MODULE_5]
	}
};

/* A 32-bit variable that keeps track of the TCNT overflows.
 * An overflow happens when TCNT is equal to TOP.
 */
static uint32_t tov_cntr[ENC_NUMBER_OF_ENCODERS];

/* MODULE ENTRY POINTS */

void enc_init(uint8_t enc_id)
{
	cbuf_reset(enc_id, 40000);

	/* The Input Capture Pins are INPUTS. This call will
	 * set the Data Direction Registers accordingly.
	 */

	tc_init_ddr(encoders[enc_id].DDR_ptr,
			    encoders[enc_id].PORT_ptr,
				encoders[enc_id].pin,
				0,		/* no pull-up resistor */
				0);		/* INPUT */

	/* Input Capture Interrupt Enable */

	*encoders[enc_id].tc_p->TIMSK_ptr  |= (1 << 5);

	/* Timer Overflow Interrupt Enable */

	*encoders[enc_id].tc_p->TIMSK_ptr  |= (1 << 0);
}

int enc_read_timestamp(uint8_t enc_id, uint32_t* timestamp)
{
	return cbuf_read_timestamp(enc_id, timestamp);
}

void enc_get_time(uint8_t enc_id, uint32_t* ts)
{
	/* TO DO: atomically */
	*ts =  *encoders[enc_id].tc_p->TCNT_ptr;
	*ts += (encoders[enc_id].tc_p->TOP_value + 1)*tov_cntr[enc_id];
}

uint16_t enc_get_count(uint8_t enc_id)
{
	return cbuf_count(enc_id);
}

void enc_reset(uint8_t enc_id)
{
	cbuf_reset(enc_id, 40000);
}

/* INTERNAL FUNCTIONS */

static inline void increment_tov_cntr(uint8_t enc_id)
{
	tov_cntr[enc_id]++;
}

static inline void handle_transition(uint8_t enc_id)
{
	cbuf_write(enc_id,
               *encoders[enc_id].tc_p->ICR_ptr,
               tov_cntr[enc_id]);

	enc_toggle_trigger_edge(enc_id);

	/* clear the Input Capture Flag */

	*encoders[enc_id].tc_p->TIFR_ptr |= (1 << 5);
}

static inline void enc_toggle_trigger_edge(uint8_t enc_id)
{
	if ((*encoders[enc_id].tc_p->TCCR_B_ptr & (1 << 6)) == 0)
	{
		/* A falling edge has been used as trigger. Now use a rising edge. */
		*encoders[enc_id].tc_p->TCCR_B_ptr |= 1 << 6;
	}
	else
	{
		/* A rising edge has been used as trigger. Now use a falling edge. */
		*encoders[enc_id].tc_p->TCCR_B_ptr &= ~(1 << 6);
	}
}

/* INTERRUPT SERVICE ROUTINES */

ISR(TIMER4_OVF_vect)
{
	increment_tov_cntr(ENC_ID_ENCODER_LEFT);
}

ISR(TIMER5_OVF_vect)
{
	increment_tov_cntr(ENC_ID_ENCODER_RIGHT);
}

ISR(TIMER4_CAPT_vect)
{
	handle_transition(ENC_ID_ENCODER_LEFT);
}

ISR(TIMER5_CAPT_vect)
{
	handle_transition(ENC_ID_ENCODER_RIGHT);
}
