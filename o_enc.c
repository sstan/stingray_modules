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

static inline void increment_tov_cntr(uint8_t enc_id);
static inline void handle_transition(uint8_t enc_id);
static inline void enc_toggle_trigger_edge(uint8_t enc_id);

struct encoder_config_t {
	volatile uint8_t*			PORT_ptr;
	volatile uint8_t*			DDR_ptr;
	uint8_t						pin;
	struct tc_module_registers*	tc_p;
};

/* Initialization of the array of structures that holds all the registers
 * that are needed to use the optical encoders.
 *
 */
static const struct encoder_config_t encoders[ENC_NUMBER_OF_ENCODERS] =
{
	{	/* ENC_ID_ENCODER_LEFT */
		/* Port L (PL0) Digital pin 49 ( ICP4 )  */
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		pin:		0,
		tc_p:		&tc_module_s[TC_MODULE_4]
	},
	{   /* ENC_ID_ENCODER_RIGHT */
		/* Port L (PL1) Digital pin 48 ( ICP5 ) */
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		pin:		1,
		tc_p:		&tc_module_s[TC_MODULE_5]
	}
};

/* A 32-bit variable that keeps track of the TCNT overflows.
 * An overflow happens when TCNT is equal to TOP.
 */
static uint32_t tov_cntr[ENC_NUMBER_OF_ENCODERS];
static uint16_t last_icr[ENC_NUMBER_OF_ENCODERS];
static uint32_t incr_ctr[ENC_NUMBER_OF_ENCODERS];
static uint32_t tov_cntr_cumulative[ENC_NUMBER_OF_ENCODERS];

/* MODULE ENTRY POINTS */

void enc_init(uint8_t enc_id)
{
	enc_reset(enc_id);

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

int enc_read(uint8_t enc_id, uint32_t tbl[])
{
	return cbuf_read(enc_id, tbl);
}

int enc_new_data(uint8_t enc_id)
{
	return cbuf_get_size(enc_id);
}

uint32_t enc_get_ticks(uint8_t enc_id)
{
	uint16_t tcnt;
	uint16_t cpo;
	uint32_t ticks;

	tcnt = *encoders[enc_id].tc_p->TCNT_ptr;
	cpo  =  encoders[enc_id].tc_p->TOP_value + 1;

	ticks = cpo*tov_cntr_cumulative[enc_id] + tcnt;

	return ticks;
}

uint32_t enc_get_count(uint8_t enc_id)
{
	return incr_ctr[enc_id];
}

void enc_reset(uint8_t enc_id)
{
	cbuf_reset(enc_id);

	tov_cntr_cumulative[enc_id] = 0;
	incr_ctr[enc_id] = 0;
	last_icr[enc_id] = 0;
	tov_cntr[enc_id] = 0;

}

/* INTERNAL FUNCTIONS */

static inline void increment_tov_cntr(uint8_t enc_id)
{
	tov_cntr[enc_id]++;
	tov_cntr_cumulative[enc_id]++;
}

static inline void handle_transition(uint8_t enc_id)
{
	uint32_t delta;
	uint16_t cpo;
	uint16_t icr;
	uint32_t tov;

	enc_toggle_trigger_edge(enc_id);

	/* clear the Input Capture Flag */

	*encoders[enc_id].tc_p->TIFR_ptr |= (1 << 5);

	cpo =  encoders[enc_id].tc_p->TOP_value + 1;
	icr = *encoders[enc_id].tc_p->ICR_ptr;
	tov =  tov_cntr[enc_id];

	tov_cntr[enc_id] = 0;

	if (tov == 0)
	{
		delta = icr - last_icr[enc_id];
	}
	else
	{
		delta = icr + tov*cpo - last_icr[enc_id];
	}

	cbuf_write(enc_id, delta);

	last_icr[enc_id] = icr;
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
	incr_ctr[ENC_ID_ENCODER_LEFT]++;
	handle_transition(ENC_ID_ENCODER_LEFT);
}

ISR(TIMER5_CAPT_vect)
{
	incr_ctr[ENC_ID_ENCODER_RIGHT]++;
	handle_transition(ENC_ID_ENCODER_RIGHT);
}
