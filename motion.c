/*
 * motor_pwm.c
 *
 *  Created on: Jan 23, 2015
 *      Author: sstan
 */

#include <avr/io.h>

#include "motion.h"

// Module Defines
//Servo motors
#define MAX_PULSE_WIDTH_TICKS     ((uint16_t) 4800)
#define MIN_PULSE_WIDTH_TICKS     ((uint16_t) 1100)
#define INITIAL_PULSE_WIDTH_TICKS ((uint16_t) 2640)
#define NUMBER_OF_SERVOS          3
// Encoders
#define ENC_NUMBER_OF_ENCODERS 	2
// Timer Defines
/* Timer/Counter Channels */
#define TC_CHANNEL_A 0
#define TC_CHANNEL_B 1
#define TC_CHANNEL_C 2
#define TC_MODULE_4				0
#define TC_MODULE_5				1

struct motor_registers_s
{
	volatile uint16_t*          OCR_ptr;
	volatile uint8_t*           DDR_ptr;
	volatile uint8_t*           PORT_ptr;
	uint8_t                     pin;
	uint8_t                     channel;
	struct tc_module_registers* tc_p;
};

const struct motor_registers_s motors[NUMBER_OF_SERVOS] =
{
	{	/* MOTOR_PWM_LEFT
		 * PL4 ( OC5B )	Digital pin 45 (PWM)
		 *
		 */
		OCR_ptr:	&OCR5B,
		DDR_ptr:	&DDRL,
		PORT_ptr:	&PORTL,
		pin:		4,
		channel:	TC_CHANNEL_B,
		tc_p:		&tc_module_s[TC_MODULE_5]
	},
	{	/* MOTOR_PWM_RIGHT
	 	 * PH5 ( OC4C )	Digital pin 8 (PWM)
	 	 *
	 	 */
		OCR_ptr:	&OCR4C,
		DDR_ptr:	&DDRH,
		PORT_ptr:	&PORTH,
		pin:		5,
		channel:	TC_CHANNEL_C,
		tc_p:		&tc_module_s[TC_MODULE_4]
	},
	{	/* MOTOR_PWM_CENTER
		 *
		 * PH4 ( OC4B )	Digital pin 7 (PWM)
		 */
		OCR_ptr:	&OCR4B,
		DDR_ptr:	&DDRH,
		PORT_ptr:	&PORTH,
		pin:		4,
		channel:	TC_CHANNEL_B,
		tc_p:		&tc_module_s[TC_MODULE_4]
	}
};

/* MODULE ENTRY POINTS (PUBLIC FUNCTIONS) */

void servo_start(int arg)
{
	/* OCR must contain a valid value before the servo is started. */

	servo_pulse_width_set(arg, servo_pulse_width_get(arg));

	/* Initialize the Data Direction Register of the OCnX pin. */

	tc_init_ddr(motors[arg].DDR_ptr,
			    motors[arg].PORT_ptr,
			    motors[arg].pin,
			    0,			/* output 0 (0V) */
			    1);			/* OUTPUT */

	/* Set the Compare Output Mode to non-inverted. */

	tc_set_com(motors[arg].tc_p->TCCR_A_ptr,
			   motors[arg].channel,
			   (uint8_t) 0b00000010); /* non-inverting mode 0b10 */
}

void servo_stop(int arg)
{
	/* Disconnect the output pin from the Timer/Counter module. */

	tc_set_com(motors[arg].tc_p->TCCR_A_ptr,
			   motors[arg].channel,
			   0b00);

	/* Set the pin as an output of 0. */

	tc_init_ddr(motors[arg].DDR_ptr,
			    motors[arg].PORT_ptr,
				motors[arg].pin, 0, 1);
}

/* Sets the pulse width in cycles by changing the
 * value of the Output Compare Register
 */
void servo_pulse_width_set(int arg, uint16_t pulse_width_cycles)
{
	if (pulse_width_cycles <= MAX_PULSE_WIDTH_TICKS &&
		pulse_width_cycles >= MIN_PULSE_WIDTH_TICKS)
	{
		*(motors[arg].OCR_ptr) = pulse_width_cycles;
	}
}

uint16_t servo_pulse_width_get(int arg)
{
	return *(motors[arg].OCR_ptr);
}

//-------------------------------------------------------------------------
//Encoder Functions
//-------------------------------------------------------------------------

// Entry points
#define TIFR_ICF_MASK ((uint8_t) 0b00100000)

static inline void increment_tov_cntr(uint8_t enc_id);
static inline void handle_transition(uint8_t enc_id);
static inline void enc_toggle_trigger_edge(uint8_t enc_id);

struct encoder_config_t
{
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

	incr_ctr[enc_id]++;

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
	increment_tov_cntr(MOTION_WHEEL_LEFT);
}

ISR(TIMER5_OVF_vect)
{
	increment_tov_cntr(MOTION_WHEEL_RIGHT);
}

ISR(TIMER4_CAPT_vect)
{
	handle_transition(MOTION_WHEEL_LEFT);
}

ISR(TIMER5_CAPT_vect)
{
	handle_transition(MOTION_WHEEL_RIGHT);
}


// Local functions




//-------------------------------------------------------------------------
// Local Timer Functions
//-------------------------------------------------------------------------

/* The structure that stores the configuration of the TC modules as well as
 * the pointers to the configuration registers belonging to the TC modules.
 */
/* Number of 500-nanosecond cycles in 20 milliseconds. */
#define TC_20_MS_PERIOD_AT_PS_8 (uint16_t) 40000

struct tc_module_registers {
	volatile uint8_t*	TIFR_ptr;
	volatile uint8_t*	TIMSK_ptr;
	volatile uint16_t*	TCNT_ptr;
	volatile uint16_t*	ICR_ptr;
	volatile uint8_t*	TCCR_A_ptr;
	volatile uint8_t*	TCCR_B_ptr;
	uint8_t				prescale;
	uint8_t				wgm_mode;
	volatile uint16_t*	TOP_value_p;
	uint16_t			TOP_value;
};

/* Using the Waveform Generation Mode 15 (OCRA holds the TOP value). */
const struct tc_module_registers tc_module_s[] =
{
	{	/* TC_MODULE_4 */
		TIFR_ptr:	&TIFR4,
		TCNT_ptr:	&TCNT4,
		ICR_ptr:	&ICR4,
		TOP_value_p:&OCR4A,
		TCCR_A_ptr:	&TCCR4A,
		TCCR_B_ptr:	&TCCR4B,
		prescale:	8,
		wgm_mode:	(uint8_t) 15,
		TOP_value:  TC_20_MS_PERIOD_AT_PS_8 - 1,
		TIMSK_ptr:   &TIMSK4,
	},
	{	/* TC_MODULE_5 */
		TIFR_ptr:	&TIFR5,
		TCNT_ptr:	&TCNT5,
		ICR_ptr:	&ICR5,
		TOP_value_p:&OCR5A,
		TCCR_A_ptr:	&TCCR5A,
		TCCR_B_ptr:	&TCCR5B,
		prescale:	8,
		wgm_mode:	(uint8_t) 15,
		TOP_value:  TC_20_MS_PERIOD_AT_PS_8 - 1,
		TIMSK_ptr:   &TIMSK5,
	}
};


