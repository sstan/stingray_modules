/*
 * motor_pwm.c
 *
 *  Created on: Jan 23, 2015
 *      Author: sstan
 */

#include <avr/io.h>

#include "motor_pwm.h"
#include "tc_tools.h"

// Lowest allowed pulse width : 550  us
// Highest allowed pulse width: 2400 us
// Pulse width for speed = 0  : 1320 us

#define MAX_PULSE_WIDTH_CYCLES ((uint16_t) 4800)
#define MIN_PULSE_WIDTH_CYCLES ((uint16_t) 1100)

#define PWM_PERIOD_CYCLES ((uint16_t) 40000)

struct motor_registers_s {
	volatile uint16_t*	OCR_ptr;
	volatile uint8_t*	DDR_ptr;
	volatile uint8_t*	PORT_ptr;
	uint8_t				pin;
	volatile uint8_t*	TCCR_A_ptr;
	uint8_t				channel;
};

const struct motor_registers_s motors[3] =
{
	{	/* MOTOR_PWM_LEFT
		 * PB5 ( OC1A/PCINT5 )	Digital pin 11 (PWM)
		 *
		 */
		OCR_ptr:	&OCR1A,
		DDR_ptr:	&DDRB,
		PORT_ptr:	&PORTB,
		pin:		5,
		TCCR_A_ptr:	&TCCR1A,
		channel:	TC_CHANNEL_A
	},
	{	/* MOTOR_PWM_RIGHT
	 	 * Digital pin 12 (PWM)
	 	 *
	 	 */
		OCR_ptr:	&OCR1B,
		DDR_ptr:	&DDRB,
		PORT_ptr:	&PORTB,
		pin:		6,
		TCCR_A_ptr:	&TCCR1A,
		channel:	TC_CHANNEL_B
	},
	{	/* MOTOR_PWM_CENTER
		 * Digital pin 13 (PWM)
		 *
		 */
		OCR_ptr:	&OCR1C,
		DDR_ptr:	&DDRB,
		PORT_ptr:	&PORTB,
		pin:		7,
		TCCR_A_ptr:	&TCCR1A,
		channel:	TC_CHANNEL_C
	}
};

/* MODULE ENTRY POINTS (PUBLIC FUNCTIONS) */

/*
 * Configure the TC module 1.
 * configuration that is common to one Timer/Counter module.
 */
void motor_pwm_tc_init(void)
{
	/* set Waveform Generation Mode */

	tc_set_wgm(&TCCR1A, &TCCR1B, 0b00001110);


	/* output disconnected from the pins
	 * Compare Output Mode: 0b00
	 */

	tc_set_com(&TCCR1A, TC_CHANNEL_A, 0b00);
	tc_set_com(&TCCR1A, TC_CHANNEL_B, 0b00);
	tc_set_com(&TCCR1A, TC_CHANNEL_C, 0b00);


	/* set Timer Counter prescaler */

	tc_set_prescaler(&TCCR1B, 8);


	/* This value determines the rate at which the pulses are sent. */
	/* Store the value in the Input Capture Register */
	ICR1 = PWM_PERIOD_CYCLES;
}


void motor_pwm_start(int arg, uint16_t pulse_width_cycles)
{
	/* initialize the Data Direction Register */

	tc_init_ddr(motors[arg].DDR_ptr,
			    motors[arg].PORT_ptr,
			    motors[arg].pin,
			    0,			/* output 0 (0V) */
			    1);			/* OUTPUT */

	/* set the Compare Output Mode to non-inverted */

	tc_set_com(motors[arg].TCCR_A_ptr,
			   motors[arg].channel,
			   (uint8_t) 0b00000010); /* non-inverting mode 0b10 */

	/* set the pulse width's initial value */

	motor_pwm_pulse_width_set(arg, pulse_width_cycles);
}

void motor_pwm_stop(int arg)
{
	/* disconnect output pins from the Timer/Counter module */
	tc_set_com(motors[arg].TCCR_A_ptr,
			   motors[arg].channel,
			   0b00);

	/* set them to output 0V */
	tc_init_ddr(motors[arg].DDR_ptr,
			    motors[arg].PORT_ptr,
				motors[arg].pin, 0, 1);
}

/* Sets the pulse width in cycles by changing the value of the Output Compare Register */
void motor_pwm_pulse_width_set(int arg, uint16_t pulse_width_cycles)
{
	if (pulse_width_cycles <= MAX_PULSE_WIDTH_CYCLES &&
		pulse_width_cycles >= MIN_PULSE_WIDTH_CYCLES)
	{
		*(motors[arg].OCR_ptr) = pulse_width_cycles;
	}
}

uint16_t motor_pwm_pulse_width_get(int arg)
{
	return *(motors[arg].OCR_ptr);
}
