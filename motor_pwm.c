/*
 * motor_pwm.c
 *
 *  Created on: Jan 23, 2015
 *      Author: sstan
 */

#include <avr/io.h>

#include "motor_pwm.h"
#include "tc_tools.h"

#define MAX_PULSE_WIDTH_TICKS     ((uint16_t) 4800)
#define MIN_PULSE_WIDTH_TICKS     ((uint16_t) 1100)
#define INITIAL_PULSE_WIDTH_TICKS ((uint16_t) 2640)
#define NUMBER_OF_SERVOS          3

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
