/*
 * motor_pwm.c
 *
 *  Created on: Feb 25, 2015
 *      Author: sstan
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "motion.h"


/***************************************************
 *                Forward Declarations             *
 ***************************************************/

static inline void enc_init(int deviceId);

inline static void tc_init(int tcModule);

inline static void tc_init_ddr(volatile uint8_t* DDR_ptr,
		                       volatile uint8_t* PORT_ptr,
				               uint8_t pin,
				               volatile uint8_t port_value,
				               volatile uint8_t ddr_value);

inline static void tc_set_com(volatile uint8_t* TCCR_A_ptr,
		                      uint8_t channel,
				              uint8_t co_mode);

inline static void tc_set_wgm(volatile uint8_t* TCCR_A_ptr,
		                      volatile uint8_t* TCCR_B_ptr,
				              uint8_t wgm);

inline static void tc_set_prescaler(volatile uint8_t* TCCR_B_ptr,
		                            int prescaler);

static inline void increment_tov_cntr(int enc_id);

static inline void handle_transition(int enc_id);

static inline void enc_toggle_trigger_edge(int enc_id);

/***************************************************
 *                      DEFINES                    *
 ***************************************************/

/* Servomotor */

#define MAX_PULSE_WIDTH_TICKS       ((uint16_t) 4800)
#define MIN_PULSE_WIDTH_TICKS       ((uint16_t) 1100)
#define INITIAL_PULSE_WIDTH_TICKS   ((uint16_t) 2640)
#define NUMBER_OF_SERVOS            3

/* Encoders */

#define ENC_NUMBER_OF_ENCODERS      2

#define TIFR_ICF_MASK ((uint8_t) 0b00100000)

/* Timer/Counter */

#define TC_MODULE_4                 0
#define TC_MODULE_5                 1

#define TC_CHANNEL_A                0
#define TC_CHANNEL_B                1
#define TC_CHANNEL_C                2

#define TC_OCM_NON_INVERTED         ((uint8_t) 2)

/* Number of 500-nanosecond cycles in 20 milliseconds. */

#define TC_20_MS_PERIOD_AT_PS_8 (uint16_t) 40000


/***************************************************
 *             Configuration Structures            *
 ***************************************************/

/* The structure that stores the configuration of the TC modules as well as
 * the pointers to the configuration registers belonging to the TC modules.
 */

struct tc_module_registers
{
	volatile uint8_t*	TIFR_ptr;
	volatile uint8_t*	TIMSK_ptr;
	volatile uint16_t*	TCNT_ptr;
	volatile uint16_t*	ICR_ptr;
	volatile uint8_t*	TCCR_A_ptr;
	volatile uint8_t*	TCCR_B_ptr;
	uint8_t				prescale;
	uint8_t				wgm_mode;
	volatile uint16_t*	TOP_value_p;
};

struct encoder_config_t
{
	volatile uint8_t*			PORT_ptr;
	volatile uint8_t*			DDR_ptr;
	uint8_t						pin;
	struct tc_module_registers*	tc_p;
};

struct motor_registers_s
{
	volatile uint16_t*          OCR_ptr;
	volatile uint8_t*           DDR_ptr;
	volatile uint8_t*           PORT_ptr;
	uint8_t                     pin;
	uint8_t                     channel;
	struct tc_module_registers* tc_p;
};

/***************************************************
 *                       Globals                   *
 ***************************************************/

/* A 32-bit variable that keeps track of the TCNT overflows.
 * An overflow happens when TCNT is equal to TOP.
 */
static uint32_t tov_cntr[ENC_NUMBER_OF_ENCODERS];
static uint16_t last_icr[ENC_NUMBER_OF_ENCODERS];

static uint32_t last_timestamp     [ENC_NUMBER_OF_ENCODERS];
static uint32_t tov_cntr_cumulative[ENC_NUMBER_OF_ENCODERS];
static uint32_t new_data           [ENC_NUMBER_OF_ENCODERS];
static int      new_data_available [ENC_NUMBER_OF_ENCODERS];

static const struct tc_module_registers tc_module_s[] =
{
    /* Using the Waveform Generation Mode 15 (OCRA holds the TOP value). */
	{	/* TC_MODULE_4 */
		TIFR_ptr:	&TIFR4,
		TCNT_ptr:	&TCNT4,
		ICR_ptr:	&ICR4,
		TOP_value_p:&OCR4A,
		TCCR_A_ptr:	&TCCR4A,
		TCCR_B_ptr:	&TCCR4B,
		prescale:	8,
		wgm_mode:	(uint8_t) 15,
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
		TIMSK_ptr:  &TIMSK5,
	}
};

/* Initialization of the array of structures that holds all the registers
 * that are needed to use the optical encoders.
 */

static const struct encoder_config_t encoders[] =
{
	{	/* Left encoder */
		/* Port L (PL0) Digital pin 49 ( ICP4 )  */
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		pin:		0,
		tc_p:		&tc_module_s[TC_MODULE_4]
	},
	{   /* Right encoder */
		/* Port L (PL1) Digital pin 48 ( ICP5 ) */
		PORT_ptr:	&PORTL,
		DDR_ptr:	&DDRL,
		pin:		1,
		tc_p:		&tc_module_s[TC_MODULE_5]
	}
};

static const struct motor_registers_s motors[] =
{
	{	/* Left servo
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
	{	/* Right servo
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
	{	/* Center servo
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


/**************************************************
 *            Motion module entry points          *
 **************************************************/

void motion_init(void)
{
	tc_init(TC_MODULE_4);
	tc_init(TC_MODULE_5);
	enc_init(MOTION_WHEEL_LEFT);
	enc_init(MOTION_WHEEL_RIGHT);
}

void motion_servo_start(int arg)
{
	uint16_t pulse_width_cycles = *(motors[arg].OCR_ptr);

    /* This call will ensure that the Output Compare Register
     * contains a valid value before the servo is started.
     */

	if (!(pulse_width_cycles <= MAX_PULSE_WIDTH_TICKS &&
		  pulse_width_cycles >= MIN_PULSE_WIDTH_TICKS))
	{
		*(motors[arg].OCR_ptr) = INITIAL_PULSE_WIDTH_TICKS;
	}

    /* The OCnX pin needs to be an output.
     * Initialize the Data Direction Register of the OCnX pin.
     */

	tc_init_ddr(motors[arg].DDR_ptr,
			    motors[arg].PORT_ptr,
			    motors[arg].pin,
			    0,			/* output 0 (0V) */
			    1);			/* OUTPUT */

	/* Set the Compare Output Mode to non-inverted. */

	tc_set_com(motors[arg].tc_p->TCCR_A_ptr,
			   motors[arg].channel,
               TC_OCM_NON_INVERTED); /* non-inverting mode */
}

void motion_servo_stop(int arg)
{
    /* Set the pin as an output of 0V. */

    tc_init_ddr(motors[arg].DDR_ptr,
                motors[arg].PORT_ptr,
                motors[arg].pin, 0, 1);

	/* Disconnect the output pin from the Timer/Counter module. */

	tc_set_com(motors[arg].tc_p->TCCR_A_ptr,
			   motors[arg].channel,
			   0b00);
}

/* Sets the pulse width by changing the
 * value of the Output Compare Register
 */
void motion_servo_set_pulse_width(int arg, uint16_t pulse_width_cycles)
{
	if (pulse_width_cycles <= MAX_PULSE_WIDTH_TICKS &&
		pulse_width_cycles >= MIN_PULSE_WIDTH_TICKS)
	{
		*(motors[arg].OCR_ptr) = pulse_width_cycles;
	}
}

/* Returns the pulse width by reading the
 * value of the Output Compare Register
 */
uint16_t motion_servo_get_pulse_width(int arg)
{
	return *(motors[arg].OCR_ptr);
}

int motion_enc_read(int arg, uint32_t *val)
{
	int retVal = 0;

	/* If new data is available */

	cli();

	if (new_data_available[arg] == 1)
	{
		*val = new_data[arg];

		new_data_available[arg] = 0;

		retVal = 1;
	}

	sei();

	return retVal;
}

uint32_t motion_enc_get_last_ts(int deviceId)
{
	uint32_t ts;

	cli();

	ts = last_timestamp[deviceId];

	sei();

	return ts;
}

uint32_t motion_get_tsc(void)
{
	uint32_t retVal;

	cli();

	retVal = tov_cntr_cumulative[0]*TC_20_MS_PERIOD_AT_PS_8
	        + *encoders[0].tc_p->TCNT_ptr;

	sei();

	return retVal;
}


/**************************************************
 *        Motion Module Internal Functions        *
 **************************************************/

static inline void enc_init(int enc_id)
{

	/* initialize variables */

	tov_cntr_cumulative[enc_id] = 0;
	last_icr[enc_id] = 0;
	tov_cntr[enc_id] = 0;

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

static inline void increment_tov_cntr(int enc_id)
{
	tov_cntr[enc_id]++;
	tov_cntr_cumulative[enc_id]++;
}

static inline void handle_transition(int enc_id)
{
	uint32_t delta;
	uint16_t icr;
	uint32_t tov;

	enc_toggle_trigger_edge(enc_id);

	/* clear the Input Capture Flag */

	*encoders[enc_id].tc_p->TIFR_ptr |= (1 << 5);

	icr = *encoders[enc_id].tc_p->ICR_ptr;
	tov =  tov_cntr[enc_id];

	tov_cntr[enc_id] = 0;

	if (tov == 0)
	{
		delta = icr - last_icr[enc_id];
	}
	else
	{
		delta = icr + tov*TC_20_MS_PERIOD_AT_PS_8 - last_icr[enc_id];
	}

	last_timestamp[enc_id] = tov_cntr_cumulative[0]*TC_20_MS_PERIOD_AT_PS_8
	        + *encoders[0].tc_p->TCNT_ptr;

	new_data[enc_id] = delta;

	new_data_available[enc_id] = 1;

	last_icr[enc_id] = icr;
}

static inline void enc_toggle_trigger_edge(int enc_id)
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

/* TIMER/COUNTER CONFIGURATION FUNCTIONS */

/* tc_init -- Initialize a Timer/Counter module
 *
 * Will set the output compare mode, the waveform generation mode,
 * the prescaler value and the TOP value of a Timer/Counter module.
 *
 * The configuration that is used is defined in a structure in the source file.
 *
 * Parameter:
 *   - arg
 *   The Timer/Counter module.
 */

inline static void tc_init(int arg)
{
	/* For all the channels, set the output compare mode
	 * to 0 (pin is disconnected from the TC module).
	 */

	tc_set_com(tc_module_s[arg].TCCR_A_ptr,
	           TC_CHANNEL_A,
	           0b00);

	tc_set_com(tc_module_s[arg].TCCR_A_ptr,
	           TC_CHANNEL_B,
	           0b00);

	tc_set_com(tc_module_s[arg].TCCR_A_ptr,
	           TC_CHANNEL_C,
	           0b00);

	/* Set the Waveform Generation Mode */

	tc_set_wgm(tc_module_s[arg].TCCR_A_ptr,
			   tc_module_s[arg].TCCR_B_ptr,
			   tc_module_s[arg].wgm_mode);

	tc_set_prescaler(tc_module_s[arg].TCCR_B_ptr,
	                 tc_module_s[arg].prescale);

	/* This value determines the rate at which the pulses are sent. */
	/* Store the value in the Input Capture Register */


	*tc_module_s[arg].TOP_value_p = TC_20_MS_PERIOD_AT_PS_8 - 1;

}

/* tc_init_ddr -- Set a pin as an input or as an output
 *
 * Parameters:
 *  - DDR_ptr
 *      A pointer to the Data Direction Register associated with the pin.
 *
 *  - PORT_ptr
 *      A pointer to the PORT Register associated with the pin.
 *
 *  - pin
 *      The bit that corresponds to the pin in the PORT Register.
 *
 *  - port_value
 *      The value that will be used to initialize the PORT register.
 *
 *  - ddr_value
 *      The value that will be used to initialize the DDR register.
 */

inline static void tc_init_ddr(volatile uint8_t* DDR_ptr,
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


/* tc_set_com -- Set the Compare Output Mode for channel A, B, or C.
 *
 * Parameters:
 *  - TCCR_A_ptr
 *      A pointer to the TC module's configuration register A.
 *
 *  - channel
 *      The TC channel for which the compare output mode is set.
 *
 *  - co_mode
 *      The desired compare output mode of the channel.
 *
 */

inline static void tc_set_com(volatile uint8_t* TCCR_A_ptr,
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


/* tc_set_wgm -- Set the Waveform Generation Mode
 *
 * Parameters:
 *  - TCCR_A_ptr
 *      A pointer to the TC module's configuration register A.
 *
 *  - TCCR_B_ptr
 *      A pointer to the configuration register B.
 *
 *  - wgm
 *      The desired waveform generation mode.
 *
 */

inline static void tc_set_wgm(volatile uint8_t* TCCR_A_ptr,
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

/* tc_set_prescaler -- Set the prescaler
 *
 * Parameters:
 *  - TCCR_B_ptr
 *      A pointer to the TC module's configuration register B.
 *
 *  - prescaler
 *      The desired clock frequency divider.
 *
 */

inline static void tc_set_prescaler(volatile uint8_t* TCCR_B_ptr,
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

	/* write the new value to the configuration register */

	*TCCR_B_ptr = tccrb;
}


