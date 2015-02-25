/*
 * tc_tools.h
 *
 *  Created on: Jan 28, 2015
 *      Author: sstan
 *
 *  Timer/Counter configuration tools.
 *
 */

#ifndef TC_TOOLS_H_
#define TC_TOOLS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <avr/io.h>

/* 16-bit Timer/Counter Modules */
#define TC_MODULE_1				0
#define TC_MODULE_3				1
#define TC_MODULE_4				2
#define TC_MODULE_5				3
#define TC_NUMBER_OF_MODULES	4

/* Timer/Counter Channels */
#define TC_CHANNEL_A 0
#define TC_CHANNEL_B 1
#define TC_CHANNEL_C 2

/* Number of 500-nanosecond cycles in 20 milliseconds. */
#define TC_20_MS_PERIOD_AT_PS_8 (uint16_t) 40000

/* The structure that stores the configuration of the TC modules as well as
 * the pointers to the configuration registers belonging to the TC modules.
 */

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

/* These structures stores the Timer/Counter configuration that is used
 * to initialize the Timer/Counter modules.
 */

extern const struct tc_module_registers tc_module_s[TC_NUMBER_OF_MODULES];

/* tc_module_init -- Initialize a Timer/Counter module
 *
 * Will set the output compare mode, the waveform generation mode,
 * the prescaler value and the TOP value of a Timer/Counter module.
 *
 * The configuration that is used is defined in a structure in the source file.
 *
 * Parameter:
 *   - arg
 *     The Timer/Counter module.
 */

void tc_module_init(int arg);

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

void tc_init_ddr(volatile uint8_t* DDR_ptr,
		         volatile uint8_t* PORT_ptr,
				 uint8_t pin,
				 volatile uint8_t port_value,
				 volatile uint8_t ddr_value);

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

void tc_set_com(volatile uint8_t* TCCR_A_ptr,
		        uint8_t channel,
				uint8_t co_mode);

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

void tc_set_wgm(volatile uint8_t* TCCR_A_ptr,
		        volatile uint8_t* TCCR_B_ptr,
				uint8_t wgm);

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

void tc_set_prescaler(volatile uint8_t* TCCR_B_ptr,
		              int prescaler);


#ifdef __cplusplus
}
#endif

#endif /* TC_TOOLS_H_ */
