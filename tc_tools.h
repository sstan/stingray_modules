/*
 * tc_tools.h
 *
 *  Created on: Jan 28, 2015
 *      Author: sstan
 */

#ifndef TC_TOOLS_H_
#define TC_TOOLS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <avr/io.h>

#define TC_CHANNEL_A 0
#define TC_CHANNEL_B 1
#define TC_CHANNEL_C 2

#define TC_MODULE_1				0
#define TC_MODULE_3				1
#define TC_MODULE_4				2
#define TC_MODULE_5				3
#define TC_NUMBER_OF_MODULES	4

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

extern const struct tc_module_registers tc_module_s[TC_NUMBER_OF_MODULES];


/* Timer/Counter Helper Functions */

void tc_module_init(int arg);

void tc_init_ddr(volatile uint8_t* DDR_ptr,
		         volatile uint8_t* PORT_ptr,
				 uint8_t pin,
				 volatile uint8_t port_value,
				 volatile uint8_t ddr_value);

void tc_set_com(volatile uint8_t* TCCR_A_ptr,
		        uint8_t channel,
				uint8_t co_mode);

void tc_set_wgm(volatile uint8_t* TCCR_A_ptr,
		        volatile uint8_t* TCCR_B_ptr,
				uint8_t wgm);

void tc_set_prescaler(volatile uint8_t* TCCR_B_ptr,
		              int prescaler);


#ifdef __cplusplus
}
#endif

#endif /* TC_TOOLS_H_ */
