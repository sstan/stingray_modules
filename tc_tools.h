/*
 * tc_tools.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Nco
 */

#ifndef TC_TOOLS_H_
#define TC_TOOLS_H_

#include <stdint.h>

#define TC_CHANNEL_A 0
#define TC_CHANNEL_B 1
#define TC_CHANNEL_C 2

/* Timer/Counter Helper Functions */

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


#endif /* TC_TOOLS_H_ */
