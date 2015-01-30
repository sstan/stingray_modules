/*
 * o_enc.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Nco
 */

#ifndef O_ENC_H_
#define O_ENC_H_

#define OPTICAL_ENCODER_LEFT	0
#define OPTICAL_ENCODER_RIGHT	1

#define OENC_STATE_STOPPED			0
#define OENC_STATE_NEED_ONE_MORE	1
#define OENC_STATE_VALID			2

#define OENC_SAMPLE_EXPIRES_CYCLES	((uint16_t) 20000)

struct encoder_state {
	uint8_t		encoder_id;
	uint8_t 	state;
	uint16_t	last_icr_val;
	uint16_t	counter;
	uint16_t	period;
};

struct encoder_state* enc_init(int arg);

struct encoder_state* enc_get_handle(int arg);

void enc_compute_tr_period(int arg);

void enc_clear_counter(int arg);

#endif /* O_ENC_H_ */
