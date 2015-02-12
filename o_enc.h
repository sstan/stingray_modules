/*
 * o_enc.h
 *
 *  Created on: Jan 28, 2015
 *      Author: sstan
 */

#ifndef O_ENC_H_
#define O_ENC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ENC_ID_ENCODER_LEFT     0
#define ENC_ID_ENCODER_RIGHT    1
#define ENC_NUMBER_OF_ENCODERS  2

void enc_init(uint8_t enc_id);

void enc_read_n_timestamps(uint8_t enc_id, int n, uint32_t timestamp[]);

uint32_t enc_time_elapsed_since(uint8_t enc_id, uint32_t timestamp);

uint16_t enc_get_count(uint8_t enc_id);

void enc_reset(uint8_t enc_id);

#ifdef __cplusplus
}
#endif

#endif /* O_ENC_H_ */
