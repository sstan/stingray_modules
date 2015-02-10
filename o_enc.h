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

#define ENC_ID_ENCODER_LEFT     0
#define ENC_ID_ENCODER_RIGHT    1
#define ENC_NUMBER_OF_ENCODERS  2

/* Input Capture Event Callback
 *
 * This module will call the provided function with the following parameters
 * provided.
 *
 *   - uint8_t enc_id
 *      The optical encoder associated with the event.
 *
 *   - uint32_t timestamp
 *      TC cycles since initialization at the moment of the event.
 *
 *
 */
typedef void (*evt_callback_ptr_t)(uint8_t enc_id, uint32_t timestamp);


/* enc_init -- Initialize an optical encoder
 *
 * Call this function one time per optical encoder.
 *
 * parameters:
 *   - uint8_t enc_id
 *       The ID of the optical encoder.
 *
 *   - evt_callback_ptr_t handler_fn_ptr
 *       A pointer to the function that will be called when an Input Capture
 *       event happens. IMPORTANT: the function will be executed in the
 *       context of an interrupt.
 *
 */
void enc_init(uint8_t enc_id, evt_callback_ptr_t handler_fn_ptr);

/* enc_get_time -- get current time since initialization
 *
 * parameters:
 *   - uint8_t enc_id
 *       The ID of the optical encoder.
 *
 *   - uint32_t* timestamp
 *       Pointer to the timestamp. This function will write the timestamp
 *       at that memory location.
 *
 */
void enc_get_time(uint8_t enc_id, uint32_t* timestamp);

#ifdef __cplusplus
}
#endif

#endif /* O_ENC_H_ */
