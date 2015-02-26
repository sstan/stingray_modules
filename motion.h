/*
 * motion.h
 *
 *  Created on: Feb 26, 2015
 *      Author: sstan
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <stdint.h>

#define MOTION_WHEEL_LEFT        0
#define MOTION_WHEEL_RIGHT       1
#define MOTION_SERVO_CENTER      2

/**************************************************
 *         Motion Module Initialization           *
 **************************************************/

/* motion_init -- Module initialisation
 *
 * Call at the beginning of the program.
 *
 * This function initializes this module.
 *
 */
void motion_init(void);


/**************************************************
 *                  Servo Motors                  *
 **************************************************/

/* motion_servo_set_pulse_width -- set the pulse width length
 *
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 *   - uint16_t ticks
 *     The pulse width length in ticks.
 *
 */
void motion_servo_set_pulse_width(int arg, uint16_t ticks);


/* motion_servo_get_pulse_width -- get the pulse width length
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 * Return value:
 *   The current pulse width length (in ticks).
 *
 */
uint16_t motion_servo_get_pulse_width(int arg);


/* motion_servo_start -- start a servo motor
 *
 * This function connects the control signal to the appropriate output pin.
 *
 * The control signal is an electric pulse (step function) that is generated
 * at a rate of 20 Hz.
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 *
 */
void motion_servo_start(int arg);


/* motion_servo_stop -- stop a servo motor
 *
 * This function disconnects the control signal from the output pin.
 *
 * Instead of the control signal, a constant value of 0V is applied to the
 * output. This turns the servo off.
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 */
void motion_servo_stop(int arg);


/**************************************************
 *                 Optical Encoder                *
 **************************************************/

/* motion_enc_read -- poll an optical encoder
 *
 * Parameter:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *
 *   - uint32_t* val
 *     Provide the pointer to the variable to which the time elapsed
 *     (in ticks) between the most recent and the second most
 *     recent input capture events will be written. Nothing is written
 *     if no new data is available.
 *
 *
 * Return value:
 *   - 0 if no new data is available. Otherwise, 1.
 *
 */
int motion_enc_read(int arg, uint32_t* val);


/* motion_enc_get_count -- input capture event counter
 *
 * This function returns the number of input capture events that have
 * occurred since the last reset. Each input capture event corresponds
 * to an optical encoder increment.
 *
 * An optical encoder increment represents an angular rotation
 * of 360/64 degrees.
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *
 * Return value:
 *   - The count (the number of input capture events)
 *
 */
uint32_t motion_enc_get_count(int arg);


/* motion_enc_reset_count -- Clear the input capture event counter
 *
 * This function resets an input capture event counter.
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *
 */
void motion_enc_reset_count(int arg);


/* motion_enc_get_teslice -- time elapsed since the last input capture event
 *
 * This function returns the number of ticks elapsed since the last input
 * capture event has occurred.
 *
 * Parameters:
 *   - int arg
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *
 */
uint32_t motion_enc_get_teslice(int arg);


/* motion_enc_get_tsc -- The 32-bit Time Stamp Counter of this module
 *
 * This function returns the TSC.
 *
 * The Time Stamp Counter (TSC) is a high-resolution way of
 * getting timing information. It holds the count of the number of cycles
 * (ticks) since the initialization of this module.
 *
 * The TSC is incremented at a rate of 2MHz.
 *
 */
uint32_t motion_enc_get_tsc(void);


#endif /* MOTION_H_ */
