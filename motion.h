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

/* motion_init -- Module initialization
 *
 * Call at the beginning of the program.
 *
 * This function initializes this module.
 *
 */
void motion_init(void);


/**************************************************
 *                  Servomotors                   *
 **************************************************/

/* motion_servo_set_pulse_width -- set the pulse width length
 *
 *
 * Parameters:
 *   - int deviceId
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 *   - uint16_t ticks
 *     The pulse width length in ticks.
 *
 */
void motion_servo_set_pulse_width(int deviceId, uint16_t pulseWidthTicks);


/* motion_servo_get_pulse_width -- get the pulse width length
 *
 * Parameters:
 *   - int deviceId
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 * Return value:
 *   The current pulse width length (in ticks).
 *
 */
uint16_t motion_servo_get_pulse_width(int deviceId);


/* motion_servo_start -- start a servomotor
 *
 * This function connects the control signal to the appropriate output pin.
 *
 * The control signal is an electric pulse (step function) that is generated
 * at a rate of 20 Hz.
 *
 * Parameters:
 *   - int deviceId
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 *
 */
void motion_servo_start(int deviceId);


/* motion_servo_stop -- stop a servomotor
 *
 * This function disconnects the control signal from the output pin.
 *
 * Instead of the control signal, a constant value of 0V is applied to the
 * output. This turns the servo off.
 *
 * Parameters:
 *   - int deviceId
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *         MOTION_SERVO_CENTER
 *
 */
void motion_servo_stop(int deviceId);


/**************************************************
 *                 Optical Encoder                *
 **************************************************/

/* motion_enc_read -- poll an optical encoder
 *
 * Parameter:
 *   - int deviceId
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
int motion_enc_read(int deviceId, uint32_t* tickCount);


/* motion_enc_get_last_ts -- value of the TSC at the last input capture event
 *
 * This function returns the time stamp of the last input capture event.
 *
 * Parameters:
 *   - int deviceId
 *     The valid options are:
 *         MOTION_WHEEL_LEFT
 *         MOTION_WHEEL_RIGHT
 *
 */
uint32_t motion_enc_get_last_ts(int deviceId);


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
