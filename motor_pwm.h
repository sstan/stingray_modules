/*
 * motor_pwm.h
 *
 *  Created on: Jan 23, 2015
 *      Author: sstan
 */

#ifndef MOTOR_PWM_H_
#define MOTOR_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTOR_PWM_LEFT				0
#define MOTOR_PWM_RIGHT				1
#define MOTOR_PWM_CENTER 			2


/* Motor PWM module entry points */

void servo_start(int arg, uint16_t pulse_width_cycles);

void servo_stop(int arg);

void servo_pulse_width_set(int arg, uint16_t pulse_width_cycles);

uint16_t servo_pulse_width_get(int arg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H_ */
