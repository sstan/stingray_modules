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

void motor_pwm_tc_init(void);

void motor_pwm_start(int arg, uint16_t pulse_width_cycles);

void motor_pwm_stop(int arg);

void motor_pwm_pulse_width_set(int arg, uint16_t pulse_width_cycles);

uint16_t motor_pwm_pulse_width_get(int arg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H_ */
