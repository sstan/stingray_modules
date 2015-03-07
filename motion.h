/*
 * motion.h
 *
 *  Created on: Feb 26, 2015
 *      Author: Stefan Stanisic
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <stdint.h>

/* Stingray devices */

#define MOTION_WHEEL_LEFT        0
#define MOTION_WHEEL_RIGHT       1
#define MOTION_SERVO_CENTER      2


void     motion_init(void);

void     motion_servo_set_pulse_width(int deviceId, uint16_t pulseWidthTicks);
uint16_t motion_servo_get_pulse_width(int deviceId);
void     motion_servo_start          (int deviceId);
void     motion_servo_stop           (int deviceId);

int      motion_enc_read(int deviceId, uint32_t* tickCount);


#endif /* MOTION_H_ */
