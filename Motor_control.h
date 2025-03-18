/*
 * Motor_control.h
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

void motor_speed(short left, short right);
void stopMotors(void);
void move_forward(void);

/* Global base PWM value used by the PID system */
extern int base_pwm;

#endif /* MOTOR_CONTROL_H_ */
