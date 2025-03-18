/*
 * Enconder_PID.h
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */

#ifndef ENCODER_PID_H_
#define ENCODER_PID_H_

void initEncoderAndPID(void);
void PIT0_IRQHandler(void);
void PORTA_IRQHandler(void);

/* Global variables used for PID control */
extern volatile int encoderCountLeft;
extern volatile int encoderCountRight;
extern volatile int target_speed;
extern volatile float pid_integral_left;
extern volatile float pid_prev_error_left;
extern volatile float pid_integral_right;
extern volatile float pid_prev_error_right;

#endif /* ENCODER_PID_H_ */
