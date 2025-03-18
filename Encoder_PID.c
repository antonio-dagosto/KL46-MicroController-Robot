/*
 * Encoder_PID.c
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */

#include "MKL46Z4.h"
#include "Encoder_PID.h"
#include "motor_control.h"
#include "fsl_debug_console.h"

volatile int encoderCountLeft = 0;
volatile int encoderCountRight = 0;
volatile int target_speed = 20;  /* Target encoder pulses per PID interval */
volatile float pid_integral_left = 0;
volatile float pid_prev_error_left = 0;
volatile float pid_integral_right = 0;
volatile float pid_prev_error_right = 0;

#define KP  1.0f    /* Proportional gain */
#define KI  0.1f    /* Integral gain */
#define KD  0.05f   /* Derivative gain */

void initEncoderAndPID(void)
{
    /* Enable clock for PORTA (for encoder inputs) */
    SIM->SCGC5 |= (1 << 9);  /* Enable PORTA clock */

    /* Configure PTA6 for encoder input (Motor 1) */
    PORTA->PCR[6] = 0x100;
    PORTA->PCR[6] |= (0x0A << 16);  /* Interrupt on falling edge (example) */
    GPIOA->PDDR &= ~(1 << 6);

    /* Configure PTA14 for encoder input (Motor 2) */
    PORTA->PCR[14] = 0x100;
    PORTA->PCR[14] |= (0x0A << 16);
    GPIOA->PDDR &= ~(1 << 14);

    NVIC_EnableIRQ(PORTA_IRQn);

    /* Configure PIT (Periodic Interrupt Timer) for PID updates */
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0x00;  /* Enable PIT, timers run in debug mode */
    PIT->CHANNEL[0].LDVAL = (SystemCoreClock / 128) / 10 - 1;  /* 100 ms interval */
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
    NVIC_EnableIRQ(PIT0_IRQn);
}

/* PIT0 Interrupt Handler: PID update */
void PIT0_IRQHandler(void)
{
    PIT->CHANNEL[0].TFLG = 1;  /* Clear PIT0 interrupt flag */

    int measured_left = encoderCountLeft;
    int measured_right = encoderCountRight;
    encoderCountLeft = 0;
    encoderCountRight = 0;

    int error_left = target_speed - measured_left;
    pid_integral_left += error_left;
    int derivative_left = error_left - pid_prev_error_left;
    float pid_output_left = KP * error_left + KI * pid_integral_left + KD * derivative_left;
    pid_prev_error_left = error_left;

    int error_right = target_speed - measured_right;
    pid_integral_right += error_right;
    int derivative_right = error_right - pid_prev_error_right;
    float pid_output_right = KP * error_right + KI * pid_integral_right + KD * derivative_right;
    pid_prev_error_right = error_right;

    int new_pwm_left = base_pwm + (int)pid_output_left;
    int new_pwm_right = base_pwm + (int)pid_output_right;

    /* Clamp new PWM values */
    if (new_pwm_left > 100) new_pwm_left = 100;
    if (new_pwm_left < 0) new_pwm_left = 0;
    if (new_pwm_right > 100) new_pwm_right = 100;
    if (new_pwm_right < 0) new_pwm_right = 0;

    motor_speed(new_pwm_left, new_pwm_right);
}

/* PORTA Interrupt Handler: Encoder pulse count update */
void PORTA_IRQHandler(void)
{
    if (PORTA->ISFR & (1 << 6))
    {
        encoderCountLeft++;
        PORTA->ISFR = (1 << 6);
    }

    if (PORTA->ISFR & (1 << 14))
    {
        encoderCountRight++;
        PORTA->ISFR = (1 << 14);
    }
}

