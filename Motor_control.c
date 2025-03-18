/*
 * Motor_control.c
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */

#include "MKL46Z4.h"
#include "Motor_control.h"
#include <stdlib.h>

int base_pwm = 70;  /* Base PWM value (0 - 100) */

void motor_speed(short left, short right)
{
    if (left > 100)   left = 100;
    if (left < -100)  left = -100;
    if (right > 100)  right = 100;
    if (right < -100) right = -100;

    /* Left motor: PTB0 (IN1), PTB1 (IN2) */
    if (left > 0)
    {
        GPIOB->PSOR = (1 << 0); /* Set motor pins for forward polarity */
        GPIOB->PCOR = (1 << 1);
    }
    else if (left < 0)
    {
        GPIOB->PCOR = (1 << 0); /* Set motor pins for reverse polarity */
        GPIOB->PSOR = (1 << 1);
    }
    else
    {
        GPIOB->PSOR = (1 << 0);
        GPIOB->PSOR = (1 << 1);
    }

    /* Right motor: PTC1 (IN1), PTC2 (IN2) */
    if (right > 0)
    {
        GPIOC->PSOR = (1 << 1); /* Set motor pins for forward polarity */
        GPIOC->PCOR = (1 << 2);
    }
    else if (right < 0)
    {
        GPIOC->PCOR = (1 << 1); /* Set motor pins for reverse polarity */
        GPIOC->PSOR = (1 << 2);
    }
    else
    {
        GPIOC->PSOR = (1 << 1);
        GPIOC->PSOR = (1 << 2);
    }

    uint16_t leftDuty  = (uint16_t)((abs(left)  * TPM2->MOD) / 100);
    uint16_t rightDuty = (uint16_t)((abs(right) * TPM2->MOD) / 100);

    TPM2->CONTROLS[0].CnV = leftDuty;   /* Update left motor PWM */
    TPM2->CONTROLS[1].CnV = rightDuty;  /* Update right motor PWM */
}

void stopMotors(void)
{
    motor_speed(0, 0);
}

void move_forward(void)
{
    motor_speed(100, 100);
}

