/*
 * Hardware_init.c
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */

#include "MKL46Z4.h"
#include "Hardware_init.h"

void initAllHardware(void)
{
    /* Enable clocks for PORTB and PORTC */
    SIM->SCGC5 |= (1 << 10) | (1 << 11);

    /* Left Motor: PTB0 (IN1), PTB1 (IN2) */
    PORTB->PCR[0] = 0x100;  /* MUX = 1 (GPIO) */
    PORTB->PCR[1] = 0x100;
    GPIOB->PDDR |= (1 << 0) | (1 << 1);

    /* Right Motor: PTC1 (IN1), PTC2 (IN2) */
    PORTC->PCR[1] = 0x100;
    PORTC->PCR[2] = 0x100;
    GPIOC->PDDR |= (1 << 1) | (1 << 2);

    /* PTB2 -> TPM2_CH0, PTB3 -> TPM2_CH1, set MUX = 3 */
    PORTB->PCR[2] = 0x300;  /* set mux to alt3 for TPM2 CH0 */
    PORTB->PCR[3] = 0x300;  /* set mux to alt3 for TPM2 CH1 */

    /* Enable TPM2 clock and select OSCERCLK */
    SIM->SCGC6 |= (1 << 26);
    SIM->SOPT2 &= ~(0x3 << 24);
    SIM->SOPT2 |= (0x2 << 24);

    /* Configure TPM2 for PWM */
    TPM2->MOD = 7999;
    TPM2->CONTROLS[0].CnSC = 0x28; /* edge-aligned PWM (high at start, low at end) */
    TPM2->CONTROLS[1].CnSC = 0x28; /* edge-aligned PWM (high at start, low at end) */
    TPM2->SC = (1 << 3);          /* Start TPM2 clock */
    TPM2->CONTROLS[0].CnV = 0;
    TPM2->CONTROLS[1].CnV = 0;

    /* Configure switches on PORTC pins 3 and 12 */
    PORTC->PCR[3] &= ~0x700;
    PORTC->PCR[3] = 0x100;
    PORTC->PCR[3] |= 0x3;
    GPIOC->PDDR &= ~(1 << 3);

    PORTC->PCR[12] &= ~0x700;
    PORTC->PCR[12] = 0x100;
    PORTC->PCR[12] |= 0x3;
    GPIOC->PDDR &= ~(1 << 12);
}

void delay_ms(unsigned short delay_t)
{
    SIM->SCGC6 |= (1 << 24);    /* Clock Enable TPM0 */
    SIM->SOPT2 |= (0x2 << 24);  /* Set TPMSRC to OSCERCLK */
    TPM0->CONF |= (0x1 << 17);  /* Stop on Overflow */
    TPM0->SC = (0x1 << 7) | (0x07); /* Reset Timer Overflow Flag, Prescaler = 128 */
    TPM0->MOD = delay_t * 61 + delay_t / 2;
    TPM0->SC |= (0x1 << 3);     /* Start the clock! */
    while (!(TPM0->SC & 0x80))
    {
        /* Wait until Overflow Flag is set */
    }
}

