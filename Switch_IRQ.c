/*
 * Switch_IRQ.c
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */
#include "MKL46Z4.h"
#include "Switch_IRQ.h"

volatile int switch_pressed = 0;

void initSwitchIRQ(void)
{
    /* Enable Port C Clock (bit 11 of SIM->SCGC5) */
    SIM->SCGC5 |= (1 << 11);

    /* Configure PORTC pin 3 (SW1) for falling edge interrupts */
    /* Clear first using a mask similar to the sample */
    PORTC->PCR[3] &= ~0xF0703;
    /* Set MUX bits, enable pull-ups, and set interrupt on falling edge
       (0xA for falling edge in bits 19-16; bit 8 for pull enable; bits 1:0 for pull select) */
    PORTC->PCR[3] |= ((0xA << 16) | (1 << 8) | 0x3);
    /* Set as input */
    GPIOC->PDDR &= ~(1 << 3);

    /* Configure PORTC pin 12 (SW2) similarly */
    PORTC->PCR[12] &= ~0xF0703;
    PORTC->PCR[12] |= ((0xA << 16) | (1 << 8) | 0x3);
    GPIOC->PDDR &= ~(1 << 12);

    /* Enable NVIC for PORTC interrupts.
       (Using NVIC_EnableIRQ(PORTC_IRQn) is equivalent to NVIC_EnableIRQ(31) on many systems) */
    NVIC_EnableIRQ(PORTC_IRQn);
}

/* PORTC IRQ Handler for switch interrupts */
void PORTC_IRQHandler(void)
{
    /* Check if interrupt on PORTC pin 3 (SW1) occurred */
    if (PORTC->ISFR & (1 << 3))
    {
        switch_pressed = 1;
        /* Clear interrupt flag for pin 3 */
        PORTC->ISFR |= (1 << 3);
    }

    /* Check if interrupt on PORTC pin 12 (SW2) occurred */
    if (PORTC->ISFR & (1 << 12))
    {
        switch_pressed = 2;
        /* Clear interrupt flag for pin 12 */
        PORTC->ISFR |= (1 << 12);
    }
}
