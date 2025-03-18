/*
 * Switch_IRQ.h
 *
 *  Created on: Mar 18, 2025
 *      Author: antho
 */

#ifndef SWITCH_IRQ_H_
#define SWITCH_IRQ_H_

/* Global flag for switch interrupts:
   0 = no switch pressed,
   1 = SW1 (PORTC pin 3),
   2 = SW2 (PORTC pin 12),
   3 = SW3 (PORTD, if used) */
extern volatile int switch_pressed;

void initSwitchIRQ(void);

#endif /* SWITCH_IRQ_H_ */
