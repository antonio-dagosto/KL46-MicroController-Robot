#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

#include "Hardware_init.h"
#include "Motor_control.h"
#include "Encoder_PID.h"
#include "Switch_IRQ.h"

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    /* Initialize hardware, encoder/PID system, and switch interrupts */
    initAllHardware();
    initEncoderAndPID();
    initSwitchIRQ();
    stopMotors();

    /* Main loop: Wait for a switch press (handled via IRQ) */
    while (1)
    {
        if (switch_pressed == 1)  /* SW1 (PORTC pin 3) */
        {
            PRINTF("SW1 pressed - Starting maintain speed routine...\n");
            switch_pressed = 0;  /* Clear flag */
            delay_ms(2000);      /* Wait 2 seconds before starting */

            /* Set initial speed */
            motor_speed(base_pwm, base_pwm);

            /* Maintain speed for 6 seconds via PID updates */
            delay_ms(6000);

            stopMotors();
            PRINTF("Motion complete. Motors stopped.\n");
        }
        else if (switch_pressed == 2)  /* SW2 (PORTC pin 12) */
        {
            PRINTF("SW2 pressed - Alternate routine...\n");
            switch_pressed = 0;
            /* Insert alternate behavior here */
        }
        /* Optionally check for switch_pressed == 3 for a PORTD switch */
    }
}
