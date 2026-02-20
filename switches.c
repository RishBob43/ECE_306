//------------------------------------------------------------------------------
// Switch Processing for SMCLK Control
// SW1: Toggle P3.4 between GPIO and SMCLK output
// SW2: Toggle SMCLK frequency between 8MHz and 500kHz
//------------------------------------------------------------------------------

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include <string.h>

// External variables
extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;

// External SMCLK control variables
extern volatile unsigned char smclk_output_enabled;
extern volatile unsigned char smclk_freq_500k;

// Switch press flags
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;

//------------------------------------------------------------------------------
// Initialize Switches
//------------------------------------------------------------------------------
void Init_Switches(void) {
    // SW1 on P4.1
    P4OUT |= SW1;           // Configure pullup resistor
    P4DIR &= ~SW1;          // Direction = input
    P4REN |= SW1;           // Enable pullup resistor
    P4IES |= SW1;           // High to Low edge
    P4IE |= SW1;            // Enable interrupt
    P4IFG &= ~SW1;          // Clear interrupt flag

    // SW2 on P2.3
    P2OUT |= SW2;           // Configure pullup resistor
    P2DIR &= ~SW2;          // Direction = input
    P2REN |= SW2;           // Enable pullup resistor
    P2IES |= SW2;           // High to Low edge
    P2IE |= SW2;            // Enable interrupt
    P2IFG &= ~SW2;          // Clear interrupt flag
}

//------------------------------------------------------------------------------
// Switch Interrupt Service Routines
//------------------------------------------------------------------------------
#pragma vector=PORT4_VECTOR
__interrupt void switch1_interrupt(void) {
    if(P4IFG & SW1) {
        // Debounce: Check if button is actually pressed (LOW)
        if(!(P4IN & SW1)) {
            sw1_pressed = 1;
        }
        P4IFG &= ~SW1;      // Clear interrupt flag
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void switch2_interrupt(void) {
    if(P2IFG & SW2) {
        // Debounce: Check if button is actually pressed (LOW)
        if(!(P2IN & SW2)) {
            sw2_pressed = 1;
        }
        P2IFG &= ~SW2;      // Clear interrupt flag
    }
}

//------------------------------------------------------------------------------
// Process SW1 - Toggle P3.4 between GPIO and SMCLK output
//------------------------------------------------------------------------------
void Switch1_Process(void) {
    if(sw1_pressed) {
        sw1_pressed = 0;

        // Toggle SMCLK output on/off
        if(smclk_output_enabled) {
            // Turn OFF SMCLK output - make P3.4 GPIO
            smclk_output_enabled = FALSE;
            Init_Port3(USE_GPIO);

            // Update display
            strcpy(display_line[0], "P3.4:GPIO ");
            display_changed = TRUE;

            // Optional: Flash LED to indicate change
            P1OUT ^= RED_LED;
        } else {
            // Turn ON SMCLK output
            smclk_output_enabled = TRUE;
            Init_Port3(USE_SMCLK);

            // Update display
            strcpy(display_line[0], "P3.4:SMCLK");
            display_changed = TRUE;

            // Optional: Flash LED to indicate change
            P1OUT ^= RED_LED;
        }

        // Clear flag and re-enable interrupt
        P4IFG &= ~SW1;
        P4IE |= SW1;
    }
}

//------------------------------------------------------------------------------
// Process SW2 - Toggle SMCLK frequency between 8MHz and 500kHz
//------------------------------------------------------------------------------
void Switch2_Process(void) {
    if(sw2_pressed) {
        sw2_pressed = 0;

        // Toggle SMCLK frequency
        if(smclk_freq_500k) {
            // Switch to 8MHz
            smclk_freq_500k = FALSE;
            Set_SMCLK_8MHz();

            // Update display
            strcpy(display_line[1], "FREQ:8MHz ");
            display_changed = TRUE;

            // Optional: Flash LED to indicate change
            P6OUT ^= GRN_LED;
        } else {
            // Switch to 500kHz
            smclk_freq_500k = TRUE;
            Set_SMCLK_500kHz();

            // Update display
            strcpy(display_line[1], "FREQ:500kHz");
            display_changed = TRUE;

            // Optional: Flash LED to indicate change
            P6OUT ^= GRN_LED;
        }

        // Clear flag and re-enable interrupt
        P2IFG &= ~SW2;
        P2IE |= SW2;
    }
}

//------------------------------------------------------------------------------
// Main Switches Process - Called from main loop
//------------------------------------------------------------------------------
void Switches_Process(void) {
    Switch1_Process();
    Switch2_Process();
}

//------------------------------------------------------------------------------
// Enable/Disable switch interrupts (utility functions)
//------------------------------------------------------------------------------
void enable_switch_SW1(void) {
    P4IFG &= ~SW1;
    P4IE |= SW1;
}

void enable_switch_SW2(void) {
    P2IFG &= ~SW2;
    P2IE |= SW2;
}

void disable_switch_SW1(void) {
    P4IE &= ~SW1;
}

void disable_switch_SW2(void) {
    P2IE &= ~SW2;
}
