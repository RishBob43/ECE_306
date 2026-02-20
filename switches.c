//------------------------------------------------------------------------------
// Switch Processing with Shape Selection
// SW1: Cycle through shapes (Circle -> Figure-8 -> Triangle -> Idle)
// SW2: Execute selected shape
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
extern volatile unsigned char current_shape;

// Switch variables - defined in globals.c
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;
extern unsigned char selected_shape;

// Debounce timing
#define DEBOUNCE_TIME 200  // 200ms debounce

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
// Update display with selected shape
//------------------------------------------------------------------------------
void update_shape_display(void) {
    strcpy(display_line[0], "SELECT:   ");

    switch(selected_shape) {
        case SHAPE_NONE:
            strcpy(display_line[1], "  IDLE    ");
            strcpy(display_line[2], " SW1:Next ");
            strcpy(display_line[3], " SW2:Start");
            break;

        case SHAPE_CIRCLE:
            strcpy(display_line[1], "  CIRCLE  ");
            strcpy(display_line[2], " SW1:Next ");
            strcpy(display_line[3], " SW2:Start");
            break;

        case SHAPE_FIGURE8:
            strcpy(display_line[1], " FIGURE-8 ");
            strcpy(display_line[2], " SW1:Next ");
            strcpy(display_line[3], " SW2:Start");
            break;

        case SHAPE_TRIANGLE:
            strcpy(display_line[1], " TRIANGLE ");
            strcpy(display_line[2], " SW1:Next ");
            strcpy(display_line[3], " SW2:Start");
            break;
    }

    display_changed = TRUE;
}

//------------------------------------------------------------------------------
// Process SW1 - Select Shape
//------------------------------------------------------------------------------
void Switch1_Process(void) {
    if(sw1_pressed) {
        sw1_pressed = 0;

        // Only allow shape selection when no shape is running
        if(current_shape == SHAPE_NONE) {
            // Cycle through shapes
            selected_shape++;
            if(selected_shape > SHAPE_TRIANGLE) {
                selected_shape = SHAPE_NONE;
            }

            update_shape_display();
        }

        // Clear flag and re-enable interrupt immediately
        P4IFG &= ~SW1;
        P4IE |= SW1;
    }
}

//------------------------------------------------------------------------------
// Process SW2 - Execute Shape
//------------------------------------------------------------------------------
void Switch2_Process(void) {
    if(sw2_pressed) {
        sw2_pressed = 0;

        // Only start shape if one is selected and none is running
        if(selected_shape != SHAPE_NONE && current_shape == SHAPE_NONE) {
            start_shape(selected_shape);
        }

        // Clear flag and re-enable interrupt immediately
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
// Enable switch interrupts
//------------------------------------------------------------------------------
void enable_switch_SW1(void) {
    P4IFG &= ~SW1;
    P4IE |= SW1;
}

void enable_switch_SW2(void) {
    P2IFG &= ~SW2;
    P2IE |= SW2;
}

//------------------------------------------------------------------------------
// Disable switch interrupts
//------------------------------------------------------------------------------
void disable_switch_SW1(void) {
    P4IE &= ~SW1;
}

void disable_switch_SW2(void) {
    P2IE &= ~SW2;
}
