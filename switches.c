//------------------------------------------------------------------------------
// switches.c  –  Switch Processing
//
// WHAT EACH SWITCH DOES:
//
//   SW1 (P4.1) – TOGGLES P3.4 between SMCLK output and GPIO input.
//                Each press flips the mode.
//                LCD line 0 always shows the current P3.4 state.
//
//   SW2 (P2.3) – CYCLES shape selection when no shape is running.
//                Starts the selected shape when one is selected.
//                (IDLE -> CIRCLE -> FIGURE-8 -> TRIANGLE -> IDLE -> ...)
//
// The two functions are completely independent of each other.
//------------------------------------------------------------------------------

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include <string.h>

extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char current_shape;
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;
extern unsigned char selected_shape;

// Track current P3.4 mode so SW1 can toggle it
static char p3_4_mode = USE_GPIO;

//------------------------------------------------------------------------------
// Init_Switches
//------------------------------------------------------------------------------
void Init_Switches(void){
    // SW1  P4.1 – active-low, pull-up, falling-edge interrupt
    P4OUT |=  SW1;
    P4DIR &= ~SW1;
    P4REN |=  SW1;
    P4IES |=  SW1;
    P4IE  |=  SW1;
    P4IFG &= ~SW1;

    // SW2  P2.3 – active-low, pull-up, falling-edge interrupt
    P2OUT |=  SW2;
    P2DIR &= ~SW2;
    P2REN |=  SW2;
    P2IES |=  SW2;
    P2IE  |=  SW2;
    P2IFG &= ~SW2;
}

//------------------------------------------------------------------------------
// ISRs – flag only, all logic runs in the foreground
//------------------------------------------------------------------------------
#pragma vector = PORT4_VECTOR
__interrupt void switch1_interrupt(void){
    if(P4IFG & SW1){
        if(!(P4IN & SW1)) sw1_pressed = 1;
        P4IFG &= ~SW1;
    }
}

#pragma vector = PORT2_VECTOR
__interrupt void switch2_interrupt(void){
    if(P2IFG & SW2){
        if(!(P2IN & SW2)) sw2_pressed = 1;
        P2IFG &= ~SW2;
    }
}

//------------------------------------------------------------------------------
// Update the full display
// Line 0: P3.4 pin mode  (set by SW1)
// Line 1: selected shape  (set by SW2)
// Line 2: SW1 hint
// Line 3: SW2 hint
//------------------------------------------------------------------------------
static void update_display_all(void){
    // Line 0 – current P3.4 mode
    if(p3_4_mode == USE_SMCLK){
        strcpy(display_line[0], "P3.4=SMCLK");
    } else {
        strcpy(display_line[0], "P3.4= GPIO");
    }

    // Line 1 – selected shape
    switch(selected_shape){
        case SHAPE_NONE:     strcpy(display_line[1], "  IDLE    "); break;
        case SHAPE_CIRCLE:   strcpy(display_line[1], "  CIRCLE  "); break;
        case SHAPE_FIGURE8:  strcpy(display_line[1], " FIGURE-8 "); break;
        case SHAPE_TRIANGLE: strcpy(display_line[1], " TRIANGLE "); break;
        default:             strcpy(display_line[1], "          "); break;
    }

    // Line 2 – SW1 hint
    strcpy(display_line[2], "SW1:CLK   ");

    // Line 3 – SW2 hint
    if(current_shape == SHAPE_NONE){
        strcpy(display_line[3], "SW2:Select");
    } else {
        strcpy(display_line[3], " RUNNING  ");
    }

    display_changed = TRUE;
}

// Expose for use by main.c initial display setup
void update_shape_display(void){ update_display_all(); }

//------------------------------------------------------------------------------
// Switch 1 Process – TOGGLE P3.4 between SMCLK output and GPIO input
//
// Each press of SW1 flips the mode:
//   GPIO  -> SMCLK : Init_Port3(USE_SMCLK)  SEL1=0 SEL0=1 DIR=output
//   SMCLK -> GPIO  : Init_Port3(USE_GPIO)   SEL1=0 SEL0=0 DIR=input
//------------------------------------------------------------------------------
void Switch1_Process(void){
    if(sw1_pressed){
        sw1_pressed = 0;

        if(p3_4_mode == USE_GPIO){
            Init_Port3(USE_SMCLK);   // Route 500 kHz SMCLK to P3.4
            p3_4_mode = USE_SMCLK;
        } else {
            Init_Port3(USE_GPIO);    // Return P3.4 to plain GPIO input
            p3_4_mode = USE_GPIO;
        }

        update_display_all();

        P4IFG &= ~SW1;
        P4IE  |=  SW1;
    }
}

//------------------------------------------------------------------------------
// Switch 2 Process – CYCLE shape selection / START shape
//
// When no shape is running:
//   First press  – advance selection (IDLE->CIRCLE->FIGURE8->TRIANGLE->IDLE)
//   If a shape other than IDLE is already selected, start it instead
//
// Simpler rule actually used here:
//   If current_shape == NONE and selected_shape == NONE  -> advance selection
//   If current_shape == NONE and selected_shape != NONE  -> start shape
//   If current_shape != NONE                             -> ignore (shape running)
//------------------------------------------------------------------------------
void Switch2_Process(void){
    if(sw2_pressed){
        sw2_pressed = 0;

        if(current_shape == SHAPE_NONE){
            if(selected_shape == SHAPE_NONE){
                // Nothing selected yet – advance to first shape
                selected_shape = SHAPE_CIRCLE;
            } else {
                // A shape is selected – start it, then reset selection
                start_shape(selected_shape);
                selected_shape = SHAPE_NONE;
            }
        }
        // If a shape is already running, SW2 does nothing

        update_display_all();

        P2IFG &= ~SW2;
        P2IE  |=  SW2;
    }
}

//------------------------------------------------------------------------------
// Switches_Process – called every pass of the main while loop
//------------------------------------------------------------------------------
void Switches_Process(void){
    Switch1_Process();
    Switch2_Process();
}

//------------------------------------------------------------------------------
// Enable / Disable helpers
//------------------------------------------------------------------------------
void enable_switch_SW1(void){ P4IFG &= ~SW1; P4IE |= SW1; }
void enable_switch_SW2(void){ P2IFG &= ~SW2; P2IE |= SW2; }
void disable_switch_SW1(void){ P4IE &= ~SW1; }
void disable_switch_SW2(void){ P2IE &= ~SW2; }
