//------------------------------------------------------------------------------
// switches.c  –  Switch Processing
//
// SW1 (P4.1) – Toggles P3.4 between SMCLK output (500 kHz) and GPIO input.
//              Each press flips the mode.
//              LCD line 0 always shows the current P3.4 state.
//
// SW2 (P2.3) – Reserved / unused.
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
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;

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
// Update display to reflect current P3.4 mode
//------------------------------------------------------------------------------
static void update_display_all(void){
    if(p3_4_mode == USE_SMCLK){
        strcpy(display_line[0], "P3.4=SMCLK");
        strcpy(display_line[1], " 500  kHz ");
    } else {
        strcpy(display_line[0], "P3.4= GPIO");
        strcpy(display_line[1], "          ");
    }
    strcpy(display_line[2], "SW1:Toggle");
    strcpy(display_line[3], " CLK/GPIO ");
    display_changed = TRUE;
}

//------------------------------------------------------------------------------
// Switch 1 Process – Toggle P3.4 between SMCLK output and GPIO input
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
// Switch 2 Process – Reserved, clears flag only
//------------------------------------------------------------------------------
void Switch2_Process(void){
    if(sw2_pressed){
        sw2_pressed = 0;
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
