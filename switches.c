//------------------------------------------------------------------------------
// switches.c  –  Switch Processing
//
// SW1 (P4.1) – Cycles through motor H-bridge test states:
//
//   State 0: ALL OFF   – all motor port pins LOW
//   State 1: R_FORWARD – P6.1 driven HIGH  (check TP1, TP3)
//   State 2: R_REVERSE – P6.3 driven HIGH  (check TP2, TP4)
//   State 3: L_FORWARD – P6.2 driven HIGH  (check TP5, TP7)
//   State 4: L_REVERSE – P6.4 driven HIGH  (check TP6, TP8)
//
//   Each pin is reconfigured as plain GPIO output (SEL0=0, SEL1=0) and
//   driven HIGH so the H-bridge sees a steady DC level – easier to measure
//   with a DMM than a PWM signal.
//
//   With motor pin LOW  (off): both TPs should equal VBAT
//   With motor pin HIGH (on):  high-side TP <= VBAT/2, low-side TP ~= GND
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

//------------------------------------------------------------------------------
// Test states
//   0 = ALL OFF
//   1 = R_FORWARD  P6.1
//   2 = R_REVERSE  P6.3
//   3 = L_FORWARD  P6.2
//   4 = L_REVERSE  P6.4
//------------------------------------------------------------------------------
#define TEST_STATES  (5)

static unsigned char test_state = 0;

//------------------------------------------------------------------------------
// all_motors_off
// Reconfigure all four motor pins as GPIO outputs driven LOW.
// Clears SEL0/SEL1 to disconnect Timer B3 peripheral function first.
//------------------------------------------------------------------------------
static void all_motors_off(void){
    // R_FORWARD  P6.1
    P6SEL0 &= ~R_FORWARD;
    P6SEL1 &= ~R_FORWARD;
    P6OUT  &= ~R_FORWARD;
    P6DIR  |=  R_FORWARD;

    // L_FORWARD  P6.2
    P6SEL0 &= ~L_FORWARD;
    P6SEL1 &= ~L_FORWARD;
    P6OUT  &= ~L_FORWARD;
    P6DIR  |=  L_FORWARD;

    // R_REVERSE  P6.3
    P6SEL0 &= ~R_REVERSE;
    P6SEL1 &= ~R_REVERSE;
    P6OUT  &= ~R_REVERSE;
    P6DIR  |=  R_REVERSE;

    // L_REVERSE  P6.4
    P6SEL0 &= ~L_REVERSE;
    P6SEL1 &= ~L_REVERSE;
    P6OUT  &= ~L_REVERSE;
    P6DIR  |=  L_REVERSE;
}

//------------------------------------------------------------------------------
// apply_test_state
// All motors off first, then drive exactly one pin HIGH for the active state.
//
// LCD layout:
//   Line 0: signal being driven HIGH (or "ALL  OFF")
//   Line 1: test points to probe
//   Line 2: expected reading on high-side TP
//   Line 3: expected reading on low-side  TP
//------------------------------------------------------------------------------
static void apply_test_state(void){
    all_motors_off();   // always start clean – only one pin HIGH at a time

    switch(test_state){
      case 0:
        // All LOW – every TP should read VBAT
        strcpy(display_line[0], " ALL  OFF ");
        strcpy(display_line[1], "All TPs=  ");
        strcpy(display_line[2], "  = VBAT  ");
        strcpy(display_line[3], "SW1: next ");
        break;

      case 1:
        // R_FORWARD: drive P6.1 HIGH
        P6OUT |= R_FORWARD;
        strcpy(display_line[0], "R_FORWARD ");
        strcpy(display_line[1], "TP1  TP3  ");
        strcpy(display_line[2], "TP1<=Vb/2 ");
        strcpy(display_line[3], "TP3~= GND ");
        break;

      case 2:
        // R_REVERSE: drive P6.3 HIGH
        P6OUT |= R_REVERSE;
        strcpy(display_line[0], "R_REVERSE ");
        strcpy(display_line[1], "TP2  TP4  ");
        strcpy(display_line[2], "TP2<=Vb/2 ");
        strcpy(display_line[3], "TP4~= GND ");
        break;

      case 3:
        // L_FORWARD: drive P6.2 HIGH
        P6OUT |= L_FORWARD;
        strcpy(display_line[0], "L_FORWARD ");
        strcpy(display_line[1], "TP5  TP7  ");
        strcpy(display_line[2], "TP5<=Vb/2 ");
        strcpy(display_line[3], "TP7~= GND ");
        break;

      case 4:
        // L_REVERSE: drive P6.4 HIGH
        P6OUT |= L_REVERSE;
        strcpy(display_line[0], "L_REVERSE ");
        strcpy(display_line[1], "TP6  TP8  ");
        strcpy(display_line[2], "TP6<=Vb/2 ");
        strcpy(display_line[3], "TP8~= GND ");
        break;

      default:
        test_state = 0;
        break;
    }

    display_changed = TRUE;
}

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

    // Show initial "ALL OFF" state on LCD at startup
    apply_test_state();
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
// Switch1_Process – advance to next test state on each press
//------------------------------------------------------------------------------
void Switch1_Process(void){
    if(sw1_pressed){
        sw1_pressed = 0;

        test_state++;
        if(test_state >= TEST_STATES) test_state = 0;

        apply_test_state();

        P4IFG &= ~SW1;
        P4IE  |=  SW1;
    }
}

//------------------------------------------------------------------------------
// Switch2_Process – reserved, clears flag only
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
