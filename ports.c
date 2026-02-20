#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

void Init_Ports(void){
    Init_Port1();
    Init_Port2();
    Init_Port3();
    Init_Port4();
    Init_Port5();
    Init_Port6();
}

void Init_Port1(void){

    // Configure Port 1
    //---
    P1OUT = 0x00;           // P1 set Low
    P1DIR = 0x00;           // Set P1 direction to output

    P1SEL0 &= ~RED_LED;    // SLOW_CLK GPIO operation
    P1SEL1 &= ~RED_LED;    // SLOW_CIK GPIO operation
    P1OUT &= ~RED_LED;     // Initial Value = Low / Off
    P1DIR |= RED_LED;     // Direction = output

    P1SEL0 &= ~V_A1_SEEED;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~V_A1_SEEED;   // CHECK BAT GPIO operation
    P1OUT &= ~V_A1_SEEED;    // Initial Value = Low / Off
    P1DIR &= ~V_A1_SEEED;      // Direction = Input

    P1SEL0 &= ~V_DETECT_L;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~V_DETECT_L;   // CHECK BAT GPIO operation
    P1OUT &= ~V_DETECT_L;    // Initial Value = Low / Off
    P1DIR &= ~V_DETECT_L;      // Direction = Input

    P1SEL0 &= ~V_DETECT_R;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~V_DETECT_R;   // CHECK BAT GPIO operation
    P1OUT &= ~V_DETECT_R;    // Initial Value = Low / Off
    P1DIR &= ~V_DETECT_R;      // Direction = Input

    P1SEL0 &= ~V_A4_SEEED;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~V_A4_SEEED;   // CHECK BAT GPIO operation
    P1OUT &= ~V_A4_SEEED;    // Initial Value = Low / Off
    P1DIR &= ~V_A4_SEEED;      // Direction = Input

    P1SEL0 &= ~V_THUMB ;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~V_THUMB ;   // CHECK BAT GPIO operation
    P1OUT &= ~V_THUMB ;    // Initial Value = Low / Off
    P1DIR &= ~V_THUMB ;      // Direction = Input

    P1SEL0 &= ~UCA0RXD;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~UCA0RXD;   // CHECK BAT GPIO operation
    P1OUT &= ~UCA0RXD;    // Initial Value = Low / Off
    P1DIR &= ~UCA0RXD;      // Direction = Input

    P1SEL0 &= ~UCA0TXD;   // CHECK_ BAT GPIO operation
    P1SEL1 &= ~UCA0TXD;   // CHECK BAT GPIO operation
    P1OUT &= ~UCA0TXD;    // Initial Value = Low / Off
    P1DIR &= ~UCA0TXD;      // Direction = Input
}



void Init_Port2(void){

    // Configure Port 2

    //---

    P2OUT = 0x00;           // P2 set Low
    P2DIR = 0x00;           // Set P2 direction to output

    P2SEL0 &= ~SLOW_CLK;    // SLOW_CLK GPIO operation
    P2SEL1 &= ~SLOW_CLK;    // SLOW_CIK GPIO operation
    P2OUT &= ~SLOW_CLK;     // Initial Value = Low / Off
    P2DIR |= SLOW_CLK;     // Direction = output

    P2SEL0 &= ~CHECK_BAT;   // CHECK_ BAT GPIO operation
    P2SEL1 &= ~CHECK_BAT;   // CHECK BAT GPIO operation
    P2OUT &= ~CHECK_BAT;    // Initial Value = Low / Off
    P2DIR |= CHECK_BAT;      // Direction = output

    P2SEL0 &= ~IR_LED;      // P2 2 GPIO operation
    P2SEL1 &= ~IR_LED;      // P2 2 GPIO operation
    P2OUT &= ~IR_LED;       // Initial Value = Low / Off
    P2DIR |= IR_LED;        // Direction = output

    P2SEL0 &= ~SW2;         // SW2 Operation
    P2SEL1 &= ~SW2;         // SW2 Operation
    P2OUT |= SW2;           // Configure pullup resistor
    P2DIR &= ~SW2;          // Direction = input
    P2REN |= SW2;            // Enable pullup resistor

    P2SEL0 &= ~IOT_RUN_RED; //IOT_RUN_CPU GPIO operation
    P2SEL1 &= ~IOT_RUN_RED; // IOT RUN_ CPU GPIO operation
    P2OUT &= ~IOT_RUN_RED;  // Initial Value = Low / off
    P2DIR |= IOT_RUN_RED;  // Direction = output

    P2SEL0 &= ~DAC_ENB;     // DAC_ENB GPIO operation
    P2SEL1 &= ~DAC_ENB;     // DAC_ENB GPIO operation
    P2OUT |= DAC_ENB;       // Initial Value = High
    P2DIR |= DAC_ENB;       // Direction = output

    P2SEL0 &= ~LFXOUT;      // LFXOUT Clock operation
    P2SEL1 |= LFXOUT;       // LFXOUT Clock operation

    P2SEL0 &= ~LFXIN;       // LFXIN Clock operation
    P2SEL1 |= LFXIN;        // LFXIN Clock operation

}

//-----------------------------------------------------------

void Init_Port3(void) {
    // Configure Port 3
    //---
    P3OUT = 0x00;           // P2 set Low
    P3DIR = 0x00;           // Set P2 direction to output

    P3SEL0 &= ~TEST_PROBE;    // SLOW_CLK GPIO operation
    P3SEL1 &= ~TEST_PROBE ;    // SLOW_CIK GPIO operation
    P3OUT &= ~TEST_PROBE;     // Initial Value = Low / Off
    P3DIR &= ~TEST_PROBE;     // Direction = Input

    P3SEL0 &= ~OA2O;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~OA2O;   // CHECK BAT GPIO operation
    P3OUT &= ~OA2O;    // Initial Value = Low / Off
    P3DIR &= ~OA2O;      // Direction = Input

    P3SEL0 &= ~OA2N;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~OA2N;   // CHECK BAT GPIO operation
    P3OUT &= ~OA2N;    // Initial Value = Low / Off
    P3DIR &= ~OA2N;      // Direction = Input

    P3SEL0 &= ~OA2P;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~OA2P;   // CHECK BAT GPIO operation
    P3OUT &= ~OA2P;    // Initial Value = Low / Off
    P3DIR &= ~OA2P;      // Direction = Input

    P3SEL0 &= ~SMCLK;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~SMCLK;   // CHECK BAT GPIO operation
    P3OUT &= ~SMCLK;    // Initial Value = Low / Off
    P3DIR &= ~SMCLK;      // Direction = Input

    P3SEL0 &= ~DAC_CNTL;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~DAC_CNTL;   // CHECK BAT GPIO operation
    P3OUT &= ~DAC_CNTL;    // Initial Value = Low / Off
    P3DIR &= ~DAC_CNTL;      // Direction = Input

    P3SEL0 &= ~IOT_LINK_CPU;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~IOT_LINK_CPU;   // CHECK BAT GPIO operation
    P3OUT &= ~IOT_LINK_CPU;    // Initial Value = Low / Off
    P3DIR &= ~IOT_LINK_CPU;      // Direction = Input

    P3SEL0 &= ~IOT_EN_CPU;   // CHECK_ BAT GPIO operation
    P3SEL1 &= ~IOT_EN_CPU;   // CHECK BAT GPIO operation
    P3OUT &= ~IOT_EN_CPU;    // Initial Value = Low / Off
    P3DIR &= ~IOT_EN_CPU;      // Direction = Input
}

//-----------------------------------------------------------

void Init_Port4(void) {

// Configure PORT 4

    P4OUT = 0x00;               // P4 set Low
    P4DIR = 0x00;               // Set P4 direction to output

    P4SEL0 &= ~RESET_LCD;       // RESET_LCD GPIO operation
    P4SEL1 &= ~RESET_LCD;       // RESET_LCD GPIO operation
    P4OUT &= ~RESET_LCD;        // Initial Value = Low / Off
    P4DIR |= RESET_LCD;        // Direction = output

    P4SEL0 &= ~SW1;             // SWI GPIO operation
    P4SEL1 &= ~SW1;             // SW1 GPIO operation
    P4OUT |= SW1;               // Configure pullup resistor
    P4DIR&= ~SW1;               // Direction = input
    P4REN|= SW1;                 // Enable pullup resistor

    P4SEL0 |= UCA1TXD;          // USCI_A1 UART operation
    P4SEL1 &= ~UCA1TXD;         // USCI_A1 UART operation

    P4SEL0 |= UCA1RXD;          // USCI_A1 UART operation
    P4SEL1 &= ~UCA1RXD;         // USCI_A1 UART operation

    P4SEL0 &= ~UCB1_CS_LCD;     // UCB1_CS_LCD GPIO operation
    P4SEL1 &= ~UCB1_CS_LCD;     // UCB1_CS_LCD GPIO operation
    P4OUT |= UCB1_CS_LCD;       // Set SPI_CS_LCD Off [High]
    P4DIR |= UCB1_CS_LCD;       // Set SPI_CS_LCD direction to output

    P4SEL0 |= UCB1CLK;
    P4SEL1 &= ~UCB1CLK;

    P4SEL0 |= UCB1SIMO;         // UCB1SIMO SPI BUS operation
    P4SEL1 &= ~UCB1SIMO;        // UCB1SIMO SPI BUS operation

    P4SEL0 |= UCB1SOMI;         // UCB1SOMI SPI BUS operation
    P4SEL1 &= ~UCB1SOMI;        // UCB1SOMI SPI BUS operation

}
//-----------------------------------------------------------
void Init_Port5(void){

    //Configure Port 5
    P5OUT = 0x00;
    P5DIR = 0x00;

    P5SEL0 &= ~V_BAT;
    P5SEL1 &= ~V_BAT;
    P5OUT  &= ~V_BAT;
    P5DIR  &= ~V_BAT;

    P5SEL0 &= ~V_5;
    P5SEL1 &= ~V_5;
    P5OUT  &= ~V_5;
    P5DIR  &= ~V_5;

    P5SEL0 &= ~V_DAC;
    P5SEL1 &= ~V_DAC;
    P5OUT  &= ~V_DAC;
    P5DIR  &= ~V_DAC;

    P5SEL0 &= ~V_3_3;
    P5SEL1 &= ~V_3_3;
    P5OUT  &= ~V_3_3;
    P5DIR  &= ~V_3_3;

    P5SEL0 &= ~IOT_BOOT_CPU;
    P5SEL1 &= ~IOT_BOOT_CPU;
    P5OUT  &= ~IOT_BOOT_CPU;
    P5DIR  &= ~IOT_BOOT_CPU;

}
//-----------------------------------------------------------
//-----------------------------------------------------------
// UPDATED Init_Port6 Function with PWM Configuration
// Replace your existing Init_Port6() function with this
//-----------------------------------------------------------

void Init_Port6(void){

   //Configure Port 6
  P6OUT = 0x00;
  P6DIR = 0x00;

  P6SEL0 &= ~LCD_BACKLITE;
  P6SEL1 &= ~LCD_BACKLITE;
  P6OUT |= LCD_BACKLITE;
  P6DIR |= LCD_BACKLITE;

  // RIGHT FORWARD - P6.1 - Configure for Timer B3.2 PWM
  P6SEL0 |= R_FORWARD;       // Set SEL0 for Timer function
  P6SEL1 &= ~R_FORWARD;      // Clear SEL1 for Timer function
  P6OUT  &= ~R_FORWARD;      // Initial output LOW
  P6DIR  |= R_FORWARD;       // Direction = output

  // LEFT FORWARD - P6.2 - Configure for Timer B3.3 PWM
  P6SEL0 |= L_FORWARD;       // Set SEL0 for Timer function
  P6SEL1 &= ~L_FORWARD;      // Clear SEL1 for Timer function
  P6OUT  &= ~L_FORWARD;      // Initial output LOW
  P6DIR  |= L_FORWARD;       // Direction = output

  // RIGHT REVERSE - P6.3 - Configure for Timer B3.4 PWM
  P6SEL0 |= R_REVERSE;       // Set SEL0 for Timer function
  P6SEL1 &= ~R_REVERSE;      // Clear SEL1 for Timer function
  P6OUT  &= ~R_REVERSE;      // Initial output LOW
  P6DIR  |= R_REVERSE;       // Direction = output

  // LEFT REVERSE - P6.4 - Configure for Timer B3.5 PWM
  P6SEL0 |= L_REVERSE;       // Set SEL0 for Timer function
  P6SEL1 &= ~L_REVERSE;      // Clear SEL1 for Timer function
  P6OUT  &= ~L_REVERSE;      // Initial output LOW
  P6DIR  |= L_REVERSE;       // Direction = output

  P6SEL0 &= ~P6_5;
  P6SEL1 &= ~P6_5;
  P6OUT  &= ~P6_5;
  P6DIR  &= ~P6_5;

  P6SEL0 &= ~GRN_LED;
  P6SEL1 &= ~GRN_LED;
  P6OUT  &= ~GRN_LED;
  P6DIR  |= GRN_LED;

  //----------------------------------------------------------

}
