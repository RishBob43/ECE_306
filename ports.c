//------------------------------------------------------------------------------
// ports.c  –  Port Initialization
//
// Init_Port3(char smclk)
//   USE_GPIO  (0x00)  P3.4 = General Purpose I/O input (default / safe)
//   USE_SMCLK (0x01)  P3.4 = SMCLK output
//                     MSP430FR2355 datasheet Table 6-22:
//                       P3SEL1 = 0, P3SEL0 = 1  ->  primary function = SMCLK
//                     SMCLK must already be divided to 500 kHz in clocks.c
//                     (DIVS = /16, 8 MHz / 16 = 500 kHz) before this is called.
//------------------------------------------------------------------------------

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

//------------------------------------------------------------------------------
void Init_Ports(void){
    Init_Port1();
    Init_Port2();
    Init_Port3(USE_GPIO);   // Start as GPIO; SW1 will switch to USE_SMCLK
    Init_Port4();
    Init_Port5();
    Init_Port6();
}

//------------------------------------------------------------------------------
void Init_Port1(void){
    P1OUT = 0x00;
    P1DIR = 0x00;

    // 1.0  RED_LED  – output
    P1SEL0 &= ~RED_LED;
    P1SEL1 &= ~RED_LED;
    P1OUT  &= ~RED_LED;
    P1DIR  |=  RED_LED;

    // 1.1  V_A1_SEEED – input
    P1SEL0 &= ~V_A1_SEEED;
    P1SEL1 &= ~V_A1_SEEED;
    P1OUT  &= ~V_A1_SEEED;
    P1DIR  &= ~V_A1_SEEED;

    // 1.2  V_DETECT_L – input
    P1SEL0 &= ~V_DETECT_L;
    P1SEL1 &= ~V_DETECT_L;
    P1OUT  &= ~V_DETECT_L;
    P1DIR  &= ~V_DETECT_L;

    // 1.3  V_DETECT_R – input
    P1SEL0 &= ~V_DETECT_R;
    P1SEL1 &= ~V_DETECT_R;
    P1OUT  &= ~V_DETECT_R;
    P1DIR  &= ~V_DETECT_R;

    // 1.4  V_A4_SEEED – input
    P1SEL0 &= ~V_A4_SEEED;
    P1SEL1 &= ~V_A4_SEEED;
    P1OUT  &= ~V_A4_SEEED;
    P1DIR  &= ~V_A4_SEEED;

    // 1.5  V_THUMB – input
    P1SEL0 &= ~V_THUMB;
    P1SEL1 &= ~V_THUMB;
    P1OUT  &= ~V_THUMB;
    P1DIR  &= ~V_THUMB;

    // 1.6  UCA0RXD – input
    P1SEL0 &= ~UCA0RXD;
    P1SEL1 &= ~UCA0RXD;
    P1OUT  &= ~UCA0RXD;
    P1DIR  &= ~UCA0RXD;

    // 1.7  UCA0TXD – input
    P1SEL0 &= ~UCA0TXD;
    P1SEL1 &= ~UCA0TXD;
    P1OUT  &= ~UCA0TXD;
    P1DIR  &= ~UCA0TXD;
}

//------------------------------------------------------------------------------
void Init_Port2(void){
    P2OUT = 0x00;
    P2DIR = 0x00;

    // 2.0  SLOW_CLK – output
    P2SEL0 &= ~SLOW_CLK;
    P2SEL1 &= ~SLOW_CLK;
    P2OUT  &= ~SLOW_CLK;
    P2DIR  |=  SLOW_CLK;

    // 2.1  CHECK_BAT – output
    P2SEL0 &= ~CHECK_BAT;
    P2SEL1 &= ~CHECK_BAT;
    P2OUT  &= ~CHECK_BAT;
    P2DIR  |=  CHECK_BAT;

    // 2.2  IR_LED – output
    P2SEL0 &= ~IR_LED;
    P2SEL1 &= ~IR_LED;
    P2OUT  &= ~IR_LED;
    P2DIR  |=  IR_LED;

    // 2.3  SW2 – input with pull-up
    P2SEL0 &= ~SW2;
    P2SEL1 &= ~SW2;
    P2OUT  |=  SW2;    // pull-up
    P2DIR  &= ~SW2;
    P2REN  |=  SW2;

    // 2.4  IOT_RUN_RED – output
    P2SEL0 &= ~IOT_RUN_RED;
    P2SEL1 &= ~IOT_RUN_RED;
    P2OUT  &= ~IOT_RUN_RED;
    P2DIR  |=  IOT_RUN_RED;

    // 2.5  DAC_ENB – output, init high
    P2SEL0 &= ~DAC_ENB;
    P2SEL1 &= ~DAC_ENB;
    P2OUT  |=  DAC_ENB;
    P2DIR  |=  DAC_ENB;

    // 2.6  LFXOUT – clock function
    P2SEL0 &= ~LFXOUT;
    P2SEL1 |=  LFXOUT;

    // 2.7  LFXIN – clock function
    P2SEL0 &= ~LFXIN;
    P2SEL1 |=  LFXIN;
}

//------------------------------------------------------------------------------
// Init_Port3
//
// Parameter smclk:
//   USE_GPIO  (0x00) – P3.4 is a GPIO input  (SEL1=0, SEL0=0, DIR=input)
//   USE_SMCLK (0x01) – P3.4 outputs SMCLK    (SEL1=0, SEL0=1, DIR=output)
//
// All other Port 3 pins are always configured the same way regardless of
// the smclk argument.
//
// IMPORTANT: clocks.c must set DIVS=__16 so SMCLK = 8 MHz / 16 = 500 kHz
// before Init_Port3(USE_SMCLK) is called.  You cannot output 8 MHz on P3.4.
//------------------------------------------------------------------------------
void Init_Port3(char smclk){
    P3OUT = 0x00;
    P3DIR = 0x00;

    // 3.0  TEST_PROBE – GPIO output
    P3SEL0 &= ~TEST_PROBE;
    P3SEL1 &= ~TEST_PROBE;
    P3OUT  &= ~TEST_PROBE;
    P3DIR  |=  TEST_PROBE;   // output (used as test toggle in main loop)

    // 3.1  OA2O – GPIO input
    P3SEL0 &= ~OA2O;
    P3SEL1 &= ~OA2O;
    P3OUT  &= ~OA2O;
    P3DIR  &= ~OA2O;

    // 3.2  OA2N – GPIO input
    P3SEL0 &= ~OA2N;
    P3SEL1 &= ~OA2N;
    P3OUT  &= ~OA2N;
    P3DIR  &= ~OA2N;

    // 3.3  OA2P – GPIO input
    P3SEL0 &= ~OA2P;
    P3SEL1 &= ~OA2P;
    P3OUT  &= ~OA2P;
    P3DIR  &= ~OA2P;

    //--------------------------------------------------------------------------
    // 3.4  SMCLK / GPIO  ← controlled by 'smclk' argument
    //--------------------------------------------------------------------------
    if(smclk == USE_SMCLK){
        // Route SMCLK to P3.4 pin
        // MSP430FR2355 datasheet: SEL1=0, SEL0=1 → primary function = SMCLK out
        P3SEL1 &= ~SMCLK;   // SEL1 = 0
        P3SEL0 |=  SMCLK;   // SEL0 = 1  → SMCLK output function
        P3DIR  |=  SMCLK;   // Must be output
    } else {
        // USE_GPIO: plain digital I/O, input, driven low
        P3SEL0 &= ~SMCLK;   // SEL0 = 0
        P3SEL1 &= ~SMCLK;   // SEL1 = 0  → GPIO
        P3OUT  &= ~SMCLK;   // Low
        P3DIR  &= ~SMCLK;   // Input
    }
    //--------------------------------------------------------------------------

    // 3.5  DAC_CNTL – GPIO input
    P3SEL0 &= ~DAC_CNTL;
    P3SEL1 &= ~DAC_CNTL;
    P3OUT  &= ~DAC_CNTL;
    P3DIR  &= ~DAC_CNTL;

    // 3.6  IOT_LINK_CPU – GPIO input
    P3SEL0 &= ~IOT_LINK_CPU;
    P3SEL1 &= ~IOT_LINK_CPU;
    P3OUT  &= ~IOT_LINK_CPU;
    P3DIR  &= ~IOT_LINK_CPU;

    // 3.7  IOT_EN_CPU – GPIO input
    P3SEL0 &= ~IOT_EN_CPU;
    P3SEL1 &= ~IOT_EN_CPU;
    P3OUT  &= ~IOT_EN_CPU;
    P3DIR  &= ~IOT_EN_CPU;
}

//------------------------------------------------------------------------------
void Init_Port4(void){
    P4OUT = 0x00;
    P4DIR = 0x00;

    // 4.0  RESET_LCD – output
    P4SEL0 &= ~RESET_LCD;
    P4SEL1 &= ~RESET_LCD;
    P4OUT  &= ~RESET_LCD;
    P4DIR  |=  RESET_LCD;

    // 4.1  SW1 – input with pull-up
    P4SEL0 &= ~SW1;
    P4SEL1 &= ~SW1;
    P4OUT  |=  SW1;
    P4DIR  &= ~SW1;
    P4REN  |=  SW1;

    // 4.2  UCA1RXD – UART function
    P4SEL0 |=  UCA1RXD;
    P4SEL1 &= ~UCA1RXD;

    // 4.3  UCA1TXD – UART function
    P4SEL0 |=  UCA1TXD;
    P4SEL1 &= ~UCA1TXD;

    // 4.4  UCB1_CS_LCD – GPIO output, init high (de-assert CS)
    P4SEL0 &= ~UCB1_CS_LCD;
    P4SEL1 &= ~UCB1_CS_LCD;
    P4OUT  |=  UCB1_CS_LCD;
    P4DIR  |=  UCB1_CS_LCD;

    // 4.5  UCB1CLK – SPI clock
    P4SEL0 |=  UCB1CLK;
    P4SEL1 &= ~UCB1CLK;

    // 4.6  UCB1SIMO – SPI
    P4SEL0 |=  UCB1SIMO;
    P4SEL1 &= ~UCB1SIMO;

    // 4.7  UCB1SOMI – SPI
    P4SEL0 |=  UCB1SOMI;
    P4SEL1 &= ~UCB1SOMI;
}

//------------------------------------------------------------------------------
void Init_Port5(void){
    P5OUT = 0x00;
    P5DIR = 0x00;

    P5SEL0 &= ~V_BAT;   P5SEL1 &= ~V_BAT;   P5OUT &= ~V_BAT;   P5DIR &= ~V_BAT;
    P5SEL0 &= ~V_5;     P5SEL1 &= ~V_5;     P5OUT &= ~V_5;     P5DIR &= ~V_5;
    P5SEL0 &= ~V_DAC;   P5SEL1 &= ~V_DAC;   P5OUT &= ~V_DAC;   P5DIR &= ~V_DAC;
    P5SEL0 &= ~V_3_3;   P5SEL1 &= ~V_3_3;   P5OUT &= ~V_3_3;   P5DIR &= ~V_3_3;

    P5SEL0 &= ~IOT_BOOT_CPU;
    P5SEL1 &= ~IOT_BOOT_CPU;
    P5OUT  &= ~IOT_BOOT_CPU;
    P5DIR  &= ~IOT_BOOT_CPU;
}

//------------------------------------------------------------------------------
void Init_Port6(void){
    P6OUT = 0x00;
    P6DIR = 0x00;

    // 6.0  LCD_BACKLITE – output, init high (backlight on)
    P6SEL0 &= ~LCD_BACKLITE;
    P6SEL1 &= ~LCD_BACKLITE;
    P6OUT  |=  LCD_BACKLITE;
    P6DIR  |=  LCD_BACKLITE;

    // 6.1  R_FORWARD – Timer B3.2 PWM
    P6SEL0 |=  R_FORWARD;
    P6SEL1 &= ~R_FORWARD;
    P6OUT  &= ~R_FORWARD;
    P6DIR  |=  R_FORWARD;

    // 6.2  L_FORWARD – Timer B3.3 PWM
    P6SEL0 |=  L_FORWARD;
    P6SEL1 &= ~L_FORWARD;
    P6OUT  &= ~L_FORWARD;
    P6DIR  |=  L_FORWARD;

    // 6.3  R_REVERSE – Timer B3.4 PWM
    P6SEL0 |=  R_REVERSE;
    P6SEL1 &= ~R_REVERSE;
    P6OUT  &= ~R_REVERSE;
    P6DIR  |=  R_REVERSE;

    // 6.4  L_REVERSE – Timer B3.5 PWM
    P6SEL0 |=  L_REVERSE;
    P6SEL1 &= ~L_REVERSE;
    P6OUT  &= ~L_REVERSE;
    P6DIR  |=  L_REVERSE;

    // 6.5  P6_5 – unused input
    P6SEL0 &= ~P6_5;
    P6SEL1 &= ~P6_5;
    P6OUT  &= ~P6_5;
    P6DIR  &= ~P6_5;

    // 6.6  GRN_LED – output
    P6SEL0 &= ~GRN_LED;
    P6SEL1 &= ~GRN_LED;
    P6OUT  &= ~GRN_LED;
    P6DIR  |=  GRN_LED;
}
