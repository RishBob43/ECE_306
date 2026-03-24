/*------------------------------------------------------------------------------
 * File:        ports.c
 * Author:      [Student Name]
 * Date:        [Date]
 * Revised:     [Date]
 * Compiler:    TI Code Composer Studio / msp430-gcc
 * Target:      MSP430FR2355
 *
 * Description: GPIO port initialization for all six ports on the MSP430FR2355.
 *
 *              Key changes from previous revision:
 *                - SW1 (P4.1) and SW2 (P2.3) are now fully configured as
 *                  falling-edge interrupt inputs in Init_Port4() and
 *                  Init_Port2() respectively.  Both interrupts start ENABLED.
 *                - LCD_BACKLITE (P6.0) output starts HIGH (backlight on).
 *
 *              Init_Port3(char smclk)
 *                USE_GPIO  (0x00)  P3.4 = General Purpose I/O input
 *                USE_SMCLK (0x01)  P3.4 = SMCLK output
 *                  MSP430FR2355 datasheet Table 6-22:
 *                    P3SEL1=0, P3SEL0=1 -> primary function = SMCLK
 *                  SMCLK = 8 MHz (clocks.c: CSCTL5 DIVS__1)
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

/*------------------------------------------------------------------------------
 * Init_Ports
 * Description: Calls each port's individual initializer in order.
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Ports(void){
    Init_Port1();
    Init_Port2();
    Init_Port3(USE_GPIO);
    Init_Port4();
    Init_Port5();
    Init_Port6();
}


/*==============================================================================
 * Init_Port1
 *
 * P1.0  RED_LED      – GPIO output, init LOW
 * P1.1  V_A1_SEEED   – analog input  (SELC disables digital buffer)
 * P1.2  V_DETECT_L   – analog input  (SELC) – ADC channel A2
 * P1.3  V_DETECT_R   – analog input  (SELC) – ADC channel A3
 * P1.4  V_A4_SEEED   – analog input  (SELC) – ADC channel A4
 * P1.5  V_THUMB      – analog input  (SELC) – ADC channel A5
 * P1.6  UCA0RXD      – UART primary function (SEL0=1, SEL1=0)
 * P1.7  UCA0TXD      – UART primary function (SEL0=1, SEL1=0)
 *==============================================================================*/
void Init_Port1(void){

    P1OUT = RESET_STATE;
    P1DIR = RESET_STATE;

    /*--------------------------------------------------------------------------
     * P1.0  RED_LED – digital GPIO output, initial state LOW
     *------------------------------------------------------------------------*/
    P1SEL0 &= ~RED_LED;
    P1SEL1 &= ~RED_LED;
    P1OUT  &= ~RED_LED;           /* Start LED off                             */
    P1DIR  |=  RED_LED;

    /*--------------------------------------------------------------------------
     * P1.1  V_A1_SEEED – analog input
     *   P1SELC sets both SEL0 and SEL1 high, disabling the digital buffer.
     *------------------------------------------------------------------------*/
    P1SELC |= V_A1_SEEED;

    /*--------------------------------------------------------------------------
     * P1.2  V_DETECT_L – analog input (ADC A2, left IR detector)
     *------------------------------------------------------------------------*/
    P1SELC |= V_DETECT_L;

    /*--------------------------------------------------------------------------
     * P1.3  V_DETECT_R – analog input (ADC A3, right IR detector)
     *------------------------------------------------------------------------*/
    P1SELC |= V_DETECT_R;

    /*--------------------------------------------------------------------------
     * P1.4  V_A4_SEEED – analog input
     *------------------------------------------------------------------------*/
    P1SELC |= V_A4_SEEED;

    /*--------------------------------------------------------------------------
     * P1.5  V_THUMB – analog input (ADC A5, thumbwheel)
     *------------------------------------------------------------------------*/
    P1SELC |= V_THUMB;

    /*--------------------------------------------------------------------------
     * P1.6  UCA0RXD – UART primary function (SEL0=1, SEL1=0)
     *------------------------------------------------------------------------*/
    P1SEL0 |=  UCA0RXD;
    P1SEL1 &= ~UCA0RXD;

    /*--------------------------------------------------------------------------
     * P1.7  UCA0TXD – UART primary function (SEL0=1, SEL1=0)
     *------------------------------------------------------------------------*/
    P1SEL0 |=  UCA0TXD;
    P1SEL1 &= ~UCA0TXD;
}

/*------------------------------------------------------------------------------
 * Init_Port2
 * Description: Configures all Port 2 pins.
 *   P2.0  SLOW_CLK     - GPIO output
 *   P2.1  CHECK_BAT    - GPIO output
 *   P2.2  IR_LED       - GPIO output
 *   P2.3  SW2          - GPIO input, pull-up, falling-edge interrupt ENABLED
 *   P2.4  IOT_RUN_RED  - GPIO output
 *   P2.5  DAC_ENB      - GPIO output, init high
 *   P2.6  LFXOUT       - Crystal function (SEL1=1)
 *   P2.7  LFXIN        - Crystal function (SEL1=1)
 *
 * SW2 interrupt configuration:
 *   Pull-up resistor enabled (active-low switch).
 *   P2IES |= SW2  -> falling edge triggers interrupt (button press = high->low).
 *   P2IE  |= SW2  -> interrupt enabled at startup.
 *   P2IFG cleared before enable to prevent spurious first interrupt.
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Port2(void){
    P2OUT = RESET_STATE;
    P2DIR = RESET_STATE;

    /* P2.0  SLOW_CLK – output */
    P2SEL0 &= ~SLOW_CLK;
    P2SEL1 &= ~SLOW_CLK;
    P2OUT  &= ~SLOW_CLK;
    P2DIR  |=  SLOW_CLK;

    /* P2.1  CHECK_BAT – output */
    P2SEL0 &= ~CHECK_BAT;
    P2SEL1 &= ~CHECK_BAT;
    P2OUT  &= ~CHECK_BAT;
    P2DIR  |=  CHECK_BAT;

    /* P2.2  IR_LED – output */
    P2SEL0 &= ~IR_LED;
    P2SEL1 &= ~IR_LED;
    P2OUT  &= ~IR_LED;
    P2DIR  |=  IR_LED;

    /* P2.3  SW2 – input, pull-up, falling-edge interrupt */
    P2SEL0 &= ~SW2;
    P2SEL1 &= ~SW2;
    P2OUT  |=  SW2;           /* Pull-up (active-low switch)                    */
    P2DIR  &= ~SW2;           /* Input direction                                */
    P2REN  |=  SW2;           /* Resistor enable                                */
    P2IES  |=  SW2;           /* Interrupt on falling edge (high -> low = press) */
    P2IFG  &= ~SW2;           /* Clear any pending flag before enabling         */
    P2IE   |=  SW2;           /* Enable SW2 interrupt                           */

    /* P2.4  IOT_RUN_RED – output */
    P2SEL0 &= ~IOT_RUN_RED;
    P2SEL1 &= ~IOT_RUN_RED;
    P2OUT  &= ~IOT_RUN_RED;
    P2DIR  |=  IOT_RUN_RED;

    /* P2.5  DAC_ENB – output, init high (de-assert) */
    P2SEL0 &= ~DAC_ENB;
    P2SEL1 &= ~DAC_ENB;
    P2OUT  |=  DAC_ENB;
    P2DIR  |=  DAC_ENB;

    /* P2.6  LFXOUT – crystal function */
    P2SEL0 &= ~LFXOUT;
    P2SEL1 |=  LFXOUT;

    /* P2.7  LFXIN – crystal function */
    P2SEL0 &= ~LFXIN;
    P2SEL1 |=  LFXIN;
}

/*------------------------------------------------------------------------------
 * Init_Port3
 * Description: Configures Port 3 pins.
 *   P3.0  TEST_PROBE  - GPIO output
 *   P3.1  OA2O        - GPIO input
 *   P3.2  OA2N        - GPIO input
 *   P3.3  OA2P        - GPIO input
 *   P3.4  SMCLK/GPIO  - controlled by 'smclk' parameter
 *   P3.5  DAC_CNTL    - GPIO input
 *   P3.6  IOT_LINK_CPU - GPIO input
 *   P3.7  IOT_EN_CPU   - GPIO input
 *
 * Parameter smclk:
 *   USE_GPIO  (0x00) – P3.4 is GPIO input (SEL1=0, SEL0=0)
 *   USE_SMCLK (0x01) – P3.4 outputs SMCLK (SEL1=0, SEL0=1, DIR=output)
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Port3(char smclk){
    P3OUT = RESET_STATE;
    P3DIR = RESET_STATE;

    /* P3.0  TEST_PROBE – GPIO output */
    P3SEL0 &= ~TEST_PROBE;
    P3SEL1 &= ~TEST_PROBE;
    P3OUT  &= ~TEST_PROBE;
    P3DIR  |=  TEST_PROBE;

    /* P3.1  OA2O – GPIO input */
    P3SEL0 &= ~OA2O;
    P3SEL1 &= ~OA2O;
    P3OUT  &= ~OA2O;
    P3DIR  &= ~OA2O;

    /* P3.2  OA2N – GPIO input */
    P3SEL0 &= ~OA2N;
    P3SEL1 &= ~OA2N;
    P3OUT  &= ~OA2N;
    P3DIR  &= ~OA2N;

    /* P3.3  OA2P – GPIO input */
    P3SEL0 &= ~OA2P;
    P3SEL1 &= ~OA2P;
    P3OUT  &= ~OA2P;
    P3DIR  &= ~OA2P;

    /* P3.4  SMCLK / GPIO */
    if(smclk == USE_SMCLK){
        P3SEL1 &= ~SMCLK;     /* SEL1=0                                        */
        P3SEL0 |=  SMCLK;     /* SEL0=1 -> primary function = SMCLK output     */
        P3DIR  |=  SMCLK;     /* Must be configured as output                  */
    } else {
        P3SEL0 &= ~SMCLK;     /* SEL0=0 -> GPIO                                */
        P3SEL1 &= ~SMCLK;     /* SEL1=0                                        */
        P3OUT  &= ~SMCLK;
        P3DIR  &= ~SMCLK;     /* Input                                          */
    }

    /* P3.5  DAC_CNTL – GPIO input */
    P3SEL0 &= ~DAC_CNTL;
    P3SEL1 &= ~DAC_CNTL;
    P3OUT  &= ~DAC_CNTL;
    P3DIR  &= ~DAC_CNTL;

    /* P3.6  IOT_LINK_CPU – GPIO input */
    P3SEL0 &= ~IOT_LINK_CPU;
    P3SEL1 &= ~IOT_LINK_CPU;
    P3OUT  &= ~IOT_LINK_CPU;
    P3DIR  &= ~IOT_LINK_CPU;

    /* P3.7  IOT_EN_CPU – GPIO input */
    P3SEL0 &= ~IOT_EN_CPU;
    P3SEL1 &= ~IOT_EN_CPU;
    P3OUT  &= ~IOT_EN_CPU;
    P3DIR  &= ~IOT_EN_CPU;
}

/*------------------------------------------------------------------------------
 * Init_Port4
 * Description: Configures Port 4 pins.
 *   P4.0  RESET_LCD   - GPIO output
 *   P4.1  SW1         - input, pull-up, falling-edge interrupt ENABLED
 *   P4.2  UCA1RXD     - UART primary function
 *   P4.3  UCA1TXD     - UART primary function
 *   P4.4  UCB1_CS_LCD - GPIO output, init high (CS de-asserted)
 *   P4.5  UCB1CLK     - SPI clock primary function
 *   P4.6  UCB1SIMO    - SPI MOSI primary function
 *   P4.7  UCB1SOMI    - SPI MISO primary function
 *
 * SW1 interrupt configuration:
 *   Pull-up resistor enabled (active-low switch).
 *   P4IES |= SW1  -> falling edge triggers interrupt (button press = high->low).
 *   P4IE  |= SW1  -> interrupt enabled at startup.
 *   P4IFG cleared before enable to prevent spurious first interrupt.
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Port4(void){
    P4OUT = RESET_STATE;
    P4DIR = RESET_STATE;

    /* P4.0  RESET_LCD – output */
    P4SEL0 &= ~RESET_LCD;
    P4SEL1 &= ~RESET_LCD;
    P4OUT  &= ~RESET_LCD;
    P4DIR  |=  RESET_LCD;

    /* P4.1  SW1 – input, pull-up, falling-edge interrupt */
    P4SEL0 &= ~SW1;
    P4SEL1 &= ~SW1;
    P4OUT  |=  SW1;           /* Pull-up (active-low switch)                    */
    P4DIR  &= ~SW1;           /* Input direction                                */
    P4REN  |=  SW1;           /* Resistor enable                                */
    P4IES  |=  SW1;           /* Interrupt on falling edge (high -> low = press) */
    P4IFG  &= ~SW1;           /* Clear any pending flag before enabling         */
    P4IE   |=  SW1;           /* Enable SW1 interrupt                           */

    /* P4.2  UCA1RXD – UART primary function (SEL0=1, SEL1=0) */
    P4SEL0 |=  UCA1RXD;
    P4SEL1 &= ~UCA1RXD;

    /* P4.3  UCA1TXD – UART primary function */
    P4SEL0 |=  UCA1TXD;
    P4SEL1 &= ~UCA1TXD;

    /* P4.4  UCB1_CS_LCD – GPIO output, init high (CS de-asserted) */
    P4SEL0 &= ~UCB1_CS_LCD;
    P4SEL1 &= ~UCB1_CS_LCD;
    P4OUT  |=  UCB1_CS_LCD;
    P4DIR  |=  UCB1_CS_LCD;

    /* P4.5  UCB1CLK – SPI clock */
    P4SEL0 |=  UCB1CLK;
    P4SEL1 &= ~UCB1CLK;

    /* P4.6  UCB1SIMO – SPI MOSI */
    P4SEL0 |=  UCB1SIMO;
    P4SEL1 &= ~UCB1SIMO;

    /* P4.7  UCB1SOMI – SPI MISO */
    P4SEL0 |=  UCB1SOMI;
    P4SEL1 &= ~UCB1SOMI;
}

/*==============================================================================
 * Init_Port5  (Project 6 version)
 *
 * P5.0  V_BAT        – analog input  (SELC) – ADC A8
 * P5.1  V_5          – analog input  (SELC) – ADC A9
 * P5.2  V_DAC        – analog input  (SELC) – ADC A10
 * P5.3  V_3_3        – analog input  (SELC) – ADC A11
 * P5.4  IOT_BOOT_CPU – GPIO output, init HIGH (inactive)
 *==============================================================================*/
void Init_Port5(void){

    P5OUT = RESET_STATE;
    P5DIR = RESET_STATE;

    /*--------------------------------------------------------------------------
     * Analog ADC input pins – disable digital buffers with SELC
     *------------------------------------------------------------------------*/
    P5SELC |= V_BAT;
    P5SELC |= V_5;
    P5SELC |= V_DAC;
    P5SELC |= V_3_3;

    /*--------------------------------------------------------------------------
     * P5.4  IOT_BOOT_CPU – GPIO output, init HIGH (de-assert boot signal)
     *------------------------------------------------------------------------*/
    P5SEL0 &= ~IOT_BOOT_CPU;
    P5SEL1 &= ~IOT_BOOT_CPU;
    P5OUT  |=  IOT_BOOT_CPU;      /* High = inactive                           */
    P5DIR  |=  IOT_BOOT_CPU;
}




/*------------------------------------------------------------------------------
 * Init_Port6
 * Description: Configures Port 6 pins.
 *   P6.0  LCD_BACKLITE - GPIO output, init HIGH (backlight on at startup)
 *   P6.1  R_FORWARD    - Timer B3.2 PWM output
 *   P6.2  L_FORWARD    - Timer B3.3 PWM output
 *   P6.3  R_REVERSE    - Timer B3.4 PWM output
 *   P6.4  L_REVERSE    - Timer B3.5 PWM output
 *   P6.5  P6_5         - unused, GPIO input
 *   P6.6  GRN_LED      - GPIO output, init low
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Port6(void){
    P6OUT = RESET_STATE;
    P6DIR = RESET_STATE;

    /* P6.0  LCD_BACKLITE – output, start HIGH (backlight on) */
    P6SEL0 &= ~LCD_BACKLITE;
    P6SEL1 &= ~LCD_BACKLITE;
    P6OUT  |=  LCD_BACKLITE;  /* Initial state: backlight ON                   */
    P6DIR  |=  LCD_BACKLITE;

    /* P6.1  R_FORWARD – GPIO output */
    P6SEL0 |= R_FORWARD;   // was: P6SEL0 |= R_FORWARD
    P6SEL1 &= ~R_FORWARD;
    P6OUT  &= ~R_FORWARD;
    P6DIR  |=  R_FORWARD;

    /* P6.2  L_FORWARD – GPIO output */
    P6SEL0 |= L_FORWARD;   // was: P6SEL0 |= L_FORWARD
    P6SEL1 &= ~L_FORWARD;
    P6OUT  &= ~L_FORWARD;
    P6DIR  |=  L_FORWARD;

    /* P6.3  R_REVERSE – GPIO output */
    P6SEL0 |= R_REVERSE;   // was: P6SEL0 |= R_REVERSE
    P6SEL1 &= ~R_REVERSE;
    P6OUT  &= ~R_REVERSE;
    P6DIR  |=  R_REVERSE;

    /* P6.4  L_REVERSE – GPIO output */
    P6SEL0 |= L_REVERSE;   // was: P6SEL0 |= L_REVERSE
    P6SEL1 &= ~L_REVERSE;
    P6OUT  &= ~L_REVERSE;
    P6DIR  |=  L_REVERSE;

    /* P6.5  P6_5 – unused input */
    P6SEL0 &= ~P6_5;
    P6SEL1 &= ~P6_5;
    P6OUT  &= ~P6_5;
    P6DIR  &= ~P6_5;

    /* P6.6  GRN_LED – output, init low */
    P6SEL0 &= ~GRN_LED;
    P6SEL1 &= ~GRN_LED;
    P6OUT  &= ~GRN_LED;
    P6DIR  |=  GRN_LED;
}
