/*------------------------------------------------------------------------------
 * File:        timerB0.h
 * Author:      [Student Name]
 * Date:        [Date]
 * Revised:     [Date]
 * Compiler:    TI Code Composer Studio / msp430-gcc
 * Target:      MSP430FR2355
 *
 * Description: Header file for Timer B0 configuration and all CCR-based
 *              interrupt service routines.
 *
 *              CCR0 - LCD backlight blink / display update gate (200 ms)
 *              CCR1 - Switch 1 (SW1, P4.1) debounce timer
 *              CCR2 - Switch 2 (SW2, P2.3) debounce timer
 *
 *              Timer clock derivation (CCR0 / CCR1 / CCR2 all share TB0):
 *                SMCLK          = 8,000,000 Hz  (DIVS__1 in clocks.c)
 *                ID__8          = divide by 8  ->  1,000,000 Hz
 *                TBIDEX__8      = divide by 8  ->    125,000 Hz  (1 tick = 8 us)
 *                Combined /64   = 125 kHz effective timer clock
 *
 *              CCR0  200 ms backlight period:
 *                8,000,000 / 8 / 8 / 5 = 25,000 ticks
 *                -> blink toggles every 200 ms = 2.5 blinks per second
 *
 *              CCR1 / CCR2  debounce slice = 100 ms per interrupt:
 *                8,000,000 / 8 / 8 / 10 = 12,500 ticks
 *                Threshold = 10 slices  ->  1,000 ms total debounce
 *------------------------------------------------------------------------------*/

#ifndef TIMERB0_H_
#define TIMERB0_H_

#include <msp430.h>
#include "macros.h"

/*------------------------------------------------------------------------------
 * Timer B0 Clock Source and Divider
 *   SMCLK  = 8,000,000 Hz  (clocks.c: DIVS__1, so SMCLK = MCLK = 8 MHz)
 *   ID__8  = first  stage /8  ->  1,000,000 Hz
 *   TBIDEX__8 = second stage /8  ->    125,000 Hz  (1 tick = 8 us)
 *   Total divisor = /64
 *------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 * CCR0 – Backlight blink and update_display gate
 *   Interrupt period: 200 ms
 *   8,000,000 / 8 / 8 / 5  =  25,000 ticks
 *------------------------------------------------------------------------------*/
#define TB0CCR0_INTERVAL        (25000)            /* 200 ms compare count       */

/*------------------------------------------------------------------------------
 * CCR1 – Switch 1 (SW1, P4.1) debounce slice timer
 * CCR2 – Switch 2 (SW2, P2.3) debounce slice timer
 *   Slice period: 100 ms
 *   8,000,000 / 8 / 8 / 10  =  12,500 ticks per slice
 *   Threshold:  DEBOUNCE_THRESHOLD slices * 100 ms = 1,000 ms total
 *------------------------------------------------------------------------------*/
#define TB0CCR1_DEBOUNCE_SLICE  (12500)            /* 100 ms per CCR1 interrupt  */
#define TB0CCR2_DEBOUNCE_SLICE  (12500)            /* 100 ms per CCR2 interrupt  */
#define DEBOUNCE_THRESHOLD      (10u)              /* 10 slices * 100 ms = 1 s   */

/*------------------------------------------------------------------------------
 * CCR Control Bits
 *------------------------------------------------------------------------------*/
#define CCR_INTERRUPT_ENABLE    (CCIE)             /* Enable  CCR interrupt      */
#define CCR_INTERRUPT_DISABLE   (~CCIE)            /* Mask to disable CCR int    */
#define CCR_INTERRUPT_FLAG_CLR  (~CCIFG)           /* Mask to clear CCR IFG      */

/*------------------------------------------------------------------------------
 * Debounce-in-progress flag bits (stored in a single char variable)
 *   Bit 0 = SW1 debounce active
 *   Bit 1 = SW2 debounce active
 *------------------------------------------------------------------------------*/
#define SW1_DEBOUNCE_ACTIVE_BIT (0x01)
#define SW2_DEBOUNCE_ACTIVE_BIT (0x02)
#define NO_DEBOUNCE_ACTIVE      (0x00)

/*------------------------------------------------------------------------------
 * Function Prototypes
 *------------------------------------------------------------------------------*/
void Init_Timer_B0(void);
void Init_Timer_B3(void);
void Init_Timers(void);

#endif /* TIMERB0_H_ */
