/*------------------------------------------------------------------------------
 * File:        timerB0.h
 * Target:      MSP430FR2355
 *
 * Description: Header file for Timer B0 configuration and CCR-based ISRs.
 *
 *              CCR0 - update_display_count increment / display gate (200 ms)
 *              CCR1 - Switch 1 (SW1, P4.1) debounce timer
 *              CCR2 - Switch 2 (SW2, P2.3) debounce timer
 *              OVF  - DAC soft-start ramp (TB0IV = 14)
 *
 *              Timer clock derivation:
 *                SMCLK          = 8,000,000 Hz
 *                ID__8          = /8  -> 1,000,000 Hz
 *                TBIDEX__8      = /8  ->   125,000 Hz  (1 tick = 8 us)
 *
 *              CCR0  200 ms period:
 *                8,000,000 / 8 / 8 / 5 = 25,000 ticks
 *
 *              CCR1 / CCR2  debounce slice = 50 ms per interrupt:
 *                8,000,000 / 8 / 8 / 20 = 6,250 ticks
 *                Threshold = 2 slices -> 100 ms total debounce lockout
 *                (reduced from the original 1,000 ms / 10 slices so the
 *                button responds normally after a single clean press)
 *------------------------------------------------------------------------------*/

#ifndef TIMERB0_H_
#define TIMERB0_H_

#include <msp430.h>
#include "macros.h"

/*------------------------------------------------------------------------------
 * CCR0 – display tick gate
 *   Interrupt period: 200 ms
 *   8,000,000 / 8 / 8 / 5  =  25,000 ticks
 *------------------------------------------------------------------------------*/
#define TB0CCR0_INTERVAL        (25000u)   /* 200 ms compare count             */

/*------------------------------------------------------------------------------
 * CCR1 / CCR2 – switch debounce slices
 *
 *   Slice period: 50 ms
 *   8,000,000 / 8 / 8 / 20  =  6,250 ticks per slice
 *
 *   DEBOUNCE_THRESHOLD = 2 slices × 50 ms = 100 ms total lockout.
 *
 *   Why 100 ms instead of the original 1,000 ms?
 *     The original 10-slice / 1-second lockout was safe against contact bounce
 *     but made the button feel unresponsive – especially problematic now that
 *     sw1_pressed is polled every loop pass rather than once per 200 ms tick.
 *     100 ms is long enough to suppress mechanical bounce (typically < 20 ms)
 *     while still feeling instant to the user.
 *     If you see double-triggers on your specific switch hardware, raise
 *     DEBOUNCE_THRESHOLD to 3 or 4 (150 ms or 200 ms).
 *------------------------------------------------------------------------------*/
#define TB0CCR1_DEBOUNCE_SLICE  (6250u)    /* 50 ms per CCR1 interrupt         */
#define TB0CCR2_DEBOUNCE_SLICE  (6250u)    /* 50 ms per CCR2 interrupt         */
#define DEBOUNCE_THRESHOLD      (2u)       /* 2 slices × 50 ms = 100 ms        */

/*------------------------------------------------------------------------------
 * CCR Control Bits
 *------------------------------------------------------------------------------*/
#define CCR_INTERRUPT_ENABLE    (CCIE)
#define CCR_INTERRUPT_DISABLE   (~CCIE)
#define CCR_INTERRUPT_FLAG_CLR  (~CCIFG)

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
