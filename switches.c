/*------------------------------------------------------------------------------
 * File:        switches.c
 * Author:      [Student Name]
 * Date:        [Date]
 * Revised:     [Date]
 * Compiler:    TI Code Composer Studio / msp430-gcc
 * Target:      MSP430FR2355
 *
 * Description: Switch interrupt service routines and foreground processing
 *              for SW1 (P4.1) and SW2 (P2.3).
 *
 *              When a switch is pressed its ISR:
 *                1. Disables that switch's port interrupt to prevent chatter.
 *                2. Forces LCD_BACKLITE off and disables the CCR0 backlight
 *                   timer interrupt (requirement 12).
 *                3. Sets the corresponding debounce-in-progress flag bit.
 *                4. Resets that switch's debounce counter to zero.
 *                5. Arms (enables) the corresponding CCR debounce timer
 *                   (CCR1 for SW1, CCR2 for SW2).
 *                6. Writes "Switch 1" or "Switch 2" to display line 4 and
 *                   sets display_changed = TRUE.
 *
 *              The CCR1/CCR2 ISRs (in timerB0.c) count 100 ms slices and
 *              re-enable the switch interrupt and CCR0 once the debounce
 *              threshold is reached.
 *
 * Port interrupt vectors:
 *   switch1_interrupt -> PORT4_VECTOR  (SW1 on P4.1, falling edge)
 *   switch2_interrupt -> PORT2_VECTOR  (SW2 on P2.3, falling edge)
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * Externs from timerB0.c
 *------------------------------------------------------------------------------*/
extern volatile char         debounce_flags;
extern volatile unsigned int sw1_debounce_count;
extern volatile unsigned int sw2_debounce_count;

/*------------------------------------------------------------------------------
 * Externs from globals.c / Display.c
 *------------------------------------------------------------------------------*/
extern char                  display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;

/*------------------------------------------------------------------------------
 * sw1_pressed / sw2_pressed
 * DEFINED in globals.obj (globals.c) – extern references only here.
 * Set in the port ISRs below; consumed by the foreground Switch*_Process().
 *------------------------------------------------------------------------------*/
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;

/*==============================================================================
 * PORT4_VECTOR  –  switch1_interrupt
 *
 * Interrupt source:  Port 4 interrupt flag register (P4IFG)
 * Trigger:           Falling edge on P4.1 (SW1 pressed, active-low)
 *
 * Actions:
 *   1. Verify P4IFG bit for SW1 is set and pin is actually low (pressed).
 *   2. Disable SW1 port interrupt to prevent chatter during debounce.
 *   3. Turn off LCD_BACKLITE output (requirement 12).
 *   4. Disable CCR0 backlight toggle interrupt (requirement 12).
 *   5. Set SW1_DEBOUNCE_ACTIVE_BIT in debounce_flags (requirement 7).
 *   6. Reset sw1_debounce_count to zero (requirement 8).
 *   7. Advance TB0CCR1 beyond current TB0R and enable CCR1 interrupt
 *      so debounce slices begin counting (requirement 7).
 *   8. Update display line 4 with "Switch 1" and flag display_changed.
 *   9. Clear P4IFG SW1 flag.
 *
 * Globals used:
 *   debounce_flags        - SW1_DEBOUNCE_ACTIVE_BIT set
 *   sw1_debounce_count    - cleared to zero
 *   display_line[3]       - written with SW1_LINE4_MSG
 *   display_changed       - set TRUE
 *
 * Globals changed:
 *   debounce_flags
 *   sw1_debounce_count
 *   display_line[3]
 *   display_changed
 *   P4IE    (SW1 disabled)
 *   P4IFG   (SW1 flag cleared)
 *   P6OUT   (LCD_BACKLITE off)
 *   TB0CCTL0 (CCR0 interrupt disabled)
 *   TB0CCR1  (advanced for next slice)
 *   TB0CCTL1 (CCR1 interrupt enabled)
 *
 * Local variables:   none
 *============================================================================*/
#pragma vector = PORT4_VECTOR
__interrupt void switch1_interrupt(void){

    if(P4IFG & SW1){

        if(!(P4IN & SW1)){              /* Confirm pin is actually low (pressed) */

            /* Step 2: Disable SW1 interrupt to block chatter */
            P4IE  &= ~SW1;

            /* Step 3: Turn backlight off immediately */
            P6OUT &= ~LCD_BACKLITE;

            /* Step 4: Disable CCR0 backlight timer interrupt */
            TB0CCTL0 &= CCR_INTERRUPT_DISABLE;

            /* Step 5: Mark SW1 debounce as in progress */
            debounce_flags |= SW1_DEBOUNCE_ACTIVE_BIT;

            /* Step 6: Reset SW1 debounce slice counter */
            sw1_debounce_count = RESET_STATE;

            /* Step 7: Arm CCR1 for the first 100 ms slice.
             *         Place CCR1 ahead of the current timer value so it fires
             *         100 ms from now, not potentially behind TB0R.           */
            TB0CCR1  = TB0R + TB0CCR1_DEBOUNCE_SLICE;
            TB0CCTL1 |= CCR_INTERRUPT_ENABLE;

            /* Step 8: Write "Switch 1" to display line 4 */
            strncpy(display_line[3], SW1_LINE4_MSG, DISPLAY_LINE_LENGTH - 1);
            display_line[3][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;

            /* Step 8 continued: flag that display content has changed */
            display_changed = TRUE;

            /* Step 9: Set sw1_pressed so foreground can run Display_Process */
            sw1_pressed = TRUE;
        }

        /* Clear the interrupt flag regardless of pin state */
        P4IFG &= ~SW1;
    }
}

/*==============================================================================
 * PORT2_VECTOR  –  switch2_interrupt
 *
 * Interrupt source:  Port 2 interrupt flag register (P2IFG)
 * Trigger:           Falling edge on P2.3 (SW2 pressed, active-low)
 *
 * Actions:
 *   1. Verify P2IFG bit for SW2 is set and pin is actually low (pressed).
 *   2. Disable SW2 port interrupt to prevent chatter during debounce.
 *   3. Turn off LCD_BACKLITE output (requirement 12).
 *   4. Disable CCR0 backlight toggle interrupt (requirement 12).
 *   5. Set SW2_DEBOUNCE_ACTIVE_BIT in debounce_flags (requirement 7).
 *   6. Reset sw2_debounce_count to zero (requirement 8).
 *   7. Advance TB0CCR2 beyond current TB0R and enable CCR2 interrupt
 *      so debounce slices begin counting (requirement 7).
 *   8. Update display line 4 with "Switch 2" and flag display_changed.
 *   9. Clear P2IFG SW2 flag.
 *
 * Globals used:
 *   debounce_flags        - SW2_DEBOUNCE_ACTIVE_BIT set
 *   sw2_debounce_count    - cleared to zero
 *   display_line[3]       - written with SW2_LINE4_MSG
 *   display_changed       - set TRUE
 *
 * Globals changed:
 *   debounce_flags
 *   sw2_debounce_count
 *   display_line[3]
 *   display_changed
 *   P2IE    (SW2 disabled)
 *   P2IFG   (SW2 flag cleared)
 *   P6OUT   (LCD_BACKLITE off)
 *   TB0CCTL0 (CCR0 interrupt disabled)
 *   TB0CCR2  (advanced for next slice)
 *   TB0CCTL2 (CCR2 interrupt enabled)
 *
 * Local variables:   none
 *============================================================================*/
#pragma vector = PORT2_VECTOR
__interrupt void switch2_interrupt(void){

    if(P2IFG & SW2){

        if(!(P2IN & SW2)){              /* Confirm pin is actually low (pressed) */

            /* Step 2: Disable SW2 interrupt to block chatter */
            P2IE  &= ~SW2;

            /* Step 3: Turn backlight off immediately */
            P6OUT &= ~LCD_BACKLITE;

            /* Step 4: Disable CCR0 backlight timer interrupt */
            TB0CCTL0 &= CCR_INTERRUPT_DISABLE;

            /* Step 5: Mark SW2 debounce as in progress */
            debounce_flags |= SW2_DEBOUNCE_ACTIVE_BIT;

            /* Step 6: Reset SW2 debounce slice counter */
            sw2_debounce_count = RESET_STATE;

            /* Step 7: Arm CCR2 for the first 100 ms slice */
            TB0CCR2  = TB0R + TB0CCR2_DEBOUNCE_SLICE;
            TB0CCTL2 |= CCR_INTERRUPT_ENABLE;

            /* Step 8: Write "Switch 2" to display line 4 */
            strncpy(display_line[3], SW2_LINE4_MSG, DISPLAY_LINE_LENGTH - 1);
            display_line[3][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;

            /* Step 8 continued: flag that display content has changed */
            display_changed = TRUE;

            /* Step 9: Set sw2_pressed so foreground can run Display_Process */
            sw2_pressed = TRUE;
        }

        /* Clear the interrupt flag regardless of pin state */
        P2IFG &= ~SW2;
    }
}

/*------------------------------------------------------------------------------
 * Init_Switches
 *
 * Description: Configures SW1 and SW2 as falling-edge GPIO interrupts.
 *              Port pin direction, resistor, and interrupt settings are
 *              already applied in ports.c (Init_Port2 / Init_Port4).
 *              This function performs a final flag-clear-and-enable to
 *              guarantee a clean start after all other init has run.
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Switches(void){

    /* SW1 (P4.1): clear any startup flag, ensure interrupt is enabled */
    P4IFG &= ~SW1;
    P4IE  |=  SW1;

    /* SW2 (P2.3): clear any startup flag, ensure interrupt is enabled */
    P2IFG &= ~SW2;
    P2IE  |=  SW2;
}

/*------------------------------------------------------------------------------
 * Switch1_Process
 *
 * Description: Foreground handler called from the main loop.
 *              When sw1_pressed is set, clears the flag and calls
 *              Display_Process() so the line 4 change written in the ISR
 *              is sent to the LCD hardware (SPI calls not allowed in ISR).
 *
 * Globals used:
 *   sw1_pressed     - read and cleared
 *   update_display  - read by Display_Process()
 *   display_changed - read by Display_Process()
 *
 * Globals changed:
 *   sw1_pressed
 *
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Switch1_Process(void){
    if(sw1_pressed){
        sw1_pressed = FALSE;

        /* Force update_display true so Display_Process runs immediately.
         * The 200 ms gate has already been respected because the ISR happened
         * after at least one CCR0 tick, but we set it explicitly here to
         * ensure the line 4 "Switch 1" message is not missed.              */
        update_display  = TRUE;
        Display_Process();
    }
}

/*------------------------------------------------------------------------------
 * Switch2_Process
 *
 * Description: Foreground handler for SW2. Mirrors Switch1_Process.
 *
 * Globals used:
 *   sw2_pressed     - read and cleared
 *   update_display  - set to force Display_Process
 *   display_changed - read by Display_Process()
 *
 * Globals changed:
 *   sw2_pressed
 *
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Switch2_Process(void){
    if(sw2_pressed){
        sw2_pressed = FALSE;

        update_display  = TRUE;
        Display_Process();
    }
}

/*------------------------------------------------------------------------------
 * Switches_Process
 *
 * Description: Called every pass of the main while(ALWAYS) loop.
 *              Dispatches to individual switch handlers.
 *
 * Globals used:   sw1_pressed, sw2_pressed (via sub-functions)
 * Globals changed: sw1_pressed, sw2_pressed (via sub-functions)
 * Locals used:    none
 *------------------------------------------------------------------------------*/
void Switches_Process(void){
    Switch1_Process();
    Switch2_Process();
}

/*------------------------------------------------------------------------------
 * enable_switch_SW1 / disable_switch_SW1
 * enable_switch_SW2 / disable_switch_SW2
 *
 * Description: Convenience helpers that other modules may call.
 *              The debounce ISR in timerB0.c calls these internally via
 *              direct register access for interrupt-context speed, but
 *              these wrappers are available for foreground use.
 *
 * Globals changed: P4IE, P4IFG (SW1) / P2IE, P2IFG (SW2)
 * Locals used:     none
 *------------------------------------------------------------------------------*/
void enable_switch_SW1(void){
    P4IFG &= ~SW1;
    P4IE  |=  SW1;
}

void disable_switch_SW1(void){
    P4IE  &= ~SW1;
}

void enable_switch_SW2(void){
    P2IFG &= ~SW2;
    P2IE  |=  SW2;
}

void disable_switch_SW2(void){
    P2IE  &= ~SW2;
}
