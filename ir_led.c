/*------------------------------------------------------------------------------
 * File:        ir_led.c
 * Target:      MSP430FR2355
 *
 * Description: IR emitter control and black-line detection helpers.
 *
 *              The IR LED is driven by P2.2 (IR_LED, active-HIGH output).
 *              Turning the emitter ON increases reflected energy from white
 *              surfaces, lowering the phototransistor collector voltage and
 *              therefore lowering ADC_Left_Detect / ADC_Right_Detect.
 *              Over a black surface reflection is minimal so the pull-up
 *              keeps the collector high -> large ADC value.
 *
 *              Detection logic (emitter ON):
 *                ADC value > BLACK_LINE_THRESHOLD  -> black detected
 *                ADC value < WHITE_THRESHOLD       -> white detected
 *
 *              IR_LED_control(selection):
 *                IR_LED_ON  (1)  -> P2OUT |=  IR_LED
 *                IR_LED_OFF (0)  -> P2OUT &= ~IR_LED
 *
 * Functions:
 *   IR_LED_control(char)  – turn emitter on or off
 *   Is_Black_Left(void)   – TRUE when left  detector reads black line
 *   Is_Black_Right(void)  – TRUE when right detector reads black line
 *   Get_Line_State(void)  – returns combined BOTH/LEFT/RIGHT/NONE code
 *------------------------------------------------------------------------------*/

#include <ADC.h>
#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

/*------------------------------------------------------------------------------
 * IR_LED_ON / IR_LED_OFF selector values
 *------------------------------------------------------------------------------*/
#define IR_LED_ON               (0x01)
#define IR_LED_OFF              (0x00)

/*------------------------------------------------------------------------------
 * Line-state return codes for Get_Line_State()
 *------------------------------------------------------------------------------*/
#define LINE_NONE               (0x00)   /* Neither detector over black         */
#define LINE_LEFT               (0x01)   /* Left  detector only over black      */
#define LINE_RIGHT              (0x02)   /* Right detector only over black      */
#define LINE_BOTH               (0x03)   /* Both  detectors over black          */

/*==============================================================================
 * IR_LED_control
 *
 * Description: Sets or clears the IR LED output on P2.2.
 *
 * Parameters:
 *   selection  IR_LED_ON  (non-zero) -> LED on
 *              IR_LED_OFF (0)        -> LED off
 *
 * Globals changed: P2OUT (IR_LED bit)
 * Locals used:     none
 *==============================================================================*/
void IR_LED_control(char selection){
    if(selection){
        P2OUT |=  IR_LED;    /* Drive P2.2 high -> IR LED on  */
    } else {
        P2OUT &= ~IR_LED;    /* Drive P2.2 low  -> IR LED off */
    }
}

/*==============================================================================
 * Backlite_control
 *
 * Description: Convenience wrapper to turn the LCD backlight on or off.
 *
 * Parameters:
 *   selection  non-zero -> backlight on
 *              0        -> backlight off
 *
 * Globals changed: P6OUT (LCD_BACKLITE bit)
 * Locals used:     none
 *==============================================================================*/
void Backlite_control(char selection){
    if(selection){
        P6OUT |=  LCD_BACKLITE;
    } else {
        P6OUT &= ~LCD_BACKLITE;
    }
}

/*==============================================================================
 * Is_Black_Left
 *
 * Description: Returns TRUE when the left IR detector reading exceeds the
 *              black-line threshold (emitter must be ON for valid result).
 *
 * Globals used: ADC_Left_Detect (from adc.c)
 * Returns:      unsigned char  TRUE / FALSE
 *==============================================================================*/
unsigned char Is_Black_Left(void){
    return (ADC_Left_Detect > BLACK_LINE_THRESHOLD) ? TRUE : FALSE;
}

/*==============================================================================
 * Is_Black_Right
 *
 * Description: Returns TRUE when the right IR detector reading exceeds the
 *              black-line threshold.
 *
 * Globals used: ADC_Right_Detect (from adc.c)
 * Returns:      unsigned char  TRUE / FALSE
 *==============================================================================*/
unsigned char Is_Black_Right(void){
    return (ADC_Right_Detect > BLACK_LINE_THRESHOLD) ? TRUE : FALSE;
}

/*==============================================================================
 * Get_Line_State
 *
 * Description: Returns a combined bitmask indicating which detectors see
 *              the black line.
 *
 * Returns:
 *   LINE_BOTH   (0x03)  both  detectors over black
 *   LINE_RIGHT  (0x02)  right detector only
 *   LINE_LEFT   (0x01)  left  detector only
 *   LINE_NONE   (0x00)  neither detector over black
 *==============================================================================*/
unsigned char Get_Line_State(void){
    unsigned char state = LINE_NONE;
    if(Is_Black_Left())  state |= LINE_LEFT;
    if(Is_Black_Right()) state |= LINE_RIGHT;
    return state;
}
