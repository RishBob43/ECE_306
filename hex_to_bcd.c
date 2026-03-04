/*------------------------------------------------------------------------------
 * File:        hex_to_bcd.c
 * Target:      MSP430FR2355
 *
 * Description: Converts a scaled ADC integer (0-1023) into four ASCII
 *              decimal digit characters and writes them to display_line[1].
 *
 *              Algorithm – successive subtraction:
 *                THOUSANDS  if value >= 1000, subtract and set digit to 1
 *                HUNDREDS   while value >= 100, subtract 100, increment
 *                TENS       while value >= 10,  subtract 10,  increment
 *                ONES       remainder
 *                ASCII      OR each digit with 0x30
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "functions.h"
#include "ADC.h"
#include "hex_to_bcd.h"
#include "macros.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * BCD digit globals
 *------------------------------------------------------------------------------*/
char thousands = '0';
char hundreds  = '0';
char tens      = '0';
char ones      = '0';

/*------------------------------------------------------------------------------
 * Externs
 *------------------------------------------------------------------------------*/
extern char                   display_line[4][11];
extern volatile unsigned char display_changed;

/*==============================================================================
 * HEX_to_BCD
 *
 * Converts a scaled ADC value (0-1023) into four ASCII digit globals.
 *==============================================================================*/
void HEX_to_BCD(unsigned int value){

    thousands = RESET_STATE;
    hundreds  = RESET_STATE;
    tens      = RESET_STATE;
    ones      = RESET_STATE;

    /* THOUSANDS - only 0 or 1 in range 0-1023 */
    if(value >= 1000u){
        value    -= 1000u;
        thousands = 1;
    }

    /* HUNDREDS */
    while(value >= 100u){
        value -= 100u;
        hundreds++;
    }

    /* TENS */
    while(value >= 10u){
        value -= 10u;
        tens++;
    }

    /* ONES */
    ones = (char)value;

    /* ASCII conversion - OR each digit with 0x30 */
    thousands |= ASCII_OFFSET;
    hundreds  |= ASCII_OFFSET;
    tens      |= ASCII_OFFSET;
    ones      |= ASCII_OFFSET;
}

/*==============================================================================
 * Update_Thumb_Display
 *
 * Converts ADC_Thumb to ASCII and writes "Thumb:xxxx" to display_line[1].
 *==============================================================================*/
void Update_Thumb_Display(void){

    HEX_to_BCD(ADC_Thumb);

    display_line[1][0] = 'T';
    display_line[1][1] = 'h';
    display_line[1][2] = 'u';
    display_line[1][3] = 'm';
    display_line[1][4] = 'b';
    display_line[1][5] = ':';
    display_line[1][6] = thousands;
    display_line[1][7] = hundreds;
    display_line[1][8] = tens;
    display_line[1][9] = ones;
    display_line[1][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;

    display_changed = TRUE;
}
