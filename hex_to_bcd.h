/*------------------------------------------------------------------------------
 * File:        hex_to_bcd.h
 * Target:      MSP430FR2355
 *
 * Description: Declares the four BCD digit globals and the conversion
 *              function used to display a 12-bit ADC result (0-4095,
 *              but practically 0-1023 after the ÷4 scale shift) on the
 *              four-character LCD field.
 *
 *              Conversion pipeline:
 *                1. ADC ISR stores raw 12-bit result >> 2  (0-1023)
 *                2. HEX_to_BCD() splits the integer into four decimal
 *                   digit variables: thousands, hundreds, tens, ones
 *                3. Each digit is OR'd with 0x30 to produce its ASCII
 *                   character ('0'-'9') for direct LCD output
 *
 * Usage:
 *   #include "hex_to_bcd.h"
 *   HEX_to_BCD(ADC_Thumb);
 *   display_line[2][6] = thousands;   // already ASCII after conversion
 *   display_line[2][7] = hundreds;
 *   display_line[2][8] = tens;
 *   display_line[2][9] = ones;
 *------------------------------------------------------------------------------*/

#ifndef HEX_TO_BCD_H_
#define HEX_TO_BCD_H_

/*------------------------------------------------------------------------------
 * ASCII conversion offset
 *   ORing a single BCD digit (0-9) with 0x30 yields its ASCII code
 *   ('0' = 0x30, '1' = 0x31, … '9' = 0x39).
 *------------------------------------------------------------------------------*/
#define ASCII_OFFSET            (0x30)

/*------------------------------------------------------------------------------
 * BCD digit globals – defined in hex_to_bcd.c, extern'd for display modules
 *------------------------------------------------------------------------------*/
extern char thousands;
extern char hundreds;
extern char tens;
extern char ones;

/*------------------------------------------------------------------------------
 * Function prototype
 *------------------------------------------------------------------------------*/
void HEX_to_BCD(unsigned int value);

#endif /* HEX_TO_BCD_H_ */
