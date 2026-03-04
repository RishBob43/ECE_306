/*------------------------------------------------------------------------------
 * File:        macros.h
 * Author:      [Student Name]
 * Date:        [Date]
 * Revised:     [Date]
 * Compiler:    TI Code Composer Studio / msp430-gcc
 * Target:      MSP430FR2355
 *
 * Description: Project-wide symbolic constants.  No magic numbers are
 *              permitted in any source file; every numeric literal used in
 *              logic must trace back to a #define in this file or in the
 *              relevant peripheral header.
 *------------------------------------------------------------------------------*/

#ifndef MACROS_H_
#define MACROS_H_

/*------------------------------------------------------------------------------
 * General boolean / state values
 *------------------------------------------------------------------------------*/
#define ALWAYS                  (1)
#define RESET_STATE             (0)
#define TRUE                    (0x01)
#define FALSE                   (0x00)

/*------------------------------------------------------------------------------
 * GPIO pin aliases  (also defined in ports.h by bit position;
 * kept here for LED references in led.c)
 *------------------------------------------------------------------------------*/
#define RED_LED                 (0x01)     /* P1.0  */
#define GRN_LED                 (0x40)     /* P6.6  */
#define TEST_PROBE              (0x01)     /* P3.0  */

/*------------------------------------------------------------------------------
 * Port 3 SMCLK output mode selector (used by Init_Port3)
 *------------------------------------------------------------------------------*/
#define USE_GPIO                (0x00)
#define USE_SMCLK               (0x01)

/*------------------------------------------------------------------------------
 * MCLK / SMCLK frequency
 *   MCLK  = 8,000,000 Hz
 *   SMCLK = 8,000,000 Hz  (DIVS__1 in clocks.c)
 *------------------------------------------------------------------------------*/
#define MCLK_HZ                 (8000000UL)
#define CYCLES_PER_USEC         (8)         /* 8 MHz -> 8 cycles per µs        */
#define TEN_USEC                (10)        /* Multiplier for usleep10()        */
#define CYCLES_PER_MS           (8000)      /* 8 MHz -> 8000 cycles per ms      */
#define FIVE_MS_MULTIPLIER      (5)         /* Used in five_msec_sleep()        */

/*------------------------------------------------------------------------------
 * Motor PWM (Timer B3)
 *------------------------------------------------------------------------------*/
#define WHEEL_PERIOD            (50005)     /* TB3 period ticks                 */
#define WHEEL_OFF               (0)         /* Duty cycle = 0 (motor off)       */

/*------------------------------------------------------------------------------
 * Display buffer dimensions
 *------------------------------------------------------------------------------*/
#define DISPLAY_LINE_COUNT      (4)
#define DISPLAY_LINE_LENGTH     (11)        /* 10 chars + null terminator       */

/*------------------------------------------------------------------------------
 * Switch line-4 display strings  (exactly 10 printable chars)
 *------------------------------------------------------------------------------*/
#define SW1_LINE4_MSG           "Switch 1  "
#define SW2_LINE4_MSG           "Switch 2  "

#endif /* MACROS_H_ */
