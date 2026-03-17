/*------------------------------------------------------------------------------
 * File:        macros.h  (Project 6)
 * Target:      MSP430FR2355
 *
 * Description: Project-wide symbolic constants.
 *              Project 6 additions marked with // [P6]
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
 * GPIO pin aliases
 *------------------------------------------------------------------------------*/
#define RED_LED                 (0x01)     /* P1.0  */
#define GRN_LED                 (0x40)     /* P6.6  */
#define TEST_PROBE              (0x01)     /* P3.0  */

/*------------------------------------------------------------------------------
 * Port 3 SMCLK output mode selector
 *------------------------------------------------------------------------------*/
#define USE_GPIO                (0x00)
#define USE_SMCLK               (0x01)

/*------------------------------------------------------------------------------
 * MCLK / SMCLK frequency
 *------------------------------------------------------------------------------*/
#define MCLK_HZ                 (8000000UL)
#define CYCLES_PER_USEC         (8)
#define TEN_USEC                (10)
#define CYCLES_PER_MS           (8000)
#define FIVE_MS_MULTIPLIER      (5)

/*------------------------------------------------------------------------------
 * Motor PWM (Timer B3)
 *------------------------------------------------------------------------------*/
#define WHEEL_PERIOD            (50005)
#define WHEEL_OFF               (0)

/*------------------------------------------------------------------------------
 * Display buffer dimensions
 *------------------------------------------------------------------------------*/
#define DISPLAY_LINE_COUNT      (4)
#define DISPLAY_LINE_LENGTH     (11)

/*------------------------------------------------------------------------------
 * Switch line-4 display strings
 *------------------------------------------------------------------------------*/
#define SW1_LINE4_MSG           "Switch 1  "
#define SW2_LINE4_MSG           "Switch 2  "

/*------------------------------------------------------------------------------
 * IR LED control values  [P6]
 *------------------------------------------------------------------------------*/
#define IR_LED_ON               (0x01)
#define IR_LED_OFF              (0x00)

/*------------------------------------------------------------------------------
 * ADC channel indices for round-robin sequence  [P6]
 *   0 -> A2  V_DETECT_L
 *   1 -> A3  V_DETECT_R
 *   2 -> A5  V_THUMB
 *------------------------------------------------------------------------------*/
#define ADC_CHANNEL_L_DETECT    (0x00)
#define ADC_CHANNEL_R_DETECT    (0x01)
#define ADC_CHANNEL_THUMB       (0x02)
#define ADC_CHANNEL_COUNT       (3u)
#define ADC_SCALE_SHIFT         (2u)      /* Right-shift ÷4 to fit 4 digits    */

/*------------------------------------------------------------------------------
 * Line detection thresholds (scaled ÷4 ADC counts)  [P6]
 *   Calibrate these values during lab verification.
 *   Black surface -> high ADC value (low reflection -> pull-up dominates)
 *   White surface -> low  ADC value (high reflection -> transistor conducts)
 *------------------------------------------------------------------------------*/
#define BLACK_LINE_THRESHOLD    (650u)
#define WHITE_THRESHOLD         (125u)

/*------------------------------------------------------------------------------
 * Line state bitmask return codes  [P6]
 *------------------------------------------------------------------------------*/
#define LINE_NONE               (0x00)
#define LINE_LEFT               (0x01)
#define LINE_RIGHT              (0x02)
#define LINE_BOTH               (0x03)

/*------------------------------------------------------------------------------
 * Vehicle state machine states  [P6]
 *------------------------------------------------------------------------------*/
#define STATE_IDLE              (0x00)
#define STATE_DELAY             (0x01)
#define STATE_FORWARD           (0x02)
#define STATE_LINE_STOP         (0x03)
#define STATE_TURN              (0x04)
#define STATE_ON_LINE           (0x05)

/*------------------------------------------------------------------------------
 * Motor PWM duty cycles  [P6]
 *------------------------------------------------------------------------------*/
#define DRIVE_SPEED             (6000u)
#define TURN_SPEED              (5000u)

/*------------------------------------------------------------------------------
 * State machine tick counts (1 tick = 200 ms from CCR0)  [P6]
 *------------------------------------------------------------------------------*/
#define TICKS_1_SEC             (5u)     /* 5  * 200 ms = 1 s                  */
#define TICKS_4_SEC             (20u)    /* 20 * 200 ms = 4 s                  */
#define TICKS_TURN              (8u)     /* 8  * 200 ms = 1.6 s per unit       */

#endif /* MACROS_H_ */
