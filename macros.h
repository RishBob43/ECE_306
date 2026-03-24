/*------------------------------------------------------------------------------
 * File:        macros.h  (Project 7)
 * Target:      MSP430FR2355
 *
 * Description: Project-wide symbolic constants.
 *              Project 7 additions marked with // [P7]
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
 * Motor PWM (Timer B3) – period register
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
 * IR LED control values
 *------------------------------------------------------------------------------*/
#define IR_LED_ON               (0x01)
#define IR_LED_OFF              (0x00)

/*------------------------------------------------------------------------------
 * ADC channel indices for round-robin sequence
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
 * Line detection thresholds (scaled ÷4 ADC counts)
 *
 * BLACK_LINE_THRESHOLD is the STATIC FALLBACK used when dynamic calibration
 * cannot compute a valid midpoint.  The runtime threshold is stored in
 * g_threshold (main.c) and updated by update_dynamic_threshold().
 *
 * Calibrate these values if the static fallback is needed:
 *   Black surface -> high ADC value (low reflection, pull-up dominates)
 *   White surface -> low  ADC value (high reflection, transistor conducts)
 *------------------------------------------------------------------------------*/
#define BLACK_LINE_THRESHOLD    (650u)    /* Static fallback – tune on bench    */
#define WHITE_THRESHOLD         (125u)    /* Lower bound for "definitely white" */

/*------------------------------------------------------------------------------
 * Line state bitmask return codes
 *------------------------------------------------------------------------------*/
#define LINE_NONE               (0x00)
#define LINE_LEFT               (0x01)
#define LINE_RIGHT              (0x02)
#define LINE_BOTH               (0x03)

/*------------------------------------------------------------------------------
 * Vehicle state machine states  [P7 – expanded from P6]
 *   Values match the STATE_* defines in main.c; kept here for shared reference.
 *------------------------------------------------------------------------------*/
#define STATE_IDLE              (0x00)
#define STATE_DELAY             (0x01)
#define STATE_FORWARD           (0x02)
#define STATE_LINE_STOP         (0x03)
#define STATE_TURN              (0x04)
#define STATE_ON_LINE           (0x05)
/* P7 additions */
#define STATE_CALIBRATE         (0x06)    /* [P7] Ambient / white / black cal   */
#define STATE_INTERCEPT         (0x07)    /* [P7] Drive to line                 */
#define STATE_WAIT              (0x08)    /* [P7] Hold on detected line         */
#define STATE_ALIGN             (0x09)    /* [P7] Rotate until both on line     */
#define STATE_CIRCLE            (0x0A)    /* [P7] Follow line for 2 laps        */
#define STATE_EXIT              (0x0B)    /* [P7] Turn inward, drive to center  */
#define STATE_STOPPED           (0x0C)    /* [P7] All done, await SW1           */

/*------------------------------------------------------------------------------
 * Motor PWM duty cycles (for future PWM use)
 *------------------------------------------------------------------------------*/
#define DRIVE_SPEED             (6000u)
#define TURN_SPEED              (5000u)

/*------------------------------------------------------------------------------
 * State machine tick counts (1 tick = 200 ms from CCR0)
 *------------------------------------------------------------------------------*/
#define TICKS_1_SEC             (5u)      /* 5  * 200 ms = 1 s                  */
#define TICKS_4_SEC             (20u)     /* 20 * 200 ms = 4 s                  */
#define TICKS_TURN              (8u)      /* 8  * 200 ms = 1.6 s                */

/*------------------------------------------------------------------------------
 * Project 7 – circle follower parameters  [P7]
 *------------------------------------------------------------------------------*/
/*  Number of right-detector white->black rising edges that constitute 2 laps.
 *  Empirically determined: on a 36-inch circle, the right sensor crosses the
 *  black-to-edge boundary approximately twice per lap.  Start with 4 and tune. */
#define LAP_EDGE_COUNT          (4u)

/*  Duration of the inward exit turn + straight drive (in 200 ms ticks).
 *  12 ticks = 2.4 s.  Tune based on circle radius and motor speed.           */
#define TICKS_EXIT_DRIVE        (12u)

/*  Maximum ticks allowed for the ALIGN rotation before safety timeout.
 *  40 ticks = 8 s.                                                            */
#define TICKS_ALIGN_TIMEOUT     (40u)

/*  Settle ticks after toggling emitter during calibration (3 = 600 ms).       */
#define TICKS_CAL_SETTLE        (15u)  /* 15 × 200ms = 3s per phase            */

/*  Number of ADC samples averaged per calibration phase.                      */
#define NUM_CAL_SAMPLES         (16u)

/*  Minimum ADC count margin between white average and black average required
 *  before accepting the dynamic threshold (prevents noise from setting a
 *  useless midpoint).                                                          */
#define CAL_THRESHOLD_MARGIN    (50u)

/*------------------------------------------------------------------------------
 * ASCII digit conversion offset (used in hex_to_bcd.c and display helpers)
 *------------------------------------------------------------------------------*/
#define ASCII_OFFSET            (0x30)

#endif /* MACROS_H_ */
