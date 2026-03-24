/*------------------------------------------------------------------------------
 * File:        main.c  (Project 7 – PID + DAC)
 * Target:      MSP430FR2355
 *
 * Description: Black-line circle follower with:
 *   - Three-phase calibration (ambient / white / black)
 *   - Dynamic threshold computed from calibration data
 *   - PID steering controller (proportional + integral + derivative)
 *   - DAC-controlled motor supply voltage via LT1935 buck-boost
 *   - TB3 PWM differential steering (PID correction splits left/right CCR)
 *   - Two-lap counter using right-detector edge detection
 *   - Automatic inward exit to circle center
 *
 * PID line-following strategy:
 *   error      = (int)ADC_Right_Detect - (int)ADC_Left_Detect
 *   correction = PID_Update(&pid, error)
 *   RIGHT CCR  = PID_BASE_SPEED - correction
 *   LEFT  CCR  = PID_BASE_SPEED + correction
 *
 *   Positive error (car drifted right): left speeds up, right slows -> steers left
 *   Negative error (car drifted left):  right speeds up, left slows -> steers right
 *
 * DAC motor voltage:
 *   Init_DAC() ramps SAC3DAT from DAC_BEGIN (~2V) to DAC_ADJUST (~6V) via
 *   the TB0 overflow ISR.  Foreground waits for DAC_ready before driving.
 *   During line following, DAC holds at DAC_CRUISE (~5V) for safe speed.
 *
 * PWM channel assignment (TB3, OUTMOD_7):
 *   TB3CCR2 -> R_FORWARD (P6.1) – right wheel forward speed
 *   TB3CCR3 -> L_FORWARD (P6.2) – left  wheel forward speed
 *   TB3CCR4 -> R_REVERSE (P6.3) – WHEEL_OFF during circle following
 *   TB3CCR5 -> L_REVERSE (P6.4) – WHEEL_OFF during circle following
 *
 * IMPORTANT: To use PWM on P6.1-P6.4, Init_Port6() in ports.c must set
 *   P6SEL0 |= (R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE)
 *   instead of the current GPIO configuration.  GPIO macros (MOTOR_*_GPIO)
 *   are still used for alignment turns and the exit maneuver.
 *
 * State machine:
 *   CALIBRATE -> IDLE -> DELAY -> INTERCEPT -> WAIT -> ALIGN
 *             -> CIRCLE -> EXIT -> STOPPED
 *
 * Display (4 lines x 10 chars, updated every 200 ms):
 *   Line 0: state label     "CIRCLING  "
 *   Line 1: right detector  "R:0742    "
 *   Line 2: left  detector  "L:0318    "
 *   Line 3: elapsed time    "T: 12.4s  "
 *
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include "ADC.h"
#include "hex_to_bcd.h"
#include "dac.h"
#include "pid.h"

/*==============================================================================
 * Externs
 *============================================================================*/
extern char                   display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;

/*==============================================================================
 * Local symbolic constants
 *
 * NOTE: STATE_*, TICKS_*, LAP_EDGE_COUNT, TICKS_EXIT_DRIVE,
 *       TICKS_ALIGN_TIMEOUT, TICKS_CAL_SETTLE, NUM_CAL_SAMPLES, and
 *       CAL_THRESHOLD_MARGIN are defined in macros.h.
 *       PID_BASE_SPEED, PID_MIN_SPEED, PID_MAX_SPEED are in dac.h.
 *       PID gains and limits are in pid.h.
 *============================================================================*/

/*-- TB3 PWM motor macros ----------------------------------------------------*/
/*  Straight: both forward CCRs at base speed, reverse CCRs off              */
#define MOTORS_PWM_STRAIGHT()   do { TB3CCR2 = PID_BASE_SPEED;   \
                                     TB3CCR3 = PID_BASE_SPEED;   \
                                     TB3CCR4 = WHEEL_OFF;        \
                                     TB3CCR5 = WHEEL_OFF; } while(0)

/*  Full stop via PWM registers                                               */
#define MOTORS_STOP()           do { TB3CCR2 = WHEEL_OFF;        \
                                     TB3CCR3 = WHEEL_OFF;        \
                                     TB3CCR4 = WHEEL_OFF;        \
                                     TB3CCR5 = WHEEL_OFF; } while(0)

/*-- GPIO motor macros (for alignment and exit turns) ------------------------*/
/*  Turn left: right wheel GPIO forward, left stopped                        */
#define MOTOR_TURN_LEFT_GPIO()   do { P6OUT |=  R_FORWARD;                    \
                                      P6OUT &= ~(L_FORWARD|R_REVERSE|L_REVERSE); } while(0)
/*  Inward exit: left forward, right reverse                                  */
#define MOTOR_INWARD_TURN_GPIO() do { P6OUT |=  (L_FORWARD | R_REVERSE);      \
                                      P6OUT &= ~(R_FORWARD | L_REVERSE);      } while(0)
/*  GPIO stop: clear all motor pins                                           */
#define MOTORS_STOP_GPIO()       do { P6OUT &= ~(R_FORWARD|L_FORWARD|          \
                                                  R_REVERSE|L_REVERSE); } while(0)

/*-- Display line indices ----------------------------------------------------*/
#define DISP_STATE_LINE         (0u)
#define DISP_RIGHT_LINE         (1u)
#define DISP_LEFT_LINE          (2u)
#define DISP_TIMER_LINE         (3u)

/*==============================================================================
 * Module globals
 *============================================================================*/
static unsigned int  g_ambient_left  = RESET_STATE;
static unsigned int  g_ambient_right = RESET_STATE;
static unsigned int  g_white_left    = RESET_STATE;
static unsigned int  g_white_right   = RESET_STATE;
static unsigned int  g_black_left    = RESET_STATE;
static unsigned int  g_black_right   = RESET_STATE;
static unsigned int  g_threshold     = BLACK_LINE_THRESHOLD;

static volatile unsigned int  g_timer_ticks  = RESET_STATE;
static volatile unsigned char g_timer_active = FALSE;

static unsigned char g_lap_edges        = RESET_STATE;
static unsigned char g_prev_right_black = FALSE;

static PID_State g_pid;

/*==============================================================================
 * Forward declarations
 *============================================================================*/
static void          set_line(int line, const char *msg);
static void          show_detector_values(void);
static void          show_timer(void);
static unsigned int  average_adc_samples(unsigned char channel_flag);
static void          run_calibration(void);
static void          update_dynamic_threshold(void);
static unsigned char is_black_left_dyn(void);
static unsigned char is_black_right_dyn(void);
static unsigned char get_line_state_dyn(void);
static void          line_follow_pid(void);
static void          clamp_ccr(volatile unsigned int *ccr, unsigned int val);

/*==============================================================================
 * set_line
 *==============================================================================*/
static void set_line(int line, const char *msg){
    strncpy(display_line[line], msg, DISPLAY_LINE_LENGTH - 1u);
    display_line[line][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;
    display_changed = TRUE;
}

/*==============================================================================
 * show_detector_values
 *==============================================================================*/
static void show_detector_values(void){

    HEX_to_BCD(ADC_Right_Detect);
    display_line[DISP_RIGHT_LINE][0] = 'R';
    display_line[DISP_RIGHT_LINE][1] = ':';
    display_line[DISP_RIGHT_LINE][2] = thousands;
    display_line[DISP_RIGHT_LINE][3] = hundreds;
    display_line[DISP_RIGHT_LINE][4] = tens;
    display_line[DISP_RIGHT_LINE][5] = ones;
    display_line[DISP_RIGHT_LINE][6] = ' ';
    display_line[DISP_RIGHT_LINE][7] = ' ';
    display_line[DISP_RIGHT_LINE][8] = ' ';
    display_line[DISP_RIGHT_LINE][9] = ' ';
    display_line[DISP_RIGHT_LINE][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;

    HEX_to_BCD(ADC_Left_Detect);
    display_line[DISP_LEFT_LINE][0] = 'L';
    display_line[DISP_LEFT_LINE][1] = ':';
    display_line[DISP_LEFT_LINE][2] = thousands;
    display_line[DISP_LEFT_LINE][3] = hundreds;
    display_line[DISP_LEFT_LINE][4] = tens;
    display_line[DISP_LEFT_LINE][5] = ones;
    display_line[DISP_LEFT_LINE][6] = ' ';
    display_line[DISP_LEFT_LINE][7] = ' ';
    display_line[DISP_LEFT_LINE][8] = ' ';
    display_line[DISP_LEFT_LINE][9] = ' ';
    display_line[DISP_LEFT_LINE][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;

    display_changed = TRUE;
}

/*==============================================================================
 * show_timer
 *   1 tick = 200 ms = 0.2 s -> tenths = ticks * 2, range 0.0 - 999.8 s
 *==============================================================================*/
static void show_timer(void){

    unsigned int  tenths;
    unsigned int  secs;
    unsigned char tenth_digit;
    unsigned int  s_hundreds;
    unsigned int  s_tens;
    unsigned int  s_ones;

    tenths = g_timer_ticks * 2u;
    if(tenths > 9998u){ tenths = 9998u; }

    secs        = tenths / 10u;
    tenth_digit = (unsigned char)(tenths % 10u);

    s_hundreds = RESET_STATE;
    s_tens     = RESET_STATE;
    s_ones     = secs;
    while(s_ones >= 100u){ s_ones -= 100u; s_hundreds++; }
    while(s_ones >= 10u) { s_ones -= 10u;  s_tens++;     }

    display_line[DISP_TIMER_LINE][0] = 'T';
    display_line[DISP_TIMER_LINE][1] = ':';
    display_line[DISP_TIMER_LINE][2] = (char)(s_hundreds + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][3] = (char)(s_tens     + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][4] = (char)(s_ones     + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][5] = '.';
    display_line[DISP_TIMER_LINE][6] = (char)(tenth_digit + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][7] = 's';
    display_line[DISP_TIMER_LINE][8] = ' ';
    display_line[DISP_TIMER_LINE][9] = ' ';
    display_line[DISP_TIMER_LINE][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;

    display_changed = TRUE;
}

/*==============================================================================
 * average_adc_samples
 *   channel_flag: 0 = left detector, non-zero = right detector
 *==============================================================================*/
static unsigned int average_adc_samples(unsigned char channel_flag){

    unsigned int  sum   = RESET_STATE;
    unsigned char count = RESET_STATE;

    while(count < NUM_CAL_SAMPLES){
        if(ADC_updated){
            ADC_updated = FALSE;
            sum += (channel_flag == RESET_STATE) ? ADC_Left_Detect
                                                 : ADC_Right_Detect;
            count++;
        }
    }
    return (sum / NUM_CAL_SAMPLES);
}

/*==============================================================================
 * run_calibration
 *   Phase 0: emitter OFF -> ambient baseline
 *   Phase 1: emitter ON,  car on white paper
 *   Phase 2: emitter ON,  car on black tape (user repositions during pause)
 *==============================================================================*/
static void run_calibration(void){

    unsigned char settle;

    /* Phase 0 – ambient (emitter OFF) */
    IR_LED_control(IR_LED_OFF);
    set_line(DISP_STATE_LINE, "CAL:AMBT  ");
    set_line(DISP_RIGHT_LINE, "Emitter   ");
    set_line(DISP_LEFT_LINE,  "OFF       ");
    set_line(DISP_TIMER_LINE, "Place:WHT ");
    settle = RESET_STATE;
    while(settle < TICKS_CAL_SETTLE){
        if(update_display){ update_display = FALSE; settle++; }
    }
    g_ambient_left  = average_adc_samples(0u);
    g_ambient_right = average_adc_samples(1u);

    /* Phase 1 – white (emitter ON, on white paper) */
    IR_LED_control(IR_LED_ON);
    set_line(DISP_STATE_LINE, "CAL:WHITE ");
    set_line(DISP_RIGHT_LINE, "Emitter ON");
    set_line(DISP_LEFT_LINE,  "On White  ");
    set_line(DISP_TIMER_LINE, "Place:WHT ");
    settle = RESET_STATE;
    while(settle < TICKS_CAL_SETTLE){
        if(update_display){ update_display = FALSE; settle++; }
    }
    g_white_left  = average_adc_samples(0u);
    g_white_right = average_adc_samples(1u);

    /* Phase 2 – black (emitter ON, user moves car to black tape) */
    set_line(DISP_STATE_LINE, "CAL:BLACK ");
    set_line(DISP_RIGHT_LINE, "Move to   ");
    set_line(DISP_LEFT_LINE,  "BLACK line");
    set_line(DISP_TIMER_LINE, "Place:BLK ");
    settle = RESET_STATE;
    while(settle < TICKS_CAL_SETTLE * 3u){
        if(update_display){ update_display = FALSE; settle++; }
    }
    g_black_left  = average_adc_samples(0u);
    g_black_right = average_adc_samples(1u);

    update_dynamic_threshold();
}

/*==============================================================================
 * update_dynamic_threshold
 *   Subtracts ambient baseline before computing the white/black midpoint.
 *   The ambient-subtracted midpoint is then shifted back up by ambient_avg
 *   so g_threshold is in the same ADC count space as the live readings.
 *==============================================================================*/
static void update_dynamic_threshold(void){

    unsigned int white_avg;
    unsigned int black_avg;
    unsigned int ambient_avg;

    ambient_avg = (g_ambient_left + g_ambient_right) / 2u;
    white_avg   = (g_white_left   + g_white_right)   / 2u;
    black_avg   = (g_black_left   + g_black_right)   / 2u;

    if(white_avg > ambient_avg){ white_avg -= ambient_avg; } else { white_avg = 0u; }
    if(black_avg > ambient_avg){ black_avg -= ambient_avg; } else { black_avg = 0u; }

    if(black_avg > (white_avg + CAL_THRESHOLD_MARGIN)){
        g_threshold = ((white_avg + black_avg) / 2u) + ambient_avg;
    } else {
        g_threshold = BLACK_LINE_THRESHOLD;   /* static fallback */
    }
}

/*==============================================================================
 * Threshold-based detection helpers
 *==============================================================================*/
static unsigned char is_black_left_dyn(void){
    return (ADC_Left_Detect  > g_threshold) ? TRUE : FALSE;
}

static unsigned char is_black_right_dyn(void){
    return (ADC_Right_Detect > g_threshold) ? TRUE : FALSE;
}

static unsigned char get_line_state_dyn(void){
    unsigned char s = LINE_NONE;
    if(is_black_left_dyn())  s |= LINE_LEFT;
    if(is_black_right_dyn()) s |= LINE_RIGHT;
    return s;
}

/*==============================================================================
 * clamp_ccr
 *   Clamps val to [PID_MIN_SPEED, PID_MAX_SPEED] then writes to *ccr.
 *   Prevents PID from driving a wheel to zero (stalling) or past the period.
 *==============================================================================*/
static void clamp_ccr(volatile unsigned int *ccr, unsigned int val){
    if(val < PID_MIN_SPEED){ val = PID_MIN_SPEED; }
    if(val > PID_MAX_SPEED){ val = PID_MAX_SPEED; }
    *ccr = val;
}

/*==============================================================================
 * line_follow_pid
 *
 * PID differential steering using TB3 PWM.
 *
 * Error = right_ADC - left_ADC (signed integer arithmetic)
 *   > 0 : right sensor over more black -> car drifted right -> steer left
 *   < 0 : left  sensor over more black -> car drifted left  -> steer right
 *   = 0 : sensors balanced -> car centered on line edge -> go straight
 *
 * The correction from PID_Update() is added to the left CCR and subtracted
 * from the right CCR, creating a smooth differential that follows the edge
 * of the black line rather than just switching between full-on and full-off.
 *==============================================================================*/
static void line_follow_pid(void){

    int error;
    int correction;
    int right_speed;
    int left_speed;

    /* Signed error: positive = drifted right, negative = drifted left */
    error = (int)ADC_Right_Detect - (int)ADC_Left_Detect;

    /* PID controller step */
    correction = PID_Update(&g_pid, error);

    /* Differential application */
    right_speed = (int)PID_BASE_SPEED - correction;
    left_speed  = (int)PID_BASE_SPEED + correction;

    /* Write clamped values; reverse channels stay off */
    clamp_ccr(&TB3CCR2, (unsigned int)right_speed);   /* R_FORWARD */
    clamp_ccr(&TB3CCR3, (unsigned int)left_speed);    /* L_FORWARD */
    TB3CCR4 = WHEEL_OFF;                               /* R_REVERSE */
    TB3CCR5 = WHEEL_OFF;                               /* L_REVERSE */
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){

    unsigned char state       = STATE_CALIBRATE;
    unsigned int  state_timer = RESET_STATE;

    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();
    Init_ADC();
    Init_Switches();

    MOTORS_STOP();
    MOTORS_STOP_GPIO();
    IR_LED_control(IR_LED_OFF);
    PID_Init(&g_pid);

    set_line(DISP_STATE_LINE, "PROJECT 7 ");
    set_line(DISP_RIGHT_LINE, "Calibrate ");
    set_line(DISP_LEFT_LINE,  "SW1=start ");
    set_line(DISP_TIMER_LINE, "T:  0.0s  ");

    /*==========================================================================
     * Main loop
     *========================================================================*/
    while(ALWAYS){

        Display_Process();

        /*----------------------------------------------------------------------
         * 200 ms tick: advance timer, refresh display during active states
         *--------------------------------------------------------------------*/
        if(update_display){
            update_display = FALSE;

            if(g_timer_active){
                if(g_timer_ticks < 4999u){ g_timer_ticks++; }
            }

            if((state == STATE_INTERCEPT) ||
               (state == STATE_CIRCLE)    ||
               (state == STATE_EXIT)){
                show_detector_values();
                show_timer();
            }
        }

        /*----------------------------------------------------------------------
         * State machine
         *--------------------------------------------------------------------*/
        switch(state){

            /*-- CALIBRATE: three-phase cal, then start DAC ramp -------------*/
            case STATE_CALIBRATE:
                run_calibration();

                HEX_to_BCD(g_threshold);
                display_line[DISP_RIGHT_LINE][0] = 'T';
                display_line[DISP_RIGHT_LINE][1] = 'H';
                display_line[DISP_RIGHT_LINE][2] = ':';
                display_line[DISP_RIGHT_LINE][3] = thousands;
                display_line[DISP_RIGHT_LINE][4] = hundreds;
                display_line[DISP_RIGHT_LINE][5] = tens;
                display_line[DISP_RIGHT_LINE][6] = ones;
                display_line[DISP_RIGHT_LINE][7] = ' ';
                display_line[DISP_RIGHT_LINE][8] = ' ';
                display_line[DISP_RIGHT_LINE][9] = ' ';
                display_line[DISP_RIGHT_LINE][DISPLAY_LINE_LENGTH-1u] = RESET_STATE;

                set_line(DISP_STATE_LINE, "CAL DONE  ");
                set_line(DISP_LEFT_LINE,  "SW1=GO    ");
                set_line(DISP_TIMER_LINE, "T:  0.0s  ");
                display_changed = TRUE;

                /* Begin DAC soft-start ramp; voltage rises while user reads display */
                Init_DAC();

                state = STATE_IDLE;
                break;

            /*-- IDLE: show live ADC, wait for DAC ready and SW1 press -------*/
            case STATE_IDLE:
                show_detector_values();

                if(DAC_ready == FALSE){
                    set_line(DISP_TIMER_LINE, "DAC RAMP  ");
                }

                if(sw1_pressed && (DAC_ready == TRUE)){
                    sw1_pressed    = FALSE;
                    g_timer_ticks  = RESET_STATE;
                    g_timer_active = FALSE;
                    state_timer    = RESET_STATE;
                    PID_Init(&g_pid);
                    state = STATE_DELAY;
                    set_line(DISP_STATE_LINE, "WAITING   ");
                    set_line(DISP_LEFT_LINE,  "          ");
                    set_line(DISP_TIMER_LINE, "T:  0.0s  ");
                }
                break;

            /*-- DELAY: 1 s pause, set cruise voltage, drive forward ---------*/
            case STATE_DELAY:
                state_timer++;
                if(state_timer >= TICKS_1_SEC){
                    state_timer    = RESET_STATE;
                    g_timer_active = TRUE;
                    DAC_Set(DAC_CRUISE);          /* Set controlled speed voltage */
                    MOTORS_PWM_STRAIGHT();
                    state = STATE_INTERCEPT;
                    set_line(DISP_STATE_LINE, "INTERCEPT ");
                }
                break;

            /*-- INTERCEPT: drive forward until line detected ----------------*/
            case STATE_INTERCEPT:
                if(get_line_state_dyn() != LINE_NONE){
                    MOTORS_STOP();
                    state_timer = RESET_STATE;
                    state       = STATE_WAIT;
                    set_line(DISP_STATE_LINE, "WAITING   ");
                }
                break;

            /*-- WAIT: hold 4 s on line, then begin alignment turn -----------*/
            case STATE_WAIT:
                state_timer++;
                if(state_timer >= TICKS_4_SEC){
                    state_timer = RESET_STATE;
                    state       = STATE_ALIGN;
                    MOTOR_TURN_LEFT_GPIO();
                    set_line(DISP_STATE_LINE, "TURNING   ");
                }
                break;

            /*-- ALIGN: rotate until both detectors on black -----------------*/
            case STATE_ALIGN:
                if(get_line_state_dyn() == LINE_BOTH){
                    MOTORS_STOP_GPIO();
                    PID_Init(&g_pid);
                    g_lap_edges        = RESET_STATE;
                    g_prev_right_black = is_black_right_dyn();
                    state_timer        = RESET_STATE;
                    state              = STATE_CIRCLE;
                    set_line(DISP_STATE_LINE, "CIRCLING  ");
                    MOTORS_PWM_STRAIGHT();
                    line_follow_pid();            /* First PID step immediately */
                } else {
                    state_timer++;
                    if(state_timer >= TICKS_ALIGN_TIMEOUT){
                        MOTORS_STOP_GPIO();
                        g_timer_active = FALSE;
                        state = STATE_STOPPED;
                        set_line(DISP_STATE_LINE, "STOPPED   ");
                    }
                }
                break;

            /*-- CIRCLE: PID line follow for two laps ------------------------*/
            case STATE_CIRCLE:
                line_follow_pid();

                /* Count white->black rising edges on right detector */
                {
                    unsigned char cur_right = is_black_right_dyn();
                    if((cur_right == TRUE) && (g_prev_right_black == FALSE)){
                        g_lap_edges++;
                    }
                    g_prev_right_black = cur_right;
                }

                if(g_lap_edges >= LAP_EDGE_COUNT){
                    MOTORS_STOP();
                    state_timer = RESET_STATE;
                    state       = STATE_EXIT;
                    MOTOR_INWARD_TURN_GPIO();
                    set_line(DISP_STATE_LINE, "EXITING   ");
                }
                break;

            /*-- EXIT: inward turn + drive to center, then stop --------------*/
            case STATE_EXIT:
                state_timer++;
                if(state_timer >= TICKS_EXIT_DRIVE){
                    MOTORS_STOP_GPIO();
                    g_timer_active = FALSE;
                    state = STATE_STOPPED;
                    set_line(DISP_STATE_LINE, "STOPPED   ");
                    show_detector_values();
                    show_timer();
                }
                break;

            /*-- STOPPED: freeze; disable converter; SW1 restarts cal --------*/
            case STATE_STOPPED:
                MOTORS_STOP();
                MOTORS_STOP_GPIO();
                IR_LED_control(IR_LED_OFF);
                P2OUT &= ~DAC_ENB;       /* Cut LT1935 to save battery         */

                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    P2OUT |= DAC_ENB;    /* Re-enable converter for next run    */
                    state = STATE_CALIBRATE;
                }
                break;

            default:
                MOTORS_STOP();
                MOTORS_STOP_GPIO();
                state = STATE_IDLE;
                break;
        }

    } /* while(ALWAYS) */
}
