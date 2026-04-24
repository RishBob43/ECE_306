/*------------------------------------------------------------------------------
 * File:        main.c  (Project 10)
 * Target:      MSP430FR2355
 *
 * Description: Project 10 – WiFi-controlled IoT course + autonomous black-line
 *              circle follower.
 *
 * ── IoT Course (Phase 1) ────────────────────────────────────────────────────
 *   Startup:  Auto-connect, display SSID + IP -> calibrate -> "Waiting for input"
 *   Course:   Drive to pads 1-8 via web commands; display "Arrived 0X" on pad.
 *             First command starts a seconds counter displayed on line 3.
 *
 *   Web command format (via TCP port 6767):
 *     ^1234Dtttt   D = F/B/R/L/S/A/E  tttt = time units (500 ms each)
 *     ^1234PX      Pad arrival: X = 1-8, displays "Arrived 0X"
 *     ^1234G       Go autonomous (triggers BL state machine)
 *     ^1234X       Exit circle (triggers BL_EXIT)
 *
 * ── Autonomous Phase (Phase 2) ──────────────────────────────────────────────
 *   State machine with mandatory 10-20 s stops between stages:
 *
 *     BL_START   -> drive forward toward line        display "BL Start  "
 *     BL_PAUSE1  -> 15 s stop after intercept        display "Intercept "
 *     BL_TURN    -> rotate until both on black       display "BL Turn   "
 *     BL_PAUSE2  -> 15 s stop after turn             display "BL Turn   "
 *     BL_TRAVEL  -> follow line to circle            display "BL Travel "
 *     BL_PAUSE3  -> 15 s stop (within first lap)     display "BL Circle "
 *     BL_CIRCLE  -> follow line, count 2 laps        display "BL Circle "
 *     BL_EXIT    -> commanded exit: turn + straight  display "BL Exit   "
 *     BL_STOP    -> stopped, display completion      display "BL Stop   "
 *
 * ── Display Layout ───────────────────────────────────────────────────────────
 *   IoT mode:
 *     Line 0: "Arrived 0X" or blank / BL status in autonomous
 *     Line 1: IP octet 1-2   (e.g. " 10.155  ")
 *     Line 2: IP octet 3-4   (e.g. " .102.202")
 *     Line 3: "Dcmd  NNNs" – last cmd char + seconds counter
 *
 *   BL_STOP final screen:
 *     Line 0: "BL Stop   "
 *     Line 1: "Kachow!   "   (custom completion message)
 *     Line 2: "Good job! "
 *     Line 3: "Time:NNNs "
 *
 * ── Calibration Display ──────────────────────────────────────────────────────
 *   After calibration completes:
 *     Line 0: "Waiting   "
 *     Line 1: "for input "
 *     Line 2: ip_oct12
 *     Line 3: ip_oct34
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
#include "serial.h"
#include "dac.h"

/*==============================================================================
 * Externs
 *==============================================================================*/
extern char                   display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned char display_changed;

/*==============================================================================
 * Motor GPIO macros
 *==============================================================================*/
#define MOTORS_ALL_OFF()    (P6OUT &= ~(R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE))
#define MOTORS_FORWARD()    do { P6OUT |=  (R_FORWARD | L_FORWARD); \
                                 P6OUT &= ~(R_REVERSE | L_REVERSE);  } while(0)
#define MOTORS_REVERSE()    do { P6OUT |=  (R_REVERSE | L_REVERSE); \
                                 P6OUT &= ~(R_FORWARD | L_FORWARD);  } while(0)
/*  Turn RIGHT – LEFT wheel drives, right wheel off  */
#define MOTOR_TURN_RIGHT()  do { P6OUT |=  L_FORWARD; \
                                 P6OUT &= ~(R_FORWARD | R_REVERSE | L_REVERSE); } while(0)
/*  Turn LEFT  – RIGHT wheel drives, left wheel off */
#define MOTOR_TURN_LEFT()   do { P6OUT |=  R_FORWARD; \
                                 P6OUT &= ~(L_FORWARD | R_REVERSE | L_REVERSE); } while(0)
/*  Spin for IoT course turns */
#define MOTOR_IOT_RIGHT()   do { P6OUT |=  (L_FORWARD | R_REVERSE); \
                                 P6OUT &= ~(R_FORWARD | L_REVERSE);  } while(0)
#define MOTOR_IOT_LEFT()    do { P6OUT |=  (R_FORWARD | L_REVERSE); \
                                 P6OUT &= ~(L_FORWARD | R_REVERSE);  } while(0)
/*  Exit turn: spin away from circle (away from pad 8 direction) */
#define MOTOR_EXIT_TURN()   do { P6OUT |=  (R_FORWARD | L_REVERSE); \
                                 P6OUT &= ~(L_FORWARD | R_REVERSE);  } while(0)

/*==============================================================================
 * IoT / motor timing
 *==============================================================================*/
#define TIME_UNIT_MS            (500u)    /* each 'tttt' unit = 500 ms           */


/*==============================================================================
 * Security PIN
 *==============================================================================*/
#define SECRET_PIN              "1234"
#define SECRET_PIN_LEN          (4u)

/*==============================================================================
 * Autonomous BL state identifiers
 *==============================================================================*/
#define BL_NONE                 (0u)
#define BL_START                (1u)   /* driving toward line                    */
#define BL_PAUSE1               (2u)   /* 15 s stop: "Intercept" displayed       */
#define BL_TURN                 (3u)   /* rotating until both sensors on black   */
#define BL_PAUSE2               (4u)   /* 15 s stop after turn                   */
#define BL_TRAVEL               (5u)   /* following line toward circle           */
#define BL_PAUSE3               (6u)   /* 15 s stop to show "BL Circle"          */
#define BL_CIRCLE               (7u)   /* following circle, counting laps        */
#define BL_EXIT                 (8u)   /* exit commanded: turn + straight        */
#define BL_STOP                 (9u)   /* fully stopped, show completion screen  */

/*==============================================================================
 * Timing constants (1 tick = 200 ms from TB0 CCR0)
 *==============================================================================*/
#define TICKS_15_SEC            (75u)   /* 15 s mandatory pause                  */
#define TICKS_EXIT_TURN         (10u)   /* ~2 s exit spin                        */
#define TICKS_CAL_SETTLE        (15u)   /* 3 s settle per calibration phase      */
#define SPLASH_TICKS            (25u)   /* 5 s splash                            */

/*==============================================================================
 * Calibration
 *==============================================================================*/
#define CAL_THRESHOLD_MARGIN    (50u)

/*==============================================================================
 * White confirmation filter
 *   The sensor must see white for WHITE_CONFIRM_COUNT consecutive ADC updates
 *   (~10 ms each) before black detection is re-armed.  This prevents floor
 *   specks and debris from triggering a false black reading.
 *   Increase to 8-10 if false triggers still occur.
 *==============================================================================*/
#define WHITE_CONFIRM_COUNT     (5u)

/*==============================================================================
 * Lost-line recovery
 *   If neither sensor sees the line for LOST_LINE_LIMIT consecutive
 *   line_follow_step() calls the car spins back toward the circle rather
 *   than driving straight off the mat.
 *   LOST_LINE_LIMIT is in units of main-loop passes (not ms ticks).
 *==============================================================================*/
#define LOST_LINE_LIMIT         (20u)

/*==============================================================================
 * Module globals – calibration
 *==============================================================================*/
static unsigned int  g_white_left    = 0u;
static unsigned int  g_white_right   = 0u;
static unsigned int  g_black_left    = 0u;
static unsigned int  g_black_right   = 0u;
static unsigned int  g_threshold     = BLACK_LINE_THRESHOLD;

/*==============================================================================
 * Module globals – autonomous
 *==============================================================================*/
static unsigned char g_bl_state        = BL_NONE;
static unsigned int  g_bl_timer        = 0u;
static unsigned char g_lap_edges       = 0u;
static unsigned char g_prev_right_black = FALSE;
static unsigned char g_exit_phase      = 0u;  /* 0=turn, 1=straight             */
static unsigned char g_exit_requested  = FALSE;

/*==============================================================================
 * Module globals – white-confirmation filter
 *   Separate counters for each sensor.  Counts consecutive ADC-updated passes
 *   where the sensor reads white.  Black is trusted immediately; white is only
 *   accepted after WHITE_CONFIRM_COUNT consecutive white readings.
 *==============================================================================*/
static unsigned char g_white_confirm_L = 0u;
static unsigned char g_white_confirm_R = 0u;

/*==============================================================================
 * Module globals – lost-line recovery counter
 *==============================================================================*/
static unsigned int  g_lost_line_count = 0u;

/*==============================================================================
 * Module globals – IoT course
 *==============================================================================*/
static char     ssid_display[11] = "          ";
static char     ip_oct12[11]     = "          ";
static char     ip_oct34[11]     = "          ";

static unsigned char  g_on_pad         = FALSE;  /* TRUE when Arrived cmd received */
static unsigned char  g_pad_number     = 0u;
static unsigned char  g_course_active  = FALSE;  /* TRUE after first command       */
static unsigned int   g_course_secs    = 0u;     /* seconds counter                */
static unsigned int   g_sec_ticks      = 0u;     /* ticks toward next second       */
static char           g_last_cmd[6]    = "     "; /* 5 relevant chars + null       */
static unsigned char  g_autonomous     = FALSE;  /* TRUE when G command received   */
static unsigned char  g_cal_done       = FALSE;

/*==============================================================================
 * Helper: center_string – write text centered into a 10-char buffer
 *==============================================================================*/
static void center_string(const char *text, unsigned int len, char *out){
    unsigned int pad_left, pad_right, i, pos;
    if(len > 10u){ len = 10u; }
    pad_left  = (10u - len) / 2u;
    pad_right = 10u - len - pad_left;
    pos = 0u;
    for(i = 0u; i < pad_left;  i++){ out[pos++] = ' '; }
    for(i = 0u; i < len;       i++){ out[pos++] = text[i]; }
    for(i = 0u; i < pad_right; i++){ out[pos++] = ' '; }
    out[10] = '\0';
}

/*==============================================================================
 * Helper: set_line – copy msg into display_line[line] and flag changed
 *==============================================================================*/
static void set_line(int line, const char *msg){
    strncpy(display_line[line], msg, 10u);
    display_line[line][10] = '\0';
    display_changed = TRUE;
}

/*==============================================================================
 * Helper: flag_display
 *==============================================================================*/
static void flag_display(void){
    display_changed = TRUE;
}

/*==============================================================================
 * show_course_display
 * Updates all four LCD lines for IoT remote-control mode.
 *   Line 0: "Arrived 0X" if on pad, else "          " (or BL status)
 *   Line 1: ip_oct12
 *   Line 2: ip_oct34
 *   Line 3: last cmd (5 chars) + "  " + seconds counter
 *==============================================================================*/
static void show_course_display(void){
    char line3[11];
    unsigned int s = g_course_secs;
    unsigned char d2, d1, d0;

    /* Line 0: pad arrived or blank (BL states handled by caller) */
    if(!g_autonomous){
        if(g_on_pad){
            char pad_msg[11] = "Arrived 0 ";
            pad_msg[9] = (char)('0' + g_pad_number);
            set_line(0, pad_msg);
        } else {
            set_line(0, "          ");
        }
    }

    /* Lines 1-2: IP address */
    set_line(1, ip_oct12);
    set_line(2, ip_oct34);

    /* Line 3: "Dcmd  NNNs"  (last cmd char, then seconds) */
    d2 = (unsigned char)(s / 100u);
    s %= 100u;
    d1 = (unsigned char)(s / 10u);
    d0 = (unsigned char)(s % 10u);

    line3[0] = g_last_cmd[0];
    line3[1] = g_last_cmd[1];
    line3[2] = g_last_cmd[2];
    line3[3] = g_last_cmd[3];
    line3[4] = g_last_cmd[4];
    line3[5] = ' ';
    line3[6] = (char)('0' + d2);
    line3[7] = (char)('0' + d1);
    line3[8] = (char)('0' + d0);
    line3[9] = 's';
    line3[10] = '\0';
    set_line(3, line3);
}

/*==============================================================================
 * show_waiting_display – shown after calibration, before first command
 *==============================================================================*/
static void show_waiting_display(void){
    set_line(0, "Waiting   ");
    set_line(1, "for input ");
    set_line(2, ip_oct12);
    set_line(3, ip_oct34);
    Display_Update(0,0,0,0);
}

/*==============================================================================
 * show_bl_stop_display – final completion screen
 *==============================================================================*/
static void show_bl_stop_display(void){
    char time_line[11];
    unsigned int s = g_course_secs;
    unsigned char d2, d1, d0;

    d2 = (unsigned char)(s / 100u);
    s %= 100u;
    d1 = (unsigned char)(s / 10u);
    d0 = (unsigned char)(s % 10u);

    time_line[0] = 'T'; time_line[1] = 'i'; time_line[2] = 'm';
    time_line[3] = 'e'; time_line[4] = ':';
    time_line[5] = (char)('0' + d2);
    time_line[6] = (char)('0' + d1);
    time_line[7] = (char)('0' + d0);
    time_line[8] = 's'; time_line[9] = ' '; time_line[10] = '\0';

    set_line(0, "BL Stop   ");
    set_line(1, "Kachow!   ");
    set_line(2, "Good job! ");
    set_line(3, time_line);
    Display_Update(0,0,0,0);
}

/*==============================================================================
 * Calibration helpers
 *==============================================================================*/
static unsigned int average_adc_samples(unsigned char channel_flag){
    unsigned int  sum   = 0u;
    unsigned char count = 0u;
    while(count < NUM_CAL_SAMPLES){
        if(ADC_updated){
            ADC_updated = FALSE;
            sum += (channel_flag == 0u) ? ADC_Left_Detect : ADC_Right_Detect;
            count++;
        }
    }
    return (sum / NUM_CAL_SAMPLES);
}

static void update_dynamic_threshold(void){
    unsigned int white_avg = (g_white_left  + g_white_right)  / 2u;
    unsigned int black_avg = (g_black_left  + g_black_right)  / 2u;
    if(black_avg > (white_avg + CAL_THRESHOLD_MARGIN)){
        g_threshold = (white_avg + black_avg) / 2u;
    } else {
        g_threshold = BLACK_LINE_THRESHOLD;
    }
}

static void run_calibration(void){
    unsigned char settle;

    /* Phase 0: ambient (emitter OFF) */
    IR_LED_control(IR_LED_OFF);
    set_line(0, "CAL:AMBT  ");
    set_line(1, "Emitter   ");
    set_line(2, "OFF       ");
    set_line(3, "Place:WHT ");
    Display_Update(0,0,0,0);
    settle = 0u;
    while(settle < TICKS_CAL_SETTLE){
        if(update_display_count > 0u){
            update_display_count--;
            update_display = TRUE;
            settle++;
        }
        Display_Process();
    }
    average_adc_samples(0u);  /* discard ambient */
    average_adc_samples(1u);

    /* Phase 1: white baseline (emitter ON) */
    IR_LED_control(IR_LED_ON);
    set_line(0, "CAL:WHITE ");
    set_line(1, "Emitter ON");
    set_line(2, "On White  ");
    set_line(3, "Place:WHT ");
    Display_Update(0,0,0,0);
    settle = 0u;
    while(settle < TICKS_CAL_SETTLE){
            if(update_display_count > 0u){
                update_display_count--;
                update_display = TRUE;
                settle++;
            }
            Display_Process();
    }
    g_white_left  = average_adc_samples(0u);
    g_white_right = average_adc_samples(1u);

    /* Phase 2: black line (emitter ON) */
    set_line(0, "CAL:BLACK ");
    set_line(1, "Move to   ");
    set_line(2, "BLACK line");
    set_line(3, "Place:BLK ");
    Display_Update(0,0,0,0);
    settle = 0u;
    while(settle < TICKS_CAL_SETTLE * 3u){
        if(update_display_count > 0u){
            update_display_count--;
            update_display = TRUE;
            settle++;
        }
        Display_Process();
    }
    g_black_left  = average_adc_samples(0u);
    g_black_right = average_adc_samples(1u);

    update_dynamic_threshold();
    IR_LED_control(IR_LED_ON);   /* leave on for driving */
    g_cal_done = TRUE;
}

/*==============================================================================
 * Line detection – base dynamic threshold (no filter)
 *==============================================================================*/
static unsigned char is_black_left_raw(void){
    return (ADC_Left_Detect  > g_threshold) ? TRUE : FALSE;
}
static unsigned char is_black_right_raw(void){
    return (ADC_Right_Detect > g_threshold) ? TRUE : FALSE;
}

/*==============================================================================
 * Line detection – white-confirmation filtered versions
 *
 *   Black is trusted immediately (fast reaction to finding the line).
 *   White is only accepted after WHITE_CONFIRM_COUNT consecutive white
 *   readings, preventing floor specks from briefly interrupting detection.
 *
 *   Call these instead of is_black_left_raw() / is_black_right_raw() in all
 *   line-following and lap-counting code.
 *==============================================================================*/
static unsigned char is_black_left_filtered(void){
    if(is_black_left_raw()){
        g_white_confirm_L = 0u;   /* reset white streak – definitely black now */
        return TRUE;
    }
    /* Reading is white; only accept it after enough consecutive white reads   */
    if(g_white_confirm_L < WHITE_CONFIRM_COUNT){
        g_white_confirm_L++;
        return TRUE;              /* still treating as "black" until confirmed  */
    }
    return FALSE;                 /* confirmed white                            */
}

static unsigned char is_black_right_filtered(void){
    if(is_black_right_raw()){
        g_white_confirm_R = 0u;
        return TRUE;
    }
    if(g_white_confirm_R < WHITE_CONFIRM_COUNT){
        g_white_confirm_R++;
        return TRUE;
    }
    return FALSE;
}

/*==============================================================================
 * get_line_state_dyn – uses filtered sensor readings
 *==============================================================================*/
static unsigned char get_line_state_dyn(void){
    unsigned char s = LINE_NONE;
    if(is_black_left_filtered())  s |= LINE_LEFT;
    if(is_black_right_filtered()) s |= LINE_RIGHT;
    return s;
}

/*==============================================================================
 * line_follow_step – reactive two-sensor circle follower
 *
 * Improvements over the original bang-bang follower:
 *
 *   1. LINE_BOTH applies an inside bias (MOTOR_TURN_RIGHT) so the car
 *      hugs the inside of the circle instead of straightening out and
 *      overshooting to the outside.
 *      → Swap MOTOR_TURN_RIGHT ↔ MOTOR_TURN_LEFT if your circle runs CCW.
 *
 *   2. Lost-line recovery: if neither sensor sees black for LOST_LINE_LIMIT
 *      consecutive calls the car spins toward the circle (MOTOR_TURN_RIGHT)
 *      instead of driving straight off the mat.
 *      → Swap to MOTOR_TURN_LEFT if your circle runs CCW.
 *
 *   Filter state (g_white_confirm_L/R, g_lost_line_count) is reset at the
 *   start of BL_CIRCLE so it does not carry stale data from earlier states.
 *==============================================================================*/
static void line_follow_step(void){
    unsigned char state = get_line_state_dyn();

    if(state == LINE_NONE){
        g_lost_line_count++;
        if(g_lost_line_count > LOST_LINE_LIMIT){
            /*------------------------------------------------------------------
             * Line lost too long – spin back toward the circle.
             * MOTOR_TURN_RIGHT keeps a CW circle; swap to LEFT for CCW.
             *----------------------------------------------------------------*/
            MOTOR_TURN_RIGHT();
        } else {
            /* Grace period: coast straight hoping to re-acquire line          */
            MOTORS_FORWARD();
        }
        return;
    }

    /* Line seen – reset lost counter */
    g_lost_line_count = 0u;

    switch(state){
        case LINE_BOTH:
            /*------------------------------------------------------------------
             * Both sensors on the line.  Apply a gentle inside bias so the
             * car stays on the circular arc rather than cutting straight across.
             * Change MOTOR_TURN_RIGHT to MOTOR_TURN_LEFT for a CCW circle.
             *----------------------------------------------------------------*/
            MOTOR_TURN_RIGHT();
            break;

        case LINE_LEFT:
            /* Left sensor on black – drifting right, correct left            */
            MOTOR_TURN_LEFT();
            break;

        case LINE_RIGHT:
            /* Right sensor on black – drifting left, correct right           */
            MOTOR_TURN_RIGHT();
            break;

        default:
            MOTOR_TURN_RIGHT();
            break;
    }
}

/*==============================================================================
 * IoT message helpers
 *==============================================================================*/
static unsigned char ch_match(const char *s, const char *p, unsigned int n){
    unsigned int i;
    for(i = 0u; i < n; i++){ if(s[i] != p[i]){ return FALSE; } }
    return TRUE;
}

static unsigned int parse_uint(const char *s, unsigned int *consumed){
    unsigned int val = 0u, i = 0u;
    while(s[i] >= '0' && s[i] <= '9'){
        val = val * 10u + (unsigned int)(s[i] - '0');
        i++;
    }
    *consumed = i;
    return val;
}

static void parse_cwjap(const char *line){
    unsigned int i = 0u, j = 0u;
    char ssid_raw[11];
    while(line[i] != '\0' && line[i] != '"'){ i++; }
    if(line[i] == '\0'){ return; }
    i++;
    while(line[i] != '"' && line[i] != '\0' && j < 10u){
        ssid_raw[j++] = line[i++];
    }
    ssid_raw[j] = '\0';
    if(j > 0u){ center_string(ssid_raw, j, ssid_display); }
}

static void parse_cifsr(const char *line){
    unsigned int i = 0u, j = 0u, dot_count = 0u;
    char ip[16];
    char half1[8], half2[8];
    unsigned int h1len, h2len;

    while(line[i] != '\0'){
        if(ch_match(&line[i], "STAIP", 5u)){ break; }
        i++;
    }
    if(line[i] == '\0'){ return; }

    while(line[i] != '"' && line[i] != '\0'){ i++; }
    if(line[i] == '\0'){ return; }
    i++;

    j = 0u;
    while(line[i] != '"' && line[i] != '\0' && j < 15u){
        ip[j++] = line[i++];
    }
    ip[j] = '\0';
    if(j == 0u){ return; }

    for(i = 0u; i < j; i++){
        if(ip[i] == '.'){ dot_count++; }
        if(dot_count == 2u){
            h1len = i;
            h2len = j - i - 1u;
            if(h1len > 7u){ h1len = 7u; }
            if(h2len > 7u){ h2len = 7u; }
            strncpy(half1, ip,        h1len); half1[h1len] = '\0';
            strncpy(half2, &ip[i+1u], h2len); half2[h2len] = '\0';
            center_string(half1, h1len, ip_oct12);
            center_string(half2, h2len, ip_oct34);
            return;
        }
    }
    center_string(ip, j, ip_oct12);
}
/*==============================================================================
 * parse_web_command
 *
 * Supported commands after PIN (^1234):
 *   F tttt  – forward  tttt × 500 ms
 *   B tttt  – reverse  tttt × 500 ms
 *   R tttt  – right    tttt × 500 ms
 *   L tttt  – left     tttt × 500 ms
 *   S 0     – stop immediately
 *   P X     – arrived at pad X  (1-8)
 *   G 0     – go autonomous (start BL sequence)
 *   X 0     – exit circle   (trigger BL_EXIT)
 *==============================================================================*/
static unsigned char g_motor_direction  = 'S';
static unsigned int  g_motor_remaining  = 0u;

static void parse_web_command(const char *payload){
    unsigned int i = 0u, consumed = 0u, units = 0u;
    char dir;

    /* Find '^' */
    while(payload[i] != '\0' && payload[i] != '^'){ i++; }
    if(payload[i] == '\0'){ return; }
    i++;

    /* Check PIN */
    if(!ch_match(&payload[i], SECRET_PIN, SECRET_PIN_LEN)){
        UCA1_Transmit_String("BAD PIN\r\n", 9u);
        return;
    }
    i += SECRET_PIN_LEN;

    dir   = payload[i++];
    units = parse_uint(&payload[i], &consumed);

    /* Build 5-char last-command display: "Dtttt" */
    g_last_cmd[0] = dir;
    g_last_cmd[1] = (units >= 1000u) ? (char)('0' + units/1000u) : ' ';
    g_last_cmd[2] = (units >=  100u) ? (char)('0' + (units%1000u)/100u) : '0';
    g_last_cmd[3] = (char)('0' + (units % 100u) / 10u);
    g_last_cmd[4] = (char)('0' + units % 10u);
    g_last_cmd[5] = '\0';

    /* Start seconds counter on first command */
    if(!g_course_active){
        g_course_active = TRUE;
        g_course_secs   = 0u;
        g_sec_ticks     = 0u;
    }

    switch(dir){
        case 'F': case 'f':
        case 'B': case 'b':
        case 'R': case 'r':
        case 'L': case 'l':
            g_motor_direction = (unsigned char)dir;
            g_motor_remaining = units;
            g_on_pad = FALSE;
            break;

        case 'S': case 's':
            MOTORS_ALL_OFF();
            g_motor_remaining = 0u;
            g_on_pad = FALSE;
            break;

        case 'P': case 'p':
            /* Pad arrival: units holds pad number */
            g_pad_number = (unsigned char)units;
            g_on_pad     = TRUE;
            break;

        case 'G': case 'g':
            /* Go autonomous – start BL sequence */
            if(!g_autonomous){
                g_autonomous = TRUE;
                g_on_pad     = FALSE;
                MOTORS_ALL_OFF();
                g_bl_state  = BL_START;
                g_bl_timer  = 0u;
                g_lap_edges = 0u;
                g_prev_right_black = is_black_right_filtered();
                g_exit_requested   = FALSE;
                g_exit_phase       = 0u;
                /* Reset filter state for clean start */
                g_white_confirm_L  = 0u;
                g_white_confirm_R  = 0u;
                g_lost_line_count  = 0u;
                set_line(0, "BL Start  ");
                show_course_display();
                Display_Update(0,0,0,0);
                MOTORS_FORWARD();
            }
            break;

        case 'X': case 'x':
            /* Exit circle command */
            if(g_autonomous && (g_bl_state == BL_CIRCLE || g_bl_state == BL_PAUSE3)){
                g_exit_requested = TRUE;
            }
            break;

        default:
            break;
    }

    UCA1_Transmit_String("CMD OK\r\n", 8u);
    show_course_display();
    Display_Update(0,0,0,0);
}

static void process_iot_msg(void){
    unsigned char ridx = iot_rx_read_idx;
    const char *msg;
    unsigned int k;

    /* nothing ready */
    if(!iot_msg_ready[ridx]){ return; }

    msg = (const char *)iot_rx_msg[ridx];

    if(ch_match(msg, "+CWJAP:", 7u)){
        parse_cwjap(msg);
    } else if(ch_match(msg, "+CIFSR:", 7u)){
        parse_cifsr(msg);
    } else if(ch_match(msg, "+IPD", 4u)){
        k = 0u;
        while(msg[k] != '\0' && msg[k] != ':'){ k++; }
        if(msg[k] == ':'){ parse_web_command(&msg[k + 1u]); }
    }

    /* mark slot consumed and advance read index */
    iot_msg_ready[ridx] = FALSE;
    iot_rx_read_idx = (unsigned char)((ridx + 1u) % IOT_MSG_COUNT);
    /* no need to disable/re-enable UCA0 RX – ISR runs freely now */
}
/*==============================================================================
 * process_fram_cmd – handle PC backdoor ^ commands
 *==============================================================================*/
static void process_fram_cmd(void){
    if(!fram_cmd_ready){ return; }
    fram_cmd_ready = FALSE;
    switch(fram_cmd_buf[0]){
        case '^': UCA1_Transmit_String("I'm here\r\n", 10u); break;
        case 'F': case 'f':
            Set_Baud_UCA0(BAUD_115200);
            UCA1_Transmit_String("115200\r\n", 8u);
            break;
        case 'S': case 's':
            Set_Baud_UCA0(BAUD_9600);
            UCA1_Transmit_String("9600\r\n", 6u);
            break;
        default:
            UCA1_Transmit_String("?\r\n", 3u);
            break;
    }
}

/*==============================================================================
 * execute_motor_unit – one TIME_UNIT_MS burst of IoT motor motion
 *==============================================================================*/
static void execute_motor_unit(void){
    if(g_motor_remaining == 0u){
        MOTORS_ALL_OFF();
        return;
    }

    /* Apply direction every pass so motor stays on between ticks */
    switch(g_motor_direction){
        case 'F': case 'f': MOTORS_FORWARD();    break;
        case 'B': case 'b': MOTORS_REVERSE();    break;
        case 'R': case 'r': MOTOR_IOT_RIGHT();   break;
        case 'L': case 'l': MOTOR_IOT_LEFT();    break;
        default:            MOTORS_ALL_OFF();    break;
    }

    /* Each unit = 1 tick = 200 ms – only decrement on tick */
    if(tick_200ms){
        g_motor_remaining--;
        if(g_motor_remaining == 0u){
            MOTORS_ALL_OFF();
        }
    }
}

/*==============================================================================
 * bl_state_machine
 *
 * Called once per main loop pass when g_autonomous == TRUE.
 * Uses update_display tick for timing.
 * tick_consumed: TRUE when the current pass consumed a 200 ms tick.
 *==============================================================================*/
static void bl_state_machine(unsigned char tick_consumed){

    switch(g_bl_state){

        /*----------------------------------------------------------------------
         * BL_START – drive forward until either sensor detects line
         *--------------------------------------------------------------------*/
        case BL_START:
            if(get_line_state_dyn() != LINE_NONE){
                MOTORS_ALL_OFF();
                g_bl_state = BL_PAUSE1;
                g_bl_timer = 0u;
                set_line(0, "Intercept ");
                show_course_display();
                Display_Update(0,0,0,0);
            }
            break;

        /*----------------------------------------------------------------------
         * BL_PAUSE1 – 15 s mandatory stop, display "Intercept"
         *--------------------------------------------------------------------*/
        case BL_PAUSE1:
            if(tick_consumed){ g_bl_timer++; }
            if(g_bl_timer >= TICKS_15_SEC){
                g_bl_timer = 0u;
                g_bl_state = BL_TURN;
                set_line(0, "BL Turn   ");
                show_course_display();
                Display_Update(0,0,0,0);
                MOTOR_TURN_LEFT();    /* begin rotating to align */
            }
            break;

        /*----------------------------------------------------------------------
         * BL_TURN – rotate until BOTH sensors on black
         *--------------------------------------------------------------------*/
        case BL_TURN:
            if(get_line_state_dyn() == LINE_BOTH){
                MOTORS_ALL_OFF();
                g_bl_state = BL_PAUSE2;
                g_bl_timer = 0u;
                /* Keep "BL Turn" on display during the pause */
                show_course_display();
                Display_Update(0,0,0,0);
            } else {
                if(tick_consumed){ g_bl_timer++; }
                if(g_bl_timer >= TICKS_ALIGN_TIMEOUT){
                    /* Safety: give up aligning, proceed anyway */
                    MOTORS_ALL_OFF();
                    g_bl_state = BL_PAUSE2;
                    g_bl_timer = 0u;
                    show_course_display();
                    Display_Update(0,0,0,0);
                }
            }
            break;

        /*----------------------------------------------------------------------
         * BL_PAUSE2 – 15 s stop after turn, display "BL Turn"
         *--------------------------------------------------------------------*/
        case BL_PAUSE2:
            if(tick_consumed){ g_bl_timer++; }
            if(g_bl_timer >= TICKS_15_SEC){
                g_bl_timer = 0u;
                g_bl_state = BL_TRAVEL;
                set_line(0, "BL Travel ");
                show_course_display();
                Display_Update(0,0,0,0);
                /* Start following – will naturally enter circle */
            }
            break;

        /*----------------------------------------------------------------------
         * BL_TRAVEL – follow line toward circle.
         * Transition to BL_PAUSE3 when we detect we've entered the circle
         * (simple heuristic: timer counts a fixed travel window then pauses).
         * Using a lap-entry edge: once the right sensor crosses white->black
         * once while moving, we consider the circle entered.
         *--------------------------------------------------------------------*/
        case BL_TRAVEL:
            line_follow_step();
            if(tick_consumed){ g_bl_timer++; }
            {
                unsigned char cur = is_black_right_filtered();
                if((cur == TRUE) && (g_prev_right_black == FALSE)){
                    /* First rising edge after travel start = circle entry */
                    g_lap_edges++;
                }
                g_prev_right_black = cur;
            }
            /* Enter pause when first edge seen OR after 60-tick (12s) safety */
            if(g_lap_edges >= 1u || g_bl_timer >= 60u){
                MOTORS_ALL_OFF();
                g_bl_state  = BL_PAUSE3;
                g_bl_timer  = 0u;
                g_lap_edges = 0u;
                g_prev_right_black = is_black_right_filtered();
                set_line(0, "BL Circle ");
                show_course_display();
                Display_Update(0,0,0,0);
            }
            break;

        /*----------------------------------------------------------------------
         * BL_PAUSE3 – 15 s stop to display "BL Circle" before full lap
         *--------------------------------------------------------------------*/
        case BL_PAUSE3:
            if(tick_consumed){ g_bl_timer++; }
            /* Allow exit command during pause */
            if(g_exit_requested){
                g_exit_requested = FALSE;
                g_bl_state  = BL_EXIT;
                g_bl_timer  = 0u;
                g_exit_phase = 0u;
                set_line(0, "BL Exit   ");
                show_course_display();
                Display_Update(0,0,0,0);
                MOTOR_EXIT_TURN();
                break;
            }
            if(g_bl_timer >= TICKS_15_SEC){
                g_bl_timer  = 0u;
                g_bl_state  = BL_CIRCLE;
                g_lap_edges = 0u;
                /* Reset filter state so stale TRAVEL readings don't affect   */
                /* the lap-edge counter on the very first circle pass.         */
                g_white_confirm_L  = 0u;
                g_white_confirm_R  = 0u;
                g_lost_line_count  = 0u;
                g_prev_right_black = is_black_right_filtered();
                /* Already showing "BL Circle" – continue following */
            }
            break;

        /*----------------------------------------------------------------------
         * BL_CIRCLE – follow circle, count laps
         *   line_follow_step() now uses the filtered sensor readings and the
         *   inside-bias + lost-line recovery logic defined above.
         *--------------------------------------------------------------------*/
        case BL_CIRCLE:
            line_follow_step();
            {
                unsigned char cur = is_black_right_filtered();
                if((cur == TRUE) && (g_prev_right_black == FALSE)){
                    g_lap_edges++;
                }
                g_prev_right_black = cur;
            }
            /* Check for exit command (set by parse_web_command) */
            if(g_exit_requested){
                g_exit_requested = FALSE;
                MOTORS_ALL_OFF();
                g_bl_state   = BL_EXIT;
                g_bl_timer   = 0u;
                g_exit_phase = 0u;
                set_line(0, "BL Exit   ");
                show_course_display();
                Display_Update(0,0,0,0);
                MOTOR_EXIT_TURN();
            }
            break;

        /*----------------------------------------------------------------------
         * BL_EXIT – turn then drive straight > 2 feet
         *   Phase 0: spin away from circle for TICKS_EXIT_TURN
         *   Phase 1: drive straight for TICKS_EXIT_DRIVE then stop
         *--------------------------------------------------------------------*/
        case BL_EXIT:
            if(tick_consumed){ g_bl_timer++; }
            if(g_exit_phase == 0u){
                if(g_bl_timer >= TICKS_EXIT_TURN){
                    g_exit_phase = 1u;
                    g_bl_timer   = 0u;
                    MOTORS_FORWARD();
                }
            } else {
                if(g_bl_timer >= TICKS_EXIT_DRIVE){
                    MOTORS_ALL_OFF();
                    g_bl_state = BL_STOP;
                    g_bl_timer = 0u;
                    show_bl_stop_display();
                }
            }
            break;

        /*----------------------------------------------------------------------
         * BL_STOP – course complete, display final screen; stay here
         *--------------------------------------------------------------------*/
        case BL_STOP:
            /* Motors off, nothing more to do */
            IR_LED_control(IR_LED_OFF);
            break;

        default:
            MOTORS_ALL_OFF();
            g_bl_state = BL_NONE;
            break;
    }
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){
    unsigned char tick_200ms = FALSE;
    unsigned int  state_timer = 0u;

    /* ── Unlock GPIO ── */
    PM5CTL0 &= ~LOCKLPM5;

    /* ── Hardware init ── */
    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    /* DAC init + soft-start ramp */
    Init_DAC();
    /* Wait for ramp to complete (DAC_Ramp_Step disables TBIE when done) */
    while(!DAC_ready){
        /* TB0 overflow ISR is running the ramp; foreground just waits */
    }
    /* Set cruise voltage */
    DAC_Set(DAC_CRUISE);
    Init_LCD();
    Init_ADC();
    Init_Timer_B1();


    MOTORS_ALL_OFF();
    IR_LED_control(IR_LED_OFF);

    /* ── Serial (IOT + PC backdoor) ── */
    Init_Serial_UCA0(BAUD_115200);
    Init_Serial_UCA1(BAUD_115200);

    /* ── Splash ── */
    set_line(0, "ECE306    ");
    set_line(1, "NCSU P10  ");
    set_line(2, "Kachow!   ");
    set_line(3, "Port:6767 ");
    Display_Update(0,0,0,0);

    /* ── IOT init ── */
    state_timer = 0u;
    /* Wait ~5 s splash */
    while(state_timer < SPLASH_TICKS){
        if(update_display_count > 0u){
            update_display_count--;
            update_display = TRUE;
            state_timer++;
        }
        Display_Process();
    }

    set_line(0, "Init IOT  ");
    set_line(1, "          ");
    set_line(2, "          ");
    set_line(3, "          ");
    Display_Update(0,0,0,0);

    Init_IOT();

    /* ── Query SSID + IP ── */
    {
        unsigned int  iot_timer  = 0u;

        while(iot_timer < 150u){   /* 30 s max – ESP8266 needs time to connect */
            if(update_display_count > 0u){
                update_display_count--;
                update_display = TRUE;
                iot_timer++;
            }
            Display_Process();

            while(iot_msg_ready[iot_rx_read_idx]){
                process_iot_msg();
            }

            /* Send CIFSR later to give ESP8266 time to get an IP */
            if(iot_timer >= 40u){
                UCA0_Transmit_String("AT+CIFSR\r\n", 10u);

            }
            if(ip_oct12[0] != ' '){ break; }

        }
    }
    /* ── Calibration ── */
    run_calibration();

    /* ── Show "Waiting for input" ── */
    show_waiting_display();

    /*==========================================================================
     * Main loop
     *========================================================================*/
    while(ALWAYS){

        /*----------------------------------------------------------------------
         * 200 ms tick
         *--------------------------------------------------------------------*/
        tick_200ms = FALSE;
        if(update_display_count > 0u){
            update_display_count--;
            update_display = TRUE;
            tick_200ms     = TRUE;

            /* Seconds counter (only while course active) */
            if(g_course_active && g_bl_state != BL_STOP){
                g_sec_ticks++;
                if(g_sec_ticks >= 5u){   /* 5 × 200 ms = 1 s */
                    g_sec_ticks = 0u;
                    if(g_course_secs < 999u){ g_course_secs++; }
                }
            }
        }

        Display_Process();

        /*----------------------------------------------------------------------
         * IOT message handler
         *--------------------------------------------------------------------*/
        /* drain all pending messages each pass */
            while(iot_msg_ready[iot_rx_read_idx]){
                process_iot_msg();
            }

        /*----------------------------------------------------------------------
         * FRAM ^ command handler
         *--------------------------------------------------------------------*/
        process_fram_cmd();

        /*----------------------------------------------------------------------
         * Autonomous BL state machine
         *--------------------------------------------------------------------*/
        if(g_autonomous){
            bl_state_machine(tick_200ms);

            /* Refresh course display every tick during autonomous phases */
            if(tick_200ms && g_bl_state != BL_STOP){
                show_course_display();
                Display_Update(0,0,0,0);
            }

        } else {
            /*------------------------------------------------------------------
             * IoT remote-control mode: execute queued motor unit
             *----------------------------------------------------------------*/
            if(g_motor_remaining > 0u){
                execute_motor_unit();
            }

            /* Refresh display every tick */
            if(tick_200ms && g_course_active){
                show_course_display();
                Display_Update(0,0,0,0);
            }
        }

    } /* while(ALWAYS) */
}
