/*------------------------------------------------------------------------------
 * File:        main.c  (Project 9)
 * Target:      MSP430FR2355
 *
 * Description: IOT bridge using Init_IOT() / iot_rx_msg approach.
 *
 *              Startup:
 *                Splash 5 s -> Init_IOT() -> parse SSID/IP from iot_rx_msg
 *                -> display on LCD -> bridge active
 *
 *              LCD layout (after connect):
 *                Line 0:  SSID           (centered, max 10 chars)
 *                Line 1:  "IP address"   (literal, centered)
 *                Line 2:  octets 1-2     (centered)
 *                Line 3:  octets 3-4     (centered)
 *
 *              When web command received:
 *                lcd_BIG_mid() shows "D NN" on enlarged middle line
 *                lcd_4line()  + SSID/IP restored when command finishes
 *
 *              FRAM terminal ^ commands (not forwarded to IOT):
 *                ^^  -> "I'm here\r\n"
 *                ^F  -> set IOT baud 115,200
 *                ^S  -> set IOT baud 9,600
 *
 *              Web command format: ^1234Dtttt
 *                ^ = start, 1234 = PIN, D = F/B/R/L, tttt = time units
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include "serial.h"

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
#define MOTORS_ALL_OFF()  (P6OUT &= ~(R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE))
#define MOTOR_FORWARD()   { MOTORS_ALL_OFF(); P6OUT |= (R_FORWARD | L_FORWARD); }
#define MOTOR_REVERSE()   { MOTORS_ALL_OFF(); P6OUT |= (R_REVERSE | L_REVERSE); }
#define MOTOR_RIGHT()     { MOTORS_ALL_OFF(); P6OUT |= (L_FORWARD | R_REVERSE); }
#define MOTOR_LEFT()      { MOTORS_ALL_OFF(); P6OUT |= R_FORWARD; }

/*==============================================================================
 * Motor timing  (tune on bench)
 *==============================================================================*/
#define TIME_UNIT_MS        (500u)

/*==============================================================================
 * Security PIN
 *==============================================================================*/
#define SECRET_PIN          "1234"
#define SECRET_PIN_LEN      (4u)

/*==============================================================================
 * State machine
 *==============================================================================*/
#define STATE_SPLASH        (0u)
#define STATE_IOT_INIT      (1u)
#define STATE_ACTIVE        (2u)
#define STATE_MOTOR         (3u)
#define STATE_IOT_QUERY     (4u)

/*==============================================================================
 * Motor state
 *==============================================================================*/
static unsigned char motor_direction       = 'F';
static unsigned int  motor_units_remaining = 0u;

/*==============================================================================
 * SSID / IP display strings (centered, 10 chars each)
 *==============================================================================*/
static char ssid_display[11] = "          ";
static char ip_oct12[11]     = "          ";
static char ip_oct34[11]     = "          ";

/*==============================================================================
 * center_string
 * Writes 'text' (len chars) centered into a 10-char null-terminated 'out'.
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
 * set_line  – copy msg into display_line[line] and flag changed
 *==============================================================================*/
static void set_line(int line, const char *msg){
    strncpy(display_line[line], msg, 10u);
    display_line[line][10] = '\0';
    display_changed = TRUE;
}

/*==============================================================================
 * display_ssid_ip
 * Restores the 4-line info screen.
 *==============================================================================*/
static void display_ssid_ip(void){
    lcd_4line();
    set_line(0, ssid_display);
    set_line(1, "IP address");
    set_line(2, ip_oct12);
    set_line(3, ip_oct34);
    Display_Update(0, 0, 0, 0);
}

/*==============================================================================
 * process_iot_msg
 *
 * Called from main whenever iot_msg_ready is set.
 * Parses the single line in iot_rx_msg[] for:
 *   +CWJAP:"ssid",...  -> extract and center SSID
 *   +CIFSR:STAIP,"ip"  -> split and center IP octets
 *   +IPD,x,n:payload   -> extract TCP payload, look for web command
 *
 * After processing, clears iot_msg_ready and re-enables UCA0 RX interrupt.
 *==============================================================================*/

/* helpers ------------------------------------------------------------------ */
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

/* parse +CWJAP:"ssid",... -------------------------------------------------- */
static void parse_cwjap(const char *line){
    unsigned int i = 0u, j = 0u;
    char ssid_raw[11];

    /* Scan to first '"' */
    while(line[i] != '\0' && line[i] != '"'){ i++; }
    if(line[i] == '\0'){ return; }
    i++;   /* skip opening quote */

    while(line[i] != '"' && line[i] != '\0' && j < 10u){
        ssid_raw[j++] = line[i++];
    }
    ssid_raw[j] = '\0';
    if(j > 0u){ center_string(ssid_raw, j, ssid_display); }
}

/* parse +CIFSR:STAIP,"w.x.y.z" -------------------------------------------- */
static void parse_cifsr(const char *line){
    unsigned int i = 0u, j = 0u, dot_count = 0u;
    char ip[16];
    char half1[8], half2[8];
    unsigned int h1len, h2len;

    /* Must contain STAIP */
    while(line[i] != '\0'){
        if(ch_match(&line[i], "STAIP", 5u)){ break; }
        i++;
    }
    if(line[i] == '\0'){ return; }

    /* Find opening quote */
    while(line[i] != '"' && line[i] != '\0'){ i++; }
    if(line[i] == '\0'){ return; }
    i++;

    while(line[i] != '"' && line[i] != '\0' && j < 15u){
        ip[j++] = line[i++];
    }
    ip[j] = '\0';
    if(j == 0u){ return; }

    /* Split at second dot */
    for(i = 0u; i < j; i++){
        if(ip[i] == '.'){ dot_count++; }
        if(dot_count == 2u){
            h1len = i;              /* "w.x"  */
            h2len = j - i - 1u;    /* "y.z" (skip the dot at i) */
            if(h1len > 7u){ h1len = 7u; }
            if(h2len > 7u){ h2len = 7u; }
            strncpy(half1, ip,        h1len); half1[h1len] = '\0';
            strncpy(half2, &ip[i+1u], h2len); half2[h2len] = '\0';
            center_string(half1, h1len, ip_oct12);
            center_string(half2, h2len, ip_oct34);
            return;
        }
    }
    /* Fewer than 2 dots – whole IP on line 2 */
    center_string(ip, j, ip_oct12);
}

/* parse web command from +IPD payload ------------------------------------- */
static void parse_web_command(const char *payload){
    unsigned int i = 0u, consumed = 0u, units = 0u;
    char dir;
    char cmd_str[6];
    char big_line[11];
    unsigned int clen = 0u;

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

    /* Build display string e.g. "F 4" */
    cmd_str[clen++] = dir;
    cmd_str[clen++] = ' ';
    if(units >= 10u){ cmd_str[clen++] = (char)('0' + units / 10u); }
    cmd_str[clen++] = (char)('0' + units % 10u);
    cmd_str[clen]   = '\0';

    /* Show on enlarged middle line */
    lcd_BIG_mid();
    center_string(cmd_str, clen, big_line);
    set_line(1, big_line);
    Display_Update(0, 0, 0, 0);

    UCA1_Transmit_String("CMD OK\r\n", 8u);

    motor_direction       = (unsigned char)dir;
    motor_units_remaining = units;
}

/* main dispatcher ---------------------------------------------------------- */
static void process_iot_msg(void){
    const char *msg = (const char *)iot_rx_msg;
    unsigned int k;

    if(ch_match(msg, "+CWJAP:", 7u)){
        parse_cwjap(msg);
    } else if(ch_match(msg, "+CIFSR:", 7u)){
        parse_cifsr(msg);
    } else if(ch_match(msg, "+IPD", 4u)){
        /* Find ':' then pass the rest as payload */
        k = 0u;
        while(msg[k] != '\0' && msg[k] != ':'){ k++; }
        if(msg[k] == ':'){ parse_web_command(&msg[k + 1u]); }
    }

    /* Clear flag and re-enable UCA0 RX */
    iot_msg_ready = FALSE;
    UCA0IE |= UCRXIE;
}

/*==============================================================================
 * process_fram_cmd
 * Handles single-char FRAM ^ commands from PC terminal.
 *==============================================================================*/
static void process_fram_cmd(void){
    if(!fram_cmd_ready){ return; }
    fram_cmd_ready = FALSE;

    switch(fram_cmd_buf[0]){
        case '^':
            UCA1_Transmit_String("I'm here\r\n", 10u);
            break;
        case 'F': case 'f':
            Set_Baud_UCA0(BAUD_115200);
            UCA1_Transmit_String("115,200\r\n", 9u);
            break;
        case 'S': case 's':
            Set_Baud_UCA0(BAUD_9600);
            UCA1_Transmit_String("9,600\r\n", 7u);
            break;
        default:
            UCA1_Transmit_String("?\r\n", 3u);
            break;
    }
}

/*==============================================================================
 * execute_motor_unit  –  one TIME_UNIT_MS burst
 *==============================================================================*/
static void execute_motor_unit(void){
    if(motor_units_remaining == 0u){ return; }

    switch(motor_direction){
        case 'F': case 'f': MOTOR_FORWARD(); break;
        case 'B': case 'b': MOTOR_REVERSE(); break;
        case 'R': case 'r': MOTOR_RIGHT();   break;
        case 'L': case 'l': MOTOR_LEFT();    break;
        default:            MOTORS_ALL_OFF(); break;
    }

    __delay_cycles((unsigned long)TIME_UNIT_MS * CYCLES_PER_MS);
    motor_units_remaining--;

    if(motor_units_remaining == 0u){
        MOTORS_ALL_OFF();
        display_ssid_ip();
    }
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){

    unsigned char state       = STATE_SPLASH;
    unsigned int  state_timer = RESET_STATE;
    unsigned char tick_200ms  = FALSE;

    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();

    MOTORS_ALL_OFF();

    Init_Serial_UCA0(BAUD_115200);
    Init_Serial_UCA1(BAUD_115200);

    set_line(0, "ECE306    ");
    set_line(1, "NCSU  P9  ");
    set_line(2, "Kachow    ");
    set_line(3, "Port:6767 ");
    Display_Update(0, 0, 0, 0);

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
        }

        Display_Process();

        /*----------------------------------------------------------------------
         * IOT message handler
         * iot_msg_ready set by UCA0 ISR when \r received.
         * process_iot_msg() handles it then re-enables UCA0 RX.
         *--------------------------------------------------------------------*/
        if(iot_msg_ready){
            process_iot_msg();
        }

        /*----------------------------------------------------------------------
         * FRAM ^ command handler
         *--------------------------------------------------------------------*/
        process_fram_cmd();

        /*----------------------------------------------------------------------
         * State machine
         *--------------------------------------------------------------------*/
        switch(state){

            /*------------------------------------------------------------------
             * SPLASH – 5 s
             *----------------------------------------------------------------*/
            case STATE_SPLASH:
                if(tick_200ms){ state_timer++; }
                if(state_timer >= SPLASH_TICKS){
                    state_timer = RESET_STATE;
                    sw1_pressed = FALSE;
                    sw2_pressed = FALSE;
                    set_line(0, "Init IOT  ");
                    set_line(1, "          ");
                    set_line(2, "          ");
                    set_line(3, "          ");
                    Display_Update(0, 0, 0, 0);
                    state = STATE_IOT_INIT;
                }
                break;

            /*------------------------------------------------------------------
             * IOT_INIT
             *   Init_IOT() is blocking – it resets the module, sends AT
             *   commands, and queries CWJAP + CIFSR.  Responses arrive in
             *   UCA0 ISR and are parsed when iot_msg_ready fires above.
             *   After Init_IOT() returns we go straight to ACTIVE; the
             *   SSID/IP display will fill in as the last IOT responses arrive.
             *----------------------------------------------------------------*/
            // In main.c switch(state)
            case STATE_IOT_INIT:
                Init_IOT();
                state_timer = 0; // Reuse timer for query pacing
                state = STATE_IOT_QUERY;
                break;

            case STATE_IOT_QUERY:
                // Every 2 seconds (10 ticks), send a query until we have data
                if(tick_200ms){ state_timer++; }

                if(state_timer == 5){
                    UCA0_Transmit_String("AT+CWJAP?\r\n", 11u);
                }
                if(state_timer == 30){
                    UCA0_Transmit_String("AT+CIFSR\r\n", 10u);
                }

                // If we successfully grabbed both, move to active
                // You can check if the first char of your display strings isn't a space
                if(ssid_display[5] != ' ' && ip_oct12[5] != ' '){
                    display_ssid_ip();
                    state = STATE_ACTIVE;
                }

                // Safety timeout: move to active after 10 seconds anyway
                if(state_timer > 50){
                    display_ssid_ip();
                    state = STATE_ACTIVE;
                }
                break;
            /*------------------------------------------------------------------
             * ACTIVE
             *   SW1 -> IOT baud 115,200
             *   SW2 -> IOT baud   9,600
             *   motor_units_remaining set by parse_web_command -> STATE_MOTOR
             *----------------------------------------------------------------*/
            case STATE_ACTIVE:
                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    Set_Baud_UCA0(BAUD_115200);
                    UCA1_Transmit_String("IOT->115200\r\n", 13u);
                }
                if(sw2_pressed){
                    sw2_pressed = FALSE;
                    Set_Baud_UCA0(BAUD_9600);
                    UCA1_Transmit_String("IOT->9600\r\n", 11u);
                }
                if(motor_units_remaining > 0u){
                    state = STATE_MOTOR;
                }
                break;

            /*------------------------------------------------------------------
             * STATE_MOTOR
             *   One TIME_UNIT_MS burst per loop pass.
             *   execute_motor_unit() stops motors and restores LCD when done.
             *----------------------------------------------------------------*/
            case STATE_MOTOR:
                execute_motor_unit();
                if(motor_units_remaining == 0u){
                    state = STATE_ACTIVE;
                }
                break;

            default:
                state = STATE_ACTIVE;
                break;
        }

    } /* while(ALWAYS) */
}
