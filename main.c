/*------------------------------------------------------------------------------
 * File:        main.c  (Project 9)
 * Target:      MSP430FR2355
 *
 * Description: Bidirectional FRAM <-> IOT bridge with PC backdoor.
 *              Motor control via GPIO P6.1-P6.4.
 *
 *              LCD layout after IOT connects:
 *                Line 0:  SSID (centered, max 10 chars)
 *                Line 1:  "IP address" (centered)
 *                Line 2:  first two IP octets  e.g. " 152.14  "  (centered)
 *                Line 3:  last  two IP octets  e.g. " 100.5   "  (centered)
 *
 *              When a web command arrives the display switches to:
 *                lcd_BIG_mid() shows direction + units on the enlarged line
 *              When the command completes:
 *                lcd_4line() restores the 4-line view with SSID/IP
 *
 *              Web command format:  ^PINDtttt
 *                ^ = start, PIN = 1234, D = F/B/R/L, tttt = time units
 *
 *              FRAM terminal commands (^ prefix, NOT sent to IOT):
 *                ^^  -> "I'm here"
 *                ^F  -> IOT baud 115200
 *                ^S  -> IOT baud 9600
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
#define MOTOR_LEFT()      { MOTORS_ALL_OFF(); P6OUT |= (R_FORWARD | L_REVERSE); }

/*==============================================================================
 * Motor timing
 *==============================================================================*/
#define TIME_UNIT_MS        (500u)
#define TURN_90_UNITS       (8u)
#define TURN_45_UNITS       (4u)

/*==============================================================================
 * Security PIN
 *==============================================================================*/
#define SECRET_PIN          "1234"
#define SECRET_PIN_LEN      (4u)

/*==============================================================================
 * IOT network settings
 *==============================================================================*/
#define MY_PORT             "6767"
#define MY_HOSTNAME         "Kachow"

/*==============================================================================
 * State machine
 *==============================================================================*/
#define STATE_SPLASH        (0u)
#define STATE_IOT_RESET     (1u)
#define STATE_WAIT_INFO     (2u)   /* waiting for SSID + IP from IOT           */
#define STATE_ACTIVE        (3u)
#define STATE_MOTOR         (4u)

/*==============================================================================
 * AT commands
 *==============================================================================*/
static const char at_check[]     = "AT";
static const char at_cipmux[]    = "AT+CIPMUX=1";
static const char at_cipserver[] = "AT+CIPSERVER=1," MY_PORT;
static const char at_hostname[]  = "AT+CWHOSTNAME=\"" MY_HOSTNAME "\"";
static const char at_cifsr[]     = "AT+CIFSR";
static const char at_cwjap[]     = "AT+CWJAP?";

/*==============================================================================
 * Motor state
 *==============================================================================*/
static unsigned char motor_direction       = 'F';
static unsigned int  motor_units_remaining = 0u;

/*==============================================================================
 * Line buffer
 *==============================================================================*/
#define LINE_BUF_SIZE       (80u)
static char          line_buf[LINE_BUF_SIZE];
static unsigned int  line_len = 0u;

/*==============================================================================
 * SSID and IP storage
 *==============================================================================*/
#define SSID_MAX            (10u)
static char          ssid_display[11]  = "          ";  /* centered, 10 chars */
static char          ip_oct12[11]      = "          ";  /* centered octets 1-2 */
static char          ip_oct34[11]      = "          ";  /* centered octets 3-4 */
static unsigned char ssid_obtained     = FALSE;
static unsigned char ip_obtained       = FALSE;

/*==============================================================================
 * center_string
 *
 * Writes 'text' (up to 'textlen' chars) centered into a 10-char field 'out'.
 * Pads with spaces on both sides. 'out' must be at least 11 bytes.
 *
 * Example: center_string("ncsu", 4, out)
 *   -> "   ncsu   "
 *==============================================================================*/
static void center_string(const char *text, unsigned int textlen, char *out){
    unsigned int pad_left, pad_right, i, pos;

    if(textlen > 10u){ textlen = 10u; }        /* truncate to display width   */
    pad_left  = (10u - textlen) / 2u;
    pad_right = 10u - textlen - pad_left;

    pos = 0u;
    for(i = 0u; i < pad_left;  i++){ out[pos++] = ' '; }
    for(i = 0u; i < textlen;   i++){ out[pos++] = text[i]; }
    for(i = 0u; i < pad_right; i++){ out[pos++] = ' '; }
    out[10] = '\0';
}

/*==============================================================================
 * display_ssid_ip
 *
 * Restores the 4-line SSID / IP layout on the LCD.
 * Call after a motor command completes or whenever the info screen is needed.
 *==============================================================================*/
static void display_ssid_ip(void){
    lcd_4line();
    strncpy(display_line[0], ssid_display, 10u); display_line[0][10] = '\0';
    strncpy(display_line[1], "IP address", 10u); display_line[1][10] = '\0';
    strncpy(display_line[2], ip_oct12,     10u); display_line[2][10] = '\0';
    strncpy(display_line[3], ip_oct34,     10u); display_line[3][10] = '\0';
    display_changed = TRUE;
    Display_Update(0, 0, 0, 0);
}

/*==============================================================================
 * parse_cwjap_line
 *
 * Parses:  +CWJAP:"ssid","bssid",channel,rssi,...
 * Extracts the SSID (first quoted field), truncates to 10 chars, centers it.
 *==============================================================================*/
static void parse_cwjap_line(const char *line){
    const char prefix[] = "+CWJAP:\"";
    unsigned int plen   = 8u;
    unsigned int i, j;
    char ssid_raw[11];

    if(line[0] != '+' || line[1] != 'C'){ return; }

    /* Find prefix */
    i = 0u;
    while(line[i] != '\0'){
        if(line[i] == '"' && line[i+1u] != '\0'){
            /* First opening quote found – SSID starts at i+1 */
            i++;
            j = 0u;
            while(line[i] != '"' && line[i] != '\0' && j < 10u){
                ssid_raw[j++] = line[i++];
            }
            ssid_raw[j] = '\0';
            if(j > 0u){
                center_string(ssid_raw, j, ssid_display);
                ssid_obtained = TRUE;
            }
            return;
        }
        i++;
    }
    (void)prefix; (void)plen;   /* suppress unused warnings */
}

/*==============================================================================
 * parse_cifsr_line
 *
 * Parses:  +CIFSR:STAIP,"w.x.y.z"
 * Splits IP at the second dot:
 *   ip_oct12  <- "w.x"   centered in 10 chars
 *   ip_oct34  <- "y.z"   centered in 10 chars  (no leading dot)
 *==============================================================================*/
static void parse_cifsr_line(const char *line){
    unsigned int i, j, dot_count;
    char ip[16];
    char half1[8], half2[8];

    /* Must start with +CIFSR:STAIP */
    if(!( line[0]=='+' && line[1]=='C' && line[2]=='I' &&
          line[3]=='F' && line[4]=='S' && line[5]=='R' &&
          line[6]==':' && line[7]=='S' && line[8]=='T' &&
          line[9]=='A' && line[10]=='I' && line[11]=='P' )){
        return;
    }

    /* Find opening quote */
    i = 12u;
    while(line[i] != '"' && line[i] != '\0'){ i++; }
    if(line[i] == '\0'){ return; }
    i++;   /* skip '"' */

    /* Copy IP digits until closing quote */
    j = 0u;
    while(line[i] != '"' && line[i] != '\0' && j < 15u){
        ip[j++] = line[i++];
    }
    ip[j] = '\0';
    if(j == 0u){ return; }

    /* Split at second dot */
    dot_count = 0u;
    for(i = 0u; i < j; i++){
        if(ip[i] == '.'){ dot_count++; }
        if(dot_count == 2u){
            /* First half: ip[0 .. i-1] */
            unsigned int h1len = i;
            unsigned int h2len = j - i - 1u;   /* skip the dot at position i */
            if(h1len > 7u){ h1len = 7u; }
            if(h2len > 7u){ h2len = 7u; }
            strncpy(half1, ip,        h1len); half1[h1len] = '\0';
            strncpy(half2, &ip[i+1u], h2len); half2[h2len] = '\0';
            center_string(half1, h1len, ip_oct12);
            center_string(half2, h2len, ip_oct34);
            ip_obtained = TRUE;
            return;
        }
    }

    /* Fewer than 2 dots – put whole IP on line 2 */
    center_string(ip, j, ip_oct12);
    strncpy(ip_oct34, "          ", 10u); ip_oct34[10] = '\0';
    ip_obtained = TRUE;
}

/*==============================================================================
 * process_cmd  –  FRAM terminal ^ commands
 *==============================================================================*/
static void process_cmd(void){
    if(!cmd_ready){ return; }
    cmd_ready = FALSE;

    if(cmd_buf[0] == CMD_CHAR || cmd_buf[0] == '\0'){
        UCA1_Transmit_String("I'm here\r\n", 10u);

    } else if(cmd_buf[0] == 'F' || cmd_buf[0] == 'f'){
        Set_Baud_UCA0(BAUD_115200);
        UCA1_Transmit_String("115,200\r\n", 9u);

    } else if(cmd_buf[0] == 'S' || cmd_buf[0] == 's'){
        Set_Baud_UCA0(BAUD_9600);
        UCA1_Transmit_String("9,600\r\n", 7u);

    } else {
        UCA1_Transmit_String("?\r\n", 3u);
    }
}

/*==============================================================================
 * str_match  –  compare n chars
 *==============================================================================*/
static unsigned char str_match(const char *s, const char *p, unsigned int n){
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

/*==============================================================================
 * process_ipd_payload  –  web command from TCP client
 *==============================================================================*/
static void process_ipd_payload(char *buf){
    unsigned int i = 0u, consumed = 0u, units = 0u;
    char dir;
    char big_line[11];

    /* Find '^' */
    while(buf[i] != '\0' && buf[i] != '^'){ i++; }
    if(buf[i] == '\0'){ return; }
    i++;

    /* Validate PIN */
    if(!str_match(&buf[i], SECRET_PIN, SECRET_PIN_LEN)){
        UCA1_Transmit_String("BAD PIN\r\n", 9u);
        return;
    }
    i += SECRET_PIN_LEN;

    dir   = buf[i++];
    units = parse_uint(&buf[i], &consumed);

    /*--------------------------------------------------------------------------
     * Display command on enlarged middle line
     * Format: "D  NN" centered – direction letter + space + units
     *--------------------------------------------------------------------------*/
    lcd_BIG_mid();

    /* Build a short string like "F 4" and center it into big_line */
    {
        char cmd_str[6];
        unsigned int clen = 0u;
        cmd_str[clen++] = dir;
        cmd_str[clen++] = ' ';
        if(units >= 10u){
            cmd_str[clen++] = (char)('0' + units / 10u);
        }
        cmd_str[clen++] = (char)('0' + units % 10u);
        cmd_str[clen]   = '\0';
        center_string(cmd_str, clen, big_line);
    }

    /* lcd_BIG_mid uses lines 1 and 2 for the enlarged text */
    strncpy(display_line[1], big_line, 10u); display_line[1][10] = '\0';
    display_changed = TRUE;
    Display_Update(0, 0, 0, 0);

    UCA1_Transmit_String("CMD OK\r\n", 8u);

    motor_direction       = (unsigned char)dir;
    motor_units_remaining = units;
}

/*==============================================================================
 * scan_iot_rx
 *
 * Drains IOT_2_PC ring buffer one character at a time.
 * Accumulates complete lines into line_buf[], then dispatches to:
 *   parse_cwjap_line()    – extract SSID
 *   parse_cifsr_line()    – extract IP
 *   process_ipd_payload() – motor command from TCP client
 *==============================================================================*/
static void scan_iot_rx(void){
    unsigned char c;
    unsigned int k;

    while(iot_rx_rd != iot_rx_wr){
        c = IOT_2_PC[iot_rx_rd++];
        if(iot_rx_rd >= SMALL_RING_SIZE){ iot_rx_rd = BEGINNING; }

        if(c == '\r' || c == '\n'){
            if(line_len > 0u){
                line_buf[line_len] = '\0';

                /* Try SSID parse */
                parse_cwjap_line(line_buf);

                /* Try IP parse */
                parse_cifsr_line(line_buf);

                /* Try IPD payload (look for ':' separator) */
                for(k = 0u; k < line_len; k++){
                    if(line_buf[k] == ':'){
                        process_ipd_payload(&line_buf[k + 1u]);
                        break;
                    }
                }

                line_len = 0u;
            }
        } else {
            if(line_len < (LINE_BUF_SIZE - 1u)){
                line_buf[line_len++] = (char)c;
            }
        }
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
        display_ssid_ip();   /* restore SSID/IP screen when done              */
    }
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){

    unsigned char state         = STATE_SPLASH;
    unsigned int  state_timer   = RESET_STATE;
    unsigned char tick_200ms    = FALSE;
    unsigned int  info_ticks    = RESET_STATE;

    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();

    MOTORS_ALL_OFF();

    Init_Serial_UCA1(BAUD_115200);
    Init_Serial_UCA0(BAUD_115200);
    pc_tx_enabled = TRUE;   /* no gate – bridge open immediately              */

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
        process_cmd();
        scan_iot_rx();

        /*----------------------------------------------------------------------
         * Once both SSID and IP are obtained, draw the info screen once
         *--------------------------------------------------------------------*/
        if(ssid_obtained && ip_obtained){
            ssid_obtained = FALSE;
            ip_obtained   = FALSE;
            display_ssid_ip();
        }

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
                    set_line(0, "Rst IOT.. ");
                    Display_Update(0, 0, 0, 0);
                    state = STATE_IOT_RESET;
                }
                break;

            /*------------------------------------------------------------------
             * IOT_RESET
             *   1. HW reset IOT
             *   2. Send startup AT commands every power-on
             *   3. Query SSID and IP
             *----------------------------------------------------------------*/
            case STATE_IOT_RESET:
                IOT_Reset();

                IOT_Send_Command(at_check,     2u);
                IOT_Send_Command(at_hostname,  (unsigned int)strlen(at_hostname));
                IOT_Send_Command(at_cipmux,    11u);
                IOT_Send_Command(at_cipserver, (unsigned int)strlen(at_cipserver));

                /* Query SSID and IP – responses parsed by scan_iot_rx */
                IOT_Send_Command(at_cwjap,  9u);
                IOT_Send_Command(at_cifsr,  8u);

                set_line(0, "          ");
                set_line(1, "Getting   ");
                set_line(2, "SSID + IP ");
                set_line(3, "          ");
                Display_Update(0, 0, 0, 0);

                info_ticks = RESET_STATE;
                state = STATE_WAIT_INFO;
                break;

            /*------------------------------------------------------------------
             * WAIT_INFO
             *   Poll until both SSID and IP are parsed from IOT responses.
             *   Re-query every second. Timeout after 10 s.
             *----------------------------------------------------------------*/
            case STATE_WAIT_INFO:
                if(tick_200ms){ info_ticks++; }

                /* Re-request every 1 s in case first response was missed */
                if(info_ticks > 0u && (info_ticks % TICKS_1_SEC) == 0u){
                    if(!ssid_obtained){ IOT_Send_Command(at_cwjap, 9u); }
                    if(!ip_obtained)  { IOT_Send_Command(at_cifsr, 8u); }
                }

                /* Move to ACTIVE on timeout regardless */
                if(info_ticks >= (SPLASH_TICKS * 2u)){
                    /* Fill any missing fields with defaults */
                    if(!ssid_obtained){
                        center_string("No SSID", 7u, ssid_display);
                    }
                    if(!ip_obtained){
                        center_string("No IP", 5u, ip_oct12);
                        strncpy(ip_oct34, "          ", 10u);
                        ip_oct34[10] = '\0';
                    }
                    display_ssid_ip();
                    state = STATE_ACTIVE;
                }
                break;

            /*------------------------------------------------------------------
             * ACTIVE
             *   SW1 -> IOT baud 115,200
             *   SW2 -> IOT baud   9,600
             *   Motor command armed -> STATE_MOTOR
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
             * STATE_MOTOR – one TIME_UNIT_MS burst per loop pass
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
