/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned int  msec_count;


void motors_off(void){
    P6SEL0 &= ~(R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE);
    P6SEL1 &= ~(R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE);
    P6OUT  &= ~(R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE);
}

void go_forward(void){
    motors_off();
    P6OUT |= (R_FORWARD | L_FORWARD);
}

void go_reverse(void){
    motors_off();
    P6OUT |= (R_REVERSE | L_REVERSE);
}

void spin_cw(void){
    motors_off();
    P6OUT |= (R_REVERSE | L_FORWARD);
}

void spin_ccw(void){
    motors_off();
    P6OUT |= (R_FORWARD | L_REVERSE);
}

//------------------------------------------------------------------------------
// wait_ms – block for ms milliseconds using the ISR tick
//------------------------------------------------------------------------------
void wait_ms(unsigned int ms){
    unsigned int start = msec_count;
    while((unsigned int)(msec_count - start) < ms);
}

//------------------------------------------------------------------------------
// show – write 4 lines to LCD and trigger update
//------------------------------------------------------------------------------
void show(char *l0, char *l1, char *l2, char *l3){
    strncpy(display_line[0], l0, 10);
    strncpy(display_line[1], l1, 10);
    strncpy(display_line[2], l2, 10);
    strncpy(display_line[3], l3, 10);
    display_changed = TRUE;
    Display_Process();
}

//------------------------------------------------------------------------------
// main
//------------------------------------------------------------------------------
void main(void){
    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();

    motors_off();

    // Step 1: Forward 1s
    show("  FORWARD ", "          ", " 1 second ", "          ");
    go_forward();
    wait_ms(1000);

    // Pause 1s
    show("  PAUSED  ", "          ", " 1 second ", "          ");
    motors_off();
    wait_ms(1000);

    // Step 2: Reverse 2s
    show("  REVERSE ", "          ", "2 seconds ", "          ");
    go_reverse();
    wait_ms(2000);

    // Pause 1s
    show("  PAUSED  ", "          ", " 1 second ", "          ");
    motors_off();
    wait_ms(1000);

    // Step 3: Forward 1s
    show("  FORWARD ", "          ", " 1 second ", "          ");
    go_forward();
    wait_ms(1000);

    // Pause 1s
    show("  PAUSED  ", "          ", " 1 second ", "          ");
    motors_off();
    wait_ms(1000);

    // Step 4: Spin CW 3s
    show(" SPIN  CW ", "          ", "3 seconds ", "          ");
    spin_cw();
    wait_ms(3000);

    // Pause 2s
    show("  PAUSED  ", "          ", "2 seconds ", "          ");
    motors_off();
    wait_ms(2000);

    // Step 5: Spin CCW 3s
    show(" SPIN CCW ", "          ", "3 seconds ", "          ");
    spin_ccw();
    wait_ms(3000);

    // Pause 2s
    show("  PAUSED  ", "          ", "2 seconds ", "          ");
    motors_off();
    wait_ms(2000);

    // Done
    show("   DONE   ", " SEQUENCE ", " COMPLETE ", "          ");
    motors_off();

    while(ALWAYS){
        Display_Process();
    }
}
