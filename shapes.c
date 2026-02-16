//------------------------------------------------------------------------------
// Shape Drawing State Machines
// Implements Circle, Figure-8, and Triangle movements
//------------------------------------------------------------------------------

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include <string.h>

// External variables
extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;

// Shape state variables - defined in globals.c
extern volatile unsigned char current_shape;
extern volatile unsigned char shape_state;
extern volatile unsigned char shape_iteration;
extern volatile unsigned int shape_start_time;

// Motor speed definitions for shapes
#define CIRCLE_FAST_SPEED    7000
#define CIRCLE_SLOW_SPEED    3500
#define FIGURE8_FAST_SPEED   7000
#define FIGURE8_SLOW_SPEED   3500
#define TRIANGLE_SPEED       6000
#define TURN_SPEED           4000

// Shape timing definitions (in milliseconds)
#define SAFETY_DELAY         2500    // 2.5 second safety delay
#define CIRCLE_TIME          5000    // Time for one circle
#define FIGURE8_LOOP_TIME    4500    // Time for one loop of figure-8
#define TRIANGLE_STRAIGHT    2000    // Straight leg time
#define TRIANGLE_TURN        600     // Turn time at vertices

//------------------------------------------------------------------------------
// Motor Control Functions
//------------------------------------------------------------------------------

void stop_motors(void) {
    RIGHT_FORWARD_SPEED = WHEEL_OFF;
    RIGHT_REVERSE_SPEED = WHEEL_OFF;
    LEFT_FORWARD_SPEED = WHEEL_OFF;
    LEFT_REVERSE_SPEED = WHEEL_OFF;
}

void move_forward(unsigned int left_speed, unsigned int right_speed) {
    LEFT_FORWARD_SPEED = left_speed;
    RIGHT_FORWARD_SPEED = right_speed;
    LEFT_REVERSE_SPEED = WHEEL_OFF;
    RIGHT_REVERSE_SPEED = WHEEL_OFF;
}

void turn_right_pivot(unsigned int speed) {
    LEFT_FORWARD_SPEED = speed;
    RIGHT_REVERSE_SPEED = speed;
    LEFT_REVERSE_SPEED = WHEEL_OFF;
    RIGHT_FORWARD_SPEED = WHEEL_OFF;
}

void turn_left_pivot(unsigned int speed) {
    RIGHT_FORWARD_SPEED = speed;
    LEFT_REVERSE_SPEED = speed;
    RIGHT_REVERSE_SPEED = WHEEL_OFF;
    LEFT_FORWARD_SPEED = WHEEL_OFF;
}

//------------------------------------------------------------------------------
// Circle State Machine
// Makes two complete circles with differential wheel speeds
//------------------------------------------------------------------------------
void Circle_StateMachine(void) {
    unsigned int elapsed_time = get_shape_time() - shape_start_time;

    switch(shape_state) {
        case 0:  // Initialize
            strcpy(display_line[0], "          ");
            strcpy(display_line[1], "  CIRCLE  ");
            strcpy(display_line[2], "          ");
            strcpy(display_line[3], "          ");
            display_changed = TRUE;
            shape_state = 1;
            shape_start_time = get_shape_time();
            shape_iteration = 0;
            break;

        case 1:  // Safety delay
            if(elapsed_time >= SAFETY_DELAY) {
                shape_state = 2;
                shape_start_time = get_shape_time();
            }
            break;

        case 2:  // First circle
            // Right wheel faster, left wheel slower for clockwise circle
            move_forward(CIRCLE_SLOW_SPEED, CIRCLE_FAST_SPEED);

            if(elapsed_time >= CIRCLE_TIME) {
                shape_iteration++;
                if(shape_iteration >= 2) {
                    shape_state = 3;  // Done with both circles
                } else {
                    shape_start_time = get_shape_time();  // Reset for second circle
                }
            }
            break;

        case 3:  // Complete
            stop_motors();
            strcpy(display_line[1], " COMPLETE ");
            display_changed = TRUE;
            current_shape = SHAPE_NONE;
            shape_state = 0;
            stop_shape_timer();
            break;
    }
}

//------------------------------------------------------------------------------
// Figure-8 State Machine
// Makes two complete figure-8 patterns
//------------------------------------------------------------------------------
void Figure8_StateMachine(void) {
    unsigned int elapsed_time = get_shape_time() - shape_start_time;

    switch(shape_state) {
        case 0:  // Initialize
            strcpy(display_line[0], "          ");
            strcpy(display_line[1], " FIGURE-8 ");
            strcpy(display_line[2], "          ");
            strcpy(display_line[3], "          ");
            display_changed = TRUE;
            shape_state = 1;
            shape_start_time = get_shape_time();
            shape_iteration = 0;
            break;

        case 1:  // Safety delay
            if(elapsed_time >= SAFETY_DELAY) {
                shape_state = 2;
                shape_start_time = get_shape_time();
            }
            break;

        case 2:  // First loop (clockwise)
            // Right wheel faster for clockwise circle
            move_forward(FIGURE8_SLOW_SPEED, FIGURE8_FAST_SPEED);

            if(elapsed_time >= FIGURE8_LOOP_TIME) {
                shape_state = 3;
                shape_start_time = get_shape_time();
            }
            break;

        case 3:  // Second loop (counter-clockwise)
            // Left wheel faster for counter-clockwise circle
            move_forward(FIGURE8_FAST_SPEED, FIGURE8_SLOW_SPEED);

            if(elapsed_time >= FIGURE8_LOOP_TIME) {
                shape_iteration++;
                if(shape_iteration >= 2) {
                    shape_state = 4;  // Done
                } else {
                    shape_state = 2;  // Start next figure-8
                    shape_start_time = get_shape_time();
                }
            }
            break;

        case 4:  // Complete
            stop_motors();
            strcpy(display_line[1], " COMPLETE ");
            display_changed = TRUE;
            current_shape = SHAPE_NONE;
            shape_state = 0;
            stop_shape_timer();
            break;
    }
}

//------------------------------------------------------------------------------
// Triangle State Machine
// Makes two complete triangles with three sides each
//------------------------------------------------------------------------------
void Triangle_StateMachine(void) {
    unsigned int elapsed_time = get_shape_time() - shape_start_time;

    switch(shape_state) {
        case 0:  // Initialize
            strcpy(display_line[0], "          ");
            strcpy(display_line[1], " TRIANGLE ");
            strcpy(display_line[2], "          ");
            strcpy(display_line[3], "          ");
            display_changed = TRUE;
            shape_state = 1;
            shape_start_time = get_shape_time();
            shape_iteration = 0;
            break;

        case 1:  // Safety delay
            if(elapsed_time >= SAFETY_DELAY) {
                shape_state = 2;
                shape_start_time = get_shape_time();
            }
            break;

        // First side
        case 2:
            move_forward(TRIANGLE_SPEED, TRIANGLE_SPEED);
            if(elapsed_time >= TRIANGLE_STRAIGHT) {
                shape_state = 3;
                shape_start_time = get_shape_time();
            }
            break;

        // First turn (120 degrees)
        case 3:
            turn_left_pivot(TURN_SPEED);
            if(elapsed_time >= TRIANGLE_TURN) {
                shape_state = 4;
                shape_start_time = get_shape_time();
            }
            break;

        // Second side
        case 4:
            move_forward(TRIANGLE_SPEED, TRIANGLE_SPEED);
            if(elapsed_time >= TRIANGLE_STRAIGHT) {
                shape_state = 5;
                shape_start_time = get_shape_time();
            }
            break;

        // Second turn (120 degrees)
        case 5:
            turn_left_pivot(TURN_SPEED);
            if(elapsed_time >= TRIANGLE_TURN) {
                shape_state = 6;
                shape_start_time = get_shape_time();
            }
            break;

        // Third side
        case 6:
            move_forward(TRIANGLE_SPEED, TRIANGLE_SPEED);
            if(elapsed_time >= TRIANGLE_STRAIGHT) {
                shape_state = 7;
                shape_start_time = get_shape_time();
            }
            break;

        // Third turn (120 degrees)
        case 7:
            turn_left_pivot(TURN_SPEED);
            if(elapsed_time >= TRIANGLE_TURN) {
                shape_iteration++;
                if(shape_iteration >= 2) {
                    shape_state = 8;  // Done
                } else {
                    shape_state = 2;  // Start second triangle
                    shape_start_time = get_shape_time();
                }
            }
            break;

        case 8:  // Complete
            stop_motors();
            strcpy(display_line[1], " COMPLETE ");
            display_changed = TRUE;
            current_shape = SHAPE_NONE;
            shape_state = 0;
            stop_shape_timer();
            break;
    }
}

//------------------------------------------------------------------------------
// Main Shape Process - Called from main loop
//------------------------------------------------------------------------------
void Shapes_Process(void) {
    switch(current_shape) {
        case SHAPE_CIRCLE:
            Circle_StateMachine();
            break;

        case SHAPE_FIGURE8:
            Figure8_StateMachine();
            break;

        case SHAPE_TRIANGLE:
            Triangle_StateMachine();
            break;

        case SHAPE_NONE:
        default:
            // No active shape
            break;
    }
}

//------------------------------------------------------------------------------
// Start a shape sequence
//------------------------------------------------------------------------------
void start_shape(unsigned char shape) {
    if(current_shape == SHAPE_NONE) {  // Only start if no shape is running
        current_shape = shape;
        shape_state = 0;
        shape_iteration = 0;
        start_shape_timer();
    }
}
