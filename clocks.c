#include "msp430.h"
#include "functions.h"
#include "macros.h"

void Init_Clocks(void) {
    FRCTL0 = FRCTLPW | NWAITS_1;  // Set FRAM wait state for >8MHz
}

void Set_SMCLK_8MHz(void) {
    CSCTL5 &= ~SMCLK_DIV_MASK;    // DIVS = 000 -> divide by 1 = 8MHz
}

void Set_SMCLK_500kHz(void) {
    CSCTL5 &= ~SMCLK_DIV_MASK;    // Clear divider bits
    CSCTL5 |= SMCLK_DIV_500KHZ;   // DIVS = 100 -> divide by 16 = 500kHz
}

void Set_SMCLK_1MHz(void) {
    CSCTL5 &= ~SMCLK_DIV_MASK;    // Clear divider bits
    CSCTL5 |= SMCLK_DIV_1MHZ;     // DIVS = 011 -> divide by 8 = 1MHz
}
