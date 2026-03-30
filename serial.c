/*------------------------------------------------------------------------------
 * File:        serial.c
 * Target:      MSP430FR2355
 *
 * Description: eUSCI_A0 (IOT/J9) and eUSCI_A1 (PC/J14) UART initialization
 *              and cross-connect interrupt service routines.
 *
 *              Cross-connect data path:
 *                PC → UCA1 RX ISR → UCA0TXBUF → J9 loopback
 *                   → UCA0 RX ISR → UCA1TXBUF → PC  (echo back to Termite)
 *
 *              The ISR also maintains rx_display_buf, a 10-character rolling
 *              window of received characters.  Each new character shifts the
 *              buffer left and appends at position 9, then sets
 *              rx_display_updated = TRUE so the main loop redraws line 0.
 *
 * eUSCI_A init sequence (TI User Guide):
 *   1. Set   UCSWRST
 *   2. Write control registers (UCSWRST still set)
 *   3. Port pins already configured in ports.c
 *   4. Clear UCSWRST
 *   5. Enable UCRXIE
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "serial.h"
#include "macros.h"
#include "ports.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * Ring buffers
 *------------------------------------------------------------------------------*/
volatile unsigned int  uca0_rx_wr = SERIAL_BEGINNING;
volatile unsigned int  uca0_rx_rd = SERIAL_BEGINNING;
volatile char          UCA0_Char_Rx[SERIAL_RING_SIZE];

volatile unsigned int  uca1_rx_wr = SERIAL_BEGINNING;
volatile unsigned int  uca1_rx_rd = SERIAL_BEGINNING;
volatile char          UCA1_Char_Rx[SERIAL_RING_SIZE];

/*------------------------------------------------------------------------------
 * 10-char rolling display buffer for LCD line 0
 *   Initialised to spaces so the LCD shows a blank line on startup.
 *   +1 for null terminator (used by strncpy in main if needed).
 *------------------------------------------------------------------------------*/
volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u] = "          ";
volatile unsigned char rx_display_updated = FALSE;

/*------------------------------------------------------------------------------
 * Current baud rate selector
 *------------------------------------------------------------------------------*/
volatile unsigned char current_baud = BAUD_115200;

/*==============================================================================
 * apply_baud_registers  (internal helper)
 *
 * Writes UCBRx and MCTLW for the chosen baud rate.
 * Caller must hold UCSWRST = 1 before calling.
 *==============================================================================*/
static void apply_baud_registers(unsigned char baud,
                                 volatile unsigned int *pBRW,
                                 volatile unsigned int *pMCTL)
{
    if(baud == BAUD_460800){
        *pBRW  = BR_460800_UCBRx;    /* 1                                      */
        *pMCTL = BR_460800_MCTLW;    /* 0x4911: UCBRSx=0x49, UCFx=1, UCOS16=1 */
    } else {
        /* Default 115,200 */
        *pBRW  = BR_115200_UCBRx;    /* 4                                      */
        *pMCTL = BR_115200_MCTLW;    /* 0x5551: UCBRSx=0x55, UCFx=5, UCOS16=1 */
    }
}

/*==============================================================================
 * Init_Serial_UCA0
 *
 * Initializes eUSCI_A0 for the IOT module on J9.
 * P1.6 = UCA0RXD, P1.7 = UCA0TXD  (configured as UART in Init_Port1).
 *==============================================================================*/
void Init_Serial_UCA0(unsigned char baud){
    unsigned int i;

    for(i = SERIAL_BEGINNING; i < SERIAL_RING_SIZE; i++){
        UCA0_Char_Rx[i] = 0x00;
    }
    uca0_rx_wr = SERIAL_BEGINNING;
    uca0_rx_rd = SERIAL_BEGINNING;

    UCA0CTLW0  = RESET_STATE;
    UCA0CTLW0 |= UCSWRST;            /* Step 1: assert software reset          */
    UCA0CTLW0 |= UCSSEL__SMCLK;      /* Step 2: BRCLK = SMCLK = 8 MHz         */
    /* 8N1 is the default (all other CTLW0 bits = 0)                           */

    apply_baud_registers(baud, &UCA0BRW, &UCA0MCTLW);

    /* Step 3: port pins already configured in Init_Port1()                    */

    UCA0CTLW0 &= ~UCSWRST;           /* Step 4: release from reset             */
    UCA0IE    |=  UCRXIE;            /* Step 5: enable RX interrupt            */
}

/*==============================================================================
 * Init_Serial_UCA1
 *
 * Initializes eUSCI_A1 for the PC backdoor on J14.
 * P4.2 = UCA1RXD, P4.3 = UCA1TXD  (configured as UART in Init_Port4).
 *==============================================================================*/
void Init_Serial_UCA1(unsigned char baud){
    unsigned int i;

    for(i = SERIAL_BEGINNING; i < SERIAL_RING_SIZE; i++){
        UCA1_Char_Rx[i] = 0x00;
    }
    uca1_rx_wr = SERIAL_BEGINNING;
    uca1_rx_rd = SERIAL_BEGINNING;

    current_baud = baud;

    /* Init display buffer to spaces */
    for(i = 0; i < RX_DISPLAY_LEN; i++){
        rx_display_buf[i] = ' ';
    }
    rx_display_buf[RX_DISPLAY_LEN] = '\0';
    rx_display_updated = FALSE;

    UCA1CTLW0  = RESET_STATE;
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;

    apply_baud_registers(baud, &UCA1BRW, &UCA1MCTLW);

    UCA1CTLW0 &= ~UCSWRST;
    UCA1IE    |=  UCRXIE;
}

/*==============================================================================
 * Set_Baud_UCA0
 *
 * Runtime baud change without reinitializing ring buffers.
 *==============================================================================*/
void Set_Baud_UCA0(unsigned char baud){
    UCA0CTLW0 |=  UCSWRST;
    apply_baud_registers(baud, &UCA0BRW, &UCA0MCTLW);
    UCA0CTLW0 &= ~UCSWRST;
    UCA0IE    |=  UCRXIE;
}

/*==============================================================================
 * Set_Baud_UCA1
 *
 * Runtime baud change without reinitializing ring buffers.
 * Also resets the display buffer to spaces so stale characters are not shown.
 *==============================================================================*/
void Set_Baud_UCA1(unsigned char baud){
    unsigned int i;
    current_baud = baud;

    /* Clear rolling display window on every baud change */
    for(i = 0; i < RX_DISPLAY_LEN; i++){
        rx_display_buf[i] = ' ';
    }
    rx_display_buf[RX_DISPLAY_LEN] = '\0';
    rx_display_updated = TRUE;

    UCA1CTLW0 |=  UCSWRST;
    apply_baud_registers(baud, &UCA1BRW, &UCA1MCTLW);
    UCA1CTLW0 &= ~UCSWRST;
    UCA1IE    |=  UCRXIE;
}

/*==============================================================================
 * UCA1_Transmit_String
 *
 * Blocking transmit out of UCA1 (PC side).
 * Polls UCTXIFG before each byte – safe to call from foreground.
 *==============================================================================*/
void UCA1_Transmit_String(const char *str, unsigned int len){
    unsigned int i;
    for(i = 0; i < len; i++){
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (unsigned char)str[i];
    }
}

/*==============================================================================
 * EUSCI_A1_VECTOR  –  UCA1 ISR  (PC backdoor, J14)
 *
 * RX: character arrived from PC / Termite.
 *   1. Read UCA1RXBUF immediately (clears flag).
 *   2. Store in UCA1 ring buffer.
 *   3. Append to 10-char rolling display buffer; flag for LCD update.
 *   4. Forward to UCA0TXBUF (cross-connect to IOT / J9 loopback).
 *==============================================================================*/
#pragma vector = EUSCI_A1_VECTOR
__interrupt void eUSCI_A1_ISR(void){
    unsigned int temp;
    char rx_char;
    unsigned int i;

    switch(__even_in_range(UCA1IV, 0x08)){

        case 0:    /* No interrupt                                             */
            break;

        case 2:    /* RXIFG                                                    */
            rx_char = (char)UCA1RXBUF;    /* Read immediately to clear flag   */

            /* --- Ring buffer ------------------------------------------------*/
            temp = uca1_rx_wr++;
            UCA1_Char_Rx[temp] = rx_char;
            if(uca1_rx_wr >= SERIAL_RING_SIZE){
                uca1_rx_wr = SERIAL_BEGINNING;
            }

            /* --- Rolling 10-char display buffer ----------------------------*/
            /* Shift left by 1, place new char at position 9                  */
            for(i = 0; i < (RX_DISPLAY_LEN - 1u); i++){
                rx_display_buf[i] = rx_display_buf[i + 1u];
            }
            rx_display_buf[RX_DISPLAY_LEN - 1u] = rx_char;
            rx_display_updated = TRUE;

            /* --- Cross-connect: forward to UCA0 TX -------------------------*/
            while(!(UCA0IFG & UCTXIFG));
            UCA0TXBUF = (unsigned char)rx_char;
            break;

        case 4:    /* TXIFG – not used                                        */
            break;

        default:
            break;
    }
}

/*==============================================================================
 * EUSCI_A0_VECTOR  –  UCA0 ISR  (IOT port / J9 loopback)
 *
 * RX: character returned from J9 loopback.
 *   1. Read UCA0RXBUF.
 *   2. Store in UCA0 ring buffer.
 *   3. Forward to UCA1TXBUF -> PC (completes the echo).
 *
 * Note: we do NOT update rx_display_buf here because the UCA1 ISR already
 * captured the character on the way in.  Displaying it again would duplicate.
 *==============================================================================*/
#pragma vector = EUSCI_A0_VECTOR
__interrupt void eUSCI_A0_ISR(void){
    unsigned int temp;
    char rx_char;

    switch(__even_in_range(UCA0IV, 0x08)){

        case 0:
            break;

        case 2:    /* RXIFG                                                    */
            rx_char = (char)UCA0RXBUF;

            temp = uca0_rx_wr++;
            UCA0_Char_Rx[temp] = rx_char;
            if(uca0_rx_wr >= SERIAL_RING_SIZE){
                uca0_rx_wr = SERIAL_BEGINNING;
            }

            /* Cross-connect: forward back to PC via UCA1                     */
            while(!(UCA1IFG & UCTXIFG));
            UCA1TXBUF = (unsigned char)rx_char;
            break;

        case 4:
            break;

        default:
            break;
    }
}
