/*------------------------------------------------------------------------------
 * File:        serial.c
 * Target:      MSP430FR2355
 *
 * Description: eUSCI_A0 (IOT/J9) and eUSCI_A1 (PC/J14) UART.
 *
 *              Data path:
 *                PC  -> UCA1 RX ISR -> UCA0TXBUF (passthrough, if not ^ cmd)
 *                IOT -> UCA0 RX ISR -> UCA1TXBUF (forward to PC if pc_ready)
 *
 *              UCA0 ISR collects one line at a time into iot_rx_msg[].
 *              When \r is received iot_msg_ready is set and RX interrupt
 *              is disabled until main clears it and re-enables.
 *
 *              UCA1 ISR detects ^ prefix: characters after ^ go into
 *              fram_cmd_buf[] and are NOT forwarded to the IOT.
 *              fram_cmd_ready is set after the first non-^ character
 *              (single-char commands: F, S, ^, etc.).
 *
 *              Init_IOT() drives the hardware reset sequence and sends
 *              the three power-on AT commands to the IOT module.
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
 * IOT message buffer  (one AT response line, CR-terminated)
 *------------------------------------------------------------------------------*/
volatile char          iot_rx_msg[IOT_MSG_SIZE];
volatile unsigned char iot_msg_ready  = FALSE;
volatile unsigned int  iot_rx_count   = 0u;

/*------------------------------------------------------------------------------
 * PC ready flag and FRAM ^ command parser state
 *------------------------------------------------------------------------------*/
volatile unsigned char pc_ready       = FALSE;
volatile unsigned char fram_cmd_active = FALSE;
volatile unsigned char fram_cmd_ready  = FALSE;
volatile unsigned int  fram_cmd_count  = 0u;
volatile char          fram_cmd_buf[FRAM_CMD_BUF_SIZE];

/*------------------------------------------------------------------------------
 * LCD rolling display buffer
 *------------------------------------------------------------------------------*/
volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u] = "          ";
volatile unsigned char rx_display_updated = FALSE;

/*------------------------------------------------------------------------------
 * Current baud
 *------------------------------------------------------------------------------*/
volatile unsigned char current_baud = BAUD_115200;

/*==============================================================================
 * apply_baud_registers  (internal)
 *==============================================================================*/
static void apply_baud_registers(unsigned char baud,
                                  volatile unsigned int *pBRW,
                                  volatile unsigned int *pMCTL)
{
    switch(baud){
        case BAUD_460800:
            *pBRW  = BR_460800_UCBRx;
            *pMCTL = BR_460800_MCTLW;
            break;
        case BAUD_9600:
            *pBRW  = BR_9600_UCBRx;
            *pMCTL = BR_9600_MCTLW;
            break;
        default:   /* BAUD_115200 */
            *pBRW  = BR_115200_UCBRx;
            *pMCTL = BR_115200_MCTLW;
            break;
    }
}

/*==============================================================================
 * Init_Serial_UCA0  (IOT side, J9)
 *==============================================================================*/
void Init_Serial_UCA0(unsigned char baud){
    unsigned int i;
    for(i = SERIAL_BEGINNING; i < SERIAL_RING_SIZE; i++){
        UCA0_Char_Rx[i] = 0x00;
    }
    uca0_rx_wr    = SERIAL_BEGINNING;
    uca0_rx_rd    = SERIAL_BEGINNING;
    iot_rx_count  = 0u;
    iot_msg_ready = FALSE;

    UCA0CTLW0  = RESET_STATE;
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;
    apply_baud_registers(baud, &UCA0BRW, &UCA0MCTLW);
    UCA0CTLW0 &= ~UCSWRST;
    UCA0IE    |=  UCRXIE;
}

/*==============================================================================
 * Init_Serial_UCA1  (PC backdoor, J14)
 *==============================================================================*/
void Init_Serial_UCA1(unsigned char baud){
    unsigned int i;
    for(i = SERIAL_BEGINNING; i < SERIAL_RING_SIZE; i++){
        UCA1_Char_Rx[i] = 0x00;
    }
    uca1_rx_wr = SERIAL_BEGINNING;
    uca1_rx_rd = SERIAL_BEGINNING;
    current_baud = baud;

    for(i = 0u; i < RX_DISPLAY_LEN; i++){
        rx_display_buf[i] = ' ';
    }
    rx_display_buf[RX_DISPLAY_LEN] = '\0';
    rx_display_updated = FALSE;

    pc_ready       = FALSE;
    fram_cmd_active = FALSE;
    fram_cmd_ready  = FALSE;
    fram_cmd_count  = 0u;

    UCA1CTLW0  = RESET_STATE;
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    apply_baud_registers(baud, &UCA1BRW, &UCA1MCTLW);
    UCA1CTLW0 &= ~UCSWRST;
    UCA1IE    |=  UCRXIE;
}

/*==============================================================================
 * Set_Baud_UCA0
 *==============================================================================*/
void Set_Baud_UCA0(unsigned char baud){
    UCA0CTLW0 |=  UCSWRST;
    apply_baud_registers(baud, &UCA0BRW, &UCA0MCTLW);
    UCA0CTLW0 &= ~UCSWRST;
    UCA0IE    |=  UCRXIE;
}

/*==============================================================================
 * Set_Baud_UCA1
 *==============================================================================*/
void Set_Baud_UCA1(unsigned char baud){
    unsigned int i;
    current_baud = baud;
    for(i = 0u; i < RX_DISPLAY_LEN; i++){
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
 * UCA1_Transmit_String  (blocking, foreground -> PC)
 *==============================================================================*/
void UCA1_Transmit_String(const char *str, unsigned int len){
    unsigned int i;
    for(i = 0u; i < len; i++){
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (unsigned char)str[i];
    }
}

/*==============================================================================
 * UCA0_Transmit_String  (blocking, foreground -> IOT)
 *==============================================================================*/
void UCA0_Transmit_String(const char *str, unsigned int len){
    unsigned int i;
    for(i = 0u; i < len; i++){
        while(!(UCA0IFG & UCTXIFG));
        UCA0TXBUF = (unsigned char)str[i];
    }
}

/*==============================================================================
 * Init_IOT
 *
 * Hardware reset sequence + power-on AT commands.
 * Matches the provided reference implementation.
 * Call after Init_Serial_UCA0() and Init_Serial_UCA1().
 *==============================================================================*/
// In serial.c
void Init_IOT(void){
    P6OUT |= GRN_LED;

    // Hardware Reset Pulse
    P3OUT &= ~IOT_EN_CPU;
    __delay_cycles(800000u);
    P3OUT |=  IOT_EN_CPU;

    pc_ready = TRUE;
    __delay_cycles(8000000u); // 1s settle

    // Basic Config
    UCA0_Transmit_String("AT+SYSSTORE=0\r\n", 15u);
    __delay_cycles(4000000u); // 0.5s

    UCA0_Transmit_String("AT+CIPMUX=1\r\n", 13u);
    __delay_cycles(4000000u);

    UCA0_Transmit_String("AT+CIPSERVER=1,6767\r\n", 21u);
    __delay_cycles(4000000u);

    P6OUT &= ~GRN_LED;
}

/*==============================================================================
 * EUSCI_A1_VECTOR  –  UCA1 ISR  (PC backdoor, J14)
 *
 * On every received character:
 *   1. Set pc_ready TRUE (opens IOT->PC forward path).
 *   2. Store in UCA1 ring buffer.
 *   3. Update LCD rolling display buffer.
 *   4a. If '^' detected: start FRAM command, do NOT forward to IOT.
 *   4b. If FRAM command active: accumulate one char then set fram_cmd_ready.
 *   4c. Otherwise: passthrough to IOT via UCA0TXBUF.
 *==============================================================================*/
#pragma vector = EUSCI_A1_VECTOR
__interrupt void eUSCI_A1_ISR(void){
    unsigned int temp;
    char rx_char;
    unsigned int i;

    switch(__even_in_range(UCA1IV, 0x08)){

        case 0:
            break;

        case 2:   /* RXIFG */
            rx_char = (char)UCA1RXBUF;

            /* Open PC TX path on first received character */
            pc_ready = TRUE;

            /* Ring buffer */
            temp = uca1_rx_wr++;
            UCA1_Char_Rx[temp] = rx_char;
            if(uca1_rx_wr >= SERIAL_RING_SIZE){
                uca1_rx_wr = SERIAL_BEGINNING;
            }

            /* Rolling LCD display buffer */
            for(i = 0u; i < (RX_DISPLAY_LEN - 1u); i++){
                rx_display_buf[i] = rx_display_buf[i + 1u];
            }
            rx_display_buf[RX_DISPLAY_LEN - 1u] = rx_char;
            rx_display_updated = TRUE;

            /* ^ command detection */
            if(rx_char == '^'){
                fram_cmd_active = TRUE;
                fram_cmd_count  = 0u;
            } else if(fram_cmd_active){
                /* Accumulate one character then signal ready */
                fram_cmd_buf[fram_cmd_count++] = rx_char;
                if(fram_cmd_count >= 1u){
                    fram_cmd_buf[fram_cmd_count] = '\0';
                    fram_cmd_ready  = TRUE;
                    fram_cmd_active = FALSE;
                    fram_cmd_count  = 0u;
                }
                /* Do NOT forward to IOT */
            } else {
                /* Passthrough to IOT */
                while(!(UCA0IFG & UCTXIFG));
                UCA0TXBUF = (unsigned char)rx_char;
            }
            break;

        case 4:   /* TXIFG – not used */
            break;

        default:
            break;
    }
}

/*==============================================================================
 * EUSCI_A0_VECTOR  –  UCA0 ISR  (IOT port, J9)
 *
 * Collects one line from the IOT into iot_rx_msg[]:
 *   \r  -> null-terminate, set iot_msg_ready, disable RX interrupt
 *   \n  -> ignore
 *   printable -> store if room
 *
 * Forwards every character to PC (UCA1) if pc_ready is TRUE.
 *==============================================================================*/
#pragma vector = EUSCI_A0_VECTOR
__interrupt void eUSCI_A0_ISR(void){
    char rx_char;

    switch(__even_in_range(UCA0IV, 0x08)){

        case 0:
            break;

        case 2:   /* RXIFG */
            rx_char = (char)UCA0RXBUF;

            if(rx_char == '\r'){
                iot_rx_msg[iot_rx_count] = '\0';  /* terminate line           */
                iot_rx_count  = 0u;
                iot_msg_ready = TRUE;
                UCA0IE &= ~UCRXIE;                /* disable until main clears */
            } else if(rx_char == '\n'){
                /* ignore LF */
            } else if(rx_char >= 0x20 && iot_rx_count < (IOT_MSG_SIZE - 1u)){
                iot_rx_msg[iot_rx_count++] = rx_char;
            }

            /* Forward to PC if ready */
            if(pc_ready){
                while(!(UCA1IFG & UCTXIFG));
                UCA1TXBUF = (unsigned char)rx_char;
            }
            break;

        case 4:
            break;

        default:
            break;
    }
}
