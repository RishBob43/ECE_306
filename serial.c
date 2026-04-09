/*------------------------------------------------------------------------------
 * File:        serial.c
 * Target:      MSP430FR2355
 *
 * Description: eUSCI_A0 (IOT/J9) and eUSCI_A1 (PC/J14) UART bridge.
 *
 *              Project 9 data path:
 *
 *                PC → UCA1 RX ISR → PC_2_IOT[] → UCA0 TX ISR → IOT
 *                IOT → UCA0 RX ISR → IOT_2_PC[] → UCA1 TX ISR → PC
 *
 *              Gate:
 *                pc_tx_enabled starts FALSE.  The first character received
 *                on UCA1 sets it TRUE and opens the PC transmit path.
 *
 *              ^ command parser:
 *                Characters arriving on UCA1 that start with '^' are FRAM
 *                commands and are NOT forwarded to the IOT.  They are
 *                accumulated in cmd_buf[] until 0x0D (CR) is received, at
 *                which point cmd_ready is set for the foreground to act on.
 *
 *              TX interrupt pattern  (matches IOT code skeleton):
 *                RX ISR stores char in ring buffer, then enables the TX
 *                interrupt on the other UART.
 *                TX ISR drains the ring buffer one char at a time, then
 *                disables itself when the buffer is empty.
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "serial.h"
#include "macros.h"
#include "ports.h"
#include <string.h>

/*------------------------------------------------------------------------------
 * IOT bridge ring buffers
 *------------------------------------------------------------------------------*/
volatile unsigned char IOT_2_PC[SMALL_RING_SIZE];
volatile unsigned int  iot_rx_wr  = BEGINNING;
         unsigned int  iot_rx_rd  = BEGINNING;
         unsigned int  direct_iot = BEGINNING;

volatile unsigned char PC_2_IOT[SMALL_RING_SIZE];
volatile unsigned int  usb_rx_wr  = BEGINNING;
         unsigned int  usb_rx_rd  = BEGINNING;
         unsigned int  direct_usb = BEGINNING;

/*------------------------------------------------------------------------------
 * PC TX gate and ^ command state
 *------------------------------------------------------------------------------*/
volatile unsigned char pc_tx_enabled = FALSE;

volatile unsigned char cmd_active = FALSE;
volatile unsigned char cmd_buf[CMD_BUF_SIZE];
volatile unsigned int  cmd_len    = 0u;
volatile unsigned char cmd_ready  = FALSE;

/*------------------------------------------------------------------------------
 * Legacy display buffer (LCD line 0 passthrough view)
 *------------------------------------------------------------------------------*/
volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u] = "          ";
volatile unsigned char rx_display_updated = FALSE;

/*------------------------------------------------------------------------------
 * Baud tracking
 *------------------------------------------------------------------------------*/
volatile unsigned char current_baud = BAUD_115200;
volatile unsigned char iot_baud     = BAUD_115200;

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
        default:  /* BAUD_115200 */
            *pBRW  = BR_115200_UCBRx;
            *pMCTL = BR_115200_MCTLW;
            break;
    }
}

/*==============================================================================
 * Init_Serial_UCA0   (IOT side)
 *==============================================================================*/
void Init_Serial_UCA0(unsigned char baud){
    unsigned int i;

    for(i = BEGINNING; i < SMALL_RING_SIZE; i++){
        IOT_2_PC[i] = 0x00u;
        PC_2_IOT[i] = 0x00u;
    }
    iot_rx_wr  = BEGINNING;
    iot_rx_rd  = BEGINNING;
    direct_iot = BEGINNING;
    usb_rx_wr  = BEGINNING;
    usb_rx_rd  = BEGINNING;
    direct_usb = BEGINNING;

    iot_baud = baud;

    UCA0CTLW0  = RESET_STATE;
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;

    apply_baud_registers(baud, &UCA0BRW, &UCA0MCTLW);

    UCA0CTLW0 &= ~UCSWRST;
    UCA0IE    |=  UCRXIE;    /* Enable RX; TX interrupt enabled on demand     */
}

/*==============================================================================
 * Init_Serial_UCA1   (PC backdoor)
 *==============================================================================*/
void Init_Serial_UCA1(unsigned char baud){
    unsigned int i;

    current_baud     = baud;
    pc_tx_enabled    = FALSE;   /* Gate: wait for first PC char               */

    cmd_active       = FALSE;
    cmd_len          = 0u;
    cmd_ready        = FALSE;

    for(i = 0u; i < RX_DISPLAY_LEN; i++){
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
 * Set_Baud_UCA0  (runtime baud change for IOT port)
 *==============================================================================*/
void Set_Baud_UCA0(unsigned char baud){
    iot_baud = baud;
    UCA0CTLW0 |=  UCSWRST;
    apply_baud_registers(baud, &UCA0BRW, &UCA0MCTLW);
    UCA0CTLW0 &= ~UCSWRST;
    UCA0IE    |=  UCRXIE;
}

/*==============================================================================
 * Set_Baud_UCA1  (runtime baud change for PC port)
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
 * UCA1_Transmit_String  (blocking, foreground, to PC)
 *   Respects the pc_tx_enabled gate.
 *==============================================================================*/
void UCA1_Transmit_String(const char *str, unsigned int len){
    unsigned int i;
    if(!pc_tx_enabled){ return; }
    for(i = 0u; i < len; i++){
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = (unsigned char)str[i];
    }
}

/*==============================================================================
 * UCA0_Transmit_String  (blocking, foreground, to IOT)
 *==============================================================================*/
void UCA0_Transmit_String(const char *str, unsigned int len){
    unsigned int i;
    for(i = 0u; i < len; i++){
        while(!(UCA0IFG & UCTXIFG));
        UCA0TXBUF = (unsigned char)str[i];
    }
}

/*==============================================================================
 * IOT_Send_Command
 *   Transmits a null-terminated command string to the IOT, appending CR+LF.
 *   Always use this when sending AT commands from FRAM code.
 *==============================================================================*/
void IOT_Send_Command(const char *cmd, unsigned int len){
    UCA0_Transmit_String(cmd, len);
    /* Append CR then LF */
    while(!(UCA0IFG & UCTXIFG)); UCA0TXBUF = 0x0Du;
    while(!(UCA0IFG & UCTXIFG)); UCA0TXBUF = 0x0Au;
}

/*==============================================================================
 * IOT_Reset
 *   Drives P3.7 (IOT_EN) LOW for IOT_RESET_MS then releases it HIGH.
 *   Call after Init_Ports() and before issuing any AT commands.
 *==============================================================================*/
void IOT_Reset(void){
    P3OUT &= ~IOT_EN_CPU;            /* Assert reset (active LOW)              */
    __delay_cycles(8000u * IOT_RESET_MS);   /* 150 ms @ 8 MHz                 */
    P3OUT |=  IOT_EN_CPU;            /* Release reset                          */
    __delay_cycles(8000u * IOT_SETTLE_MS);  /* 500 ms settle                  */
}

/*==============================================================================
 * EUSCI_A0_VECTOR  –  UCA0 ISR  (IOT side)
 *
 * Case 2 (RX):
 *   Store received byte in IOT_2_PC ring buffer.
 *   Enable UCA1 TX interrupt so it drains toward PC.
 *
 * Case 4 (TX):
 *   Drain next byte from PC_2_IOT toward IOT.
 *   Disable TX interrupt when buffer empty.
 *==============================================================================*/
#pragma vector = EUSCI_A0_VECTOR
__interrupt void eUSCI_A0_ISR(void){
    unsigned int temp;

    switch(__even_in_range(UCA0IV, 0x08)){

        case 0:
            break;

        case 2:   /* RXIFG – char arrived from IOT                            */
            temp = iot_rx_wr++;
            IOT_2_PC[temp] = UCA0RXBUF;
            if(iot_rx_wr >= SMALL_RING_SIZE){
                iot_rx_wr = BEGINNING;
            }
            /* Forward to PC if gate is open                                  */
            if(pc_tx_enabled){
                UCA1IE |= UCTXIE;   /* Let UCA1 TX ISR drain IOT_2_PC         */
            }
            break;

        case 4:   /* TXIFG – ready to send next byte to IOT                  */
            UCA0TXBUF = PC_2_IOT[direct_iot++];
            if(direct_iot >= SMALL_RING_SIZE){
                direct_iot = BEGINNING;
            }
            if(direct_iot == usb_rx_wr){
                UCA0IE &= ~UCTXIE;  /* Buffer drained – disable TX interrupt  */
            }
            break;

        default:
            break;
    }
}

/*==============================================================================
 * EUSCI_A1_VECTOR  –  UCA1 ISR  (PC backdoor)
 *
 * Case 2 (RX):
 *   First char received: open the pc_tx_enabled gate.
 *   If char is '^': start command accumulation mode.
 *   If cmd_active: accumulate into cmd_buf until CR; set cmd_ready on CR.
 *     Commands are NOT forwarded to IOT.
 *   Otherwise: store in PC_2_IOT and enable UCA0 TX to forward to IOT.
 *   Update LCD rolling display buffer.
 *
 * Case 4 (TX):
 *   Drain IOT_2_PC toward PC.
 *   Disable when buffer empty.
 *==============================================================================*/
#pragma vector = EUSCI_A1_VECTOR
__interrupt void eUSCI_A1_ISR(void){
    unsigned int temp;
    unsigned char rx_char;
    unsigned int i;

    switch(__even_in_range(UCA1IV, 0x08)){

        case 0:
            break;

        case 2:   /* RXIFG – char arrived from PC                            */
            rx_char = (unsigned char)UCA1RXBUF;

            /* Open PC TX gate on first received character                    */
            if(!pc_tx_enabled){
                pc_tx_enabled = TRUE;
            }

            /* ---- ^ command parser ---- */
            if(rx_char == CMD_CHAR){
                /* Start of a FRAM command sequence                           */
                cmd_active = TRUE;
                cmd_len    = 0u;
                cmd_ready  = FALSE;
                /* Do NOT forward '^' to IOT                                  */
                break;
            }

            if(cmd_active){
                if(rx_char == CMD_TERMINATOR){
                    /* Command complete                                        */
                    cmd_buf[cmd_len] = '\0';
                    cmd_active = FALSE;
                    cmd_ready  = TRUE;
                    /* Do NOT forward CR to IOT                               */
                } else {
                    if(cmd_len < (CMD_BUF_SIZE - 1u)){
                        cmd_buf[cmd_len++] = rx_char;
                    }
                    /* Do NOT forward command chars to IOT                    */
                }
                break;
            }

            /* ---- Normal passthrough: store in PC_2_IOT ---- */
            temp = usb_rx_wr++;
            PC_2_IOT[temp] = rx_char;
            if(usb_rx_wr >= SMALL_RING_SIZE){
                usb_rx_wr = BEGINNING;
            }
            UCA0IE |= UCTXIE;   /* Enable UCA0 TX to drain toward IOT         */

            /* ---- Update LCD rolling display buffer ---- */
            for(i = 0u; i < (RX_DISPLAY_LEN - 1u); i++){
                rx_display_buf[i] = rx_display_buf[i + 1u];
            }
            rx_display_buf[RX_DISPLAY_LEN - 1u] = (char)rx_char;
            rx_display_updated = TRUE;
            break;

        case 4:   /* TXIFG – ready to send next byte to PC                  */
            UCA1TXBUF = IOT_2_PC[direct_usb++];
            if(direct_usb >= SMALL_RING_SIZE){
                direct_usb = BEGINNING;
            }
            if(direct_usb == iot_rx_wr){
                UCA1IE &= ~UCTXIE;  /* Buffer drained – disable TX interrupt  */
            }
            break;

        default:
            break;
    }
}
