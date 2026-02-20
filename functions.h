//******************************************************************************
//
//  Description: Function prototypes for SMCLK Demo
//
//******************************************************************************

// Main
void main(void);

// Initialization
void Init_Conditions(void);

// Interrupts
void enable_interrupts(void);
__interrupt void Timer0_B0_ISR(void);
__interrupt void switch1_interrupt(void);
__interrupt void switch2_interrupt(void);

// Clocks
void Init_Clocks(void);
void Set_SMCLK_8MHz(void);
void Set_SMCLK_500kHz(void);
void Set_SMCLK_1MHz(void);

// LED Configurations
void Init_LEDs(void);

// LCD
void Display_Process(void);
void Display_Update(char p_L1,char p_L2,char p_L3,char p_L4);
void Init_LCD(void);
void lcd_clear(void);
void lcd_BIG_mid(void);
void lcd_BIG_bot(void);
void lcd_4line(void);

// Ports
void Init_Ports(void);
void Init_Port1(void);
void Init_Port2(void);
void Init_Port3(char smclk);  // Now accepts argument!
void Init_Port4(void);
void Init_Port5(void);
void Init_Port6(void);

// Switches
void Init_Switches(void);
void enable_switch_SW1(void);
void enable_switch_SW2(void);
void disable_switch_SW1(void);
void disable_switch_SW2(void);
void Switches_Process(void);
void Switch1_Process(void);
void Switch2_Process(void);

// Timers
void Init_Timers(void);
void Init_Timer_B0(void);
void Init_Timer_B3(void);
void delay_ms(unsigned int ms);
