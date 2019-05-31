/* 
 * File:   main.c
 * Author: Conan
 *
 * Created on May 31, 2019, 15:51
 */

#include <p32xxxx.h> // Include PIC32 specifics header file
#include <plib.h> // Include the PIC32 Peripheral Library
#include <stdio.h>
#include <stdlib.h>

// DEVCFG2
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider (PLL Divide by 256)
// DEVCFG1
#pragma config FNOSC = PRIPLL // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = ON // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_8 // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576 // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
// DEVCFG0
#pragma config DEBUG = OFF // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2 // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF // Program Flash Write Protect (Disable)
#pragma config BWP = OFF // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF // Code Protect (Protection Disabled)
/// timer
// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
// Let compile time pre-processor calculate the PR1 (period)
#define SYS_FREQ 			(80000000L)
#define PB_DIV         		8
#define PRESCALE       		256
#define TOGGLES_PER_SEC		1000
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define SYSCLK 80000000L // Give the system?s clock frequency

// rising edge: when port is 0 and oldport is 1
#define rTrig(port, btn, old) ((~port & btn) & (old & btn))
// falling edge: when port is 1 and oldport is 0
#define fTrig(port, btn, old) ((port & btn) & (~old & btn))

#define mapA 1
#define mapB 2
#define mapping 1

/*						
 *	CON4 ? DIGITAL:			PORT    A           B
 *	Pin #	Signal Name				
 *	1       D0(RXD1)        RD2     ClockTX		ClockTX
 *	2       D1(TXD1)        RD3     ClockRX		ClockRX
 *	3       D2(BUT)         RD4                 cicla i parametri sul display
 *	4       D3              RD5     DataTX		DataTX
 *	5       D4              RD6     DataRX		DataRX
 *	6       D5              RD7			
 *	7       D6              RD8     btn4Up		
 *	8       D7              RD11	btn4Down	Disp1   a
 *						
 *	CON5 ? DIGITAL:			PORT    A           B
 *	Pin #	Signal Name				
 *	1       D8_MMC_#SS      RB13	btn3Up		Disp2   b
 *	2       D9              RB14	btn3Down	Disp3   c
 *	3       D10(#SS)        RG9     btn2Up		Disp4   g
 *	4       D11(MOSI)       RG8     btn2Down	Disp5   f
 *	5       D12(MISO)       RG7     btn1Up		Disp6   e
 *	6       D13(SCK/LED1)	RG6     btn1Down	Disp7   d
 *	7       GND				
 *	8       AREF				
 *						
 *	CON2 ? ANALOG:					
 *	Pin #	Signal Name				
 *	1       A0              RB1			
 *	2       A1              RB2			
 *	3       A2              RB3			
 *	4       A3              RB4			
 *	5       A4(SDA1)				
 *	6       A5(SCL1)				
 */
/*	7 segments Display
 * 									
 *		  --a--							
 *		|		|						
 *		f		b						
 *		|		|						
 *		  --g--							
 *		|		|						
 *		e		c						
 *		|		|						
 *		  --d--							
 */										
static const unsigned char display7s[] = {										
	0x7E,	0x30,	0x6D,	0x79,		//	0	1	2	3
	0x33,	0x5B,	0x5F,	0x70,		//	4	5	6	7
	0x7F,	0x73,	0x77,	0x1F,		//	8	9	A	b
	0x4E,	0x3D,	0x4F,	0x47		//	C	d	E	F
};										

char timerElapsed = 0;

void showOnDisplay(unsigned int indexArray) {
    char dato = display7s[indexArray];
//	RD11    Disp1   a   6 >> 11
    LATD = (LATD & 0xF7FF) | ((dato & 0xFFBF) << 5); // a
//	RB13	Disp2   b   5 >> 14
//	RB14	Disp3   c   4 >> 13 
    LATB = (LATB & 0x9FFF) | ((dato & 0xFFCF) << 9); //  bc
//	RG9     Disp4   g   3 >> 9
//	RG8     Disp5   f   2 >> 8
//	RG7     Disp6   e   1 >> 7
//	RG6     Disp7   d   0 >> 6
    LATG = (LATG & 0xFC3F) | ((dato & 0xFFF0) << 6); //    gfed
}



void initializePORTIO() {
    // configure PORTs
    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_2); //Configure pin as output. Clock TX
    PORTSetPinsDigitalIn(IOPORT_D, BIT_3); // Configure pin as input. Clock RX    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_5); //Configure pin as output. Data TX
    PORTSetPinsDigitalIn(IOPORT_D, BIT_6); // Configure pin as input. Data RX
 
    mJTAGPortEnable(0); // Disable JTAG
    if (mapping == mapA) {
        PORTSetPinsDigitalIn(IOPORT_D, BIT_8); // Configure pin as input. btn4Up
        PORTSetPinsDigitalIn(IOPORT_D, BIT_11); //Configure pin as input. btn4Down
        PORTSetPinsDigitalIn(IOPORT_B, BIT_13); //Configure pin as input. btn3Up
        PORTSetPinsDigitalIn(IOPORT_B, BIT_14); //Configure pin as input. btn3Down
        PORTSetPinsDigitalIn(IOPORT_G, BIT_9); // Configure pin as input. btn2Up
        PORTSetPinsDigitalIn(IOPORT_G, BIT_8); // Configure pin as input. btn2Down
        PORTSetPinsDigitalIn(IOPORT_G, BIT_7); // Configure pin as input. btn1Up
        PORTSetPinsDigitalIn(IOPORT_G, BIT_6); // Configure pin as input. btn1Down
    }
    else if (mapping == mapB) {
        PORTSetPinsDigitalOut(IOPORT_D, BIT_11); //Configure pin as output. Disp1
        PORTSetPinsDigitalOut(IOPORT_B, BIT_13); //Configure pin as output. Disp2
        PORTSetPinsDigitalOut(IOPORT_B, BIT_14); //Configure pin as output. Disp3
        PORTSetPinsDigitalOut(IOPORT_G, BIT_9); // Configure pin as output. Disp4
        PORTSetPinsDigitalOut(IOPORT_G, BIT_8); // Configure pin as output. Disp5
        PORTSetPinsDigitalOut(IOPORT_G, BIT_7); // Configure pin as output. Disp6
        PORTSetPinsDigitalOut(IOPORT_G, BIT_6); // Configure pin as output. Disp7
    }

    LATB = 0;
    LATD = 0;
    LATG = 0;
}


/*
 * 
 */
int main() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //STEP 1. Configure cache, wait states and peripheral bus clock
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // STEP 2. configure Timer 1 using internal clock, 1:256 prescale

    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

    // enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();


    SYSTEMConfigPerformance(SYSCLK);
    TRISB = 0x0002;

    initializePORTIO();

    while (1) {

    }
    return (EXIT_SUCCESS);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// STEP 3. configure the Timer 1 interrupt handler

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1Handler(void) {
    // clear the interrupt flag
    mT1ClearIntFlag();

    // set the notify bit
    timerElapsed = 1;
}