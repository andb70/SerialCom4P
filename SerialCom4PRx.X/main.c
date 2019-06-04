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
#include "disp7seg.h"  // include 7 segments display mapping 
#include "buttons.h"  // basic button management 


// Configuration Bit settings
// SYS_FREQ = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
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

#define SYS_FREQ 			(80000000L)
// timers management
//      IF we let compile time pre-processor calculate the PR1 (period)
//          PR_1 = Freq_clk/freq/PBDIV/PS-1
//          PBDIV = 1
//          PR_1 = SYS_FREQ/PRESCALE/TMR_FREQ-1
// precalculated values for frequencies, just choose TMR_FREQ value among: 
//  5, 10, 100 Hz; 1, 10, 100 kHz; 1, 10 MHz
#define TMR_FREQ            1000
// Interrupt frequency: 10 Hz
#if (TMR_FREQ==5)
#define PRESCALE       		T1_PS_1_256
#define PR_1                0xF423
// Interrupt frequency: 100 Hz
#elif (TMR_FREQ==10)
#define PRESCALE       		T1_PS_1_256
#define PR_1                0x7A11
// Interrupt frequency: 100 Hz
#elif (TMR_FREQ==100)
#define PRESCALE       		T1_PS_1_256
#define PR_1                0xC34
// Interrupt frequency: 1 kHz
#elif (TMR_FREQ==1000)
#define PRESCALE       		T1_PS_1_64
#define PR_1                0x4E1
// Interrupt frequency: 10 kHz
#elif (TMR_FREQ==10000)
#define PRESCALE       		T1_PS_1_8
#define PR_1                0x3E7
// Interrupt frequency: 100 kHz
#elif (TMR_FREQ==100000)
#define PRESCALE       		T1_PS_1_8
#define PR_1                0x63
// Interrupt frequency: 1 MHz
#elif (TMR_FREQ==1000000)
#define PRESCALE       		T1_PS_1_8
#define PR_1                0x9
// Interrupt frequency: 10 MHz
#elif (TMR_FREQ==10000000)
#define PRESCALE       		T1_PS_1_1
#define PR_1                0x7
#endif
void checkTimers();

struct {
    unsigned int elapsed : 1; // raised by the timer, is consumed in the main cycle
    unsigned int bounce : 1; // used to wait after edge and skip rebounce
    unsigned int pulse1s : 1; // used to do something every 1 sec
    unsigned int slWait : 1; // used to wait for serial line timeout
    unsigned int slWrite : 1; // serial Write state      
    unsigned int slClk : 1; // serial clock state      
} tmrFlags;



// initialize system and basic timer
void initializeSystem();

// initialize the IO configuration depending on which mapping is used
void initializePORTIO();
#define mapA 1
#define mapB 2
#define mapping mapB
/*	CONnector, PIN, PORT, A-map, B-map      mapping table
 * 					
 *	Pin 	Signal Name		PORT        A           B
 *   ------+---------------+-----------+-----------+-----------------------------
 *	CON4    DIGITAL:	
 *	1       D0(RXD1)        RD2         ClockTX		ClockTX
 *	2       D1(TXD1)        RD3         DataTX		DataTX
 *	3       D2(BUT)         RD4(+RD0)               cicla slBufferPosBit parametri sul display
 *	4       D3              RD5         btn4Up		Disp1   a
 *	5       D4              RD6         btn4Down	Disp2   b
 *	6       D5              RD7         btn3Up		Disp3   c
 *	7       D6              RD8         btn3Down	Disp4   d
 *	8       D7              RD11        btn2Up		Disp8   dp
 *		  |               |           |           |			
 *	CON5 	
 *	1       D8_MMC_#SS      RB13        ClockRX		ClockRX
 *	2       D9              RB14        DataRX		DataRX
 *	3       D10(#SS)        RG9         btn2Down	Disp7   g
 *	4       D11(MOSI)       RG8         btn1Up		Disp6   f
 *	5       D12(MISO)       RG7         btn1Down	Disp5   e
 *	6       D13(SCK/LED1)	RG6			
 *	7       GND				
 *	8       AREF	
 *		  |               |           |           |				
 *	CON2   ANALOG:			
 *	1       A0              RB1			
 *	2       A1              RB2			
 *	3       A2              RB3			
 *	4       A3              RB4			
 *	5       A4(SDA1)				
 *	6       A5(SCL1)				
 */

// hold the previous button state
unsigned int oldB;
unsigned int oldD;
unsigned int oldG;
#if (mapping == mapA)    
unsigned int oldG;
struct Button buttons[8]; // hold 8 button structures to inc/dec 4 parameters  
#elif (mapping == mapB) 
struct Button buttons[1]; // hold 1 button structure to cycle through 4 parameters 
#endif
// button configuration 
unsigned int maxBounceCount = 200 * TMR_FREQ / 1000; // anti-bounce period ms
void initializeButtons();

//hold the data sent/received through the serial line
#define slBufferMaxBit      31
char serialData[] = {0, 1, 2, 3};
unsigned char serialDataIndex = 0x00;

// SerialLine
void slRead();
void slWrite();
#define slPortTx            IOPORT_D
#define slLatTx             LATD
#define slClkBitTx          0x2
#define slClkPortBitTx      LATDbits.RD2
#define slDataBitTx         0x3
#define slDataPortBitTx     LATDbits.RD3
#define slPortRx            IOPORT_B
#define slClkBitRx          0xD
#define slClkPortBitRx      PORTBbits.RB13
#define slDataBitRx         0xE
#define slDataPortBitRx     PORTBbits.RB14

unsigned int slTimeoutCounter = 0;
//slMaxWait esempio: 5 * TMR_FREQ / 1000; > 5 ms
#define slMaxWait           20000
#define tickPerClkEdge      0
    
// use the green led to monitor serial RX clock
#define monitorLed(state)   LATGbits.LATG6 = state
#define monitorLedToggle()  LATGbits.LATG6 = !LATGbits.LATG6

#define ON                  1
#define OFF                 0

// program state definitions
// general
#define stateIdle 0x00
// input
#define stateSkipBounce 0x01
#define stateWrite 0x02
// serial line
#define stateSLReady 0x20
#define stateSLClockON 0x30
#define stateSLClockOFF 0x40
#define stateSLSuccess 0x50
#define stateSLFailure 0x60
#define stateSLReset 0x70 // wait for clock falling edge
// process
#define stateCycleParams 0x01

int main() {
    initializeSystem();

    initializePORTIO();
    showOnDisplay(display7s[0x20]); //display off
    monitorLed(OFF);
    initializeButtons();



    int programState = stateIdle;
    while (1) {
        checkTimers();
        checkButtons();
        slRead();

        switch (programState) {
            case stateIdle:
                if (buttons[CYCLE].btnDown == 1)
                    programState = stateCycleParams;
                break;
            case stateCycleParams:
                if (++serialDataIndex > 0x03)
                    serialDataIndex = 0x00;
                programState = stateIdle;
                tmrFlags.slWrite = ON;
                break;
        }
        showOnDisplay(display7s[serialData[serialDataIndex]]);
        // repeat the button state with display dot state
        LATDbits.LATD11 = PORTDbits.RD4;
        /*
        // flash the green led with 2 sec period
        if (tmrFlags.pulse1s == ON)
        {
            tmrFlags.pulse1s = OFF;
            monitorLedToggle(); 
                
        }*/
        slWrite();
    }
    return (EXIT_SUCCESS);
}

void initializeSystem() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //STEP 1. Configure cache, wait states and peripheral bus clock
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // STEP 2. configure Timer 1 using internal clock, 1:256 prescale

    OpenTimer1(T1_ON | T1_SOURCE_INT | PRESCALE, PR_1);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

    // enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();

    SYSTEMConfigPerformance(SYS_FREQ);
}

void initializePORTIO() {
    // configure PORTs
    // serial IO: clock & data, Tx & Rx
    PORTSetPinsDigitalOut(slPortTx, 1 << slClkBitTx); // ClockTX
    PORTSetPinsDigitalOut(slPortTx, 1 << slDataBitTx); // DataTX

    PORTSetPinsDigitalIn(slPortRx, 1 << slClkBitRx); // ClockRX
    PORTSetPinsDigitalIn(slPortRx, 1 << slDataBitRx); // DataRX


    mJTAGPortEnable(0); // Disable JTAG
#if (mapping == mapA) 
    PORTSetPinsDigitalIn(IOPORT_D, BIT_5); // btn4Up
    PORTSetPinsDigitalIn(IOPORT_D, BIT_6); // btn4Down
    PORTSetPinsDigitalIn(IOPORT_D, BIT_7); // btn3Up
    PORTSetPinsDigitalIn(IOPORT_D, BIT_8); // btn3Down
    PORTSetPinsDigitalIn(IOPORT_D, BIT_11); // btn2Up
    PORTSetPinsDigitalIn(IOPORT_G, BIT_9); // btn2Down
    PORTSetPinsDigitalIn(IOPORT_G, BIT_8); // btn1Up
    PORTSetPinsDigitalIn(IOPORT_G, BIT_7); // btn1Down    
#elif (mapping == mapB) 
    // RD4 D2(BUT) cicla slBufferPosBit parametri sul display
    PORTSetPinsDigitalIn(IOPORT_D, BIT_0); // (D2)BUT
    PORTSetPinsDigitalIn(IOPORT_D, BIT_4); // (D2)BUT

    PORTSetPinsDigitalOut(IOPORT_D, BIT_5); // Disp1
    PORTSetPinsDigitalOut(IOPORT_D, BIT_6); // Disp2
    PORTSetPinsDigitalOut(IOPORT_D, BIT_7); // Disp3
    PORTSetPinsDigitalOut(IOPORT_D, BIT_8); // Disp4

    PORTSetPinsDigitalOut(IOPORT_D, BIT_11); // Disp8

    PORTSetPinsDigitalOut(IOPORT_G, BIT_9); // Disp5
    PORTSetPinsDigitalOut(IOPORT_G, BIT_8); // Disp6
    PORTSetPinsDigitalOut(IOPORT_G, BIT_7); // Disp7

    PORTSetPinsDigitalOut(IOPORT_G, BIT_6); // Green LED
#endif
    // turn everything off
    slLatTx = 0;
    LATD = 0;
    LATG = 0;
}

void initializeButtons() {
    /*
     *	3       D2(BUT)         RD4     cicla slBufferPosBit parametri sul display
     * 
     *	4       D3              RD5     btn4Up
     *	5       D4              RD6     btn4Down
     *	6       D5              RD7     btn3Up
     *	7       D6              RD8     btn3Down
     *	8       D7              RD11	btn2Up
     * 
     *	3       D10(#SS)        RG9     btn2Down
     *	4       D11(MOSI)       RG8     btn1Up
     *	5       D12(MISO)       RG7     btn1Down
     */
#if (mapping == mapA)  
    // PORTG
    buttons[DEC1].btnBit = 0x7;
    buttons[INC1].btnBit = 0x8;
    buttons[DEC2].btnBit = 0x9;
    // PORTD
    buttons[INC2].btnBit = 0xB;
    buttons[DEC3].btnBit = 0x8;
    buttons[INC3].btnBit = 0x7;
    buttons[DEC4].btnBit = 0x6;
    buttons[INC4].btnBit = 0x5;

    oldG = PORTG;

#elif (mapping == mapB) 
    buttons[CYCLE].btnBit = 0x4;
#endif
    oldD = PORTD;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// STEP 3. configure the Timer 1 interrupt handler

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1Handler(void) {
    // clear the interrupt flag
    mT1ClearIntFlag();

    // set the notify bit
    tmrFlags.elapsed = 1;
}

unsigned char slTimeout(unsigned char enable)
{
    // reset the counter
    if (enable==OFF)
    {
        slTimeoutCounter = 0;
        return OFF;
    }
    if (tmrFlags.slWait == OFF)
       return OFF;
 
    // when we get the timer on then we inc the counter
    tmrFlags.slWait == OFF;   
    if (++slTimeoutCounter < slMaxWait)
        return OFF;
    
    // and return ON only when timer goes off
    slTimeoutCounter = slMaxWait;
    return ON;
}
void checkTimers() {
    static unsigned int pulse = 0;
    static unsigned int bounceCounter = 0;
    static unsigned int clkTickCount = 0;

    if (!tmrFlags.elapsed)
        return;

    tmrFlags.elapsed = OFF;
    if (++pulse > 999) {
        pulse = 0;
        tmrFlags.pulse1s = ON;
    }

    if (tmrFlags.bounce == ON) {
        if (++bounceCounter > maxBounceCount) {
            tmrFlags.bounce = OFF;
            bounceCounter = 0;
        }
    }
    
    tmrFlags.slWait = ON;
    
    if (tmrFlags.slWrite == ON) {
        if (++clkTickCount > tickPerClkEdge) {
            tmrFlags.slClk = !tmrFlags.slClk;
            clkTickCount = 0;
        }
    }
    if (tmrFlags.slWrite == OFF) {
        clkTickCount = 0;
    }
}

void checkButtons() {
    // https://www.allaboutcircuits.com/technical-articles/switch-bounce-how-to-deal-with-it/
    static unsigned char programState = stateIdle;
    // temporary buffer for button states
    // its used to hold button states during skipBounce period

    static struct {
        unsigned int f0 : 1;
        /*unsigned int f1: 1;
        unsigned int f2: 1;
        unsigned int f3: 1;
        unsigned int f4: 1;
        unsigned int f5: 1;
        unsigned int f6: 1;
        unsigned int f7: 1;*/
    } btnDowns;

    //static unsigned int oldD;
    int unsigned thisD;

    // if we get a button pressed wait and then write the output
    switch (programState) {
        case stateIdle:
            // trigger the button pressure
            thisD = PORTD;
            btnDowns.f0 = fTrig(thisD, buttons[CYCLE].btnBit, oldD);
            oldD = thisD;

            // if no pressure fall through write button 
            // else jump to anti-bounce wait state
            if (btnDowns.f0 == ON) {
                tmrFlags.bounce = ON;
                programState = stateSkipBounce;
                break;
            }
            programState = stateWrite;
        case stateWrite:
            buttons[CYCLE].btnDown = btnDowns.f0;
            btnDowns.f0 = OFF;
            programState = stateIdle;
            break;

        case stateSkipBounce:
            // wait in millisecons
            if (tmrFlags.bounce == ON)
                break;
            programState = stateWrite;
    }

    /* #if (mapping == mapA)    
         //static int oldG;
         int thisG;

         thisG = PORTG;    
         buttons[DEC1].btnDown = fTrig(thisG, buttons[DEC1].btnBit, oldG);
         buttons[INC1].btnDown = fTrig(thisG, buttons[INC1].btnBit, oldG);
         buttons[DEC2].btnDown = fTrig(thisG, buttons[DEC2].btnBit, oldG);
         oldG = thisG;
         buttons[INC2].btnDown = fTrig(thisD, buttons[INC2].btnBit, oldD);
         buttons[DEC3].btnDown = fTrig(thisD, buttons[DEC3].btnBit, oldD);
         buttons[INC3].btnDown = fTrig(thisD, buttons[INC3].btnBit, oldD);
         buttons[DEC4].btnDown = fTrig(thisD, buttons[DEC4].btnBit, oldD);
         buttons[INC4].btnDown = fTrig(thisD, buttons[INC4].btnBit, oldD);       
     #elif (mapping == mapB) 
         buttons[CYCLE].btnDown =  fTrig(thisD, buttons[CYCLE].btnBit, oldD);
         //buttons[CYCLE].btnUp = rTrig(thisD, buttons[CYCLE].btnBit, oldD);
     #endif
     oldD = thisD;  */
}

void slRead() {
    // wait for Clock raising edge
    // first Raise edge: reset SL and
    // wait for Clock falling edge
    // Falling edge: store data and
    // wait for next raising edge: count bit
    // if timeout > end failure
    // if count ended > end success

    static unsigned long buffer;
    static unsigned char slBufferPosBit;
    static unsigned char programState = stateSLReset;
    
    switch (programState) {
        case stateSLReset:
            monitorLed(OFF);
            // wait for clock down state
            if (slClkPortBitRx == ON) {
                break;
            }
            // clock ready, start another session
            programState = stateSLReady;
            break;
            
        case stateSLReady:
            // wait for the first CLOCK RAISING EDGE
            // this happens when the data is loaded, then we must wait for a 
            // CLOCK FALLING EDGE to READ the data
            if (slClkPortBitRx == OFF)
                break;
            // exit state condition found    
            // initialize session
            slBufferPosBit = 0;
            buffer = 0;
            slTimeout(OFF);
            monitorLed(ON);
            programState = stateSLClockON;
            
        case stateSLClockON:
            // FAILURE exit if the serial line is dead
//            if (slTimeout(ON)==ON)
//            {
//                programState = stateSLFailure;
//                break;
//            }
            // wait for CLOCK FALLING EDGE
            if (slClkPortBitRx == ON)
                break;
            // exit state condition found, reset timeout
            slTimeout(OFF);
            monitorLed(OFF);

            // load data bit into buffer
            buffer |= slDataPortBitRx << slBufferPosBit;
            // increment buffer position for the next loop
            // and check for completion
            if (++slBufferPosBit > slBufferMaxBit) {
                // transmission complete
                programState = stateSLSuccess;
                break;
            }
            // don't fall through, because we wait for timout reset
            programState = stateSLClockOFF;
            break;

        case stateSLClockOFF:
            // FAILURE exit if the serial line is dead
//            if (slTimeout(ON)==ON)
//            {
//                programState = stateSLFailure;
//                break;
//            }
            // wait for the clock raising edge
            if (slClkPortBitRx == OFF) {
                break;
            }
            // exit state condition found, reset timeout
            slTimeout(OFF);
            monitorLed(ON);

            programState = stateSLClockON;
            break;

        case stateSLSuccess:
            // stop timeout monitor
            tmrFlags.slWait = OFF;

            // copy buffer into data 
            serialData[0] = (buffer & 0xFF);
            serialData[1] = (buffer >> 8 & 0xFF);
            serialData[2] = (buffer >> 16 & 0xFF);
            serialData[3] = (buffer >> 24 & 0xFF);
            // continue listening, just fall through

        case stateSLFailure:
            // notify error
            programState = stateSLReset;
    }

}

void slWrite() {
    static unsigned long buffer;
    static unsigned char slBufferPosBit;
    static unsigned char programState = stateIdle;
    switch (programState) {
        case stateIdle:
            // wait for the begin of operation
            if (tmrFlags.slWrite == OFF)
                break;
            // exit state condition found    
            // initialize session
            slBufferPosBit = 0;
            buffer = serialData[0] | serialData[1] << 8 | serialData[2] << 16 | serialData[3] << 24;
            buffer = 0x0A0B0C0D;
            programState = stateSLClockON;

        case stateSLClockON:
            // At raising edge we load the buffer bit
            if (tmrFlags.slClk == OFF)
                break;
            unsigned int lb = slLatTx;
            unsigned int lm = (slLatTx & ~(1 << slClkBitTx | 1 << slDataBitTx));
            unsigned int lq = tmrFlags.slClk << slClkBitTx;
            unsigned int lz = (buffer >> slBufferPosBit & 0x1) << slDataBitTx;
            slLatTx = (slLatTx & ~(1 << slClkBitTx | 1 << slDataBitTx)) // get the actual port value
                    | tmrFlags.slClk << slClkBitTx // write the clock state
                    | (buffer >> slBufferPosBit & 0x1) << slDataBitTx; // write the data bit

            programState = stateSLClockOFF;
            break;

        case stateSLClockOFF:

            if (tmrFlags.slClk == ON)
                break;
            lb = slLatTx;
            lm = (slLatTx & ~(1 << slClkBitTx | 1 << slDataBitTx));
            lq = tmrFlags.slClk << slClkBitTx;
            lz = (buffer >> slBufferPosBit & 0x1) << slDataBitTx;
            slLatTx = (slLatTx & ~(1 << slClkBitTx)) // get the actual port value
                    | tmrFlags.slClk << slClkBitTx; // write the clock state

            // increment buffer position for the next loop
            // and check for completion
            if (++slBufferPosBit > slBufferMaxBit) {
                // transmission complete
                programState = stateSLSuccess;
                break;
            }

            programState = stateSLClockON;
            break;

        case stateSLSuccess:
            // do something
            tmrFlags.slWrite = OFF;
            programState = stateIdle;
            break;
    }

}

void showOnDisplay(unsigned char dato) {
    // https://www.microchip.com/forums/m821163.aspx > about setting port bits
    //	D3              RD5     	Disp1   a
    //	D4              RD6     	Disp2   b
    //	D5              RD7     	Disp3   c
    //	D6              RD8     	Disp4   d    
    //	D7              RD11		Disp8   dp
    int d = (LATD & 0xF61F);
    int abcd = ((dato & 0x0F) << 5);
    int dp = ((dato & 0x80) << 4);
    LATD = d | abcd | dp;

    //	D10(#SS)        RG9     	Disp7   g
    //	D11(MOSI)       RG8     	Disp6   f
    //	D12(MISO)       RG7     	Disp5   e
    int g = (LATG & 0xFC7F);
    int efg = ((dato & 0x0070) << 3);
    LATG = g | efg;
}