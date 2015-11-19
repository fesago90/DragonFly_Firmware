
#include "DebugTiming.h"



// Instructions: When the Stopwatch doesn't work, put breakpoints on the lines
//               of code indicated in the "Start" and "Stop" functions.




void Debug_Timer_Init(void)
{
    T8CONbits.TON = 0;
    T8CONbits.TSIDL = 0;
    T8CONbits.TGATE = 0;
    T8CONbits.TCKPS = 0b00; // Prescale is 1:1
    T8CONbits.TCS = 0; // Use peripheral clock
    T8CONbits.T32 = 1; // use T8 and T9 as a 32-bit timer
    TMR8 = 0;

    T9CONbits.TSIDL = 0;
    TMR9 = 0;

    PR9 = 0xFFFF;
    PR8 = 0xFFFF; // Can timer for up to 71 seconds

    _T9IE = 0;
    _T9IF = 0; // Disable associated interrupt
}

inline void Debug_Timer_Reset(void)
{
    T8CONbits.TON = 0;
    TMR9HLD = 0;
    TMR8 = 0;
    T8CONbits.TON = 1;
}

inline void Debug_Timer_Stop(void)
{
    T8CONbits.TON = 0;
    long TotalCycles;
    TotalCycles = TMR9;
    TotalCycles <<= 16;
    TotalCycles |= TMR8;
    // Place a breakpoint on the line below:
    TotalCycles += 13;
}

inline void Debug_Timer_Start(void)
{
    TMR9 = 0;
    TMR9HLD = 0;
    TMR8 = 0;
    // Place a breakpoint on the line below
    T8CONbits.TON = 1;
}
