
#include "Common.h"
#include "Debug.h"

void Display_Error(ErrNum errNum)
{
    INTCON2bits.GIE = 0;        // Disable maskable interrupts

    _TRISbit(LED1_PORT, LED1_BIT) = 0;
    _TRISbit(LED2_PORT, LED2_BIT) = 0;
    _TRISbit(LED3_PORT, LED3_BIT) = 0;
    _TRISbit(LED4_PORT, LED4_BIT) = 0;

    __builtin_write_OSCCONH(0b110);     // New osc will be FRC div 16
    __builtin_write_OSCCONL(0x01);      // Request osc switch
    while(OSCCONbits.COSC != 0b110);

    while (1) {
        LED1_W = errNum >> 0 & 0x1;
        LED2_W = errNum >> 1 & 0x1;
        LED3_W = errNum >> 2 & 0x1;
        LED4_W = errNum >> 3 & 0x1;

        __delay32(250000);     // About 1 sec at 8MHz FRC div 16
        LED1_W = 0;
        LED2_W = 0;
        LED3_W = 0;
        LED4_W = 0;
        __delay32(62500);     // About 250 msec at 8MHz FRC div 16
    }
}
