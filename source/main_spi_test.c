#include "ConfigBits.h"
#include "Pins.h"
#include "WiRaComm.h"

#ifdef DRONE
#include "SDProtocol.h"
#endif

extern int8_t UserConfig; // actually not necessary to use a 1000 hz timer here
// System initialization
void Sys_Init();
void PLL_Init();
void Calibrate_Remote_Control();
void Prioritize_Interrupts();
void Motor_Voltage_Control();

int main()
{
    int i = 0;
    _SWDTEN = 0;    // Disable watchdog timer
    _GIE    = 0;    // Disable interrupts

    ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;

    __delay32(8000000/2);   // Wait about 500ms before change to PLL

    Sys_Init();
    LED1_W = 1; LED2_W = 1; LED3_W = 1; LED4_W = 1; // turn off

    LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOn, 1000);
    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 1000);
   // LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOn, 2000);
   // LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 2000);
   // LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOn, 3000);
   // LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 3000);

    LED1_W = 1; LED2_W = 1; LED3_W = 1; LED4_W = 1; // turn off

    while(1)
    {
        WR_Handle_Comms();
        if(led)
        {
            i++;
            //if (rx_data == 200)
            //{
                LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOn, 100);
                LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 100);
            //}
            //led = 0;
        }
    }
    return 0;
}

void PLL_Init()
{
    _TRISbit(LED4_PORT, LED4_BIT) = 0;

    LED4_W = 0; // on

    // Fcy = Fosc/2 = Fin*M/(N1*N2) / 2 = 25*240/(25*2) / 2 = 60 MHz, a.k.a.,
    // 60MIPS with BLUE external oscillator of 25 MHz.
    PLLFBDbits.PLLDIV   = (PLL_M - 2);       // PLL_M = SYSTEM_FREQUENCY_IN_MHZ * PLL_N2 * 2 = 60*2*2 = 240 Mhz
    CLKDIVbits.PLLPOST  = (PLL_N2 >> 1) - 1; // PLLPOST = 0, a.k.a., postscaler = 2
    CLKDIVbits.PLLPRE   = PLL_N1 - 2;        // PLL_N1 = 25

    __builtin_write_OSCCONH(0b011);          // New osc will be Pri w/ PLL
    __builtin_write_OSCCONL(0x01);           // Request osc switch

    while(OSCCONbits.COSC != 0b011);         // Wait for new oscillator to become Pri w/ PLL
    while(OSCCONbits.LOCK == 0);             // Wait for PLL to lock

    LED4_W = 1;                              // off
}

void Sys_Init()
{
    _TRISbit(D, 11) = 0;                     // TRISDbits.TRISD11 = 0
    _LATbit(D, 11) = 0;                      // LATDbits.LATD11 = 0; Turn off analog 3.0V regulator

    LED_Init();
    PLL_Init();

    __delay_ms(100);                         // Give PLL more time than it needs to stabilize
    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 0);

    LED_Set_And_Wait(LEDOn, LEDOff, LEDOff, LEDOff, 80);

    WR_Init();

    LED_Set_And_Wait(LEDOn, LEDOn, LEDOff, LEDOff, 80);
    Prioritize_Interrupts();

    LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOn, 80);
}

void Prioritize_Interrupts()
{
    // Establish priority of all interrupts
    VSYNC_IP       = VSYNC_PRIORITY;
    PCLK_IP        = PCLK_PRIORITY;
    NRF_DMA_IP     = NRF_DMA_PRIORITY;
    NRF_INT_IP     = NRF_INT_PRIORITY;
    FLAP_TIMER_IP  = FLAP_TIMER_PRIORITY;
    ACC_INT_IP     = ACC_INT_PRIORITY;
    ACC_DMA_IP     = ACC_DMA_PRIORITY;
    GYRO_INT_IP    = GYRO_INT_PRIORITY;
    GYRO_DMA_IP    = GYRO_DMA_PRIORITY;
    THROTTLE_IC_IP = THROTTLE_IC_PRIORITY;
    ROLL_IC_IP     = ROLL_IC_PRIORITY;
    PITCH_IC_IP    = PITCH_IC_PRIORITY;
    YAW_IC_IP      = YAW_IC_PRIORITY;
    SHORT_HREF_IP  = SHORT_HREF_PRIORITY;
    LONG_HREF_IP   = LONG_HREF_PRIORITY;
    LOOP_TMR_IP    = LOOP_TMR_PRIORITY;

    // Enable interrupt nesting
    _NSTDIS = 0; // | should be 1

    // Enable interrupts globally
    _GIE = 1;

}

