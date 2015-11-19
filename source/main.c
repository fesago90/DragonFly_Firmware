#include "ConfigBits.h"
#include "Pins.h"
#include "RCComm.h"
#include "VMeas.h"
#include "Motors.h"
#include "FlightControl.h"

#ifdef DRAGONFLY_DRONE
#include "DFProtocol.h"
#include "RFComm.h"
#endif

#ifdef DRONE
#include "SDProtocol.h"
#include "WiRaComm.h"
#endif

// Options: nRF pipes, wing sequence, RC calibration.
int8_t wing_enable = -1;
int8_t rc_enable   = -1;
extern int8_t UserConfig; // actually not necessary to use a 1000 hz timer here
// System initialization
void Sys_Init();
void PLL_Init();
void Calibrate_Remote_Control();
void Prioritize_Interrupts();
void Motor_Voltage_Control();

int main()
{
    _SWDTEN = 0;    // Disable watchdog timer
    _GIE    = 0;    // Disable interrupts

    ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;

    __delay32(8000000/2);   // Wait about 500ms before change to PLL

    Sys_Init();

    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 100);
    while(wing_enable == -1 || rc_enable == -1) {
        while(UserConfig == 0); // actually not necessary to use a 1000 hz timer here
        // Wait for user inputs from SDK.
        #ifdef DRAGONFLY_DRONE
        DFP_Continue();
        #endif
        #ifdef DRONE
        WR_Handle_Comms();
        SDP_Continue();
        #endif
        UserConfig = 0; // actually not necessary to use a 1000 hz timer here
    }
    
    if(rc_enable == 1) {
        Calibrate_Remote_Control(); // no need rc calibration for the tuning phase
    }
    FC_Start_Calibrating();
    LED_Set_And_Wait(LEDOn,  LEDOff, LEDOff, LEDOff, 100);
    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOn, 100);
    LED_Set_And_Wait(LEDOff, LEDOn,  LEDOff, LEDOff, 100);
    LED_Set_And_Wait(LEDOff, LEDOff, LEDOn,  LEDOff, 100);
    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 100);

    // Apply a sequence of wing movements for dragonfly, or prop movements for
    // quadcopter with servos (brushless motors).
    if(wing_enable == 1) {
        FC_Servo_Check();
    }
    
    // Main loop, check sensor data and exute flight controls.
    while(1) {
        while((FC_Get_Status() & FCStatusReady) == 0);

        FC_Try_Process_Controls();
        if (!CAMReadNextFrame) // camera is disabled
            #ifdef DRAGONFLY_DRONE
            DFP_Continue();
            #endif
            #ifdef DRONE
            SDP_Continue();
            #endif
        Motor_Voltage_Control();
    }

    return 0;
}

void Calibrate_Remote_Control()
{
    int counter, tries = 0;
    
    do {
        for (counter = 0; counter < 8; counter++) {
            LED_Set_And_Wait(LEDOff, LEDOff, LEDOn, LEDOff, 50);
            LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOn, 50);
        }

        RC_Start_Calibrating_Boundaries();

        for (counter = 0; counter < 8; counter++) {
            LED_Set_And_Wait(LEDOff, LEDOff, LEDOn, LEDOff, 250);
            LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOn, 250);
        }

        tries++;
        
    } while (RC_Stop_Calibrating_Boundaries() == 0 && tries < 4);

    for (counter = 0; counter < 8; counter++) {
        LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOn, 100);
        LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 100);
    }

    RC_Start_Calibrating_Averages();

    for (counter = 0; counter < 4; counter++) {
        LED_Set_And_Wait(LEDOn, LEDOff, LEDOff, LEDOff, 250);
        LED_Set_And_Wait(LEDOff, LEDOn, LEDOff, LEDOff, 250);
    }

    RC_Stop_Calibrating_Averages();

    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 0);

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
/*
    MC_Init(); // tried to eliminate servo power-up jerk by sending min servo position immediately after dsPIC frequency configuration but didn't succeed
*/
    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 0);

    Acc_Init();
    Gyro_Init();

    LED_Set_And_Wait(LEDOn, LEDOff, LEDOff, LEDOff, 80);
    #ifdef DRAGONFLY_DRONE
    RF_Init();
    #endif
    #ifdef DRONE
    WR_Init();
    #endif


    Camera_Init();

    LED_Set_And_Wait(LEDOn, LEDOn, LEDOff, LEDOff, 80);

    MC_Init();
    RC_Init();

    LED_Set_And_Wait(LEDOn, LEDOn, LEDOn, LEDOff, 80);

    FC_Init();
    VMeas_Init();
    
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

void Motor_Voltage_Control()
{
    static int VoltageCounter = 100;
    VoltageData vd = *(VMeas_Get_Current_Filt_Data());
    VMeas_Filter_Current_Sample();
    if ((unsigned int)vd.V_Batt < VMEAS_MIN_LIBATT)
    {
        VoltageCounter--;
        if (VoltageCounter < 0)
        {
            MC_Kill_Motors();
            LED4_W = 0;
            VoltageCounter = 0;
        }
    }
    else if ((unsigned int)vd.V_Batt > VMEAS_GOOD_LIBATT)
    {
        VoltageCounter++;
        if (VoltageCounter > 100)
        {
            MC_Restart_Motors();
            LED4_W = 1;
            VoltageCounter = 100;
        }
    }
}

void FC_Set_Wing_Sequence(int8_t wing)
{
    if(wing != -1) {
        wing_enable = wing;
        LED1_W = 0;
    }
}

void FC_Set_RC_Calibration(int8_t rc)
{
    if(rc != -1) {
        rc_enable = rc;
        LED2_W = 0;
    }
}
