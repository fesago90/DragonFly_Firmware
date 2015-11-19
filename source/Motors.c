#include "Motors.h"
#include "Common.h"

#define FLAP_PERIOD  (int)((float)FCY / ((float)64 * (float)FLAP_FREQ_IN_HZ))

MotorPWM MCCurrentData = {0, 0, 0, 0};
PropPWM  PCCurrentData = {0, 0, 0};
uint8_t  MCMotorsKilled = 0;
extern int16_t FCPitchAdd, FCRollAdd;

void MC_Init_PWM();
void MC_Init_Flap_Timer();
void MC_Init_Flap_Motor();
void MC_Saturate(MotorPWM* mst);

void _ISR_NO_PSV FLAP_INTERRUPT()
{
    FLAP_IF = 0;
//    IOCON1bits.SWAP ^= 1;
//    IOCON2bits.SWAP ^= 1;
//    IOCON3bits.SWAP ^= 1;
//    IOCON4bits.SWAP ^= 1;
//    _BTG(_LAT(LED4_PORT), LED4_BIT);
}

void MC_Init()
{
    #if (MOTOR_EN == 1)
        int err = 0;

        _TRISbit(SERV_PG1_PORT, SERV_PG1_BIT) = 1;
        _TRISbit(SERV_PG2_PORT, SERV_PG2_BIT) = 1;

        do {
            if (_PORTbit(SERV_PG1_PORT, SERV_PG1_BIT) == 1 ||
                _PORTbit(SERV_PG2_PORT, SERV_PG2_BIT) == 1) {
                break;
            } else {
                __delay_ms(20);
            }
            err++;
        } while(err < 50);

    //    if (err >= 50)
    //        Display_Error(ErrServPow);

        MC_Init_PWM();
        MC_Init_Flap_Timer();
#ifdef DRAGONFLY
        MC_Init_Flap_Motor();
        
// Tried to eliminate servo power-up jerk by sending min servo position
// immediately after dsPIC frequency configuration but didn't succeed.
        // Bring servo signals to min wing amplitude in the very beginning.
        PDC4 = FRONTRIGHT_SERVO_MAX;
        SDC1 = FRONTLEFT_SERVO_MAX;
        SDC2 = REARLEFT_SERVO_MAX;
        SDC3 = REARRIGHT_SERVO_MAX;
 
#endif
    #endif
}

void MC_Init_PWM()
{
    _ANSELbit(PWM_JP6_1A_PORT, PWM_JP6_1A_BIT)  = 0;
    _TRISbit(PWM_JP6_1A_PORT, PWM_JP6_1A_BIT)   = 0;
    _ANSELbit(PWM_JP6_1B_PORT, PWM_JP6_1B_BIT)  = 0;
    _TRISbit(PWM_JP6_1B_PORT, PWM_JP6_1B_BIT)   = 0;

    _ANSELbit(PWM_JP7_2A_PORT, PWM_JP7_2A_BIT)  = 0;
    _TRISbit(PWM_JP7_2A_PORT, PWM_JP7_2A_BIT)   = 0;
    _ANSELbit(PWM_JP7_2B_PORT, PWM_JP7_2B_BIT)  = 0;
    _TRISbit(PWM_JP7_2B_PORT, PWM_JP7_2B_BIT)   = 0;

    _ANSELbit(PWM_JP8_1A_PORT, PWM_JP8_1A_BIT)  = 0;
    _TRISbit(PWM_JP8_1A_PORT, PWM_JP8_1A_BIT)   = 0;
    _ANSELbit(PWM_JP8_1B_PORT, PWM_JP8_1B_BIT)  = 0;
    _TRISbit(PWM_JP8_1B_PORT, PWM_JP8_1B_BIT)   = 0;

    _ANSELbit(PWM_JP9_2A_PORT, PWM_JP9_2A_BIT)  = 0;
    _TRISbit(PWM_JP9_2A_PORT, PWM_JP9_2A_BIT)   = 0;
    _ANSELbit(PWM_JP9_2B_PORT, PWM_JP9_2B_BIT)  = 0;
    _TRISbit(PWM_JP9_2B_PORT, PWM_JP9_2B_BIT)   = 0;

    _PTEN   = 0;                // PWM module is disabled
#ifdef QUAD
#ifdef SERVOS
    // 120000000/(64*37500) = 50 Hz
    PTPER   = 37500;            // 50Hz PWM switching frequency @ 120MHz FOSC and 64:1 PWM clock prescaler
    PTCON2bits.PCLKDIV  = 6;    // 64:1 PWM prescaler (max PWM resolution)
#else
    PTPER   =  12000;           //10KHz PWM switching frequency @ 120MHz FOSC and 1:1 PWM clock prescaler
    PTCON2bits.PCLKDIV  = 0;    // 1:1 PWM prescaler (max PWM resolution)
#endif
#endif
#ifdef DRAGONFLY
    PTPER   = 37500;             // 50Hz PWM switching frequency @ 120MHz FOSC and 64:1 PWM clock prescaler
    PTCON2bits.PCLKDIV  = 0b110; // 64:1 PWM prescaler (max PWM resolution)
/*
    // 120000000/(9375 *32) = 400 Hz, cannot use 400 Hz PWMs, since the servos can only allow ~50 Hz PWM frequency.
    PTPER   = 9375;              // 400Hz PWM switching frequency @ 120MHz FOSC and 32:1 PWM clock prescaler
    PTCON2bits.PCLKDIV  = 0b101; // 32:1 PWM prescaler (max PWM resolution)
*/
#endif
    //////////////////////////// PWM1
    PWMCON1bits.ITB     = 0;    // NON-independent time-base (all outputs have same period)
    PWMCON1bits.MDCS    = 0;    // This PWM generator has its own duty cycle (set with PDCx and SDCx)
    PDC1    = 0;          // 0% duty cycle
    SDC1    = 0;
    IOCON1bits.PMOD = 0b11; // PWM IO pin pair is in the True-Independent-Output-Mode
    IOCON1bits.PENH = 1;    // PWM output pin control assigned to pin generator
    IOCON1bits.PENL = 1;    // PWM output pin control assigned to pin generator
    IOCON1bits.POLH = 0;    // High and low switches set to active-high state
    IOCON1bits.POLL = 0;    // High and low switches set to active-high state

    //////////////////////////// PWM2
    PWMCON2bits.ITB     = 0;    // NON-independent time-base (all outputs have same period)
    PWMCON2bits.MDCS    = 0;    // This PWM generator has its own duty cycle (set with PDCx and SDCx)
    PDC2    = 0;          // 0% duty cycle
    SDC2    = 0;
    IOCON2bits.PMOD = 0b11; // PWM IO pin pair is in the True-Independent-Output-Mode
    IOCON2bits.PENH = 1;    // PWM output pin control assigned to pin generator
    IOCON2bits.PENL = 1;    // PWM output pin control assigned to pin generator
    IOCON2bits.POLH = 0;    // High and low switches set to active-high state
    IOCON2bits.POLL = 0;    // High and low switches set to active-high state

    //////////////////////////// PWM3
    PWMCON3bits.ITB     = 0;    // NON-independent time-base (all outputs have same period)
    PWMCON3bits.MDCS    = 0;    // This PWM generator has its own duty cycle (set with PDCx and SDCx)
    PDC3    = 0;          // 0% duty cycle
    SDC3    = 0;
    IOCON3bits.PMOD = 0b11; // PWM IO pin pair is in the True-Independent-Output-Mode
    IOCON3bits.PENH = 1;    // PWM output pin control assigned to pin generator
    IOCON3bits.PENL = 1;    // PWM output pin control assigned to pin generator
    IOCON3bits.POLH = 0;    // High and low switches set to active-high state
    IOCON3bits.POLL = 0;    // High and low switches set to active-high state

    //////////////////////////// PWM4
    PWMCON4bits.ITB     = 0;    // NON-independent time-base (all outputs have same period)
    PWMCON4bits.MDCS    = 0;    // This PWM generator has its own duty cycle (set with PDCx and SDCx)
    PDC4    = 0;          // 0% duty cycle
    SDC4    = 0;
    IOCON4bits.PMOD = 0b11; // PWM IO pin pair is in the True-Independent-Output-Mode
    IOCON4bits.PENH = 1;    // PWM output pin control assigned to pin generator
    IOCON4bits.PENL = 1;    // PWM output pin control assigned to pin generator
    IOCON4bits.POLH = 0;    // High and low switches set to active-high state
    IOCON4bits.POLL = 0;    // High and low switches set to active-high state

    _PTEN           = 1;    // PWM module is enabled
}

void MC_Init_Flap_Motor(){
    _RP97R = 0b010000;   //set output of pin to OC1

   OC1CON1 = 0;                                 // It is a good practice to initially clear the control bits
   OC1CON2 = 0;
   OC1CON1bits.OCTSEL = 0x02;                   // The peripheral clock is selected as the clock input to the OC module
   OC1CON1bits.TRIGMODE = 1;                    //automatically reset trigger?
   OC1R = 935;                                 // This is just a typical number, user must calculate based on
                                                //      the waveform requirements and the system clock
                                                //1400=1.5ms pulse
                                                // 935=1ms
                                                //1875=2ms
   OC1RS = 18750;                                // Determines the Period  18750=20ms period
   OC1CON1bits.OCM = 6;                         // This selects the Edge Aligned PWM mode
   OC1CON2bits.SYNCSEL = 0x1F;                  // No trigger or sync source is selected

}

/*
 * Initializes a 60Hz timer that swaps PWM outputs to enable the wing flapping
 * WARNING: The input capture modules (used in RCComm) are set to use the same input clock as this timer
 * so keep that in mind when changing prescaler values or if disabling the timer.
 */
void MC_Init_Flap_Timer()
{
    FLAP_IF             = 0;            // Clear Timer4 iterrupt flag
    FLAP_IE             = 0;            // Disable Timer4 interrupt
    FLAP_TCONR          = 0;            // Resets timer
    FLAP_TCON.TCS       = 0;            // Use internal clock
    FLAP_TCON.TCKPS     = 0b10;         // 1:64 clock precaler
    FLAP_PR             = FLAP_PERIOD;  // 64/60MHz * 15625 = 16.667ms clock period (60Hz)
    FLAP_TCON.TON       = 1;            // Enable Timer4 and start the counter
    FLAP_IE             = 1;            // Enable Timer4 interrupt
}


MotorPWM* MC_Get_Motor_PWM()
{
    return &MCCurrentData;
}

#ifdef QUAD
void MC_Set_Motor_PWM(MotorPWM* mst)
{
    if (MCMotorsKilled)
        return;
    
    MC_Saturate(mst);
    MCCurrentData = *mst;
#ifdef SERVOS
    SDC1 = mst->MC_FrontLeft ;
    SDC2 = mst->MC_RearLeft ;
    SDC3 = mst->MC_RearRight ;
    PDC4 = mst->MC_FrontRight ;

    PDC1 = 0 ;
    PDC2 = 0 ;
    PDC3 = 0 ;
    SDC4 = 0 ;
#else
    PDC1 = mst->MC_FrontLeft ;
    PDC2 = mst->MC_RearLeft ;
    PDC3 = mst->MC_RearRight ;
    PDC4 = mst->MC_FrontRight ;
#endif
}
#endif
#ifdef DRAGONFLY
void MC_Set_Motor_PWM(MotorPWM* mst)
{
    if (MCMotorsKilled)
        return;
/*
    //MC_Saturate(mst);
    MCCurrentData = *mst;
    if(mst->MC_FrontLeft>0){ // SDC1
        SDC1 = mst->MC_FrontLeft ;
        PDC1 = 0;
    }
    else{
        PDC1 = -1*mst->MC_FrontLeft;
        SDC1 = 0;
    }
    if(mst->MC_RearLeft>0){ // SDC2
        SDC2 = mst->MC_RearLeft ;
        PDC2 = 0;
    }
    else{
        PDC2 = -1*mst->MC_RearLeft;
        SDC2 = 0;
    }
    if(mst->MC_RearRight>0){ // SDC3
        SDC3 = mst->MC_RearRight ;
        PDC3 = 0;
    }
    else{
        PDC3 = -1*mst->MC_RearRight;
        SDC3 = 0;
    }
    if(mst->MC_FrontRight>0){ // PDC4
        PDC4 = mst->MC_FrontRight ;
        SDC4 = 0;
    }
    else{
        SDC4 = -1*mst->MC_FrontRight;
        PDC4 = 0;
    }
*/
/*    PDC1 = 1875; // test code for generating 8 individual PWMs - yangbo
    SDC1 = 2062.5;
    PDC2 = 2250;
    SDC2 = 2437.5;
    PDC3 = 2625;
    SDC3 = 2812.5;
    PDC4 = 3000;
    SDC4 = 3187.5;*/

    MCCurrentData = *mst;
    SDC1 = mst->MC_FrontLeft;
    PDC1 = 0;
    SDC2 = mst->MC_RearLeft;
    PDC2 = 0;
    SDC3 = mst->MC_RearRight;
    PDC3 = 0;
    PDC4 = mst->MC_FrontRight;
    SDC4 = 0;

}

void MC_Set_Servo_Prop_PWM(MotorPWM* servo, PropPWM* prop)
{
    if (MCMotorsKilled)
    return;

    MCCurrentData = *servo;
    SDC1 = servo->MC_FrontLeft;
    SDC2 = servo->MC_RearLeft;
    SDC3 = servo->MC_RearRight;
    PDC4 = servo->MC_FrontRight;

    PCCurrentData = *prop;
    PDC2 = prop->MC_P1;
    PDC1 = prop->MC_P2;
    SDC4 = prop->MC_P3;

}
#endif

void MC_Saturate(MotorPWM* mst)
{
    if (mst->MC_FrontLeft < 0)
        mst->MC_FrontLeft = 0;
    else if (mst->MC_FrontLeft > MC_MAX_PWM)
        mst->MC_FrontLeft = MC_MAX_PWM;

    if (mst->MC_FrontRight < 0)
        mst->MC_FrontRight = 0;
    else if (mst->MC_FrontRight > MC_MAX_PWM)
        mst->MC_FrontRight = MC_MAX_PWM;

    if (mst->MC_RearLeft < 0)
        mst->MC_RearLeft = 0;
    else if (mst->MC_RearLeft > MC_MAX_PWM)
        mst->MC_RearLeft = MC_MAX_PWM;

    if (mst->MC_RearRight < 0)
        mst->MC_RearRight = 0;
    else if (mst->MC_RearRight > MC_MAX_PWM)
        mst->MC_RearRight = MC_MAX_PWM;
}

void MC_Kill_Motors()
{
    MCMotorsKilled = 1;
    PDC1 = SERVO_MIN;
    PDC2 = SERVO_MIN;
    PDC3 = 0;
    PDC4 = FRONTRIGHT_SERVO_MAX;

    SDC1 = FRONTLEFT_SERVO_MAX;
    SDC2 = REARLEFT_SERVO_MAX;
    SDC3 = REARRIGHT_SERVO_MAX;
    SDC4 = SERVO_MID;

    OC1R = 935;
}

void MC_Restart_Motors()
{
    MCMotorsKilled = 0;
}