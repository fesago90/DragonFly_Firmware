#include "Common.h"
#include "Motors.h"
#include "RCComm.h"
#include "FlightControl.h"

/* Notes:
 * Ail controls roll
 * Elev controls pitch
 * Rud controls yaw
 * Thr is throttle
 */

#define RC_MAX_HIGH_LOW_DIFF    1100
#define RC_MIN_HIGH_LOW_DIFF    400
#define RC_DEFAULT_CH_VAL_LOW   1000
#define RC_DEFAULT_CH_VAL_HIGH  2000

uint16_t            RCThrottleToFractionalScale;
uint16_t            RCPitchToFractionalScale;
uint16_t            RCYawToFractionalScale;
uint16_t            RCRollToFractionalScale;

fractional          RCThrottleToReferenceScale;
fractional          RCYawToReferenceScale;
fractional          RCPitchToReferenceScale;
fractional          RCRollToReferenceScale;

unsigned int RCMaxThrottle = 0,  RCMinThrottle   = 2000;
unsigned int RCMaxPitch    = 0,  RCMinPitch      = 2000;
unsigned int RCMaxRoll     = 0,  RCMinRoll       = 2000;
unsigned int RCMaxYaw      = 0,  RCMinYaw        = 2000;

uint32_t            RCPitchAverageSum = 0;
uint32_t            RCYawAverageSum = 0;
uint32_t            RCRollAverageSum = 0;

unsigned int                 RCRollAverage, RCRollPulseCount = 0;
unsigned int                 RCYawAverage, RCYawPulseCount = 0;
unsigned int                 RCPitchAverage, RCPitchPulseCount = 0;

volatile int16_t    RCThrottle;
volatile int16_t    RCPitch;
volatile int16_t    RCRoll;
volatile int16_t    RCYaw;

int                 RCIsCalibratingBoundaries = 0;
int                 RCIsCalibratingAverages = 0;

void        RC_Init_Capture();

inline void RC_Set_Throttle(uint16_t thr);
inline void RC_Set_Roll(uint16_t roll);
inline void RC_Set_Pitch(uint16_t pit);
inline void RC_Set_Yaw(uint16_t yaw);

inline int16_t RC_Saturate(
        int16_t value,
        int16_t average,
        fractional high,
        fractional low,
        uint16_t toFractScale,
        fractional toReferenceScale
);

void RC_Init()
{
    #if (RC_EN == 1)
        RC_Init_Capture();
    #endif
}

/*
 * The following four interrupts are triggered on rising and falling edges
 * of the input RC channel. The IC counter is reset at rising edges and then its
 * value is captured at the falling edge.
 */
void _ISR_NO_PSV YAW_IC_INTERRUPT()
{
    YAW_IC_IF = 0;

    if (YAW_IC_CON1.ICM == 0b011) {
        // In this scope, ISR was triggered due to rising edge
        YAW_IC_CON1.ICM = 0b000;        // Reset
        YAW_IC_CON1.ICM = 0b010;        // Configure to capture on falling edge
    } else {
        // In this scope, ISR was triggered due to falling edge
        RC_Set_Yaw(YAW_ICBUF);
        YAW_IC_CON1.ICM = 0b011;        // Configure to capture on rising edge
    }
}

void _ISR_NO_PSV PITCH_IC_INTERRUPT()
{
    PITCH_IC_IF = 0;
    if (PITCH_IC_CON1.ICM == 0b011) {
        // In this scope, ISR was triggered due to rising edge
        PITCH_IC_CON1.ICM = 0b000;        // Reset
        PITCH_IC_CON1.ICM = 0b010;        // Configure to capture on falling edge
    } else {
        // In this scope, ISR was triggered due to falling edge
        RC_Set_Pitch(PITCH_ICBUF);
        PITCH_IC_CON1.ICM = 0b011;        // Configure to capture on rising edge
    }
}

void _ISR_NO_PSV ROLL_IC_INTERRUPT()
{
    ROLL_IC_IF = 0;
    if (ROLL_IC_CON1.ICM == 0b011) {
        // In this scope, ISR was triggered due to rising edge
        ROLL_IC_CON1.ICM = 0b000;        // Reset
        ROLL_IC_CON1.ICM = 0b010;        // Configure to capture on falling edge
    } else {
        // In this scope, ISR was triggered due to falling edge
        RC_Set_Roll(ROLL_ICBUF);
        ROLL_IC_CON1.ICM = 0b011;        // Configure to capture on rising edge
    }
}

void _ISR_NO_PSV THROTTLE_IC_INTERRUPT()
{
    LED3_W ^= 1;
    THROTTLE_IC_IF = 0;
    if (THROTTLE_IC_CON1.ICM == 0b011) {
        // In this scope, ISR was triggered due to rising edge
        THROTTLE_IC_CON1.ICM = 0b000;        // Reset
        THROTTLE_IC_CON1.ICM = 0b010;        // Configure to capture on falling edge
    } else {
        // In this scope, ISR was triggered due to falling edge
        RC_Set_Throttle(THROTTLE_ICBUF);
        THROTTLE_IC_CON1.ICM = 0b011;        // Configure to capture on rising edge
    }
}

/*
 * Initializes the timers to capture RC servo inputs. The IC modules are driven
 * from Timer4, which is initialized in Motors.c
 */
void RC_Init_Capture()
{
    _TRISbit(RC_IN1_PORT, RC_IN1_BIT) = 1;
    _TRISbit(RC_IN2_PORT, RC_IN2_BIT) = 1;
    _TRISbit(RC_IN3_PORT, RC_IN3_BIT) = 1;
    _TRISbit(RC_IN4_PORT, RC_IN4_BIT) = 1;

    _ANSELbit(RC_IN1_PORT, RC_IN1_BIT) = 0;
    _ANSELbit(RC_IN2_PORT, RC_IN2_BIT) = 0;
    _ANSELbit(RC_IN3_PORT, RC_IN3_BIT) = 0;
    _ANSELbit(RC_IN4_PORT, RC_IN4_BIT) = 0;

    YAW_IC_IE           = 0;    // Disable IC1 interrupt
    YAW_IC_IF           = 0;    // Clear IC1 flag
    YAW_IC_CON1.ICTSEL  = 2;    // Use clock source of Timer4 (60MHz/64 = .9375 MHz)
    YAW_IC_CON1.ICM     = 2;    // Capture mode, every falling edge
    YAW_IC_CON2.ICTRIG  = 0;    // The IC counter operates in synchronous mode (rolls over after 0xffff)
    YAW_IC_CON2.SYNCSEL = 0;    // No sync or trigger source
    YAW_IC_MODULE       = YAW_IC_RP;   // Map RPI44 to input compare input
    YAW_IC_IE           = 1;    // Enable IC1 interrupt

    PITCH_IC_IE           = 0;    // Disable IC2 interrupt
    PITCH_IC_IF           = 0;    // Clear IC2 flag
    PITCH_IC_CON2.ICTRIG  = 0;    // The IC counter operates in synchronous mode (rolls over after 0xffff)
    PITCH_IC_CON2.SYNCSEL = 0;    // No sync or trigger source
    PITCH_IC_CON1.ICTSEL  = 2;    // Use clock source of Timer4 (60MHz/64 = .9375 MHz)
    PITCH_IC_CON1.ICM     = 2;    // Capture mode, every falling edge
    PITCH_IC_MODULE       = PITCH_IC_RP;   // Map RPI45 to input compare input
    PITCH_IC_IE           = 1;    // Enable IC2 interrupt

    ROLL_IC_IE           = 0;    // Disable IC3 interrupt
    ROLL_IC_IF           = 0;    // Clear IC3 flag
    ROLL_IC_CON2.ICTRIG  = 0;    // The IC counter operates in synchronous mode (rolls over after 0xffff)
    ROLL_IC_CON2.SYNCSEL = 0;    // No sync or trigger source
    ROLL_IC_CON1.ICTSEL  = 2;    // Use clock source of Timer4 (60MHz/64 = .9375 MHz)
    ROLL_IC_CON1.ICM     = 2;    // Capture mode, every falling edge
    ROLL_IC_MODULE       = ROLL_IC_RP;   // Map RPI46 to input compare input
    ROLL_IC_IE           = 1;    // Enable IC3 interrupt

    THROTTLE_IC_IE           = 0;    // Disable IC4 interrupt
    THROTTLE_IC_IF           = 0;    // Clear IC4 flag
    THROTTLE_IC_CON2.ICTRIG  = 0;    // The IC counter operates in synchronous mode (rolls over after 0xffff)
    THROTTLE_IC_CON2.SYNCSEL = 0;    // No sync or trigger source
    THROTTLE_IC_CON1.ICTSEL  = 2;    // Use clock source of Timer4 (60MHz/64 = .9375 MHz)
    THROTTLE_IC_CON1.ICM     = 2;    // Capture mode, every falling edge
    THROTTLE_IC_MODULE       = THROTTLE_IC_RP;   // Map RPI47 to input compare input
    THROTTLE_IC_IE           = 1;    // Enable IC4 interrupt
}

void RC_Start_Calibrating_Boundaries()
{
    RCIsCalibratingBoundaries = 1; 
}

/*
 * Return 0 if calibration was OK, 1 if it isnt. 
 */
int RC_Stop_Calibrating_Boundaries()
{
//    if (RCMaxThrottle - RCMinThrottle < RC_MIN_HIGH_LOW_DIFF || RCMaxThrottle - RCMinThrottle > RC_MAX_HIGH_LOW_DIFF ||
//            RCMaxPitch - RCMinPitch < RC_MIN_HIGH_LOW_DIFF || RCMaxPitch - RCMinPitch > RC_MAX_HIGH_LOW_DIFF ||
//            RCMaxYaw - RCMinYaw < RC_MIN_HIGH_LOW_DIFF || RCMaxYaw - RCMinYaw > RC_MAX_HIGH_LOW_DIFF ||
//            RCMaxRoll - RCMinRoll < RC_MIN_HIGH_LOW_DIFF || RCMaxRoll - RCMinRoll > RC_MAX_HIGH_LOW_DIFF)
//#ifndef TEST
    if (RCMaxThrottle - RCMinThrottle < RC_MIN_HIGH_LOW_DIFF || RCMaxThrottle - RCMinThrottle > RC_MAX_HIGH_LOW_DIFF ||
            RCMaxPitch - RCMinPitch < RC_MIN_HIGH_LOW_DIFF || RCMaxPitch - RCMinPitch > RC_MAX_HIGH_LOW_DIFF ||
            RCMaxYaw - RCMinYaw < RC_MIN_HIGH_LOW_DIFF || RCMaxYaw - RCMinYaw > RC_MAX_HIGH_LOW_DIFF ||
            RCMaxRoll - RCMinRoll < RC_MIN_HIGH_LOW_DIFF || RCMaxRoll - RCMinRoll > RC_MAX_HIGH_LOW_DIFF) {
        return 0;
    }
//#else
//    if (RCMaxThrottle - RCMinThrottle < RC_MIN_HIGH_LOW_DIFF || RCMaxThrottle - RCMinThrottle > RC_MAX_HIGH_LOW_DIFF ) {
//        return 0;
//    }
//#endif

    RCIsCalibratingBoundaries = 0;
    
    return 1;
}

void RC_Start_Calibrating_Averages()
{
    RCIsCalibratingAverages = 1;
}

void RC_Stop_Calibrating_Averages()
{
    RCIsCalibratingAverages = 0;
    
    RCRollAverage   = RCRollAverageSum / RCRollPulseCount;
    RCYawAverage    = RCYawAverageSum / RCYawPulseCount;
    RCPitchAverage  = RCPitchAverageSum / RCPitchPulseCount;

    RCThrottleToFractionalScale    = 32768/(RCMaxThrottle - RCMinThrottle);
    RCRollToFractionalScale        = 32768/MAX(RCMaxRoll - RCRollAverage, RCRollAverage - RCMinRoll);
    RCPitchToFractionalScale       = 32768/MAX(RCMaxPitch - RCPitchAverage, RCPitchAverage - RCMinPitch);
    RCYawToFractionalScale         = 32768/MAX(RCMaxYaw - RCYawAverage, RCYawAverage - RCMinYaw);

    RCThrottleToReferenceScale    = Float2Fract(255/32768.0f);
    RCRollToReferenceScale        = Float2Fract(FC_MAX_ACC_REF/32768.0f);
    RCPitchToReferenceScale       = Float2Fract(FC_MAX_ACC_REF/32768.0f); 
    RCYawToReferenceScale         = Float2Fract(FC_MAX_GYRO_REF/32768.0f);

}

inline void RC_Set_Throttle(uint16_t thr) {
    if (thr != RCThrottle) {
        if (RCIsCalibratingBoundaries) {
            RCMaxThrottle = MAX(thr, RCMaxThrottle);
            RCMinThrottle = MIN(thr, RCMinThrottle);
        } else {
            RCThrottle = RC_Saturate(thr, RCMinThrottle, RCMaxThrottle, RCMinThrottle, RCThrottleToFractionalScale, RCThrottleToReferenceScale);
            FC_Set_Throttle_Reference((uint16_t)(RCThrottle/1000.0*255.0));   //limit to 8-bit uint16_t
        }
    }
}

inline void RC_Set_Roll(uint16_t roll) {
    if (roll != RCRoll) {
        if (RCIsCalibratingBoundaries) {
            RCMaxRoll = MAX(roll, RCMaxRoll);
            RCMinRoll = MIN(roll, RCMinRoll);
        } else if (RCIsCalibratingAverages) {
            RCRollAverageSum += roll;
            RCRollPulseCount++;
        } else {
            RCRoll = RC_Saturate(roll, RCRollAverage, RCMaxRoll, RCMinRoll, RCRollToFractionalScale, RCRollToReferenceScale);
            RCRoll = (int16_t) (RCRoll * 60/400.);    // ~±35 degree max/min
            FC_Set_Roll_Reference(RCRoll);
        }
    }
}

inline void RC_Set_Pitch(uint16_t pit){
    if (pit != RCPitch) {
        if (RCIsCalibratingBoundaries) {
            RCMaxPitch = MAX(pit, RCMaxPitch);
            RCMinPitch = MIN(pit, RCMinPitch);
        } else if (RCIsCalibratingAverages) {
            RCPitchAverageSum += pit;
            RCPitchPulseCount++;
        } else {
            RCPitch = RC_Saturate(pit, RCPitchAverage, RCMaxPitch, RCMinPitch, RCPitchToFractionalScale, RCPitchToReferenceScale);
            RCPitch = (int16_t) (RCPitch * 40/400.);    // ~±35 degree max/min
            FC_Set_Pitch_Reference(RCPitch);
        }
    }
}

inline void RC_Set_Yaw(uint16_t yaw) {
    if (yaw != RCYaw) {
        if (RCIsCalibratingBoundaries) {
            RCMaxYaw = MAX(yaw, RCMaxYaw);
            RCMinYaw = MIN(yaw, RCMinYaw);
        } else if (RCIsCalibratingAverages) {
            RCYawAverageSum += yaw;
            RCYawPulseCount++;
        } else {
            RCYaw = RC_Saturate(yaw, RCYawAverage, RCMaxYaw, RCMinYaw, RCYawToFractionalScale, RCYawToReferenceScale);
            RCYaw =  (int) (RCYaw * 30/400.);    // ~±35 degree max/min
            FC_Set_Yaw_Reference(RCYaw);
        }
    }
}

/*
 * Maps a pulse-width reading into an appropriate offset/reference range.
 * e.g. places a pulse width reading in range [0, 255] for throttle, [-127, 127]
 * for roll/pitch.
 */
inline int16_t RC_Saturate(int16_t value, int16_t average, fractional high, fractional low, uint16_t toFractScale, fractional toReferenceScale)
{

    if (value >= high)  // Cap input at calibrated maximum
        value = high;
    else if (value <= low) // Floor input at calibrated minimum
        value = low;

    value -= average; // Subtract the average; this makes the "average" 

//    value *= toFractScale; // Multiply by a number which makes the potential
                           // range of the input variable full scale for
                           // fractional numbers

//    register int A asm("A");
//    A = __builtin_mpy(value, toReferenceScale, 0, 0, 0, 0, 0 ,0); // Multiply by
//                           // a number which makes fractional numbers full scale
//                           // over the range of actuation values
//    value = __builtin_sac(A,0); // shift by 0?
    
    return value;
}