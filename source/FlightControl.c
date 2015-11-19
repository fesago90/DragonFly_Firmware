#include "Common.h"
#include "FlightControl.h"
#include "Motors.h"
#include "FilterCoeffs.h"
#include <math.h>
#include <dsp.h>

/*
 * Accelerometer values are 10-bit for +-2g range
 * Therefore,
 *  -2g = -512
 *  -1g = -256
 *   0g = 0
 *   1g = 256
 *  +2g ~ 511
 *
 * To put it in fractional format, the output must be
 * multiplied by 64 (shift left 6).
 * 
 */

#define ACC_FILT                100.0
#define COMP_FILT               0.9995
#define DERIV_NUM               25   //just something to turn down the value

#define ACCEL_2_GS          256.0f   //@2G resolution    -- full resolution is ~4 LSB/mg (just more bits)= ~250
//#define ACCEL_2_GS          128.0f   //@2G resolution
//#define ACCEL_2_GS          64.0f   //@2G resolution
//#define ACCEL_2_GS          32.0f   //@2G resolution


//#define GYRO_2_MDPS         8.75/1000.0f    //@250 dps resolution
//#define GYRO_2_MDPS         17.50/1000.0f    //@500 dps resolution
#define GYRO_2_MDPS         70/1000.0f    //@2000 dps resolution


typedef struct {
    float Gy_X;
    float Gy_Y;
    float Gy_Z;
    } GyroIntData;
typedef struct {
    float Acc_X;
    float Acc_Y;
    float Acc_Z;
} AccLowFilt;

uint8_t     FCFilterIsEnabled = 1;

int8_t UserConfig = 0; // for user configuration, actually not necessary

AccData     FCCurrentAcc;
GyroData    FCCurrentGyro;
GyroIntData FCGyroIntegral;
AccLowFilt  FCCFiltAccLow;

// final filtered sensor data
AccData     FCCurrentFiltAcc;
GyroData    FCCurrentFiltGyro;

// averaged accelerations during calibration on ground
AccData     FCGroundAverageAcc;
GyroData    FCGroundAverageGyro;

MotorPWM    FCMotorPWM;
PropPWM     FCPropPWM;

float prev_error_yaw;

// Throttle from RC/RF is between 0 to 255.
uint8_t     FCThrottleReferencePWM = 0;
int8_t      FCTrimRoll  = 0;
int8_t      FCTrimPitch = 0;
int8_t      FCTrimYaw   = 0;

int16_t     FCRollAdd = 0;
int16_t     FCPitchAdd = 0;
int16_t     FCYawAdd = 0;

int8_t deriv_num_pitch=0;
int8_t deriv_num_roll=0;
int8_t deriv_num_yaw=0;

volatile FCState    FCStatus;

void        FC_Init_Filters();
void        FC_Timer_Init();
void        FC_Calibrate_Sensors();
void        FC_Control_Logic_Step();
void        FC_Update_Motors();
//void        FC_GetAngle();
//void        FC_CompAngle();
void FC_PID_Pitch(TJ_PID* pid);
void FC_PID_Roll(TJ_PID* pid);
void FC_PID_Yaw(TJ_PID* pid);

// Saturation, Nominal, etc/
uint16_t PitchSaturation; // range from 0 - 50%
uint16_t YawSaturation;   // range from 0 - 50%
uint16_t ServoNominal;    // range from 0 - 30%
uint8_t ServoSaturation;  // range from 0 - 30%
uint16_t MainMotorPower;  // range from 40 - 80%

int16_t FLTrim, FRTrim, RLTrim, RRTrim;

// FL, RR - inside servo; FR, RL - outside servo.
uint16_t FCServoFLMin = 1700, FCServoFRMin = 3800, FCServoRLMin = 3800, FCServoRRMin = 1700;
uint16_t FCServoFLMax = 3800, FCServoFRMax = 1700, FCServoRLMax = 1700, FCServoRRMax = 3800;

#define FC_SENSOR_FILTER        LPF_5_1_25_50
#define FC_SENSOR_FILTER_SIZE   LPF_5_1_25_50_SIZE

fractional FCAccXFilterDelay[FC_SENSOR_FILTER_SIZE]  __attribute__ ((space(ymemory), eds));
//fractional FCAccXFilterOutput[FCLPFNumCoeffs];

fractional FCAccYFilterDelay[FC_SENSOR_FILTER_SIZE]  __attribute__ ((space(ymemory), eds));
//fractional FCAccYFilterOutput[FCLPFNumCoeffs];

fractional FCAccZFilterDelay[FC_SENSOR_FILTER_SIZE]  __attribute__ ((space(ymemory), eds));
//fractional FCAccZFilterOutput[FCLPFNumCoeffs];

fractional FCGyXFilterDelay[FC_SENSOR_FILTER_SIZE]   __attribute__ ((space(ymemory), eds));
//fractional FCGyXFilterOutput[FCLPFNumCoeffs];

fractional FCGyYFilterDelay[FC_SENSOR_FILTER_SIZE]   __attribute__ ((space(ymemory), eds));
//fractional FCGyYFilterOutput[FCLPFNumCoeffs];

fractional FCGyZFilterDelay[FC_SENSOR_FILTER_SIZE]   __attribute__ ((space(ymemory), eds));
//fractional FCGyZFilterOutput[FCLPFNumCoeffs];

FIRStruct FCAccXLowPassFilter;
FIRStruct FCAccYLowPassFilter;
FIRStruct FCAccZLowPassFilter;

FIRStruct FCGyXLowPassFilter;
FIRStruct FCGyYLowPassFilter;
FIRStruct FCGyZLowPassFilter;

TJ_PID PIDPitch;
TJ_PID PIDRoll;
TJ_PID PIDYaw;

/*
 * This timer sets a flag that allows the next control iteration to begin
 */
void _ISR_NO_PSV LOOP_INTERRUPT()
{
    LOOP_IF = 0;

    FCStatus |= FCStatusReady;
    UserConfig = 1; // for user configuration, actually not necessary
}

void FC_Init()
{
    
    // Roll PID setup
    FC_Set_Roll_Reference(0);
    FC_PID_Set_Roll_Gains(&PIDRoll, Float2Fract(ROLL_Kp), Float2Fract(ROLL_Ki), Float2Fract(ROLL_Kd));

    // Pitch PID setup
    FC_Set_Pitch_Reference(0);
    FC_PID_Set_Pitch_Gains(&PIDPitch, Float2Fract(PITCH_Kp), Float2Fract(PITCH_Ki), Float2Fract(PITCH_Kd));

    // Yaw PID setup
    FC_Set_Yaw_Reference(0);
    FC_PID_Set_Yaw_Gains(&PIDYaw, Float2Fract(YAW_Kp), Float2Fract(YAW_Ki), Float2Fract(YAW_Kd));
    
    FC_Init_Filters();

    // Begin control timer
    FCStatus = 0;
    prev_error_yaw = 0;
    FC_Timer_Init();

    FCGyroIntegral.Gy_X = 0.0;
    FCGyroIntegral.Gy_Y = 0.0;
    FCGyroIntegral.Gy_Z = 0.0;

}

void FC_Try_Process_Controls()
{
    if ((FCStatus & FCStatusReady) == 0)
        return;

    if (FCStatus & FCStatusCalibrating) {
        FC_Calibrate_Sensors();
    } else {
        FC_Control_Logic_Step();
        FC_Update_Motors();
    }

    // Wait for next timer tick
    FCStatus &= ~FCStatusReady;

}

/*
 * Gathers sampled sensor data and performs a PID control loop iteration.
 */
void FC_Flip_Gyro(){
   FCCurrentGyro.Gy_X = ((FCCurrentGyro.Gy_X << 8) & 0xFF00)|((FCCurrentGyro.Gy_X >> 8) & 0x00FF);
   FCCurrentGyro.Gy_Y = ((FCCurrentGyro.Gy_Y << 8) & 0xFF00)|((FCCurrentGyro.Gy_Y >> 8) & 0x00FF);
   FCCurrentGyro.Gy_Z = ((FCCurrentGyro.Gy_Z << 8) & 0xFF00)|((FCCurrentGyro.Gy_Z >> 8) & 0x00FF);
}

void FC_Control_Logic_Step()
{
    asm("push	DSRPAG");
    asm("movpag #0x0001, DSRPAG");

    FCCurrentAcc    = *(Acc_Get_Current_Data());
    FCCurrentGyro   = *(Gyro_Get_Current_Data());

    //FC_Flip_Gyro();

/*   FCCurrentAcc.Acc_X -= FCGroundAverageAcc.Acc_X;
    FCCurrentAcc.Acc_Y -= FCGroundAverageAcc.Acc_Y;
    FCCurrentAcc.Acc_Z -= FCGroundAverageAcc.Acc_Z;
*/
    // Account for sensor offsets
    FCCurrentGyro.Gy_X -= FCGroundAverageGyro.Gy_X;
    FCCurrentGyro.Gy_Y -= FCGroundAverageGyro.Gy_Y;
    FCCurrentGyro.Gy_Z -= FCGroundAverageGyro.Gy_Z;

    // Flip Y-axis to conform to standard coordinates
    //FCCurrentGyro.Gy_Y *= -1;

    // Always Filter ACC
    FIR(1, &(FCCurrentFiltAcc.Acc_X), &(FCCurrentAcc.Acc_X), &FCAccXLowPassFilter);
    FIR(1, &(FCCurrentFiltAcc.Acc_Y), &(FCCurrentAcc.Acc_Y), &FCAccYLowPassFilter);
    FIR(1, &(FCCurrentFiltAcc.Acc_Z), &(FCCurrentAcc.Acc_Z), &FCAccZLowPassFilter);

    if (FCFilterIsEnabled) {
        //extra low-pass filter for accelerometer
        FCCFiltAccLow.Acc_X = (((ACC_FILT-1)*FCCFiltAccLow.Acc_X+FCCurrentFiltAcc.Acc_X)/ACC_FILT);
        FCCFiltAccLow.Acc_Y = (((ACC_FILT-1)*FCCFiltAccLow.Acc_Y+FCCurrentFiltAcc.Acc_Y)/ACC_FILT);
        FCCFiltAccLow.Acc_Z = (((ACC_FILT-1)*FCCFiltAccLow.Acc_Z+FCCurrentFiltAcc.Acc_Z)/ACC_FILT);
        FCCurrentFiltAcc.Acc_X    = (int)FCCFiltAccLow.Acc_X;
        FCCurrentFiltAcc.Acc_Y    = (int)FCCFiltAccLow.Acc_Y;
        FCCurrentFiltAcc.Acc_Z    = (int)FCCFiltAccLow.Acc_Z;

        //regular Gyro low-pass filter
        FIR(1, &(FCCurrentFiltGyro.Gy_X), &(FCCurrentGyro.Gy_X), &FCGyXLowPassFilter);
        FIR(1, &(FCCurrentFiltGyro.Gy_Y), &(FCCurrentGyro.Gy_Y), &FCGyYLowPassFilter);
        FIR(1, &(FCCurrentFiltGyro.Gy_Z), &(FCCurrentGyro.Gy_Z), &FCGyZLowPassFilter);
    } else {
        FCCurrentFiltAcc    = FCCurrentAcc;
        FCCurrentFiltGyro   = FCCurrentGyro;
    }

    asm("pop    DSRPAG");

    PIDPitch.measuredOutput = 1.0*(FCCurrentFiltGyro.Gy_X); 
    PIDRoll.measuredOutput = 1.0*(FCCurrentFiltGyro.Gy_Y);
    PIDYaw.measuredOutput = 1.0*(FCCurrentFiltGyro.Gy_Z); 
    
    FC_PID_Pitch(&PIDPitch);
    FC_PID_Roll(&PIDRoll);
    FC_PID_Yaw(&PIDYaw);
}

void FC_PID_Pitch(TJ_PID* pid){
    float angle_error, rate_error;
    float kp, ki, kd, output = 0, pitch_stab_output = 0;
    float ax, ay, az; // acceleration measured by each of the accelerometer axes
    float pitch_rate, pitch_acc;
    static float pitch_angle = 0, angle_err_integral = 0;
    int16_t desired_angle;

    //kp = (pid->PIDCoefficients[0]);
    kp = (pid->PIDCoefficients[0]) * 10;
    ki = (pid->PIDCoefficients[1]);
    kd = (pid->PIDCoefficients[2]);
    //kp = 2.58;
    //kd = 0.314;
    
    // pitch_acc is the pitch angle computed using acc data.
    ax = FCCurrentFiltAcc.Acc_X/ACCEL_2_GS;
    ay = FCCurrentFiltAcc.Acc_Y/ACCEL_2_GS;
    az = FCCurrentFiltAcc.Acc_Z/ACCEL_2_GS;
    if(ax == 0 && ay == 0 && az == 0) // to avoid the undefined condition of atan2f when both arguments of it are 0s
        pitch_acc = 0;
    else // well defined function atan2f
        pitch_acc = atan2f(ay, sqrtf(ax * ax + az * az)) * 180/PI;            // negative pitch angle when tilting around -Y
    
    // pitch angular rate using gyro data
    pitch_rate = -pid->measuredOutput * 70/1000.0;                            // negative pitch angular rate when tilting around -Y
    // pitch angle by integrating gyro data
    pitch_angle += pitch_rate * dt;                                           // negative pitch angle when tilting around -Y

    // pitch angle estimated using complementary filter
    pitch_angle = COMP_FILT * pitch_angle + (1 - COMP_FILT) * pitch_acc;      // negative pitch angle when tilting around -Y
    // desired angle reference input is between -16 deg to +16 deg (*3)
    desired_angle = pid->controlReference + FCTrimPitch;
    angle_error   = desired_angle - pitch_angle;                              // positive when tilting around -Y

    // compute proportional component
    pitch_stab_output = kp * angle_error;                                     // positive

    // compute integral component for small angle error less than 5 deg
    if(fabs(angle_error) < 5) {
        angle_err_integral += ki * angle_error * dt;                          // positive when tilting around -Y
        // integral anti-windup, unit of 50 is dps
        if (angle_err_integral < -10)
            angle_err_integral = -10;
        else if (angle_err_integral > 10)
            angle_err_integral = 10;

        pitch_stab_output += angle_err_integral;
    }

    // constrain reference pitch rate signal between +200 and -200 degrees/sec
    if(pitch_stab_output < -200)  pitch_stab_output = -200;
    else if(pitch_stab_output > 200)    pitch_stab_output = 200;

    // pitch angular rate error
    rate_error = pitch_stab_output - pitch_rate;                              // positive when reference signal pitch_stab_output = 0
    output = -1 * kd * rate_error;                                            // negative

    //if(output > 120)    output = 120;
    //if(output < -120)   output = -120;
    // 10% pitch saturation
    //if(output > 25)    output = 25;
    //if(output < -25)   output = -25;
    // Pitch saturaion value from SDK
    if(output > (int16_t)PitchSaturation)    output = (int16_t)PitchSaturation;
    if(output < -1 * (int16_t)PitchSaturation)   output = -1 * (int16_t)PitchSaturation;
    
    pid->controlOutput = output;                                              // negative when tilting around -Y
}

void FC_PID_Roll(TJ_PID* pid){
    float angle_error, rate_error;
    float kp, ki, kd, output = 0, roll_stab_output = 0;
    float ax, ay, az; // acceleration measured by each of the accelerometer axes
    float roll_rate, roll_acc;
    static float roll_angle = 0, angle_err_integral = 0;
    int16_t desired_angle;

    kp = (pid->PIDCoefficients[0]) * 10;
    ki = (pid->PIDCoefficients[1]);
    kd = (pid->PIDCoefficients[2]);
    //kp = 3.7;
    //kd = 0.191;

    // roll_acc is the roll angle computed using acc data.
    ax = FCCurrentFiltAcc.Acc_X/ACCEL_2_GS;
    ay = FCCurrentFiltAcc.Acc_Y/ACCEL_2_GS;
    az = FCCurrentFiltAcc.Acc_Z/ACCEL_2_GS;
    if(ax == 0 && ay == 0 && az == 0) // to avoid the undefined condition of atan2f when both arguments of it are 0s
        roll_acc = 0;
    else // well defined function atan2f
        roll_acc = atan2f(ax, sqrtf(ay * ay + az * az)) * 180/PI;             // positive roll angle when tilting around +X
    
    // roll angular rate using gyro data
    roll_rate = pid->measuredOutput*70/1000.0;                                // positive roll anglular rate when tilting around +X
    // roll angle by integrating gyro data
    roll_angle += roll_rate * dt;                                             // positive roll angle when tilting around +X

    // roll angle estimated using complementary filter
    roll_angle = COMP_FILT * roll_angle + (1 - COMP_FILT) * roll_acc;         // positive roll angle when tilting around +X
    // desired angle reference input is between -90 deg to +90 deg
    desired_angle = pid->controlReference + FCTrimRoll;
    angle_error   = desired_angle - roll_angle;                               // negative when tilting around +X

    // compute proportional component
    roll_stab_output = kp * angle_error;                                      // negative

    // compute integral component for small angle error less than 5 deg
    if(fabs(angle_error) < 5) {
        angle_err_integral += ki * angle_error * dt;                          // negative when tilting around +X
        // ingetral anti-windup, unit of 50 is dps
        if (angle_err_integral < -50)
            angle_err_integral = -50;
        else if (angle_err_integral > 50)
            angle_err_integral = 50;

        roll_stab_output += angle_err_integral;
    }

    // constrain reference pitch rate signal between +200 and -200 degrees/sec
    if(roll_stab_output < -300)  roll_stab_output = -300;
    else if(roll_stab_output > 300)    roll_stab_output = 300;

    // roll angular rate error
    rate_error = roll_stab_output - roll_rate;                                // negative when reference signal pitch_stab_output = 0
    output = kd * rate_error;                                                 // negative

    pid->controlOutput = output;                                              // negative
}

void FC_PID_Yaw(TJ_PID* pid){
    float angle_error, rate_error;
    float kp, kd;
    float output = 0, yaw_stab_output = 0;
    float yaw_rate;
    static float yaw_angle = 0;
    int8_t desired_angle;
    // p and d gains
    kp = (pid->PIDCoefficients[0]) * 10;
    kd = (pid->PIDCoefficients[2]);
    //kp = 4;
    //kd = 1;

    // yaw anglar rate using gyro data
    yaw_rate = pid->measuredOutput*70/1000.0;                                       // positive -- when rotating CCW around +Z
    // yaw angle by integrating gyro data
    yaw_angle += yaw_rate * dt;                                                     // positive -- when rotating CCW around +Z

    // desired angle reference input is between -33 deg to +33 deg
    desired_angle = pid->controlReference + FCTrimYaw;
    angle_error   = desired_angle - yaw_angle;                                      // negative when tilting around +Z
    // compute proportional component
    yaw_stab_output = kp * angle_error;                                             // negative

    // constrain reference pitch rate signal between +200 and -200 degrees/sec
    if(yaw_stab_output < -200)  yaw_stab_output = -200;
    else if(yaw_stab_output > 200)    yaw_stab_output = 200;

    // roll angular rate error
    rate_error = yaw_stab_output - yaw_rate;                                        // negative when reference signal yaw_stab_output = 0
    output = kd * rate_error;

    pid->controlOutput = output;
}

#ifdef DRAGONFLY
void FC_Update_Motors()
{
    static int16_t FL, FR, RR, RL;
    static int16_t FP, RP, RS;
    // Dragonfly four wing servos
    FL =  PIDRoll.controlOutput;
    FR = -PIDRoll.controlOutput;
    RL =  PIDRoll.controlOutput;
    RR = -PIDRoll.controlOutput;
/*
    // For scaling from 255 to 1875.
    //FP = (int16_t)FCThrottleReferencePWM + PIDPitch.controlOutput;
    //RP = (int16_t)FCThrottleReferencePWM - PIDPitch.controlOutput;
*/
    // PID outputs need to be scaled to fit the servo PWM range.
    FP =  PIDPitch.controlOutput;
    RP = -PIDPitch.controlOutput;
    // Added tail servo
    RS = -PIDYaw.controlOutput;

    FL = SATURATE(FL*5, 800);
    FR = SATURATE(FR*5, 800);
    RL = SATURATE(RL*5, 800);
    RR = SATURATE(RR*5, 800);
/*
    // For scaling from 255 to 1875
    //FP = SATURATE(FP, 255);
    //RP = SATURATE(RP, 255);
*/
    // For scaling from PID outputs to half of servo PWM range, 1875/2.
    FP = SATURATE(FP, 450);
    RP = SATURATE(RP, 450);
/*
    // YAW ANGLE SATURATION - 10.67%, 200/1875*100%
    RS = SATURATE(RS*5, 200);
*/
    // Yaw angle saturation adjustable from SDK.
    RS = SATURATE(RS*5, (int16_t)YawSaturation);

    // Servo nominal changable from SDK.
    // Servo min and max PWMs changable from SDK.
    FCMotorPWM.MC_FrontLeft  = FCServoFLMin + ServoNominal + FL;
    FCMotorPWM.MC_FrontRight = FCServoFRMin - ServoNominal - FR;
    FCMotorPWM.MC_RearLeft   = FCServoRLMin - ServoNominal - RL;
    FCMotorPWM.MC_RearRight  = FCServoRRMin + ServoNominal + RR;
/*
    // For scaling from 255 to 1875
    //FCPropPWM.MC_P1 = SERVO_MIN + FP * 8;
    //FCPropPWM.MC_P2 = SERVO_MIN + RP * 8;
*/
    // "* 8" to scale from 255 to 1875; "* 2" to scale from 450 to 900.
    FCPropPWM.MC_P1 = SERVO_MIN + (int16_t)FCThrottleReferencePWM * 8 + FP * 2;
    FCPropPWM.MC_P2 = SERVO_MIN + (int16_t)FCThrottleReferencePWM * 8 + RP * 2;
    
    FCPropPWM.MC_P3 = SERVO_MID + RS;
    
    // Constrain servo saturation to be never larger than servo norminal.
    ServoSaturation = SATURATE(ServoSaturation, ServoNominal/21);
    // Constrain servo outputs to be within servo saturation around servo nominal.
    FCMotorPWM.MC_FrontLeft  = CONSTRAIN(FCMotorPWM.MC_FrontLeft, FCServoFLMin+(FCServoFLMax-FCServoFLMin)*((ServoNominal/21)-ServoSaturation)/100.0, FCServoFLMax-(FCServoFLMax-FCServoFLMin)*(1-((ServoNominal/21)+ServoSaturation)/100.0));
    FCMotorPWM.MC_FrontRight = CONSTRAIN(FCMotorPWM.MC_FrontRight, FCServoFRMax+(FCServoFRMin-FCServoFRMax)*(1-((ServoNominal/21)+ServoSaturation)/100.0), FCServoFRMin-(FCServoFRMin-FCServoFRMax)*((ServoNominal/21)-ServoSaturation)/100.0);
    FCMotorPWM.MC_RearLeft   = CONSTRAIN(FCMotorPWM.MC_RearLeft, FCServoRLMax+(FCServoRLMin-FCServoRLMax)*(1-((ServoNominal/21)+ServoSaturation)/100.0), FCServoRLMin-(FCServoRLMin-FCServoRLMax)*((ServoNominal/21)-ServoSaturation)/100.0);
    FCMotorPWM.MC_RearRight  = CONSTRAIN(FCMotorPWM.MC_RearRight, FCServoRRMin+(FCServoRRMax-FCServoRRMin)*((ServoNominal/21)-ServoSaturation)/100.0, FCServoRRMax-(FCServoRRMin-FCServoRRMin)*(1-((ServoNominal/21)+ServoSaturation)/100.0));
    // Trim value for each of the servos.
    FCMotorPWM.MC_FrontLeft  += FLTrim;
    FCMotorPWM.MC_FrontRight += FRTrim;
    FCMotorPWM.MC_RearLeft   += RLTrim;
    FCMotorPWM.MC_RearRight  += RRTrim;
    // Dynamically change the servos' range after adding servo trims.
    FCMotorPWM.MC_FrontLeft  = CONSTRAIN(FCMotorPWM.MC_FrontLeft, FCServoFLMin + FLTrim, FCServoFLMax + FLTrim);  // inside servo
    FCMotorPWM.MC_FrontRight = CONSTRAIN(FCMotorPWM.MC_FrontRight, FCServoFRMax + FRTrim, FCServoFRMin + FRTrim); // outside servo
    FCMotorPWM.MC_RearLeft   = CONSTRAIN(FCMotorPWM.MC_RearLeft, FCServoRLMax + RLTrim, FCServoRLMin + RLTrim);   // inside servo
    FCMotorPWM.MC_RearRight  = CONSTRAIN(FCMotorPWM.MC_RearRight, FCServoRRMin + RRTrim, FCServoRRMax + RRTrim);  // outside servo

    // Front and rear motor satuation/limit
    if(FCPropPWM.MC_P1 > SERVO_MAX) FCPropPWM.MC_P1 = SERVO_MAX;
    if(FCPropPWM.MC_P2 > SERVO_MAX) FCPropPWM.MC_P2 = SERVO_MAX;
    if(FCPropPWM.MC_P1 < SERVO_MIN) FCPropPWM.MC_P1 = SERVO_MIN;
    if(FCPropPWM.MC_P2 < SERVO_MIN) FCPropPWM.MC_P2 = SERVO_MIN;

    // Front and rear props keep static with min throttle.
    if(FCThrottleReferencePWM < 10) {
        FCPropPWM.MC_P1 = SERVO_MIN;
        FCPropPWM.MC_P2 = SERVO_MIN;
    }
    MC_Set_Servo_Prop_PWM(&FCMotorPWM, &FCPropPWM);
}
#endif

void FC_Timer_Init()
{
    LOOP_IF           = 0;            // Clear Timer4 iterrupt flag
    LOOP_IE           = 0;            // Disable Timer4 interrupt
    LOOP_TCONR        = 0;            // Resets timer
    LOOP_TCON.TCS     = 0;            // Use internal clock
    LOOP_TCON.TCKPS   = 0b10;         // 1:64 clock precaler
    LOOP_PR           = FC_TIMER_CNT; // 64/60MHz * 938 = 1000.5 us clock period
    LOOP_TCON.TON     = 1;            // Enable Timer4 and start the counter
    LOOP_IE           = 1;            // Enable Timer4 interrupt
}

void FC_Calibrate_Sensors()
{
    static uint16_t count = 0;        // static variable will only be initialized once
    static int32_t  AccX = 0, AccY = 0, AccZ = 0;
    static int32_t  GyX = 0, GyY = 0, GyZ = 0;

    FCCurrentAcc    = *(Acc_Get_Current_Data());
    FCCurrentGyro   = *(Gyro_Get_Current_Data());

    count++;

    if(count >= 1024)                  // delay a bit of time
    {
        AccX += FCCurrentAcc.Acc_X;
        AccY += FCCurrentAcc.Acc_Y;
        AccZ += FCCurrentAcc.Acc_Z;

        GyX += FCCurrentGyro.Gy_X;
        GyY += FCCurrentGyro.Gy_Y;
        GyZ += FCCurrentGyro.Gy_Z;
    }
    //1024 total data points
    if (count >= 2048) {
        // Calculate averages
        FCGroundAverageAcc.Acc_X = AccX >> 10;
        FCGroundAverageAcc.Acc_Y = AccY >> 10;
        FCGroundAverageAcc.Acc_Z = AccZ >> 10;

        FCGroundAverageGyro.Gy_X = GyX >> 10;
        FCGroundAverageGyro.Gy_Y = GyY >> 10;
        FCGroundAverageGyro.Gy_Z = GyZ >> 10;

        // Stop calibrating
        FCStatus &= ~FCStatusCalibrating;
        count = 0;
        AccX  = AccY = AccZ = 0;
        GyX   = GyY  = GyZ  = 0;
    }
}

FCState FC_Get_Status()
{
    return FCStatus;
}

void FC_Start_Calibrating()
{
    FCStatus = FCStatusCalibrating;
}

void FC_Set_Throttle_Reference(int16_t throttle)
{
    FCThrottleReferencePWM = (throttle);

    #ifdef DRAGONFLY
    // For wing flapping frequency control, after throttle larger than 30, which 
    // was arbitrarily chosen, freq will be at maximum.
/*    if(throttle > 50)
        //OC1R = 1875; // 100% PWM
        //OC1R = 1687; // 80% PWM
        //OC1R = 1500; // 60% PWM
        OC1R = MainMotorPower;
    else {
        #ifndef TEST
            OC1R = 935 + throttle * 5;
        #endif
    }
*/
    #endif

}

void FC_Set_Pitch_Saturation(uint16_t saturation)
{
    // Scale 50 to half of PID output range, 255/2.
    //PitchSaturation = saturation * 127 / 50;
    // Scale 50 to half of PID output range, 450.
    PitchSaturation = saturation * 450 / 50;
}

void FC_Set_Yaw_Saturation(uint16_t saturation)
{
    YawSaturation = saturation * 20;
}

void FC_Set_Servo_Nominal(uint16_t nominal)
{
    // Servo range is 2100, so it's scaled by 21 here.
    ServoNominal = nominal * 21; 
}

void FC_Set_Servo_FLTrim(int16_t trim)
{
    FLTrim = trim * 21;
}

void FC_Set_Servo_FRTrim(int16_t trim)
{
    FRTrim = trim * 21;
}

void FC_Set_Servo_RLTrim(int16_t trim)
{
    RLTrim = trim * 21;
}

void FC_Set_Servo_RRTrim(int16_t trim)
{
    RRTrim = trim * 21;
}

void FC_Set_Servo_Saturation(uint8_t saturation)
{
    ServoSaturation = saturation;
}

void FC_Set_Main_Motor_Power(uint16_t power)
{
    MainMotorPower = 9 * power + 935;
    OC1R = MainMotorPower;
}

void FC_Set_Roll_Trim(int16_t trim)
{
    FCTrimRoll = trim;
}

void FC_Set_Pitch_Trim(int16_t trim)
{
    FCTrimPitch = trim;
}

void FC_Set_Yaw_Trim(int16_t trim)
{
    FCTrimYaw = trim;
}

void FC_Set_Servo_FLMin(uint16_t min)
{
    FCServoFLMin = min;
}

void FC_Set_Servo_FRMin(uint16_t min)
{
    FCServoFRMin = min;
}

void FC_Set_Servo_RLMin(uint16_t min)
{
    FCServoRLMin = min;
}

void FC_Set_Servo_RRMin(uint16_t min)
{
    FCServoRRMin = min;
}

void FC_Set_Servo_FLMax(uint16_t max)
{
    FCServoFLMax = max;
}

void FC_Set_Servo_FRMax(uint16_t max)
{
    FCServoFRMax = max;
}

void FC_Set_Servo_RLMax(uint16_t max)
{
    FCServoRLMax = max;
}

void FC_Set_Servo_RRMax(uint16_t max)
{
    FCServoRRMax = max;
}

void FC_Set_Roll_Reference(int16_t roll)
{
    PIDRoll.controlReference = -roll ;
}

void FC_Set_Pitch_Reference(int16_t pitch)
{
    PIDPitch.controlReference = pitch  ;
}

void FC_Set_Yaw_Reference(int16_t yaw)
{
    PIDYaw.controlReference = -yaw  ;
}

void FC_Init_Filters()
{
    // Accel filter setup
    FIRStructInit(&FCAccXLowPassFilter, FC_SENSOR_FILTER_SIZE, FC_SENSOR_FILTER, 0xFF00, FCAccXFilterDelay);
    FIRDelayInit(&FCAccXLowPassFilter);

    FIRStructInit(&FCAccYLowPassFilter, FC_SENSOR_FILTER_SIZE, FC_SENSOR_FILTER, 0xFF00, FCAccYFilterDelay);
    FIRDelayInit(&FCAccYLowPassFilter);

    FIRStructInit(&FCAccZLowPassFilter, FC_SENSOR_FILTER_SIZE, FC_SENSOR_FILTER, 0xFF00, FCAccZFilterDelay);
    FIRDelayInit(&FCAccZLowPassFilter);

    // Gyroscope filter setup
    FIRStructInit(&FCGyXLowPassFilter, FC_SENSOR_FILTER_SIZE, FC_SENSOR_FILTER, 0xFF00, FCGyXFilterDelay);
    FIRDelayInit(&FCGyXLowPassFilter);

    FIRStructInit(&FCGyYLowPassFilter, FC_SENSOR_FILTER_SIZE, FC_SENSOR_FILTER, 0xFF00, FCGyYFilterDelay);
    FIRDelayInit(&FCGyYLowPassFilter);

    FIRStructInit(&FCGyZLowPassFilter, FC_SENSOR_FILTER_SIZE, FC_SENSOR_FILTER, 0xFF00, FCGyZFilterDelay);
    FIRDelayInit(&FCGyZLowPassFilter);
}

AccData* FC_Get_Current_Filtered_Accel()
{
    return &FCCurrentFiltAcc;
}

GyroData* FC_Get_Current_Filtered_Gyro()
{
    return &FCCurrentFiltGyro;
}

void FC_Toggle_Filter()
{
    FCFilterIsEnabled ^= 1;
}

void FC_PID_Set_Pitch_Gains(TJ_PID* mc,fractional kp, fractional ki, fractional kd){
    mc->PIDCoefficients[0] = Fract2Float(kp); 
    mc->PIDCoefficients[1] = Fract2Float(ki);
    mc->PIDCoefficients[2] = Fract2Float(kd); 
    
}

void FC_PID_Set_Roll_Gains(TJ_PID* mc,fractional kp, fractional ki, fractional kd){
    mc->PIDCoefficients[0] = Fract2Float(kp);
    mc->PIDCoefficients[1] = Fract2Float(ki);
    mc->PIDCoefficients[2] = Fract2Float(kd); 
    
}

void FC_PID_Set_Yaw_Gains(TJ_PID* mc,fractional kp, fractional ki, fractional kd){
    mc->PIDCoefficients[0] = Fract2Float(kp);
    mc->PIDCoefficients[1] = Fract2Float(ki);
    mc->PIDCoefficients[2] = Fract2Float(kd); 
    
}

void FC_Update_PID_Reference(int ref, TJ_PID* mc)
{
    mc->controlReference = ref;
}

#ifdef DRAGONFLY
void FC_Servo_Check(){
/*
    // for calibration
    while(1) {
        // All motors start in minimum amplitude.
        FCMotorPWM.MC_FrontRight = FRONTRIGHT_SERVO_MAX;
        FCMotorPWM.MC_FrontLeft  = FRONTLEFT_SERVO_MAX;
        FCMotorPWM.MC_RearRight  = REARRIGHT_SERVO_MAX;
        FCMotorPWM.MC_RearLeft   = REARLEFT_SERVO_MAX;
        MC_Set_Motor_PWM(&FCMotorPWM);
        __delay_ms(5000);
        FCMotorPWM.MC_FrontRight = FRONTRIGHT_SERVO_MIN;
        FCMotorPWM.MC_FrontLeft  = FRONTLEFT_SERVO_MIN;
        FCMotorPWM.MC_RearRight  = REARRIGHT_SERVO_MIN;
        FCMotorPWM.MC_RearLeft   = REARLEFT_SERVO_MIN;
        MC_Set_Motor_PWM(&FCMotorPWM);
        __delay_ms(5000);
        OC1R = 1035;
    }
*/
    // Minimum servo signals go to four wings, to level all the wings.
    FCMotorPWM.MC_FrontRight = FRONTRIGHT_SERVO_MIN;
    FCMotorPWM.MC_RearRight  = REARRIGHT_SERVO_MIN;
    FCMotorPWM.MC_RearLeft   = REARLEFT_SERVO_MIN;
    FCMotorPWM.MC_FrontLeft  = FRONTLEFT_SERVO_MIN;
    MC_Set_Motor_PWM(&FCMotorPWM);
    // Stay at level position for 5 secs for all the wings.
    LED_Set_And_Wait(LEDOn, LEDOff, LEDOff, LEDOff, 5000);

    // Maximum servo signals go to four wings, and keep them at maximum position.
    FCMotorPWM.MC_FrontRight = FRONTRIGHT_SERVO_MAX;
    FCMotorPWM.MC_RearRight  = REARRIGHT_SERVO_MAX;
    FCMotorPWM.MC_RearLeft   = REARLEFT_SERVO_MAX;
    FCMotorPWM.MC_FrontLeft  = FRONTLEFT_SERVO_MAX;
    MC_Set_Motor_PWM(&FCMotorPWM);
    // Stay at maximum position for 5 secs for all the wings.
    LED_Set_And_Wait(LEDOff, LEDOn, LEDOff, LEDOff, 5000);

    // Start-up wing sequence: move each wing by 20% of range each time.
    // Inside servo: FrontLeft,  RearRight.
    // Outside servo:  FrontRight, RearLeft.
    char i, j;
    for(j = 1; j < 2; j++) {
        FC_Set_Throttle_Reference(0); // 50*j
        for(i = 0; i < 5; i++) {
            FCMotorPWM.MC_FrontRight = FRONTRIGHT_SERVO_MIN - (FRONTRIGHT_SERVO_MIN - FRONTRIGHT_SERVO_MAX) * i * 0.2;

            MC_Set_Motor_PWM(&FCMotorPWM);
            __delay_ms(500);                  

            FCMotorPWM.MC_RearRight = REARRIGHT_SERVO_MIN + (REARRIGHT_SERVO_MAX - REARRIGHT_SERVO_MIN) * i * 0.2;

            MC_Set_Motor_PWM(&FCMotorPWM);
            __delay_ms(500);

            FCMotorPWM.MC_RearLeft = REARLEFT_SERVO_MIN - (REARLEFT_SERVO_MIN - REARLEFT_SERVO_MAX) * i * 0.2;

            MC_Set_Motor_PWM(&FCMotorPWM);
            __delay_ms(500);

            FCMotorPWM.MC_FrontLeft = FRONTLEFT_SERVO_MIN + (FRONTLEFT_SERVO_MAX - FRONTLEFT_SERVO_MIN) * i * 0.2;
            
            MC_Set_Motor_PWM(&FCMotorPWM);
            __delay_ms(1000);
       }

        // All motors start in minimum amplitude.
        FCMotorPWM.MC_FrontLeft  = FRONTLEFT_SERVO_MIN;
        FCMotorPWM.MC_FrontRight = FRONTRIGHT_SERVO_MIN;
        FCMotorPWM.MC_RearLeft   = REARLEFT_SERVO_MIN;
        FCMotorPWM.MC_RearRight  = REARRIGHT_SERVO_MIN;
        MC_Set_Motor_PWM(&FCMotorPWM);
        LED3_W = 0;
        __delay_ms(2000);
        LED3_W = 1;
    }

    FC_Set_Throttle_Reference(0);
}
#endif