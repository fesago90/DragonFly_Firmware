#ifndef FLIGHTCONTROL_H
#define	FLIGHTCONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Gyro.h"
#include "Accel.h"
#include "Settings.h"
#include <dsp.h>

#define FC_MAX_ACC_REF  MAX_ANGLE_REFERENCE   // Maximum angle command for accelerometer
#define FC_MAX_GYRO_REF MAX_ANGULAR_VELOCITY_REFERENCE  // Maximum Z-AXIS absolute angle rate change

#define FC_MAX_GYRO_REF_X (0x7FFF/50)

#define FC_TIMER_CNT (int)((((long)FC_LOOP_TIME_IN_US)*((long)SYSTEM_FREQUENCY_IN_MHZ))/(long)64)

#ifdef QUAD
#define SATURATE(out, limit) (out > 0) ? MIN(limit, out) : 0//  MAX(-limit, out)
#endif
#ifdef DRAGONFLY
#define SATURATE(out, limit) (out > 0) ? MIN(limit, out) : MAX(-limit, out)
#endif

#define CONSTRAIN(out, limitL, limitH) (out>limitH)? limitH : ( out<limitL?limitL:out)


#define PWM_MULT    50 // for scaling 255 to 12000

#define dt	1/1000

extern uint8_t FCFilterIsEnabled;

typedef enum {
    FCStatusReady                 = 1,
    FCStatusCalibrating           = 2
} FCState;

typedef struct {
    float PIDCoefficients[3];    /* Pointer to A, B & C coefficients located in X-space */
                                        /* These coefficients are derived from */
                                        /* the PID gain values - Kp, Ki and Kd */
    int16_t controlOutput;       /* PID Controller Output  */
    float measuredOutput;      /* Measured Output sample */
    int16_t controlReference;    /* Reference Input sample */
} TJ_PID;

    void        FC_Init();
    void        FC_Start_Calibrating();
    void        FC_Try_Process_Controls();
    FCState     FC_Get_Status();

    void        FC_Toggle_Filter();
    GyroData*   FC_Get_Current_Filtered_Gyro();
    AccData*    FC_Get_Current_Filtered_Accel();

    void        FC_Set_Throttle_Reference(int16_t throttle); // from either sdk or rc
    void        FC_Set_Roll_Trim(int16_t trim); // from sdk
    void        FC_Set_Pitch_Trim(int16_t trim);// from sdk
    void        FC_Set_Yaw_Trim(int16_t trim);  // from sdk
    void        FC_Set_Pitch_Reference(int16_t pitch);
    void        FC_Set_Yaw_Reference(int16_t yaw);
    void        FC_Set_Roll_Reference(int16_t roll);

    void FC_Motors_Check();
    void FC_Dragonfly_Test();
#ifdef SERVOS
    void FC_Servo_Check();
#endif
    void FC_Update_PID_Reference(int ref, TJ_PID* mc);
    //void FC_PID_Set_Gains(tPID* mc, fractional kp, fractional ki, fractional kd);
    void FC_PID_Set_Pitch_Gains(TJ_PID* mc,fractional kp, fractional ki, fractional kd); // from sdk
    void FC_PID_Set_Roll_Gains(TJ_PID* mc,fractional kp, fractional ki, fractional kd);  // from sdk
    void FC_PID_Set_Yaw_Gains(TJ_PID* mc,fractional kp, fractional ki, fractional kd);   // from sdk

    void FC_Set_Pitch_Saturation(uint16_t saturation); // from sdk
    void FC_Set_Yaw_Saturation(uint16_t saturation);   // from sdk
    void FC_Set_Servo_Nominal(uint16_t nominal);       // from sdk
    void FC_Set_Servo_Saturation(uint8_t saturation);  // from sdk
    void FC_Set_Main_Motor_Power(uint16_t power);      // from sdk

    void FC_Set_Servo_FLTrim(int16_t trim); // from sdk
    void FC_Set_Servo_FRTrim(int16_t trim); // from sdk
    void FC_Set_Servo_RLTrim(int16_t trim); // from sdk
    void FC_Set_Servo_RRTrim(int16_t trim); // from sdk

    void FC_Set_Servo_FLMin(uint16_t min); // from sdk
    void FC_Set_Servo_FRMin(uint16_t min); // from sdk
    void FC_Set_Servo_RLMin(uint16_t min); // from sdk
    void FC_Set_Servo_RRMin(uint16_t min); // from sdk

    void FC_Set_Servo_FLMax(uint16_t max); // from sdk
    void FC_Set_Servo_FRMax(uint16_t max); // from sdk
    void FC_Set_Servo_RLMax(uint16_t max); // from sdk
    void FC_Set_Servo_RRMax(uint16_t max); // from sdk

#ifdef	__cplusplus
}
#endif

#endif	/* DFFLIGHTCONTROL_H */

