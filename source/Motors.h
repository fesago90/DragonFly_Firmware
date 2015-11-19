/*
 * File:   Motors.h
 * Author: Felipe
 *
 * Created on March 4, 2013, 12:12 PM
 */

#ifndef MOTORS_H
#define	MOTORS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MC_MAX_PWM 12000
#define MC_MIN_PWM 0

    typedef struct {
        int16_t MC_FrontLeft;
        int16_t MC_FrontRight;
        int16_t MC_RearLeft;
        int16_t MC_RearRight;
    } MotorPWM;

    typedef struct {
        int16_t MC_P1;
        int16_t MC_P2;
        int16_t MC_P3;
    } PropPWM;

    void        MC_Init();
    MotorPWM*   MC_Get_Motor_PWM();
    void        MC_Set_Motor_PWM(MotorPWM* mst);
    void        MC_Set_Servo_Prop_PWM(MotorPWM* servo, PropPWM* prop);
    void        MC_Kill_Motors();
    void        MC_Restart_Motors();

#ifdef	__cplusplus
}
#endif

#endif	/* ALPHAMC_H */

