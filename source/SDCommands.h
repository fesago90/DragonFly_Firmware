/* 
 * File:   SDCommands.h
 * Author: Parav Nagarsheth
 * Derived from DFCommands.h
 * Modified for use for Selfie Drone
 * Created on May 11, 2015, 2:25 PM
 */

#ifndef SDCOMMANDS_H
#define	SDCOMMANDS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SDIdentifyCmdID         0x00
#define SDControlCmdID          0x01
#define SDSetGainsCmdID         0x02
#define SDTrimCmdID             0x03
#define SDSettingCmdID          0x04
#define SDSaturationCmdID       0x20
#define SDServoTrimCmdID        0x21
#define SDServoMinCmdID         0x22
#define SDServoMaxCmdID         0x23
#define SDToggleFilterCmdID     0x80
#define SDGetOneFrameCmdID      0x90
#define SDToggleCameraEnable    0x91
#define SDSetTogglesCmdID       0x50

#define ROLL_CONTROL            0x01
#define PITCH_CONTROL           0x02
#define YAW_CONTROL             0x04
#define THROTTLE_CONTROL        0x08
#define TAKE_OFF_CONTROL        0x10
#define LAND_CONTROL            0x20


    typedef struct __attribute__((packed)) {
        uint8_t length;
        uint8_t CID;
    } SDCommandHeader;

    typedef struct {
        union {
            SDCommandHeader head;
            uint8_t     bytes[32];
            uint16_t    words[32];
        };
    } SDCommand;

/*
 * SDControl is the reference structure for setting
 * the desired throttle (0 to 255), yaw (-127 to 127), pitch (-127 to 127) and
 * roll(-127 to 127).
 */
    
    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                uint8_t control_type;
                uint8_t roll_set;
                int8_t pitch_set;
                int8_t yaw_set;
                uint8_t throttle_set;
                uint8_t take_off_set;
                uint8_t land_set;
            };
        };
    } SDControlCmd;

/*
 * SDTrimCmd is the reference structure for setting
 * the trim for yaw (-128 to 127), pitch (-128 to 127) and
 * roll(-128 to 127) rates.
 */
    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                int8_t  RollTrim;
                int8_t  PitchTrim;
                int8_t  YawTrim;
            };
        };
    } SDTrimCmd;

    typedef enum {
        ControlTypeRoll  = 1,
        ControlTypePitch = 2,
        ControlTypeYaw   = 4
    } ControlType;

    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                uint8_t     ControlMask;
                uint8_t     __reserved;
                uint16_t    RollKp;
                uint16_t    RollKi;
                uint16_t    RollKd;
                uint16_t    PitchKp;
                uint16_t    PitchKi;
                uint16_t    PitchKd;
                uint16_t    YawKp;
                uint16_t    YawKi;
                uint16_t    YawKd;
            };
        };
    } SDSetGainsCmd;

    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                uint8_t main_motor;
                uint8_t servo_norm;
                uint8_t servo_satu;
                uint8_t pitch_satu;
                uint8_t yaw_satu;
            };
        };
    } SDSaturationCmd;

    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                int8_t fl;
                int8_t fr;
                int8_t rl;
                int8_t rr;
            };
        };
    } SDServoTrimCmd; // individual servo trim value

    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                uint8_t fl_low;
                uint8_t fl_high;
                uint8_t fr_low;
                uint8_t fr_high;
                uint8_t rl_low;
                uint8_t rl_high;
                uint8_t rr_low;
                uint8_t rr_high;
            };
        };
    } SDServoMinCmd;

    typedef struct {
        union {
            SDCommand raw;
            struct __attribute__((packed)) {
                SDCommandHeader head;
                uint8_t fl_low;
                uint8_t fl_high;
                uint8_t fr_low;
                uint8_t fr_high;
                uint8_t rl_low;
                uint8_t rl_high;
                uint8_t rr_low;
                uint8_t rr_high;
            };
        };
    } SDServoMaxCmd;

#ifdef	__cplusplus
}
#endif

#endif	/* SDCOMMANDS_H */

