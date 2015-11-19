/* 
 * File:   DFCommands.h
 * Author: Felipe
 *
 * Created on April 10, 2013, 6:34 PM
 */

#ifndef DFCOMMANDS_H
#define	DFCOMMANDS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DFIdentifyCmdID         0x00
#define DFMoveCmdID             0x01
#define DFSetGainsCmdID         0x02
#define DFTrimCmdID             0x03
#define DFSettingCmdID          0x04
#define DFSaturationCmdID       0x20
#define DFServoTrimCmdID        0x21
#define DFServoMinCmdID         0x22
#define DFServoMaxCmdID         0x23
#define DFToggleFilterCmdID     0x80
#define DFGetOneFrameCmdID      0x90
#define DFToggleCameraEnable    0x91
#define DFSetTogglesCmdID       0x50


    typedef struct __attribute__((packed)) {
        uint8_t CID;
        uint8_t Seq;
    } DFCommandHeader;

    typedef struct {
        union {
            DFCommandHeader head;
            uint8_t     bytes[32];
            uint16_t    words[32];
        };
    } DFCommand;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
                uint8_t ThrottleRef;
            };
        };
    } DFMoveCmd;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
                int8_t  RollTrim;
                int8_t  PitchTrim;
                int8_t  YawTrim;
            };
        };
    } DFTrimCmd;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
                int8_t PipeID;
                int8_t WingSeq;
                int8_t RCCal;
            };
        };
    } DFSettingCmd;
    
    typedef enum {
        ControlTypeRoll  = 1,
        ControlTypePitch = 2,
        ControlTypeYaw   = 4
    } ControlType;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
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
    } DFSetGainsCmd;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
                uint8_t main_motor;
                uint8_t servo_norm;
                uint8_t servo_satu;
                uint8_t pitch_satu;
                uint8_t yaw_satu;
            };
        };
    } DFSaturationCmd;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
                int8_t fl;
                int8_t fr;
                int8_t rl;
                int8_t rr;
            };
        };
    } DFServoTrimCmd; // individual servo trim value

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
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
    } DFServoMinCmd;

    typedef struct {
        union {
            DFCommand raw;
            struct __attribute__((packed)) {
                DFCommandHeader head;
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
    } DFServoMaxCmd;

#ifdef	__cplusplus
}
#endif

#endif	/* DFCOMMANDS_H */

