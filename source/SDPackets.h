/* 
 * File:   SDPackets.h
 * Author: Parav Nagarsheth
 * Derived from DFPackets.h
 * Created on May 11, 2015, 3:27 PM
 */

#ifndef SDPACKETS_H
#define	SDPACKETS_H

#ifdef	__cplusplus
extern "C" {
#endif


    #define SDStatePacketID 0x00
    #define SDCameraPacketID 0x90
    #define SDStartFramePacketID 0x91
    #define SDEndFramePacketID 0x92

    typedef struct __attribute__((packed)) {
        uint8_t PID;
        uint8_t Seq;
    } SDPacketHeader;

    typedef struct {
        union {
            SDPacketHeader head;
            uint8_t bytes[32];
            uint16_t words[32];
        };
    } SDPacket;

    typedef struct {
        union {
            SDPacket raw;
            struct __attribute__((packed)) {
                SDPacketHeader head;
                uint16_t Acc_X;
                uint16_t Acc_Y;
                uint16_t Acc_Z;

                uint16_t Gy_X;
                uint16_t Gy_Y;
                uint16_t Gy_Z;

                uint16_t V_Batt;
                uint16_t V_Servo;
                uint16_t V_Supply;

                uint8_t M_FrontLeft;
                uint8_t M_FrontRight;
                uint8_t M_RearLeft;
                uint8_t M_RearRight;
            };
        };
    } SDStatePacket;

    typedef struct {
        union {
            SDPacket raw;
            struct __attribute__((packed)) {
                SDPacketHeader head;
                uint16_t CAM_SegmentID;
                uint8_t CAM_PixelData[28];
            };
        };
    } SDCameraPacket;

    typedef struct {
        union {
            SDPacket raw;
            struct __attribute__((packed)) {
                SDPacketHeader head;
                uint16_t numCols;
                uint16_t numRows;
                uint16_t SegmentLength;
            };
        };
    } SDStartFramePacket;

    typedef struct {
        union {
            SDPacket raw;
            struct __attribute__((packed)) {
                SDPacketHeader head;
            };
        };
    }SDEndFramePacket;



#ifdef	__cplusplus
}
#endif

#endif	/* SDPACKETS_H */

