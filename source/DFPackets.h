/* 
 * File:   DFPackets.h
 * Author: Felipe
 *
 * Created on April 10, 2013, 6:34 PM
 */

#ifndef DFPACKETS_H
#define	DFPACKETS_H

#ifdef	__cplusplus
extern "C" {
#endif

    #define DFStatePacketID 0x00
    #define DFCameraPacketID 0x90
    #define DFStartFramePacketID 0x91
    #define DFEndFramePacketID 0x92

    typedef struct __attribute__((packed)) {
        uint8_t PID;
        uint8_t Seq;
    } DFPacketHeader;

    typedef struct {
        union {
            DFPacketHeader head;
            uint8_t bytes[32];
            uint16_t words[32];
        };
    } DFPacket;

    typedef struct {
        union {
            DFPacket raw;
            struct __attribute__((packed)) {
                DFPacketHeader head;
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
    } DFStatePacket;

    typedef struct {
        union {
            DFPacket raw;
            struct __attribute__((packed)) {
                DFPacketHeader head;
                uint16_t CAM_SegmentID;
                uint8_t CAM_PixelData[28];
            };
        };
    } DFCameraPacket;

    typedef struct {
        union {
            DFPacket raw;
            struct __attribute__((packed)) {
                DFPacketHeader head;
                uint16_t numCols;
                uint16_t numRows;
                uint16_t SegmentLength;
            };
        };
    } DFStartFramePacket;

    typedef struct {
        union {
            DFPacket raw;
            struct __attribute__((packed)) {
                DFPacketHeader head;
            };
        };
    }DFEndFramePacket;


#ifdef	__cplusplus
}
#endif

#endif	/* DFPACKETS_H */

