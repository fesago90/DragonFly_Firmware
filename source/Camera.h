/* 
 * File:   Camera.h
 * Author: Ian
 *
 * Created on June 26, 2013, 12:44 PM
 */

#ifndef CAMERA_H
#define	CAMERA_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include "I2CInterfaces.h"
    #include "Common.h"

    // Camera register definitions
    #define CAMERA_ADDR 0x78 // 0x42 for 7690
    #define PLL_ADDR 0x29 // only for 7690
    #define R_PLL0 0x30   // only for 7692
    #define R_PLL1 0x31   // only for 7692
    #define CLKRC 0x11
    #define REG0C 0x0C
    #define REG0E 0x0E
    #define ROW_MSB_OUT 0xCC
    #define ROW_LSB_OUT 0xCD
    #define COL_MSB_OUT 0xCE
    #define COL_LSB_OUT 0xCF
    #define ROW_MSB_IN 0xC8
    #define ROW_LSB_IN 0xC9
    #define COL_MSB_IN 0xCA
    #define COL_LSB_IN 0xCB
    #define SUB_SAMPLE 0x12
    #define SCALING 0x81
    #define H_SIZE 0x18
    #define V_SIZE 0x1A
    #define SKIP 0x85

    #define COLS FRAME_COLS
    #define ROWS FRAME_ROWS

    typedef __eds__ unsigned char FrameType[ROWS][COLS];
    

    #define PWDN _LATbit(CAM_PWDN_PORT, CAM_PWDN_BIT)

    // Determine delay lengths.  Short delay should be longer than PCLK.
    // Long delay should be shorter than HREF.
    #define Short_Delay_us (SHORT_DELAY_IN_US)
    #define Short_Delay_cycles (unsigned long)(Short_Delay_us*(((double)FCY)/(1000000.0)))

    #define Long_Delay_us (LONG_DELAY_IN_US)
    #define Long_Delay_cycles (unsigned long)(Long_Delay_us*(((double)FCY)/1000000.0))

    #define XVCLK_FREQ_IN_MHZ (((float)SYSTEM_FREQUENCY_IN_MHZ)/((float) XVCLK_DIVIDER))
    #define CLKRC_DIVDER (int)(XVCLK_FREQ_IN_MHZ / PCLK_FREQ_IN_MHZ)

    void Camera_Init(void);

    // These defintions have to do with I/O
    #define SEGMENT_LENGTH 28
    typedef uint8_t SEGMENT_TYPE[SEGMENT_LENGTH];

    typedef enum {
        STATE_PACKET,
        CAMERA_PACKET,
        CAMERA_START_FRAME_PACKET,
        CAMERA_END_FRAME_PACKET
    } TXState;

    extern volatile TXState PacketState;
    extern volatile uint8_t CAMReadNextFrame;
    int16_t CAM_Get_Next_Segment_ID(void);
    void CAM_Send_One_Frame(void);
    inline void CAM_Toggle_Enable(void);
    inline void CAM_Change_DF_State(void);
    void CAM_Fill_TX_Segment(SEGMENT_TYPE* Cam_Pixel_Data);
    

#ifdef	__cplusplus
}
#endif

#endif	/* CAMERA_H */

