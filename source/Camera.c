#include "Camera.h"
#ifdef DRAGONFLY_DRONE
#include "DFProtocol.h"
#endif

#ifdef DRONE
#include "SDProtocol.h"
#endif

// Function prototypes
void XVCLK_Init(void); // XVCLK
void VSYNC_Init(void); // VSYNC
void PCLK_Init(void); // PCLK
void HREF_Init(void); // HREF

// I/O Function prototypes

// Enable/disable function prototypes
inline void CAM_Enable(void);
inline void CAM_Disable(void);

// Declare space for two frames
FrameType frame0 __attribute__((far));
FrameType frame1 __attribute__((far));
//FrameType* frame1;

// Near data line for quicker storage
unsigned char nearLine[COLS*3] __attribute__((near));

// Frame management variables
unsigned char currentFrame = 0;
//FrameType* storedFrames[2];
FrameType* storedFrames[2] = {&frame0, &frame1};

// Keep track of line and pixel
volatile unsigned char lineCount = 0;
volatile unsigned char maxLineCount = ROWS - 1 + 20; // There are 18 dummy lines at the beginning of each frame
volatile unsigned char* currentPixel = &nearLine[0];
volatile unsigned char* maxPixel = &nearLine[COLS*3 - 1];
volatile unsigned int ndx = 0; // for interrupts
volatile unsigned int ii = 0; // for non-interrupts

// Set delay lengths
const unsigned long Short_Delay_Timer_LSW = (Short_Delay_cycles & 0xFFFF);
const unsigned long Short_Delay_Timer_MSW = (Short_Delay_cycles >> 16);
const unsigned long Long_Delay_Timer_LSW  = (Long_Delay_cycles & 0xFFFF);
const unsigned long Long_Delay_Timer_MSW  = (Long_Delay_cycles >> 16);

// This is all I/O information:
volatile uint16_t CAM_Segment_ID = 0;
volatile uint16_t CAM_Last_Segment_Length = (((ROWS*COLS)%SEGMENT_LENGTH) > 0) ? ((ROWS*COLS)%SEGMENT_LENGTH) : (SEGMENT_LENGTH);
volatile uint16_t CAM_Max_Segment_ID = (ROWS*COLS)/SEGMENT_LENGTH + MIN((ROWS*COLS)%SEGMENT_LENGTH, 1) - 1;
//uint8_t CAM_Pixel_Data[SEGMENT_LENGTH] = {0};
volatile TXState PacketState = STATE_PACKET;
unsigned int outputFrame = 0;
uint16_t CAM_Current_Row = 0;
uint16_t CAM_Current_Col = 0;

// To enable/disable camera
volatile uint8_t CAMReadNextFrame = 0;


// Function definitions

// This function configures the camera in the following steps:
// 1. Makes the PWDN control signal an output pin and drives it high
// 2. Initializes the desired configurations in the I2C module
// 3. Waits for an amount of time specified by the data sheet before...
// 4. Initializes the configuration registers for the Output Compare module to
//    produce a PWM to use as the camera input XVCLK.
// 5. Pulls the PWDN control signal low to turn on the camera.
// 6. Uses I2C to configure the camera's internal registers to their desired
//    settings.
// 7. Initializes the VSYNC input pin of the MCU and sets up its interrupt.
// 8. Initializes the PCLK input pin of the MCU and sets up its interrupt.
// 9. Initializes the timer interrupt that is used to simulate HREF.
// 10. Globally enables interrupts so that normal function may begin.
void Camera_Init()
{
    TRISDbits.TRISD10 = 0; // remove
    _RD10 = 1; // remove
    #if (CAM_EN == 1)
        // Dynamically allocate space for second frame
        //frame1 = (FrameType*)malloc(sizeof(unsigned char) * ROWS*COLS);
        //storedFrames[0] = &frame0;
        //storedFrames[1] = frame1;

        // Map Pins
        _TRISbit(CAM_VSYNC_PORT, CAM_VSYNC_BIT)   = 1; Nop(); Nop();
        _TRISbit(CAM_PCLK_PORT, CAM_PCLK_BIT)     = 1;

        _TRISbit(CAM_XVCLK_PORT, CAM_XVCLK_BIT)   = 0;
        _TRISbit(CAM_PWDN_PORT, CAM_PWDN_BIT)     = 0;

        _TRISbit(CAM_DATA_7_PORT, CAM_DATA_7_BIT) = 1;
        _TRISbit(CAM_DATA_6_PORT, CAM_DATA_6_BIT) = 1;
        _TRISbit(CAM_DATA_5_PORT, CAM_DATA_5_BIT) = 1;
        _TRISbit(CAM_DATA_4_PORT, CAM_DATA_4_BIT) = 1;
        _TRISbit(CAM_DATA_3_PORT, CAM_DATA_3_BIT) = 1;
        _TRISbit(CAM_DATA_2_PORT, CAM_DATA_2_BIT) = 1;
        _TRISbit(CAM_DATA_1_PORT, CAM_DATA_1_BIT) = 1;
        _TRISbit(CAM_DATA_0_PORT, CAM_DATA_0_BIT) = 1;

        _ODCbit(CAM_SIOC_PORT, CAM_SIOC_BIT)      = 1;
        _ODCbit(CAM_SIOD_PORT, CAM_SIOD_BIT)      = 1;
        _TRISbit(CAM_SIOC_PORT, CAM_SIOC_BIT)     = 0;
        _TRISbit(CAM_SIOD_PORT, CAM_SIOD_BIT)     = 0;

        // Map an output to control PWDN, pin 60 on J11
        PWDN = 0;
        Nop();
        PWDN = 1;

        // Initialize the I2C module
        I2C_Init();

        // Wait for power down
        long Start_Up_Timer = FCY/500; // Double the minimum recommended delay (2 ms)
        while(--Start_Up_Timer);
        XVCLK_Init(); // Set up XVCLK
        PWDN = 0; // Pull PWRDN signal low

        I2C_Robust_Write(CAMERA_ADDR, 0x0E, 0x08); // Sleep mode
        I2C_Robust_Write(CAMERA_ADDR, 0x69, 0x52); // Change BLC window
        I2C_Robust_Write(CAMERA_ADDR, 0x1E, 0xB3); // Less than 1 frame dummy lines
        I2C_Robust_Write(CAMERA_ADDR, 0x2B, 0x00); // No dummy pixels
        I2C_Robust_Write(CAMERA_ADDR, 0x48, 0x42); // Reserved
        I2C_Robust_Write(CAMERA_ADDR, 0x16, 0x03); // Reserved
        I2C_Robust_Write(CAMERA_ADDR, 0x62, 0x10); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x22, ((I2C_Robust_Read(CAMERA_ADDR, 0x22)) & (0xCF)) | (0x10)); // Set flip register to conform to skip mode
        I2C_Robust_Write(CAMERA_ADDR, 0x12, 0x40); // Enable sub-sample (skip) mode| 0x00
        I2C_Robust_Write(CAMERA_ADDR, 0x17, 0x65); // Horizontal start point
        I2C_Robust_Write(CAMERA_ADDR, 0x18, 0xA4); // Horizontal sensor size
        I2C_Robust_Write(CAMERA_ADDR, 0x19, 0x0C); // Vertical start point
        I2C_Robust_Write(CAMERA_ADDR, 0x1A, 0xF6); // Vertical sensor size | 0xF6
        I2C_Robust_Write(CAMERA_ADDR, 0x3E, 0x60); // Qualify PCLK with HREF
        I2C_Robust_Write(CAMERA_ADDR, 0x64, 0x11); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x67, 0x20); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x81, 0x3F); // Module enables

        // Define outputs as 160 x 120 (QQVGA)
        I2C_Robust_Write(CAMERA_ADDR, ROW_MSB_OUT, 0x00);
        I2C_Robust_Write(CAMERA_ADDR, ROW_LSB_OUT, COLS);
        I2C_Robust_Write(CAMERA_ADDR, COL_MSB_OUT, 0x00);
        I2C_Robust_Write(CAMERA_ADDR, COL_LSB_OUT, ROWS);
        // Define inputs as 640 x 480 (VGA)
        I2C_Robust_Write(CAMERA_ADDR, ROW_MSB_IN, 0x02);
        I2C_Robust_Write(CAMERA_ADDR, ROW_LSB_IN, 0x80);
        I2C_Robust_Write(CAMERA_ADDR, COL_MSB_IN, 0x01); // should be 0x00 & 0xF0
        I2C_Robust_Write(CAMERA_ADDR, COL_LSB_IN, 0xE0);

        //I2C_Robust_Write(CAMERA_ADDR, 0xD0, 0x48); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x82, 0x03); // Make output YUV422 format, bar test
        I2C_Robust_Write(CAMERA_ADDR, 0x0E, 0x00); // No longer sleep mode
        //I2C_Robust_Write(CAMERA_ADDR, 0x70, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x71, 0x34); // Band counter threshold
        I2C_Robust_Write(CAMERA_ADDR, 0x74, 0x28); // Low sum threshold
        I2C_Robust_Write(CAMERA_ADDR, 0x75, 0x98); // High sum threshold
        I2C_Robust_Write(CAMERA_ADDR, 0x76, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x77, 0x64); // Low 8 bits of light meter low threshold
        I2C_Robust_Write(CAMERA_ADDR, 0x78, 0x01); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x79, 0xC2); // Low 8 bits of light meter high threshold
        I2C_Robust_Write(CAMERA_ADDR, 0x7A, 0x4E); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x7B, 0x1F); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x7C, 0x00); // Default

        // Set frame rate: 63 MHz/5 = 12.6 MHz XVCLK
        I2C_Robust_Write(CAMERA_ADDR, R_PLL0, 0x04); // Divide by 1 = 12.6 MHz
        I2C_Robust_Write(CAMERA_ADDR, CLKRC, CLKRC_DIVDER);  // Divide SCLK by 7, yield 1.8 MHz

        I2C_Robust_Write(CAMERA_ADDR, 0x20, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x21, 0x57); // Banding filter step for 50 Hz and 60 Hz
        I2C_Robust_Write(CAMERA_ADDR, 0x50, 0x4D); // 50 Hz Banding AEC 8 bits
        I2C_Robust_Write(CAMERA_ADDR, 0x51, 0x40); // 60 Hz Banding AEC 8 bits
        I2C_Robust_Write(CAMERA_ADDR, 0x4C, 0x7D); // Reserved
        //I2C_Robust_Write(CAMERA_ADDR, 0x0E, 0x00); // Not sleep mode
        I2C_Robust_Write(CAMERA_ADDR, 0x80, 0x7F); // Lens correction enable
        I2C_Robust_Write(CAMERA_ADDR, 0x85, 0x00); // Default | 0x08
        I2C_Robust_Write(CAMERA_ADDR, 0x86, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x87, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x88, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x89, 0xF0); // LC R gain 2a
        I2C_Robust_Write(CAMERA_ADDR, 0x8A, 0xF0); // LC G gain 22
        I2C_Robust_Write(CAMERA_ADDR, 0x8B, 0xF0); // LC B gain 20
        I2C_Robust_Write(CAMERA_ADDR, 0xBB, 0xAB); // M1 AB
        I2C_Robust_Write(CAMERA_ADDR, 0xBC, 0x84); // M2 84
        I2C_Robust_Write(CAMERA_ADDR, 0xBD, 0x27); // M3 27
        I2C_Robust_Write(CAMERA_ADDR, 0xBE, 0x0E); // M4 0E
        I2C_Robust_Write(CAMERA_ADDR, 0xBF, 0xB8); // M5 B8
        I2C_Robust_Write(CAMERA_ADDR, 0xC0, 0xC5); // M6 C5
        I2C_Robust_Write(CAMERA_ADDR, 0xC1, 0x1E); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0xB7, 0x05); // Offset (half of default)
        I2C_Robust_Write(CAMERA_ADDR, 0xB8, 0x09); // Base 1
        I2C_Robust_Write(CAMERA_ADDR, 0xB9, 0x00); // Base 2
        I2C_Robust_Write(CAMERA_ADDR, 0xBA, 0x18); // Gain x4 is limited to 16
        I2C_Robust_Write(CAMERA_ADDR, 0x5A, 0x1F); // Change slope of UV curve
        I2C_Robust_Write(CAMERA_ADDR, 0x5B, 0x9F); // Change UV adj gain high threshold LSB
        I2C_Robust_Write(CAMERA_ADDR, 0x5C, 0x69); // Change UV adj gain high threshold MSB
        I2C_Robust_Write(CAMERA_ADDR, 0x5D, 0x42); // Change UV adj low threshold, 1/4 center to average
        I2C_Robust_Write(CAMERA_ADDR, 0x24, 0x78); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x25, 0x68); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0x26, 0xB3); // Change fast mode operating region
        I2C_Robust_Write(CAMERA_ADDR, 0xA3, 0x0B); // YST1  10, 0b
        I2C_Robust_Write(CAMERA_ADDR, 0xA4, 0x15); // YST2  12, 15
        I2C_Robust_Write(CAMERA_ADDR, 0xA5, 0x29); // YST3  35, 29
        I2C_Robust_Write(CAMERA_ADDR, 0xA6, 0x4A); // YST4  5a, 4a
        I2C_Robust_Write(CAMERA_ADDR, 0xA7, 0x58); // YST5  69, 58
        I2C_Robust_Write(CAMERA_ADDR, 0xA8, 0x65); // YST6  76, 65
        I2C_Robust_Write(CAMERA_ADDR, 0xA9, 0x70); // YST7  80, 70
        I2C_Robust_Write(CAMERA_ADDR, 0xAA, 0x7B); // YST8  88, 7b
        I2C_Robust_Write(CAMERA_ADDR, 0xAB, 0x85); // YST9  8f, 85
        I2C_Robust_Write(CAMERA_ADDR, 0xAC, 0x8E); // YST10 96, 8e
        I2C_Robust_Write(CAMERA_ADDR, 0xAD, 0xA0); // YST11 a3, a0
        I2C_Robust_Write(CAMERA_ADDR, 0xAE, 0xB0); // YST12 af, b0
        I2C_Robust_Write(CAMERA_ADDR, 0xAF, 0xCB); // YST13 c4, cb
        I2C_Robust_Write(CAMERA_ADDR, 0xB0, 0xE1); // YST14 d7, e1
        I2C_Robust_Write(CAMERA_ADDR, 0xB1, 0xF1); // YST15 e8, f1
        I2C_Robust_Write(CAMERA_ADDR, 0xB2, 0x14); // YSLP15 20, 14
        I2C_Robust_Write(CAMERA_ADDR, 0x8E, 0x92); // AWB simple
        I2C_Robust_Write(CAMERA_ADDR, 0x96, 0xFF); // Changes value top limit
        I2C_Robust_Write(CAMERA_ADDR, 0x97, 0x00); // Changes value bottom limit
        I2C_Robust_Write(CAMERA_ADDR, 0x14, 0x3B); // x16 is auto gain ceiling, set banding width to 50 Hz 3b

        // Make image black and white
        I2C_Robust_Write(CAMERA_ADDR, 0x81, 0x3F); // 0F
        I2C_Robust_Write(CAMERA_ADDR, 0x28, 0x00); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0xD2, 0x18); // Fix V enable, fix U enable
        I2C_Robust_Write(CAMERA_ADDR, 0xDA, 0x80); // Default
        I2C_Robust_Write(CAMERA_ADDR, 0xDB, 0x80); // Default

        I2C_Robust_Write(CAMERA_ADDR, 0x0E, 0x00); // Normal mode

        // Configure VSYNC operation
        VSYNC_Init();

        // Configure PCLK operation
        PCLK_Init();

        // Set up Timer 1
        HREF_Init();
    #endif
}

// This function sets up the XVCLK pin to use the Output Compare 1 Module to
// generate an output "clock" at the highest possible rate.  The pin is also
// configured to be an output in this function, and uses the Edge-Aligned PWM
// setting.
void XVCLK_Init(void)
{
    // Clear config registers
    XVCLK_CON1R = 0;
    XVCLK_CON2R = 0;

    XVCLK_DUTY   = (XVCLK_DIVIDER - 1)/2; // Initialize duty cycle to 50%
    XVCLK_PERIOD = XVCLK_DIVIDER - 1; // Set fastest possible period

    XVCLK_CON2.SYNCSEL = 0x1F; // No sync or trigger source
    XVCLK_CON1.OCTSEL = 0b111; // Use peripheral clock
    XVCLK_CON1.OCSIDL = 0; // Continue operation in Idle

    XVCLK_CON1.OCM = 0b110; // Edge-aligned PWM mode
    XVCLK_OC_RP = XVCLK_OC_MODULE; // Map OC1 to RF1
}

// This function sets up the timer interrupt module to be used in lieu of an
// HREF input pin
void HREF_Init()
{
    SHORT_HREF_TCON.TON = 0;
    LONG_HREF_TCON.TON = 0;

    SHORT_HREF_TCON.T32 = 0;

    SHORT_HREF_TCON.TSIDL = 0; // Continue in idle mode
    LONG_HREF_TCON.TSIDL = 0;
    SHORT_HREF_TCON.TGATE = 0; // Disable gated mode
    SHORT_HREF_TCON.TCKPS = 0b00; // 1:1 prescaler. each count is 15.8 ns
    SHORT_HREF_TCON.TCS = 0; // Use internal peripheral clock

    SHORT_HREF_TMR = 0; // Clear LSW
    LONG_HREF_TMR = 0; // Clear MSW

    LONG_HREF_TMR_IE = 1; // Enable interrupt.  Even though interrupt is enabled,
                       // it won't initially do anything because TON = 0, which
                       // means the timer isn't even counting
    SHORT_HREF_TMR_IE = 1; // The same goes for this interrupt.

    SHORT_HREF_PR = Short_Delay_Timer_LSW; // Set the 16-bit timer to go off
    LONG_HREF_PR = Long_Delay_Timer_MSW;  // Set the MSW of the 32-bit timer

}

// This function sets up the VSYNC pin to use the External Interrupt 1 Module to
// trigger on a rising edge.  The pin is also configured to be an input in this
// function.
void VSYNC_Init()
{
    _GIE = 0;

    VSYNC_INT_EP = 0; // Trigger on rising edge

    VSYNC_INT_IF = 0; // Reset flag

    VSYNC_INT_IE = 0; // Disable interrupt

    #if (VSYNC_INT != 0)
        VSYNC_INT_MODULE = VSYNC_INT_RP;
    #endif
}

// Change this
// This interrupt handles the process of events that need to occur when VSYNC
// goes high.
void __attribute__((interrupt, no_auto_psv)) VSYNC_INT_INTERRUPT(void)
{
    VSYNC_INT_IF = 0; // Reset flag

    VSYNC_INT_IE = 0; // Disable self.  Re-enable after frame.

    SHORT_HREF_TCON.TON = 1; // Turn on HREF short timer
    PCLK_INT_IF = 0; // Reset pixel interrupt flag
    PCLK_INT_IE = 1; // Enable PCLK interrupt
    LED3_W ^= 1;
    SHORT_HREF_TMR_IE = 1;
}

// This function sets up the PCLK pin to use the External Interrupt 2 Module to
// trigger on a rising edge.
void PCLK_Init()
{
    _GIE = 0;
    PCLK_INT_EP = 0; // Trigger on rising edge

    PCLK_INT_IF = 0; // Reset flag
    PCLK_INT_IE = 0; // Have PCLK interrupt disabled to start

    // Remap pin to PCLK input
    #if (PCLK_INT != 0)
        PCLK_INT_MODULE = PCLK_INT_RP;
    #endif
}

// This interrupt handles the process of events that need to occur when PCLK
// goes high.
void __attribute__((interrupt, no_auto_psv)) PCLK_INT_INTERRUPT(void)
{
    *currentPixel = (PORTB >> 4); // Read the pixel value
    PCLK_INT_IF = 0; // Reset flag
    SHORT_HREF_TMR = 0; // Reset the short timer before it can reset.
    currentPixel++; // Increments the pointer in the near line
}

// This is the "short timer" interrupt.
// This interrupt handles the "end-of-line" and "end-of-frame" transitions
// It has three cases.
// In the first case, the interrupt has triggered for the
// first time after a line (positive duty of HREF) has concluded.  In this case,
// it transfers the data from nearLine to frame, extends the time that it waits
// before interrupting, updates the linecount, and resets the current pixel
// pointer to the first element of nearLine.
// In the second case, the interrupt has triggered again after a line has
// concluded.  This interrupt does nothing.
// In the third case, the interrupt has triggered for the first time after the
// last line in a frame.  It copies over the last nearLine into frame, resets
// lineCount, turns off the timer, resets the timer accumulator, and resets the
// the current pixel pointer to the first element of nearLine.  By stopping the
// timer, the interrupt is effectively disabled until the next VSYNC interrupt
// occurs.
// In all cases, the interrupt flag is cleared.
void __attribute__((interrupt, no_auto_psv)) _T6Interrupt(void)
{
    LED2_W = 1;
    if ((lineCount < maxLineCount) && (currentPixel != &nearLine[0]))
    {
        SHORT_HREF_IP = 6;
        LONG_HREF_IP  = 6;   // HREF has fourth highest priority
        // This case is the first time this interrupt goes off during the "off"
        // period of an HREF cycle
        SHORT_HREF_PR = Long_Delay_Timer_LSW; // Make the short timer the LSW of the long timer

        // Loop through, storing nearLine values in frame:
        if (lineCount > 20) // Do not store dummy lines
        {
            for(ndx = 0; ndx < COLS; ndx++)
            {
                (*storedFrames[currentFrame])[lineCount-20][ndx] = nearLine[2*ndx];
            }
        }
        
        lineCount++; // update linecount
        currentPixel = &nearLine[0]; // Reset pixel to restart line.

        SHORT_HREF_TCON.T32 = 1; // Enable the 32-bit timer
        if (CAMReadNextFrame) {
        #ifdef DRAGONFLY_DRONE
            DFP_Continue();
        #endif
        #ifdef DRONE
            SDP_Continue();
        #endif
        }
        SHORT_HREF_TMR_IE = 0;
    }
    else if (lineCount >= maxLineCount)
    {
        SHORT_HREF_IP = 6;
        LONG_HREF_IP  = 6;   // HREF has fourth highest priority
        for(ndx = 0; ndx < COLS; ndx++) (*storedFrames[currentFrame])[ROWS-1][ndx] = nearLine[2*ndx];
        lineCount = 0; // Frame is finished, reset line count.
        SHORT_HREF_TCON.TON = 0; // Stop timer.  It will restart with new frame.
        SHORT_HREF_TMR = 0; // Reset timer value.
        LONG_HREF_TMR = 0;
        LONG_HREF_TMRHLD = 0;
        currentPixel = &nearLine[0]; // Reset pixel to restart line.
        PCLK_INT_IE = 0; // Disable pixel clock interrupt
        currentFrame ^= 1; // Switch frames
        SHORT_HREF_TMR_IE = 0;

        // Turn VSYNC back on if camera is enabled
        if (CAMReadNextFrame)
        {
            //VSYNC_INT_IF = 0;
            VSYNC_INT_IE = 1; // Enable VSYNC interrupt
        }
    }
    else
    {
        SHORT_HREF_IP = 1;
        LONG_HREF_IP  = 1;   // HREF has fourth highest priority
    }
    SHORT_HREF_TMR_IF = 0; // Reset interrupt flag
}

// This is the "long timer" interrupt.
// This interrupt only occurs in the gap of time between HREFs, and supplants
// the short timer interrupt in order to allow more uninterrupted time for
// the non-camera data-capturing processes to operate.
void __attribute__((interrupt, no_auto_psv)) _T7Interrupt(void)
{
    // Switch back to the short timer settings:
    SHORT_HREF_PR = Short_Delay_Timer_LSW;
    SHORT_HREF_TCON.T32 = 0;

    // Make sure the timer is reset
    SHORT_HREF_TMR = 0;
    LONG_HREF_TMR = 0;
    LONG_HREF_TMRHLD = 0;

    // Reset interrupt flag
    LONG_HREF_TMR_IF = 0;
    SHORT_HREF_TMR_IE = 1;
}

void CAM_Send_One_Frame(void)
{
    PacketState = CAMERA_START_FRAME_PACKET; // Set image-sending flag
    CAMReadNextFrame = 0; // Disable camera so image won't overwrite
    outputFrame = currentFrame ^ 1; // Send the last completed frame
    while(VSYNC_INT_IE);
    CAM_Current_Col = 0;
    CAM_Current_Row = 0;
}

int16_t CAM_Get_Next_Segment_ID(void)
{
    return CAM_Segment_ID;
}

void CAM_Fill_TX_Segment(SEGMENT_TYPE* CAM_Pixel_Data)
{    
    if (CAM_Segment_ID == CAM_Max_Segment_ID) // last segment
    {
        for (ii = 0; ii < CAM_Last_Segment_Length; ii++)
        {
            (*CAM_Pixel_Data)[ii] = (*storedFrames[outputFrame])[CAM_Current_Row][CAM_Current_Col];
            CAM_Current_Col++;
        }
        CAM_Current_Row = 0; // Reset
        CAM_Current_Col = 0; // Reset
        CAM_Segment_ID = 0; // Reset
    }
    else if ((CAM_Current_Col + SEGMENT_LENGTH) < COLS) // fits on one line
    {
        for (ii = 0; ii < SEGMENT_LENGTH; ii++)
        {
            (*CAM_Pixel_Data)[ii] = (*storedFrames[outputFrame])[CAM_Current_Row][CAM_Current_Col];
            CAM_Current_Col++;
        }
        CAM_Segment_ID++; // Update
        if (CAM_Current_Col == (COLS))
        {
            CAM_Current_Row++;
            CAM_Current_Col = 0;
        }
    }
    else // doesn't fit on one line
    {
        for(ii = 0; CAM_Current_Col < COLS; ii++)
        {
            (*CAM_Pixel_Data)[ii] = (*storedFrames[outputFrame])[CAM_Current_Row][CAM_Current_Col];
            CAM_Current_Col++;
        }
        CAM_Current_Row++;
        CAM_Current_Col = 0;
        for(; ii < SEGMENT_LENGTH; ii++)
        {
            (*CAM_Pixel_Data)[ii] = (*storedFrames[outputFrame])[CAM_Current_Row][CAM_Current_Col];
            CAM_Current_Col++;
        }
        CAM_Segment_ID++; // Update
    }
}

inline void CAM_Change_DF_State(void)
{
    if (CAM_Segment_ID == 0)
    {
        PacketState = CAMERA_END_FRAME_PACKET;
        LED4_W ^= 1;
        CAM_Enable(); // Turn camera back on
    }
}

inline void CAM_Enable(void)
{
    // Clear camera related interrupt flags
    VSYNC_INT_IF = 0; // VSYNC
    PCLK_INT_IF  = 0; // PCLK
    LONG_HREF_TMR_IF   = 0; // Long timer
    SHORT_HREF_TMR_IF   = 0; // HREF timer

    // Turn on flag
    CAMReadNextFrame = 1;

    // Enable VSYNC interrupt
    VSYNC_INT_IE = 1;
}

inline void CAM_Disable(void)
{
    // Let the camera finish its current frame and then turn off
    CAMReadNextFrame = 0;
}

inline void CAM_Toggle_Enable(void)
{
    if (CAMReadNextFrame) CAM_Disable();
    else CAM_Enable();
}
