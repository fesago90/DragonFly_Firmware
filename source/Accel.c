#include "Common.h"
#include "Accel.h"

//__eds__ AccData AccDataBuffer[2] __attribute__((space(dma), eds)) =
//{{0xaa, 0xaaaa, 0xaaaa, 0xaaaa},{0xaa, 0xaaaa, 0xaaaa, 0xaaaa}};

AccData AccDataBufferA __attribute__((space(dma), eds)) = {0xaaaa, 0xaaaa, 0xaaaa, 0xaaaa};
AccData AccDataBufferB __attribute__((space(dma), eds)) = {0xaaaa, 0xaaaa, 0xaaaa, 0xaaaa};

int AccBufferIndex = 0;

void        Acc_Init_SPI();
void        Acc_Init_DMA();
void        Acc_Init_Interrupt();
void        Acc_Check_ID();
void        Acc_Write_Register(uint8_t regAddr, uint8_t value);
uint16_t    Acc_Read_Register(uint8_t regAddr, uint8_t isByte);
inline void Acc_CS_Assert();
inline void Acc_CS_Disassert();
inline void Acc_Trigger_Read_Data();
inline void Acc_Data_Received();

inline uint8_t SPI3_read_write_byte(uint8_t data_byte);

/*
 * This ISR is executed when the Accel's INT1 pin goes high, indicating
 * that a new sample is ready.
 */
void _ISR_NO_PSV ACC_INT_INTERRUPT()
{
    ACC_INT_IF     = 0;
    Acc_Trigger_Read_Data();
}

/* This interrupt is triggered when SPI3 received a full block of data (7 bytes)
 * and the DMA has copied it to RAM
 */
void _ISR_NO_PSV ACC_DMA_INTERRUPT()
{
    ACC_DMA_IF = 0;
    Acc_Data_Received();
}

void Acc_Init()
{
    #if (ACC_EN == 1)
        uint8_t measuring = 0;

        do {
            Acc_Init_SPI();     // Initialize SPI module
            Acc_Check_ID();     // Test & check Acc connection.

            Acc_Write_Register(0x2D, 0b00000000);   // Place into standby mode
            Acc_Write_Register(0x2E, 0b00000000);   // Disables all interrupts
            Acc_Write_Register(0x2C, 0b00001101);   // Write to BW_RATE for 800 Hz operation (400 Hz BW)
            Acc_Write_Register(0x31, 0b00001010);   // Full resolution (~128 LSB/g), g-range to be plus/minus 4g
            Acc_Write_Register(0x38, 0b00000000);   // Use bypass mode
            Acc_Write_Register(0x2F, 0b00000000);   // DATA_READY uses INT1 pin

            Acc_Init_Interrupt();

            Acc_Write_Register(0x2D, 0b00001000);   // Place into normal measuring mode
            measuring = Acc_Read_Register(0x2D, 1); // Read current status (to check if it is in measuring mode)

            Acc_Write_Register(0x2E, 0b10000000);   // Enables DATA_READY interrupt

            // Ensures that accel is in measuring mode and that the
            // interrupt pin is not yet set
        } while ( ((measuring & 0b00001000) == 0) && (ACC_INT1_R == 1));

        Acc_Init_DMA();     // Initializes DMA0 channel for auto RX to RAM
    #endif
}

/*
 * DMA0 channel is initialized to read data from SPI3 whenever a byte is received
 * Null data peripheral write is enabled so that it automatically triggers SPI
 * activity to read all 7-bytes of the accelerometer data transaction. The
 * transaction consists of 1 command byte and 6 data bytes
 */
void Acc_Init_DMA() 
{
    // Set up DMA0 channel to RX accelerometer data through SPI
    ACC_DMA_IE         = 0;        // Disable DMA0 interrupt
    ACC_DMA_IF         = 0;        // Clear DMA0 interrupt flag
    ACC_DMACON.CHEN    = 0;        // Disable DMA0 channel

    ACC_DMAPAD         = 0x02A8;   // DMA0 channel is connected to SPI3BUF
    ACC_DMACON.SIZE    = 1;        // Byte data transfer size (not Word)
    ACC_DMACON.DIR     = 0;        // Read from peripheral, write to RAM
    ACC_DMACON.AMODE   = 0b00;     // Register indirect with post-increment
    ACC_DMACON.MODE    = 0b01;     // One-shot, with ping-pong mode DISABLED
    ACC_DMACON.NULLW   = 1;        // Null data writes generated to peripheral
    ACC_DMAREQ.IRQSEL  = 0b01011011;    // SPI3 transfer done triggers DMA transfer

    /* AccData structure:
     * | J  | X  | Y  | Z  |   --> J is junk. X, Y & Z are data values (16-bit)
     *    |C| X  | Y  | Z  |   --> C is the junk SPI data rx'ed when tx'ing the command byte
     *  ^^ --> The + 1 in the following line is needed to properly align
     *         the DMA's data range with where the structure is actually
     *         placed in memory
     */
    ACC_DMASTAL        = __builtin_dmaoffset(&AccDataBufferA) + 1;
    ACC_DMASTAH        = 0;

    ACC_DMACNT         = 6;        // A 6 actually means 7 DMA transfers (1 byte each)
    ACC_DMACON.CHEN    = 1;        // Enable DMA0 channel
    ACC_DMA_IE         = 1;        // Enable DMA0 interrupt
}

/*
 * This function places a read DATAX0 command with multibyte support in SPI3BUF
 * to trigger a transaction. DMA0 takes care of retrieving the data into RAM and
 * starting the 6 extra SPI transactions. With multi-byte option, the accel
 * sends the values of registers DATAX1 to DATAZ1 in the following SPI transactions
 */
inline void Acc_Trigger_Read_Data()
{
    ACC_DMACON.CHEN = 1;       // Re-enable DMA0 (one-shot mode turns it off)
    Acc_CS_Assert();            // Assert SPI channel
    //SPI3BUF = 0b11110010;       // Read register 0x32 (DATAX0) w/ Multi-Byte
    ACC_SPIBUF = 0b11110010;
}

inline void Acc_Data_Received()
{
    Acc_CS_Disassert();         // De-assert SPI channel
    if (AccBufferIndex == 0)
        ACC_DMASTAL = (uint16_t)&AccDataBufferB + 1;
    else
        ACC_DMASTAL = (uint16_t)&AccDataBufferA + 1;
    AccBufferIndex ^= 1;
}

/*
 * Initialize SPI3 module for communicating with Accel ADXL345
 * Clock speed is 1/16 of instruction frequency.
*/
void Acc_Init_SPI()
{
//    _ANSELbit(ACC_CS_PORT, ACC_CS_BIT)     = 0;  // CS pin is digital
//    _ANSELbit(ACC_CLK_PORT, ACC_CLK_BIT)   = 0;  // CLK pin is digital
//    _ANSELbit(ACC_MOSI_PORT, ACC_MOSI_BIT) = 0;  // MOSI pin is digital
//    _ANSELbit(ACC_MISO_PORT, ACC_MISO_BIT) = 0;  // MISO pin is digital

    _TRISbit(ACC_CS_PORT, ACC_CS_BIT)     = 0;  // CS pin is output
    _TRISbit(ACC_CLK_PORT, ACC_CLK_BIT)   = 0;  // CLK pin is output
    _TRISbit(ACC_MOSI_PORT, ACC_MOSI_BIT) = 0;  // MOSI pin is output
    _TRISbit(ACC_MISO_PORT, ACC_MISO_BIT) = 1;  // MISO pin is input

    Acc_CS_Disassert();

    ACC_SCK_RP  = SCK3;             // Map RP67 to SCLK3
    ACC_SDO_RP  = SDO3;             // Map RP68 to MOSI3
    SDI3        = ACC_SDI_RP;       // Map RP66 to MISO3

    ACC_SPI_IF = 0;                // Clear the Interrupt Flag
    ACC_SPI_IE = 0;                // Disable the interrupt (DMA request is performed nonetheless)

    ACC_SPI_CON1.DISSCK = 0;    // Internal Serial Clock is Enabled
    ACC_SPI_CON1.DISSDO = 0;    // SDOx pin is controlled by the module
    ACC_SPI_CON1.MODE16 = 0;    // Communication is byte-wide (8 bits)
    ACC_SPI_CON1.MSTEN  = 1;    // Master mode Enabled
    ACC_SPI_CON1.SSEN   = 0;    // SSN is GPIO
    ACC_SPI_STAT.SPISIDL = 0;   // Continue operation in Idle mode

    ACC_SPI_CON1.CKE  = 0;      // Serial output data changes on clock transition from idle to active
    ACC_SPI_CON1.CKP  = 1;      // Idle state for clock is a high level
    ACC_SPI_CON1.SMP  = 0;      // Input data is sampled at middle of data output time
    ACC_SPI_CON1.SPRE = 0b111;  // Secondary prescale 1:1
    ACC_SPI_CON1.PPRE = 0b01;   // Primary prescale 16:1
    //ACC_SPI_CON1.SPRE = 0b101;  // Secondary prescale 3:1
    //ACC_SPI_CON1.PPRE = 0b10;   // Primary prescale 4:1

    ACC_SPI_STAT.SPIEN = 1;     // Enable SPI module
}

/*
 * Checks SPI settings and connection by reading the Accel's device ID.
 * Attempts reading up to 10 times with delays in betweeb.
 */
void Acc_Check_ID()
{
    uint8_t devId;
    int errCount = 0;

    devId = Acc_Read_Register(0x0, 1);     // get device ID

    while (devId != 0b11100101) {       // datasheet says it should be 0xE5
        __delay_us(100);
        if (++errCount == 10)           // Attempt reading up to 10 times
            Display_Error(ErrAcc);
        devId = Acc_Read_Register(0x0, 1);
    }
}

/*
 * Enables the dsPIC's INT2 interrupt on the pin that is connected to
 * the accel's interrupt pin. Positive edge triggered.
 */
void Acc_Init_Interrupt()
{
    _TRISbit(ACC_INT1_PORT, ACC_INT1_BIT) = 1;  // Set RP61 as input
    //_ANSELbit(ACC_INT1_PORT, ACC_INT1_BIT) = 0; // Set interrupt input pin as digital

    ACC_INT_IE     = 0;            // Disable interrupt
    //ACC_INT_MODULE = ACC_INT_RP;   // Map RP61 (Acc's INT1 pin) to dsPIC's INT2
    ACC_INT_EP     = 0;            // Interrupt on positive edge
    ACC_INT_IF     = 0;            // Clear flag
    ACC_INT_IE     = 1;            // Enable interrupt
}

uint16_t Acc_Read_Register(uint8_t regAddr, uint8_t isByte)
{
    uint8_t controlByte = 0b10000000 | (regAddr & 0b00111111);;
    uint16_t result;

    Acc_CS_Assert();
    SPI3_read_write_byte(controlByte);
    if (isByte == 1) {
        result = SPI3_read_write_byte(0x0);     // Gets value in 2nd read
    } else {
        result = SPI3_read_write_byte(0x0) << 8;
        result |= SPI3_read_write_byte(0x0);
    }
    Acc_CS_Disassert();
    
    return result;
}

void Acc_Write_Register(uint8_t regNum, uint8_t value)
{
    Acc_CS_Assert();
    SPI3_read_write_byte(regNum & 0b00111111);
    SPI3_read_write_byte(value);
    Acc_CS_Disassert();
}

inline uint8_t SPI3_read_write_byte(uint8_t data_byte)
{
    ACC_SPIBUF = data_byte;                // Write  data to be transmitted
    while(!ACC_SPI_STAT.SPIRBF);
    return ACC_SPIBUF;
}

inline void Acc_CS_Assert()
{
    ACC_CS_W = 0;
}

inline void Acc_CS_Disassert()
{
    ACC_CS_W = 1;
}

AccData* Acc_Get_Current_Data()
{
    return AccBufferIndex == 0 ? &AccDataBufferA : &AccDataBufferB;
}
