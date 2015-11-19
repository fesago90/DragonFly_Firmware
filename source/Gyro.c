#include "Pins.h"
#include "Gyro.h"


GyroData GyroDataBuffer[2] __attribute__((space(dma), eds)) =
{{0xaa, 0xaaaa, 0xaaaa, 0xaaaa},{0xaa, 0xaaaa, 0xaaaa, 0xaaaa}};

static uint8_t GyroBufferIndex = 0;

void        Gyro_Init_SPI();
void        Gyro_Init_DMA();
void        Gyro_Init_Interrupt();
void        Gyro_Check_ID();
void        Gyro_Write_Register(uint8_t regAddr, uint8_t value);
uint16_t    Gyro_Read_Register(uint8_t regAddr, uint8_t isByte);
inline void Gyro_CS_Assert();
inline void Gyro_CS_Disassert();
//inline void Gyro_Trigger_Read_Data();
inline void Gyro_Data_Received();

inline uint8_t SPI1_read_write_byte(uint8_t data_byte);

/*
 * This ISR is executed when the Gyro's DRDY pin goes high, indicating
 * that a new sample is ready.
 */
void _ISR_NO_PSV GYRO_INT_INTERRUPT()
{

    GYRO_INT_IF = 0;
    Gyro_Trigger_Read_Data();
}

/* This interrupt is triggered when SPI1 received a full block of data (7 bytes)
 * and the DMA has copied it to RAM
 */
void _ISR_NO_PSV GYRO_DMA_INTERRUPT()
{
    GYRO_DMA_IF = 0;
    Gyro_Data_Received();

}

void Gyro_Init()
{
    #if (GYRO_EN == 1)
        Gyro_Init_SPI();
        Gyro_Check_ID();
        Gyro_Init_Interrupt();

        // Todo: Select appropriate range, LPF and HPF parameters
        Gyro_Write_Register(0x20, 0b00000111);  // Power-down mode
        Gyro_Write_Register(0x22, 0b00000000);  // Disable data-ready interrupt
        Gyro_Write_Register(0x21, 0b00000000);  // HPF cut-off = 56Hz @ ODR = 800 Hz (if enabled)
        Gyro_Write_Register(0x22, 0b00001000);  // Enable data-ready interrupt
        Gyro_Write_Register(0x23, 0b00100000);  // Continuous data update, 2000dps range, Big-endian
        Gyro_Write_Register(0x24, 0b00000000);  // Disable FIFO, disable HPF
        Gyro_Write_Register(0x20, 0b11111111);  // Power up, ODR = 800Hz Cut-off=110

        Gyro_Init_DMA();
    #endif
}

void Gyro_Init_SPI()
{
//    _ANSELbit(GY_CS_PORT, GY_CS_BIT)     = 0;    // CS pin is digital
//    _ANSELbit(GY_CLK_PORT, GY_CLK_BIT)   = 0;    // CLK pin is digital
//    _ANSELbit(GY_MOSI_PORT, GY_MOSI_BIT) = 0;    // MOSI pin is digital
//    _ANSELbit(GY_MISO_PORT, GY_MISO_BIT) = 0;    // MISO pin is digital

    _TRISbit(GY_CS_PORT, GY_CS_BIT)     = 0;    // CS pin is output
    _TRISbit(GY_CLK_PORT, GY_CLK_BIT)   = 0;    // CLK pin is output
    _TRISbit(GY_MOSI_PORT, GY_MOSI_BIT) = 0;    // MOSI pin is output
    _TRISbit(GY_MISO_PORT, GY_MISO_BIT) = 1;    // MISO pin is input

    Gyro_CS_Disassert();

    GYRO_SCK_RP = SCK1;         // Map SCLK to RP101
    GYRO_SDO_RP = SDO1;         // Map MOSI to RP100
    SDI1        = GYRO_SDI_RP;  // Map MISO to RP99

    GYRO_SPI_IF = 0;                // Clear the Interrupt Flag
    GYRO_SPI_IE = 0;                // Disable the Interrupt

    GYRO_SPI_CON1.DISSCK  = 0;   // Internal Serial Clock is Enabled
    GYRO_SPI_CON1.DISSDO  = 0;   // SDOx pin is controlled by the module
    GYRO_SPI_CON1.MODE16  = 0;   // Communication is byte-wide (8 bits)
    GYRO_SPI_CON1.MSTEN   = 1;   // Master mode Enabled
    GYRO_SPI_CON1.SSEN    = 0;   // SSN is GPIO
    GYRO_SPI_STAT.SPISIDL = 0;   // Continue operation in Idle mode

    GYRO_SPI_CON1.CKE = 0;       // Serial output data changes on clock transition from idle to active
    GYRO_SPI_CON1.CKP = 1;       // Idle state for clock is a logic HIGH level
    GYRO_SPI_CON1.SMP = 0;       // Input data is sampled at middle of data output time
    GYRO_SPI_CON1.SPRE = 0b111;  // Secondary prescale 1:1
    GYRO_SPI_CON1.PPRE = 0b01;   // Primary prescale 16:1
    //GYRO_SPI_CON1.SPRE = 0b001;  // Secondary prescale 7:1
    //GYRO_SPI_CON1.PPRE = 0b11;   // Primary prescale 1:1

    GYRO_SPI_STAT.SPIEN = 1;     // Enable SPI module

    GYRO_SPI_IF = 0;        // Clear the Interrupt Flag
    GYRO_SPI_IE = 0;        // Disable the Interrupt
}

/*
 * DMA1 channel is initialized to read data from SPI1 whenever a byte is received
 * Null data peripheral write is enabled so that it automatically triggers SPI
 * activity to read all 7-bytes of the gyro data transaction. The
 * transaction consists of 1 command byte and 6 data bytes
 */
void Gyro_Init_DMA()
{
    // Set up DMA1 channel to RX gyro data through SPI
    GYRO_DMA_IE         = 0;        // Disable DMA1 interrupt
    GYRO_DMA_IF         = 0;        // Clear DMA1 interrupt flag
    GYRO_DMAPAD         = 0x0248;   // DMA1 channel is connected to SPI1BUF
    GYRO_DMACON.SIZE    = 1;        // Byte data transfer size (not Word)
    GYRO_DMACON.DIR     = 0;        // Read from peripheral, write to RAM
    GYRO_DMACON.AMODE   = 0b00;     // Register indirect with post-increment
    GYRO_DMACON.MODE    = 0b01;     // One-shot, with NO ping-pong mode
    GYRO_DMACON.NULLW   = 1;        // Null data writes generated to peripheral
    GYRO_DMAREQ.IRQSEL  = 0b00001010;    // SPI1 transfer done triggers DMA transfer

    GYRO_DMASTAL        = __builtin_dmaoffset(&GyroDataBuffer[0]) + 1;
    GYRO_DMASTAH        = 0;

    GYRO_DMACNT         = 6;        // A 6 actually means 7 DMA transfers (1 byte each)
    GYRO_DMACON.CHEN    = 1;        // Enable DMA1 channel
    GYRO_DMA_IE         = 1;        // Enable DMA1 interrupt
}

void Gyro_Check_ID()
{
    uint8_t devId;
    int errCount = 0;

    devId = Gyro_Read_Register(0xF, 1);       // get device ID

    while (devId != 0b11010011) {               // datasheet says it should be 0xD3
        __delay_us(100);
        if (++errCount == 10)                   // Attempt reading up to 10 times
            Display_Error(ErrGyro);
        devId = Gyro_Read_Register(0x0, 1);
    }
}

uint16_t Gyro_Read_Register(uint8_t regAddr, uint8_t isByte)
{
    uint8_t controlByte = 0b10000000 | (regAddr & 0b00111111);;
    uint16_t result;

    Gyro_CS_Assert();
    SPI1_read_write_byte(controlByte);
    if (isByte == 1) {
        result = SPI1_read_write_byte(0x0);     // Gets value in 2nd read
    } else {
        result = SPI1_read_write_byte(0x0) << 8;
        result |= SPI1_read_write_byte(0x0);
    }
    Gyro_CS_Disassert();

    return result;
}

void Gyro_Write_Register(uint8_t regAddr, uint8_t value)
{
    Gyro_CS_Assert();
    SPI1_read_write_byte(regAddr & 0b00111111);
    SPI1_read_write_byte(value);
    Gyro_CS_Disassert();
}

inline uint8_t SPI1_read_write_byte(uint8_t data_byte)
{
    GYRO_SPIBUF = data_byte;                // Write  data to be transmitted
    while(!GYRO_SPI_STAT.SPIRBF);
    return GYRO_SPIBUF;
}

inline void Gyro_CS_Assert()
{
    GY_CS_W = 0;
}

inline void Gyro_CS_Disassert()
{
    GY_CS_W = 1;
}

/*
 * Enables the dsPIC's INT1 interrupt on the pin that is connected to
 * the gyro's DR interrupt pin. Positive edge triggered.
 */
void Gyro_Init_Interrupt()
{
    _TRISbit(GY_DR_PORT, GY_DR_BIT) = 1;    // Set Data-Ready pin as input
    //_ANSELbit(GY_DR_PORT, GY_DR_BIT) = 0;    // Set Data-Ready pin as digital

    GYRO_INT_IE     = 0;            // Disable interrupt
    GYRO_INT_MODULE = GYRO_INT_RP;  // Map RPI72 (Gyro Data-Ready) to dsPIC's INT1
    GYRO_INT_EP     = 0;            // Interrupt on positive edge
    GYRO_INT_IF     = 0;            // Clear interrupt flag
    GYRO_INT_IE     = 1;            // Enable interrupt
}

/*
 * This function places a read OUT_X_L command with multibyte support in SPI3BUF
 * to trigger a transaction. DMA1 takes care of retrieving the data into RAM and
 * starting the 6 extra SPI transactions. With multi-byte option, the gyro
 * sends the values of registers OUT_X_H to OUT_Z_H in the following SPI transactions
 */
inline void Gyro_Trigger_Read_Data()
{
    GYRO_DMACON.CHEN = 1;   // Re-enable DMA1 (one-shot mode turns it off)
    Gyro_CS_Assert();       // Assert SPI channel
    GYRO_SPIBUF = 0b11101000;   // Read register 0x28 (OUT_X_L) w/ Multi-Byte
}

inline void Gyro_Data_Received()
{
    Gyro_CS_Disassert();
    if (GyroBufferIndex == 0)
        GYRO_DMASTAL    = (uint16_t)&GyroDataBuffer[1] + 1;
    else
        GYRO_DMASTAL    = (uint16_t)&GyroDataBuffer[0] + 1;
    GyroBufferIndex ^= 1;
}

/*
 * Returns the current gyroscope reading.
 */
GyroData* Gyro_Get_Current_Data()
{
    asm("push	DSRPAG");
    asm("movpag #0x0001, DSRPAG");

    GyroData *ptr = &GyroDataBuffer[GyroBufferIndex];

    asm("pop    DSRPAG");
    return ptr;
}
