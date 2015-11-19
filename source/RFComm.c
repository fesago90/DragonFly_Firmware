#include "Pins.h"
#include "RFComm.h"
#include "LED.h"

RFPayload RFTXPayload __attribute__((space(dma), eds));
RFPayload RFRXPayload __attribute__((space(dma), eds));
volatile uint8_t RFDataReady;


//uint8_t RFTXRXJunkByte __attribute__((space(dma), eds));
uint8_t RFTXRXJunkByte __attribute__((address(0xDFFE)));


uint8_t RFRTCount = 0;

volatile uint8_t RFReadyForTX = 0;
volatile uint8_t RFLastTXFailed = 0;

void RF_Init_SPI();
void RF_Init_nRF();

void RF_Config_TX();
void RF_Config_RX();

void RF_Init_Interrupt();
void RF_Init_DMA();

void retransmit(void);
char SPI2_read_write_byte(char data_byte);
void receive_data(uint8_t *buf_ptr);
char read_status_flag( void );
void clear_status_flag(char value);
void flush_rx_fifo(void);
void flush_tx_fifo(void);
char is_data_received(void);
void int_to_payload_buffer_actual_values(char* buffer, const int * int_array);
void payload_buffer_to_int_actual_values(const char* buffer, int * int_array);
void send_ready_to_MATLAB(char * buffer);
void send_finished_to_MATLAB(char * h_buffer);
void int_to_payload_buffer_MATLAB(char* buffer, const unsigned int * int_array, char array_size, char endline);
void int_to_payload_buffer(char* buffer, const unsigned int * int_array, char array_size, char option123);
void payload_buffer_to_dir(char* buffer, char * dir);
void flush_buffer(char * buff);

inline void RF_CS_Assert();
inline void RF_CS_Disassert();

inline void RF_CE_Assert();
inline void RF_CE_Disassert();

/*
 * This handles the nRF's IRQ pin
 */
void _ISR_NO_PSV NRF_INT_INTERRUPT()
{
    NRF_INT_IF = 0;
    RF_CE_Disassert();

    NRF_BUF_DMACON.CHEN = 0;   // Stop DMA3 to prevent SPI-to-junk-RAM transfers

    if (RFReadyForTX == 0) {

        uint8_t curstatus = read_status_flag();

        if (curstatus & 0x40) {             // Data received interrupt
            RFDataReady = 1;
            RFRTCount++;
            RFRTCount--;
        }

        if (curstatus & 0x20) {             // Data sent interrupt
            RFLastTXFailed = 0;
            RFRTCount++;
            RFRTCount--;
        }

        if (curstatus & 0x10) {            // Max retransmission interrupt
            RFLastTXFailed = 1;
            RFRTCount++;
            RFRTCount--;
        }

        clear_status_flag(0xf0);

        RFReadyForTX = 1;
    }
}

void _ISR_NO_PSV NRF_DMA_INTERRUPT()
{
    NRF_DMA_IF = 0;
    RF_CS_Disassert();
    //SRbits.IPL = 4;
    RF_CE_Assert(); // Start RF transmission
}

void RF_Init(void)
{
    RF_Init_SPI();
    RF_Init_nRF();
    RF_Config_TX();
    RF_Init_Interrupt();
    RF_Init_DMA();

    RFReadyForTX = 1;
}

/*---------------------------------------------------------------------
  Function Name: RF_Init_SPI
  Description:   Initialize SPI2 module for communicating with nRF24L01+
                 SPI2 clock is set at Fcy / (2*4) = 60MHz/16
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void RF_Init_SPI( void )
{
    _TRISbit(RF_CS_PORT, RF_CS_BIT)     = 0;
    _TRISbit(RF_CE_PORT, RF_CE_BIT)     = 0;
    _TRISbit(RF_CLK_PORT, RF_CLK_BIT)   = 0;
    _TRISbit(RF_MOSI_PORT, RF_MOSI_BIT) = 0;
    _TRISbit(RF_MISO_PORT, RF_MISO_BIT) = 1;

    //_ANSELbit(RF_CS_PORT, RF_CS_BIT)     = 0;
    //_ANSELbit(RF_CE_PORT, RF_CE_BIT)     = 0;
    _ANSELbit(RF_CLK_PORT, RF_CLK_BIT)   = 0;
    _ANSELbit(RF_MOSI_PORT, RF_MOSI_BIT) = 0;
    _ANSELbit(RF_MISO_PORT, RF_MISO_BIT) = 0;

    RF_CS_Disassert();
    RF_CE_Disassert();

    NRF_SPI_IF = 0;        // Clear the Interrupt Flag
    NRF_SPI_IE = 0;        // Disable the Interrupt

    // SPIxCON1 Register Settings
    NRF_SPI_CON1.DISSCK = 0;    // Internal Serial Clock is Enabled
    NRF_SPI_CON1.DISSDO = 0;    // SDOx pin is controlled by the module
    NRF_SPI_CON1.MODE16 = 0;    // Communication is byte-wide (8 bits)
    NRF_SPI_CON1.MSTEN = 1;     // Master mode Enabled

    NRF_SPI_CON1.CKP = 0;       // Idle state for clock is a low level;
    NRF_SPI_CON1.CKE = 1;       // Serial output data changes on clock transition from active to idle
    NRF_SPI_CON1.SMP = 0;       // Input data is sampled at the middle of data output time
    NRF_SPI_CON1.SPRE = 0b110;  // Secondary prescale 2:1
    NRF_SPI_CON1.PPRE = 0b10;   // Primary prescale 4:1
    // SPI settings for nRF verified 1/28/13

    NRF_SPI_CON1.SSEN = 0;	// Chip select (CS) controlled by port, not module
    NRF_SPI_STAT.SPIEN = 1;     // Enable SPI module

    NRF_SPI_IF = 0;        // Clear the Interrupt Flag
    NRF_SPI_IE = 0;        // Disable the Interrupt

}

void RF_Init_Interrupt()
{
    _TRISbit(RF_INT_PORT, RF_INT_BIT) = 1;
    _ANSELbit(RF_INT_PORT, RF_INT_BIT) = 0;

    NRF_INT_MODULE         = NRF_INT_RP;  // Map RP121 (nRF's IRQ pin) to INT3 input

    NRF_INT_IE             = 0;    // Disable interrupt
    NRF_INT_IF             = 0;    // Clear interrupt flag
    NRF_INT_EP             = 1;    // Interrupt on negative edge
    NRF_INT_IE             = 1;    // Enable interrupt
}

void RF_Init_DMA()
{
    // Set up DMA4 channel to TX packet through SPI
    NRF_DMA_IE         = 0;        // Disable DMA4 interrupt
    NRF_DMA_IF         = 0;        // Clear DMA4 interrupt flag
    NRF_DMACON.CHEN    = 0;        // Disable DMA4 channel

    NRF_DMAPAD         = 0x0268;   // DMA4 channel is connected to SPI2BUF
    NRF_DMACON.SIZE    = 1;        // Byte data transfer size (not Word)
    NRF_DMACON.DIR     = 1;        // Read RAM, write to peripheral
    NRF_DMACON.AMODE   = 0b00;     // Register indirect with post-increment
    NRF_DMACON.MODE    = 0b01;     // One-shot, with ping-pong mode DISABLED
    NRF_DMAREQ.IRQSEL  = 0b00100001;    // SPI2 transfer done triggers DMA transfer

    NRF_DMASTAL         = (unsigned int)&RFTXPayload;
    NRF_DMASTAH         = 0;

    NRF_DMACNT         = 32;       //
    NRF_DMACON.CHEN    = 0;        // Disable DMA4 channel

    NRF_DMA_IP          = 5;         // VERY IMPORTANT
    NRF_DMA_IE          = 1;        // Enable DMA4 interrupt

    // Set up DMA3 channel to prevent RX overflows when performing payload TX
    NRF_DMA_BUF_IE      = 0;        // Disable DMA3 interrupt
    NRF_DMA_BUF_IF      = 0;        // Clear DMA3 interrupt flag
    NRF_BUF_DMACON.CHEN = 0;        // Disable DMA3 channel

    NRF_BUF_DMAPAD         = 0x0268;   // DMA3 channel is connected to SPI2BUF
    NRF_BUF_DMACON.SIZE    = 1;        // Byte data transfer size (not Word)
    NRF_BUF_DMACON.DIR     = 0;        // Read Peripheral, write to RAM
    NRF_BUF_DMACON.AMODE   = 0b00;     // Register indirect with post-increment
    NRF_BUF_DMACON.MODE    = 0b00;     // Continuous, with ping-pong mode DISABLED
    NRF_BUF_DMAREQ.IRQSEL  = 0b00100001;    // SPI2 transfer done triggers DMA transfer

    NRF_BUF_DMASTAL        = (unsigned int)&RFTXRXJunkByte;
    NRF_BUF_DMASTAH        = 0;

    NRF_BUF_DMACNT         = 0 ;       // A 0 actually means 1 DMA transfers (1 byte each)
    NRF_DMA_BUF_IE         = 0;        // Disable DMA3 interrupt
    NRF_BUF_DMACON.CHEN    = 0;        // Disable DMA3 channel
}

void RF_Try_Start_Packet_TX()
{
    if ( !RFReadyForTX )
        return;

    if ( RFLastTXFailed ) {
        flush_tx_fifo();
        RFLastTXFailed = 0;
    }

    RFReadyForTX = 0;

    NRF_DMACON.CHEN = 1;
    NRF_BUF_DMACON.CHEN = 1;

    RF_CS_Assert();
    NRF_SPIBUF = 0b10100000;
    //SPI2_read_write_byte();   //W_TX_PAYLOAD
}

RFPayload* RF_Get_TX_Payload()
{
    return &RFTXPayload;
}

RFPayload* RF_Get_RX_Payload()
{
    return &RFRXPayload;
}

void RF_Init_nRF()
{
    RF_CS_Disassert();
    RF_CE_Disassert();

    __delay_ms(100);

    RF_CS_Assert();
    SPI2_read_write_byte(0x20);
    SPI2_read_write_byte(0x09); // Power down
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x21); // Enable auto-ack on data pipes 0 & 1
    SPI2_read_write_byte(0x03);
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x22); // Enable pipes 0 and 1
    SPI2_read_write_byte(0x03);
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x23); // 3-byte RX/TX address field width
    SPI2_read_write_byte(0x01);
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x24); 
    SPI2_read_write_byte(0x15); // Wait 500us, retransmit up to 5 times on auto-ack failures ||used to be 0x15
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x25);
    SPI2_read_write_byte(0x02); // Use channel 2 frequency
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x26);
    SPI2_read_write_byte(0x0E); // 2mbps, 0dBm RF output power
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x27);
    SPI2_read_write_byte(0x70); // Clear all interrupts
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x31);
    SPI2_read_write_byte(32);   // 32-byte RX payload width on pipe 0
    RF_CS_Disassert();

    /*RF_CS_Assert();
    SPI2_read_write_byte(0x32);
    SPI2_read_write_byte(32);   // 32-byte RX payload width on pipe 1
    RF_CS_Disassert();*/

    RF_CS_Assert();
    SPI2_read_write_byte(0x3C);
    SPI2_read_write_byte(0x01);   // Enable dynamic payload length data pipe 0
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x3D);
    SPI2_read_write_byte(0x06);   // Enable dynamic payload length & ack payload
    RF_CS_Disassert();

    RF_CS_Assert();
    SPI2_read_write_byte(0x2A);
    SPI2_read_write_byte(0x71); // Set RX address of pipe 0 same as TX address for auto-ack feature
    SPI2_read_write_byte(0x71);
    SPI2_read_write_byte(0x71);
    RF_CS_Disassert();

//    RF_CS_Assert();
//    SPI2_read_write_byte(0x2B);
//    SPI2_read_write_byte(0x8E); // Set RX address of pipe 1 (this is the dragonfly's address)
//    SPI2_read_write_byte(0x8E);
//    SPI2_read_write_byte(0x8E);
//    RF_CS_Disassert();
    
    RF_CS_Assert();
    SPI2_read_write_byte(0x30);
    SPI2_read_write_byte(0x71); // Set TX address (this is the computer's address)
    SPI2_read_write_byte(0x71);
    SPI2_read_write_byte(0x71);
    RF_CS_Disassert();

    flush_rx_fifo();
    flush_tx_fifo();

    RF_CS_Assert();
    SPI2_read_write_byte(0x20); // Power UP, all interrupts enabled, 2-byte CRC, PTX
    SPI2_read_write_byte(0x0E);
    RF_CS_Disassert();

    __delay_ms(5);
    
    flush_rx_fifo();
    flush_tx_fifo();
}

void RF_Config_TX(void)
{
    RF_CE_Disassert();
    RF_CS_Assert();
    SPI2_read_write_byte(0x20);
    SPI2_read_write_byte(0x0E); // All interrupts used, CRC enabled, CRC encoding 2 byte, PTX mode
    
    RF_CS_Disassert();
}

void RF_Config_RX(void)
{
    RF_CS_Assert();
    SPI2_read_write_byte(0x20); //write to config
    SPI2_read_write_byte(0x0F); // All interrupts used, CRC enabled, CRC encoding 2 byte, PRX mode
    RF_CS_Disassert();

    RF_CE_Assert();
}

void RF_Transmit(char *buf_ptr)
{
    //static uint16_t lastSeq = 123;

    if (RFReadyForTX != 1)
        return;
    
    RFReadyForTX = 0;

    if ( !RFLastTXFailed ) {

        /*if (RFTXPayload[15] == lastSeq)
        {
            RFRTCount++;
            RFRTCount--;
        }

        lastSeq = RFTXPayload[15];*/


        int i;
        RF_CS_Assert();

        SPI2_read_write_byte(0b10100000);   //W_TX_PAYLOAD
        
        for(i = 0; i<32; i++) {
                SPI2_read_write_byte(*buf_ptr++);
        }
        
        RF_CS_Disassert();
    }

    RF_CE_Assert();
    
}

void retransmit(void)
{
	clear_status_flag(0x20);
	clear_status_flag(0x10);

	RF_CE_Assert();
	__delay_us(15);
	RF_CE_Disassert();

	while((!(read_status_flag() & 0x20)) && (!(read_status_flag() & 0x10)));
}

char SPI2_read_write_byte(char data_byte)
{
	NRF_SPIBUF = data_byte; // Write data to be transmitted
	while(!NRF_SPI_STAT.SPIRBF);
	return NRF_SPIBUF;
}

void RF_Receive_Data()
{
    uint8_t *buf_ptr = (uint8_t*)&RFRXPayload;
    int i = 0;

    flush_buffer((char*)buf_ptr);

    RF_CS_Assert();
    SPI2_read_write_byte(0x61); // R_RX_PAYLOAD -> read payload
    for(i = 0; i<32; i++)
    {
        buf_ptr[i] = SPI2_read_write_byte(0xFF);
    }
    RF_CS_Disassert();
    
    RFDataReady = 0;
}

char read_status_flag( void )
{
	char status;

	RF_CS_Assert();
	status = SPI2_read_write_byte(0xff); //NOP, to read status register
	RF_CS_Disassert();

        RFRTCount++;
        RFRTCount--;
        
	return status;
}

void clear_status_flag(char value)
{
	RF_CS_Assert();
	SPI2_read_write_byte(0x27);
	SPI2_read_write_byte(value); // clear flag
	RF_CS_Disassert();
}

void flush_rx_fifo(void)
{
	RF_CS_Assert();
	SPI2_read_write_byte(0xe2);
	RF_CS_Disassert();
}

void flush_tx_fifo(void)
{
	RF_CS_Assert();
	SPI2_read_write_byte(0xe1);
	RF_CS_Disassert();
}

void flush_buffer(char * buff)
{
	int i;
	for(i=0; i<32+1; i++)
	{
		buff[i] = 0;
	}
}

inline void RF_CS_Assert()
{
    RF_CS_W = 0;
}

inline void RF_CS_Disassert()
{
    RF_CS_W = 1;
}

inline void RF_CE_Assert()
{
    RF_CE_W = 1;
}

inline void RF_CE_Disassert()
{
    RF_CE_W = 0;
}
