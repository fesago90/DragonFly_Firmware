#include "Pins.h"
#include "WiRaComm.h"
#include <string.h>

WRPacket WRTXPacket __attribute__((space(dma), eds));
WRPacket WRRXPacket __attribute__((space(dma), eds));

uint8_t WRTXRXJunkByte __attribute__((address(0xDFFE)));

unsigned int led = 0;
unsigned int rx_data[SPI_HEADER_LEN];

WRFlags wrflags;

void WR_Init_SPI();
void WR_Init_DMA();

void enable_dma_rx(unsigned int payload_length);
void enable_dma_tx(unsigned int payload_length);
void disable_dma_interrupts();
void enable_spi_interrupts();
void disable_spi_interrupts();

void _ISR_NO_PSV WR_SPI_INTERRUPT(void)
{
    // one byte already transferred when interrupt called
    static int count_byte = 1;
    WR_SPI_IF = 0;
    if (wrflags.header_rcvd == 1) {
        return;
    }
    led = 0;
    rx_data[count_byte-1] = WR_SPIBUF;
    if (count_byte == SPI_HEADER_LEN) {
        count_byte = 1;
        led = 1;
        wrflags.header_rcvd = 1;  // Two bytes received, which could be the header
        WR_SPIBUF = 0x55; // Send BUSY bytes until told otherwise
        return;
    }
    count_byte++;
    WR_SPIBUF = 0x16;//SPI_ACK_BYTE;
}

void _ISR_NO_PSV WR_DMA_TX_INTERRUPT()
{
    WR_DMA_TX_IF = 0;
    wrflags.dma_transfer_done = 1;
}

void _ISR_NO_PSV WR_DMA_RX_INTERRUPT()
{
    WR_DMA_RX_IF = 0;
    wrflags.dma_transfer_done = 1;
}

void disable_spi_interrupts()
{
    WR_SPI_IF = 0;
    WR_SPI_IE = 0;
}

void enable_spi_interrupts()
{
    // Clear the buffer overflow flag and flush the buffer
    WR_SPI_STAT.SPIROV  = 0;
    WR_SPI_IF = 0;
    WRTXRXJunkByte      = WR_SPIBUF;
    WR_SPIBUF = 0x17;
    WR_SPI_IE = 1;
}

void enable_dma_tx(unsigned int payload_length)
{
    // Clear the buffer overflow flag and flush the buffer
    WR_SPI_STAT.SPIROV  = 0;
    WR_SPI_IF           = 0;
    WRTXRXJunkByte      = WR_SPIBUF;
    // Setup DMA Rx to prevent SPIBUF overflows. Since the Master has initiated
    // transfer, it is implied acknowledgement. Packet integrity can be checked
    // using CRC.
    WR_RX_DMACON.MODE   = 0b01;     // Continuous, with ping-pong mode disabled
    WR_RX_DMACON.AMODE  = 0b01;     // Register Indirect without Post-Increment
    WR_RX_DMASTAL       = (unsigned int)&WRTXRXJunkByte;
    WR_RX_DMASTAH       = 0;
    WR_RX_DMACNT        = payload_length+1;
    WR_DMA_RX_IF        = 0;
    WR_DMA_RX_IE        = 0;
    // Set up Tx
    WR_TX_DMACON.DIR     = 1;                // Transmit here
    WR_TX_DMASTAL         = (unsigned int)&WRTXPacket;
    WR_TX_DMASTAH         = 0;
    WR_TX_DMACNT         = payload_length-1; // DMACNT+1 until transfer complete
    
    WR_DMA_TX_IF         = 0;
    WR_DMA_TX_IE         = 1;        // Enable DMA4 interrupt

    WR_RX_DMACON.CHEN   = 1;        // Enable DMA3 channel
    WR_TX_DMACON.CHEN    = 1;        // Enable DMA4 channel
}

void enable_dma_rx(unsigned int payload_length)
{
    WR_TX_DMACNT         = payload_length-1; // DMACNT+1 until transfer complete
    WR_TX_DMACON.DIR     = 0;                // Now receive here
    WR_TX_DMASTAL        = (unsigned int)&WRRXPacket+SPI_HEADER_LEN;
    WR_TX_DMASTAH        = 0;
    WR_DMA_TX_IE         = 1;        // Enable DMA4 interrupt
    WR_TX_DMACON.CHEN    = 1;        // Enable DMA4 channel
}

void disable_dma_interrupts()
{
    WR_DMA_TX_IE         = 0;        // Disable DMA4 interrupt
    WR_DMA_TX_IF         = 0;        // Clear DMA4 interrupt flag
    WR_TX_DMACON.CHEN    = 0;        // Disable DMA4 channel
    WR_DMA_RX_IE      = 0;        // Disable DMA3 interrupt
    WR_DMA_RX_IF      = 0;        // Clear DMA3 interrupt flag
    WR_RX_DMACON.CHEN = 0;        // Disable DMA3 channel
}

/*---------------------------------------------------------------------
 * Function Name: WR_Handle_Comms
 * Description:   Communication Handler Function. Runs as a thread in main
 *
 * Inputs:        None
 * Returns:       None
-----------------------------------------------------------------------*/

void WR_Handle_Comms(void)
{
    if (wrflags.header_rcvd) {
        if (wrflags.rx_packet_ready) {
            // New packet request received but
            // previous packet still not handled
            // TODO: Handle WRRXPacket Overflow
            return;
        }
        // Copy the header into the received packet buffer
        memcpy(&WRRXPacket, rx_data, SPI_HEADER_LEN);
        // Disable SPI interrupts to allow DMA to handle incoming/outgoing
        // packet
        disable_spi_interrupts();
        // Enable DMA Tx if payload length is zero, else enable DMA Rx
        if (rx_data[0] == 0x00) {
            if (wrflags.tx_packet_ready)
                enable_dma_tx(WRTXPacket.header.payload_length);
            else
                return;
            // Reset SPI module to clear FIFO buffers.
            // TODO: test for potential problems
            WR_SPI_STAT.SPIEN = 0;
            WR_SPI_STAT.SPIEN = 1;
            led = 0;
            WR_TX_DMAREQ.FORCE=1; // Force transfer of first byte
        }
        else {
            enable_dma_rx(((WRHeader *)&rx_data)->payload_length);
            // Reset SPI module to clear FIFO buffers.
            // TODO: test for potential problems
            WR_SPI_STAT.SPIEN = 0;
            WR_SPI_STAT.SPIEN = 1;
            WR_SPIBUF         = 0x00;
        }
        wrflags.DMA_Ready = 1;
        wrflags.header_rcvd = 0;
    }
    if (wrflags.dma_transfer_done) {
        // Transfer complete, re-enable everything.
        // Packet Handled in SD_PContinue()
        disable_dma_interrupts();
        if(!rx_data[0]) {
            while (!WR_SPI_STAT.SPIRBF) // IMP! Makes sure last transfer is complete.
            wrflags.tx_packet_ready = 0;
        }
        else
            wrflags.rx_packet_ready = 1;
        enable_spi_interrupts();
        wrflags.dma_transfer_done = 0;
    }
}

void WR_Init(void)
{
    WR_Init_SPI();
    WR_Init_DMA();
}

/*---------------------------------------------------------------------
 * Function Name: WR_Init_SPI
 * Description:   Initialize SPI2 module for communicating with Master
 *
 * Inputs:        None
 * Returns:       None
-----------------------------------------------------------------------*/
void WR_Init_SPI( void )
{
    _TRISbit(WR_CS_PORT, WR_CS_BIT)     = 1;                // C13 on updated schematic
    _TRISbit(WR_CLK_PORT, WR_CLK_BIT)   = 1;                // G6 to input
    _TRISbit(WR_MOSI_PORT, WR_MOSI_BIT) = 1;                // G7 to input
    _TRISbit(WR_MISO_PORT, WR_MISO_BIT) = 0;                // G8 to input

    _SS2R = WR_SS_RP;               // Map SS2 slave select to RC13 011 1101
    
    // Write default value to SPIBUF
    WR_SPIBUF = 0x17;//SPI_ACK_BYTE;

    WR_SPI_IF = 0;        // Clear the Interrupt Flag
    WR_SPI_IE = 0;        // Disable the Interrupt

    // SPIxCON1 Register Settings
    WR_SPI_CON1.DISSCK = 0;    // Internal Serial Clock is Enabled
    WR_SPI_CON1.DISSDO = 0;    // SDOx pin is controlled by the module
    WR_SPI_CON1.MODE16 = 0;    // Communication is byte-wide (8 bits)
    WR_SPI_CON1.SMP = 0;       // Input data is sampled at the middle of data output time

    WR_SPI_CON1.CKE = 1;       // Serial output data changes on clock transition from active to idle

    WR_SPI_CON1.CKP = 0;       // Idle state for clock is a low level;

    WR_SPI_CON1.MSTEN = 0;     // Master mode Disabled
    WR_SPI_STAT.SPIROV = 0;

    WR_SPI_STAT.SISEL = 0b011; // Interrupt when SPI receive buffer is full

    WR_SPI_CON1.SSEN = 1;	// Chip select (CS) controlled by module
    
    WR_SPI_STAT.SPIEN = 1;     // Enable SPI module
    
    WR_SPI_IF = 0;        // Clear the Interrupt Flag
    WR_SPI_IE = 1;        // Enable the Interrupt
}

void WR_Init_DMA(void)
{
    // Set up DMA4 channel to TX packet through SPI
    WR_DMA_TX_IE         = 0;        // Disable DMA4 interrupt
    WR_DMA_TX_IF         = 0;        // Clear DMA4 interrupt flag
    WR_TX_DMACON.CHEN    = 0;        // Disable DMA4 channel

    // DMA4 channel is connected to SPI2BUF
    WR_TX_DMAPAD         = (volatile unsigned int) &WR_SPIBUF;
    WR_TX_DMACON.SIZE    = 1;        // Byte data transfer size (not Word)
    WR_TX_DMACON.DIR     = 1;        // Read RAM, write to peripheral
    WR_TX_DMACON.AMODE   = 0b00;     // Register indirect with post-increment
    WR_TX_DMACON.MODE    = 0b01;     // One-shot, with ping-pong mode DISABLED
    WR_TX_DMAREQ.IRQSEL  = 0b00100001;    // SPI2 transfer done triggers DMA transfer

    WR_TX_DMASTAL         = (unsigned int)&WRTXPacket;
    WR_TX_DMASTAH         = 0;

    WR_TX_DMACNT         = 0;       // Will know from payload length
    WR_TX_DMACON.CHEN    = 0;        // Disable DMA4 channel

    WR_DMA_TX_IP          = 5;         // VERY IMPORTANT?

    // Set up DMA3 channel to RX packet through SPI
    WR_DMA_RX_IE      = 0;        // Disable DMA3 interrupt
    WR_DMA_RX_IF      = 0;        // Clear DMA3 interrupt flag
    WR_RX_DMACON.CHEN = 0;        // Disable DMA3 channel

    WR_RX_DMAPAD         = (volatile unsigned int) &WR_SPIBUF;   // DMA3 channel is connected to SPI2BUF
    WR_RX_DMACON.SIZE    = 1;        // Byte data transfer size (not Word)
    WR_RX_DMACON.DIR     = 0;        // Read Peripheral, write to RAM
    WR_RX_DMACON.AMODE   = 0b00;     // Register indirect with post-increment
    WR_RX_DMACON.MODE    = 0b01;     // Continuous, with ping-pong mode DISABLED
    WR_RX_DMAREQ.IRQSEL  = 0b00100001;    // SPI2 transfer done triggers DMA transfer

    WR_RX_DMASTAL        = (unsigned int)&WRRXPacket;
    WR_RX_DMASTAH        = 0;

    WR_DMA_RX_IP          = 5;         // VERY IMPORTANT?

    WR_RX_DMACNT         = 0 ;       // A 0 actually means 1 DMA transfers (1 byte each)
    WR_RX_DMACON.CHEN    = 0;        // Disable DMA3 channel
}

void* WR_Get_RX_Payload()
{
    return &WRRXPacket;
}

void* WR_Get_TX_Payload()
{
    return &WRTXPacket;
}
