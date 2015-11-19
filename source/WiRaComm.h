/* 
 * File:   WiRaComm.h
 * Author: Parav
 *
 * Created on May 12, 2015, 4:22 PM
 */

#ifndef WIRACOMM_H
#define	WIRACOMM_H

#define SPI_NACK_BYTE 0xFF
#define SPI_ACK_BYTE 0x00
#define SPI_MTU 255
#define SPI_HEADER_LEN 2
#define SPI_MAX_DATA_LEN (SPI_MTU-SPI_HEADER_LEN)

#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct __attribute__((packed)) {
        uint8_t payload_length;
        uint8_t payload_type;
    } WRHeader;

    typedef struct __attribute__((packed)) {
        uint8_t bytes[SPI_MAX_DATA_LEN];
    } WRData;

    typedef struct {
        union {
            uint8_t bytes[SPI_MTU];
            struct __attribute__((packed)) {
                WRHeader header;
                WRData data;
            };
        };
    } WRPacket;

    typedef struct{
        unsigned int header_rcvd;
        unsigned int DMA_Ready; // Enabled when data is ready to transmitted/recvd
        unsigned int dma_transfer_done;
        unsigned int rx_packet_ready; // Set when rx packet ready to be processed
        unsigned int tx_packet_ready; // Set when tx packet built/ready to be sent
    } WRFlags;

    void WR_Init();
    void WR_Transmit(char *buf_ptr);
    void WR_Receive_Data();
    void WR_Try_Start_Packet_TX();
    void WR_Handle_Comms();
    void* WR_Get_RX_Payload();
    void* WR_Get_TX_Payload();
    
    //RFPayload* WR_Get_TX_Payload();
    //RFPayload* WR_Get_RX_Payload();

    extern unsigned int led;
    extern WRFlags wrflags;


#ifdef	__cplusplus
}
#endif

#endif	/* WIRACOMM_H */

