/* 
 * File:   RFComm.h
 * Author: Felipe
 *
 * Created on January 29, 2013, 3:05 PM
 */

#ifndef RFCOMM_H
#define	RFCOMM_H

#ifdef	__cplusplus
extern "C" {
#endif

//    typedef enum {
//        RFDataReady     = 1,
//        RFLastTXFailed  = 2,
//        RFReadyForTX    = 4
//    } RFStatus;

    typedef struct {
        union {
            uint8_t bytes[32];
            uint16_t words[16];
        };
    } RFPayload;

    void RF_Init();
    void RF_Transmit(char *buf_ptr);
    void RF_Receive_Data();
    void RF_Try_Start_Packet_TX();
    
    RFPayload* RF_Get_TX_Payload();
    RFPayload* RF_Get_RX_Payload();

    extern volatile uint8_t RFDataReady;
    extern volatile uint8_t RFLastTXFailed;
    extern volatile uint8_t RFReadyForTX;


#ifdef	__cplusplus
}
#endif

#endif	/* RFCOMM_H */

