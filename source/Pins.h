/* 
 * File:   Pins.h
 * Author: Felipe
 *
 * Created on January 24, 2013, 12:56 PM
 */

#ifndef PINS_H
#define	PINS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Common.h"

    // Peripheral pin definitions

#define LED1_PORT   D
#define LED1_BIT    5
#define LED1_W      _LATbit(LED1_PORT,LED1_BIT)

#define LED2_PORT   D
#define LED2_BIT    6
#define LED2_W      _LATbit(LED2_PORT,LED2_BIT)

#define LED3_PORT   D
#define LED3_BIT    7
#define LED3_W      _LATbit(LED3_PORT,LED3_BIT)

#define LED4_PORT   F
#define LED4_BIT    0
#define LED4_W      _LATbit(LED4_PORT,LED4_BIT)

    // PWM Generators
#define PWM_JP6_1A_PORT E
#define PWM_JP6_1A_BIT  0

#define PWM_JP6_1B_PORT E
#define PWM_JP6_1B_BIT  1

#define PWM_JP7_2A_PORT E
#define PWM_JP7_2A_BIT  3

#define PWM_JP7_2B_PORT E
#define PWM_JP7_2B_BIT  2

#define PWM_JP8_1A_PORT E
#define PWM_JP8_1A_BIT  4

#define PWM_JP8_1B_PORT E
#define PWM_JP8_1B_BIT  5

#define PWM_JP9_2B_PORT E
#define PWM_JP9_2B_BIT  6

#define PWM_JP9_2A_PORT E
#define PWM_JP9_2A_BIT  7

    /* ############# Digital Peripheral Pins #############*/
    // If modifying, remember that PPS configurations might change too

    // nRF24L01+
#define RF_CE_PORT      C
#define RF_CE_BIT       15
#define RF_CE_W         _LATbit(RF_CE_PORT, RF_CE_BIT)

#define RF_CS_PORT      C //D
#define RF_CS_BIT       13 //0
#define RF_CS_W         _LATbit(RF_CS_PORT, RF_CS_BIT)

#define RF_CLK_PORT     G
#define RF_CLK_BIT      6

#define RF_MISO_PORT    G
#define RF_MISO_BIT     7

#define RF_MOSI_PORT    G
#define RF_MOSI_BIT     8

#define RF_INT_PORT     G
#define RF_INT_BIT      9

        // WiRA pins
#define WR_CS_PORT      C    // D
#define WR_CS_BIT       13    // 0
//#define WR_CS_W         _LATbit(WR_CS_PORT, WR_CS_BIT)

#define WR_CLK_PORT     G
#define WR_CLK_BIT      6

#define WR_MISO_PORT    G
#define WR_MISO_BIT     8

#define WR_MOSI_PORT    G
#define WR_MOSI_BIT     7

// #define WR_INT_PORT     G
// #define WR_INT_BIT      9


    // Gyro L3G4200DTR
#define GY_CS_PORT      F
#define GY_CS_BIT       2
#define GY_CS_W         _LATbit(GY_CS_PORT, GY_CS_BIT)

#define GY_MISO_PORT    F
#define GY_MISO_BIT     3

#define GY_MOSI_PORT    F
#define GY_MOSI_BIT     4

#define GY_CLK_PORT     F
#define GY_CLK_BIT      5

#define GY_DR_PORT      D
#define GY_DR_BIT       8
#define GY_DR_R         _PORTbit(GY_DR_PORT, GY_DR_BIT)

    // Accel
#define ACC_CS_PORT     D
#define ACC_CS_BIT      1
#define ACC_CS_W        _LATbit(ACC_CS_PORT, ACC_CS_BIT)

#define ACC_MISO_PORT   D
#define ACC_MISO_BIT    2

#define ACC_MOSI_PORT   D
#define ACC_MOSI_BIT    4

#define ACC_CLK_PORT    D
#define ACC_CLK_BIT     3

#define ACC_INT1_PORT   D
#define ACC_INT1_BIT    0
#define ACC_INT1_R      _PORTbit(ACC_INT1_PORT, ACC_INT1_BIT)

    // RC Input
    
#define RC_IN1_PORT     B  // yaw
#define RC_IN1_BIT      12

#define RC_IN2_PORT     B  //pitch
#define RC_IN2_BIT      13

#define RC_IN3_PORT     B  //roll, switch to 14 on old RC
#define RC_IN3_BIT      15

#define RC_IN4_PORT     B  //throttle, switch to 15 on old RC
#define RC_IN4_BIT      14

    // Servo power good pins

#define SERV_PG1_PORT    D
#define SERV_PG1_BIT     9

#define SERV_PG2_PORT    D
#define SERV_PG2_BIT     10

    // Camera pins

#define CAM_XVCLK_PORT   F
#define CAM_XVCLK_BIT    6 //1 //6

#define CAM_VSYNC_PORT   D //B //D
#define CAM_VSYNC_BIT    11 //6 //11

#define CAM_PCLK_PORT    D
#define CAM_PCLK_BIT     9 //0 //9

#define CAM_SIOC_PORT    G
#define CAM_SIOC_BIT     2

#define CAM_SIOD_PORT    G
#define CAM_SIOD_BIT     3

#define CAM_PWDN_PORT    C //D   // C 14 for new board, B 7 for old
#define CAM_PWDN_BIT     14 //14 //7 //10

#define CAM_DATA_7_PORT  B
#define CAM_DATA_7_BIT   11

#define CAM_DATA_6_PORT  B
#define CAM_DATA_6_BIT   10

#define CAM_DATA_5_PORT  B
#define CAM_DATA_5_BIT   9

#define CAM_DATA_4_PORT  B
#define CAM_DATA_4_BIT   8

#define CAM_DATA_3_PORT  B
#define CAM_DATA_3_BIT   7

#define CAM_DATA_2_PORT  B
#define CAM_DATA_2_BIT   6

#define CAM_DATA_1_PORT  B
#define CAM_DATA_1_BIT   5

#define CAM_DATA_0_PORT  B
#define CAM_DATA_0_BIT   4

    /* ############# Analog Pins #############*/

#define ADC_SERVO_PORT  B
#define ADC_SERVO_BIT   3

#define ADC_3V_PORT     B
#define ADC_3V_BIT      4

#define ADC_BATT_PORT   B
#define ADC_BATT_BIT    2 //5 on old board

    
#ifdef	__cplusplus
}
#endif

#endif	/* PINS_H */

