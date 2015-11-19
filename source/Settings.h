#ifndef SETTINGS_H
#define	SETTINGS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "PeripheralSelect.h"

#define TAN_BOARDS

// Dragonfly Controls
#define DRAGONFLY
#define DRONE
// Defines motor output--DC/PWM signal or servos (1-2ms pulse)
#define SERVOS

#ifdef DRAGONFLY
// Inside servo: FrontLeft, RearRight.
#define FRONTLEFT_SERVO_MAX    3700
#define FRONTLEFT_SERVO_MIN    1600
#define REARRIGHT_SERVO_MAX    3700
#define REARRIGHT_SERVO_MIN    1600
// Outside servo:  FrontRight, RearLeft.
#define FRONTRIGHT_SERVO_MIN   3650
#define FRONTRIGHT_SERVO_MAX   1550
#define REARLEFT_SERVO_MIN     3650
#define REARLEFT_SERVO_MAX     1550

// servo midpoint
#define SERVO_MID   2812
#define SERVO_MIN   1875
#define SERVO_MAX   3750
#endif

#define SYSTEM_FREQUENCY_IN_MHZ 60

/* ####### Flight Control Settings ####### */
#define FC_EN 1 // doesn't do anything yet
#define MAX_ANGLE_REFERENCE 1500
#define MAX_ANGULAR_VELOCITY_REFERENCE 4500
#define FC_LOOP_TIME_IN_US 1000

#define LOOP_TIMER 5

// dragonfly pid gains
#define ROLL_Kp     0.0f
#define ROLL_Ki     0
#define ROLL_Kd     0
#define PITCH_Kp    0
#define PITCH_Ki    0
#define PITCH_Kd    0
#define YAW_Kp      0
#define YAW_Ki      0
#define YAW_Kd      0

/* ####### I2C Settings ####### */
#define I2C_EN 1 // doesn't do anything yet
#define TIME_OUT_ERROR_CODE 72
#define BAD_ACK_ERROR_CODE 73
#define TIME_TO_WAIT 100

/* ####### Battery Settings ####### */
#define VMEAS_EN 1
#define MINIMUM_SAFE_BATTERY_VOLTAGE 3.1
#define GOOD_BATTERY_VOLTAGE 4.2

#define VMEAS_DMA   2
#define VMEAS_ADC   1
#define VMEAS_TIMER 3

/* ####### Accelerometer Settings ####### */
#define ACC_EN 1

#define ACC_INT 0
#define ACC_DMA 0
#define ACC_SPI 3

/* #######Gyroscope Settings ####### */
#define GYRO_EN 1

#define GYRO_INT 1
#define GYRO_DMA 1
#define GYRO_SPI 1

/* ####### RC Settings ####### */
#define RC_EN 1

#define THROTTLE_IC   4
#define ROLL_TRIM_IC  3         
#define PITCH_TRIM_IC 2
#define YAW_TRIM_IC   1         

/* ####### Motor Settings ####### */
#define MOTOR_EN 1
#define FLAP_FREQ_IN_HZ 60

#define FLAP_TIMER 4

/* ####### Camera Settings ####### */
#define CAM_EN 0
#define XVCLK_DIVIDER 5
#define PCLK_FREQ_IN_MHZ (1)
#define FRAME_ROWS 1   //90
#define FRAME_COLS 1    //120
#define SHORT_DELAY_IN_US 3
#define LONG_DELAY_IN_US 1400

#define VSYNC_INT    4
#define HREF_INT
#define PCLK_INT     2
#define XVCLK_OC     1
#define SHORT_HREF_TIMER 6
#define LONG_HREF_TIMER  7

/* ####### nRF Settings ####### */
#define NRF_EN 1 // doesn't do anything yet
#define NRF_INT 3
#define NRF_DMA 4
#define NRF_SPI 2
#define NRF_DMA_BUF 3

/* ####### WiRA Settings ###### */

#define WR_EN 1
#define WR_INT 3
#define WR_DMA_TX 4
#define WR_SPI 2
#define WR_DMA_RX 3

/* ####### Interrupt Priorities ####### */
#define VSYNC_PRIORITY       5
#define PCLK_PRIORITY        5
#define NRF_DMA_PRIORITY     4
#define NRF_INT_PRIORITY     4
#define FLAP_TIMER_PRIORITY  3
#define ACC_INT_PRIORITY     3
#define ACC_DMA_PRIORITY     3
#define GYRO_INT_PRIORITY    3
#define GYRO_DMA_PRIORITY    3
#define THROTTLE_IC_PRIORITY 3
#define ROLL_IC_PRIORITY     3
#define PITCH_IC_PRIORITY    3
#define YAW_IC_PRIORITY      3
#define SHORT_HREF_PRIORITY  6
#define LONG_HREF_PRIORITY   6
#define LOOP_TMR_PRIORITY    2

#ifdef	__cplusplus
}
#endif

#endif	/* SETTINGS_H */