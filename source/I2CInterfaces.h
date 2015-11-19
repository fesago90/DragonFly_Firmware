/* 
 * File:   I2CInterfaces.h
 * Author: Ian
 *
 * Created on June 20, 2013, 12:50 PM
 */

#ifndef I2CINTERFACES_H
#define	I2CINTERFACES_H

#ifdef	__cplusplus
extern "C" {
#endif

/***************** USER CHANGES BELOW *****************************************/
#include <p33EP512MC806.h> // Change this to your microcontroller
#include "Pins.h"

#define SDA_PORT CAM_SIOD_PORT      // Definitions -- Change these to match pins used as
#define SDA_PIN  CAM_SIOD_BIT       // SDA and SCL
#define SCL_PORT CAM_SIOC_PORT
#define SCL_PIN  CAM_SIOC_BIT

        // Error and success codes.  Users are encouraged to customize per
        // application.  Do not make the WRITE_SUCCESS macro the same as either
        // error code, or the Robust line of functions will not perform as
        // intended.
        // Error codes should be values that you do not intend to see as valid
        // data.  If there is a possibility of overlap, do not use the Robust
        // functions and expect them to be reliable.
#define TIME_OUT_ERROR TIME_OUT_ERROR_CODE
#define BAD_ACK_ERROR BAD_ACK_ERROR_CODE
#define WRITE_SUCCESS 1

    // The following is used to specify how many cycles you wish to wait before
    // considering an I2C read, write, stop, start, or restart timed out.
#define WAIT_TIME TIME_TO_WAIT

    // You can easily change the data type that the I2C library uses here.
    // Keep in mind that you'll need to use unsigned chars for wait times
    // greater than 127, integers for wait times greater than 255, unsigned
    // integers greater than 32767, etc.
#define I2C_TYPE unsigned int // currently only ints work for some reason
/****************** END USER CHANGES ******************************************/

    // Function prototypes
    void I2C_Init(void);
    I2C_TYPE I2C_Read(I2C_TYPE DevAddr, I2C_TYPE RegAddr);
    I2C_TYPE I2C_Write(I2C_TYPE DevAddr, I2C_TYPE RegAddr, I2C_TYPE data);
    void I2C_Robust_Write(I2C_TYPE DevAddr, I2C_TYPE RegAddr, I2C_TYPE data);
    I2C_TYPE I2C_Robust_Read(I2C_TYPE DevAddr, I2C_TYPE RegAddr);

#ifdef	__cplusplus
}
#endif

#endif	/* I2CINTERFACES_H */

