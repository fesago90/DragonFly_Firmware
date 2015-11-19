// Includes
#include "I2CInterfaces.h"
#include "Common.h"
#include <i2c.h>

// Definitions
#define READ 1
#define WRITE 0

/********************The following macros are HARDWARE SPECIFIC****************/
// These macros assign the appropriate dynamically generated names to their
// respective descriptors
#define SDA     _LATbit(SDA_PORT, SDA_PIN)
#define ReadSDA _PORTbit(SDA_PORT, SDA_PIN)
#define SCL     _LATbit(SCL_PORT, SCL_PIN)

/******************* END HARDWARE SPECIFIC MACROS *****************************/


// This macro controls for time outs to prevent infinite looping.
// The number of cycles to wait can be specified in I2CInterfaces.h
#define WAIT_FOR(a) for(count = 0; count < WAIT_TIME; count++) \
                    {                                          \
                        if(a) break;                           \
                        if (count == (WAIT_TIME - 1))          \
                            time_out = 1;                      \
                        IFS1bits.MI2C1IF = 0;                  \
                    }

// This macro checks the time out flag to see if a time out has occurred.
// It will then reset the flag and prematurely terminate the function it is used
// in, sending the specified error code.
// Note: This macro WILL NOT WORK as an inline function.  A return used in an
//       inline function will only return from that inline function, not
//       prematurely terminate its parent function.  Additionally, if a return
//       value of void is specified in the parent function, the inline function
//       will not be able to handle a specifier amounting to that.  You may
//       return "void" with this macro by calling "CHECK_TIME_OUT()".
#define CHECK_TIME_OUT(a)   if(time_out)                     \
                            {                                \
                                time_out = 0;                \
                                return a;                    \
                            }

// This macro checks the value of an ack to see if a write has been properly
// received.  if it hasn't, it will prematurely end the function it is used in,
// sending the specified error code.
// Note: This macro WILL NOT WORK as an inline function.  A return used in an
//       inline function will only return from that inline function, not
//       prematurely terminate it's parent function.  Additionally, if a return
//       value of void is specified in the parent function, the inline function
//       will not be able to handle a specifier amounting to that.  You may
//       return "void" with this macro by calling "CHECK_FAILURE()".
#define CHECK_FAILURE(ack, a) if (!ack) return a

// Function prototypes
void i2c_start(void);
void i2c_stop(void);
I2C_TYPE i2c_read(I2C_TYPE ack);
I2C_TYPE i2c_write(I2C_TYPE data);
void i2c_restart(void);

// Global variables
char time_out = 0;
char count = 0;

// This function forces a correct read by looping I2C_Read until a non-error
// output is generated.
I2C_TYPE I2C_Robust_Read(I2C_TYPE DevAddr, I2C_TYPE RegAddr)
{
    volatile I2C_TYPE output = BAD_ACK_ERROR; // declare test variable with bad value

    // Loop read until the read returns a non-error code value.
    do {
        output = I2C_Read(DevAddr, RegAddr);
    } while ((output == TIME_OUT_ERROR) || (output == BAD_ACK_ERROR));
    Nop();
    return output;
}

// This function forces a correct write by verifying each write with a read
// and repeating this process until the data to be written matches the data
// read.
void I2C_Robust_Write(I2C_TYPE DevAddr, I2C_TYPE RegAddr, I2C_TYPE data)
{
     volatile I2C_TYPE check = BAD_ACK_ERROR;// = BAD_ACK_ERROR; // declare test variable with bad value

    // Internal loops forces re-writing until a successful write, followed by
    // re-reading until a successful read.  External loop requires the process
    // to repeat until the data read from the register matches the data that was
    // intended to be written to it.
    do {
        do {
            check = I2C_Write(DevAddr, RegAddr, data);
            Nop();
        } while (check != WRITE_SUCCESS);
        check = I2C_Robust_Read(DevAddr, RegAddr);
        Nop();
    } while(check != data);
}

// This function reads the register at RegAddr of the device DevAddr and returns
// the value.  It may instead return an error code specified in I2CInterfaces.h
I2C_TYPE I2C_Read(I2C_TYPE DevAddr, I2C_TYPE RegAddr)
{
    I2C_TYPE ack = 0;

    // Initialize the Start bit
    i2c_start();
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Write the device address with the "WRITE" specifier
    ack = i2c_write(DevAddr | WRITE);
    CHECK_FAILURE(ack,BAD_ACK_ERROR);
    CHECK_TIME_OUT(TIME_OUT_ERROR); // may require input of 0

    // Write the register address of the device to be read from
    ack = i2c_write(RegAddr);
    CHECK_FAILURE(ack,BAD_ACK_ERROR);
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Initialize the Repeated Start bit
    i2c_restart();
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Write the device address with the "READ" specifier
    ack = i2c_write(DevAddr | READ);
    CHECK_FAILURE(ack,BAD_ACK_ERROR);
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Read the data from the device
    ack = i2c_read(0);
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Initialize the Stop bit
    i2c_stop();
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Return the read value
    return (ack & 0xFF);
}

// This function writes the value data to the register RegAddr of device DevAddr
// It will either return an error code specified in I2CInterfaces.h or will
// return a success code
I2C_TYPE I2C_Write(I2C_TYPE DevAddr, I2C_TYPE RegAddr, I2C_TYPE data)
{
    I2C_TYPE ack = 0;

    // Initialize Start bit
    i2c_start();
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Write the device address with the "WRITE" specifier
    ack = i2c_write(DevAddr | WRITE);
    CHECK_FAILURE(ack,BAD_ACK_ERROR);
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Write the register address of the device to be read from
    ack = i2c_write(RegAddr);
    CHECK_FAILURE(ack,BAD_ACK_ERROR);
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Write the data to the specified register
    ack = i2c_write(data);
    CHECK_FAILURE(ack,BAD_ACK_ERROR);
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    // Initialize the Stop bit
    i2c_stop();
    CHECK_TIME_OUT(TIME_OUT_ERROR);

    return WRITE_SUCCESS;
}

/*****************The following functions are HARDWARE SPECIFIC****************/

// This function initiazes both SDA and SCL to be Open-Drain and digital outputs
// It also configures the I2C module
void I2C_Init(void)
{
    I2C1CONbits.I2CEN = 0;
    // Set up the pins
    _ODCbit(SDA_PORT, SDA_PIN)  = 1;
    _ODCbit(SCL_PORT, SCL_PIN)  = 1;
    _TRISbit(SDA_PORT, SDA_PIN) = 0;
    _TRISbit(SCL_PORT, SCL_PIN) = 0;
    _LATbit(SDA_PORT, SDA_PIN)  = 1;
    _LATbit(SCL_PORT, SCL_PIN)  = 1;

    // Determine BRG register settings
    //float Pulse_Gobbler_Delay = 0.000000130; // 130 nanoseconds
    //float F_CL = 100000; // 100 kHz SCL operation
    //float F_CY = (float)FCY;
    //float Baud_Rate_Generator = F_CY*((1/F_CL) - Pulse_Gobbler_Delay) - 1;

    // Initialize the I2C module
    int config1 = 0x8200; // This is the I2C1CON register
    int config2 = 500;//(int)Baud_Rate_Generator; // This is the I2C1BRG register
    OpenI2C1(config1, config2);
}

// This function establishes the Start bit, but allows the attempt to time out
void i2c_start()
{
    IdleI2C1();
    StartI2C1();
    WAIT_FOR(!I2C1CONbits.SEN);
    IFS1bits.MI2C1IF = 0;
}

// This function establishes the Stop bit, but allows the attempt to time out
void i2c_stop()
{
    IdleI2C1();
    StopI2C1();
    WAIT_FOR(!I2C1CONbits.PEN);
    IFS1bits.MI2C1IF = 0;
}

// This function sends a write command, allows the attempt to time out, and
// returns whether or not the write was acknowledged
I2C_TYPE i2c_write(I2C_TYPE data_out)
{
    MasterWriteI2C1(data_out);
    WAIT_FOR(!I2C1STATbits.TBF);
    WAIT_FOR(IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    //WAIT_FOR(!I2C1STATbits.ACKSTAT);
    return 1;
}

// This functions reads from the slave and allows the attempt to time out.
I2C_TYPE i2c_read(I2C_TYPE ack)
{
    I2C_TYPE data_in = 0;
    IdleI2C1();
    data_in = MasterReadI2C1();
    IFS1bits.MI2C1IF = 0;

    IdleI2C1();
    if (ack) AckI2C1();
    else NotAckI2C1();
    WAIT_FOR(I2C1CONbits.ACKEN);
    IFS1bits.MI2C1IF = 0;
    return data_in;
}

// This function establishes the Repeated Start condition and allows the attempt
// to time out.
void i2c_restart()
{
    IdleI2C1();
    RestartI2C1();
    WAIT_FOR(!I2C1CONbits.RSEN);
    IFS1bits.MI2C1IF = 0;
}

/**************** END HARDWARE SPECIFIC FUNCTIONS *****************************/
