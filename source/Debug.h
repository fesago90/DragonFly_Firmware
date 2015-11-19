/* 
 * File:   Debug.h
 * Author: Felipe
 *
 * Created on February 4, 2013, 4:29 PM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Common.h"
#include "Pins.h"

    typedef enum {
        ErrOsc      = 1,
        ErrAcc      = 2,
        ErrGyro     = 3,
        ErrRF       = 4,
        ErrServPow  = 5,
        ErrDefault  = 7
    } ErrNum;

    void Display_Error(ErrNum errNum);


#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */

