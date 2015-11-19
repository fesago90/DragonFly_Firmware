/* 
 * File:   Common.h
 * Author: Felipe
 *
 * Created on January 24, 2013, 12:21 PM
 */

#ifndef COMMON_H
#define	COMMON_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Settings.h"

#define FCY (long)(SYSTEM_FREQUENCY_IN_MHZ * 1000000L)

#define PLL_N1 (25)
#define PLL_N2 (2)
#ifdef TAN_BOARDS
    #define PLL_M  (SYSTEM_FREQUENCY_IN_MHZ * PLL_N2 )
#else
    #define PLL_M  (SYSTEM_FREQUENCY_IN_MHZ * PLL_N2 * 2)
#endif


#include <p33EP512MC806.h>
#include <libpic30.h>
#include <stdint.h>
#include "Debug.h"
#include "LED.h"

#define _JOIN2(a,b)         a##b
#define _JOIN3(a,b,c)       a##b##c
#define _JOIN4(a,b,c,d)     a##b##c##d
#define _JOIN5(a,b,c,d,e)   a##b##c##d##e
#define _JOIN6(a,b,c,d,e,f) a##b##c##d##e##f
    
#define _LAT(port) _JOIN2(LAT, port)
#define _TRIS(port) _JOIN2(TRIS,port)
#define _PORT(port) _JOIN2(PORT,port)
#define _ANSEL(port) _JOIN2(ANSEL,port)

#define _TRISbit(port,num)  _JOIN5(TRIS, port, bits.TRIS, port, num)
#define _LATbit(port,num)   _JOIN5(LAT, port, bits.LAT, port, num)
#define _PORTbit(port,num)  _JOIN5(PORT, port, bits.R, port, num)
#define _ANSELbit(port,num) _JOIN5(ANSEL, port, bits.ANS, port, num)
#define _ODCbit(port,num)   _JOIN5(ODC, port, bits.ODC, port, num)

#define _BTG(reg,bitnum) __builtin_btg((unsigned int *) (& reg) , bitnum )

#define _ISR_NO_PSV __attribute__((__interrupt__, no_auto_psv))
#define _ISR_PSV __attribute__((__interrupt__, no_auto_psv))

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define ON 1
#define OFF 0

#ifdef	__cplusplus
}
#endif

#endif	/* COMMON_H */

