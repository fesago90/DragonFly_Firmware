/* 
 * File:   DebugTiming.h
 * Author: Ian
 *
 * Created on July 10, 2013, 3:20 PM
 */

#ifndef DEBUGTIMING_H
#define	DEBUGTIMING_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Common.h"

    void Debug_Timer_Init(void);
    inline void Debug_Timer_Reset(void);
    inline void Debug_Timer_Stop(void);
    inline void Debug_Timer_Start(void);

#ifdef	__cplusplus
}
#endif

#endif	/* DEBUGTIMING_H */

