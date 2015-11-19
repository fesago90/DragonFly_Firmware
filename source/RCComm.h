/* 
 * File:   RCComm.h
 * Author: Felipe
 *
 * Created on March 6, 2013, 10:58 AM
 */

#ifndef RCCOMM_H
#define	RCCOMM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "Common.h"

void RC_Init();

void RC_Start_Calibrating_Boundaries();
int  RC_Stop_Calibrating_Boundaries();

void RC_Start_Calibrating_Averages();
void RC_Stop_Calibrating_Averages();

#ifdef	__cplusplus
}
#endif

#endif	/* RCCOMM_H */

