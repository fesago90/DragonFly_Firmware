/* 
 * File:   ConfigBits.h
 * Author: Felipe
 *
 * Created on January 24, 2013, 12:24 PM
 */

#ifndef CONFIGBITS_H
#define	CONFIGBITS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <p33EP512MC806.h>

_FGS(GWRP_OFF & GSS_OFF & GSSK_OFF)

_FOSCSEL(FNOSC_FRC & IESO_OFF)

_FOSC(POSCMD_EC & OSCIOFNC_ON  & FCKSM_CSECME & IOL1WAY_OFF);

_FWDT(FWDTEN_OFF & PLLKEN_OFF);

_FICD(ICS_PGD3 & JTAGEN_OFF)

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIGBITS_H */

