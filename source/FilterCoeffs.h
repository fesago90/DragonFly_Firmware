/* 
 * File:   FilterCoeffs.h
 * Author: Felipe
 *
 * Created on May 1, 2013, 1:39 PM
 */

#ifndef FILTERCOEFFS_H
#define	FILTERCOEFFS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <dsp.h>

/*
 * Low pass filter... we didnt note down the characteristics :(
 */
#define LPFNumCoeffs 50
extern fractional LPFCoeffs[LPFNumCoeffs] __attribute__ ((space(xmemory)));

/*
 * Low-pass filter coeffs with 1dB attenuation at 5Hz and 50dB attenuation at 25Hz
 * and 1000Hz sample rate.
 */
#define LPF_5_1_25_50_SIZE 86
extern fractional   LPF_5_1_25_50[LPF_5_1_25_50_SIZE] __attribute__ ((space(xmemory)));
/*
// FIR filter for 800Hz sample rate
#define LPF_5_1_25_50_SIZE 69
extern fractional   LPF_5_1_25_50[LPF_5_1_25_50_SIZE] __attribute__ ((space(xmemory)));
*/
#ifdef	__cplusplus
}
#endif

#endif	/* FILTERCOEFFS_H */

