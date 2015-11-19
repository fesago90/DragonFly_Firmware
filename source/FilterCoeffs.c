/* 
 * File:   FilterCoeffs.c
 * Author: Felipe
 *
 * Created on May 1, 2013, 1:39 PM
 */

#include "Common.h"
#include "FilterCoeffs.h"

fractional LPFCoeffs[LPFNumCoeffs] __attribute__ ((space(xmemory))) =
{
    8,13,25,41,65,96,137,188,249,323,408,503,608,721,839,959,1079,1195,1303,1399,
    1481,1545,1589,1611,1611,1589,1545,1481,1399,1303,1195,1079,959,839,721,608,
    503,408,323,249,188,137,96,65,41,25,13,8
};


fractional LPF_5_1_25_50[LPF_5_1_25_50_SIZE] __attribute__ ((space(xmemory))) =
{0, 54, 38, 39, 46, 55, 66, 78, 91, 106, 122, 139, 157, 176, 197, 218, 241, 265, 289, 314, 340, 366, 393, 420, 447, 473, 500, 526, 552, 576, 600, 623, 644, 664, 683, 700, 715, 728, 739, 748, 755, 759, 761, 761, 759, 755, 748, 739, 728, 715, 700, 683, 664, 644, 623, 600, 576, 552, 526, 500, 473, 447, 420, 393, 366, 340, 314, 289, 265, 241, 218, 197, 176, 157, 139, 122, 106, 91, 78, 66, 55, 46, 39, 38, 54, 0};
/*{
    0, 68, 50, 60, 74, 92, 114, 138, 164, 192, 224, 258, 294, 332, 372, 414, 458, 502, 546, 592, 636, 682, 724, 766, 806, 844, 880, 912, 940, 966, 986, 1002,
    1014, 1022, 1024, 1022, 1014, 1002, 986, 966, 940, 912, 880, 844, 806, 766, 724, 682, 636, 592, 546, 502, 458, 414, 372, 332, 294, 258, 224, 192, 164, 138,
    114, 92, 74, 60, 50, 68, 0
};*/