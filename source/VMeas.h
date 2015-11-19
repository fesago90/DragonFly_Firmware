/* 
 * File:   VMeas.h
 * Author: Felipe
 *
 * Created on February 21, 2013, 12:50 PM
 */

#ifndef VMEAS_H
#define	VMEAS_H

#ifdef	__cplusplus
extern "C" {
#endif

//#include <stdint.h>

#define VMEAS_MIN_LIBATT ((MINIMUM_SAFE_BATTERY_VOLTAGE/(3.0f))*2048)    // Corresponds to 2.7V at Batt line
#define VMEAS_GOOD_LIBATT ((GOOD_BATTERY_VOLTAGE/(3.0f))*2048)

    /* !!!!WARNING!!!!
     * Order of the fields is VERY IMPORTANT since ADC data is 'pasted'
     * onto DMARAM block corresponding to an instance of this structure.
     */

    typedef struct {
        int16_t V_Supply;
        int16_t V_Batt;
        int16_t V_Servo;
    } VoltageData;

    extern uint16_t VMeasFilteredSampleCount;

    void VMeas_Init();
    void VMeas_Start();
    void VMeas_Filter_Current_Sample();
    VoltageData* VMeas_Get_Current_Data();
    VoltageData* VMeas_Get_Current_Filt_Data();

#ifdef	__cplusplus
}
#endif

#endif	/* VMEAS_H */

