/*
 * File:   Accel.h
 * Author: Felipe
 *
 * Created on January 29, 2013, 2:56 PM
 */

#ifndef ACCEL_H
#define	ACCEL_H

#ifdef	__cplusplus
extern "C" {
#endif

    /* !!!!WARNING!!!!
     * Order of the fields is VERY IMPORTANT since accel data is 'pasted'
     * onto DMARAM block corresponding to an instance of this structure.
     */

    /*
     * In the ALPHA, the nRF side of the board is at the BACK and the microchip
     * faces UP then DragonflyY points toward the front and DragonflyX points
     * toward the right. Therefore:
     *  Dragonfly_X = Accel's X axis
     *  Dragonfly_Y = Accel's Y axis
     *  Dragonfly_Z = Accel's Z axis
     *
     * The registers are extracted from SPI in X, Y, Z order therefore to match
     * the above orientations, the following structure will need to have its
     * fields in the order Acc_X, Acc_Y, Acc_Z.
     *
     */

    typedef struct {
        uint16_t  junk;  // This word holds junk data. IGNORE IT
        int16_t Acc_X;
        int16_t Acc_Y;
        int16_t Acc_Z;
    } AccData;

    void Acc_Init();
    AccData* Acc_Get_Current_Data();
    inline void Acc_Trigger_Read_Data();

#ifdef	__cplusplus
}
#endif

#endif	/* ACCEL_H */

