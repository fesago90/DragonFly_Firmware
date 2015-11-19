/* 
 * File:   Gyro.h
 * Author: Felipe
 *
 * Created on January 29, 2013, 3:05 PM
 */

#ifndef GYRO_H
#define	GYRO_H

#ifdef	__cplusplus
extern "C" {
#endif

    /* !!!!WARNING!!!!
     * Order of the fields is VERY IMPORTANT since gyro data is 'pasted'
     * onto DMARAM block corresponding to an instance of this structure.
     */
    
    /*
     * In the ALPHA, the nRF side of the board is at the BACK and the microchip
     * faces UP then DragonflyY points toward the front and DragonflyX points
     * toward the right. Therefore:
     *  Dragonfly_X =      Gyroscope's X axis
     *  Dragonfly_Y =      Gyroscope's Y axis
     *  Dragonfly_Z =      Gyroscope's Z axis
     *
     * The registers are extracted from SPI in X, Y, Z order therefore to match
     * the above orientations, the following structure will need to have its
     * fields in the order Gy_X, Gy_Y, GyZ.
     *
     */

    typedef struct {
        uint16_t  junk; // This byte holds junk data
        int16_t Gy_X;
        int16_t Gy_Y;
        int16_t Gy_Z;
    } GyroData;

    

    void      Gyro_Init();
    GyroData* Gyro_Get_Current_Data();
    inline void Gyro_Trigger_Read_Data();

    
    
#ifdef	__cplusplus
}
#endif

#endif	/* GYRO_H */

