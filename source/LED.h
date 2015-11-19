/* 
 * File:   LED.h
 * Author: Felipe
 *
 * Created on April 17, 2013, 3:12 PM
 */

#ifndef LED_H
#define	LED_H

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        LEDOn = 0,
        LEDOff = 1
    } LEDState;

    void LED_Init();
    void LED_Set_And_Wait(LEDState led1, LEDState led2, LEDState led3, LEDState led4, uint16_t millis);

#ifdef	__cplusplus
}
#endif

#endif	/* LED_H */

