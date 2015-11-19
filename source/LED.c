#include "Common.h"
#include "LED.h"

void LED_Init()
{
    _TRISbit(LED1_PORT, LED1_BIT) = 0;
    _TRISbit(LED2_PORT, LED2_BIT) = 0;
    _TRISbit(LED3_PORT, LED3_BIT) = 0;
    _TRISbit(LED4_PORT, LED4_BIT) = 0;

    LED_Set_And_Wait(LEDOff, LEDOff, LEDOff, LEDOff, 0);

}

void LED_Set_And_Wait(LEDState led1, LEDState led2, LEDState led3, LEDState led4, uint16_t millis)
{
    LED1_W = led1;
    LED2_W = led2;
    LED3_W = led3;
    LED4_W = led4;

    if (millis > 0)
        __delay_ms(millis);
}