#ifndef ADS1232
#define ADS1232

#include "stm32f1xx_hal.h"

typedef struct _hx711
{
    GPIO_TypeDef* gpioSck;
    GPIO_TypeDef* gpioData;
    uint16_t pinSck;
    uint16_t pinData;
    int offset;
    /*int gain;
    // 1: channel A, gain factor 128
    // 2: channel B, gain factor 32
    // 3: channel A, gain factor 64*/
} ADS123x;


void ADS_Init(ADS123x data);
ADS123x ADS_Tare(ADS123x data, uint8_t times);
int ADS_Value(ADS123x data);
int ADS_AverageValue(ADS123x data, uint8_t times);




#endif //ADS1232
