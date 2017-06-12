#ifndef ADS1232
#define ADS1232

#include "stm32f1xx_hal.h"
#include "stdbool.h"

typedef struct
{
    GPIO_TypeDef* gpioSck;
    GPIO_TypeDef* gpioData;
    uint16_t pinSck;
    uint16_t pinData;
    int32_t offset;
    bool needCalib;
} ADS123x;


void ADS_Init(const ADS123x *data);
ADS123x *ADS_Tare(ADS123x *data, uint8_t times);
int32_t ADS_Value(ADS123x *data);
int32_t ADS_AverageValue(ADS123x *data, uint8_t times);




#endif //ADS1232
