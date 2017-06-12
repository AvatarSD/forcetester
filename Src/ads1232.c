#include "ads1232.h"
#include "cmsis_os.h"

#define delay() for(uint32_t i = 0; i < 20; i++) __NOP();

void ADS_Init(const ADS123x * data)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = data->pinSck;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(data->gpioSck, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = data->pinData;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(data->gpioData, &GPIO_InitStruct);

}

int32_t ADS_Value(ADS123x *data)
{
    int32_t adsVal = 0;
    while (HAL_GPIO_ReadPin(data->gpioData, data->pinData)) osThreadYield();

    for (uint8_t i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(data->gpioSck, data->pinSck, GPIO_PIN_SET);
        adsVal = adsVal << 1 ;
        delay();
        if (HAL_GPIO_ReadPin(data->gpioData, data->pinData)) adsVal ++;
        HAL_GPIO_WritePin(data->gpioSck, data->pinSck, GPIO_PIN_RESET);
        delay();
    }
    for(uint8_t i = 0; i < 2; i++){
        HAL_GPIO_TogglePin(data->gpioSck, data->pinSck);
        delay();
    }
    if(data->needCalib){
        for(uint8_t i = 0; i < 4; i++)
            HAL_GPIO_TogglePin(data->gpioSck, data->pinSck);
        delay();
    }
    data->needCalib = false;
    if(adsVal > 0x7fffff) adsVal-=0xffffff;
    return adsVal;
}

int32_t ADS_AverageValue(ADS123x * data, uint8_t times)
{
    int32_t sum = 0;
    for (int i = 0; i < times; i++)
        sum += ADS_Value(data);
    return sum / times;
}

ADS123x *ADS_Tare(ADS123x *data, uint8_t times)
{
    data->offset = ADS_AverageValue(data, times);
    return data;
}

