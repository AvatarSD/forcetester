#include "ads1232.h"



void ADS_Init(ADS123x data)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = data.pinSck;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(data.gpioSck, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = data.pinData;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(data.gpioData, &GPIO_InitStruct);

    /*HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(data.gpioData, data.pinSck, GPIO_PIN_RESET);*/
}

int ADS_Value(ADS123x data)
{

}

int ADS_AverageValue(ADS123x data, uint8_t times)
{
    int sum = 0;
    for (int i = 0; i < times; i++)
    {
        sum += ADS_Value(data);
    }

    return sum / times;
}

ADS123x ADS_Tare(ADS123x data, uint8_t times)
{
    int sum = ADS_AverageValue(data, times);
    data.offset = sum;
    return data;
}

