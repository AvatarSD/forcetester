/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "ads1232.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

osThreadId ledTaskHandle;
osThreadId adsPoolingHandle;
osThreadId stopDiffDetectHandle;
osThreadId strtStpLinDetecHandle;
osThreadId serialOutputHandle;
osMessageQId startStopEventQueueHandle;
osMailQId mainDataSerialQueueHandle;
osMailQId mainDataDiffDetectQueueHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef struct{
    int32_t load;
    int16_t pos;
}MainData;


typedef enum{
    StopEvent = 35,
    StartEvent = 78
} StartStopEvent;

#define ENCODER_VAL ((int16_t)htim3.Instance->CNT)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
void ledDefaultTask(void const * argument);
void adsTask(void const * argument);
void stopDiffDetectButtonTask(void const * argument);
void startStopLinearDetectTask(void const * argument);
void serialOutputTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */
    /* init code for USB_DEVICE */
    HAL_Delay(10);
    MX_USB_DEVICE_Init();
    HAL_Delay(50);
    HAL_GPIO_WritePin(ADS_PWRD_GPIO_Port, ADS_PWRD_Pin, GPIO_PIN_SET);
    HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of ledTask */
    osThreadDef(ledTask, ledDefaultTask, osPriorityNormal, 0, 128);
    ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

    /* definition and creation of adsPooling */
    osThreadDef(adsPooling, adsTask, osPriorityNormal, 0, 256);
    adsPoolingHandle = osThreadCreate(osThread(adsPooling), NULL);

    /* definition and creation of stopDiffDetect */
    osThreadDef(stopDiffDetect, stopDiffDetectButtonTask, osPriorityNormal, 0, 128);
    stopDiffDetectHandle = osThreadCreate(osThread(stopDiffDetect), NULL);

    /* definition and creation of strtStpLinDetec */
    osThreadDef(strtStpLinDetec, startStopLinearDetectTask, osPriorityNormal, 0, 128);
    strtStpLinDetecHandle = osThreadCreate(osThread(strtStpLinDetec), NULL);

    /* definition and creation of serialOutput */
    osThreadDef(serialOutput, serialOutputTask, osPriorityNormal, 0, 256);
    serialOutputHandle = osThreadCreate(osThread(serialOutput), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Create the queue(s) */
    /* definition and creation of startStopEventQueue */
    osMessageQDef(startStopEventQueue, 8, StartStopEvent);
    startStopEventQueueHandle = osMessageCreate(osMessageQ(startStopEventQueue), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

    /* definition and creation of mainDataSerialQueue */
    osMailQDef(mainDataSerialQueue, 16, MainData);
    mainDataSerialQueueHandle = osMailCreate(osMailQ(mainDataSerialQueue), NULL);

    /* definition and creation of mainDataDiffDetectQueue */
    osMailQDef(mainDataDiffDetectQueue, 16, MainData);
    mainDataDiffDetectQueueHandle = osMailCreate(osMailQ(mainDataDiffDetectQueue), NULL);

    /* USER CODE END RTOS_QUEUES */


    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xffff;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

}

/** Configure pins as 
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, ADS_PWRD_Pin|ADS_SCK_Pin|STOP_BUTTON_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : ADS_PWRD_Pin ADS_SCK_Pin STOP_BUTTON_Pin */
    GPIO_InitStruct.Pin = ADS_PWRD_Pin|ADS_SCK_Pin|STOP_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ADS_DATA_Pin */
    GPIO_InitStruct.Pin = ADS_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADS_DATA_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* ledDefaultTask function */
void ledDefaultTask(void const * argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        osDelay(150);
    }
    /* USER CODE END 5 */
}

/* adsTask function */
void adsTask(void const * argument)
{
    /* USER CODE BEGIN adsTask */
#define TARE_TIMES 250

    ADS123x ads1232 = {ADS_SCK_GPIO_Port, ADS_DATA_GPIO_Port,
                       ADS_SCK_Pin, ADS_DATA_Pin, 0, true};
    ADS_Init(&ads1232);
    ADS_Tare(&ads1232, TARE_TIMES);

    /* Infinite loop */
    for(;;)
    {
        int32_t load = ADS_Value(&ads1232);
        int16_t pos = ENCODER_VAL;

        MainData * serialMsg = (MainData*)osMailAlloc(mainDataSerialQueueHandle, 0);
        serialMsg->load = load;
        serialMsg->pos = pos;
        osMailPut(mainDataSerialQueueHandle, serialMsg);

        MainData * diffMsg = (MainData*)osMailAlloc(mainDataDiffDetectQueueHandle, 0);
        diffMsg->load = load;
        diffMsg->pos = pos;
        osMailPut(mainDataDiffDetectQueueHandle, diffMsg);
    }
    /* USER CODE END adsTask */
}

/* stopDiffDetectButtonTask function */
void stopDiffDetectButtonTask(void const * argument)
{
    /* USER CODE BEGIN stopDiffDetectButtonTask */
    /* Infinite loop */
    for(;;)
    {
        osEvent msg = osMailGet(mainDataDiffDetectQueueHandle, 0);
        if(msg.status == osEventMail){
            //CDC_Transmit_FS((uint8_t*)"lo\r\n", 4);
        }
        osMailFree(mainDataDiffDetectQueueHandle, msg.value.p);

        /*HAL_GPIO_WritePin(STOP_BUTTON_GPIO_Port, STOP_BUTTON_Pin, GPIO_PIN_SET);
        osDelay(20);
        HAL_GPIO_WritePin(STOP_BUTTON_GPIO_Port, STOP_BUTTON_Pin, GPIO_PIN_RESET);
        osDelay(50);*/
    }
    /* USER CODE END stopDiffDetectButtonTask */
}

/* startStopLinearDetectTask function */
void startStopLinearDetectTask(void const * argument)
{
    /* USER CODE BEGIN startStopLinearDetectTask */
    /* Infinite loop */
    for(;;)
    {
        osMessagePut(startStopEventQueueHandle, StartEvent, 0);
        osDelay(1000);
        osMessagePut(startStopEventQueueHandle, StopEvent, 0);
        osDelay(2000);/**/
    }
    /* USER CODE END startStopLinearDetectTask */
}

/* serialOutputTask function */
void serialOutputTask(void const * argument)
{
    /* USER CODE BEGIN serialOutputTask */

    /* Infinite loop */
    for(;;)
    {
        osEvent msg = osMailGet(mainDataSerialQueueHandle, 0);
        if(msg.status == osEventMail){
            char buff[30];
            sprintf(buff, "!sd/66|%ld|%d\\\r\n", ((MainData*)msg.value.p)->load, ((MainData*)msg.value.p)->pos);
            CDC_Transmit_FS((uint8_t*)buff, strlen(buff));
        }
        osMailFree(mainDataSerialQueueHandle, msg.value.p);

        msg = osMessageGet(startStopEventQueueHandle, 0);
        if(msg.status == osEventMessage){
            StartStopEvent event = msg.value.v;
            switch (event) {
            case StartEvent :
                CDC_Transmit_FS((uint8_t*)"!sd/12\\\r\n", 9);
                break;
            case StopEvent :
                CDC_Transmit_FS((uint8_t*)"!sd/13\\\r\n", 9);
                break;
            }
        }
    }
    /* USER CODE END serialOutputTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
