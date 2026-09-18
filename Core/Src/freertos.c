/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Minimal FreeRTOS application set for Ackermann-only control.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>

#include "queue.h"
#include "bsp_buzzer.h"
#include "esc_telemetry_stm32.h"
#include "servo_basic_task.h"

QueueHandle_t g_xQueueROSserial = NULL;
TaskHandle_t g_servoTaskHandle = NULL;

osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

void show_task(void* param);
void SerialControlTask(void* param);
void RobotDataTransmitTask(void* param);

void StartInitTask(void *argument);

void MX_FREERTOS_Init(void);

__weak void configureTimerForRunTimeStats(void)
{
    TIM6->CNT = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
    static unsigned long time = 0;
    static uint16_t lasttime = 0;
    uint16_t nowtime = TIM6->CNT;

    if (nowtime < lasttime)
    {
        time += (unsigned long)(nowtime + 0xFFFFU - lasttime);
    }
    else
    {
        time += (unsigned long)(nowtime - lasttime);
    }

    lasttime = nowtime;
    return time;
}

void vApplicationIdleHook(void)
{
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    printf("%s stack overflow\r\n", pcTaskName);
    (void)xTask;
}

void vApplicationMallocFailedHook(void)
{
    printf("malloc failed.check heapsize\r\n");
}

void MX_FREERTOS_Init(void)
{
    BaseType_t ret;

    g_xQueueROSserial = xQueueCreate(50, sizeof(char));

    InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

    ret = xTaskCreate(ServoBasic_Task, "ServoTask", 128 * 2, NULL, osPriorityHigh, &g_servoTaskHandle);
    if (ret != pdPASS)
    {
        Error_Handler();
    }

    ret = xTaskCreate(show_task, "showTask", 128 * 4, NULL, osPriorityBelowNormal7, NULL);
    if (ret != pdPASS)
    {
        Error_Handler();
    }

    ret = xTaskCreate(SerialControlTask, "SerialConTask", 128 * 2, NULL, osPriorityNormal, NULL);
    if (ret != pdPASS)
    {
        Error_Handler();
    }

    ret = xTaskCreate(RobotDataTransmitTask, "transmit_task", 128 * 4, NULL, osPriorityNormal, NULL);
    if (ret != pdPASS)
    {
        Error_Handler();
    }

    ret = xTaskCreate(EscTelemetryTask, "escTelemTask", 128 * 2, NULL, osPriorityNormal, NULL);
    if (ret != pdPASS)
    {
        Error_Handler();
    }
}

void StartInitTask(void *argument)
{
    (void)argument;

    pBuzzerInterface_t tips = &UserBuzzer;
    tips->AddTask(1, 700);

    vTaskDelete(NULL);
}
