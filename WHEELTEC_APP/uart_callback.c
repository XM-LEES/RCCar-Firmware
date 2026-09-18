#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t rosbuffer;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart4)
    {
        extern QueueHandle_t g_xQueueROSserial;
        if (g_xQueueROSserial != NULL)
        {
            xQueueSendFromISR(g_xQueueROSserial, &rosbuffer, &xHigherPriorityTaskWoken);
        }
        HAL_UART_Receive_IT(&huart4, &rosbuffer, 1U);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}
