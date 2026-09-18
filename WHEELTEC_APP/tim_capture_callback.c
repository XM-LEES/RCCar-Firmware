/**
 * @file    tim_capture_callback.c
 * @brief   Timer input capture callback for RC PWM inputs.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "servo_rc_capture.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    BaseType_t schedulerState = xTaskGetSchedulerState();
    if (schedulerState != taskSCHEDULER_RUNNING)
    {
        return;
    }

    if (htim == &htim4)
    {
        ServoRC_IC_CaptureCallback(htim);
    }
}
