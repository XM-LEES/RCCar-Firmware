#ifndef RCCAR_HOST_STUB_TASK_H
#define RCCAR_HOST_STUB_TASK_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previous_wake_time, TickType_t time_increment);
void vTaskDelay(TickType_t ticks);

#ifdef __cplusplus
}
#endif

#endif
