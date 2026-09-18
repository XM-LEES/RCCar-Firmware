#ifndef RCCAR_HOST_STUB_QUEUE_H
#define RCCAR_HOST_STUB_QUEUE_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *QueueHandle_t;

BaseType_t xQueueReceive(QueueHandle_t queue,
                         void *buffer,
                         TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif

#endif
