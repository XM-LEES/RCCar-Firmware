#ifndef RCCAR_HOST_STUB_FREERTOS_H
#define RCCAR_HOST_STUB_FREERTOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;

#define pdPASS 1
#define pdFAIL 0
#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif
