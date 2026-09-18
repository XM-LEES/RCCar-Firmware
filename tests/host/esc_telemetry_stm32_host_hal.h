#ifndef ESC_TELEMETRY_STM32_HOST_HAL_H
#define ESC_TELEMETRY_STM32_HOST_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t HAL_GetTick(void);
void vTaskDelay(uint32_t ticks);

#define pdMS_TO_TICKS(ms) (ms)

#ifdef __cplusplus
}
#endif

#endif
