#ifndef __ESC_TELEMETRY_STM32_H
#define __ESC_TELEMETRY_STM32_H

#include <stdint.h>

#include "esc_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

void EscTelemetryStm32_Init(void);
uint8_t EscTelemetryStm32_Start(void);
void EscTelemetryStm32_Service(void);
void EscTelemetryTask(void *param);

#ifdef __cplusplus
}
#endif

#endif
