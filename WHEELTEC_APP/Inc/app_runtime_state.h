#ifndef __APP_RUNTIME_STATE_H
#define __APP_RUNTIME_STATE_H

#include <stdint.h>

#define APP_FAULT_SOURCE_HALL             (1UL << 0)
#define APP_FAULT_SOURCE_STEERING         (1UL << 1)
#define APP_FAULT_SOURCE_BATTERY_LOW      (1UL << 2)
#define APP_FAULT_SOURCE_BATTERY_CRITICAL (1UL << 3)
#define APP_FAULT_SOURCE_FRAME_ERROR      (1UL << 4)

typedef struct
{
    float voltage_v;
    uint8_t debug_level;
    uint8_t uart4_rx_frame_error_seen;
    uint8_t fault_latched;
    uint32_t active_fault_sources;
    uint32_t uart4_tx_busy_count;
    uint32_t uart4_tx_error_count;
    uint32_t usart1_debug_tx_busy_count;
    uint32_t usart1_debug_tx_error_count;
} AppRuntimeState_t;

extern AppRuntimeState_t g_app_runtime_state;

void AppRuntime_UpdateFaultSources(uint32_t active_sources);
void AppRuntime_TryClearFaultLatch(void);

#endif
