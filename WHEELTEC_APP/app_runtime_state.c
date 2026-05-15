#include "app_runtime_state.h"

AppRuntimeState_t g_app_runtime_state = {
    .voltage_v = 0.0f,
    .debug_level = 0U,
    .uart4_rx_frame_error_seen = 0U,
    .fault_latched = 0U,
    .active_fault_sources = 0U,
    .uart4_tx_busy_count = 0U,
    .uart4_tx_error_count = 0U,
    .usart1_debug_tx_busy_count = 0U,
    .usart1_debug_tx_error_count = 0U,
};

void AppRuntime_UpdateFaultSources(uint32_t active_sources)
{
    g_app_runtime_state.active_fault_sources = active_sources;
    if (active_sources != 0U)
    {
        g_app_runtime_state.fault_latched = 1U;
    }
}

void AppRuntime_TryClearFaultLatch(void)
{
    if (g_app_runtime_state.active_fault_sources == 0U)
    {
        g_app_runtime_state.fault_latched = 0U;
    }
}
