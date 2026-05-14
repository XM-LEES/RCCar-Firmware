#include "app_runtime_state.h"

AppRuntimeState_t g_app_runtime_state = {
    .voltage_v = 0.0f,
    .debug_level = 0U,
    .uart4_rx_frame_error_seen = 0U,
    .uart4_tx_busy_count = 0U,
    .uart4_tx_error_count = 0U,
    .usart1_debug_tx_busy_count = 0U,
    .usart1_debug_tx_error_count = 0U,
};
