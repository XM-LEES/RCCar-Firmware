#include "esc_telemetry_stm32.h"

#include "esc_soft_uart_stm32.h"

#if defined(ESC_TELEMETRY_STM32_HOST_TEST)
#include "esc_telemetry_stm32_host_hal.h"
#else
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#endif

static void esc_telemetry_stm32_drain_soft_uart(void)
{
    uint8_t chunk[ESC_TELEMETRY_PENDING_CHUNK_SIZE];
    size_t chunk_len = 0U;
    uint32_t chunk_tick_ms = 0U;
    EscSoftUartStm32Byte_t item;

    while (EscSoftUartStm32_ReadByte(&item) != 0U)
    {
        chunk[chunk_len] = item.byte;
        chunk_len++;
        chunk_tick_ms = item.received_tick_ms;
        if (chunk_len == sizeof(chunk))
        {
            EscTelemetry_RecordBytes(chunk, chunk_len, chunk_tick_ms);
            chunk_len = 0U;
        }
    }

    if (chunk_len != 0U)
    {
        EscTelemetry_RecordBytes(chunk, chunk_len, chunk_tick_ms);
    }
}

void EscTelemetryStm32_Init(void)
{
    EscTelemetry_Init();
    EscSoftUartStm32_Init();
}

uint8_t EscTelemetryStm32_Start(void)
{
    const uint32_t tick_ms = HAL_GetTick();

    EscTelemetry_ResetReceiveBoundary(
        tick_ms,
        ESC_TELEMETRY_INVALIDATION_RECEIVER_RESTART);
    return EscSoftUartStm32_Start(tick_ms);
}

void EscTelemetryStm32_Service(void)
{
    const uint32_t tick_ms = HAL_GetTick();
    const uint32_t fault_flags = EscSoftUartStm32_TakeFaults();

    EscSoftUartStm32_UpdateTickBase(tick_ms);
    if (fault_flags != 0UL)
    {
        EscTelemetry_RecordRxError(tick_ms, fault_flags);
    }
    esc_telemetry_stm32_drain_soft_uart();
}

void EscTelemetryTask(void *param)
{
    (void)param;

    for (;;)
    {
        EscTelemetryStm32_Service();
        EscTelemetry_ProcessPending();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}
