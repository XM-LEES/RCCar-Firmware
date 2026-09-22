#include "esc_telemetry_stm32.h"

#include "esc_soft_uart_stm32.h"

#if defined(ESC_TELEMETRY_STM32_HOST_TEST)
#include "esc_telemetry_stm32_host_hal.h"
#else
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

extern TaskHandle_t g_servoTaskHandle;
#endif

static void esc_telemetry_stm32_notify_servo(void)
{
#if !defined(ESC_TELEMETRY_STM32_HOST_TEST)
    if (g_servoTaskHandle != NULL)
    {
        (void)xTaskNotifyGive(g_servoTaskHandle);
    }
#endif
}

static uint8_t esc_telemetry_stm32_drain_soft_uart(void)
{
    EscSoftUartStm32Byte_t item;
    uint8_t progressed = 0U;

    while (EscSoftUartStm32_ReadByte(&item) != 0U)
    {
        if (EscTelemetry_ProcessReceivedByte(item.byte,
                                             item.received_tick_ms,
                                             item.output_context) != 0U)
        {
            progressed = 1U;
        }
    }

    return progressed;
}

void EscTelemetryStm32_Init(void)
{
    EscTelemetry_Init();
    EscSoftUartStm32_Init();
    EscSoftUartStm32_SetOutputContextProvider(EscTelemetry_GetOutputContext);
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
    uint8_t notify_servo = 0U;

    EscSoftUartStm32_UpdateTickBase(tick_ms);
    if (fault_flags != 0UL)
    {
        EscTelemetry_RecordRxError(tick_ms, fault_flags);
        notify_servo = 1U;
    }
    if (esc_telemetry_stm32_drain_soft_uart() != 0U)
    {
        notify_servo = 1U;
    }
    if (notify_servo != 0U)
    {
        esc_telemetry_stm32_notify_servo();
    }
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
