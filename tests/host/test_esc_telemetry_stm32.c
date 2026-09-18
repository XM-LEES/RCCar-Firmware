#include "esc_telemetry_stm32.h"
#include "esc_soft_uart_stm32.h"
#include "esc_soft_uart_stm32_host.h"
#include "../fixtures/esc_fe32_samples.h"
#include "esc_telemetry_stm32_host_hal.h"

#include <stdio.h>
#include <string.h>

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

GPIO_TypeDef esc_soft_uart_host_gpiod;
TIM_TypeDef esc_soft_uart_host_tim5;
RCC_TypeDef esc_soft_uart_host_rcc;
SYSCFG_TypeDef esc_soft_uart_host_syscfg;
EXTI_TypeDef esc_soft_uart_host_exti;
volatile uint32_t uwTick;

static uint32_t s_fake_tick;

static void reset_host_hal(void)
{
    memset(&esc_soft_uart_host_gpiod, 0, sizeof(esc_soft_uart_host_gpiod));
    memset(&esc_soft_uart_host_tim5, 0, sizeof(esc_soft_uart_host_tim5));
    memset(&esc_soft_uart_host_rcc, 0, sizeof(esc_soft_uart_host_rcc));
    memset(&esc_soft_uart_host_syscfg, 0, sizeof(esc_soft_uart_host_syscfg));
    memset(&esc_soft_uart_host_exti, 0, sizeof(esc_soft_uart_host_exti));
    s_fake_tick = 0U;
    uwTick = 0U;
    GPIOD->IDR = ESC_SOFT_UART_STM32_RX_PIN_MASK;
}

void EscSoftUartHost_SetPriority(IRQn_Type irq, uint32_t priority,
                                 uint32_t subpriority)
{
    (void)irq;
    (void)priority;
    (void)subpriority;
}

void EscSoftUartHost_EnableIRQ(IRQn_Type irq)
{
    (void)irq;
}

void EscSoftUartHost_ClearExtiPending(uint32_t line)
{
    EXTI->PR &= ~line;
}

uint32_t HAL_GetTick(void)
{
    return s_fake_tick;
}

void vTaskDelay(uint32_t ticks)
{
    (void)ticks;
}

static void set_tick(uint32_t tick_ms)
{
    s_fake_tick = tick_ms;
    uwTick = tick_ms;
}

static void set_line(uint8_t high)
{
    if (high != 0U)
    {
        GPIOD->IDR |= ESC_SOFT_UART_STM32_RX_PIN_MASK;
    }
    else
    {
        GPIOD->IDR &= ~ESC_SOFT_UART_STM32_RX_PIN_MASK;
    }
}

static void trigger_start_edge(void)
{
    set_line(0U);
    EXTI->PR |= ESC_SOFT_UART_STM32_RX_PIN_MASK;
    (void)EscSoftUartStm32_HandleExti15Irq();
}

static void run_tim5_sample(uint8_t high)
{
    set_line(high);
    TIM5->CNT = TIM5->CCR1;
    TIM5->SR |= TIM_SR_CC1IF;
    EscSoftUartStm32_HandleTim5Irq();
}

static void feed_uart_byte(uint8_t byte)
{
    uint8_t index;

    trigger_start_edge();
    run_tim5_sample(0U);
    for (index = 0U; index < 8U; index++)
    {
        run_tim5_sample(((byte & (uint8_t)(1U << index)) != 0U) ? 1U : 0U);
    }
    run_tim5_sample(1U);
}

static void feed_uart_bytes(const uint8_t *data, size_t length)
{
    size_t index;

    for (index = 0U; index < length; index++)
    {
        feed_uart_byte(data[index]);
    }
}

static void feed_stop_low_fault(uint8_t byte)
{
    uint8_t index;

    trigger_start_edge();
    run_tim5_sample(0U);
    for (index = 0U; index < 8U; index++)
    {
        run_tim5_sample(((byte & (uint8_t)(1U << index)) != 0U) ? 1U : 0U);
    }
    run_tim5_sample(0U);
}

static int test_soft_uart_frame_reaches_parser(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;

    reset_host_hal();
    EscTelemetryStm32_Init();
    set_tick(10U);
    EXPECT_TRUE(EscTelemetryStm32_Start() == 1U);

    set_tick(77U);
    feed_uart_bytes(ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN);
    EscTelemetryStm32_Service();
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 77U);
    EXPECT_TRUE(snapshot.sample.state_candidate ==
                ESC_FE32_STATE_CANDIDATE_BRAKE);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.bytes_copied_from_receiver == ESC_FE32_FRAME_LEN);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 1U);
    EXPECT_TRUE(diagnostics.rx_error_count == 0U);
    return 0;
}

static int test_fault_invalidates_epoch_and_recovers_with_new_frame(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    uint32_t epoch_after_first_frame;

    reset_host_hal();
    EscTelemetryStm32_Init();
    set_tick(100U);
    EXPECT_TRUE(EscTelemetryStm32_Start() == 1U);

    set_tick(110U);
    feed_uart_bytes(ESC_FE32_FIXTURE_PEAK_DYN02, ESC_FE32_FRAME_LEN);
    EscTelemetryStm32_Service();
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    epoch_after_first_frame = snapshot.receive_epoch;

    set_tick(120U);
    feed_stop_low_fault(0xA5U);
    EscTelemetryStm32_Service();
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);

    set_tick(130U);
    feed_uart_bytes(ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN);
    EscTelemetryStm32_Service();
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 2U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 130U);
    EXPECT_TRUE(snapshot.receive_epoch != epoch_after_first_frame);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.rx_error_count == 1U);
    EXPECT_TRUE((diagnostics.last_rx_error_flags &
                 ESC_SOFT_UART_RX_FAULT_STOP_LOW) != 0UL);
    EXPECT_TRUE(diagnostics.receive_invalidations >= 2U);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 2U);
    return 0;
}

static int run_test(const char *name, int (*test_fn)(void))
{
    const int result = test_fn();
    if (result != 0)
    {
        fprintf(stderr, "FAIL: %s\n", name);
        return 1;
    }
    printf("PASS: %s\n", name);
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += run_test("test_soft_uart_frame_reaches_parser",
                         test_soft_uart_frame_reaches_parser);
    failures += run_test("test_fault_invalidates_epoch_and_recovers_with_new_frame",
                         test_fault_invalidates_epoch_and_recovers_with_new_frame);

    if (failures != 0)
    {
        return 1;
    }

    printf("test_esc_telemetry_stm32: 2 tests passed\n");
    return 0;
}
