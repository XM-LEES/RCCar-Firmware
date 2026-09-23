#include "main.h"
#include "stm32f4xx_it.h"
#include "esc_soft_uart_stm32.h"
#include "esc_soft_uart_stm32_host.h"

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

enum trace_event
{
    TRACE_PD15_CLEAR = 1,
    TRACE_HALL_A,
    TRACE_HALL_B
};

GPIO_TypeDef esc_soft_uart_host_gpiod;
TIM_TypeDef esc_soft_uart_host_tim5;
RCC_TypeDef esc_soft_uart_host_rcc;
SYSCFG_TypeDef esc_soft_uart_host_syscfg;
EXTI_TypeDef esc_soft_uart_host_exti;
volatile uint32_t uwTick;

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_tx;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;

static enum trace_event s_trace[8];
static size_t s_trace_len;
static uint32_t s_tim5_priority;
static uint32_t s_exti_priority;
static uint32_t s_tim5_enable_calls;
static uint32_t s_exti_enable_calls;
static uint32_t s_hall_a_calls;
static uint32_t s_hall_b_calls;
static uint32_t s_hal_tim_calls;
static uint32_t s_hal_uart_calls;
static uint32_t s_hal_dma_calls;
static EscTelemetryOutputContext_t s_provider_context;

static void push_trace(enum trace_event event)
{
    if (s_trace_len < (sizeof(s_trace) / sizeof(s_trace[0])))
    {
        s_trace[s_trace_len] = event;
        s_trace_len++;
    }
}

static void clear_trace(void)
{
    memset(s_trace, 0, sizeof(s_trace));
    s_trace_len = 0U;
}

static void reset_irq_fixture(void)
{
    memset(&esc_soft_uart_host_gpiod, 0, sizeof(esc_soft_uart_host_gpiod));
    memset(&esc_soft_uart_host_tim5, 0, sizeof(esc_soft_uart_host_tim5));
    memset(&esc_soft_uart_host_rcc, 0, sizeof(esc_soft_uart_host_rcc));
    memset(&esc_soft_uart_host_syscfg, 0, sizeof(esc_soft_uart_host_syscfg));
    memset(&esc_soft_uart_host_exti, 0, sizeof(esc_soft_uart_host_exti));
    clear_trace();
    s_tim5_priority = 99U;
    s_exti_priority = 99U;
    s_tim5_enable_calls = 0U;
    s_exti_enable_calls = 0U;
    s_hall_a_calls = 0U;
    s_hall_b_calls = 0U;
    s_hal_tim_calls = 0U;
    s_hal_uart_calls = 0U;
    s_hal_dma_calls = 0U;
    memset(&s_provider_context, 0, sizeof(s_provider_context));
    uwTick = 0U;
    GPIOD->IDR = ESC_SOFT_UART_STM32_RX_PIN_MASK;
    EscSoftUartStm32_Init();
}

static void output_context_provider(
    EscTelemetryOutputContext_t *output_context)
{
    if (output_context != NULL)
    {
        *output_context = s_provider_context;
    }
}

void EscSoftUartHost_SetPriority(IRQn_Type irq, uint32_t priority,
                                 uint32_t subpriority)
{
    (void)subpriority;
    if (irq == TIM5_IRQn)
    {
        s_tim5_priority = priority;
    }
    else if (irq == EXTI15_10_IRQn)
    {
        s_exti_priority = priority;
    }
}

void EscSoftUartHost_EnableIRQ(IRQn_Type irq)
{
    if (irq == TIM5_IRQn)
    {
        s_tim5_enable_calls++;
    }
    else if (irq == EXTI15_10_IRQn)
    {
        s_exti_enable_calls++;
    }
}

void EscSoftUartHost_ClearExtiPending(uint32_t line)
{
    if ((line & ESC_SOFT_UART_STM32_RX_PIN_MASK) != 0UL &&
        (EXTI->PR & ESC_SOFT_UART_STM32_RX_PIN_MASK) != 0UL)
    {
        push_trace(TRACE_PD15_CLEAR);
    }
    EXTI->PR &= ~line;
}

uint32_t IrqStub_ReadReg(volatile uint32_t *reg)
{
    return *reg;
}

uint32_t IrqStub_GetUartFlag(UART_HandleTypeDef *huart, uint32_t flag)
{
    return ((huart->Instance->SR & flag) != 0U) ? SET : RESET;
}

uint32_t IrqStub_GetUartItSource(UART_HandleTypeDef *huart, uint32_t source)
{
    return ((huart->Instance->CR1 & source) != 0U) ? SET : RESET;
}

uint32_t IrqStub_GetExtiIt(uint32_t pin)
{
    return ((EXTI->PR & pin) != 0UL) ? SET : RESET;
}

void IrqStub_ClearIdleFlag(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    (void)huart;
    s_hal_uart_calls++;
}

void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    s_hal_dma_calls++;
}

void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
    (void)htim;
    s_hal_tim_calls++;
}

void HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    EXTI->PR &= ~(uint32_t)pin;
    if (pin == HallA_Pin)
    {
        push_trace(TRACE_HALL_A);
        s_hall_a_calls++;
    }
    else if (pin == HallB_Pin)
    {
        push_trace(TRACE_HALL_B);
        s_hall_b_calls++;
    }
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
    EXTI15_10_IRQHandler();
}

static void run_tim5_sample(uint8_t high, uint32_t lateness_ticks)
{
    set_line(high);
    TIM5->CNT = TIM5->CCR1 + lateness_ticks;
    TIM5->SR |= TIM_SR_CC1IF;
    TIM5_IRQHandler();
}

static void feed_byte(uint8_t byte, uint32_t lateness_ticks)
{
    uint8_t index;

    trigger_start_edge();
    run_tim5_sample(0U, lateness_ticks);
    for (index = 0U; index < 8U; index++)
    {
        run_tim5_sample(((byte & (uint8_t)(1U << index)) != 0U) ? 1U : 0U,
                        lateness_ticks);
    }
    run_tim5_sample(1U, lateness_ticks);
}

static void feed_stop_low_fault(uint8_t byte)
{
    uint8_t index;

    trigger_start_edge();
    run_tim5_sample(0U, 0U);
    for (index = 0U; index < 8U; index++)
    {
        run_tim5_sample(((byte & (uint8_t)(1U << index)) != 0U) ? 1U : 0U,
                        0U);
    }
    run_tim5_sample(0U, 0U);
}

static int start_configures_pd15_tim5_priorities(void)
{
    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(12U) == 1U);
    EXPECT_TRUE((RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN) != 0UL);
    EXPECT_TRUE((RCC->APB1ENR & RCC_APB1ENR_TIM5EN) != 0UL);
    EXPECT_TRUE((RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN) != 0UL);
    EXPECT_TRUE((GPIOD->PUPDR & (3UL << 30)) == (1UL << 30));
    EXPECT_TRUE((EXTI->FTSR & ESC_SOFT_UART_STM32_RX_PIN_MASK) != 0UL);
    EXPECT_TRUE((EXTI->IMR & ESC_SOFT_UART_STM32_RX_PIN_MASK) != 0UL);
    EXPECT_TRUE((TIM5->CR1 & TIM_CR1_CEN) != 0UL);
    EXPECT_TRUE(s_tim5_priority == 3U);
    EXPECT_TRUE(s_exti_priority == 4U);
    EXPECT_TRUE(s_tim5_enable_calls == 1U);
    EXPECT_TRUE(s_exti_enable_calls == 1U);
    return 0;
}

static int pd15_exti_runs_before_hall_and_keeps_hall_pending(void)
{
    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(20U) == 1U);
    clear_trace();

    set_line(0U);
    EXTI->PR |= ESC_SOFT_UART_STM32_RX_PIN_MASK | HallA_Pin | HallB_Pin;
    EXTI15_10_IRQHandler();

    EXPECT_TRUE((TIM5->DIER & TIM_DIER_CC1IE) != 0UL);
    EXPECT_TRUE(s_hall_a_calls == 1U);
    EXPECT_TRUE(s_hall_b_calls == 1U);
    EXPECT_TRUE(s_trace_len == 3U);
    EXPECT_TRUE(s_trace[0] == TRACE_PD15_CLEAR);
    EXPECT_TRUE(s_trace[1] == TRACE_HALL_A);
    EXPECT_TRUE(s_trace[2] == TRACE_HALL_B);
    return 0;
}

static int tim5_irq_decodes_byte_without_hal_handlers(void)
{
    EscSoftUartStm32Byte_t item;

    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(30U) == 1U);
    uwTick = 31U;
    feed_byte(0xA5U, 0U);

    EXPECT_TRUE(EscSoftUartStm32_ReadByte(&item) == 1U);
    EXPECT_TRUE(item.byte == 0xA5U);
    EXPECT_TRUE(item.received_tick_ms == 31U);
    EXPECT_TRUE(s_hal_tim_calls == 0U);
    EXPECT_TRUE(s_hal_uart_calls == 0U);
    EXPECT_TRUE(s_hal_dma_calls == 0U);
    return 0;
}

static int tim5_irq_captures_coherent_output_metadata(void)
{
    EscSoftUartStm32Byte_t item;

    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(32U) == 1U);
    s_provider_context.output_context =
        (5UL << ESC_TELEMETRY_CONTEXT_SOURCE_SHIFT) | 1600UL;
    s_provider_context.output_metadata =
        ESC_TELEMETRY_METADATA_AUTO_CONTEXT_MASK |
        (uint32_t)ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE |
        (123UL << ESC_TELEMETRY_METADATA_SESSION_SHIFT);
    EscSoftUartStm32_SetOutputContextProvider(output_context_provider);

    uwTick = 33U;
    feed_byte(0x3CU, 0U);

    EXPECT_TRUE(EscSoftUartStm32_ReadByte(&item) == 1U);
    EXPECT_TRUE(item.byte == 0x3CU);
    EXPECT_TRUE(item.output_context == s_provider_context.output_context);
    EXPECT_TRUE(item.output_metadata == s_provider_context.output_metadata);
    return 0;
}

static int start_glitch_rearms_without_fault_or_ring_drop(void)
{
    EscSoftUartStm32Byte_t item;
    EscSoftUartStm32Diagnostics_t diagnostics;
    uint32_t faults;

    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(35U) == 1U);
    uwTick = 36U;
    feed_byte(0x11U, 0U);

    trigger_start_edge();
    run_tim5_sample(1U, 0U);
    faults = EscSoftUartStm32_TakeFaults();
    EXPECT_TRUE((faults & ESC_SOFT_UART_RX_FAULT_START_GLITCH) == 0UL);
    EXPECT_TRUE(faults == 0UL);
    EscSoftUartStm32_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.start_glitches == 1U);
    EXPECT_TRUE(diagnostics.fault_events == 0U);
    EXPECT_TRUE((EXTI->IMR & ESC_SOFT_UART_STM32_RX_PIN_MASK) != 0UL);

    uwTick = 37U;
    feed_byte(0x22U, 0U);
    EXPECT_TRUE(EscSoftUartStm32_ReadByte(&item) == 1U);
    EXPECT_TRUE(item.byte == 0x11U);
    EXPECT_TRUE(item.received_tick_ms == 36U);
    EXPECT_TRUE(EscSoftUartStm32_ReadByte(&item) == 1U);
    EXPECT_TRUE(item.byte == 0x22U);
    EXPECT_TRUE(item.received_tick_ms == 37U);
    return 0;
}

static int stop_low_fault_drops_ring_and_recovers(void)
{
    EscSoftUartStm32Byte_t item;
    uint32_t faults;

    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(40U) == 1U);
    feed_stop_low_fault(0x55U);
    faults = EscSoftUartStm32_TakeFaults();
    EXPECT_TRUE((faults & ESC_SOFT_UART_RX_FAULT_STOP_LOW) != 0UL);
    EXPECT_TRUE(EscSoftUartStm32_ReadByte(&item) == 0U);

    uwTick = 43U;
    feed_byte(0x5AU, 0U);
    EXPECT_TRUE(EscSoftUartStm32_ReadByte(&item) == 1U);
    EXPECT_TRUE(item.byte == 0x5AU);
    EXPECT_TRUE(item.received_tick_ms == 43U);
    return 0;
}

static int late_compare_fault_is_reported(void)
{
    uint32_t faults;

    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(50U) == 1U);
    trigger_start_edge();
    run_tim5_sample(0U, ESC_SOFT_UART_STM32_LATE_TICKS + 1U);
    faults = EscSoftUartStm32_TakeFaults();
    EXPECT_TRUE((faults & ESC_SOFT_UART_RX_FAULT_TIMING_LATE) != 0UL);
    return 0;
}

static int ring_overflow_reports_fault(void)
{
    uint16_t index;
    uint32_t faults;

    reset_irq_fixture();
    EXPECT_TRUE(EscSoftUartStm32_Start(60U) == 1U);
    for (index = 0U; index < 256U; index++)
    {
        feed_byte((uint8_t)index, 0U);
    }
    faults = EscSoftUartStm32_TakeFaults();
    EXPECT_TRUE((faults & ESC_SOFT_UART_RX_FAULT_RING_OVERFLOW) != 0UL);
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

    failures += run_test("start_configures_pd15_tim5_priorities",
                         start_configures_pd15_tim5_priorities);
    failures += run_test("pd15_exti_runs_before_hall_and_keeps_hall_pending",
                         pd15_exti_runs_before_hall_and_keeps_hall_pending);
    failures += run_test("tim5_irq_decodes_byte_without_hal_handlers",
                         tim5_irq_decodes_byte_without_hal_handlers);
    failures += run_test("tim5_irq_captures_coherent_output_metadata",
                         tim5_irq_captures_coherent_output_metadata);
    failures += run_test("start_glitch_rearms_without_fault_or_ring_drop",
                         start_glitch_rearms_without_fault_or_ring_drop);
    failures += run_test("stop_low_fault_drops_ring_and_recovers",
                         stop_low_fault_drops_ring_and_recovers);
    failures += run_test("late_compare_fault_is_reported",
                         late_compare_fault_is_reported);
    failures += run_test("ring_overflow_reports_fault",
                         ring_overflow_reports_fault);

    if (failures != 0)
    {
        return 1;
    }

    printf("test_soft_uart_irq: 8 tests passed\n");
    return 0;
}
