#include "esc_soft_uart_stm32.h"

#include <string.h>

#if defined(ESC_SOFT_UART_STM32_HOST_TEST)
#include "esc_soft_uart_stm32_host.h"
#else
#include "main.h"
#endif

#define ESC_SOFT_UART_RING_LEN 256U
#define ESC_SOFT_UART_RING_MASK (ESC_SOFT_UART_RING_LEN - 1U)
#define ESC_SOFT_UART_EXTI_LINE ESC_SOFT_UART_STM32_RX_PIN_MASK
#define ESC_SOFT_UART_SYSCFG_EXTICR_INDEX 3U
#define ESC_SOFT_UART_SYSCFG_EXTICR_SHIFT 12U
#define ESC_SOFT_UART_SYSCFG_PORTD 3U

#if defined(__GNUC__)
#define ESC_SOFT_UART_INLINE static inline __attribute__((always_inline))
#elif defined(__CC_ARM)
#define ESC_SOFT_UART_INLINE static __forceinline
#else
#define ESC_SOFT_UART_INLINE static inline
#endif

#if (ESC_SOFT_UART_RING_LEN & (ESC_SOFT_UART_RING_LEN - 1U)) != 0U
#error "ESC_SOFT_UART_RING_LEN must be a power of two"
#endif

typedef struct
{
    uint8_t byte;
    uint32_t received_tick_ms;
    uint32_t output_context;
    uint32_t output_metadata;
} EscSoftUartRingItem_t;

#if defined(ESC_SOFT_UART_STM32_HOST_TEST)
typedef uint8_t EscSoftUartIrqState_t;

static EscSoftUartIrqState_t esc_soft_uart_enter_critical(void)
{
    return 0U;
}

static void esc_soft_uart_exit_critical(EscSoftUartIrqState_t state)
{
    (void)state;
}
#else
typedef uint32_t EscSoftUartIrqState_t;

static EscSoftUartIrqState_t esc_soft_uart_enter_critical(void)
{
    EscSoftUartIrqState_t state = __get_PRIMASK();

    __disable_irq();
    return state;
}

static void esc_soft_uart_exit_critical(EscSoftUartIrqState_t state)
{
    __set_PRIMASK(state);
}
#endif

static EscSoftUartRxCore_t s_core;
static EscSoftUartRingItem_t s_ring[ESC_SOFT_UART_RING_LEN];
static volatile uint16_t s_ring_head;
static volatile uint16_t s_ring_tail;
static volatile uint32_t s_fault_flags;
static volatile uint8_t s_enabled;
static uint32_t s_start_timer_tick;
static EscSoftUartStm32Diagnostics_t s_diagnostics;
static EscSoftUartStm32OutputContextProvider_t s_output_context_provider;
static const uint16_t s_sample_offsets[ESC_SOFT_UART_RX_SAMPLES_PER_FRAME] = {
    365U, 1094U, 1823U, 2552U, 3281U,
    4010U, 4740U, 5469U, 6198U, 6927U};

extern volatile uint32_t uwTick;

ESC_SOFT_UART_INLINE void esc_soft_uart_disable_exti(void)
{
    EXTI->IMR &= ~ESC_SOFT_UART_EXTI_LINE;
}

ESC_SOFT_UART_INLINE void esc_soft_uart_clear_exti_pending(void)
{
#if defined(ESC_SOFT_UART_STM32_HOST_TEST)
    EscSoftUartHost_ClearExtiPending(ESC_SOFT_UART_EXTI_LINE);
#else
    EXTI->PR = ESC_SOFT_UART_EXTI_LINE;
#endif
}

ESC_SOFT_UART_INLINE void esc_soft_uart_enable_exti(void)
{
    esc_soft_uart_clear_exti_pending();
    EXTI->IMR |= ESC_SOFT_UART_EXTI_LINE;
}

ESC_SOFT_UART_INLINE void esc_soft_uart_disable_compare(void)
{
    TIM5->DIER &= ~TIM_DIER_CC1IE;
}

ESC_SOFT_UART_INLINE void esc_soft_uart_clear_compare_flag(void)
{
    TIM5->SR = (uint32_t)(~TIM_SR_CC1IF);
}

ESC_SOFT_UART_INLINE void esc_soft_uart_schedule_sample(uint8_t sample_index)
{
    if (sample_index >= ESC_SOFT_UART_RX_SAMPLES_PER_FRAME)
    {
        sample_index = (uint8_t)(ESC_SOFT_UART_RX_SAMPLES_PER_FRAME - 1U);
    }
    TIM5->CCR1 = s_start_timer_tick + s_sample_offsets[sample_index];
    esc_soft_uart_clear_compare_flag();
    TIM5->DIER |= TIM_DIER_CC1IE;
}

ESC_SOFT_UART_INLINE uint8_t esc_soft_uart_read_rx_level(void)
{
    return ((GPIOD->IDR & ESC_SOFT_UART_STM32_RX_PIN_MASK) != 0UL) ? 1U : 0U;
}

static void esc_soft_uart_clear_ring(void)
{
    s_ring_head = 0U;
    s_ring_tail = 0U;
}

static void esc_soft_uart_set_fault(uint32_t fault_flags)
{
    s_fault_flags |= fault_flags;
    s_diagnostics.fault_events++;
    if ((fault_flags & ESC_SOFT_UART_RX_FAULT_STOP_LOW) != 0UL)
    {
        s_diagnostics.stop_low_faults++;
    }
    if ((fault_flags & ESC_SOFT_UART_RX_FAULT_RING_OVERFLOW) != 0UL)
    {
        s_diagnostics.ring_overflows++;
    }
    if ((fault_flags & ESC_SOFT_UART_RX_FAULT_TIMING_LATE) != 0UL)
    {
        s_diagnostics.timing_late_faults++;
    }

    EscSoftUartRxCore_Init(&s_core);
    esc_soft_uart_disable_compare();
    esc_soft_uart_disable_exti();
}

static void esc_soft_uart_filter_start_glitch(void)
{
    s_diagnostics.start_glitches++;
    EscSoftUartRxCore_Init(&s_core);
    esc_soft_uart_disable_compare();
    if (s_enabled != 0U)
    {
        esc_soft_uart_enable_exti();
    }
}

ESC_SOFT_UART_INLINE void esc_soft_uart_push_byte(uint8_t byte)
{
    uint16_t next_head = (uint16_t)((s_ring_head + 1U) &
                                    ESC_SOFT_UART_RING_MASK);
    EscTelemetryOutputContext_t output_context;

    if (next_head == s_ring_tail)
    {
        s_diagnostics.bytes_dropped++;
        esc_soft_uart_set_fault(ESC_SOFT_UART_RX_FAULT_RING_OVERFLOW);
        return;
    }

    memset(&output_context, 0, sizeof(output_context));
    if (s_output_context_provider !=
        (EscSoftUartStm32OutputContextProvider_t)0)
    {
        s_output_context_provider(&output_context);
    }

    s_ring[s_ring_head].byte = byte;
    s_ring[s_ring_head].received_tick_ms = uwTick;
    s_ring[s_ring_head].output_context = output_context.output_context;
    s_ring[s_ring_head].output_metadata = output_context.output_metadata;
    s_ring_head = next_head;
    s_diagnostics.bytes_received++;
}

static void esc_soft_uart_begin_from_current_low(void)
{
    if (s_enabled == 0U || s_fault_flags != 0UL)
    {
        return;
    }

    if (esc_soft_uart_read_rx_level() != 0U)
    {
        esc_soft_uart_filter_start_glitch();
        return;
    }

    esc_soft_uart_disable_exti();
    s_start_timer_tick = TIM5->CNT;
    s_diagnostics.start_edges++;
    EscSoftUartRxCore_Start(&s_core);
    esc_soft_uart_schedule_sample(0U);
}

ESC_SOFT_UART_INLINE void esc_soft_uart_finish_byte_or_fault(
    const EscSoftUartRxSampleResult_t *result)
{
    if (result->status == ESC_SOFT_UART_RX_SAMPLE_FAULT)
    {
        if (result->fault_flags == ESC_SOFT_UART_RX_FAULT_START_GLITCH)
        {
            esc_soft_uart_filter_start_glitch();
            if (esc_soft_uart_read_rx_level() == 0U)
            {
                esc_soft_uart_begin_from_current_low();
            }
            return;
        }
        esc_soft_uart_set_fault(result->fault_flags);
        return;
    }

    if (result->status == ESC_SOFT_UART_RX_SAMPLE_BYTE)
    {
        esc_soft_uart_push_byte(result->byte);
        esc_soft_uart_disable_compare();
        if (s_fault_flags == 0UL)
        {
            esc_soft_uart_enable_exti();
            if (esc_soft_uart_read_rx_level() == 0U)
            {
                esc_soft_uart_begin_from_current_low();
            }
        }
        return;
    }

    esc_soft_uart_schedule_sample(EscSoftUartRxCore_GetSampleIndex(&s_core));
}

void EscSoftUartStm32_Init(void)
{
    EscSoftUartRxCore_Init(&s_core);
    memset(s_ring, 0, sizeof(s_ring));
    memset(&s_diagnostics, 0, sizeof(s_diagnostics));
    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_fault_flags = 0UL;
    s_enabled = 0U;
    s_start_timer_tick = 0UL;
    s_output_context_provider =
        (EscSoftUartStm32OutputContextProvider_t)0;
}

void EscSoftUartStm32_SetOutputContextProvider(
    EscSoftUartStm32OutputContextProvider_t provider)
{
    EscSoftUartIrqState_t state = esc_soft_uart_enter_critical();

    s_output_context_provider = provider;
    esc_soft_uart_exit_critical(state);
}

uint8_t EscSoftUartStm32_Start(uint32_t tick_ms)
{
    EscSoftUartIrqState_t state;

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    GPIOD->MODER &= ~(3UL << 30);
    GPIOD->PUPDR = (GPIOD->PUPDR & ~(3UL << 30)) | (1UL << 30);

    SYSCFG->EXTICR[ESC_SOFT_UART_SYSCFG_EXTICR_INDEX] =
        (SYSCFG->EXTICR[ESC_SOFT_UART_SYSCFG_EXTICR_INDEX] &
         ~(0xFUL << ESC_SOFT_UART_SYSCFG_EXTICR_SHIFT)) |
        (ESC_SOFT_UART_SYSCFG_PORTD << ESC_SOFT_UART_SYSCFG_EXTICR_SHIFT);

    EXTI->RTSR &= ~ESC_SOFT_UART_EXTI_LINE;
    EXTI->FTSR |= ESC_SOFT_UART_EXTI_LINE;
    esc_soft_uart_clear_exti_pending();

    TIM5->CR1 = 0UL;
    TIM5->PSC = 0UL;
    TIM5->ARR = 0xFFFFFFFFUL;
    TIM5->CNT = 0UL;
    TIM5->DIER = 0UL;
    TIM5->SR = 0UL;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM5_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4U, 0U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    state = esc_soft_uart_enter_critical();
    EscSoftUartRxCore_Init(&s_core);
    esc_soft_uart_clear_ring();
    s_fault_flags = 0UL;
    s_enabled = 1U;
    (void)tick_ms;
    esc_soft_uart_enable_exti();
    esc_soft_uart_exit_critical(state);

    return 1U;
}

void EscSoftUartStm32_UpdateTickBase(uint32_t tick_ms)
{
    (void)tick_ms;
}

uint32_t EscSoftUartStm32_TakeFaults(void)
{
    uint32_t faults;
    EscSoftUartIrqState_t state = esc_soft_uart_enter_critical();

    faults = s_fault_flags;
    if (faults != 0UL)
    {
        s_fault_flags = 0UL;
        esc_soft_uart_clear_ring();
        EscSoftUartRxCore_Init(&s_core);
        if (s_enabled != 0U)
        {
            esc_soft_uart_enable_exti();
        }
    }

    esc_soft_uart_exit_critical(state);
    return faults;
}

uint8_t EscSoftUartStm32_ReadByte(EscSoftUartStm32Byte_t *item)
{
    uint8_t has_item = 0U;
    EscSoftUartIrqState_t state;

    if (item == (EscSoftUartStm32Byte_t *)0)
    {
        return 0U;
    }

    state = esc_soft_uart_enter_critical();
    if (s_ring_tail != s_ring_head)
    {
        item->byte = s_ring[s_ring_tail].byte;
        item->received_tick_ms = s_ring[s_ring_tail].received_tick_ms;
        item->output_context = s_ring[s_ring_tail].output_context;
        item->output_metadata = s_ring[s_ring_tail].output_metadata;
        s_ring_tail = (uint16_t)((s_ring_tail + 1U) &
                                 ESC_SOFT_UART_RING_MASK);
        has_item = 1U;
    }
    esc_soft_uart_exit_critical(state);

    return has_item;
}

void EscSoftUartStm32_GetDiagnostics(
    EscSoftUartStm32Diagnostics_t *diagnostics)
{
    EscSoftUartIrqState_t state;

    if (diagnostics == (EscSoftUartStm32Diagnostics_t *)0)
    {
        return;
    }
    state = esc_soft_uart_enter_critical();
    *diagnostics = s_diagnostics;
    esc_soft_uart_exit_critical(state);
}

uint8_t EscSoftUartStm32_HandleExti15Irq(void)
{
    if ((EXTI->PR & ESC_SOFT_UART_EXTI_LINE) == 0UL)
    {
        return 0U;
    }

    esc_soft_uart_clear_exti_pending();
    if ((EXTI->IMR & ESC_SOFT_UART_EXTI_LINE) == 0UL ||
        s_enabled == 0U ||
        s_fault_flags != 0UL)
    {
        return 1U;
    }

    esc_soft_uart_begin_from_current_low();
    return 1U;
}

void EscSoftUartStm32_HandleTim5Irq(void)
{
    EscSoftUartRxSampleResult_t result;
    uint32_t now_tick;
    uint32_t expected_tick;
    uint32_t late_ticks;
    uint8_t level;

    if ((TIM5->SR & TIM_SR_CC1IF) == 0UL ||
        (TIM5->DIER & TIM_DIER_CC1IE) == 0UL)
    {
        return;
    }

    esc_soft_uart_clear_compare_flag();
    now_tick = TIM5->CNT;
    expected_tick = TIM5->CCR1;
    late_ticks = now_tick - expected_tick;
    if (late_ticks > ESC_SOFT_UART_STM32_LATE_TICKS)
    {
        esc_soft_uart_set_fault(ESC_SOFT_UART_RX_FAULT_TIMING_LATE);
        return;
    }

    level = esc_soft_uart_read_rx_level();
    result = EscSoftUartRxCore_Sample(&s_core, level);
    (void)now_tick;
    esc_soft_uart_finish_byte_or_fault(&result);
}
