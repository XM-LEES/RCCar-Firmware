#include "esc_soft_uart_rx.h"

#include <stdio.h>

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

static uint8_t line_level_for_byte(uint8_t byte,
                                   double ticks_since_start,
                                   double tx_bit_ticks)
{
    int bit_slot;

    if (ticks_since_start < 0.0)
    {
        return 1U;
    }

    bit_slot = (int)(ticks_since_start / tx_bit_ticks);
    if (bit_slot == 0)
    {
        return 0U;
    }
    if (bit_slot >= 1 && bit_slot <= 8)
    {
        return ((byte & (uint8_t)(1U << (uint8_t)(bit_slot - 1))) != 0U) ?
            1U : 0U;
    }
    return 1U;
}

static int decode_byte(uint8_t expected,
                       double baud_error_ratio,
                       double irq_delay_ticks)
{
    EscSoftUartRxCore_t core;
    EscSoftUartRxSampleResult_t result;
    const double tx_bit_ticks =
        (double)EscSoftUartRx_GetBitTicksRounded() *
        (1.0 + baud_error_ratio);
    uint8_t index;

    EscSoftUartRxCore_Init(&core);
    EscSoftUartRxCore_Start(&core);
    for (index = 0U; index < ESC_SOFT_UART_RX_SAMPLES_PER_FRAME; index++)
    {
        const double sample_ticks =
            (double)EscSoftUartRx_GetSampleOffsetTicks(index) +
            irq_delay_ticks;
        const uint8_t level =
            line_level_for_byte(expected, sample_ticks, tx_bit_ticks);

        result = EscSoftUartRxCore_Sample(&core, level);
        if (index < (ESC_SOFT_UART_RX_SAMPLES_PER_FRAME - 1U))
        {
            EXPECT_TRUE(result.status == ESC_SOFT_UART_RX_SAMPLE_CONTINUE);
        }
    }

    EXPECT_TRUE(result.status == ESC_SOFT_UART_RX_SAMPLE_BYTE);
    EXPECT_TRUE(result.byte == expected);
    EXPECT_TRUE(EscSoftUartRxCore_IsReceiving(&core) == 0U);
    return 0;
}

static int all_256_bytes_decode_lsb_8n1(void)
{
    uint16_t value;

    for (value = 0U; value <= 255U; value++)
    {
        if (decode_byte((uint8_t)value, 0.0, 0.0) != 0)
        {
            fprintf(stderr, "failed byte 0x%02x\n", value);
            return 1;
        }
    }
    return 0;
}

static int start_and_stop_faults_are_reported(void)
{
    EscSoftUartRxCore_t core;
    EscSoftUartRxSampleResult_t result;
    uint8_t index;

    EscSoftUartRxCore_Init(&core);
    EscSoftUartRxCore_Start(&core);
    result = EscSoftUartRxCore_Sample(&core, 1U);
    EXPECT_TRUE(result.status == ESC_SOFT_UART_RX_SAMPLE_FAULT);
    EXPECT_TRUE((result.fault_flags & ESC_SOFT_UART_RX_FAULT_START_GLITCH) !=
                0UL);
    EXPECT_TRUE(EscSoftUartRxCore_IsReceiving(&core) == 0U);

    EscSoftUartRxCore_Start(&core);
    result = EscSoftUartRxCore_Sample(&core, 0U);
    EXPECT_TRUE(result.status == ESC_SOFT_UART_RX_SAMPLE_CONTINUE);
    for (index = 1U; index <= 8U; index++)
    {
        result = EscSoftUartRxCore_Sample(&core, 0U);
        EXPECT_TRUE(result.status == ESC_SOFT_UART_RX_SAMPLE_CONTINUE);
    }
    result = EscSoftUartRxCore_Sample(&core, 0U);
    EXPECT_TRUE(result.status == ESC_SOFT_UART_RX_SAMPLE_FAULT);
    EXPECT_TRUE((result.fault_flags & ESC_SOFT_UART_RX_FAULT_STOP_LOW) != 0UL);
    EXPECT_TRUE(EscSoftUartRxCore_IsReceiving(&core) == 0U);

    return 0;
}

static int back_to_back_32_byte_burst_decodes(void)
{
    uint8_t index;

    for (index = 0U; index < 32U; index++)
    {
        const uint8_t value = (uint8_t)(index * 7U + 3U);
        if (decode_byte(value, 0.0, 0.0) != 0)
        {
            return 1;
        }
    }
    return 0;
}

static int fractional_offsets_match_84mhz_115200(void)
{
    static const uint32_t expected_offsets[
        ESC_SOFT_UART_RX_SAMPLES_PER_FRAME] = {
        365U, 1094U, 1823U, 2552U, 3281U,
        4010U, 4740U, 5469U, 6198U, 6927U};
    uint8_t index;

    for (index = 0U; index < ESC_SOFT_UART_RX_SAMPLES_PER_FRAME; index++)
    {
        EXPECT_TRUE(EscSoftUartRx_GetSampleOffsetTicks(index) ==
                    expected_offsets[index]);
        if (index != 0U)
        {
            const uint32_t gap =
                expected_offsets[index] - expected_offsets[index - 1U];
            EXPECT_TRUE(gap == 729U || gap == 730U);
        }
    }

    EXPECT_TRUE(EscSoftUartRx_GetBitTicksRounded() == 729U);
    return 0;
}

static int timer_offsets_are_wrap_safe(void)
{
    const uint32_t start = 0xFFFFFF10UL;
    uint8_t index;

    for (index = 1U; index < ESC_SOFT_UART_RX_SAMPLES_PER_FRAME; index++)
    {
        const uint32_t previous =
            start + EscSoftUartRx_GetSampleOffsetTicks((uint8_t)(index - 1U));
        const uint32_t current =
            start + EscSoftUartRx_GetSampleOffsetTicks(index);
        const uint32_t expected_delta =
            EscSoftUartRx_GetSampleOffsetTicks(index) -
            EscSoftUartRx_GetSampleOffsetTicks((uint8_t)(index - 1U));

        EXPECT_TRUE((uint32_t)(current - previous) == expected_delta);
    }
    return 0;
}

static int baud_error_and_irq_delay_margin_decodes(void)
{
    uint16_t value;

    for (value = 0U; value <= 255U; value++)
    {
        if (decode_byte((uint8_t)value, 0.015, 80.0) != 0 ||
            decode_byte((uint8_t)value, -0.015, 80.0) != 0)
        {
            fprintf(stderr, "failed margin byte 0x%02x\n", value);
            return 1;
        }
    }
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

    failures += run_test("all_256_bytes_decode_lsb_8n1",
                         all_256_bytes_decode_lsb_8n1);
    failures += run_test("start_and_stop_faults_are_reported",
                         start_and_stop_faults_are_reported);
    failures += run_test("back_to_back_32_byte_burst_decodes",
                         back_to_back_32_byte_burst_decodes);
    failures += run_test("fractional_offsets_match_84mhz_115200",
                         fractional_offsets_match_84mhz_115200);
    failures += run_test("timer_offsets_are_wrap_safe",
                         timer_offsets_are_wrap_safe);
    failures += run_test("baud_error_and_irq_delay_margin_decodes",
                         baud_error_and_irq_delay_margin_decodes);

    if (failures != 0)
    {
        return 1;
    }

    printf("test_esc_soft_uart_rx: 6 tests passed\n");
    return 0;
}
