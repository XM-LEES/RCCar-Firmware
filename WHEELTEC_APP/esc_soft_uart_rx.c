#include "esc_soft_uart_rx.h"

#define ESC_SOFT_UART_RX_BIT_NUMERATOR 4375UL
#define ESC_SOFT_UART_RX_BIT_DENOMINATOR 6UL

void EscSoftUartRxCore_Init(EscSoftUartRxCore_t *core)
{
    if (core == (EscSoftUartRxCore_t *)0)
    {
        return;
    }

    core->receiving = 0U;
    core->sample_index = 0U;
    core->byte = 0U;
}

void EscSoftUartRxCore_Start(EscSoftUartRxCore_t *core)
{
    if (core == (EscSoftUartRxCore_t *)0)
    {
        return;
    }

    core->receiving = 1U;
    core->sample_index = 0U;
    core->byte = 0U;
}

EscSoftUartRxSampleResult_t EscSoftUartRxCore_Sample(
    EscSoftUartRxCore_t *core,
    uint8_t line_high)
{
    EscSoftUartRxSampleResult_t result;
    uint8_t data_bit_index;

    result.status = ESC_SOFT_UART_RX_SAMPLE_CONTINUE;
    result.fault_flags = ESC_SOFT_UART_RX_FAULT_NONE;
    result.byte = 0U;

    if (core == (EscSoftUartRxCore_t *)0 || core->receiving == 0U)
    {
        result.status = ESC_SOFT_UART_RX_SAMPLE_FAULT;
        result.fault_flags = ESC_SOFT_UART_RX_FAULT_START_GLITCH;
        return result;
    }

    if (core->sample_index == 0U)
    {
        if (line_high != 0U)
        {
            EscSoftUartRxCore_Init(core);
            result.status = ESC_SOFT_UART_RX_SAMPLE_FAULT;
            result.fault_flags = ESC_SOFT_UART_RX_FAULT_START_GLITCH;
            return result;
        }
        core->sample_index = 1U;
        return result;
    }

    if (core->sample_index <= 8U)
    {
        data_bit_index = (uint8_t)(core->sample_index - 1U);
        if (line_high != 0U)
        {
            core->byte |= (uint8_t)(1U << data_bit_index);
        }
        core->sample_index++;
        return result;
    }

    if (line_high == 0U)
    {
        EscSoftUartRxCore_Init(core);
        result.status = ESC_SOFT_UART_RX_SAMPLE_FAULT;
        result.fault_flags = ESC_SOFT_UART_RX_FAULT_STOP_LOW;
        return result;
    }

    result.status = ESC_SOFT_UART_RX_SAMPLE_BYTE;
    result.byte = core->byte;
    EscSoftUartRxCore_Init(core);
    return result;
}

uint8_t EscSoftUartRxCore_IsReceiving(const EscSoftUartRxCore_t *core)
{
    if (core == (const EscSoftUartRxCore_t *)0)
    {
        return 0U;
    }
    return core->receiving;
}

uint8_t EscSoftUartRxCore_GetSampleIndex(const EscSoftUartRxCore_t *core)
{
    if (core == (const EscSoftUartRxCore_t *)0)
    {
        return 0U;
    }
    return core->sample_index;
}

uint32_t EscSoftUartRx_GetSampleOffsetTicks(uint8_t sample_index)
{
    uint32_t numerator;

    if (sample_index >= ESC_SOFT_UART_RX_SAMPLES_PER_FRAME)
    {
        sample_index = (uint8_t)(ESC_SOFT_UART_RX_SAMPLES_PER_FRAME - 1U);
    }

    numerator = ESC_SOFT_UART_RX_BIT_NUMERATOR *
                (uint32_t)((2U * (uint32_t)sample_index) + 1U);
    return (numerator + ESC_SOFT_UART_RX_BIT_DENOMINATOR) /
           (2UL * ESC_SOFT_UART_RX_BIT_DENOMINATOR);
}

uint32_t EscSoftUartRx_GetBitTicksRounded(void)
{
    return (ESC_SOFT_UART_RX_BIT_NUMERATOR +
            (ESC_SOFT_UART_RX_BIT_DENOMINATOR / 2UL)) /
           ESC_SOFT_UART_RX_BIT_DENOMINATOR;
}
