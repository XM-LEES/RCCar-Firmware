#ifndef __ESC_SOFT_UART_RX_H
#define __ESC_SOFT_UART_RX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESC_SOFT_UART_RX_TIMER_HZ 84000000UL
#define ESC_SOFT_UART_RX_BAUD 115200UL
#define ESC_SOFT_UART_RX_SAMPLES_PER_FRAME 10U

typedef enum
{
    ESC_SOFT_UART_RX_SAMPLE_CONTINUE = 0,
    ESC_SOFT_UART_RX_SAMPLE_BYTE,
    ESC_SOFT_UART_RX_SAMPLE_FAULT
} EscSoftUartRxSampleStatus_t;

typedef enum
{
    ESC_SOFT_UART_RX_FAULT_NONE = 0x00000000UL,
    ESC_SOFT_UART_RX_FAULT_START_GLITCH = 0x00000001UL,
    ESC_SOFT_UART_RX_FAULT_STOP_LOW = 0x00000002UL,
    ESC_SOFT_UART_RX_FAULT_RING_OVERFLOW = 0x00000004UL,
    ESC_SOFT_UART_RX_FAULT_TIMING_LATE = 0x00000008UL
} EscSoftUartRxFault_t;

typedef struct
{
    uint8_t receiving;
    uint8_t sample_index;
    uint8_t byte;
} EscSoftUartRxCore_t;

typedef struct
{
    EscSoftUartRxSampleStatus_t status;
    uint32_t fault_flags;
    uint8_t byte;
} EscSoftUartRxSampleResult_t;

void EscSoftUartRxCore_Init(EscSoftUartRxCore_t *core);
void EscSoftUartRxCore_Start(EscSoftUartRxCore_t *core);
EscSoftUartRxSampleResult_t EscSoftUartRxCore_Sample(
    EscSoftUartRxCore_t *core,
    uint8_t line_high);
uint8_t EscSoftUartRxCore_IsReceiving(const EscSoftUartRxCore_t *core);
uint8_t EscSoftUartRxCore_GetSampleIndex(const EscSoftUartRxCore_t *core);
uint32_t EscSoftUartRx_GetSampleOffsetTicks(uint8_t sample_index);
uint32_t EscSoftUartRx_GetBitTicksRounded(void);

#ifdef __cplusplus
}
#endif

#endif
