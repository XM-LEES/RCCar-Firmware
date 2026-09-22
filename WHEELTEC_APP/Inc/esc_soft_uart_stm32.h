#ifndef __ESC_SOFT_UART_STM32_H
#define __ESC_SOFT_UART_STM32_H

#include <stdint.h>

#include "esc_soft_uart_rx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESC_SOFT_UART_STM32_RX_PIN_MASK (1UL << 15)
#define ESC_SOFT_UART_STM32_LATE_TICKS 250UL

typedef struct
{
    uint8_t byte;
    uint32_t received_tick_ms;
    uint32_t output_context;
} EscSoftUartStm32Byte_t;

typedef uint32_t (*EscSoftUartStm32OutputContextProvider_t)(void);

typedef struct
{
    uint32_t bytes_received;
    uint32_t bytes_dropped;
    uint32_t start_edges;
    uint32_t start_glitches;
    uint32_t stop_low_faults;
    uint32_t ring_overflows;
    uint32_t timing_late_faults;
    uint32_t fault_events;
} EscSoftUartStm32Diagnostics_t;

void EscSoftUartStm32_Init(void);
uint8_t EscSoftUartStm32_Start(uint32_t tick_ms);
void EscSoftUartStm32_SetOutputContextProvider(
    EscSoftUartStm32OutputContextProvider_t provider);
void EscSoftUartStm32_UpdateTickBase(uint32_t tick_ms);
uint32_t EscSoftUartStm32_TakeFaults(void);
uint8_t EscSoftUartStm32_ReadByte(EscSoftUartStm32Byte_t *item);
void EscSoftUartStm32_GetDiagnostics(
    EscSoftUartStm32Diagnostics_t *diagnostics);
uint8_t EscSoftUartStm32_HandleExti15Irq(void);
void EscSoftUartStm32_HandleTim5Irq(void);

#ifdef __cplusplus
}
#endif

#endif
