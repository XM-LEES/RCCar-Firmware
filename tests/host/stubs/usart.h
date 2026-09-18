#ifndef RCCAR_HOST_STUB_USART_H
#define RCCAR_HOST_STUB_USART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t instance_id;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *data,
                                        uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
