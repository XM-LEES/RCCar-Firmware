#ifndef TEST_IRQ_STUBS_MAIN_H
#define TEST_IRQ_STUBS_MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
} USART_TypeDef;

typedef struct
{
    USART_TypeDef *Instance;
} UART_HandleTypeDef;

typedef struct
{
    uint32_t reserved;
} DMA_HandleTypeDef;

typedef struct
{
    uint32_t reserved;
} TIM_HandleTypeDef;

#define RESET 0U
#define SET 1U

#define USART_SR_PE   (1UL << 0)
#define USART_SR_FE   (1UL << 1)
#define USART_SR_NE   (1UL << 2)
#define USART_SR_ORE  (1UL << 3)
#define USART_SR_IDLE (1UL << 4)

#define USART_CR1_IDLEIE (1UL << 4)
#define USART_CR1_RXNEIE (1UL << 5)
#define USART_CR1_PEIE   (1UL << 8)
#define USART_CR3_EIE    (1UL << 0)

#define UART_FLAG_IDLE USART_SR_IDLE
#define UART_IT_IDLE USART_CR1_IDLEIE

#define HallA_Pin (1UL << 13)
#define HallB_Pin (1UL << 14)
#define EscSoftUartRx_Pin (1UL << 15)

uint32_t IrqStub_ReadReg(volatile uint32_t *reg);
uint32_t IrqStub_GetUartFlag(UART_HandleTypeDef *huart, uint32_t flag);
uint32_t IrqStub_GetUartItSource(UART_HandleTypeDef *huart, uint32_t source);
uint32_t IrqStub_GetExtiIt(uint32_t pin);
void IrqStub_ClearIdleFlag(UART_HandleTypeDef *huart);

#define READ_REG(REG) IrqStub_ReadReg(&(REG))
#define __HAL_UART_GET_FLAG(HUART, FLAG) IrqStub_GetUartFlag((HUART), (FLAG))
#define __HAL_UART_GET_IT_SOURCE(HUART, SOURCE) \
    IrqStub_GetUartItSource((HUART), (SOURCE))
#define __HAL_UART_CLEAR_IDLEFLAG(HUART) IrqStub_ClearIdleFlag((HUART))

#define __HAL_GPIO_EXTI_GET_IT(PIN) IrqStub_GetExtiIt((PIN))

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif
