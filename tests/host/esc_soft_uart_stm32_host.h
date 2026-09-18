#ifndef ESC_SOFT_UART_STM32_HOST_H
#define ESC_SOFT_UART_STM32_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} TIM_TypeDef;

typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;

typedef struct
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    volatile uint32_t CMPCR;
} SYSCFG_TypeDef;

typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_TypeDef;

extern GPIO_TypeDef esc_soft_uart_host_gpiod;
extern TIM_TypeDef esc_soft_uart_host_tim5;
extern RCC_TypeDef esc_soft_uart_host_rcc;
extern SYSCFG_TypeDef esc_soft_uart_host_syscfg;
extern EXTI_TypeDef esc_soft_uart_host_exti;
extern volatile uint32_t uwTick;

#define GPIOD (&esc_soft_uart_host_gpiod)
#define TIM5 (&esc_soft_uart_host_tim5)
#define RCC (&esc_soft_uart_host_rcc)
#define SYSCFG (&esc_soft_uart_host_syscfg)
#define EXTI (&esc_soft_uart_host_exti)

#define GPIO_PIN_15 (1UL << 15)
#define RCC_AHB1ENR_GPIODEN (1UL << 3)
#define RCC_APB1ENR_TIM5EN (1UL << 3)
#define RCC_APB2ENR_SYSCFGEN (1UL << 14)
#define TIM_DIER_CC1IE (1UL << 1)
#define TIM_SR_CC1IF (1UL << 1)
#define TIM_EGR_UG (1UL << 0)
#define TIM_CR1_CEN (1UL << 0)

typedef enum
{
    TIM5_IRQn = 50,
    EXTI15_10_IRQn = 40
} IRQn_Type;

void EscSoftUartHost_SetPriority(IRQn_Type irq, uint32_t priority,
                                 uint32_t subpriority);
void EscSoftUartHost_EnableIRQ(IRQn_Type irq);
void EscSoftUartHost_ClearExtiPending(uint32_t line);

#define __HAL_RCC_GPIOD_CLK_ENABLE() \
    do { RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; } while (0)
#define __HAL_RCC_SYSCFG_CLK_ENABLE() \
    do { RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; } while (0)
#define __HAL_RCC_TIM5_CLK_ENABLE() \
    do { RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; } while (0)
#define HAL_NVIC_SetPriority(IRQ, PRIORITY, SUBPRIORITY) \
    EscSoftUartHost_SetPriority((IRQ), (PRIORITY), (SUBPRIORITY))
#define HAL_NVIC_EnableIRQ(IRQ) EscSoftUartHost_EnableIRQ((IRQ))

#ifdef __cplusplus
}
#endif

#endif
