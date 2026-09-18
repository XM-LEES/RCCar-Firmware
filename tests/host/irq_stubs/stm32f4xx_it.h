#ifndef TEST_IRQ_STUBS_STM32F4XX_IT_H
#define TEST_IRQ_STUBS_STM32F4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream4_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM4_IRQHandler(void);
void USART1_IRQHandler(void);
void TIM5_IRQHandler(void);
void UART4_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
