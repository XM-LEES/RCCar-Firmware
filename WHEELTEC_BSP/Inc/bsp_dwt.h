#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include <stdint.h>

extern uint32_t SystemCoreClock;

//DWT寄存器地址
#define DWT_CTRL        *(uint32_t*)0xE0001000
#define DWT_CYCCNT      *(uint32_t*)0xE0001004
#define DEM_CR          *(uint32_t*)0xE000EDFC

//DWT的32位计数器最大值
#define DWT_CNT_MAX  4294967295u

void DWT_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* __BSP_DWT_H */

