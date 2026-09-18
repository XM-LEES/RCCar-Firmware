#ifndef RCCAR_HOST_STUB_BSP_ADC_H
#define RCCAR_HOST_STUB_BSP_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define userconfigADC_VOL_CHANNEL 0U

uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif
