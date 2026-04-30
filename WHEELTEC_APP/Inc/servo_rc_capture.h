/**
 * @file servo_rc_capture.h
 * @brief RC input capture for servo basic control (TIM4 CH1~CH3, guard on CH3).
 */

#ifndef SERVO_RC_CAPTURE_H
#define SERVO_RC_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"

extern volatile uint32_t g_rc_capture_active_high;

void ServoRC_Capture_Init(void);
void ServoRC_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint16_t ServoRC_GetThrottlePulse(void);
uint16_t ServoRC_GetSteeringPulse(void);
uint16_t ServoRC_GetGuardPulse(void);
uint8_t ServoRC_IsThrottleActive(uint32_t timeout_ms);
uint8_t ServoRC_IsSteeringActive(uint32_t timeout_ms);
uint8_t ServoRC_IsGuardActive(uint32_t timeout_ms);
uint8_t ServoRC_HasThrottleFault(void);
uint8_t ServoRC_HasSteeringFault(void);
uint8_t ServoRC_HasGuardFault(void);

#ifdef __cplusplus
}
#endif

#endif
