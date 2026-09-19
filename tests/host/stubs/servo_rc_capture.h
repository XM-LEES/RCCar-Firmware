#ifndef RCCAR_HOST_STUB_SERVO_RC_CAPTURE_H
#define RCCAR_HOST_STUB_SERVO_RC_CAPTURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t g_rc_capture_active_high;

void ServoRC_Capture_Init(void);
uint16_t ServoRC_GetThrottlePulse(void);
uint16_t ServoRC_GetSteeringPulse(void);
uint8_t ServoRC_IsThrottleActive(uint32_t timeout_ms);
uint8_t ServoRC_IsSteeringActive(uint32_t timeout_ms);
uint8_t ServoRC_HasThrottleFault(void);
uint8_t ServoRC_HasSteeringFault(void);

#ifdef __cplusplus
}
#endif

#endif
