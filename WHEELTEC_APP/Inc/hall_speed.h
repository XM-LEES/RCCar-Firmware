/**
 * @file hall_speed.h
 * @brief Hall wheel-speed measurement with configurable count and direction channels.
 */

#ifndef HALL_SPEED_H
#define HALL_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
	int32_t event_count_total;
	uint32_t last_event_us;
	uint32_t last_period_us;
	uint32_t fault_count;
	int8_t direction;
	uint8_t speed_valid;
	uint8_t timeout_active;
	uint8_t reserved0;
	uint8_t reserved1;
} hall_speed_state_t;

extern volatile hall_speed_state_t g_hall_speed_state;

void HallSpeed_Init(void);
void HallSpeed_SetCommandDirection(int8_t direction);
void HallSpeed_OnCountEvent(void);
uint8_t HallSpeed_GetSignedSpeedMps(float *speed_mps);
hall_speed_state_t HallSpeed_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
