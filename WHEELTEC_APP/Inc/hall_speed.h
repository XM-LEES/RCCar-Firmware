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
	/* Last accepted edge; subtract before converting to handle DWT rollover. */
	uint32_t last_event_cycles;
	/* Last raw edge, including rejected glitches, for burst confirmation. */
	uint32_t last_raw_event_cycles;
	uint32_t last_period_us;
	/* Start of the current continuous zero-direction request. */
	uint32_t zero_command_since_cycles;
	uint32_t fault_count;
	/* Last non-zero control direction retained while the wheel coasts. */
	int8_t direction;
	/* Current automatic/RC control direction; zero means no motion request. */
	int8_t command_direction;
	uint8_t speed_valid;
	uint8_t timeout_active;
	/* Hall no-pulse standstill inference; not an independent stop sensor. */
	uint8_t stationary_confirmed;
	/* A prior accepted edge exists for period measurement. */
	uint8_t period_origin_valid;
	/* A prior raw edge exists for consecutive short-interval detection. */
	uint8_t raw_event_origin_valid;
	uint8_t consecutive_short_event_count;
} hall_speed_state_t;

extern volatile hall_speed_state_t g_hall_speed_state;

void HallSpeed_Init(void);
void HallSpeed_SetCommandDirection(int8_t direction);
void HallSpeed_OnCountEvent(void);
void HallSpeed_ClearFaultCount(void);
uint8_t HallSpeed_GetSignedSpeedMps(float *speed_mps);
hall_speed_state_t HallSpeed_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
