#ifndef RCCAR_HOST_STUB_HALL_SPEED_H
#define RCCAR_HOST_STUB_HALL_SPEED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t event_count_total;
    uint32_t last_event_cycles;
    uint32_t last_raw_event_cycles;
    uint32_t last_period_us;
    uint32_t zero_command_since_cycles;
    uint32_t fault_count;
    int8_t direction;
    int8_t command_direction;
    uint8_t speed_valid;
    uint8_t timeout_active;
    uint8_t stationary_confirmed;
    uint8_t period_origin_valid;
    uint8_t raw_event_origin_valid;
    uint8_t consecutive_short_event_count;
} hall_speed_state_t;

extern volatile hall_speed_state_t g_hall_speed_state;

void HallSpeed_Init(void);
void HallSpeed_SetCommandDirection(int8_t direction);
void HallSpeed_OnCountEvent(void);
void HallSpeed_ClearFaultCount(void);
uint8_t HallSpeed_GetSignedSpeedMps(float *speed_mps);
uint8_t HallSpeed_GetSnapshotSpeedMps(const hall_speed_state_t *snapshot,
                                      float *speed_mps);
hall_speed_state_t HallSpeed_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
