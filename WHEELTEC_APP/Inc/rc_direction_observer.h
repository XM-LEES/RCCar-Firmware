#ifndef __RC_DIRECTION_OBSERVER_H
#define __RC_DIRECTION_OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RC_DIRECTION_OBSERVER_NEUTRAL_WINDOW 5U

typedef enum
{
    RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN = 0,
    RC_DIRECTION_OBSERVER_DIRECTION_FORWARD = 1,
    RC_DIRECTION_OBSERVER_DIRECTION_REVERSE = -1
} RcDirectionObserverDirection_t;

typedef enum
{
    RC_DIRECTION_OBSERVER_REASON_OK = 0,
    RC_DIRECTION_OBSERVER_REASON_INVALID_ARGUMENT,
    RC_DIRECTION_OBSERVER_REASON_RC_INACTIVE,
    RC_DIRECTION_OBSERVER_REASON_TELEMETRY_STALE,
    RC_DIRECTION_OBSERVER_REASON_DUPLICATE_SAMPLE,
    RC_DIRECTION_OBSERVER_REASON_ACTION_UNKNOWN,
    RC_DIRECTION_OBSERVER_REASON_NEUTRAL_NOT_LEARNED,
    RC_DIRECTION_OBSERVER_REASON_PWM_AT_NEUTRAL,
    RC_DIRECTION_OBSERVER_REASON_RPM_NOT_MOVING,
    RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING,
    RC_DIRECTION_OBSERVER_REASON_AWAITING_NON_DRIVE
} RcDirectionObserverReason_t;

typedef struct
{
    uint8_t rc_active;
    uint8_t telemetry_fresh;
    uint32_t sample_id;
    uint8_t state_raw;
    uint8_t rpm_valid;
    uint16_t rpm_raw;
    uint8_t moving_evidence;
    uint16_t applied_pwm_us;
} RcDirectionObserverInput_t;

typedef struct
{
    uint8_t direction_known;
    RcDirectionObserverDirection_t direction;
    uint8_t neutral_learned;
    uint16_t neutral_pwm_us;
    uint8_t processed_new_sample;
    uint8_t awaiting_non_drive_before_reversal;
    RcDirectionObserverReason_t reason;
} RcDirectionObserverResult_t;

typedef struct
{
    uint8_t has_last_sample;
    uint32_t last_sample_id;

    uint8_t neutral_learned;
    uint16_t neutral_pwm_us;
    uint16_t neutral_candidates[RC_DIRECTION_OBSERVER_NEUTRAL_WINDOW];
    uint8_t neutral_candidate_count;
    uint8_t neutral_candidate_next;
    uint8_t neutral_stable_count;
    uint16_t neutral_stable_pwm_us;

    RcDirectionObserverDirection_t direction;
    uint8_t output_direction_known;
    uint8_t saw_non_drive_since_confirmed;

    RcDirectionObserverDirection_t pending_direction;
    uint8_t pending_count;

    RcDirectionObserverResult_t result;
} RcDirectionObserver_t;

void RcDirectionObserver_Init(RcDirectionObserver_t *observer);
void RcDirectionObserver_ResetDirection(RcDirectionObserver_t *observer);
RcDirectionObserverResult_t RcDirectionObserver_Update(
    RcDirectionObserver_t *observer,
    const RcDirectionObserverInput_t *input);
RcDirectionObserverResult_t RcDirectionObserver_GetResult(
    const RcDirectionObserver_t *observer);

#ifdef __cplusplus
}
#endif

#endif
