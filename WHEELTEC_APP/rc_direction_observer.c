#include "rc_direction_observer.h"

#include <string.h>

#define RC_DIRECTION_OBSERVER_STATE_NEUTRAL 0U
#define RC_DIRECTION_OBSERVER_STATE_DRIVE 1U
#define RC_DIRECTION_OBSERVER_STATE_BRAKE 2U

static uint8_t rc_direction_is_new_sample(const RcDirectionObserver_t *observer,
                                          uint32_t sample_id)
{
    return (observer->has_last_sample == 0U ||
            observer->last_sample_id != sample_id) ? 1U : 0U;
}

static uint16_t rc_direction_median_pwm(const uint16_t *values, uint8_t count)
{
    uint16_t sorted[RC_DIRECTION_OBSERVER_NEUTRAL_WINDOW];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < count; i++)
    {
        sorted[i] = values[i];
    }

    for (i = 1U; i < count; i++)
    {
        const uint16_t current = sorted[i];
        j = i;
        while (j > 0U && sorted[(uint8_t)(j - 1U)] > current)
        {
            sorted[j] = sorted[(uint8_t)(j - 1U)];
            j--;
        }
        sorted[j] = current;
    }

    return sorted[count / 2U];
}

static void rc_direction_refresh_result(RcDirectionObserver_t *observer,
                                        uint8_t processed_new_sample,
                                        RcDirectionObserverReason_t reason)
{
    observer->result.direction_known = observer->output_direction_known;
    observer->result.direction =
        (observer->output_direction_known != 0U) ?
            observer->direction : RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    observer->result.neutral_learned = observer->neutral_learned;
    observer->result.neutral_pwm_us = observer->neutral_pwm_us;
    observer->result.processed_new_sample = processed_new_sample;
    observer->result.awaiting_non_drive_before_reversal =
        observer->saw_non_drive_since_confirmed == 0U &&
        observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN ? 1U : 0U;
    observer->result.reason = reason;
}

static void rc_direction_clear_pending(RcDirectionObserver_t *observer)
{
    observer->pending_direction = RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    observer->pending_count = 0U;
}

static void rc_direction_clear_neutral_learning_pair(RcDirectionObserver_t *observer)
{
    observer->neutral_stable_count = 0U;
    observer->neutral_stable_pwm_us = 0U;
}

static void rc_direction_invalidate_direction(RcDirectionObserver_t *observer)
{
    observer->direction = RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    observer->output_direction_known = 0U;
    observer->saw_non_drive_since_confirmed = 0U;
    rc_direction_clear_pending(observer);
}

static void rc_direction_add_neutral_candidate(RcDirectionObserver_t *observer,
                                               uint16_t pwm_us)
{
    observer->neutral_candidates[observer->neutral_candidate_next] = pwm_us;
    observer->neutral_candidate_next++;
    if (observer->neutral_candidate_next >= RC_DIRECTION_OBSERVER_NEUTRAL_WINDOW)
    {
        observer->neutral_candidate_next = 0U;
    }

    if (observer->neutral_candidate_count < RC_DIRECTION_OBSERVER_NEUTRAL_WINDOW)
    {
        observer->neutral_candidate_count++;
    }

    observer->neutral_pwm_us =
        rc_direction_median_pwm(observer->neutral_candidates,
                                observer->neutral_candidate_count);
    observer->neutral_learned = 1U;
}

static void rc_direction_observe_neutral(RcDirectionObserver_t *observer,
                                         const RcDirectionObserverInput_t *input)
{
    if (input->rpm_valid == 0U || input->rpm_raw != 0U ||
        input->applied_pwm_us == 0U)
    {
        rc_direction_clear_neutral_learning_pair(observer);
        return;
    }

    if (observer->neutral_stable_count == 0U ||
        observer->neutral_stable_pwm_us != input->applied_pwm_us)
    {
        observer->neutral_stable_pwm_us = input->applied_pwm_us;
        observer->neutral_stable_count = 1U;
        return;
    }

    if (observer->neutral_stable_count < 2U)
    {
        observer->neutral_stable_count++;
    }

    if (observer->neutral_stable_count >= 2U)
    {
        rc_direction_add_neutral_candidate(observer, input->applied_pwm_us);
        observer->neutral_stable_count = 1U;
    }
}

static RcDirectionObserverDirection_t rc_direction_candidate_from_pwm(
    const RcDirectionObserver_t *observer,
    uint16_t applied_pwm_us)
{
    const uint16_t neutral_pwm_us = observer->neutral_pwm_us;

    if (applied_pwm_us > neutral_pwm_us)
    {
        return RC_DIRECTION_OBSERVER_DIRECTION_FORWARD;
    }

    if (applied_pwm_us < neutral_pwm_us)
    {
        return RC_DIRECTION_OBSERVER_DIRECTION_REVERSE;
    }

    return RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
}

static uint8_t rc_direction_has_moving_evidence(
    const RcDirectionObserverInput_t *input)
{
    return (input->moving_evidence != 0U) ? 1U : 0U;
}

static RcDirectionObserverReason_t rc_direction_observe_drive(
    RcDirectionObserver_t *observer,
    const RcDirectionObserverInput_t *input)
{
    RcDirectionObserverDirection_t candidate_direction;

    rc_direction_clear_neutral_learning_pair(observer);

    if (observer->neutral_learned == 0U)
    {
        rc_direction_clear_pending(observer);
        observer->output_direction_known = 0U;
        return RC_DIRECTION_OBSERVER_REASON_NEUTRAL_NOT_LEARNED;
    }

    candidate_direction =
        rc_direction_candidate_from_pwm(observer, input->applied_pwm_us);
    if (candidate_direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN)
    {
        rc_direction_clear_pending(observer);
        /* Neutral PWM adds no direction evidence; retain confirmed motion. */
        observer->output_direction_known =
            (observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN) ? 1U : 0U;
        return RC_DIRECTION_OBSERVER_REASON_PWM_AT_NEUTRAL;
    }

    if (rc_direction_has_moving_evidence(input) == 0U)
    {
        rc_direction_clear_pending(observer);
        if (observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN &&
            candidate_direction != observer->direction)
        {
            observer->saw_non_drive_since_confirmed = 1U;
        }
        observer->output_direction_known =
            (observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN) ? 1U : 0U;
        return RC_DIRECTION_OBSERVER_REASON_RPM_NOT_MOVING;
    }

    if (observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN &&
        candidate_direction != observer->direction)
    {
        if (observer->saw_non_drive_since_confirmed == 0U)
        {
            rc_direction_clear_pending(observer);
            observer->output_direction_known = 1U;
            return RC_DIRECTION_OBSERVER_REASON_AWAITING_NON_DRIVE;
        }

        observer->direction = candidate_direction;
        observer->output_direction_known = 1U;
        observer->saw_non_drive_since_confirmed = 0U;
        rc_direction_clear_pending(observer);
        return RC_DIRECTION_OBSERVER_REASON_OK;
    }

    if (observer->direction == candidate_direction &&
        observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN)
    {
        observer->output_direction_known = 1U;
        observer->saw_non_drive_since_confirmed = 0U;
        rc_direction_clear_pending(observer);
        return RC_DIRECTION_OBSERVER_REASON_OK;
    }

    if (observer->pending_direction != candidate_direction)
    {
        observer->pending_direction = candidate_direction;
        observer->pending_count = 1U;
        observer->output_direction_known =
            (observer->direction == candidate_direction &&
             observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN) ? 1U : 0U;
        return RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING;
    }

    if (observer->pending_count < 2U)
    {
        observer->pending_count++;
    }

    if (observer->pending_count >= 2U)
    {
        observer->direction = candidate_direction;
        observer->output_direction_known = 1U;
        observer->saw_non_drive_since_confirmed = 0U;
        return RC_DIRECTION_OBSERVER_REASON_OK;
    }

    return RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING;
}

void RcDirectionObserver_Init(RcDirectionObserver_t *observer)
{
    if (observer == NULL)
    {
        return;
    }

    memset(observer, 0, sizeof(*observer));
    observer->direction = RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    observer->pending_direction = RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    rc_direction_refresh_result(observer,
                                0U,
                                RC_DIRECTION_OBSERVER_REASON_OK);
}

void RcDirectionObserver_ResetDirection(RcDirectionObserver_t *observer)
{
    if (observer == NULL)
    {
        return;
    }

    rc_direction_invalidate_direction(observer);
    rc_direction_clear_neutral_learning_pair(observer);
    observer->has_last_sample = 0U;
    observer->last_sample_id = 0U;
    rc_direction_refresh_result(observer,
                                0U,
                                RC_DIRECTION_OBSERVER_REASON_OK);
}

RcDirectionObserverResult_t RcDirectionObserver_Update(
    RcDirectionObserver_t *observer,
    const RcDirectionObserverInput_t *input)
{
    RcDirectionObserverResult_t invalid_result;
    RcDirectionObserverReason_t reason = RC_DIRECTION_OBSERVER_REASON_OK;
    uint8_t processed_new_sample = 0U;

    memset(&invalid_result, 0, sizeof(invalid_result));
    invalid_result.direction = RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    invalid_result.reason = RC_DIRECTION_OBSERVER_REASON_INVALID_ARGUMENT;

    if (observer == NULL || input == NULL)
    {
        return invalid_result;
    }

    if (input->rc_active == 0U)
    {
        rc_direction_invalidate_direction(observer);
        rc_direction_clear_neutral_learning_pair(observer);
        observer->has_last_sample = 0U;
        reason = RC_DIRECTION_OBSERVER_REASON_RC_INACTIVE;
        rc_direction_refresh_result(observer, 0U, reason);
        return observer->result;
    }

    if (input->telemetry_fresh == 0U)
    {
        rc_direction_invalidate_direction(observer);
        rc_direction_clear_neutral_learning_pair(observer);
        observer->has_last_sample = 0U;
        reason = RC_DIRECTION_OBSERVER_REASON_TELEMETRY_STALE;
        rc_direction_refresh_result(observer, 0U, reason);
        return observer->result;
    }

    if (rc_direction_is_new_sample(observer, input->sample_id) == 0U)
    {
        rc_direction_refresh_result(observer,
                                    0U,
                                    RC_DIRECTION_OBSERVER_REASON_DUPLICATE_SAMPLE);
        return observer->result;
    }

    observer->has_last_sample = 1U;
    observer->last_sample_id = input->sample_id;
    processed_new_sample = 1U;

    if (input->state_raw == RC_DIRECTION_OBSERVER_STATE_NEUTRAL)
    {
        rc_direction_observe_neutral(observer, input);
        rc_direction_clear_pending(observer);
        if (observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN)
        {
            observer->output_direction_known = 1U;
            observer->saw_non_drive_since_confirmed = 1U;
        }
    }
    else if (input->state_raw == RC_DIRECTION_OBSERVER_STATE_DRIVE)
    {
        reason = rc_direction_observe_drive(observer, input);
    }
    else if (input->state_raw == RC_DIRECTION_OBSERVER_STATE_BRAKE)
    {
        rc_direction_clear_neutral_learning_pair(observer);
        rc_direction_clear_pending(observer);
        if (observer->direction != RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN)
        {
            observer->output_direction_known = 1U;
            observer->saw_non_drive_since_confirmed = 1U;
        }
    }
    else
    {
        rc_direction_clear_neutral_learning_pair(observer);
        rc_direction_invalidate_direction(observer);
        reason = RC_DIRECTION_OBSERVER_REASON_ACTION_UNKNOWN;
    }

    rc_direction_refresh_result(observer, processed_new_sample, reason);
    return observer->result;
}

RcDirectionObserverResult_t RcDirectionObserver_GetResult(
    const RcDirectionObserver_t *observer)
{
    RcDirectionObserverResult_t result;

    memset(&result, 0, sizeof(result));
    result.direction = RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN;
    result.reason = RC_DIRECTION_OBSERVER_REASON_INVALID_ARGUMENT;

    if (observer == NULL)
    {
        return result;
    }

    return observer->result;
}
