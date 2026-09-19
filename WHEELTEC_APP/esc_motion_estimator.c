#include "esc_motion_estimator.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define ESC_MOTION_TICK_HALF_RANGE 0x80000000UL
#define ESC_MOTION_TWO_PI 6.28318530717958647692

static uint8_t esc_motion_float_is_positive_finite(float value)
{
    return (isfinite(value) && value > 0.0f) ? 1U : 0U;
}

static uint8_t esc_motion_duration_is_valid(uint32_t value)
{
    return (value != 0U && value < ESC_MOTION_TICK_HALF_RANGE) ? 1U : 0U;
}

static uint8_t esc_motion_tick_is_after(uint32_t tick_ms,
                                        uint32_t reference_tick_ms)
{
    const uint32_t delta_ms = tick_ms - reference_tick_ms;

    return (delta_ms != 0U && delta_ms < ESC_MOTION_TICK_HALF_RANGE) ? 1U : 0U;
}

static uint8_t esc_motion_tick_delta_is_forward(uint32_t later_tick_ms,
                                                uint32_t earlier_tick_ms,
                                                uint32_t *delta_ms)
{
    const uint32_t local_delta_ms = later_tick_ms - earlier_tick_ms;

    if (local_delta_ms >= ESC_MOTION_TICK_HALF_RANGE)
    {
        return 0U;
    }

    if (delta_ms != NULL)
    {
        *delta_ms = local_delta_ms;
    }
    return 1U;
}

static void esc_motion_clear_stop(EscMotionEstimator_t *estimator)
{
    estimator->stop_evidence_valid = 0U;
    estimator->stop_sample_count = 0U;
    estimator->stop_first_tick_ms = 0U;
    estimator->stop_last_tick_ms = 0U;
    estimator->stop_established_tick_ms = 0U;
}

static void esc_motion_clear_sample_and_stop(EscMotionEstimator_t *estimator)
{
    estimator->has_sample = 0U;
    estimator->last_sample_id = 0U;
    estimator->last_sample_tick_ms = 0U;
    estimator->last_rpm_valid = 0U;
    estimator->last_speed_magnitude_mps = 0.0f;
    esc_motion_clear_stop(estimator);
}

static uint8_t esc_motion_magnitude_config_equals(const EscMotionEstimatorConfig_t *left,
                                                  const EscMotionEstimatorConfig_t *right)
{
    return (left->calibration_valid == right->calibration_valid &&
            left->magnitude_config_valid == right->magnitude_config_valid &&
            left->wheel_rpm_per_raw_valid == right->wheel_rpm_per_raw_valid &&
            left->wheel_radius_valid == right->wheel_radius_valid &&
            left->telemetry_timeout_valid == right->telemetry_timeout_valid &&
            left->wheel_rpm_per_raw == right->wheel_rpm_per_raw &&
            left->wheel_radius_m == right->wheel_radius_m &&
            left->telemetry_timeout_ms == right->telemetry_timeout_ms) ? 1U : 0U;
}

static uint8_t esc_motion_stop_config_equals(const EscMotionEstimatorConfig_t *left,
                                             const EscMotionEstimatorConfig_t *right)
{
    return (left->stop_config_valid == right->stop_config_valid &&
            left->stopped_threshold_valid == right->stopped_threshold_valid &&
            left->stopped_samples_valid == right->stopped_samples_valid &&
            left->stopped_coverage_valid == right->stopped_coverage_valid &&
            left->stopped_speed_threshold_mps == right->stopped_speed_threshold_mps &&
            left->stopped_min_samples == right->stopped_min_samples &&
            left->stopped_min_coverage_ms == right->stopped_min_coverage_ms) ? 1U : 0U;
}

uint8_t EscMotionEstimator_ConfigIsValid(const EscMotionEstimatorConfig_t *config,
                                         EscMotionReason_t *reason)
{
    EscMotionReason_t local_reason = ESC_MOTION_REASON_OK;
    uint8_t valid = 1U;

    if (config == NULL)
    {
        if (reason != NULL)
        {
            *reason = ESC_MOTION_REASON_INVALID_ARGUMENT;
        }
        return 0U;
    }

    if (config->calibration_valid == 0U ||
        config->magnitude_config_valid == 0U ||
        config->wheel_rpm_per_raw_valid == 0U ||
        config->wheel_radius_valid == 0U ||
        config->telemetry_timeout_valid == 0U ||
        esc_motion_float_is_positive_finite(config->wheel_rpm_per_raw) == 0U ||
        esc_motion_float_is_positive_finite(config->wheel_radius_m) == 0U ||
        esc_motion_duration_is_valid(config->telemetry_timeout_ms) == 0U)
    {
        valid = 0U;
    }
    else
    {
        const double meters_per_raw =
            ((double)config->wheel_rpm_per_raw *
             ESC_MOTION_TWO_PI *
             (double)config->wheel_radius_m) / 60.0;

        if (isfinite(meters_per_raw) == 0 ||
            meters_per_raw <= 0.0 ||
            meters_per_raw > (double)FLT_MAX)
        {
            valid = 0U;
        }
    }

    if (valid == 0U)
    {
        local_reason = ESC_MOTION_REASON_CONFIG_INVALID;
    }

    if (reason != NULL)
    {
        *reason = local_reason;
    }
    return valid;
}

uint8_t EscMotionEstimator_StopConfigIsValid(const EscMotionEstimatorConfig_t *config,
                                             EscMotionReason_t *reason)
{
    EscMotionReason_t local_reason = ESC_MOTION_REASON_OK;
    uint8_t valid = 1U;

    if (config == NULL)
    {
        if (reason != NULL)
        {
            *reason = ESC_MOTION_REASON_INVALID_ARGUMENT;
        }
        return 0U;
    }

    if (EscMotionEstimator_ConfigIsValid(config, &local_reason) == 0U ||
        config->stop_config_valid == 0U ||
        config->stopped_threshold_valid == 0U ||
        config->stopped_samples_valid == 0U ||
        config->stopped_coverage_valid == 0U ||
        esc_motion_float_is_positive_finite(config->stopped_speed_threshold_mps) == 0U ||
        config->stopped_min_samples < 2U ||
        esc_motion_duration_is_valid(config->stopped_min_coverage_ms) == 0U)
    {
        valid = 0U;
    }

    if (valid == 0U && local_reason == ESC_MOTION_REASON_OK)
    {
        local_reason = ESC_MOTION_REASON_CONFIG_INVALID;
    }

    if (reason != NULL)
    {
        *reason = local_reason;
    }
    return valid;
}

void EscMotionEstimator_Init(EscMotionEstimator_t *estimator,
                             const EscMotionEstimatorConfig_t *config)
{
    if (estimator == NULL)
    {
        return;
    }

    memset(estimator, 0, sizeof(*estimator));
    estimator->direction = ESC_MOTION_DIRECTION_UNKNOWN;
    if (config != NULL)
    {
        (void)EscMotionEstimator_SetConfig(estimator, config);
    }
    else
    {
        estimator->config_reason = ESC_MOTION_REASON_CONFIG_INVALID;
    }
}

EscMotionReason_t EscMotionEstimator_SetConfig(EscMotionEstimator_t *estimator,
                                               const EscMotionEstimatorConfig_t *config)
{
    EscMotionReason_t reason = ESC_MOTION_REASON_OK;
    uint8_t new_config_valid;
    uint8_t new_stop_config_valid;
    uint8_t same_magnitude_config;
    uint8_t same_stop_config;

    if (estimator == NULL || config == NULL)
    {
        return ESC_MOTION_REASON_INVALID_ARGUMENT;
    }

    new_config_valid = EscMotionEstimator_ConfigIsValid(config, &reason);
    new_stop_config_valid =
        EscMotionEstimator_StopConfigIsValid(config, &estimator->stop_config_reason);
    same_magnitude_config = (estimator->config_valid != 0U &&
                             new_config_valid != 0U &&
                             esc_motion_magnitude_config_equals(&estimator->config,
                                                                config) != 0U) ? 1U : 0U;
    same_stop_config = (estimator->stop_config_valid == new_stop_config_valid &&
                        esc_motion_stop_config_equals(&estimator->config,
                                                      config) != 0U) ? 1U : 0U;

    estimator->config = *config;
    estimator->config_valid = new_config_valid;
    estimator->stop_config_valid = new_stop_config_valid;
    estimator->config_reason = reason;
    if (same_magnitude_config == 0U)
    {
        esc_motion_clear_sample_and_stop(estimator);
    }
    else if (same_stop_config == 0U)
    {
        esc_motion_clear_stop(estimator);
    }
    return reason;
}

static uint8_t esc_motion_raw_rpm_to_mps(const EscMotionEstimatorConfig_t *config,
                                         uint32_t rpm_raw,
                                         float *speed_magnitude_mps)
{
    const double wheel_axle_rpm =
        (double)rpm_raw * (double)config->wheel_rpm_per_raw;
    const double speed_mps =
        (wheel_axle_rpm * ESC_MOTION_TWO_PI *
         (double)config->wheel_radius_m) / 60.0;
    float local_speed_mps;

    if (speed_magnitude_mps == NULL ||
        isfinite(wheel_axle_rpm) == 0 ||
        isfinite(speed_mps) == 0 ||
        wheel_axle_rpm < 0.0 ||
        speed_mps < 0.0 ||
        speed_mps > (double)FLT_MAX)
    {
        return 0U;
    }

    local_speed_mps = (float)speed_mps;
    if (isfinite(local_speed_mps) == 0 ||
        (rpm_raw != 0U && local_speed_mps <= 0.0f))
    {
        return 0U;
    }

    *speed_magnitude_mps = local_speed_mps;
    return 1U;
}

static void esc_motion_update_stop_evidence(EscMotionEstimator_t *estimator,
                                            const EscFe32Sample_t *sample,
                                            float speed_magnitude_mps)
{
    if (estimator->stop_epoch_valid != 0U &&
        esc_motion_tick_is_after(sample->received_tick_ms,
                                 estimator->stop_epoch_tick_ms) == 0U)
    {
        esc_motion_clear_stop(estimator);
        return;
    }

    if (speed_magnitude_mps > estimator->config.stopped_speed_threshold_mps)
    {
        esc_motion_clear_stop(estimator);
        return;
    }

    if (estimator->stop_sample_count == 0U)
    {
        estimator->stop_first_tick_ms = sample->received_tick_ms;
        estimator->stop_sample_count = 1U;
    }
    else if (estimator->stop_sample_count < 0xFFFFFFFFUL)
    {
        estimator->stop_sample_count++;
    }

    estimator->stop_last_tick_ms = sample->received_tick_ms;
    if (estimator->stop_sample_count >= estimator->config.stopped_min_samples &&
        (uint32_t)(estimator->stop_last_tick_ms - estimator->stop_first_tick_ms) >=
            estimator->config.stopped_min_coverage_ms &&
        estimator->stop_evidence_valid == 0U)
    {
        estimator->stop_evidence_valid = 1U;
        estimator->stop_established_tick_ms = sample->received_tick_ms;
    }
}

EscMotionReason_t EscMotionEstimator_ObserveSample(EscMotionEstimator_t *estimator,
                                                   const EscFe32Sample_t *sample,
                                                   uint32_t now_tick_ms)
{
    uint32_t age_ms;
    uint32_t sample_gap_ms = 0U;
    uint8_t new_sample;

    if (estimator == NULL || sample == NULL)
    {
        return ESC_MOTION_REASON_INVALID_ARGUMENT;
    }
    if (estimator->config_valid == 0U)
    {
        return estimator->config_reason;
    }

    if (esc_motion_tick_delta_is_forward(now_tick_ms,
                                         sample->received_tick_ms,
                                         &age_ms) == 0U)
    {
        esc_motion_clear_stop(estimator);
        return ESC_MOTION_REASON_SAMPLE_OUT_OF_ORDER;
    }

    if (age_ms > estimator->config.telemetry_timeout_ms)
    {
        esc_motion_clear_stop(estimator);
        return ESC_MOTION_REASON_SAMPLE_STALE;
    }

    new_sample = (estimator->has_sample == 0U ||
                  sample->sample_id != estimator->last_sample_id) ? 1U : 0U;
    if (new_sample == 0U)
    {
        return (estimator->last_rpm_valid != 0U) ?
            ESC_MOTION_REASON_OK : ESC_MOTION_REASON_RPM_INVALID;
    }

    if (estimator->has_sample != 0U)
    {
        if (esc_motion_tick_delta_is_forward(sample->received_tick_ms,
                                             estimator->last_sample_tick_ms,
                                             &sample_gap_ms) == 0U)
        {
            esc_motion_clear_stop(estimator);
            return ESC_MOTION_REASON_SAMPLE_OUT_OF_ORDER;
        }
        if (sample_gap_ms > estimator->config.telemetry_timeout_ms)
        {
            esc_motion_clear_stop(estimator);
        }
    }

    estimator->has_sample = 1U;
    estimator->last_sample_id = sample->sample_id;
    estimator->last_sample_tick_ms = sample->received_tick_ms;
    estimator->last_rpm_valid = (sample->rpm_valid != 0U) ? 1U : 0U;

    if (estimator->last_rpm_valid == 0U)
    {
        estimator->last_speed_magnitude_mps = 0.0f;
        esc_motion_clear_stop(estimator);
        return ESC_MOTION_REASON_RPM_INVALID;
    }

    if (esc_motion_raw_rpm_to_mps(&estimator->config,
                                  sample->rpm_raw,
                                  &estimator->last_speed_magnitude_mps) == 0U)
    {
        estimator->last_rpm_valid = 0U;
        estimator->last_speed_magnitude_mps = 0.0f;
        esc_motion_clear_stop(estimator);
        return ESC_MOTION_REASON_CALCULATION_INVALID;
    }

    if (estimator->stop_config_valid != 0U)
    {
        esc_motion_update_stop_evidence(estimator, sample,
                                        estimator->last_speed_magnitude_mps);
    }
    else
    {
        esc_motion_clear_stop(estimator);
    }
    return ESC_MOTION_REASON_OK;
}

EscMotionEstimate_t EscMotionEstimator_GetEstimate(const EscMotionEstimator_t *estimator,
                                                   uint32_t now_tick_ms)
{
    EscMotionEstimate_t estimate;

    memset(&estimate, 0, sizeof(estimate));
    estimate.direction = ESC_MOTION_DIRECTION_UNKNOWN;
    estimate.reason = ESC_MOTION_REASON_INVALID_ARGUMENT;

    if (estimator == NULL)
    {
        return estimate;
    }

    estimate.config_valid = estimator->config_valid;
    estimate.magnitude_config_valid = estimator->config_valid;
    estimate.stop_config_valid = estimator->stop_config_valid;
    if (estimator->config_valid == 0U)
    {
        estimate.reason = estimator->config_reason;
        return estimate;
    }

    estimate.has_sample = estimator->has_sample;
    if (estimator->has_sample == 0U)
    {
        estimate.reason = ESC_MOTION_REASON_NO_SAMPLE;
        return estimate;
    }

    estimate.last_sample_id = estimator->last_sample_id;
    estimate.last_sample_tick_ms = estimator->last_sample_tick_ms;
    if (esc_motion_tick_delta_is_forward(now_tick_ms,
                                         estimator->last_sample_tick_ms,
                                         &estimate.sample_age_ms) == 0U)
    {
        estimate.reason = ESC_MOTION_REASON_SAMPLE_OUT_OF_ORDER;
        return estimate;
    }
    if (estimate.sample_age_ms > estimator->config.telemetry_timeout_ms)
    {
        estimate.reason = ESC_MOTION_REASON_SAMPLE_STALE;
        return estimate;
    }

    estimate.sample_fresh = 1U;
    estimate.rpm_valid = estimator->last_rpm_valid;
    if (estimator->last_rpm_valid == 0U)
    {
        estimate.reason = ESC_MOTION_REASON_RPM_INVALID;
        return estimate;
    }

    estimate.magnitude_valid = 1U;
    estimate.speed_magnitude_mps = estimator->last_speed_magnitude_mps;
    if (estimator->stop_config_valid != 0U)
    {
        estimate.stop_valid = 1U;
        estimate.moving_observed =
            (estimate.speed_magnitude_mps >
             estimator->config.stopped_speed_threshold_mps) ? 1U : 0U;
        estimate.stop_sample_count = estimator->stop_sample_count;
        estimate.stop_coverage_ms = estimator->stop_last_tick_ms -
            estimator->stop_first_tick_ms;
        estimate.stop_evidence_start_tick_ms = estimator->stop_first_tick_ms;
        estimate.stop_established_tick_ms = estimator->stop_established_tick_ms;
        estimate.stopped = estimator->stop_evidence_valid;
    }

    estimate.reason = ESC_MOTION_REASON_OK;
    return estimate;
}

void EscMotionEstimator_InvalidateStopEvidence(EscMotionEstimator_t *estimator)
{
    if (estimator == NULL)
    {
        return;
    }
    esc_motion_clear_stop(estimator);
}

void EscMotionEstimator_InvalidateDirectionAndStop(EscMotionEstimator_t *estimator)
{
    if (estimator == NULL)
    {
        return;
    }
    esc_motion_clear_stop(estimator);
}

void EscMotionEstimator_CommitAppliedActionAt(EscMotionEstimator_t *estimator,
                                              EscMotionAppliedAction_t action,
                                              uint32_t applied_tick_ms)
{
    uint8_t action_edge;
    EscMotionAppliedAction_t previous_action;

    if (estimator == NULL)
    {
        return;
    }

    previous_action = estimator->last_applied_action;
    action_edge = (estimator->has_applied_action == 0U ||
                   previous_action != action) ? 1U : 0U;
    estimator->has_applied_action = 1U;
    estimator->last_applied_action = action;

    /*
     * Call this after every real output application. Repeated BRAKE and
     * REVERSE_FIRST_STRIKE ticks share the first applied tick as their
     * stop-evidence epoch so fresh zero-speed telemetry can accumulate while
     * braking. Propulsion or external-override ticks keep stop evidence
     * invalidated because the physical state is still being driven or unknown.
     */
    switch (action)
    {
    case ESC_MOTION_APPLIED_ACTION_FORWARD:
        esc_motion_clear_stop(estimator);
        estimator->stop_epoch_valid = 1U;
        estimator->stop_epoch_tick_ms = applied_tick_ms;
        break;
    case ESC_MOTION_APPLIED_ACTION_REVERSE:
        esc_motion_clear_stop(estimator);
        estimator->stop_epoch_valid = 1U;
        estimator->stop_epoch_tick_ms = applied_tick_ms;
        break;
    case ESC_MOTION_APPLIED_ACTION_BRAKE:
        if (action_edge != 0U)
        {
            esc_motion_clear_stop(estimator);
            estimator->stop_epoch_valid = 1U;
            estimator->stop_epoch_tick_ms = applied_tick_ms;
        }
        break;
    case ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE:
        if (action_edge != 0U)
        {
            esc_motion_clear_stop(estimator);
            estimator->stop_epoch_valid = 1U;
            estimator->stop_epoch_tick_ms = applied_tick_ms;
        }
        break;
    case ESC_MOTION_APPLIED_ACTION_EXTERNAL_OVERRIDE:
        esc_motion_clear_stop(estimator);
        estimator->stop_epoch_valid = 1U;
        estimator->stop_epoch_tick_ms = applied_tick_ms;
        break;
    case ESC_MOTION_APPLIED_ACTION_NEUTRAL:
        if (action_edge != 0U &&
            previous_action == ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE)
        {
            esc_motion_clear_stop(estimator);
            estimator->stop_epoch_valid = 1U;
            estimator->stop_epoch_tick_ms = applied_tick_ms;
        }
        break;
    default:
        break;
    }
}
