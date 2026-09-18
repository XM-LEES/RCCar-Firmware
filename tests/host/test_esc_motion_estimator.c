#include "esc_motion_estimator.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

static EscMotionEstimatorConfig_t valid_config(void)
{
    EscMotionEstimatorConfig_t config;

    memset(&config, 0, sizeof(config));
    config.calibration_valid = 1U;
    config.pole_pairs_valid = 1U;
    config.gear_ratio_valid = 1U;
    config.wheel_ratio_valid = 1U;
    config.wheel_circumference_valid = 1U;
    config.telemetry_timeout_valid = 1U;
    config.stopped_threshold_valid = 1U;
    config.stopped_samples_valid = 1U;
    config.stopped_coverage_valid = 1U;
    config.motor_pole_pairs = 4U;
    config.gear_ratio = 2.0f;
    config.wheel_ratio = 3.0f;
    config.wheel_circumference_m = 0.6f;
    config.telemetry_timeout_ms = 100U;
    config.stopped_speed_threshold_mps = 0.05f;
    config.stopped_min_samples = 3U;
    config.stopped_min_coverage_ms = 50U;
    return config;
}

static EscFe32Sample_t sample(uint32_t sample_id,
                              uint32_t tick_ms,
                              uint8_t rpm_valid,
                              uint32_t erpm)
{
    EscFe32Sample_t out;

    memset(&out, 0, sizeof(out));
    out.sample_id = sample_id;
    out.received_tick_ms = tick_ms;
    out.rpm_valid = rpm_valid;
    out.erpm_candidate = erpm;
    return out;
}

static EscMotionReason_t observe(EscMotionEstimator_t *estimator,
                                 uint32_t sample_id,
                                 uint32_t tick_ms,
                                 uint8_t rpm_valid,
                                 uint32_t erpm,
                                 uint32_t now_tick_ms)
{
    EscFe32Sample_t input = sample(sample_id, tick_ms, rpm_valid, erpm);

    return EscMotionEstimator_ObserveSample(estimator, &input, now_tick_ms);
}

static int test_config_rejects_missing_and_nonfinite_values(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionReason_t reason = ESC_MOTION_REASON_OK;

    config.motor_pole_pairs = 0U;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == ESC_MOTION_REASON_CONFIG_INVALID);

    config = valid_config();
    config.gear_ratio = INFINITY;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.stopped_min_samples = 1U;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.telemetry_timeout_ms = 0x80000000UL;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.gear_ratio = FLT_MAX;
    config.wheel_ratio = FLT_MAX;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.calibration_valid = 0U;
    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 1000U, 10U) ==
                ESC_MOTION_REASON_CONFIG_INVALID);
    EXPECT_TRUE(EscMotionEstimator_GetEstimate(&estimator, 10U).config_valid == 0U);

    return 0;
}

static int test_rpm_invalid_and_direction_are_independent(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 7U, 20U, 0U, 1234U, 20U) ==
                ESC_MOTION_REASON_RPM_INVALID);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    EXPECT_TRUE(observe(&estimator, 8U, 30U, 1U, 2400U, 30U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 30U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);
    EXPECT_TRUE(fabsf(estimate.speed_magnitude_mps - 1.0f) < 0.0001f);
    EXPECT_TRUE(estimate.direction_valid == 0U);
    EXPECT_TRUE(estimate.signed_speed_valid == 0U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             30U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 30U);
    EXPECT_TRUE(estimate.direction_valid == 1U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_FORWARD);
    EXPECT_TRUE(fabsf(estimate.signed_speed_mps - 1.0f) < 0.0001f);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_BRAKE,
                                             30U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 30U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_FORWARD);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE,
                                             30U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 30U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_REVERSE);
    EXPECT_TRUE(fabsf(estimate.signed_speed_mps + 1.0f) < 0.0001f);

    return 0;
}

static int test_timeout_boundary_and_tick_wrap(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    config.telemetry_timeout_ms = 32U;
    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 0U, 0xFFFFFFF0UL, 1U, 0U,
                        0x00000010UL) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 0x00000010UL);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.sample_fresh == 1U);
    EXPECT_TRUE(estimate.last_sample_id == 0U);
    EXPECT_TRUE(estimate.sample_age_ms == 32U);

    estimate = EscMotionEstimator_GetEstimate(&estimator, 0x00000011UL);
    EXPECT_TRUE(estimate.sample_fresh == 0U);
    EXPECT_TRUE(estimate.reason == ESC_MOTION_REASON_SAMPLE_STALE);

    EXPECT_TRUE(observe(&estimator, 1U, 0xFFFFFFF0UL, 1U, 0U,
                        0x00000011UL) ==
                ESC_MOTION_REASON_SAMPLE_STALE);

    return 0;
}

static int test_stop_evidence_uses_distinct_samples_and_time_coverage(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 0U, 100U, 1U, 0U, 100U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 0U, 150U, 1U, 0U, 150U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 150U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.stop_sample_count == 1U);
    EXPECT_TRUE(estimate.stopped == 0U);

    EXPECT_TRUE(observe(&estimator, 1U, 100U, 1U, 0U, 100U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 100U, 1U, 0U, 100U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 100U);
    EXPECT_TRUE(estimate.stop_sample_count == 3U);
    EXPECT_TRUE(estimate.stop_coverage_ms == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    EXPECT_TRUE(observe(&estimator, 3U, 150U, 1U, 0U, 150U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 150U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.stop_coverage_ms == 50U);
    EXPECT_TRUE(estimate.stop_established_tick_ms == 150U);

    EXPECT_TRUE(observe(&estimator, 4U, 200U, 1U, 0U, 200U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 200U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.stop_established_tick_ms == 150U);

    return 0;
}

static int test_stop_evidence_resets_after_gap_and_rejects_backward_ticks(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 0U, 1U, 0U, 0U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 25U, 1U, 0U, 25U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 3U, 50U, 1U, 0U, 50U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 50U);
    EXPECT_TRUE(estimate.stopped == 1U);

    EXPECT_TRUE(observe(&estimator, 4U, 200U, 1U, 0U, 200U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 200U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 1U);

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 100U, 1U, 0U, 110U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 90U, 1U, 0U, 110U) ==
                ESC_MOTION_REASON_SAMPLE_OUT_OF_ORDER);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 110U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_coverage_ms == 0U);

    return 0;
}

static int test_applied_action_tick_excludes_old_stop_samples(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 0U, 10U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 35U, 1U, 0U, 35U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 3U, 60U, 1U, 0U, 60U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 60U);
    EXPECT_TRUE(estimate.stopped == 1U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE,
                                             100U);
    EXPECT_TRUE(observe(&estimator, 4U, 90U, 1U, 0U, 100U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 100U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);

    EXPECT_TRUE(observe(&estimator, 5U, 120U, 1U, 0U, 120U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 6U, 145U, 1U, 0U, 145U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 7U, 170U, 1U, 0U, 170U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 170U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.stop_established_tick_ms == 170U);

    return 0;
}

static int test_repeated_applied_action_does_not_refresh_stop_epoch(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    config.stopped_min_samples = 2U;
    config.stopped_min_coverage_ms = 80U;
    EscMotionEstimator_Init(&estimator, &config);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_BRAKE,
                                             0U);
    EXPECT_TRUE(observe(&estimator, 1U, 20U, 1U, 0U, 20U) ==
                ESC_MOTION_REASON_OK);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_BRAKE,
                                             20U);
    EXPECT_TRUE(observe(&estimator, 2U, 100U, 1U, 0U, 100U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 100U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.stop_established_tick_ms == 100U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE,
                                             120U);
    EXPECT_TRUE(observe(&estimator, 3U, 110U, 1U, 0U, 120U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 120U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);

    EXPECT_TRUE(observe(&estimator, 4U, 140U, 1U, 0U, 140U) ==
                ESC_MOTION_REASON_OK);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE,
                                             140U);
    EXPECT_TRUE(observe(&estimator, 5U, 220U, 1U, 0U, 220U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 220U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.stop_established_tick_ms == 220U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             240U);
    EXPECT_TRUE(observe(&estimator, 6U, 260U, 1U, 0U, 260U) ==
                ESC_MOTION_REASON_OK);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             260U);
    EXPECT_TRUE(observe(&estimator, 7U, 340U, 1U, 0U, 340U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 340U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 1U);

    return 0;
}

static int test_first_strike_to_neutral_requires_new_stop_evidence(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    config.stopped_min_samples = 2U;
    config.stopped_min_coverage_ms = 20U;
    EscMotionEstimator_Init(&estimator, &config);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE,
                                             100U);
    EXPECT_TRUE(observe(&estimator, 1U, 120U, 1U, 0U, 120U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 140U, 1U, 0U, 140U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 140U);
    EXPECT_TRUE(estimate.stopped == 1U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_NEUTRAL,
                                             160U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 160U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);

    EXPECT_TRUE(observe(&estimator, 3U, 150U, 1U, 0U, 160U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 4U, 160U, 1U, 0U, 160U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 160U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);

    EXPECT_TRUE(observe(&estimator, 5U, 180U, 1U, 0U, 180U) ==
                ESC_MOTION_REASON_OK);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_NEUTRAL,
                                             180U);
    EXPECT_TRUE(observe(&estimator, 6U, 200U, 1U, 0U, 200U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 200U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.stop_established_tick_ms == 200U);

    return 0;
}

static int test_set_config_change_invalidates_old_measurement(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 2400U, 10U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);

    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);

    config.wheel_circumference_m = 1.2f;
    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    EXPECT_TRUE(observe(&estimator, 2U, 20U, 1U, 2400U, 20U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(fabsf(estimate.speed_magnitude_mps - 2.0f) < 0.0001f);

    config.calibration_valid = 0U;
    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_CONFIG_INVALID);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.config_valid == 0U);
    EXPECT_TRUE(estimate.has_sample == 0U);

    return 0;
}

static int test_startup_stop_and_applied_actions_invalidate_old_evidence(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 0U, 10U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 35U, 1U, 0U, 35U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 3U, 60U, 1U, 0U, 60U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 60U);
    EXPECT_TRUE(estimate.stopped == 1U);
    EXPECT_TRUE(estimate.direction_valid == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             60U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 60U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_FORWARD);

    (void)observe(&estimator, 4U, 70U, 1U, 0U, 70U);
    (void)observe(&estimator, 5U, 95U, 1U, 0U, 95U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 95U);
    EXPECT_TRUE(estimate.stopped == 0U);

    (void)observe(&estimator, 6U, 120U, 1U, 0U, 120U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 120U);
    EXPECT_TRUE(estimate.stopped == 1U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE,
                                             120U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 120U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_FORWARD);

    EscMotionEstimator_InvalidateDirectionAndStop(&estimator);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 120U);
    EXPECT_TRUE(estimate.direction_valid == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    return 0;
}

int main(void)
{
    if (test_config_rejects_missing_and_nonfinite_values() != 0)
    {
        return 1;
    }
    if (test_rpm_invalid_and_direction_are_independent() != 0)
    {
        return 1;
    }
    if (test_timeout_boundary_and_tick_wrap() != 0)
    {
        return 1;
    }
    if (test_stop_evidence_uses_distinct_samples_and_time_coverage() != 0)
    {
        return 1;
    }
    if (test_stop_evidence_resets_after_gap_and_rejects_backward_ticks() != 0)
    {
        return 1;
    }
    if (test_applied_action_tick_excludes_old_stop_samples() != 0)
    {
        return 1;
    }
    if (test_repeated_applied_action_does_not_refresh_stop_epoch() != 0)
    {
        return 1;
    }
    if (test_first_strike_to_neutral_requires_new_stop_evidence() != 0)
    {
        return 1;
    }
    if (test_set_config_change_invalidates_old_measurement() != 0)
    {
        return 1;
    }
    if (test_startup_stop_and_applied_actions_invalidate_old_evidence() != 0)
    {
        return 1;
    }
    return 0;
}
