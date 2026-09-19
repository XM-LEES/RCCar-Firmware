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

#define TEST_TWO_PI_F 6.28318530717958647692f

static EscMotionEstimatorConfig_t valid_config(void)
{
    EscMotionEstimatorConfig_t config;

    memset(&config, 0, sizeof(config));
    config.wheel_rpm_per_raw = 0.14115f;
    config.wheel_radius_m = 0.115f;
    config.telemetry_timeout_ms = 100U;
    config.stopped_speed_threshold_mps = 0.05f;
    config.stopped_min_samples = 3U;
    config.stopped_min_coverage_ms = 50U;
    return config;
}

static EscFe32Sample_t sample(uint32_t sample_id,
                              uint32_t tick_ms,
                              uint8_t rpm_valid,
                              uint32_t rpm_raw)
{
    EscFe32Sample_t out;

    memset(&out, 0, sizeof(out));
    out.sample_id = sample_id;
    out.received_tick_ms = tick_ms;
    out.rpm_valid = rpm_valid;
    out.rpm_raw = (uint16_t)rpm_raw;
    out.erpm_candidate = rpm_raw * 10U;
    return out;
}

static EscMotionReason_t observe(EscMotionEstimator_t *estimator,
                                 uint32_t sample_id,
                                 uint32_t tick_ms,
                                 uint8_t rpm_valid,
                                 uint32_t rpm_raw,
                                 uint32_t now_tick_ms)
{
    EscFe32Sample_t input = sample(sample_id, tick_ms, rpm_valid, rpm_raw);

    return EscMotionEstimator_ObserveSample(estimator, &input, now_tick_ms);
}

static float expected_speed_mps(uint32_t rpm_raw,
                                const EscMotionEstimatorConfig_t *config)
{
    const float wheel_axle_rpm =
        (float)rpm_raw * config->wheel_rpm_per_raw;

    return wheel_axle_rpm * TEST_TWO_PI_F * config->wheel_radius_m / 60.0f;
}

static int test_config_validity_is_derived_from_values(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;
    EscMotionReason_t reason = ESC_MOTION_REASON_OK;

    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) != 0U);
    EXPECT_TRUE(reason == ESC_MOTION_REASON_OK);
    EXPECT_TRUE(EscMotionEstimator_StopConfigIsValid(&config, &reason) != 0U);

    config = valid_config();
    config.stopped_min_samples = 1U;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) != 0U);
    EXPECT_TRUE(EscMotionEstimator_StopConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == ESC_MOTION_REASON_CONFIG_INVALID);

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 300U, 10U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.magnitude_config_valid == 1U);
    EXPECT_TRUE(estimate.stop_config_valid == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);
    EXPECT_TRUE(estimate.moving_observed == 0U);
    EXPECT_TRUE(estimate.stop_valid == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    config = valid_config();
    config.wheel_rpm_per_raw = 0.0f;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == ESC_MOTION_REASON_CONFIG_INVALID);
    EXPECT_TRUE(EscMotionEstimator_StopConfigIsValid(&config, &reason) == 0U);

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 300U, 10U) ==
                ESC_MOTION_REASON_CONFIG_INVALID);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.config_valid == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 0U);
    EXPECT_TRUE(estimate.moving_observed == 0U);

    config = valid_config();
    config.wheel_rpm_per_raw = INFINITY;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.wheel_radius_m = FLT_MAX;
    config.wheel_rpm_per_raw = FLT_MAX;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.telemetry_timeout_ms = 0x80000000UL;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.stopped_speed_threshold_mps = 0.0f;
    EXPECT_TRUE(EscMotionEstimator_ConfigIsValid(&config, &reason) != 0U);
    EXPECT_TRUE(EscMotionEstimator_StopConfigIsValid(&config, &reason) == 0U);

    return 0;
}

static int test_raw_rpm_uses_empirical_axle_coefficient(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;
    const float expected = expected_speed_mps(300U, &config);

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 7U, 20U, 1U, 300U, 20U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);
    EXPECT_TRUE(estimate.moving_observed == 1U);
    EXPECT_TRUE(fabsf(expected - 0.5098f) < 0.0010f);
    EXPECT_TRUE(fabsf(estimate.speed_magnitude_mps - expected) < 0.0001f);
    EXPECT_TRUE(estimate.reason == ESC_MOTION_REASON_OK);
    EXPECT_TRUE(estimate.direction_valid == 0U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_UNKNOWN);
    EXPECT_TRUE(estimate.signed_speed_valid == 0U);
    EXPECT_TRUE(estimate.signed_speed_mps == 0.0f);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             20U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.direction_valid == 0U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_UNKNOWN);
    EXPECT_TRUE(estimate.signed_speed_valid == 0U);

    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_REVERSE,
                                             30U);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 30U);
    EXPECT_TRUE(estimate.direction_valid == 0U);
    EXPECT_TRUE(estimate.direction == ESC_MOTION_DIRECTION_UNKNOWN);
    EXPECT_TRUE(estimate.signed_speed_valid == 0U);
    EXPECT_TRUE(fabsf(estimate.speed_magnitude_mps - expected) < 0.0001f);

    return 0;
}

static int test_invalid_and_stale_samples_do_not_report_magnitude(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 7U, 20U, 0U, 1234U, 20U) ==
                ESC_MOTION_REASON_RPM_INVALID);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.rpm_valid == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 0U);
    EXPECT_TRUE(estimate.stop_valid == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    EXPECT_TRUE(observe(&estimator, 8U, 30U, 1U, 300U, 30U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 30U);
    EXPECT_TRUE(estimate.sample_fresh == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);

    estimate = EscMotionEstimator_GetEstimate(&estimator, 131U);
    EXPECT_TRUE(estimate.sample_fresh == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 0U);
    EXPECT_TRUE(estimate.reason == ESC_MOTION_REASON_SAMPLE_STALE);

    EXPECT_TRUE(observe(&estimator, 9U, 30U, 1U, 300U, 131U) ==
                ESC_MOTION_REASON_SAMPLE_STALE);

    return 0;
}

static int test_tick_wrap_and_sample_order_boundaries(void)
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

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 100U, 1U, 0U, 110U) ==
                ESC_MOTION_REASON_OK);
    EXPECT_TRUE(observe(&estimator, 2U, 90U, 1U, 0U, 110U) ==
                ESC_MOTION_REASON_SAMPLE_OUT_OF_ORDER);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 110U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);

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
    EXPECT_TRUE(estimate.stop_valid == 1U);
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

static int test_stop_evidence_resets_after_gap_or_motion(void)
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

    EXPECT_TRUE(observe(&estimator, 5U, 225U, 1U, 300U, 225U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 225U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    return 0;
}

static int test_action_boundary_invalidates_old_stop_samples(void)
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
                                             ESC_MOTION_APPLIED_ACTION_BRAKE,
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
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             120U);
    EXPECT_TRUE(observe(&estimator, 3U, 140U, 1U, 0U, 140U) ==
                ESC_MOTION_REASON_OK);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             140U);
    EXPECT_TRUE(observe(&estimator, 4U, 220U, 1U, 0U, 220U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 220U);
    EXPECT_TRUE(estimate.stopped == 0U);
    EXPECT_TRUE(estimate.stop_sample_count == 1U);

    return 0;
}

static int test_set_config_change_invalidates_old_measurement(void)
{
    EscMotionEstimatorConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    EscMotionEstimate_t estimate;

    EscMotionEstimator_Init(&estimator, &config);
    EXPECT_TRUE(observe(&estimator, 1U, 10U, 1U, 300U, 10U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);

    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);

    config.stopped_speed_threshold_mps = 0.025f;
    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(estimate.magnitude_valid == 1U);
    EXPECT_TRUE(estimate.stop_sample_count == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    config.wheel_radius_m = 0.230f;
    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 10U);
    EXPECT_TRUE(estimate.has_sample == 0U);
    EXPECT_TRUE(estimate.magnitude_valid == 0U);
    EXPECT_TRUE(estimate.stopped == 0U);

    EXPECT_TRUE(observe(&estimator, 2U, 20U, 1U, 300U, 20U) ==
                ESC_MOTION_REASON_OK);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.has_sample == 1U);
    EXPECT_TRUE(fabsf(estimate.speed_magnitude_mps -
                      expected_speed_mps(300U, &config)) < 0.0001f);

    config.wheel_rpm_per_raw = 0.0f;
    EXPECT_TRUE(EscMotionEstimator_SetConfig(&estimator, &config) ==
                ESC_MOTION_REASON_CONFIG_INVALID);
    estimate = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(estimate.config_valid == 0U);
    EXPECT_TRUE(estimate.has_sample == 0U);

    return 0;
}

int main(void)
{
    if (test_config_validity_is_derived_from_values() != 0)
    {
        return 1;
    }
    if (test_raw_rpm_uses_empirical_axle_coefficient() != 0)
    {
        return 1;
    }
    if (test_invalid_and_stale_samples_do_not_report_magnitude() != 0)
    {
        return 1;
    }
    if (test_tick_wrap_and_sample_order_boundaries() != 0)
    {
        return 1;
    }
    if (test_stop_evidence_uses_distinct_samples_and_time_coverage() != 0)
    {
        return 1;
    }
    if (test_stop_evidence_resets_after_gap_or_motion() != 0)
    {
        return 1;
    }
    if (test_action_boundary_invalidates_old_stop_samples() != 0)
    {
        return 1;
    }
    if (test_repeated_applied_action_does_not_refresh_stop_epoch() != 0)
    {
        return 1;
    }
    if (test_set_config_change_invalidates_old_measurement() != 0)
    {
        return 1;
    }
    return 0;
}
