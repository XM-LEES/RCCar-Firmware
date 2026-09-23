#include "longitudinal_controller.h"

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

#define EXPECT_NEAR(actual, expected, tolerance) \
    do { \
        if (fabsf((actual) - (expected)) > (tolerance)) { \
            fprintf(stderr, "%s:%d: expected %.6f near %.6f\n", \
                    __FILE__, __LINE__, (double)(actual), (double)(expected)); \
            return 1; \
        } \
    } while (0)

static LongitudinalControllerConfig_t valid_config(void)
{
    LongitudinalControllerConfig_t config;

    memset(&config, 0, sizeof(config));
    config.kp_us_per_mps = 100.0f;
    config.ki_us_per_mps_s = 0.0f;
    config.kd_us_per_mps2 = 0.0f;
    config.derivative_tau_s = 0.08f;
    config.antiwindup_tau_s = 0.10f;
    config.min_output_us = -500.0f;
    config.max_output_us = 500.0f;
    config.feedback_freshness_ms = 250U;
    return config;
}

static LongitudinalFeedbackSample_t sample_at(uint32_t id,
                                              uint32_t tick_ms,
                                              float speed_mps)
{
    LongitudinalFeedbackSample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.valid = 1U;
    sample.sample_id = id;
    sample.tick_ms = tick_ms;
    sample.signed_speed_mps = speed_mps;
    if (speed_mps > 0.0001f)
    {
        sample.direction = LONGITUDINAL_DIRECTION_FORWARD;
    }
    else if (speed_mps < -0.0001f)
    {
        sample.direction = LONGITUDINAL_DIRECTION_REVERSE;
    }
    else
    {
        sample.direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    }
    return sample;
}

static LongitudinalControllerInput_t input_at(uint32_t now_ms,
                                              float target_mps)
{
    LongitudinalControllerInput_t input;

    memset(&input, 0, sizeof(input));
    input.enabled = 1U;
    input.target_speed_mps = target_mps;
    input.now_tick_ms = now_ms;
    return input;
}

static LongitudinalFeedbackSample_t *sample_ptr(uint32_t id,
                                                uint32_t tick_ms,
                                                float speed_mps)
{
    static LongitudinalFeedbackSample_t sample;

    sample = sample_at(id, tick_ms, speed_mps);
    return &sample;
}

static LongitudinalControllerInput_t *input_ptr(uint32_t now_ms,
                                                float target_mps)
{
    static LongitudinalControllerInput_t input;

    input = input_at(now_ms, target_mps);
    return &input;
}

static LongitudinalAppliedOutput_t applied_output(float applied_us)
{
    LongitudinalAppliedOutput_t applied;

    memset(&applied, 0, sizeof(applied));
    applied.pid_active = 1U;
    applied.applied_output_us = applied_us;
    applied.min_output_us = -500.0f;
    applied.max_output_us = 500.0f;
    return applied;
}

static LongitudinalAppliedOutput_t applied_phase_output(float applied_us,
                                                        float min_output_us,
                                                        float max_output_us,
                                                        uint8_t pid_active,
                                                        uint8_t reset_integral)
{
    LongitudinalAppliedOutput_t applied = applied_output(applied_us);

    applied.min_output_us = min_output_us;
    applied.max_output_us = max_output_us;
    applied.pid_active = pid_active;
    applied.reset_integral = reset_integral;
    return applied;
}

static int test_p_only_outputs_error(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);
    EXPECT_TRUE(LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 100U, 0.0f)) == LONGITUDINAL_REASON_OK);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.25f));
    EXPECT_TRUE(output.valid == 1U);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_FORWARD);
    EXPECT_NEAR(output.raw_output_us, 125.0f, 0.001f);
    EXPECT_NEAR(output.bounded_output_us, 125.0f, 0.001f);
    EXPECT_TRUE(output.diagnostics.new_sample == 1U);
    EXPECT_TRUE(output.diagnostics.derivative_valid == 0U);

    return 0;
}

static int test_integral_updates_on_commit_and_uses_committed_sample_span(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalAppliedOutput_t applied;
    LongitudinalControllerOutput_t output;

    config.kp_us_per_mps = 1.0f;
    config.ki_us_per_mps_s = 20.0f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 100U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    EXPECT_NEAR(output.diagnostics.proportional_us, 1.0f, 0.001f);
    applied = applied_output(output.bounded_output_us);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);
    EXPECT_NEAR(controller.integral_us, 0.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 120U, 0.0f));
    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(3U, 180U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(180U, 1.0f));
    applied = applied_output(output.bounded_output_us);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);
    EXPECT_NEAR(controller.integral_us, 1.6f, 0.001f);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(200U, 1.0f));
    EXPECT_TRUE(output.diagnostics.duplicate_sample == 1U);
    EXPECT_NEAR(output.diagnostics.integral_us, 1.6f, 0.001f);

    return 0;
}

static int test_duplicate_sample_does_not_integrate_twice(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalAppliedOutput_t applied;
    LongitudinalControllerOutput_t output;
    float integral_after_new_sample;

    config.kp_us_per_mps = 1.0f;
    config.ki_us_per_mps_s = 50.0f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 0U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(0U, 1.0f));
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 100U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);
    integral_after_new_sample = controller.integral_us;
    EXPECT_NEAR(integral_after_new_sample, 5.0f, 0.001f);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(120U, 1.0f));
    EXPECT_TRUE(output.diagnostics.duplicate_sample == 1U);
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);
    EXPECT_NEAR(controller.integral_us, integral_after_new_sample, 0.001f);

    return 0;
}

static int test_derivative_uses_measurement_not_target_step(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerOutput_t output;

    config.kp_us_per_mps = 100.0f;
    config.kd_us_per_mps2 = 10.0f;
    config.derivative_tau_s = 0.10f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 0U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(0U, 0.0f));
    EXPECT_NEAR(output.diagnostics.derivative_us, 0.0f, 0.001f);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(20U, 1.0f));
    EXPECT_NEAR(output.diagnostics.proportional_us, 100.0f, 0.001f);
    EXPECT_NEAR(output.diagnostics.derivative_us, 0.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 100U, 0.5f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    EXPECT_TRUE(output.diagnostics.derivative_valid == 0U);
    EXPECT_NEAR(output.diagnostics.derivative_us, 0.0f, 0.001f);
    EXPECT_NEAR(output.raw_output_us, 50.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(3U, 200U, 1.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(200U, 1.0f));
    EXPECT_TRUE(output.diagnostics.derivative_valid == 1U);
    EXPECT_NEAR(output.diagnostics.measured_accel_mps2, 2.5f, 0.001f);
    EXPECT_NEAR(output.diagnostics.derivative_us, -25.0f, 0.001f);
    EXPECT_NEAR(output.raw_output_us, -25.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(4U, 451U, 1.1f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(451U, 1.1f));
    EXPECT_TRUE(output.diagnostics.derivative_valid == 0U);
    EXPECT_NEAR(output.diagnostics.measured_accel_mps2, 0.0f, 0.001f);

    return 0;
}

static int test_target_change_skips_one_error_integral_and_sign_clears(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalAppliedOutput_t applied;
    LongitudinalControllerOutput_t output;

    config.kp_us_per_mps = 1.0f;
    config.ki_us_per_mps_s = 100.0f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 0U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(0U, 1.0f));
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 100U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);
    EXPECT_NEAR(controller.integral_us, 10.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(3U, 200U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(200U, 2.0f));
    EXPECT_TRUE(output.diagnostics.target_changed == 1U);
    EXPECT_TRUE(output.diagnostics.error_integral_skipped == 1U);
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);
    EXPECT_NEAR(controller.integral_us, 10.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(4U, 300U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(300U, -1.0f));
    EXPECT_TRUE(output.diagnostics.target_sign_changed == 1U);
    EXPECT_NEAR(controller.integral_us, 0.0f, 0.001f);

    return 0;
}

static int test_antiwindup_tracks_applied_output_and_clamps_integral(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalAppliedOutput_t applied;
    LongitudinalControllerOutput_t output;

    config.kp_us_per_mps = 400.0f;
    config.ki_us_per_mps_s = 200.0f;
    config.antiwindup_tau_s = 0.10f;
    config.min_output_us = -100.0f;
    config.max_output_us = 100.0f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 0U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(0U, 1.0f));
    applied = applied_output(100.0f);
    applied.min_output_us = -100.0f;
    applied.max_output_us = 100.0f;
    (void)LongitudinalController_CommitApplied(&controller, &applied);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 100U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    EXPECT_TRUE(output.diagnostics.output_saturated == 1U);
    applied = applied_output(100.0f);
    applied.min_output_us = -100.0f;
    applied.max_output_us = 100.0f;
    (void)LongitudinalController_CommitApplied(&controller, &applied);
    EXPECT_NEAR(controller.integral_us, -100.0f, 0.001f);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(120U, 1.0f));
    EXPECT_NEAR(output.raw_output_us, 300.0f, 0.001f);

    return 0;
}

static int test_invalid_feedback_stale_feedback_and_timebase_fail_closed(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalFeedbackSample_t sample;
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);

    sample = sample_at(1U, 100U, 0.5f);
    sample.direction = LONGITUDINAL_DIRECTION_REVERSE;
    EXPECT_TRUE(LongitudinalController_ObserveFeedback(
        &controller, &sample) == LONGITUDINAL_REASON_FEEDBACK_INVALID);
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    EXPECT_TRUE(output.valid == 0U);
    EXPECT_TRUE(output.reason == LONGITUDINAL_REASON_FEEDBACK_INVALID);

    sample = sample_at(2U, 200U, 0.0f);
    EXPECT_TRUE(LongitudinalController_ObserveFeedback(
        &controller, &sample) == LONGITUDINAL_REASON_OK);
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(451U, 1.0f));
    EXPECT_TRUE(output.reason == LONGITUDINAL_REASON_FEEDBACK_STALE);

    LongitudinalController_Reset(&controller);
    sample = sample_at(3U, 150U, 0.0f);
    EXPECT_TRUE(LongitudinalController_ObserveFeedback(
        &controller, &sample) == LONGITUDINAL_REASON_OK);
    sample = sample_at(4U, 100U, 0.0f);
    EXPECT_TRUE(LongitudinalController_ObserveFeedback(
        &controller, &sample) == LONGITUDINAL_REASON_TIMEBASE_INVALID);
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(151U, 1.0f));
    EXPECT_TRUE(output.reason == LONGITUDINAL_REASON_FEEDBACK_INVALID);

    return 0;
}

static int test_future_now_and_nan_are_rejected(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerOutput_t output;
    LongitudinalFeedbackSample_t sample;

    config.kp_us_per_mps = NAN;
    EXPECT_TRUE(LongitudinalController_ConfigIsValid(
        &config, NULL) == 0U);
    config = valid_config();
    config.kp_us_per_mps = 0.0f;
    EXPECT_TRUE(LongitudinalController_ConfigIsValid(
        &config, NULL) == 0U);
    config = valid_config();
    config.derivative_tau_s = 0.0f;
    EXPECT_TRUE(LongitudinalController_ConfigIsValid(
        &config, NULL) == 0U);

    config = valid_config();
    LongitudinalController_Init(&controller, &config);
    sample = sample_at(1U, 1000U, 0.0f);
    EXPECT_TRUE(LongitudinalController_ObserveFeedback(
        &controller, &sample) == LONGITUDINAL_REASON_OK);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(900U, 1.0f));
    EXPECT_TRUE(output.reason == LONGITUDINAL_REASON_TIMEBASE_INVALID);

    output = LongitudinalController_Evaluate(
        &controller, input_ptr(1000U, NAN));
    EXPECT_TRUE(output.reason == LONGITUDINAL_REASON_INVALID_ARGUMENT);

    return 0;
}

static int test_ki_zero_keeps_integral_zero(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalAppliedOutput_t applied;
    LongitudinalControllerOutput_t output;

    config.kp_us_per_mps = 1.0f;
    config.ki_us_per_mps_s = 0.0f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 0U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(0U, 1.0f));
    applied = applied_output(output.bounded_output_us);
    (void)LongitudinalController_CommitApplied(&controller, &applied);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 100U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    applied = applied_output(100.0f);
    (void)LongitudinalController_CommitApplied(&controller, &applied);
    EXPECT_NEAR(controller.integral_us, 0.0f, 0.001f);

    return 0;
}

static int test_phase_bounds_accept_forward_reverse_brake_and_neutral(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalAppliedOutput_t applied;
    LongitudinalControllerOutput_t output;

    config.kp_us_per_mps = 1.0f;
    config.ki_us_per_mps_s = 10.0f;
    LongitudinalController_Init(&controller, &config);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(1U, 0U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(0U, 1.0f));
    applied = applied_phase_output(output.bounded_output_us,
                                   0.0f,
                                   500.0f,
                                   1U,
                                   0U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(2U, 100U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(100U, 1.0f));
    applied = applied_phase_output(output.bounded_output_us,
                                   0.0f,
                                   500.0f,
                                   1U,
                                   0U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);
    EXPECT_NEAR(controller.integral_us, 1.0f, 0.001f);

    LongitudinalController_Reset(&controller);
    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(3U, 200U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(200U, -1.0f));
    applied = applied_phase_output(output.bounded_output_us,
                                   -500.0f,
                                   0.0f,
                                   1U,
                                   0U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(4U, 300U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(300U, -1.0f));
    applied = applied_phase_output(output.bounded_output_us,
                                   -500.0f,
                                   0.0f,
                                   1U,
                                   0U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);
    EXPECT_NEAR(controller.integral_us, -1.0f, 0.001f);

    LongitudinalController_Reset(&controller);
    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(5U, 400U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(400U, -1.0f));
    applied = applied_phase_output(-50.0f,
                                   -500.0f,
                                   -50.0f,
                                   1U,
                                   0U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);
    EXPECT_NEAR(controller.last_applied_output_us, -50.0f, 0.001f);

    (void)LongitudinalController_ObserveFeedback(
        &controller, sample_ptr(6U, 500U, 0.0f));
    output = LongitudinalController_Evaluate(
        &controller, input_ptr(500U, 0.0f));
    controller.integral_us = 12.0f;
    applied = applied_phase_output(0.0f,
                                   0.0f,
                                   0.0f,
                                   0U,
                                   1U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_OK);
    EXPECT_NEAR(controller.integral_us, 0.0f, 0.001f);
    EXPECT_TRUE(controller.pid_sample_committed == 1U);
    EXPECT_TRUE(controller.pid_committed_sample_id == 6U);

    applied = applied_phase_output(0.0f,
                                   -501.0f,
                                   0.0f,
                                   1U,
                                   0U);
    EXPECT_TRUE(LongitudinalController_CommitApplied(
        &controller, &applied) == LONGITUDINAL_REASON_INVALID_ARGUMENT);

    return 0;
}

int main(void)
{
    if (test_p_only_outputs_error() != 0)
    {
        return 1;
    }
    if (test_integral_updates_on_commit_and_uses_committed_sample_span() != 0)
    {
        return 1;
    }
    if (test_duplicate_sample_does_not_integrate_twice() != 0)
    {
        return 1;
    }
    if (test_derivative_uses_measurement_not_target_step() != 0)
    {
        return 1;
    }
    if (test_target_change_skips_one_error_integral_and_sign_clears() != 0)
    {
        return 1;
    }
    if (test_antiwindup_tracks_applied_output_and_clamps_integral() != 0)
    {
        return 1;
    }
    if (test_invalid_feedback_stale_feedback_and_timebase_fail_closed() != 0)
    {
        return 1;
    }
    if (test_future_now_and_nan_are_rejected() != 0)
    {
        return 1;
    }
    if (test_ki_zero_keeps_integral_zero() != 0)
    {
        return 1;
    }
    if (test_phase_bounds_accept_forward_reverse_brake_and_neutral() != 0)
    {
        return 1;
    }
    return 0;
}
