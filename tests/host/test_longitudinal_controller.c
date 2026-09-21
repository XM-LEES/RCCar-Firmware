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

static LongitudinalControllerConfig_t valid_config(void)
{
    LongitudinalControllerConfig_t config;

    memset(&config, 0, sizeof(config));
    config.center_pwm_us = 1500U;
    config.min_pwm_us = 1000U;
    config.max_pwm_us = 2000U;
    config.forward_limit_pwm_us = 1600U;
    config.reverse_limit_pwm_us = 1400U;
    config.target_slew_rate_mps2 = 100.0f;
    config.pi_enabled = 1U;
    config.pi_kp_us_per_mps = 10.0f;
    config.pi_ki_us_per_mps_s = 4.0f;
    config.pi_trim_limit_us = 20U;
    config.tracking_brake_kp = 1.0f;
    config.tracking_brake_max = 0.70f;
    config.tracking_brake_enter_error_mps = 0.20f;
    config.tracking_brake_release_error_mps = 0.10f;
    return config;
}

static LongitudinalControllerInput_t input_at(uint32_t now_ms)
{
    LongitudinalControllerInput_t input;

    memset(&input, 0, sizeof(input));
    input.automatic_enabled = 1U;
    input.feedback_valid = 1U;
    input.stopped = 1U;
    input.current_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    input.feedback_sample_id = 1U;
    input.feedback_sample_tick_ms = now_ms;
    input.now_tick_ms = now_ms;
    return input;
}

static int test_acceleration_uses_slew_and_forward_feedforward(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    config.target_slew_rate_mps2 = 4.0f;
    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 2.0f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_NEUTRAL);

    input.now_tick_ms = 100U;
    input.feedback_sample_tick_ms = 100U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_FORWARD);
    EXPECT_TRUE(output.drive_pwm_us > config.center_pwm_us);
    EXPECT_TRUE(output.drive_pwm_us <= config.forward_limit_pwm_us);
    EXPECT_TRUE(output.diagnostics.feedforward_pwm_us > config.center_pwm_us);
    EXPECT_TRUE(output.diagnostics.feedforward_pwm_us <=
                config.forward_limit_pwm_us);
    EXPECT_TRUE(output.diagnostics.slew_limited == 1U);
    EXPECT_TRUE(fabsf(output.diagnostics.slewed_target_mps - 0.4f) < 0.001f);

    return 0;
}

static int test_small_nonzero_target_is_not_deadbanded(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 0.01f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_NEUTRAL);

    input.now_tick_ms = 100U;
    input.feedback_sample_tick_ms = 100U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_FORWARD);
    EXPECT_TRUE(fabsf(output.diagnostics.command_target_mps - 0.01f) < 0.0001f);
    EXPECT_TRUE(fabsf(output.diagnostics.slewed_target_mps - 0.01f) < 0.0001f);
    EXPECT_TRUE(output.diagnostics.feedforward_pwm_us == 1546U);

    return 0;
}

static int test_downward_target_requests_tracking_brake(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 2.0f;
    (void)LongitudinalController_Evaluate(&controller, &input);
    input.now_tick_ms = 100U;
    input.feedback_sample_tick_ms = 100U;
    input.feedback_sample_id = 2U;
    input.stopped = 0U;
    input.current_direction = LONGITUDINAL_DIRECTION_FORWARD;
    input.speed_magnitude_mps = 2.0f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);

    input.target_speed_mps = 1.0f;
    input.now_tick_ms = 120U;
    input.feedback_sample_tick_ms = 120U;
    input.feedback_sample_id = 3U;
    input.speed_magnitude_mps = 1.5f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_TRACKING_BRAKE);
    EXPECT_TRUE(output.brake_permitted == 1U);
    EXPECT_TRUE(fabsf(output.normalized_brake_request - 0.5f) < 0.001f);

    return 0;
}

static int test_tracking_brake_hysteresis_holds_then_releases(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 1.0f;
    (void)LongitudinalController_Evaluate(&controller, &input);
    input.stopped = 0U;
    input.current_direction = LONGITUDINAL_DIRECTION_FORWARD;
    input.now_tick_ms = 100U;
    input.feedback_sample_tick_ms = 100U;
    input.feedback_sample_id = 2U;
    input.speed_magnitude_mps = 1.30f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_TRACKING_BRAKE);

    input.now_tick_ms = 120U;
    input.feedback_sample_tick_ms = 120U;
    input.feedback_sample_id = 3U;
    input.speed_magnitude_mps = 1.12f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_TRACKING_BRAKE);

    input.now_tick_ms = 140U;
    input.feedback_sample_tick_ms = 140U;
    input.feedback_sample_id = 4U;
    input.speed_magnitude_mps = 1.05f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.diagnostics.tracking_brake_active == 0U);

    return 0;
}

static int test_zero_target_brakes_until_stopped(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 0.0f;
    input.stopped = 0U;
    input.current_direction = LONGITUDINAL_DIRECTION_FORWARD;
    input.speed_magnitude_mps = 0.5f;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_STOP_BRAKE);
    EXPECT_TRUE(output.normalized_brake_request == 0.0f);

    input.stopped = 1U;
    input.current_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    input.speed_magnitude_mps = 0.0f;
    input.now_tick_ms = 20U;
    input.feedback_sample_tick_ms = 20U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_NEUTRAL);

    return 0;
}

static int test_opposite_signed_target_requests_reversal(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    config.target_slew_rate_mps2 = 4.0f;
    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 1.0f;
    (void)LongitudinalController_Evaluate(&controller, &input);
    input.now_tick_ms = 250U;
    input.feedback_sample_tick_ms = 250U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(fabsf(output.diagnostics.slewed_target_mps - 1.0f) < 0.001f);

    input.target_speed_mps = -1.0f;
    input.stopped = 0U;
    input.current_direction = LONGITUDINAL_DIRECTION_FORWARD;
    input.speed_magnitude_mps = 0.5f;
    input.now_tick_ms = 270U;
    input.feedback_sample_tick_ms = 270U;
    input.feedback_sample_id = 3U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_REVERSAL_REQUEST);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_REVERSE);
    EXPECT_TRUE(output.brake_permitted == 1U);
    EXPECT_TRUE(output.diagnostics.slewed_target_mps == 0.0f);

    input.stopped = 1U;
    input.current_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    input.speed_magnitude_mps = 0.0f;
    input.now_tick_ms = 290U;
    input.feedback_sample_tick_ms = 290U;
    input.feedback_sample_id = 4U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_REVERSE);
    EXPECT_TRUE(output.diagnostics.slewed_target_mps < 0.0f);
    EXPECT_TRUE(output.diagnostics.slewed_target_mps > -0.1f);

    return 0;
}

static int test_stopped_command_sign_change_clears_old_signed_ramp(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    config.target_slew_rate_mps2 = 4.0f;
    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = -2.0f;
    (void)LongitudinalController_Evaluate(&controller, &input);
    input.now_tick_ms = 250U;
    input.feedback_sample_tick_ms = 250U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_REVERSE);
    EXPECT_TRUE(output.diagnostics.slewed_target_mps < -0.9f);

    input.target_speed_mps = 1.0f;
    input.now_tick_ms = 270U;
    input.feedback_sample_tick_ms = 270U;
    input.feedback_sample_id = 3U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_UNKNOWN);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_NEUTRAL);
    EXPECT_TRUE(output.drive_pwm_us == config.center_pwm_us);
    EXPECT_TRUE(output.diagnostics.slewed_target_mps == 0.0f);

    input.now_tick_ms = 290U;
    input.feedback_sample_tick_ms = 290U;
    input.feedback_sample_id = 4U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.target_direction == LONGITUDINAL_DIRECTION_FORWARD);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.drive_pwm_us > config.center_pwm_us);

    return 0;
}

static int test_invalid_feedback_inhibits_automatic_output(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 1.0f;
    input.feedback_valid = 0U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_NEUTRAL);
    EXPECT_TRUE(output.reason == LONGITUDINAL_REASON_FEEDBACK_INVALID);
    EXPECT_TRUE(output.diagnostics.inhibited == 1U);
    EXPECT_TRUE(output.drive_pwm_us == config.center_pwm_us);

    return 0;
}

static int test_duplicate_feedback_sample_does_not_integrate(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;
    float integral_after_new_sample;

    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 1.0f;
    (void)LongitudinalController_Evaluate(&controller, &input);

    input.stopped = 1U;
    input.speed_magnitude_mps = 0.0f;
    input.now_tick_ms = 100U;
    input.feedback_sample_tick_ms = 100U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);

    input.now_tick_ms = 200U;
    input.feedback_sample_tick_ms = 200U;
    input.feedback_sample_id = 3U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    integral_after_new_sample = output.diagnostics.pi_integral_mps_s;
    EXPECT_TRUE(integral_after_new_sample > 0.0f);

    input.now_tick_ms = 220U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.diagnostics.duplicate_sample == 1U);
    EXPECT_TRUE(output.diagnostics.pi_integral_mps_s ==
                integral_after_new_sample);

    return 0;
}

static int test_drive_outputs_are_bounded_by_direction_limits(void)
{
    LongitudinalControllerConfig_t config = valid_config();
    LongitudinalController_t controller;
    LongitudinalControllerInput_t input = input_at(0U);
    LongitudinalControllerOutput_t output;

    config.pi_kp_us_per_mps = 500.0f;
    config.pi_trim_limit_us = 200U;
    LongitudinalController_Init(&controller, &config);

    input.target_speed_mps = 10.0f;
    (void)LongitudinalController_Evaluate(&controller, &input);
    input.now_tick_ms = 100U;
    input.feedback_sample_tick_ms = 100U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.drive_pwm_us == config.forward_limit_pwm_us);
    EXPECT_TRUE(output.drive_pwm_us >= config.center_pwm_us);

    LongitudinalController_Reset(&controller);
    input = input_at(200U);
    input.target_speed_mps = -3.0f;
    input.current_direction = LONGITUDINAL_DIRECTION_REVERSE;
    (void)LongitudinalController_Evaluate(&controller, &input);
    input.stopped = 0U;
    input.speed_magnitude_mps = 0.1f;
    input.current_direction = LONGITUDINAL_DIRECTION_REVERSE;
    input.now_tick_ms = 300U;
    input.feedback_sample_tick_ms = 300U;
    input.feedback_sample_id = 2U;
    output = LongitudinalController_Evaluate(&controller, &input);
    EXPECT_TRUE(output.intent == LONGITUDINAL_INTENT_DRIVE);
    EXPECT_TRUE(output.drive_pwm_us == config.reverse_limit_pwm_us);
    EXPECT_TRUE(output.drive_pwm_us <= config.center_pwm_us);

    return 0;
}

int main(void)
{
    if (test_acceleration_uses_slew_and_forward_feedforward() != 0)
    {
        return 1;
    }
    if (test_small_nonzero_target_is_not_deadbanded() != 0)
    {
        return 1;
    }
    if (test_downward_target_requests_tracking_brake() != 0)
    {
        return 1;
    }
    if (test_tracking_brake_hysteresis_holds_then_releases() != 0)
    {
        return 1;
    }
    if (test_zero_target_brakes_until_stopped() != 0)
    {
        return 1;
    }
    if (test_opposite_signed_target_requests_reversal() != 0)
    {
        return 1;
    }
    if (test_stopped_command_sign_change_clears_old_signed_ramp() != 0)
    {
        return 1;
    }
    if (test_invalid_feedback_inhibits_automatic_output() != 0)
    {
        return 1;
    }
    if (test_duplicate_feedback_sample_does_not_integrate() != 0)
    {
        return 1;
    }
    if (test_drive_outputs_are_bounded_by_direction_limits() != 0)
    {
        return 1;
    }
    return 0;
}
