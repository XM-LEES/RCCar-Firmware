#include "mode2_drive_gate.h"

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

#define EXPECT_EQ_U32(actual, expected) \
    EXPECT_TRUE((uint32_t)(actual) == (uint32_t)(expected))

#define TEST_CENTER_PWM_US 1500U
#define TEST_FORWARD_PWM_US 2000U
#define TEST_REVERSE_PWM_US 1000U

static Mode2DriveGateConfig_t valid_config(void)
{
    Mode2DriveGateConfig_t config;

    memset(&config, 0, sizeof(config));
    config.center_pwm_us = TEST_CENTER_PWM_US;
    config.forward_pwm_us = TEST_FORWARD_PWM_US;
    config.reverse_pwm_us = TEST_REVERSE_PWM_US;
    config.brake_min_us = 50U;
    config.action_ack_ms = 300U;
    config.qualify_ms = 100U;
    config.neutral_dwell_ms = 100U;
    config.feedback_timeout_ms = 250U;
    config.command_timeout_ms = 250U;
    config.coast_eval_ms = 200U;
    config.coast_budget_ms = 600U;
    config.release_delay_ms = 80U;
    config.e_on_abs_mps = 0.20f;
    config.e_on_ratio = 0.10f;
    config.e_off_abs_mps = 0.05f;
    config.e_off_ratio = 0.02f;
    config.coast_progress_mps2 = 0.05f;
    config.stopped_speed_threshold_mps = 0.05f;
    config.stopped_min_samples = 3U;
    config.stopped_min_coverage_ms = 150U;
    return config;
}

static Mode2DriveGateInput_t input_cmd(float target_mps,
                                       float pid_us,
                                       uint32_t tick_ms)
{
    Mode2DriveGateInput_t input;

    memset(&input, 0, sizeof(input));
    input.command_valid = 1U;
    input.propulsion_authorized = 1U;
    input.pid_valid = 1U;
    input.command_tick_ms = tick_ms;
    input.target_speed_mps = target_mps;
    input.pid_raw_us = pid_us;
    return input;
}

static void init_gate(Mode2DriveGate_t *gate)
{
    Mode2DriveGateConfig_t config = valid_config();

    Mode2DriveGate_Init(gate, &config);
}

static Mode2DriveMotionObservation_t obs_base(uint32_t sample_id,
                                              uint32_t tick_ms)
{
    Mode2DriveMotionObservation_t observation;

    memset(&observation, 0, sizeof(observation));
    observation.available = 1U;
    observation.speed_magnitude_valid = 1U;
    observation.sample_id = sample_id;
    observation.sample_tick_ms = tick_ms;
    observation.esc_action = MODE2_DRIVE_ESC_ACTION_NEUTRAL;
    return observation;
}

static Mode2DriveMotionObservation_t stopped_obs(uint32_t sample_id,
                                                 uint32_t tick_ms,
                                                 uint32_t session_id,
                                                 EscTelemetryOutputPurpose_t purpose,
                                                 Mode2DriveEscAction_t esc_action)
{
    Mode2DriveMotionObservation_t observation = obs_base(sample_id, tick_ms);

    observation.speed_magnitude_mps = 0.0f;
    observation.signed_speed_mps = 0.0f;
    observation.context_valid = 1U;
    observation.context_pwm_us = TEST_CENTER_PWM_US;
    observation.context_session_id = session_id;
    observation.context_purpose = purpose;
    observation.esc_action = esc_action;
    return observation;
}

static Mode2DriveMotionObservation_t motion_obs(uint32_t sample_id,
                                                uint32_t tick_ms,
                                                float speed_mps,
                                                Mode2DriveTargetDirection_t direction,
                                                float acceleration_mps2,
                                                uint32_t session_id,
                                                EscTelemetryOutputPurpose_t purpose,
                                                uint16_t context_pwm_us,
                                                Mode2DriveEscAction_t esc_action)
{
    Mode2DriveMotionObservation_t observation = obs_base(sample_id, tick_ms);

    observation.speed_magnitude_mps = fabsf(speed_mps);
    observation.direction_known = 1U;
    observation.direction = direction;
    observation.signed_speed_mps =
        (direction == MODE2_DRIVE_TARGET_REVERSE) ?
        -fabsf(speed_mps) : fabsf(speed_mps);
    observation.acceleration_valid = 1U;
    observation.acceleration_mps2 = acceleration_mps2;
    observation.context_valid = 1U;
    observation.context_pwm_us = context_pwm_us;
    observation.context_session_id = session_id;
    observation.context_purpose = purpose;
    observation.esc_action = esc_action;
    return observation;
}

static Mode2DriveMotionObservation_t unknown_motion_obs(
    uint32_t sample_id,
    uint32_t tick_ms,
    float speed_mps,
    uint32_t session_id,
    EscTelemetryOutputPurpose_t purpose,
    uint16_t context_pwm_us,
    Mode2DriveEscAction_t esc_action)
{
    Mode2DriveMotionObservation_t observation = obs_base(sample_id, tick_ms);

    observation.speed_magnitude_mps = fabsf(speed_mps);
    observation.direction_known = 0U;
    observation.signed_speed_mps = 0.0f;
    observation.context_valid = 1U;
    observation.context_pwm_us = context_pwm_us;
    observation.context_session_id = session_id;
    observation.context_purpose = purpose;
    observation.esc_action = esc_action;
    return observation;
}

static void observe_value(Mode2DriveGate_t *gate,
                          Mode2DriveMotionObservation_t observation,
                          uint32_t tick_ms)
{
    Mode2DriveGate_Observe(gate, &observation, tick_ms);
}

static Mode2DriveGateOutput_t eval_value(Mode2DriveGate_t *gate,
                                         Mode2DriveGateInput_t input,
                                         uint32_t tick_ms)
{
    return Mode2DriveGate_EvaluateControl(gate, &input, tick_ms);
}

static Mode2DriveGateOutput_t eval_commit(Mode2DriveGate_t *gate,
                                          Mode2DriveGateInput_t input,
                                          uint32_t tick_ms)
{
    Mode2DriveGateOutput_t output =
        Mode2DriveGate_EvaluateControl(gate, &input, tick_ms);
    Mode2DriveGate_CommitApplied(gate, &output, output.final_pwm_us, tick_ms);
    return output;
}

static void observe_stop_window(Mode2DriveGate_t *gate,
                                uint32_t first_id,
                                uint32_t first_tick_ms,
                                uint32_t session_id,
                                EscTelemetryOutputPurpose_t purpose,
                                Mode2DriveEscAction_t esc_action)
{
    observe_value(gate,
                  stopped_obs(first_id, first_tick_ms,
                              session_id, purpose, esc_action),
                  first_tick_ms);
    observe_value(gate,
                  stopped_obs(first_id + 1U, first_tick_ms + 75U,
                              session_id, purpose, esc_action),
                  first_tick_ms + 75U);
    observe_value(gate,
                  stopped_obs(first_id + 2U, first_tick_ms + 150U,
                              session_id, purpose, esc_action),
                  first_tick_ms + 150U);
}

static void observe_stop_window_with_pwm(Mode2DriveGate_t *gate,
                                         uint32_t first_id,
                                         uint32_t first_tick_ms,
                                         uint32_t session_id,
                                         EscTelemetryOutputPurpose_t purpose,
                                         Mode2DriveEscAction_t esc_action,
                                         uint16_t context_pwm_us)
{
    Mode2DriveMotionObservation_t observation;

    observation = stopped_obs(first_id, first_tick_ms,
                              session_id, purpose, esc_action);
    observation.context_pwm_us = context_pwm_us;
    observe_value(gate, observation, first_tick_ms);

    observation = stopped_obs(first_id + 1U, first_tick_ms + 75U,
                              session_id, purpose, esc_action);
    observation.context_pwm_us = context_pwm_us;
    observe_value(gate, observation, first_tick_ms + 75U);

    observation = stopped_obs(first_id + 2U, first_tick_ms + 150U,
                              session_id, purpose, esc_action);
    observation.context_pwm_us = context_pwm_us;
    observe_value(gate, observation, first_tick_ms + 150U);
}

static void prime_neutral_ready(Mode2DriveGate_t *gate,
                                float target_mps,
                                float pid_us)
{
    Mode2DriveGateOutput_t output =
        eval_commit(gate, input_cmd(target_mps, pid_us, 0U), 0U);

    observe_stop_window(gate, 1U, 20U, output.session_id,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_NEUTRAL);
}

static void establish_forward_ready(Mode2DriveGate_t *gate)
{
    Mode2DriveGateInput_t input = input_cmd(1.0f, 120.0f, 200U);
    Mode2DriveGateOutput_t output;

    prime_neutral_ready(gate, 1.0f, 120.0f);
    output = eval_commit(gate, input, 200U);
    observe_value(gate,
        motion_obs(10U, 220U, 0.30f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   output.session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        220U);
}

static void establish_reverse_ready(Mode2DriveGate_t *gate)
{
    Mode2DriveGateInput_t input = input_cmd(-1.0f, -120.0f, 200U);
    Mode2DriveGateOutput_t output;

    prime_neutral_ready(gate, -1.0f, -120.0f);
    output = eval_commit(gate, input, 200U);
    observe_value(gate,
        motion_obs(10U, 220U, 0.30f, MODE2_DRIVE_TARGET_REVERSE, 0.0f,
                   output.session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        220U);
}

static int test_config_validation_is_explicit(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveReason_t reason;

    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) != 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_OK);

    config = valid_config();
    config.brake_min_us = 0U;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.reverse_pwm_us = TEST_CENTER_PWM_US;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.coast_eval_ms = config.coast_budget_ms;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    return 0;
}

static int test_startup_forward_and_drive_confirmation(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = input_cmd(1.0f, 120.0f, 200U);
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    prime_neutral_ready(&gate, 1.0f, 120.0f);

    output = eval_commit(&gate, input, 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_START);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST);
    EXPECT_TRUE(output.final_pwm_us > TEST_CENTER_PWM_US);

    /* The ESC can still report NEUTRAL for several frames while the motor
     * crosses its electrical dead zone. The just-issued propulsion request
     * must remain applied until fresh DRIVE feedback arrives. */
    output = eval_commit(&gate, input_cmd(1.0f, 120.0f, 220U), 220U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_START);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST);
    EXPECT_TRUE(output.final_pwm_us > TEST_CENTER_PWM_US);

    observe_value(&gate,
        motion_obs(10U, 220U, 0.30f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   output.session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        220U);
    output = eval_value(&gate, input_cmd(1.0f, 130.0f, 240U), 240U);
    EXPECT_TRUE(output.permission == MODE2_DRIVE_PERMISSION_F_READY);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_DRIVE);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);

    return 0;
}

static int test_target_direction_uses_strict_sign_not_stop_deadband(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    prime_neutral_ready(&gate, 0.01f, 25.0f);

    output = eval_value(&gate, input_cmd(0.01f, 25.0f, 200U), 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_START);

    init_gate(&gate);
    prime_neutral_ready(&gate, -0.01f, -25.0f);
    output = eval_value(&gate, input_cmd(-0.01f, -25.0f, 200U), 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_R_QUERY);

    return 0;
}

static int test_forward_large_overspeed_brakes_with_pid_floor(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 260U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        260U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 280U), 280U);

    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(output.brake_purpose == MODE2_DRIVE_BRAKE_PURPOSE_TRACK);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_BRAKE_TRACK);
    EXPECT_EQ_U32(output.final_pwm_us, TEST_CENTER_PWM_US - 200U);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE);

    return 0;
}

static int test_first_forward_brake_removes_existing_integral(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 260U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        260U);

    input = input_cmd(1.0f, 100.0f, 280U);
    input.pid_integral_us = 300.0f;
    output = eval_value(&gate, input, 280U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_EQ_U32(output.final_pwm_us, TEST_CENTER_PWM_US - 200U);
    EXPECT_TRUE(output.integral_reset != 0U);

    return 0;
}

static int test_small_overspeed_coasts_then_brakes_when_not_decelerating(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 1.12f, MODE2_DRIVE_TARGET_FORWARD, -0.02f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1600U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -30.0f, 300U), 300U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_COAST);

    observe_value(&gate,
        motion_obs(12U, 520U, 1.11f, MODE2_DRIVE_TARGET_FORWARD, -0.02f,
                   output.session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                   TEST_CENTER_PWM_US,
                   MODE2_DRIVE_ESC_ACTION_NEUTRAL),
        520U);
    output = eval_commit(&gate, input_cmd(1.0f, -30.0f, 520U), 520U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_EQ_U32(output.final_pwm_us, TEST_CENTER_PWM_US - 50U);

    return 0;
}

static int test_reduced_target_does_not_restart_forward_coast_budget(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 1.12f, MODE2_DRIVE_TARGET_FORWARD, -0.02f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1600U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -30.0f, 300U), 300U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_COAST);

    observe_value(&gate,
        motion_obs(12U, 520U, 1.06f, MODE2_DRIVE_TARGET_FORWARD, -0.02f,
                   output.session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                   TEST_CENTER_PWM_US,
                   MODE2_DRIVE_ESC_ACTION_NEUTRAL),
        520U);
    output = eval_value(&gate, input_cmd(0.95f, -30.0f, 520U), 520U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(output.brake_purpose == MODE2_DRIVE_BRAKE_PURPOSE_TRACK);

    return 0;
}

static int test_track_brake_release_does_not_repeat_without_forward_drive(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t brake_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    brake_session = output.session_id;
    observe_value(&gate,
        motion_obs(12U, 340U, 1.03f, MODE2_DRIVE_TARGET_FORWARD, -0.5f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    output = eval_commit(&gate, input_cmd(1.0f, 0.0f, 360U), 360U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_RECOVER);

    observe_value(&gate,
        motion_obs(13U, 390U, 1.40f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   output.session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                   TEST_CENTER_PWM_US,
                   MODE2_DRIVE_ESC_ACTION_NEUTRAL),
        390U);
    output = eval_value(&gate, input_cmd(1.0f, -100.0f, 400U), 400U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_BRAKE_REARM_REQUIRED);

    return 0;
}

static int test_positive_pid_does_not_release_track_brake_while_still_overspeed(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t brake_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    brake_session = output.session_id;
    observe_value(&gate,
        motion_obs(12U, 340U, 1.40f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 360U), 360U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_BRAKE_TRACK);

    return 0;
}

static int test_track_brake_predictive_release_uses_observed_fresh_frames(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t brake_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    brake_session = output.session_id;
    observe_value(&gate,
        motion_obs(12U, 340U, 1.40f, MODE2_DRIVE_TARGET_FORWARD, -0.2f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    output = eval_commit(&gate, input_cmd(1.0f, -120.0f, 360U), 360U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    observe_value(&gate,
        motion_obs(13U, 380U, 1.20f, MODE2_DRIVE_TARGET_FORWARD, -2.0f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        380U);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 400U), 400U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    observe_value(&gate,
        motion_obs(14U, 400U, 1.20f, MODE2_DRIVE_TARGET_FORWARD, -2.0f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        400U);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 420U), 420U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_RECOVER);

    return 0;
}

static int test_track_brake_prediction_resets_on_intervening_bad_sample(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t brake_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    brake_session = output.session_id;
    observe_value(&gate,
        motion_obs(12U, 340U, 1.40f, MODE2_DRIVE_TARGET_FORWARD, -0.2f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    output = eval_commit(&gate, input_cmd(1.0f, -120.0f, 360U), 360U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    observe_value(&gate,
        motion_obs(13U, 380U, 1.20f, MODE2_DRIVE_TARGET_FORWARD, -2.0f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        380U);
    observe_value(&gate,
        motion_obs(14U, 400U, 1.20f, MODE2_DRIVE_TARGET_FORWARD, 0.1f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        400U);
    observe_value(&gate,
        motion_obs(15U, 420U, 1.20f, MODE2_DRIVE_TARGET_FORWARD, -2.0f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        420U);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 440U), 440U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_BRAKE_TRACK);

    return 0;
}

static int test_forward_to_reverse_requires_full_same_session_brake_and_fresh_stop(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t brake_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 0.6f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1600U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(-1.0f, -300.0f, 320U), 320U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_EQ_U32(output.final_pwm_us, TEST_REVERSE_PWM_US);
    brake_session = output.session_id;

    observe_value(&gate,
        motion_obs(12U, 350U, 0.2f, MODE2_DRIVE_TARGET_FORWARD, -0.4f,
                   brake_session + 1U,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   TEST_REVERSE_PWM_US,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        350U);
    observe_stop_window_with_pwm(&gate, 20U, 430U, brake_session + 1U,
                                 ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                                 MODE2_DRIVE_ESC_ACTION_BRAKE,
                                 TEST_REVERSE_PWM_US);
    output = eval_value(&gate, input_cmd(-1.0f, -300.0f, 600U), 600U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    observe_value(&gate,
        motion_obs(30U, 610U, 0.01f, MODE2_DRIVE_TARGET_FORWARD, -0.4f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   TEST_REVERSE_PWM_US,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        610U);
    observe_stop_window_with_pwm(&gate, 31U, 620U, brake_session,
                                 ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                                 MODE2_DRIVE_ESC_ACTION_BRAKE,
                                 TEST_REVERSE_PWM_US);
    output = eval_commit(&gate, input_cmd(-1.0f, -300.0f, 800U), 800U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_R_DWELL);

    observe_value(&gate,
        stopped_obs(40U, 820U, output.session_id,
                    ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                    MODE2_DRIVE_ESC_ACTION_NEUTRAL),
        820U);
    output = eval_value(&gate, input_cmd(-1.0f, -120.0f, 890U), 890U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = eval_value(&gate, input_cmd(-1.0f, -120.0f, 920U), 920U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

static int test_reverse_to_forward_coasts_without_positive_brake(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_reverse_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 0.8f, MODE2_DRIVE_TARGET_REVERSE, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                   1350U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, 150.0f, 320U), 320U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_R_COAST);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL);

    observe_stop_window(&gate, 20U, 340U, output.session_id,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_NEUTRAL);
    observe_value(&gate,
        stopped_obs(30U, 520U, output.session_id,
                    ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                    MODE2_DRIVE_ESC_ACTION_NEUTRAL),
        520U);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 560U), 560U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 580U), 580U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_START);

    return 0;
}

static int test_reverse_start_then_forward_change_requires_new_neutral_stop(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    prime_neutral_ready(&gate, -1.0f, -120.0f);
    gate.phase = MODE2_DRIVE_PHASE_R_DWELL;
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 200U), 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_R_START);

    output = eval_commit(&gate, input_cmd(1.0f, 120.0f, 220U), 220U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_R_COAST);

    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 240U), 240U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);

    observe_stop_window(&gate, 20U, 260U, output.session_id,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_NEUTRAL);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 440U), 440U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 460U), 460U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);

    return 0;
}

static int test_reverse_query_brake_upgrades_continuous_negative_session(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t query_session;
    Mode2DriveMotionObservation_t brake_observation;

    init_gate(&gate);
    prime_neutral_ready(&gate, -1.0f, -120.0f);
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 200U), 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_R_QUERY);
    query_session = output.session_id;

    brake_observation = stopped_obs(10U, 230U, query_session,
                                    ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                                    MODE2_DRIVE_ESC_ACTION_BRAKE);
    brake_observation.context_pwm_us = output.final_pwm_us;
    observe_value(&gate, brake_observation, 230U);
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 240U), 240U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_EQ_U32(output.final_pwm_us, TEST_REVERSE_PWM_US);
    EXPECT_EQ_U32(output.session_id, query_session);

    return 0;
}

static int test_initial_brake_zero_rpm_without_neutral_ack_cannot_start(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    output = eval_commit(&gate, input_cmd(1.0f, 120.0f, 0U), 0U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);

    observe_stop_window(&gate, 1U, 20U, output.session_id,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_BRAKE);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 200U), 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_STOP);

    return 0;
}

static int test_full_brake_ack_timeout_ignores_prior_weak_reverse_query_brake(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGateOutput_t output;
    Mode2DriveMotionObservation_t brake_observation;
    uint32_t query_session;

    config.feedback_timeout_ms = 1000U;
    Mode2DriveGate_Init(&gate, &config);
    prime_neutral_ready(&gate, -1.0f, -120.0f);
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 200U), 200U);
    query_session = output.session_id;

    brake_observation = stopped_obs(10U, 230U, query_session,
                                    ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                                    MODE2_DRIVE_ESC_ACTION_BRAKE);
    brake_observation.context_pwm_us = output.final_pwm_us;
    observe_value(&gate, brake_observation, 230U);
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 240U), 240U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    observe_value(&gate,
        motion_obs(11U, 560U, 0.3f, MODE2_DRIVE_TARGET_FORWARD, -0.1f,
                   query_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   TEST_REVERSE_PWM_US + 100U,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        560U);
    output = eval_value(&gate, input_cmd(-1.0f, -120.0f, 560U), 560U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED);
    EXPECT_TRUE(output.action_conflict != 0U);

    return 0;
}

static int test_stop_brake_allows_neutral_after_fresh_stop_without_conflict(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t brake_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 0.8f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1600U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(0.0f, -100.0f, 320U), 320U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    brake_session = output.session_id;

    observe_value(&gate,
        motion_obs(12U, 340U, 0.1f, MODE2_DRIVE_TARGET_FORWARD, -0.5f,
                   brake_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    observe_stop_window_with_pwm(&gate, 20U, 360U, brake_session,
                                 ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                                 MODE2_DRIVE_ESC_ACTION_BRAKE,
                                 output.final_pwm_us);
    observe_value(&gate,
        stopped_obs(30U, 540U, brake_session,
                    ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                    MODE2_DRIVE_ESC_ACTION_NEUTRAL),
        540U);
    EXPECT_TRUE(Mode2DriveGate_HasActionConflict(&gate) == 0U);
    output = eval_value(&gate, input_cmd(0.0f, -100.0f, 560U), 560U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_TARGET_NEUTRAL);

    return 0;
}

static int test_startup_hold_does_not_require_pid_for_unknown_direction_motion(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    prime_neutral_ready(&gate, 1.0f, 120.0f);
    output = eval_commit(&gate, input_cmd(1.0f, 120.0f, 200U), 200U);
    observe_value(&gate,
        unknown_motion_obs(10U, 220U, 0.3f,
                           output.session_id,
                           ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                           output.final_pwm_us,
                           MODE2_DRIVE_ESC_ACTION_DRIVE),
        220U);

    input = input_cmd(1.0f, 0.0f, 240U);
    input.pid_valid = 0U;
    output = eval_value(&gate, input, 240U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_STARTUP_HOLD);
    EXPECT_TRUE(output.pid_active == 0U);

    return 0;
}

static int test_ready_reverse_unknown_first_motion_holds_previous_pwm(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_reverse_ready(&gate);
    observe_value(&gate,
        unknown_motion_obs(11U, 240U, 0.4f,
                           gate.current_session_id,
                           ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                           gate.last_pwm_us,
                           MODE2_DRIVE_ESC_ACTION_DRIVE),
        240U);
    output = eval_value(&gate, input_cmd(-1.0f, -160.0f, 260U), 260U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_STARTUP_HOLD);
    EXPECT_TRUE(output.pid_active == 0U);
    EXPECT_EQ_U32(output.final_pwm_us, gate.last_pwm_us);

    return 0;
}

static int test_brake_action_conflict_is_exposed(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    session = output.session_id;
    observe_value(&gate,
        motion_obs(12U, 340U, 2.8f, MODE2_DRIVE_TARGET_FORWARD, -0.2f,
                   session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    observe_value(&gate,
        motion_obs(13U, 360U, 2.7f, MODE2_DRIVE_TARGET_FORWARD, -0.2f,
                   session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        360U);
    EXPECT_TRUE(Mode2DriveGate_HasActionConflict(&gate) != 0U);
    output = eval_value(&gate, input_cmd(1.0f, -200.0f, 380U), 380U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.action_conflict != 0U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ACTION_CONFLICT);

    return 0;
}

static int test_action_conflict_recovers_after_new_neutral_stopped_window(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   gate.current_session_id,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    session = output.session_id;
    observe_value(&gate,
        motion_obs(12U, 340U, 2.8f, MODE2_DRIVE_TARGET_FORWARD, -0.2f,
                   session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_BRAKE),
        340U);
    observe_value(&gate,
        motion_obs(13U, 360U, 2.7f, MODE2_DRIVE_TARGET_FORWARD, -0.2f,
                   session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                   output.final_pwm_us,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        360U);
    EXPECT_TRUE(Mode2DriveGate_HasActionConflict(&gate) != 0U);

    output = eval_value(&gate, input_cmd(1.0f, -200.0f, 380U), 380U);
    Mode2DriveGate_CommitApplied(&gate, &output, output.final_pwm_us, 380U);
    observe_stop_window(&gate, 20U, 400U, output.session_id,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_NEUTRAL);
    EXPECT_TRUE(Mode2DriveGate_HasActionConflict(&gate) == 0U);

    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 580U), 580U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.phase == MODE2_DRIVE_PHASE_F_START);

    return 0;
}

static int test_old_forward_drive_frame_cannot_recreate_forward_ready_while_braking(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t old_forward_session;

    init_gate(&gate);
    establish_forward_ready(&gate);
    old_forward_session = gate.current_session_id;
    observe_value(&gate,
        motion_obs(11U, 300U, 3.0f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   old_forward_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        300U);
    output = eval_commit(&gate, input_cmd(1.0f, -200.0f, 320U), 320U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    observe_value(&gate,
        motion_obs(12U, 340U, 2.9f, MODE2_DRIVE_TARGET_FORWARD, 0.0f,
                   old_forward_session,
                   ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                   1700U,
                   MODE2_DRIVE_ESC_ACTION_DRIVE),
        340U);
    EXPECT_TRUE(gate.permission == MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE);
    output = eval_value(&gate, input_cmd(1.0f, -200.0f, 360U), 360U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    return 0;
}

static int test_unknown_esc_action_invalidates_feedback_until_new_valid_frame(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;

    init_gate(&gate);
    establish_forward_ready(&gate);
    observe_value(&gate,
        unknown_motion_obs(11U, 300U, 0.5f,
                           gate.current_session_id,
                           ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                           gate.last_pwm_us,
                           MODE2_DRIVE_ESC_ACTION_UNKNOWN),
        300U);
    output = eval_value(&gate, input_cmd(1.0f, 120.0f, 320U), 320U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_MOTION_UNAVAILABLE);

    return 0;
}

static int test_session_id_wraps_inside_transport_metadata_width(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    const uint32_t max_session =
        ESC_TELEMETRY_METADATA_SESSION_MASK >>
        ESC_TELEMETRY_METADATA_SESSION_SHIFT;

    init_gate(&gate);
    memset(&output, 0, sizeof(output));
    gate.current_session_id = max_session;
    gate.has_output_context = 1U;
    gate.last_purpose = ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST;
    gate.last_pwm_us = TEST_CENTER_PWM_US + 100U;
    output.purpose = ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL;
    output.phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
    output.final_pwm_us = TEST_CENTER_PWM_US;

    Mode2DriveGate_CommitApplied(&gate, &output, TEST_CENTER_PWM_US, 100U);
    EXPECT_EQ_U32(gate.current_session_id, 1U);
    EXPECT_EQ_U32(output.session_id, 1U);

    return 0;
}

static int test_center_actual_pwm_publishes_neutral_and_ends_negative_session(void)
{
    Mode2DriveGate_t gate;
    Mode2DriveGateOutput_t output;
    uint32_t neutral_session;
    uint32_t reverse_session;
    Mode2DriveMotionObservation_t old_brake;

    init_gate(&gate);
    prime_neutral_ready(&gate, -1.0f, -120.0f);

    output = eval_commit(&gate, input_cmd(-1.0f, 0.0f, 200U), 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.final_pwm_us == TEST_CENTER_PWM_US);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL);
    neutral_session = output.session_id;

    observe_stop_window(&gate, 10U, 220U, neutral_session,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_NEUTRAL);
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 400U), 400U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST);
    EXPECT_TRUE(output.final_pwm_us < TEST_CENTER_PWM_US);
    EXPECT_TRUE(output.session_id != neutral_session);
    reverse_session = output.session_id;

    output = eval_commit(&gate, input_cmd(-1.0f, 0.0f, 420U), 420U);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL);
    EXPECT_TRUE(output.session_id != reverse_session);
    neutral_session = output.session_id;

    observe_stop_window(&gate, 20U, 440U, neutral_session,
                        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                        MODE2_DRIVE_ESC_ACTION_NEUTRAL);
    output = eval_commit(&gate, input_cmd(-1.0f, -120.0f, 620U), 620U);
    EXPECT_TRUE(output.session_id != reverse_session);
    EXPECT_TRUE(output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST);

    old_brake = stopped_obs(30U, 640U, reverse_session,
                            ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                            MODE2_DRIVE_ESC_ACTION_BRAKE);
    old_brake.context_pwm_us = TEST_REVERSE_PWM_US;
    observe_value(&gate, old_brake, 640U);
    EXPECT_TRUE(gate.permission != MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE);
    EXPECT_TRUE(gate.brake_active == 0U);

    return 0;
}

int main(void)
{
    if (test_config_validation_is_explicit() != 0)
    {
        return 1;
    }
    if (test_startup_forward_and_drive_confirmation() != 0)
    {
        return 1;
    }
    if (test_target_direction_uses_strict_sign_not_stop_deadband() != 0)
    {
        return 1;
    }
    if (test_forward_large_overspeed_brakes_with_pid_floor() != 0)
    {
        return 1;
    }
    if (test_first_forward_brake_removes_existing_integral() != 0)
    {
        return 1;
    }
    if (test_small_overspeed_coasts_then_brakes_when_not_decelerating() != 0)
    {
        return 1;
    }
    if (test_reduced_target_does_not_restart_forward_coast_budget() != 0)
    {
        return 1;
    }
    if (test_track_brake_release_does_not_repeat_without_forward_drive() != 0)
    {
        return 1;
    }
    if (test_positive_pid_does_not_release_track_brake_while_still_overspeed() != 0)
    {
        return 1;
    }
    if (test_track_brake_predictive_release_uses_observed_fresh_frames() != 0)
    {
        return 1;
    }
    if (test_track_brake_prediction_resets_on_intervening_bad_sample() != 0)
    {
        return 1;
    }
    if (test_forward_to_reverse_requires_full_same_session_brake_and_fresh_stop() != 0)
    {
        return 1;
    }
    if (test_reverse_to_forward_coasts_without_positive_brake() != 0)
    {
        return 1;
    }
    if (test_reverse_start_then_forward_change_requires_new_neutral_stop() != 0)
    {
        return 1;
    }
    if (test_reverse_query_brake_upgrades_continuous_negative_session() != 0)
    {
        return 1;
    }
    if (test_initial_brake_zero_rpm_without_neutral_ack_cannot_start() != 0)
    {
        return 1;
    }
    if (test_full_brake_ack_timeout_ignores_prior_weak_reverse_query_brake() != 0)
    {
        return 1;
    }
    if (test_stop_brake_allows_neutral_after_fresh_stop_without_conflict() != 0)
    {
        return 1;
    }
    if (test_startup_hold_does_not_require_pid_for_unknown_direction_motion() != 0)
    {
        return 1;
    }
    if (test_ready_reverse_unknown_first_motion_holds_previous_pwm() != 0)
    {
        return 1;
    }
    if (test_brake_action_conflict_is_exposed() != 0)
    {
        return 1;
    }
    if (test_action_conflict_recovers_after_new_neutral_stopped_window() != 0)
    {
        return 1;
    }
    if (test_old_forward_drive_frame_cannot_recreate_forward_ready_while_braking() != 0)
    {
        return 1;
    }
    if (test_unknown_esc_action_invalidates_feedback_until_new_valid_frame() != 0)
    {
        return 1;
    }
    if (test_session_id_wraps_inside_transport_metadata_width() != 0)
    {
        return 1;
    }
    if (test_center_actual_pwm_publishes_neutral_and_ends_negative_session() != 0)
    {
        return 1;
    }
    return 0;
}
