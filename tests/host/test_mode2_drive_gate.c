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

static Mode2DriveGateConfig_t valid_config(void)
{
    Mode2DriveGateConfig_t config;

    memset(&config, 0, sizeof(config));
    config.calibration_valid = 1U;
    config.brake_calibration_valid = 1U;
    config.first_strike_calibration_valid = 1U;
    config.neutral_dwell_valid = 1U;
    config.reversal_timeout_valid = 1U;
    config.brake_request = 0.80f;
    config.reverse_first_strike_request = 0.70f;
    config.reverse_first_strike_min_ms = 60U;
    config.neutral_dwell_ms = 40U;
    config.reversal_timeout_ms = 500U;
    return config;
}

static Mode2DriveGateInput_t request(Mode2DriveTargetDirection_t direction)
{
    Mode2DriveGateInput_t input;

    memset(&input, 0, sizeof(input));
    input.target_direction = direction;
    input.propulsion_authorized = 1U;
    return input;
}

static EscMotionEstimate_t motion_forward_moving(void)
{
    EscMotionEstimate_t motion;

    memset(&motion, 0, sizeof(motion));
    motion.config_valid = 1U;
    motion.has_sample = 1U;
    motion.sample_fresh = 1U;
    motion.rpm_valid = 1U;
    motion.magnitude_valid = 1U;
    motion.direction_valid = 1U;
    motion.signed_speed_valid = 1U;
    motion.direction = ESC_MOTION_DIRECTION_FORWARD;
    motion.speed_magnitude_mps = 1.0f;
    motion.signed_speed_mps = 1.0f;
    return motion;
}

static EscMotionEstimate_t motion_reverse_moving(void)
{
    EscMotionEstimate_t motion = motion_forward_moving();

    motion.direction = ESC_MOTION_DIRECTION_REVERSE;
    motion.signed_speed_mps = -1.0f;
    return motion;
}

static EscMotionEstimate_t motion_stopped(uint32_t established_tick_ms)
{
    EscMotionEstimate_t motion = motion_forward_moving();

    motion.direction_valid = 0U;
    motion.signed_speed_valid = 0U;
    motion.direction = ESC_MOTION_DIRECTION_UNKNOWN;
    motion.speed_magnitude_mps = 0.0f;
    motion.signed_speed_mps = 0.0f;
    motion.stopped = 1U;
    motion.stop_established_tick_ms = established_tick_ms;
    return motion;
}

static EscMotionEstimatorConfig_t estimator_config(void)
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
    config.motor_pole_pairs = 1U;
    config.gear_ratio = 1.0f;
    config.wheel_ratio = 1.0f;
    config.wheel_circumference_m = 1.0f;
    config.telemetry_timeout_ms = 100U;
    config.stopped_speed_threshold_mps = 0.1f;
    config.stopped_min_samples = 2U;
    config.stopped_min_coverage_ms = 20U;
    return config;
}

static void observe_estimator_zero(EscMotionEstimator_t *estimator,
                                   uint32_t sample_id,
                                   uint32_t tick_ms)
{
    EscFe32Sample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.sample_id = sample_id;
    sample.received_tick_ms = tick_ms;
    sample.rpm_valid = 1U;
    sample.erpm_candidate = 0U;
    (void)EscMotionEstimator_ObserveSample(estimator, &sample, tick_ms);
}

static void commit_action(Mode2DriveGate_t *gate,
                          Mode2DriveAction_t action,
                          uint32_t tick_ms,
                          float applied_normalized_brake)
{
    Mode2DriveGate_CommitAppliedActionWithEvidence(gate,
                                                   action,
                                                   tick_ms,
                                                   applied_normalized_brake);
}

static void commit_output(Mode2DriveGate_t *gate,
                          const Mode2DriveGateOutput_t *output,
                          uint32_t tick_ms)
{
    Mode2DriveGate_CommitAppliedActionWithEvidence(gate,
                                                   output->action,
                                                   tick_ms,
                                                   output->normalized_brake_request);
}

static int test_config_authorization_and_feedback_fail_closed(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;
    Mode2DriveReason_t reason;

    config.brake_calibration_valid = 0U;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_CONFIG_INVALID);

    config = valid_config();
    config.reverse_first_strike_request = INFINITY;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_CONFIG_INVALID);

    config = valid_config();
    config.brake_request = 0.0f;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_CONFIG_INVALID);

    config = valid_config();
    config.neutral_dwell_ms = 0x80000000UL;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_CONFIG_INVALID);

    Mode2DriveGate_Init(&gate, &config);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.propulsion_permitted == 0U);

    config = valid_config();
    Mode2DriveGate_Init(&gate, &config);
    input.propulsion_authorized = 0U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_NOT_AUTHORIZED);

    input.propulsion_authorized = 1U;
    input.target_direction = MODE2_DRIVE_TARGET_REVERSE;
    motion = motion_stopped(20U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 25U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);

    input.target_direction = MODE2_DRIVE_TARGET_FORWARD;
    motion = motion_forward_moving();
    motion.sample_fresh = 0U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 30U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_MOTION_UNAVAILABLE);

    input.target_direction = MODE2_DRIVE_TARGET_REVERSE;
    motion = motion_stopped(40U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 40U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);

    return 0;
}

static int test_uncommitted_requests_do_not_advance_history(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);

    motion = motion_stopped(80U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    return 0;
}

static int test_forward_to_reverse_legal_sequence(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.brake_permitted == 1U);
    commit_output(&gate, &output, 10U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 40U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 40U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 69U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 69U);

    motion = motion_stopped(70U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 70U);

    motion = motion_stopped(100U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 100U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 139U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.reverse_permitted == 1U);
    commit_output(&gate, &output, 140U);

    return 0;
}

static int test_first_strike_must_be_committed_and_long_enough(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_stopped(0U);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 0U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    EXPECT_TRUE(output.normalized_brake_request == config.reverse_first_strike_request);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    commit_action(&gate,
                  MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
                  100U,
                  config.reverse_first_strike_request);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 159U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    commit_action(&gate,
                  MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
                  159U,
                  config.reverse_first_strike_request);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 160U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    commit_action(&gate,
                  MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
                  160U,
                  config.reverse_first_strike_request);

    motion = motion_stopped(220U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 220U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 220U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 259U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 260U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

static int test_reverse_to_forward_waits_for_new_stop(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    EscMotionEstimate_t motion = motion_reverse_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_REVERSE, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_COASTING_REVERSE);
    EXPECT_TRUE(output.forward_permitted == 0U);
    commit_output(&gate, &output, 20U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);

    motion = motion_stopped(240U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 279U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 280U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);

    return 0;
}

static int test_stop_or_takeover_interrupts_history(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    commit_output(&gate, &output, 10U);

    Mode2DriveGate_InvalidateAppliedHistory(&gate);
    motion = motion_stopped(100U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 160U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 160U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 180U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    commit_action(&gate,
                  MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
                  180U,
                  config.reverse_first_strike_request);
    input.propulsion_authorized = 0U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 190U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_NOT_AUTHORIZED);

    input.propulsion_authorized = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 220U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    return 0;
}

static int test_zero_target_does_not_skip_reversal_stop_or_dwell(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    input.stop_requested = 1U;
    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 10U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    commit_output(&gate, &output, 20U);

    motion = motion_stopped(100U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    commit_output(&gate, &output, 100U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    commit_output(&gate, &output, 100U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 139U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

static int test_stop_and_decel_requests_do_not_return_propulsion(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    input.decel_or_stop_requested = 1U;
    input.brake_request_valid = 1U;
    input.normalized_brake_request = 0.25f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.propulsion_permitted == 0U);
    EXPECT_TRUE(fabsf(output.normalized_brake_request - 0.25f) < 0.0001f);

    input.normalized_brake_request = 0.90f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 40U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(fabsf(output.normalized_brake_request - config.brake_request) < 0.0001f);

    input.normalized_brake_request = 0.0f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 45U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.brake_permitted == 0U);
    EXPECT_TRUE(output.normalized_brake_request == 0.0f);

    input.normalized_brake_request = NAN;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 50U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID);

    input.normalized_brake_request = -0.01f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 52U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID);

    input.normalized_brake_request = 1.01f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 54U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    input.decel_or_stop_requested = 0U;
    input.stop_requested = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 60U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.forward_permitted == 0U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    input.decel_or_stop_requested = 1U;
    motion = motion_reverse_moving();
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_COASTING_REVERSE);

    return 0;
}

static int test_forward_brake_requires_threshold_duration_and_resets(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    input.brake_request_valid = 1U;
    input.normalized_brake_request = 0.25f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(fabsf(output.normalized_brake_request - 0.25f) < 0.0001f);
    commit_output(&gate, &output, 10U);

    motion = motion_stopped(80U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    commit_output(&gate, &output, 80U);

    input.normalized_brake_request = config.reverse_first_strike_request;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 100U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 159U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 159U);

    input.normalized_brake_request = 0.25f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 170U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 170U);

    input.normalized_brake_request = config.reverse_first_strike_request;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 200U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 259U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 259U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 260U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 260U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 260U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 260U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 299U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 300U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

static int test_first_strike_requires_actual_applied_strength(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_stopped(0U);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 0U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    Mode2DriveGate_CommitAppliedActionWithEvidence(&gate,
                                                   output.action,
                                                   0U,
                                                   config.reverse_first_strike_request - 0.10f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    motion = motion_stopped(100U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN);
    commit_output(&gate, &output, 100U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 160U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 160U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    motion = motion_stopped(180U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 180U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);

    return 0;
}

static int test_interrupted_first_strike_stays_uncertain_until_forward(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_stopped(0U);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 0U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    Mode2DriveGate_CommitAppliedActionWithEvidence(&gate,
                                                   output.action,
                                                   0U,
                                                   config.reverse_first_strike_request);
    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 30U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    motion = motion_stopped(90U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 90U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN);
    commit_output(&gate, &output, 90U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 130U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 150U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 150U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    motion = motion_stopped(170U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 170U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);

    return 0;
}

static int test_brake_release_does_not_create_second_brake_without_forward(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    Mode2DriveGate_CommitAppliedActionWithEvidence(&gate,
                                                   output.action,
                                                   10U,
                                                   output.normalized_brake_request);
    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 20U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 30U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.brake_permitted == 0U);

    return 0;
}

static int test_non_reverse_brake_entries_require_active_forward_history(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);
    commit_action(&gate,
                  MODE2_DRIVE_ACTION_BRAKE,
                  10U,
                  config.reverse_first_strike_request);
    commit_action(&gate,
                  MODE2_DRIVE_ACTION_BRAKE,
                  70U,
                  config.reverse_first_strike_request);
    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 80U, 0.0f);

    input.decel_or_stop_requested = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 90U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.brake_permitted == 0U);

    input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    input.stop_requested = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.brake_permitted == 0U);

    Mode2DriveGate_InvalidateAppliedHistory(&gate);
    input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    input.stop_requested = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 110U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);
    EXPECT_TRUE(output.brake_permitted == 0U);

    motion = motion_stopped(120U);
    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 120U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 120U);

    motion = motion_forward_moving();
    input = request(MODE2_DRIVE_TARGET_FORWARD);
    input.decel_or_stop_requested = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 130U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    EXPECT_TRUE(output.brake_permitted == 1U);

    return 0;
}

static int test_zero_brake_release_does_not_rearm_brake(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    input.decel_or_stop_requested = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    commit_output(&gate, &output, 10U);

    input.brake_request_valid = 1U;
    input.normalized_brake_request = 0.0f;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 20U);

    input.normalized_brake_request = config.brake_request;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 30U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.brake_permitted == 0U);

    return 0;
}

static int test_known_reverse_history_allows_same_direction_restart(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_stopped(100U);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);
    commit_action(&gate, MODE2_DRIVE_ACTION_REVERSE, 0U, 0.0f);
    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 80U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 120U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.reverse_permitted == 1U);

    return 0;
}

static int test_set_config_change_invalidates_reverse_permission(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_stopped(100U);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);
    commit_action(&gate, MODE2_DRIVE_ACTION_REVERSE, 20U, 0.0f);
    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 60U, 0.0f);

    EXPECT_TRUE(Mode2DriveGate_SetConfig(&gate, &config) ==
                MODE2_DRIVE_REASON_OK);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    config.neutral_dwell_ms = 80U;
    EXPECT_TRUE(Mode2DriveGate_SetConfig(&gate, &config) ==
                MODE2_DRIVE_REASON_OK);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);

    config.calibration_valid = 0U;
    EXPECT_TRUE(Mode2DriveGate_SetConfig(&gate, &config) ==
                MODE2_DRIVE_REASON_CONFIG_INVALID);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 180U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_CONFIG_INVALID);

    return 0;
}

static int test_startup_reverse_requires_known_forward_history(void)
{
    EscMotionEstimatorConfig_t motion_config = estimator_config();
    Mode2DriveGateConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion;
    Mode2DriveGateOutput_t output;

    config.reverse_first_strike_min_ms = 20U;
    config.neutral_dwell_ms = 40U;
    config.reversal_timeout_ms = 1000U;
    EscMotionEstimator_Init(&estimator, &motion_config);
    Mode2DriveGate_Init(&gate, &config);

    observe_estimator_zero(&estimator, 1U, 0U);
    observe_estimator_zero(&estimator, 2U, 20U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 20U);
    EXPECT_TRUE(motion.stopped == 1U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 40U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 40U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             40U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    observe_estimator_zero(&estimator, 3U, 60U);
    observe_estimator_zero(&estimator, 4U, 80U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 80U);
    EXPECT_TRUE(motion.stopped == 1U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);

    return 0;
}

static int test_coupled_forward_brake_stop_neutral_reverse(void)
{
    EscMotionEstimatorConfig_t motion_config = estimator_config();
    Mode2DriveGateConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    uint32_t ticks[] = {20U, 40U, 60U, 80U, 100U, 140U};
    uint32_t erpms[] = {120U, 80U, 20U, 0U, 0U, 0U};
    uint32_t i;
    uint8_t reverse_seen = 0U;

    config.reverse_first_strike_min_ms = 60U;
    config.neutral_dwell_ms = 40U;
    config.reversal_timeout_ms = 1000U;
    EscMotionEstimator_Init(&estimator, &motion_config);
    Mode2DriveGate_Init(&gate, &config);

    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             0U);

    for (i = 0U; i < (uint32_t)(sizeof(ticks) / sizeof(ticks[0])); i++)
    {
        EscFe32Sample_t sample;
        EscMotionEstimate_t motion;
        Mode2DriveGateOutput_t output;

        memset(&sample, 0, sizeof(sample));
        sample.sample_id = i + 1U;
        sample.received_tick_ms = ticks[i];
        sample.rpm_valid = 1U;
        sample.erpm_candidate = erpms[i];
        (void)EscMotionEstimator_ObserveSample(&estimator, &sample, ticks[i]);

        motion = EscMotionEstimator_GetEstimate(&estimator, ticks[i]);
        output = Mode2DriveGate_Evaluate(&gate, &input, &motion, ticks[i]);
        commit_output(&gate, &output, ticks[i]);
        EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                                 Mode2DriveGate_ToMotionAction(output.action),
                                                 ticks[i]);
        if (output.action == MODE2_DRIVE_ACTION_REVERSE)
        {
            reverse_seen = 1U;
        }
    }

    EXPECT_TRUE(reverse_seen == 1U);
    return 0;
}

static int test_coupled_first_strike_neutral_requires_new_stop(void)
{
    EscMotionEstimatorConfig_t motion_config = estimator_config();
    Mode2DriveGateConfig_t config = valid_config();
    EscMotionEstimator_t estimator;
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion;
    Mode2DriveGateOutput_t output;

    config.reverse_first_strike_min_ms = 40U;
    config.neutral_dwell_ms = 40U;
    config.reversal_timeout_ms = 1000U;
    EscMotionEstimator_Init(&estimator, &motion_config);
    Mode2DriveGate_Init(&gate, &config);

    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             ESC_MOTION_APPLIED_ACTION_FORWARD,
                                             0U);

    observe_estimator_zero(&estimator, 1U, 20U);
    observe_estimator_zero(&estimator, 2U, 40U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 40U);
    EXPECT_TRUE(motion.stopped == 1U);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 40U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    commit_output(&gate, &output, 40U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             40U);

    observe_estimator_zero(&estimator, 3U, 60U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 60U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 60U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    commit_output(&gate, &output, 60U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             60U);

    observe_estimator_zero(&estimator, 4U, 80U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 80U);
    EXPECT_TRUE(motion.stopped == 1U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE);
    commit_output(&gate, &output, 80U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             80U);

    motion = EscMotionEstimator_GetEstimate(&estimator, 100U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL);
    commit_output(&gate, &output, 100U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             100U);

    observe_estimator_zero(&estimator, 5U, 80U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 120U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 120U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_STOP);
    commit_output(&gate, &output, 120U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             120U);

    motion = EscMotionEstimator_GetEstimate(&estimator, 140U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_STOP);
    commit_output(&gate, &output, 140U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             140U);

    observe_estimator_zero(&estimator, 6U, 160U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 160U);
    EXPECT_TRUE(motion.stopped == 0U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 160U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_STOP);
    commit_output(&gate, &output, 160U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             160U);

    observe_estimator_zero(&estimator, 7U, 180U);
    motion = EscMotionEstimator_GetEstimate(&estimator, 180U);
    EXPECT_TRUE(motion.stopped == 1U);
    EXPECT_TRUE(motion.stop_established_tick_ms == 180U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 180U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL);
    commit_output(&gate, &output, 180U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             180U);

    motion = EscMotionEstimator_GetEstimate(&estimator, 219U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 219U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL);
    commit_output(&gate, &output, 219U);
    EscMotionEstimator_CommitAppliedActionAt(&estimator,
                                             Mode2DriveGate_ToMotionAction(output.action),
                                             219U);

    motion = EscMotionEstimator_GetEstimate(&estimator, 220U);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 220U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    EXPECT_TRUE(output.reverse_permitted == 1U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_REVERSE_PERMITTED);

    return 0;
}

static int test_reversal_timeout_fault_and_neutral_recovery(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    EscMotionEstimate_t motion = motion_forward_moving();
    Mode2DriveGateOutput_t output;

    config.reversal_timeout_ms = 100U;
    Mode2DriveGate_Init(&gate, &config);
    commit_action(&gate, MODE2_DRIVE_ACTION_FORWARD, 0U, 0.0f);

    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 110U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_BRAKE);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 111U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_REVERSAL_TIMEOUT);
    EXPECT_TRUE(output.fault_latched == 1U);

    motion = motion_stopped(160U);
    input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 160U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.fault_latched == 0U);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 200U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);

    return 0;
}

int main(void)
{
    if (test_config_authorization_and_feedback_fail_closed() != 0)
    {
        return 1;
    }
    if (test_uncommitted_requests_do_not_advance_history() != 0)
    {
        return 1;
    }
    if (test_forward_to_reverse_legal_sequence() != 0)
    {
        return 1;
    }
    if (test_first_strike_must_be_committed_and_long_enough() != 0)
    {
        return 1;
    }
    if (test_reverse_to_forward_waits_for_new_stop() != 0)
    {
        return 1;
    }
    if (test_stop_or_takeover_interrupts_history() != 0)
    {
        return 1;
    }
    if (test_zero_target_does_not_skip_reversal_stop_or_dwell() != 0)
    {
        return 1;
    }
    if (test_stop_and_decel_requests_do_not_return_propulsion() != 0)
    {
        return 1;
    }
    if (test_forward_brake_requires_threshold_duration_and_resets() != 0)
    {
        return 1;
    }
    if (test_first_strike_requires_actual_applied_strength() != 0)
    {
        return 1;
    }
    if (test_interrupted_first_strike_stays_uncertain_until_forward() != 0)
    {
        return 1;
    }
    if (test_brake_release_does_not_create_second_brake_without_forward() != 0)
    {
        return 1;
    }
    if (test_non_reverse_brake_entries_require_active_forward_history() != 0)
    {
        return 1;
    }
    if (test_zero_brake_release_does_not_rearm_brake() != 0)
    {
        return 1;
    }
    if (test_known_reverse_history_allows_same_direction_restart() != 0)
    {
        return 1;
    }
    if (test_set_config_change_invalidates_reverse_permission() != 0)
    {
        return 1;
    }
    if (test_startup_reverse_requires_known_forward_history() != 0)
    {
        return 1;
    }
    if (test_coupled_forward_brake_stop_neutral_reverse() != 0)
    {
        return 1;
    }
    if (test_coupled_first_strike_neutral_requires_new_stop() != 0)
    {
        return 1;
    }
    if (test_reversal_timeout_fault_and_neutral_recovery() != 0)
    {
        return 1;
    }
    return 0;
}
