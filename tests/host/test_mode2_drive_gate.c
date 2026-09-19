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

#define TEST_CENTER_PWM_US 1500U
#define TEST_FWD_TO_REV_FULL_PWM_US 1000U
#define TEST_REV_TO_FWD_FULL_PWM_US 2000U
#define TEST_QUALIFY_DELTA_US 100U

static Mode2DriveGateConfig_t valid_config(void)
{
    Mode2DriveGateConfig_t config;

    memset(&config, 0, sizeof(config));
    config.fwd_to_rev_brake_request = 0.75f;
    config.rev_to_fwd_brake_request = 0.65f;
    config.fwd_to_rev_brake_min_ms = 60U;
    config.rev_to_fwd_brake_min_ms = 50U;
    config.neutral_dwell_ms = 40U;
    config.center_pwm_us = TEST_CENTER_PWM_US;
    config.fwd_to_rev_brake_full_pwm_us = TEST_FWD_TO_REV_FULL_PWM_US;
    config.rev_to_fwd_brake_full_pwm_us = TEST_REV_TO_FWD_FULL_PWM_US;
    config.fwd_to_rev_qualify_delta_us = TEST_QUALIFY_DELTA_US;
    config.rev_to_fwd_qualify_delta_us = TEST_QUALIFY_DELTA_US;
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

static Mode2DriveMotionObservation_t moving_observation_at(uint32_t sample_tick_ms)
{
    Mode2DriveMotionObservation_t observation;

    memset(&observation, 0, sizeof(observation));
    observation.available = 1U;
    observation.moving_observed = 1U;
    observation.sample_tick_ms = sample_tick_ms;
    return observation;
}

static Mode2DriveMotionObservation_t moving_observation(void)
{
    return moving_observation_at(1000U);
}

static Mode2DriveMotionObservation_t stopped_observation(uint32_t stop_tick_ms)
{
    Mode2DriveMotionObservation_t observation;

    memset(&observation, 0, sizeof(observation));
    observation.available = 1U;
    observation.stopped = 1U;
    observation.stop_established_valid = 1U;
    observation.stop_established_tick_ms = stop_tick_ms;
    observation.sample_tick_ms = stop_tick_ms;
    return observation;
}

static uint16_t pwm_for_action_delta(Mode2DriveAction_t action,
                                     uint16_t delta_us)
{
    if (action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE)
    {
        return (uint16_t)(TEST_CENTER_PWM_US - delta_us);
    }

    if (action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE)
    {
        return (uint16_t)(TEST_CENTER_PWM_US + delta_us);
    }

    return TEST_CENTER_PWM_US;
}

static void commit_action_pwm(Mode2DriveGate_t *gate,
                              Mode2DriveAction_t action,
                              uint32_t tick_ms,
                              uint16_t final_applied_pwm_us)
{
    Mode2DriveGate_CommitAppliedActionWithPwmEvidence(gate,
                                                      action,
                                                      tick_ms,
                                                      final_applied_pwm_us);
}

static void commit_action(Mode2DriveGate_t *gate,
                          Mode2DriveAction_t action,
                          uint32_t tick_ms)
{
    commit_action_pwm(gate,
                      action,
                      tick_ms,
                      pwm_for_action_delta(action, 0U));
}

static void establish_known_tracking(Mode2DriveGate_t *gate,
                                     Mode2DriveTargetDirection_t direction,
                                     uint32_t tick_ms)
{
    commit_action(gate,
                  (direction == MODE2_DRIVE_TARGET_FORWARD) ?
                      MODE2_DRIVE_ACTION_FORWARD : MODE2_DRIVE_ACTION_REVERSE,
                  tick_ms);
    gate->state = (direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_STATE_FORWARD_TRACKING :
        MODE2_DRIVE_STATE_REVERSE_TRACKING;
}

static void commit_output(Mode2DriveGate_t *gate,
                          const Mode2DriveGateOutput_t *output,
                          uint32_t tick_ms)
{
    Mode2DriveGate_CommitAppliedActionWithPwmEvidence(
        gate,
        output->action,
        tick_ms,
        pwm_for_action_delta(output->action, TEST_QUALIFY_DELTA_US));
}

static Mode2DriveGateOutput_t eval(Mode2DriveGate_t *gate,
                                   const Mode2DriveGateInput_t *input,
                                   Mode2DriveMotionObservation_t observation,
                                   uint32_t tick_ms)
{
    return Mode2DriveGate_EvaluateWithObservation(gate,
                                                  input,
                                                  &observation,
                                                  tick_ms);
}

static int test_config_is_derived_from_values(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveReason_t reason;

    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) != 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_OK);

    config = valid_config();
    config.fwd_to_rev_brake_request = 0.0f;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);
    EXPECT_TRUE(reason == MODE2_DRIVE_REASON_CONFIG_INVALID);

    config = valid_config();
    config.rev_to_fwd_brake_min_ms = 0U;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.center_pwm_us = 0U;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.fwd_to_rev_brake_full_pwm_us = TEST_CENTER_PWM_US - 50U;
    config.fwd_to_rev_qualify_delta_us = 100U;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    config = valid_config();
    config.rev_to_fwd_brake_full_pwm_us = TEST_CENTER_PWM_US + 50U;
    config.rev_to_fwd_qualify_delta_us = 100U;
    EXPECT_TRUE(Mode2DriveGate_ConfigIsValid(&config, &reason) == 0U);

    return 0;
}

static int test_authority_motion_and_unknown_safe(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveMotionObservation_t observation = stopped_observation(0U);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);

    output = eval(&gate, &input, observation, 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = eval(&gate, &input, observation, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN);

    input.forward_recovery_authorized = 1U;
    output = eval(&gate, &input, observation, 30U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_RECOVERY);
    commit_output(&gate, &output, 30U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING);

    input.forward_recovery_authorized = 0U;
    output = eval(&gate, &input, stopped_observation(40U), 40U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_RECOVERY);

    output = eval(&gate, &input, moving_observation_at(50U), 50U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_PERMITTED);
    commit_output(&gate, &output, 50U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_TRACKING);

    input.propulsion_authorized = 0U;
    output = eval(&gate, &input, moving_observation(), 60U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_NOT_AUTHORIZED);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_UNKNOWN_SAFE);

    input.propulsion_authorized = 1U;
    observation = moving_observation();
    observation.available = 0U;
    output = eval(&gate, &input, observation, 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_MOTION_UNAVAILABLE);

    return 0;
}

static int test_forward_recovery_uses_estimator_motion_threshold(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    EscMotionEstimate_t motion;
    Mode2DriveGateOutput_t output;

    memset(&motion, 0, sizeof(motion));
    motion.config_valid = 1U;
    motion.has_sample = 1U;
    motion.sample_fresh = 1U;
    motion.magnitude_valid = 1U;
    motion.stopped = 1U;
    motion.stop_established_tick_ms = 10U;
    motion.last_sample_tick_ms = 10U;

    Mode2DriveGate_Init(&gate, &config);
    input.forward_recovery_authorized = 1U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 20U);

    motion.stopped = 0U;
    motion.speed_magnitude_mps = 0.01f;
    motion.moving_observed = 0U;
    motion.last_sample_tick_ms = 30U;
    input.forward_recovery_authorized = 0U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 30U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_RECOVERY);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING);

    motion.moving_observed = 1U;
    motion.last_sample_tick_ms = 40U;
    output = Mode2DriveGate_Evaluate(&gate, &input, &motion, 40U);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_FORWARD_PERMITTED);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);

    return 0;
}

static int test_forward_to_reverse_requires_continuous_actual_brake_then_dwell(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(fabsf(output.normalized_brake_request - config.fwd_to_rev_brake_request) < 0.0001f);
    commit_output(&gate, &output, 10U);

    output = eval(&gate, &input, moving_observation(), 69U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_output(&gate, &output, 69U);

    output = eval(&gate, &input, stopped_observation(70U), 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_output(&gate, &output, 70U);

    output = eval(&gate, &input, stopped_observation(71U), 71U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL);
    commit_output(&gate, &output, 71U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_ARMED);

    output = eval(&gate, &input, stopped_observation(71U), 110U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);

    output = eval(&gate, &input, stopped_observation(71U), 111U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);
    commit_output(&gate, &output, 111U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_TRACKING);

    return 0;
}

static int test_final_pwm_delta_qualifies_brake(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_action_pwm(&gate, output.action, 10U,
                      pwm_for_action_delta(output.action, 1U));

    output = eval(&gate, &input, stopped_observation(70U), 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_action_pwm(&gate, output.action, 70U,
                      pwm_for_action_delta(output.action, 90U));

    output = eval(&gate, &input, stopped_observation(130U), 130U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_action_pwm(&gate, output.action, 130U,
                      pwm_for_action_delta(output.action, 100U));

    output = eval(&gate, &input, stopped_observation(130U), 189U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_action_pwm(&gate, output.action, 189U,
                      pwm_for_action_delta(output.action, 100U));

    output = eval(&gate, &input, stopped_observation(130U), 190U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_action_pwm(&gate, output.action, 190U,
                      pwm_for_action_delta(output.action, 100U));

    output = eval(&gate, &input, stopped_observation(130U), 191U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 191U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_ARMED);

    return 0;
}

static int test_uncommitted_requests_do_not_advance_history(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);

    output = eval(&gate, &input, stopped_observation(100U), 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_TRACKING);

    return 0;
}

static int test_underqualified_forward_brake_neutral_inhibits_reverse_no_retry(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_action_pwm(&gate,
                      output.action,
                      10U,
                      pwm_for_action_delta(output.action,
                          (uint16_t)(config.fwd_to_rev_qualify_delta_us - 10U)));

    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 20U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED);

    output = eval(&gate, &input, stopped_observation(20U), 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ARMING_UNCERTAIN);
    EXPECT_TRUE(output.reverse_permitted == 0U);
    EXPECT_TRUE(output.brake_permitted == 0U);

    return 0;
}

static int test_reverse_to_forward_is_symmetric(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_REVERSE, 0U);

    output = eval(&gate, &input, moving_observation(), 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE);
    EXPECT_TRUE(fabsf(output.normalized_brake_request - config.rev_to_fwd_brake_request) < 0.0001f);
    commit_output(&gate, &output, 20U);

    output = eval(&gate, &input, moving_observation(), 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE);
    commit_output(&gate, &output, 70U);

    output = eval(&gate, &input, stopped_observation(71U), 71U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 71U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_ARMED);

    output = eval(&gate, &input, stopped_observation(71U), 111U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    EXPECT_TRUE(output.forward_permitted == 1U);

    return 0;
}

static int test_underqualified_reverse_brake_neutral_inhibits_forward_no_retry(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_FORWARD);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_REVERSE, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE);
    commit_action_pwm(&gate,
                      output.action,
                      10U,
                      pwm_for_action_delta(output.action,
                          (uint16_t)(config.rev_to_fwd_qualify_delta_us - 10U)));
    commit_action(&gate, MODE2_DRIVE_ACTION_NEUTRAL, 20U);

    output = eval(&gate, &input, stopped_observation(20U), 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ARMING_UNCERTAIN);
    EXPECT_TRUE(output.forward_permitted == 0U);
    EXPECT_TRUE(output.brake_permitted == 0U);

    return 0;
}

static int test_old_stop_evidence_does_not_arm_reversal(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 100U);
    commit_output(&gate, &output, 100U);
    output = eval(&gate, &input, moving_observation(), 160U);
    commit_output(&gate, &output, 160U);
    EXPECT_TRUE(gate.brake_sufficient != 0U);

    output = eval(&gate, &input, stopped_observation(50U), 170U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    output = eval(&gate, &input, stopped_observation(170U), 170U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 170U);

    output = eval(&gate, &input, stopped_observation(170U), 210U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

static int test_renewed_motion_after_arming_invalidates_automatic_opposite_output(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_REVERSE);
    Mode2DriveGateOutput_t output;

    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);
    output = eval(&gate, &input, moving_observation(), 0U);
    commit_output(&gate, &output, 0U);
    output = eval(&gate, &input, moving_observation(), 60U);
    commit_output(&gate, &output, 60U);

    output = eval(&gate, &input, stopped_observation(60U), 60U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 60U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_ARMED);

    output = eval(&gate, &input, moving_observation(), 80U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reason == MODE2_DRIVE_REASON_ARMING_UNCERTAIN);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED);

    output = eval(&gate, &input, stopped_observation(100U), 140U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    EXPECT_TRUE(output.reverse_permitted == 0U);

    return 0;
}

static int test_neutral_stop_can_record_opposite_arming_without_skipping_dwell(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    Mode2DriveGateOutput_t output;

    input.stop_requested = 1U;
    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_output(&gate, &output, 10U);

    output = eval(&gate, &input, moving_observation(), 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_output(&gate, &output, 70U);

    output = eval(&gate, &input, stopped_observation(70U), 70U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    commit_output(&gate, &output, 70U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_ARMED);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    output = eval(&gate, &input, stopped_observation(70U), 109U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);

    output = eval(&gate, &input, stopped_observation(70U), 110U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

static int test_stop_then_resume_original_direction_is_permitted(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    Mode2DriveGateOutput_t output;

    input.stop_requested = 1U;
    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);

    output = eval(&gate, &input, moving_observation(), 10U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE);
    commit_output(&gate, &output, 10U);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = eval(&gate, &input, moving_observation(), 20U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);
    commit_output(&gate, &output, 20U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_TRACKING);

    return 0;
}

static int test_armed_stop_can_resume_original_direction_after_dwell(void)
{
    Mode2DriveGateConfig_t config = valid_config();
    Mode2DriveGate_t gate;
    Mode2DriveGateInput_t input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    Mode2DriveGateOutput_t output;

    input.stop_requested = 1U;
    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_FORWARD, 0U);
    output = eval(&gate, &input, moving_observation(), 10U);
    commit_output(&gate, &output, 10U);
    output = eval(&gate, &input, moving_observation(), 70U);
    commit_output(&gate, &output, 70U);
    output = eval(&gate, &input, stopped_observation(70U), 70U);
    commit_output(&gate, &output, 70U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_REVERSE_ARMED);

    input = request(MODE2_DRIVE_TARGET_FORWARD);
    output = eval(&gate, &input, stopped_observation(70U), 109U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = eval(&gate, &input, stopped_observation(70U), 110U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_FORWARD);

    input = request(MODE2_DRIVE_TARGET_NEUTRAL);
    input.stop_requested = 1U;
    Mode2DriveGate_Init(&gate, &config);
    establish_known_tracking(&gate, MODE2_DRIVE_TARGET_REVERSE, 0U);
    output = eval(&gate, &input, moving_observation(), 10U);
    commit_output(&gate, &output, 10U);
    output = eval(&gate, &input, moving_observation(), 60U);
    commit_output(&gate, &output, 60U);
    output = eval(&gate, &input, stopped_observation(61U), 61U);
    commit_output(&gate, &output, 61U);
    EXPECT_TRUE(gate.state == MODE2_DRIVE_STATE_FORWARD_ARMED);

    input = request(MODE2_DRIVE_TARGET_REVERSE);
    output = eval(&gate, &input, stopped_observation(61U), 100U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_NEUTRAL);
    output = eval(&gate, &input, stopped_observation(61U), 101U);
    EXPECT_TRUE(output.action == MODE2_DRIVE_ACTION_REVERSE);

    return 0;
}

int main(void)
{
    if (test_config_is_derived_from_values() != 0)
    {
        return 1;
    }
    if (test_authority_motion_and_unknown_safe() != 0)
    {
        return 1;
    }
    if (test_forward_recovery_uses_estimator_motion_threshold() != 0)
    {
        return 1;
    }
    if (test_forward_to_reverse_requires_continuous_actual_brake_then_dwell() != 0)
    {
        return 1;
    }
    if (test_final_pwm_delta_qualifies_brake() != 0)
    {
        return 1;
    }
    if (test_uncommitted_requests_do_not_advance_history() != 0)
    {
        return 1;
    }
    if (test_underqualified_forward_brake_neutral_inhibits_reverse_no_retry() != 0)
    {
        return 1;
    }
    if (test_reverse_to_forward_is_symmetric() != 0)
    {
        return 1;
    }
    if (test_underqualified_reverse_brake_neutral_inhibits_forward_no_retry() != 0)
    {
        return 1;
    }
    if (test_old_stop_evidence_does_not_arm_reversal() != 0)
    {
        return 1;
    }
    if (test_renewed_motion_after_arming_invalidates_automatic_opposite_output() != 0)
    {
        return 1;
    }
    if (test_neutral_stop_can_record_opposite_arming_without_skipping_dwell() != 0)
    {
        return 1;
    }
    if (test_stop_then_resume_original_direction_is_permitted() != 0)
    {
        return 1;
    }
    if (test_armed_stop_can_resume_original_direction_after_dwell() != 0)
    {
        return 1;
    }
    return 0;
}
