#include "mode2_drive_gate.h"

#include <math.h>
#include <string.h>

#define MODE2_DRIVE_TICK_HALF_RANGE 0x80000000UL
#define MODE2_DRIVE_PWM_MIN_US 500U
#define MODE2_DRIVE_PWM_MAX_US 2500U

static uint8_t mode2_float_request_is_valid(float value)
{
    return (isfinite(value) && value > 0.0f && value <= 1.0f) ? 1U : 0U;
}

static uint8_t mode2_float_command_is_valid(float value)
{
    return (isfinite(value) && value >= 0.0f && value <= 1.0f) ? 1U : 0U;
}

static uint8_t mode2_duration_is_valid(uint32_t value)
{
    return (value != 0U && value < MODE2_DRIVE_TICK_HALF_RANGE) ? 1U : 0U;
}

static uint8_t mode2_tick_at_or_after(uint32_t tick, uint32_t reference)
{
    return ((uint32_t)(tick - reference) < MODE2_DRIVE_TICK_HALF_RANGE) ? 1U : 0U;
}

static uint8_t mode2_tick_is_after(uint32_t tick, uint32_t reference)
{
    const uint32_t delta = tick - reference;

    return (delta != 0U && delta < MODE2_DRIVE_TICK_HALF_RANGE) ? 1U : 0U;
}

static uint8_t mode2_pwm_in_range(uint16_t value)
{
    return (value >= MODE2_DRIVE_PWM_MIN_US &&
            value <= MODE2_DRIVE_PWM_MAX_US) ? 1U : 0U;
}

static uint8_t mode2_pwm_config_is_valid(const Mode2DriveGateConfig_t *config)
{
    const uint32_t center = config->center_pwm_us;
    const uint32_t fwd_to_rev_full = config->fwd_to_rev_brake_full_pwm_us;
    const uint32_t rev_to_fwd_full = config->rev_to_fwd_brake_full_pwm_us;
    const uint32_t fwd_to_rev_delta = config->fwd_to_rev_qualify_delta_us;
    const uint32_t rev_to_fwd_delta = config->rev_to_fwd_qualify_delta_us;

    if (mode2_pwm_in_range(config->center_pwm_us) == 0U ||
        mode2_pwm_in_range(config->fwd_to_rev_brake_full_pwm_us) == 0U ||
        mode2_pwm_in_range(config->rev_to_fwd_brake_full_pwm_us) == 0U ||
        fwd_to_rev_delta == 0U ||
        rev_to_fwd_delta == 0U)
    {
        return 0U;
    }

    if (fwd_to_rev_full >= center ||
        rev_to_fwd_full <= center)
    {
        return 0U;
    }

    if (center - fwd_to_rev_full < fwd_to_rev_delta ||
        rev_to_fwd_full - center < rev_to_fwd_delta)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t mode2_action_is_fwd_to_rev_brake(Mode2DriveAction_t action)
{
    return (action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE) ? 1U : 0U;
}

static uint8_t mode2_action_is_rev_to_fwd_brake(Mode2DriveAction_t action)
{
    return (action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE) ? 1U : 0U;
}

static uint8_t mode2_action_is_brake(Mode2DriveAction_t action)
{
    return (mode2_action_is_fwd_to_rev_brake(action) != 0U ||
            mode2_action_is_rev_to_fwd_brake(action) != 0U) ? 1U : 0U;
}

static Mode2DriveTargetDirection_t mode2_opposite_direction(Mode2DriveTargetDirection_t direction)
{
    if (direction == MODE2_DRIVE_TARGET_FORWARD)
    {
        return MODE2_DRIVE_TARGET_REVERSE;
    }
    if (direction == MODE2_DRIVE_TARGET_REVERSE)
    {
        return MODE2_DRIVE_TARGET_FORWARD;
    }
    return MODE2_DRIVE_TARGET_NEUTRAL;
}

static Mode2DriveAction_t mode2_propulsion_action(Mode2DriveTargetDirection_t direction)
{
    return (direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_ACTION_FORWARD : MODE2_DRIVE_ACTION_REVERSE;
}

static Mode2DriveAction_t mode2_brake_action_for_target(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE :
        MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE;
}

static Mode2DriveState_t mode2_tracking_state(Mode2DriveTargetDirection_t direction)
{
    if (direction == MODE2_DRIVE_TARGET_FORWARD)
    {
        return MODE2_DRIVE_STATE_FORWARD_TRACKING;
    }
    if (direction == MODE2_DRIVE_TARGET_REVERSE)
    {
        return MODE2_DRIVE_STATE_REVERSE_TRACKING;
    }
    return MODE2_DRIVE_STATE_UNKNOWN_SAFE;
}

static Mode2DriveState_t mode2_brake_state_for_target(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_STATE_REVERSE_BRAKE_CONTINUOUS :
        MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS;
}

static Mode2DriveState_t mode2_armed_state(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_STATE_FORWARD_ARMED : MODE2_DRIVE_STATE_REVERSE_ARMED;
}

static Mode2DriveState_t mode2_maybe_state(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_STATE_FORWARD_MAYBE_ARMED : MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED;
}

static Mode2DrivePhase_t mode2_brake_phase_for_target(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_PHASE_BRAKING_TO_FORWARD :
        MODE2_DRIVE_PHASE_BRAKING_TO_REVERSE;
}

static Mode2DrivePhase_t mode2_neutral_phase_for_target(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_FORWARD :
        MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_REVERSE;
}

static Mode2DriveReason_t mode2_braking_reason_for_target(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_REASON_BRAKING_REVERSE :
        MODE2_DRIVE_REASON_BRAKING_FORWARD;
}

static Mode2DriveReason_t mode2_permitted_reason_for_target(Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        MODE2_DRIVE_REASON_FORWARD_PERMITTED :
        MODE2_DRIVE_REASON_REVERSE_PERMITTED;
}

static float mode2_required_brake_request(const Mode2DriveGate_t *gate,
                                          Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        gate->config.rev_to_fwd_brake_request :
        gate->config.fwd_to_rev_brake_request;
}

static uint32_t mode2_required_brake_min_ms(const Mode2DriveGate_t *gate,
                                            Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        gate->config.rev_to_fwd_brake_min_ms :
        gate->config.fwd_to_rev_brake_min_ms;
}

static uint16_t mode2_required_brake_delta_us(const Mode2DriveGate_t *gate,
                                              Mode2DriveTargetDirection_t target_direction)
{
    return (target_direction == MODE2_DRIVE_TARGET_FORWARD) ?
        gate->config.rev_to_fwd_qualify_delta_us :
        gate->config.fwd_to_rev_qualify_delta_us;
}

static void mode2_clear_brake_sequence(Mode2DriveGate_t *gate)
{
    gate->brake_threshold_active = 0U;
    gate->brake_sufficient = 0U;
    gate->brake_target_direction = MODE2_DRIVE_TARGET_NEUTRAL;
    gate->brake_session_start_ms = 0U;
    gate->brake_threshold_start_ms = 0U;
    gate->expect_neutral_to_arm = 0U;
    gate->expected_arm_direction = MODE2_DRIVE_TARGET_NEUTRAL;
    gate->expected_arm_stop_tick_ms = 0U;
}

static void mode2_clear_armed(Mode2DriveGate_t *gate)
{
    gate->neutral_active = 0U;
    gate->neutral_start_ms = 0U;
    gate->armed_stop_valid = 0U;
    gate->armed_stop_tick_ms = 0U;
}

static void mode2_clear_forward_recovery(Mode2DriveGate_t *gate)
{
    gate->forward_recovery_active = 0U;
    gate->forward_recovery_start_ms = 0U;
}

static void mode2_clear_runtime_history(Mode2DriveGate_t *gate)
{
    mode2_clear_brake_sequence(gate);
    mode2_clear_armed(gate);
    mode2_clear_forward_recovery(gate);
    gate->state = MODE2_DRIVE_STATE_UNKNOWN_SAFE;
    gate->last_applied_action = MODE2_DRIVE_ACTION_NEUTRAL;
    gate->has_applied_action = 0U;
    gate->last_propulsion_direction = MODE2_DRIVE_TARGET_NEUTRAL;
}

uint8_t Mode2DriveGate_ConfigIsValid(const Mode2DriveGateConfig_t *config,
                                     Mode2DriveReason_t *reason)
{
    Mode2DriveReason_t local_reason = MODE2_DRIVE_REASON_OK;
    uint8_t valid = 1U;

    if (config == NULL)
    {
        if (reason != NULL)
        {
            *reason = MODE2_DRIVE_REASON_INVALID_ARGUMENT;
        }
        return 0U;
    }

    if (mode2_float_request_is_valid(config->fwd_to_rev_brake_request) == 0U ||
        mode2_float_request_is_valid(config->rev_to_fwd_brake_request) == 0U ||
        mode2_duration_is_valid(config->fwd_to_rev_brake_min_ms) == 0U ||
        mode2_duration_is_valid(config->rev_to_fwd_brake_min_ms) == 0U ||
        mode2_duration_is_valid(config->neutral_dwell_ms) == 0U ||
        mode2_pwm_config_is_valid(config) == 0U)
    {
        valid = 0U;
    }

    if (valid == 0U)
    {
        local_reason = MODE2_DRIVE_REASON_CONFIG_INVALID;
    }

    if (reason != NULL)
    {
        *reason = local_reason;
    }
    return valid;
}

static uint8_t mode2_config_equals(const Mode2DriveGateConfig_t *left,
                                   const Mode2DriveGateConfig_t *right)
{
    return (left->fwd_to_rev_brake_request == right->fwd_to_rev_brake_request &&
            left->rev_to_fwd_brake_request == right->rev_to_fwd_brake_request &&
            left->fwd_to_rev_brake_min_ms == right->fwd_to_rev_brake_min_ms &&
            left->rev_to_fwd_brake_min_ms == right->rev_to_fwd_brake_min_ms &&
            left->neutral_dwell_ms == right->neutral_dwell_ms &&
            left->center_pwm_us == right->center_pwm_us &&
            left->fwd_to_rev_brake_full_pwm_us == right->fwd_to_rev_brake_full_pwm_us &&
            left->rev_to_fwd_brake_full_pwm_us == right->rev_to_fwd_brake_full_pwm_us &&
            left->fwd_to_rev_qualify_delta_us == right->fwd_to_rev_qualify_delta_us &&
            left->rev_to_fwd_qualify_delta_us == right->rev_to_fwd_qualify_delta_us) ? 1U : 0U;
}

void Mode2DriveGate_Init(Mode2DriveGate_t *gate,
                         const Mode2DriveGateConfig_t *config)
{
    if (gate == NULL)
    {
        return;
    }

    memset(gate, 0, sizeof(*gate));
    gate->state = MODE2_DRIVE_STATE_UNKNOWN_SAFE;
    gate->last_applied_action = MODE2_DRIVE_ACTION_NEUTRAL;
    gate->last_propulsion_direction = MODE2_DRIVE_TARGET_NEUTRAL;
    gate->config_reason = MODE2_DRIVE_REASON_CONFIG_INVALID;
    if (config != NULL)
    {
        (void)Mode2DriveGate_SetConfig(gate, config);
    }
}

Mode2DriveReason_t Mode2DriveGate_SetConfig(Mode2DriveGate_t *gate,
                                            const Mode2DriveGateConfig_t *config)
{
    Mode2DriveReason_t reason = MODE2_DRIVE_REASON_OK;
    uint8_t new_config_valid;
    uint8_t same_valid_config;

    if (gate == NULL || config == NULL)
    {
        return MODE2_DRIVE_REASON_INVALID_ARGUMENT;
    }

    new_config_valid = Mode2DriveGate_ConfigIsValid(config, &reason);
    same_valid_config = (gate->config_valid != 0U &&
                         new_config_valid != 0U &&
                         mode2_config_equals(&gate->config, config) != 0U) ? 1U : 0U;

    gate->config = *config;
    gate->config_valid = new_config_valid;
    gate->config_reason = reason;
    if (same_valid_config == 0U)
    {
        mode2_clear_runtime_history(gate);
    }
    return reason;
}

static Mode2DriveGateOutput_t mode2_output(Mode2DriveAction_t action,
                                           Mode2DriveReason_t reason,
                                           Mode2DrivePhase_t phase,
                                           const Mode2DriveGate_t *gate,
                                           float brake_request)
{
    Mode2DriveGateOutput_t output;

    memset(&output, 0, sizeof(output));
    output.action = action;
    output.reason = reason;
    output.phase = phase;
    output.state = (gate != NULL) ? gate->state : MODE2_DRIVE_STATE_UNKNOWN_SAFE;

    if (action == MODE2_DRIVE_ACTION_FORWARD)
    {
        output.propulsion_permitted = 1U;
        output.forward_permitted = 1U;
    }
    else if (action == MODE2_DRIVE_ACTION_REVERSE)
    {
        output.propulsion_permitted = 1U;
        output.reverse_permitted = 1U;
    }
    else if (mode2_action_is_brake(action) != 0U)
    {
        output.brake_permitted = 1U;
        output.normalized_brake_request = brake_request;
    }

    if (gate != NULL)
    {
        output.opposite_direction_armed =
            (gate->state == MODE2_DRIVE_STATE_FORWARD_ARMED ||
             gate->state == MODE2_DRIVE_STATE_REVERSE_ARMED) ? 1U : 0U;
        output.uncertain =
            (gate->state == MODE2_DRIVE_STATE_FORWARD_MAYBE_ARMED ||
             gate->state == MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED ||
             gate->state == MODE2_DRIVE_STATE_UNKNOWN_SAFE ||
             gate->state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING) ? 1U : 0U;
    }

    return output;
}

static Mode2DriveGateOutput_t mode2_neutral_output(Mode2DriveReason_t reason,
                                                   Mode2DrivePhase_t phase,
                                                   const Mode2DriveGate_t *gate)
{
    return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL, reason, phase, gate, 0.0f);
}

static uint8_t mode2_motion_available(const EscMotionEstimate_t *motion)
{
    if (motion == NULL)
    {
        return 0U;
    }

    return (motion->config_valid != 0U &&
            motion->has_sample != 0U &&
            motion->sample_fresh != 0U &&
            motion->magnitude_valid != 0U) ? 1U : 0U;
}

static Mode2DriveMotionObservation_t mode2_observation_from_estimate(
    const EscMotionEstimate_t *motion)
{
    Mode2DriveMotionObservation_t observation;

    memset(&observation, 0, sizeof(observation));
    if (motion != NULL)
    {
        observation.available = mode2_motion_available(motion);
        observation.stopped = motion->stopped;
        observation.stop_established_valid = motion->stopped;
        observation.moving_observed = motion->moving_observed;
        observation.stop_established_tick_ms = motion->stop_established_tick_ms;
        observation.sample_tick_ms = motion->last_sample_tick_ms;
    }
    return observation;
}

static Mode2DriveGateOutput_t mode2_evaluate_known_target(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    Mode2DriveTargetDirection_t target_direction,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms);

static uint8_t mode2_get_tracking_brake_request(const Mode2DriveGate_t *gate,
                                                const Mode2DriveGateInput_t *input,
                                                float *request)
{
    float local_request;
    const Mode2DriveTargetDirection_t target =
        (gate->state == MODE2_DRIVE_STATE_FORWARD_TRACKING ||
         gate->state == MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS) ?
            MODE2_DRIVE_TARGET_REVERSE : MODE2_DRIVE_TARGET_FORWARD;

    if (request == NULL)
    {
        return 0U;
    }

    local_request = mode2_required_brake_request(gate, target);
    if (input != NULL && input->brake_request_valid != 0U)
    {
        if (mode2_float_command_is_valid(input->normalized_brake_request) == 0U)
        {
            *request = 0.0f;
            return 0U;
        }
        local_request = input->normalized_brake_request;
    }

    if (local_request > mode2_required_brake_request(gate, target))
    {
        local_request = mode2_required_brake_request(gate, target);
    }

    *request = local_request;
    return 1U;
}

static uint8_t mode2_observation_has_fresh_stop(const Mode2DriveGate_t *gate,
                                                const Mode2DriveMotionObservation_t *observation)
{
    if (observation->stopped == 0U ||
        observation->stop_established_valid == 0U)
    {
        return 0U;
    }

    return mode2_tick_at_or_after(observation->stop_established_tick_ms,
                                  gate->brake_session_start_ms);
}

static uint8_t mode2_armed_neutral_dwell_satisfied(
    const Mode2DriveGate_t *gate,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms)
{
    if (gate->neutral_active == 0U ||
        gate->armed_stop_valid == 0U ||
        observation->stopped == 0U ||
        observation->stop_established_valid == 0U)
    {
        return 0U;
    }

    if (mode2_tick_at_or_after(observation->stop_established_tick_ms,
                               gate->armed_stop_tick_ms) == 0U)
    {
        return 0U;
    }

    if ((uint32_t)(now_tick_ms - gate->neutral_start_ms) <
        gate->config.neutral_dwell_ms)
    {
        return 0U;
    }

    if ((uint32_t)(now_tick_ms - observation->stop_established_tick_ms) <
        gate->config.neutral_dwell_ms)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t mode2_forward_recovery_confirmed(
    const Mode2DriveGate_t *gate,
    const Mode2DriveMotionObservation_t *observation)
{
    return (gate->forward_recovery_active != 0U &&
            observation->moving_observed != 0U &&
            mode2_tick_is_after(observation->sample_tick_ms,
                                gate->forward_recovery_start_ms) != 0U) ? 1U : 0U;
}

static Mode2DriveGateOutput_t mode2_definite_brake_output(
    Mode2DriveGate_t *gate,
    Mode2DriveTargetDirection_t target_direction)
{
    const float request = mode2_required_brake_request(gate, target_direction);

    return mode2_output(mode2_brake_action_for_target(target_direction),
                        mode2_braking_reason_for_target(target_direction),
                        mode2_brake_phase_for_target(target_direction),
                        gate,
                        request);
}

static Mode2DriveGateOutput_t mode2_tracking_brake_output(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    Mode2DriveTargetDirection_t target_direction)
{
    float request = 0.0f;

    if (mode2_get_tracking_brake_request(gate, input, &request) == 0U)
    {
        return mode2_neutral_output(MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    if (request <= 0.0f)
    {
        return mode2_neutral_output(MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    return mode2_output(mode2_brake_action_for_target(target_direction),
                        mode2_braking_reason_for_target(target_direction),
                        MODE2_DRIVE_PHASE_IDLE,
                        gate,
                        request);
}

static Mode2DriveGateOutput_t mode2_evaluate_armed_target(
    Mode2DriveGate_t *gate,
    Mode2DriveTargetDirection_t target_direction,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms)
{
    if (observation->stopped == 0U)
    {
        gate->state = mode2_maybe_state(target_direction);
        mode2_clear_armed(gate);
        return mode2_neutral_output(MODE2_DRIVE_REASON_ARMING_UNCERTAIN,
                                    mode2_neutral_phase_for_target(target_direction),
                                    gate);
    }

    if (mode2_armed_neutral_dwell_satisfied(gate, observation, now_tick_ms) == 0U)
    {
        return mode2_neutral_output(MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                                    mode2_neutral_phase_for_target(target_direction),
                                    gate);
    }

    return mode2_output(mode2_propulsion_action(target_direction),
                        mode2_permitted_reason_for_target(target_direction),
                        MODE2_DRIVE_PHASE_IDLE,
                        gate,
                        0.0f);
}

static Mode2DriveGateOutput_t mode2_evaluate_reversal_target(
    Mode2DriveGate_t *gate,
    Mode2DriveTargetDirection_t target_direction,
    const Mode2DriveMotionObservation_t *observation)
{
    if (gate->brake_sufficient != 0U &&
        mode2_observation_has_fresh_stop(gate, observation) != 0U)
    {
        gate->expect_neutral_to_arm = 1U;
        gate->expected_arm_direction = target_direction;
        gate->expected_arm_stop_tick_ms = observation->stop_established_tick_ms;
        return mode2_neutral_output(MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                                    mode2_neutral_phase_for_target(target_direction),
                                    gate);
    }

    gate->expect_neutral_to_arm = 0U;
    return mode2_definite_brake_output(gate, target_direction);
}

static Mode2DriveGateOutput_t mode2_evaluate_forward_recovery(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms)
{
    if (mode2_forward_recovery_confirmed(gate, observation) != 0U)
    {
        gate->state = MODE2_DRIVE_STATE_FORWARD_TRACKING;
        mode2_clear_forward_recovery(gate);
        return mode2_evaluate_known_target(gate,
                                           input,
                                           MODE2_DRIVE_TARGET_FORWARD,
                                           observation,
                                           now_tick_ms);
    }

    gate->state = MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING;
    return mode2_output(MODE2_DRIVE_ACTION_FORWARD,
                        MODE2_DRIVE_REASON_FORWARD_RECOVERY,
                        MODE2_DRIVE_PHASE_IDLE,
                        gate,
                        0.0f);
}

static Mode2DriveGateOutput_t mode2_evaluate_known_target(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    Mode2DriveTargetDirection_t target_direction,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms)
{
    const Mode2DriveTargetDirection_t opposite_direction =
        mode2_opposite_direction(target_direction);

    if (gate->state == mode2_armed_state(target_direction))
    {
        return mode2_evaluate_armed_target(gate,
                                           target_direction,
                                           observation,
                                           now_tick_ms);
    }

    /* A completed stop may arm the ESC for the opposite direction, while the
     * next command resumes the direction from before braking. That direction
     * is still unambiguous, but it observes the same neutral dwell first. */
    if (gate->state == mode2_armed_state(opposite_direction))
    {
        return mode2_evaluate_armed_target(gate,
                                           target_direction,
                                           observation,
                                           now_tick_ms);
    }

    if (gate->state == mode2_maybe_state(target_direction))
    {
        return mode2_neutral_output(MODE2_DRIVE_REASON_ARMING_UNCERTAIN,
                                    mode2_neutral_phase_for_target(target_direction),
                                    gate);
    }

    if (gate->state == mode2_tracking_state(target_direction))
    {
        if (input->decel_or_stop_requested != 0U)
        {
            return mode2_tracking_brake_output(gate,
                                               input,
                                               opposite_direction);
        }

        return mode2_output(mode2_propulsion_action(target_direction),
                            mode2_permitted_reason_for_target(target_direction),
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (gate->state == mode2_brake_state_for_target(opposite_direction))
    {
        if (input->decel_or_stop_requested != 0U)
        {
            return mode2_tracking_brake_output(gate,
                                               input,
                                               opposite_direction);
        }

        return mode2_output(mode2_propulsion_action(target_direction),
                            mode2_permitted_reason_for_target(target_direction),
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (gate->state == mode2_tracking_state(opposite_direction) ||
        gate->state == mode2_brake_state_for_target(target_direction))
    {
        return mode2_evaluate_reversal_target(gate,
                                              target_direction,
                                              observation);
    }

    return mode2_neutral_output(MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
                                MODE2_DRIVE_PHASE_IDLE,
                                gate);
}

static Mode2DriveGateOutput_t mode2_evaluate_neutral_target(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    const Mode2DriveMotionObservation_t *observation)
{
    if (gate->state == MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS ||
        gate->state == MODE2_DRIVE_STATE_REVERSE_BRAKE_CONTINUOUS)
    {
        const Mode2DriveTargetDirection_t target_direction =
            gate->brake_target_direction;

        if (gate->brake_sufficient != 0U &&
            mode2_observation_has_fresh_stop(gate, observation) != 0U)
        {
            gate->expect_neutral_to_arm = 1U;
            gate->expected_arm_direction = target_direction;
            gate->expected_arm_stop_tick_ms = observation->stop_established_tick_ms;
            return mode2_neutral_output(MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                                        mode2_neutral_phase_for_target(target_direction),
                                        gate);
        }

        if (input->decel_or_stop_requested != 0U || input->stop_requested != 0U)
        {
            return mode2_definite_brake_output(gate, target_direction);
        }

        return mode2_neutral_output(MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    if ((input->decel_or_stop_requested != 0U || input->stop_requested != 0U) &&
        gate->state == MODE2_DRIVE_STATE_FORWARD_TRACKING)
    {
        return mode2_tracking_brake_output(gate,
                                           input,
                                           MODE2_DRIVE_TARGET_REVERSE);
    }

    if ((input->decel_or_stop_requested != 0U || input->stop_requested != 0U) &&
        gate->state == MODE2_DRIVE_STATE_REVERSE_TRACKING)
    {
        return mode2_tracking_brake_output(gate,
                                           input,
                                           MODE2_DRIVE_TARGET_FORWARD);
    }

    return mode2_neutral_output(MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                                MODE2_DRIVE_PHASE_IDLE,
                                gate);
}

Mode2DriveGateOutput_t Mode2DriveGate_EvaluateWithObservation(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms)
{
    if (gate == NULL || input == NULL || observation == NULL)
    {
        return mode2_neutral_output(MODE2_DRIVE_REASON_INVALID_ARGUMENT,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    if (gate->config_valid == 0U)
    {
        return mode2_neutral_output(gate->config_reason,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    if (input->propulsion_authorized == 0U)
    {
        mode2_clear_runtime_history(gate);
        return mode2_neutral_output(MODE2_DRIVE_REASON_NOT_AUTHORIZED,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    if (observation->available == 0U)
    {
        mode2_clear_runtime_history(gate);
        return mode2_neutral_output(MODE2_DRIVE_REASON_MOTION_UNAVAILABLE,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    if (input->stop_requested != 0U ||
        input->target_direction == MODE2_DRIVE_TARGET_NEUTRAL)
    {
        return mode2_evaluate_neutral_target(gate, input, observation);
    }

    if (gate->state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING &&
        input->target_direction == MODE2_DRIVE_TARGET_FORWARD)
    {
        return mode2_evaluate_forward_recovery(gate,
                                               input,
                                               observation,
                                               now_tick_ms);
    }

    if (gate->state == MODE2_DRIVE_STATE_UNKNOWN_SAFE &&
        input->target_direction == MODE2_DRIVE_TARGET_FORWARD)
    {
        if (input->forward_recovery_authorized == 0U)
        {
            return mode2_neutral_output(MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
                                        MODE2_DRIVE_PHASE_IDLE,
                                        gate);
        }
        return mode2_evaluate_forward_recovery(gate,
                                               input,
                                               observation,
                                               now_tick_ms);
    }

    if (input->target_direction == MODE2_DRIVE_TARGET_REVERSE &&
        gate->state == MODE2_DRIVE_STATE_UNKNOWN_SAFE)
    {
        return mode2_neutral_output(MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
                                    MODE2_DRIVE_PHASE_IDLE,
                                    gate);
    }

    return mode2_evaluate_known_target(gate,
                                       input,
                                       input->target_direction,
                                       observation,
                                       now_tick_ms);
}

Mode2DriveGateOutput_t Mode2DriveGate_Evaluate(Mode2DriveGate_t *gate,
                                               const Mode2DriveGateInput_t *input,
                                               const EscMotionEstimate_t *motion,
                                               uint32_t now_tick_ms)
{
    const Mode2DriveMotionObservation_t observation =
        mode2_observation_from_estimate(motion);

    return Mode2DriveGate_EvaluateWithObservation(gate,
                                                  input,
                                                  &observation,
                                                  now_tick_ms);
}

static void mode2_reset_brake_threshold(Mode2DriveGate_t *gate)
{
    gate->brake_threshold_active = 0U;
    gate->brake_threshold_start_ms = 0U;
    gate->brake_sufficient = 0U;
}

static uint16_t mode2_applied_pwm_delta_us(const Mode2DriveGate_t *gate,
                                           Mode2DriveAction_t action,
                                           uint16_t final_applied_pwm_us)
{
    const uint16_t center = gate->config.center_pwm_us;

    if (mode2_action_is_fwd_to_rev_brake(action) != 0U)
    {
        return (final_applied_pwm_us < center) ?
            (uint16_t)(center - final_applied_pwm_us) : 0U;
    }

    if (mode2_action_is_rev_to_fwd_brake(action) != 0U)
    {
        return (final_applied_pwm_us > center) ?
            (uint16_t)(final_applied_pwm_us - center) : 0U;
    }

    return 0U;
}

static void mode2_commit_brake(Mode2DriveGate_t *gate,
                               Mode2DriveTargetDirection_t target_direction,
                               uint32_t now_tick_ms,
                               uint16_t applied_delta_us)
{
    const Mode2DriveState_t brake_state =
        mode2_brake_state_for_target(target_direction);
    const uint16_t required_delta_us =
        mode2_required_brake_delta_us(gate, target_direction);
    const uint32_t required_ms =
        mode2_required_brake_min_ms(gate, target_direction);

    if (gate->state != brake_state)
    {
        const Mode2DriveTargetDirection_t source_direction =
            mode2_opposite_direction(target_direction);

        if (gate->state != mode2_tracking_state(source_direction))
        {
            gate->state = mode2_maybe_state(target_direction);
            mode2_clear_brake_sequence(gate);
            return;
        }

        mode2_clear_armed(gate);
        mode2_clear_brake_sequence(gate);
        gate->state = brake_state;
        gate->brake_target_direction = target_direction;
        gate->brake_session_start_ms = now_tick_ms;
    }

    gate->expect_neutral_to_arm = 0U;
    if (applied_delta_us < required_delta_us)
    {
        mode2_reset_brake_threshold(gate);
        return;
    }

    if (gate->brake_threshold_active == 0U)
    {
        gate->brake_threshold_active = 1U;
        gate->brake_threshold_start_ms = now_tick_ms;
    }

    if ((uint32_t)(now_tick_ms - gate->brake_threshold_start_ms) >= required_ms)
    {
        gate->brake_sufficient = 1U;
    }
}

void Mode2DriveGate_CommitAppliedActionWithPwmEvidence(
    Mode2DriveGate_t *gate,
    Mode2DriveAction_t action,
    uint32_t now_tick_ms,
    uint16_t final_applied_pwm_us)
{
    uint16_t applied_delta_us;

    if (gate == NULL)
    {
        return;
    }

    gate->has_applied_action = 1U;
    gate->last_applied_action = action;

    if (action != MODE2_DRIVE_ACTION_NEUTRAL)
    {
        gate->neutral_active = 0U;
        gate->neutral_start_ms = 0U;
    }

    switch (action)
    {
    case MODE2_DRIVE_ACTION_FORWARD:
        if (gate->state == MODE2_DRIVE_STATE_UNKNOWN_SAFE ||
            gate->state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING)
        {
            gate->state = MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING;
            if (gate->forward_recovery_active == 0U)
            {
                gate->forward_recovery_active = 1U;
                gate->forward_recovery_start_ms = now_tick_ms;
            }
        }
        else
        {
            gate->state = MODE2_DRIVE_STATE_FORWARD_TRACKING;
            mode2_clear_forward_recovery(gate);
        }
        gate->last_propulsion_direction = MODE2_DRIVE_TARGET_FORWARD;
        mode2_clear_brake_sequence(gate);
        mode2_clear_armed(gate);
        break;
    case MODE2_DRIVE_ACTION_REVERSE:
        gate->state = MODE2_DRIVE_STATE_REVERSE_TRACKING;
        gate->last_propulsion_direction = MODE2_DRIVE_TARGET_REVERSE;
        mode2_clear_forward_recovery(gate);
        mode2_clear_brake_sequence(gate);
        mode2_clear_armed(gate);
        break;
    case MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE:
        applied_delta_us =
            mode2_applied_pwm_delta_us(gate, action, final_applied_pwm_us);
        if (applied_delta_us != 0U)
        {
            mode2_commit_brake(gate,
                               MODE2_DRIVE_TARGET_REVERSE,
                               now_tick_ms,
                               applied_delta_us);
        }
        else
        {
            mode2_reset_brake_threshold(gate);
        }
        break;
    case MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE:
        applied_delta_us =
            mode2_applied_pwm_delta_us(gate, action, final_applied_pwm_us);
        if (applied_delta_us != 0U)
        {
            mode2_commit_brake(gate,
                               MODE2_DRIVE_TARGET_FORWARD,
                               now_tick_ms,
                               applied_delta_us);
        }
        else
        {
            mode2_reset_brake_threshold(gate);
        }
        break;
    case MODE2_DRIVE_ACTION_NEUTRAL:
    default:
        if (gate->expect_neutral_to_arm != 0U &&
            gate->brake_sufficient != 0U)
        {
            gate->state = mode2_armed_state(gate->expected_arm_direction);
            gate->armed_stop_valid = 1U;
            gate->armed_stop_tick_ms = gate->expected_arm_stop_tick_ms;
            gate->neutral_active = 1U;
            gate->neutral_start_ms = now_tick_ms;
            mode2_clear_brake_sequence(gate);
        }
        else if (gate->state == MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS ||
                 gate->state == MODE2_DRIVE_STATE_REVERSE_BRAKE_CONTINUOUS)
        {
            gate->state = mode2_maybe_state(gate->brake_target_direction);
            gate->neutral_active = 1U;
            gate->neutral_start_ms = now_tick_ms;
            mode2_clear_brake_sequence(gate);
        }
        else
        {
            if (gate->neutral_active == 0U)
            {
                gate->neutral_active = 1U;
                gate->neutral_start_ms = now_tick_ms;
            }
            gate->expect_neutral_to_arm = 0U;
        }
        break;
    }
}

void Mode2DriveGate_InvalidateAppliedHistory(Mode2DriveGate_t *gate)
{
    if (gate == NULL)
    {
        return;
    }

    mode2_clear_runtime_history(gate);
}

EscMotionAppliedAction_t Mode2DriveGate_ToMotionAction(Mode2DriveAction_t action)
{
    switch (action)
    {
    case MODE2_DRIVE_ACTION_FORWARD:
        return ESC_MOTION_APPLIED_ACTION_FORWARD;
    case MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE:
    case MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE:
        return ESC_MOTION_APPLIED_ACTION_BRAKE;
    case MODE2_DRIVE_ACTION_REVERSE:
        return ESC_MOTION_APPLIED_ACTION_REVERSE;
    case MODE2_DRIVE_ACTION_NEUTRAL:
    default:
        return ESC_MOTION_APPLIED_ACTION_NEUTRAL;
    }
}
