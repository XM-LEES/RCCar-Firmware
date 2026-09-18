#include "mode2_drive_gate.h"

#include <math.h>
#include <string.h>

#define MODE2_DRIVE_TICK_HALF_RANGE 0x80000000UL

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

static void mode2_clear_sequence(Mode2DriveGate_t *gate)
{
    gate->reverse_unlocked_by_forward_brake = 0U;
    gate->forward_brake_session_active = 0U;
    gate->forward_brake_sufficient_active = 0U;
    gate->forward_brake_sufficient_start_ms = 0U;
    gate->first_strike_active = 0U;
    gate->first_strike_sufficient = 0U;
    gate->first_strike_start_ms = 0U;
    gate->neutral_active = 0U;
    gate->neutral_start_ms = 0U;
    gate->expect_forward_brake_unlock = 0U;
    gate->expect_first_strike = 0U;
    gate->transition_active = 0U;
    gate->transition = MODE2_DRIVE_TRANSITION_NONE;
    gate->transition_start_ms = 0U;
}

static void mode2_clear_uncertain_first_strike(Mode2DriveGate_t *gate)
{
    gate->first_strike_uncertain = 0U;
    gate->first_strike_uncertain_since_ms = 0U;
}

static void mode2_mark_uncertain_first_strike(Mode2DriveGate_t *gate,
                                              uint32_t now_tick_ms)
{
    gate->first_strike_active = 0U;
    gate->first_strike_sufficient = 0U;
    gate->first_strike_uncertain = 1U;
    gate->first_strike_uncertain_since_ms = now_tick_ms;
    gate->esc_state_unknown = 1U;
}

static void mode2_mark_uncertain_if_first_strike_interrupted(Mode2DriveGate_t *gate,
                                                             uint32_t now_tick_ms)
{
    if ((gate->first_strike_active != 0U &&
         gate->first_strike_sufficient == 0U) ||
        (gate->forward_brake_session_active != 0U &&
         gate->reverse_unlocked_by_forward_brake == 0U))
    {
        mode2_mark_uncertain_first_strike(gate, now_tick_ms);
    }
}

static void mode2_clear_all_history(Mode2DriveGate_t *gate)
{
    mode2_clear_sequence(gate);
    mode2_clear_uncertain_first_strike(gate);
    gate->last_applied_action = MODE2_DRIVE_ACTION_NEUTRAL;
    gate->has_applied_action = 0U;
    gate->last_propulsion_direction = MODE2_DRIVE_TARGET_NEUTRAL;
    gate->forward_propulsion_available = 0U;
    gate->esc_state_unknown = 1U;
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

    if (config->calibration_valid == 0U ||
        config->brake_calibration_valid == 0U ||
        config->first_strike_calibration_valid == 0U ||
        config->neutral_dwell_valid == 0U ||
        config->reversal_timeout_valid == 0U)
    {
        valid = 0U;
    }
    else if (mode2_float_request_is_valid(config->brake_request) == 0U ||
             mode2_float_request_is_valid(config->reverse_first_strike_request) == 0U ||
             mode2_duration_is_valid(config->reverse_first_strike_min_ms) == 0U ||
             mode2_duration_is_valid(config->neutral_dwell_ms) == 0U ||
             mode2_duration_is_valid(config->reversal_timeout_ms) == 0U)
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
    return (left->calibration_valid == right->calibration_valid &&
            left->brake_calibration_valid == right->brake_calibration_valid &&
            left->first_strike_calibration_valid == right->first_strike_calibration_valid &&
            left->neutral_dwell_valid == right->neutral_dwell_valid &&
            left->reversal_timeout_valid == right->reversal_timeout_valid &&
            left->brake_request == right->brake_request &&
            left->reverse_first_strike_request == right->reverse_first_strike_request &&
            left->reverse_first_strike_min_ms == right->reverse_first_strike_min_ms &&
            left->neutral_dwell_ms == right->neutral_dwell_ms &&
            left->reversal_timeout_ms == right->reversal_timeout_ms) ? 1U : 0U;
}

void Mode2DriveGate_Init(Mode2DriveGate_t *gate,
                         const Mode2DriveGateConfig_t *config)
{
    if (gate == NULL)
    {
        return;
    }

    memset(gate, 0, sizeof(*gate));
    gate->last_applied_action = MODE2_DRIVE_ACTION_NEUTRAL;
    gate->last_propulsion_direction = MODE2_DRIVE_TARGET_NEUTRAL;
    gate->transition = MODE2_DRIVE_TRANSITION_NONE;
    gate->config_reason = MODE2_DRIVE_REASON_CONFIG_INVALID;
    gate->esc_state_unknown = 1U;
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
        mode2_clear_all_history(gate);
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
    output.fault_latched = (gate != NULL && gate->fault_latched != 0U) ? 1U : 0U;

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
    else if (action == MODE2_DRIVE_ACTION_BRAKE ||
             action == MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE)
    {
        output.brake_permitted = 1U;
        output.normalized_brake_request = brake_request;
    }

    return output;
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

static uint8_t mode2_get_brake_request(const Mode2DriveGate_t *gate,
                                       const Mode2DriveGateInput_t *input,
                                       float *request)
{
    float local_request = gate->config.brake_request;

    if (request == NULL)
    {
        return 0U;
    }

    if (input != NULL && input->brake_request_valid != 0U)
    {
        if (mode2_float_command_is_valid(input->normalized_brake_request) == 0U)
        {
            *request = 0.0f;
            return 0U;
        }
        local_request = input->normalized_brake_request;
    }

    if (local_request > gate->config.brake_request)
    {
        local_request = gate->config.brake_request;
    }
    *request = local_request;
    return 1U;
}

static Mode2DriveGateOutput_t mode2_brake_or_neutral_output(Mode2DriveGate_t *gate,
                                                            const Mode2DriveGateInput_t *input,
                                                            Mode2DriveReason_t reason,
                                                            Mode2DrivePhase_t phase)
{
    float brake_request = 0.0f;

    if (gate->esc_state_unknown != 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
                            phase,
                            gate,
                            0.0f);
    }

    if (gate->first_strike_uncertain != 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN,
                            phase,
                            gate,
                            0.0f);
    }

    if (gate->forward_brake_session_active == 0U &&
        (gate->last_propulsion_direction != MODE2_DRIVE_TARGET_FORWARD ||
         gate->forward_propulsion_available == 0U))
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                            phase,
                            gate,
                            0.0f);
    }

    if (mode2_get_brake_request(gate, input, &brake_request) == 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID,
                            phase,
                            gate,
                            0.0f);
    }

    if (brake_request <= 0.0f)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            reason,
                            phase,
                            gate,
                            0.0f);
    }

    return mode2_output(MODE2_DRIVE_ACTION_BRAKE,
                        reason,
                        phase,
                        gate,
                        brake_request);
}

static uint8_t mode2_applied_brake_is_positive(float applied_normalized_brake)
{
    return (isfinite(applied_normalized_brake) &&
            applied_normalized_brake > 0.0f) ? 1U : 0U;
}

static uint8_t mode2_applied_brake_meets(float applied_normalized_brake,
                                         float required_normalized_brake)
{
    return (isfinite(applied_normalized_brake) &&
            applied_normalized_brake >= required_normalized_brake) ? 1U : 0U;
}

static void mode2_reset_forward_brake_threshold_timer(Mode2DriveGate_t *gate)
{
    gate->forward_brake_sufficient_active = 0U;
    gate->forward_brake_sufficient_start_ms = 0U;
}

static void mode2_update_forward_brake_evidence(Mode2DriveGate_t *gate,
                                                uint32_t now_tick_ms,
                                                float applied_normalized_brake)
{
    if (mode2_applied_brake_is_positive(applied_normalized_brake) == 0U)
    {
        mode2_reset_forward_brake_threshold_timer(gate);
        return;
    }

    gate->forward_brake_session_active = 1U;
    gate->forward_propulsion_available = 0U;

    if (mode2_applied_brake_meets(applied_normalized_brake,
                                  gate->config.reverse_first_strike_request) == 0U)
    {
        mode2_reset_forward_brake_threshold_timer(gate);
        return;
    }

    if (gate->forward_brake_sufficient_active == 0U)
    {
        gate->forward_brake_sufficient_active = 1U;
        gate->forward_brake_sufficient_start_ms = now_tick_ms;
    }

    if ((uint32_t)(now_tick_ms - gate->forward_brake_sufficient_start_ms) >=
        gate->config.reverse_first_strike_min_ms)
    {
        gate->reverse_unlocked_by_forward_brake = 1U;
    }
}

static void mode2_start_transition(Mode2DriveGate_t *gate,
                                   Mode2DriveTransition_t transition,
                                   uint32_t now_tick_ms)
{
    if (gate->transition_active == 0U || gate->transition != transition)
    {
        gate->transition_active = 1U;
        gate->transition = transition;
        gate->transition_start_ms = now_tick_ms;
    }
}

static uint8_t mode2_transition_timed_out(Mode2DriveGate_t *gate,
                                          uint32_t now_tick_ms)
{
    if (gate->transition_active == 0U)
    {
        return 0U;
    }

    if ((uint32_t)(now_tick_ms - gate->transition_start_ms) >
        gate->config.reversal_timeout_ms)
    {
        gate->fault_latched = 1U;
        gate->fault_reason = MODE2_DRIVE_REASON_REVERSAL_TIMEOUT;
        mode2_clear_all_history(gate);
        return 1U;
    }

    return 0U;
}

static uint8_t mode2_neutral_dwell_satisfied(const Mode2DriveGate_t *gate,
                                             const EscMotionEstimate_t *motion,
                                             uint32_t now_tick_ms)
{
    if (gate->neutral_active == 0U || motion->stopped == 0U)
    {
        return 0U;
    }

    if ((uint32_t)(now_tick_ms - gate->neutral_start_ms) <
        gate->config.neutral_dwell_ms)
    {
        return 0U;
    }

    if ((uint32_t)(now_tick_ms - motion->stop_established_tick_ms) <
        gate->config.neutral_dwell_ms)
    {
        return 0U;
    }

    return 1U;
}

static void mode2_clear_fault_after_stopped_neutral(Mode2DriveGate_t *gate,
                                                   const Mode2DriveGateInput_t *input,
                                                   const EscMotionEstimate_t *motion)
{
    if (gate->fault_latched == 0U ||
        input->target_direction != MODE2_DRIVE_TARGET_NEUTRAL ||
        motion->stopped == 0U)
    {
        return;
    }

    gate->fault_latched = 0U;
    gate->fault_reason = MODE2_DRIVE_REASON_OK;
    mode2_clear_all_history(gate);
}

static Mode2DriveGateOutput_t mode2_wait_for_reverse_neutral_dwell(Mode2DriveGate_t *gate,
                                                                   const EscMotionEstimate_t *motion,
                                                                   uint32_t now_tick_ms)
{
    gate->expect_forward_brake_unlock = 0U;
    gate->expect_first_strike = 0U;
    gate->transition_active = 1U;
    gate->transition = MODE2_DRIVE_TRANSITION_TO_REVERSE;

    if (mode2_neutral_dwell_satisfied(gate, motion, now_tick_ms) == 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                            MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_REVERSE,
                            gate,
                            0.0f);
    }

    return mode2_output(MODE2_DRIVE_ACTION_REVERSE,
                        MODE2_DRIVE_REASON_REVERSE_PERMITTED,
                        MODE2_DRIVE_PHASE_IDLE,
                        gate,
                        0.0f);
}

static Mode2DriveGateOutput_t mode2_evaluate_reverse_target(Mode2DriveGate_t *gate,
                                                            const Mode2DriveGateInput_t *input,
                                                            const EscMotionEstimate_t *motion,
                                                            uint32_t now_tick_ms)
{
    if (gate->first_strike_uncertain != 0U &&
        gate->last_propulsion_direction != MODE2_DRIVE_TARGET_REVERSE)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN,
                            MODE2_DRIVE_PHASE_FIRST_STRIKE,
                            gate,
                            0.0f);
    }

    if (gate->esc_state_unknown != 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    mode2_start_transition(gate, MODE2_DRIVE_TRANSITION_TO_REVERSE, now_tick_ms);
    if (mode2_transition_timed_out(gate, now_tick_ms) != 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_REVERSAL_TIMEOUT,
                            MODE2_DRIVE_PHASE_FAULT,
                            gate,
                            0.0f);
    }

    if (gate->first_strike_active != 0U &&
        gate->first_strike_sufficient == 0U)
    {
        gate->expect_first_strike = 1U;
        gate->expect_forward_brake_unlock = 0U;
        return mode2_output(MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
                            MODE2_DRIVE_REASON_FIRST_STRIKE_HOLD,
                            MODE2_DRIVE_PHASE_FIRST_STRIKE,
                            gate,
                            gate->config.reverse_first_strike_request);
    }

    if (motion->stopped == 0U)
    {
        if (motion->direction_valid != 0U &&
            motion->direction == ESC_MOTION_DIRECTION_FORWARD)
        {
            if (gate->forward_brake_session_active == 0U &&
                (gate->last_propulsion_direction != MODE2_DRIVE_TARGET_FORWARD ||
                 gate->forward_propulsion_available == 0U))
            {
                gate->expect_forward_brake_unlock = 0U;
                gate->expect_first_strike = 0U;
                return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                                    MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                                    MODE2_DRIVE_PHASE_BRAKING_TO_REVERSE,
                                    gate,
                                    0.0f);
            }

            gate->expect_forward_brake_unlock = 1U;
            gate->expect_first_strike = 0U;
            return mode2_brake_or_neutral_output(gate,
                                                 input,
                                                 MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                                 MODE2_DRIVE_PHASE_BRAKING_TO_REVERSE);
        }

        if (motion->direction_valid != 0U &&
            motion->direction == ESC_MOTION_DIRECTION_REVERSE)
        {
            return mode2_output(MODE2_DRIVE_ACTION_REVERSE,
                                MODE2_DRIVE_REASON_REVERSE_PERMITTED,
                                MODE2_DRIVE_PHASE_IDLE,
                                gate,
                                0.0f);
        }

        gate->expect_forward_brake_unlock = 0U;
        gate->expect_first_strike = 0U;
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                            MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_REVERSE,
                            gate,
                            0.0f);
    }

    if (gate->forward_brake_session_active != 0U &&
        gate->reverse_unlocked_by_forward_brake == 0U)
    {
        gate->expect_forward_brake_unlock = 1U;
        gate->expect_first_strike = 0U;
        return mode2_brake_or_neutral_output(gate,
                                             input,
                                             MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                             MODE2_DRIVE_PHASE_BRAKING_TO_REVERSE);
    }

    if (gate->last_propulsion_direction == MODE2_DRIVE_TARGET_REVERSE)
    {
        return mode2_output(MODE2_DRIVE_ACTION_REVERSE,
                            MODE2_DRIVE_REASON_REVERSE_PERMITTED,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (gate->reverse_unlocked_by_forward_brake != 0U ||
        gate->first_strike_sufficient != 0U)
    {
        return mode2_wait_for_reverse_neutral_dwell(gate, motion, now_tick_ms);
    }

    gate->expect_forward_brake_unlock = 0U;
    gate->expect_first_strike = 1U;
    return mode2_output(MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
                        MODE2_DRIVE_REASON_FIRST_STRIKE_REQUIRED,
                        MODE2_DRIVE_PHASE_FIRST_STRIKE,
                        gate,
                        gate->config.reverse_first_strike_request);
}

static Mode2DriveGateOutput_t mode2_evaluate_forward_target(Mode2DriveGate_t *gate,
                                                            const Mode2DriveGateInput_t *input,
                                                            const EscMotionEstimate_t *motion,
                                                            uint32_t now_tick_ms)
{
    gate->expect_forward_brake_unlock = 0U;
    gate->expect_first_strike = 0U;

    if (input->decel_or_stop_requested != 0U &&
        motion->stopped == 0U &&
        motion->direction_valid != 0U &&
        motion->direction == ESC_MOTION_DIRECTION_FORWARD)
    {
        return mode2_brake_or_neutral_output(gate,
                                             input,
                                             MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                             MODE2_DRIVE_PHASE_IDLE);
    }

    if (motion->stopped != 0U)
    {
        if (gate->last_propulsion_direction == MODE2_DRIVE_TARGET_REVERSE &&
            mode2_neutral_dwell_satisfied(gate, motion, now_tick_ms) == 0U)
        {
            mode2_start_transition(gate, MODE2_DRIVE_TRANSITION_TO_FORWARD,
                                   now_tick_ms);
            if (mode2_transition_timed_out(gate, now_tick_ms) != 0U)
            {
                return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                                    MODE2_DRIVE_REASON_REVERSAL_TIMEOUT,
                                    MODE2_DRIVE_PHASE_FAULT,
                                    gate,
                                    0.0f);
            }
            return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                                MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                                MODE2_DRIVE_PHASE_WAIT_STOP_TO_FORWARD,
                                gate,
                                0.0f);
        }

        return mode2_output(MODE2_DRIVE_ACTION_FORWARD,
                            MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (motion->direction_valid == 0U)
    {
        mode2_start_transition(gate, MODE2_DRIVE_TRANSITION_TO_FORWARD,
                               now_tick_ms);
        if (mode2_transition_timed_out(gate, now_tick_ms) != 0U)
        {
            return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                                MODE2_DRIVE_REASON_REVERSAL_TIMEOUT,
                                MODE2_DRIVE_PHASE_FAULT,
                                gate,
                                0.0f);
        }
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                            MODE2_DRIVE_PHASE_WAIT_STOP_TO_FORWARD,
                            gate,
                            0.0f);
    }

    if (motion->direction == ESC_MOTION_DIRECTION_REVERSE)
    {
        mode2_start_transition(gate, MODE2_DRIVE_TRANSITION_TO_FORWARD,
                               now_tick_ms);
        if (mode2_transition_timed_out(gate, now_tick_ms) != 0U)
        {
            return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                                MODE2_DRIVE_REASON_REVERSAL_TIMEOUT,
                                MODE2_DRIVE_PHASE_FAULT,
                                gate,
                                0.0f);
        }
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_COASTING_REVERSE,
                            MODE2_DRIVE_PHASE_WAIT_STOP_TO_FORWARD,
                            gate,
                            0.0f);
    }

    return mode2_output(MODE2_DRIVE_ACTION_FORWARD,
                        MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                        MODE2_DRIVE_PHASE_IDLE,
                        gate,
                        0.0f);
}

static Mode2DriveGateOutput_t mode2_evaluate_neutral_target(Mode2DriveGate_t *gate,
                                                            const Mode2DriveGateInput_t *input,
                                                            const EscMotionEstimate_t *motion)
{
    gate->expect_forward_brake_unlock = 0U;
    gate->expect_first_strike = 0U;
    gate->transition_active = 0U;
    gate->transition = MODE2_DRIVE_TRANSITION_NONE;

    if ((input->decel_or_stop_requested != 0U || input->stop_requested != 0U) &&
        motion->stopped == 0U &&
        motion->direction_valid != 0U &&
        motion->direction == ESC_MOTION_DIRECTION_FORWARD)
    {
        return mode2_brake_or_neutral_output(gate,
                                             input,
                                             MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                             MODE2_DRIVE_PHASE_IDLE);
    }

    if (motion->stopped == 0U &&
        motion->direction_valid != 0U &&
        motion->direction == ESC_MOTION_DIRECTION_REVERSE)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_COASTING_REVERSE,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                        MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                        MODE2_DRIVE_PHASE_IDLE,
                        gate,
                        0.0f);
}

Mode2DriveGateOutput_t Mode2DriveGate_Evaluate(Mode2DriveGate_t *gate,
                                               const Mode2DriveGateInput_t *input,
                                               const EscMotionEstimate_t *motion,
                                               uint32_t now_tick_ms)
{
    if (gate == NULL || input == NULL || motion == NULL)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_INVALID_ARGUMENT,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (gate->config_valid == 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            gate->config_reason,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (input->propulsion_authorized == 0U)
    {
        mode2_clear_all_history(gate);
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_NOT_AUTHORIZED,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    if (mode2_motion_available(motion) == 0U)
    {
        mode2_clear_all_history(gate);
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_MOTION_UNAVAILABLE,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    mode2_clear_fault_after_stopped_neutral(gate, input, motion);
    if (gate->fault_latched != 0U)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            gate->fault_reason,
                            MODE2_DRIVE_PHASE_FAULT,
                            gate,
                            0.0f);
    }

    if (input->stop_requested != 0U)
    {
        return mode2_evaluate_neutral_target(gate, input, motion);
    }

    if (input->decel_or_stop_requested != 0U &&
        input->target_direction == MODE2_DRIVE_TARGET_REVERSE &&
        motion->stopped == 0U &&
        motion->direction_valid != 0U &&
        motion->direction == ESC_MOTION_DIRECTION_REVERSE)
    {
        return mode2_output(MODE2_DRIVE_ACTION_NEUTRAL,
                            MODE2_DRIVE_REASON_COASTING_REVERSE,
                            MODE2_DRIVE_PHASE_IDLE,
                            gate,
                            0.0f);
    }

    switch (input->target_direction)
    {
    case MODE2_DRIVE_TARGET_FORWARD:
        return mode2_evaluate_forward_target(gate, input, motion, now_tick_ms);
    case MODE2_DRIVE_TARGET_REVERSE:
        return mode2_evaluate_reverse_target(gate, input, motion, now_tick_ms);
    case MODE2_DRIVE_TARGET_NEUTRAL:
    default:
        return mode2_evaluate_neutral_target(gate, input, motion);
    }
}

void Mode2DriveGate_CommitAppliedActionWithEvidence(Mode2DriveGate_t *gate,
                                                    Mode2DriveAction_t action,
                                                    uint32_t now_tick_ms,
                                                    float applied_normalized_brake)
{
    if (gate == NULL)
    {
        return;
    }

    gate->has_applied_action = 1U;
    gate->last_applied_action = action;

    if (action != MODE2_DRIVE_ACTION_NEUTRAL)
    {
        gate->neutral_active = 0U;
    }

    switch (action)
    {
    case MODE2_DRIVE_ACTION_FORWARD:
        gate->last_propulsion_direction = MODE2_DRIVE_TARGET_FORWARD;
        gate->forward_propulsion_available = 1U;
        gate->esc_state_unknown = 0U;
        mode2_clear_sequence(gate);
        mode2_clear_uncertain_first_strike(gate);
        gate->fault_latched = 0U;
        gate->fault_reason = MODE2_DRIVE_REASON_OK;
        break;
    case MODE2_DRIVE_ACTION_REVERSE:
        gate->last_propulsion_direction = MODE2_DRIVE_TARGET_REVERSE;
        gate->forward_propulsion_available = 0U;
        mode2_clear_sequence(gate);
        if (gate->esc_state_unknown == 0U)
        {
            mode2_clear_uncertain_first_strike(gate);
        }
        gate->fault_latched = 0U;
        gate->fault_reason = MODE2_DRIVE_REASON_OK;
        break;
    case MODE2_DRIVE_ACTION_BRAKE:
        if (mode2_applied_brake_is_positive(applied_normalized_brake) != 0U &&
            (gate->forward_brake_session_active != 0U ||
             (gate->last_propulsion_direction == MODE2_DRIVE_TARGET_FORWARD &&
              gate->forward_propulsion_available != 0U)))
        {
            mode2_update_forward_brake_evidence(gate,
                                                now_tick_ms,
                                                applied_normalized_brake);
        }
        else
        {
            mode2_reset_forward_brake_threshold_timer(gate);
        }
        gate->expect_forward_brake_unlock = 0U;
        break;
    case MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE:
        if (gate->expect_first_strike != 0U &&
            gate->first_strike_uncertain == 0U &&
            gate->esc_state_unknown == 0U &&
            mode2_applied_brake_meets(applied_normalized_brake,
                                      gate->config.reverse_first_strike_request) != 0U)
        {
            if (gate->first_strike_active == 0U)
            {
                gate->first_strike_active = 1U;
                gate->first_strike_start_ms = now_tick_ms;
            }
            if ((uint32_t)(now_tick_ms - gate->first_strike_start_ms) >=
                gate->config.reverse_first_strike_min_ms)
            {
                gate->first_strike_sufficient = 1U;
                gate->first_strike_active = 0U;
            }
        }
        else if (mode2_applied_brake_is_positive(applied_normalized_brake) != 0U)
        {
            mode2_mark_uncertain_first_strike(gate, now_tick_ms);
        }
        gate->expect_first_strike = 0U;
        gate->forward_propulsion_available = 0U;
        break;
    case MODE2_DRIVE_ACTION_NEUTRAL:
    default:
        mode2_mark_uncertain_if_first_strike_interrupted(gate, now_tick_ms);
        gate->forward_brake_session_active = 0U;
        mode2_reset_forward_brake_threshold_timer(gate);
        if (gate->neutral_active == 0U)
        {
            gate->neutral_active = 1U;
            gate->neutral_start_ms = now_tick_ms;
        }
        gate->expect_forward_brake_unlock = 0U;
        gate->expect_first_strike = 0U;
        break;
    }
}

void Mode2DriveGate_InvalidateAppliedHistory(Mode2DriveGate_t *gate)
{
    if (gate == NULL)
    {
        return;
    }

    mode2_clear_all_history(gate);
    gate->fault_latched = 0U;
    gate->fault_reason = MODE2_DRIVE_REASON_OK;
}

EscMotionAppliedAction_t Mode2DriveGate_ToMotionAction(Mode2DriveAction_t action)
{
    switch (action)
    {
    case MODE2_DRIVE_ACTION_FORWARD:
        return ESC_MOTION_APPLIED_ACTION_FORWARD;
    case MODE2_DRIVE_ACTION_BRAKE:
        return ESC_MOTION_APPLIED_ACTION_BRAKE;
    case MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE:
        return ESC_MOTION_APPLIED_ACTION_REVERSE_FIRST_STRIKE;
    case MODE2_DRIVE_ACTION_REVERSE:
        return ESC_MOTION_APPLIED_ACTION_REVERSE;
    case MODE2_DRIVE_ACTION_NEUTRAL:
    default:
        return ESC_MOTION_APPLIED_ACTION_NEUTRAL;
    }
}
