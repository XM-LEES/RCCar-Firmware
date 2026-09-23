#include "mode2_drive_gate.h"

#include <math.h>
#include <string.h>

#define MODE2_DRIVE_TICK_HALF_RANGE 0x80000000UL
#define MODE2_DRIVE_PWM_MIN_US 500U
#define MODE2_DRIVE_PWM_MAX_US 2500U
#define MODE2_DRIVE_SESSION_MAX \
    (ESC_TELEMETRY_METADATA_SESSION_MASK >> ESC_TELEMETRY_METADATA_SESSION_SHIFT)

static uint8_t finite_f32(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static uint8_t finite_positive(float value)
{
    return (finite_f32(value) != 0U && value > 0.0f) ? 1U : 0U;
}

static uint8_t valid_duration(uint32_t value)
{
    return (value != 0U && value < MODE2_DRIVE_TICK_HALF_RANGE) ? 1U : 0U;
}

static uint8_t valid_pwm(uint16_t value)
{
    return (value >= MODE2_DRIVE_PWM_MIN_US &&
            value <= MODE2_DRIVE_PWM_MAX_US) ? 1U : 0U;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return (uint32_t)(now_ms - then_ms);
}

static uint8_t tick_at_or_after(uint32_t tick_ms, uint32_t reference_ms)
{
    return ((uint32_t)(tick_ms - reference_ms) < MODE2_DRIVE_TICK_HALF_RANGE) ? 1U : 0U;
}

static float abs_f32(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float max_f32(float left, float right)
{
    return (left >= right) ? left : right;
}

static float clip_f32(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int32_t round_f32_to_i32(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static uint16_t clip_pwm_i32(int32_t value)
{
    if (value < (int32_t)MODE2_DRIVE_PWM_MIN_US)
    {
        return MODE2_DRIVE_PWM_MIN_US;
    }
    if (value > (int32_t)MODE2_DRIVE_PWM_MAX_US)
    {
        return MODE2_DRIVE_PWM_MAX_US;
    }
    return (uint16_t)value;
}

static uint16_t pwm_from_offset(const Mode2DriveGate_t *gate, float offset_us)
{
    return clip_pwm_i32((int32_t)gate->config.center_pwm_us +
                        round_f32_to_i32(offset_us));
}

static int32_t forward_span(const Mode2DriveGate_t *gate)
{
    return (int32_t)gate->config.forward_pwm_us -
        (int32_t)gate->config.center_pwm_us;
}

static int32_t reverse_span(const Mode2DriveGate_t *gate)
{
    return (int32_t)gate->config.reverse_pwm_us -
        (int32_t)gate->config.center_pwm_us;
}

static Mode2DriveTargetDirection_t target_direction(float target_speed_mps,
                                                    const Mode2DriveGateConfig_t *config)
{
    (void)config;
    if (target_speed_mps > 0.0f)
    {
        return MODE2_DRIVE_TARGET_FORWARD;
    }
    if (target_speed_mps < 0.0f)
    {
        return MODE2_DRIVE_TARGET_REVERSE;
    }
    return MODE2_DRIVE_TARGET_NEUTRAL;
}

static Mode2DriveGateOutput_t make_output(const Mode2DriveGate_t *gate,
                                          Mode2DriveAction_t action,
                                          Mode2DriveReason_t reason,
                                          Mode2DrivePhase_t phase,
                                          EscTelemetryOutputPurpose_t purpose,
                                          Mode2DriveBrakePurpose_t brake_purpose,
                                          uint16_t pwm_us)
{
    Mode2DriveGateOutput_t output;

    memset(&output, 0, sizeof(output));
    output.action = action;
    output.reason = reason;
    output.phase = phase;
    output.permission = (gate != NULL) ?
        gate->permission : MODE2_DRIVE_PERMISSION_UNKNOWN;
    output.purpose = purpose;
    output.brake_purpose = brake_purpose;
    output.session_id = (gate != NULL) ? gate->current_session_id : 0U;
    output.final_pwm_us = pwm_us;
    output.action_conflict = (gate != NULL) ? gate->action_conflict : 0U;
    output.brake_confirmed = (gate != NULL) ? gate->brake_confirmed : 0U;
    output.incomplete =
        (reason == MODE2_DRIVE_REASON_WAITING_FOR_STOP ||
         reason == MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL ||
         reason == MODE2_DRIVE_REASON_BRAKE_REARM_REQUIRED ||
         reason == MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED ||
         reason == MODE2_DRIVE_REASON_STARTUP_QUERY ||
         reason == MODE2_DRIVE_REASON_STARTUP_HOLD) ? 1U : 0U;
    output.inhibited =
        (reason == MODE2_DRIVE_REASON_NOT_AUTHORIZED ||
         reason == MODE2_DRIVE_REASON_MOTION_UNAVAILABLE ||
         reason == MODE2_DRIVE_REASON_COMMAND_STALE ||
         reason == MODE2_DRIVE_REASON_ACTION_CONFLICT ||
         reason == MODE2_DRIVE_REASON_DIRECTION_UNKNOWN ||
         reason == MODE2_DRIVE_REASON_PID_INVALID) ? 1U : 0U;
    output.integral_reset =
        (action == MODE2_DRIVE_ACTION_FORWARD ||
         action == MODE2_DRIVE_ACTION_REVERSE ||
         (action == MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE &&
          brake_purpose != MODE2_DRIVE_BRAKE_PURPOSE_REVERSE)) ? 0U : 1U;
    output.pid_active = (output.integral_reset == 0U) ? 1U : 0U;
    output.startup_pending =
        (phase == MODE2_DRIVE_PHASE_F_START ||
         phase == MODE2_DRIVE_PHASE_R_QUERY ||
         reason == MODE2_DRIVE_REASON_STARTUP_HOLD) ? 1U : 0U;
    return output;
}

static Mode2DriveGateOutput_t neutral_output(Mode2DriveGate_t *gate,
                                             Mode2DriveReason_t reason,
                                             Mode2DrivePhase_t phase)
{
    Mode2DriveGateOutput_t output =
        make_output(gate,
                    MODE2_DRIVE_ACTION_NEUTRAL,
                    reason,
                    phase,
                    ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                    MODE2_DRIVE_BRAKE_PURPOSE_NONE,
                    gate->config.center_pwm_us);
    output.pid_min_us = 0.0f;
    output.pid_max_us = 0.0f;
    output.pid_applied_us = 0.0f;
    return output;
}

static Mode2DriveGateOutput_t forward_output(Mode2DriveGate_t *gate,
                                             Mode2DriveReason_t reason,
                                             Mode2DrivePhase_t phase,
                                             float raw_pid_us)
{
    const float high = (float)forward_span(gate);
    const float applied = clip_f32(raw_pid_us, 0.0f, high);
    Mode2DriveGateOutput_t output =
        make_output(gate,
                    MODE2_DRIVE_ACTION_FORWARD,
                    reason,
                    phase,
                    ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST,
                    MODE2_DRIVE_BRAKE_PURPOSE_NONE,
                    pwm_from_offset(gate, applied));
    output.pid_min_us = 0.0f;
    output.pid_max_us = high;
    output.pid_raw_us = raw_pid_us;
    output.pid_applied_us = applied;
    return output;
}

static Mode2DriveGateOutput_t reverse_output(Mode2DriveGate_t *gate,
                                             Mode2DriveReason_t reason,
                                             Mode2DrivePhase_t phase,
                                             float raw_pid_us)
{
    const float low = (float)reverse_span(gate);
    const float applied = clip_f32(raw_pid_us, low, 0.0f);
    Mode2DriveGateOutput_t output =
        make_output(gate,
                    MODE2_DRIVE_ACTION_REVERSE,
                    reason,
                    phase,
                    ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST,
                    MODE2_DRIVE_BRAKE_PURPOSE_NONE,
                    pwm_from_offset(gate, applied));
    output.pid_min_us = low;
    output.pid_max_us = 0.0f;
    output.pid_raw_us = raw_pid_us;
    output.pid_applied_us = applied;
    return output;
}

static Mode2DriveGateOutput_t forward_brake_output(Mode2DriveGate_t *gate,
                                                   Mode2DriveReason_t reason,
                                                   Mode2DrivePhase_t phase,
                                                   Mode2DriveBrakePurpose_t brake_purpose,
                                                   float raw_pid_us,
                                                   uint8_t full_brake)
{
    const float low = (float)reverse_span(gate);
    float applied;
    Mode2DriveGateOutput_t output;

    if (full_brake != 0U)
    {
        applied = low;
    }
    else
    {
        const float requested_brake_us =
            max_f32((float)gate->brake_hold_us, -raw_pid_us);
        const float clipped_brake_us =
            clip_f32(requested_brake_us,
                     (float)gate->brake_hold_us,
                     (float)(gate->config.center_pwm_us -
                             gate->config.reverse_pwm_us));
        applied = -clipped_brake_us;
    }

    output = make_output(gate,
                         MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE,
                         reason,
                         phase,
                         ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE,
                         brake_purpose,
                         pwm_from_offset(gate, applied));
    output.pid_min_us = low;
    output.pid_max_us = -(float)gate->brake_hold_us;
    output.pid_raw_us = raw_pid_us;
    output.pid_applied_us = applied;
    return output;
}

uint8_t Mode2DriveGate_ConfigIsValid(const Mode2DriveGateConfig_t *config,
                                     Mode2DriveReason_t *reason)
{
    uint8_t valid = 1U;

    if (config == NULL)
    {
        if (reason != NULL)
        {
            *reason = MODE2_DRIVE_REASON_INVALID_ARGUMENT;
        }
        return 0U;
    }

    if (valid_pwm(config->center_pwm_us) == 0U ||
        valid_pwm(config->forward_pwm_us) == 0U ||
        valid_pwm(config->reverse_pwm_us) == 0U ||
        config->reverse_pwm_us >= config->center_pwm_us ||
        config->forward_pwm_us <= config->center_pwm_us ||
        config->brake_min_us == 0U ||
        config->brake_min_us > config->center_pwm_us - config->reverse_pwm_us ||
        valid_duration(config->action_ack_ms) == 0U ||
        valid_duration(config->qualify_ms) == 0U ||
        valid_duration(config->neutral_dwell_ms) == 0U ||
        valid_duration(config->feedback_timeout_ms) == 0U ||
        valid_duration(config->command_timeout_ms) == 0U ||
        valid_duration(config->coast_eval_ms) == 0U ||
        valid_duration(config->coast_budget_ms) == 0U ||
        config->coast_eval_ms >= config->coast_budget_ms ||
        valid_duration(config->release_delay_ms) == 0U ||
        finite_positive(config->e_on_abs_mps) == 0U ||
        finite_positive(config->e_on_ratio) == 0U ||
        finite_positive(config->e_off_abs_mps) == 0U ||
        finite_positive(config->e_off_ratio) == 0U ||
        config->e_off_abs_mps >= config->e_on_abs_mps ||
        finite_positive(config->coast_progress_mps2) == 0U ||
        finite_positive(config->stopped_speed_threshold_mps) == 0U ||
        config->stopped_min_samples == 0U ||
        valid_duration(config->stopped_min_coverage_ms) == 0U)
    {
        valid = 0U;
    }

    if (reason != NULL)
    {
        *reason = (valid != 0U) ?
            MODE2_DRIVE_REASON_OK : MODE2_DRIVE_REASON_CONFIG_INVALID;
    }
    return valid;
}

static void bump_session_nonce(Mode2DriveGate_t *gate)
{
    if (gate->current_session_id == 0U)
    {
        gate->current_session_id = 1U;
        return;
    }
    if (gate->current_session_id >= MODE2_DRIVE_SESSION_MAX)
    {
        gate->current_session_id = 1U;
        return;
    }
    ++gate->current_session_id;
}

void Mode2DriveGate_ResetHistory(Mode2DriveGate_t *gate)
{
    if (gate == NULL)
    {
        return;
    }

    gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
    gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
    gate->has_observation = 0U;
    gate->has_last_observed_sample_id = 0U;
    gate->stop_evidence_valid = 0U;
    gate->stop_sample_count = 0U;
    gate->stop_first_tick_ms = 0U;
    gate->stop_last_tick_ms = 0U;
    gate->stop_established_tick_ms = 0U;
    gate->brake_active = 0U;
    gate->brake_purpose = MODE2_DRIVE_BRAKE_PURPOSE_NONE;
    gate->brake_confirmed = 0U;
    gate->brake_ack_pending = 0U;
    gate->brake_full_confirmed = 0U;
    gate->brake_rearm_required = 0U;
    gate->action_conflict = 0U;
    gate->fault_reason = MODE2_DRIVE_REASON_OK;
    gate->full_pwm_active = 0U;
    gate->brake_start_ms = 0U;
    gate->brake_full_start_ms = 0U;
    gate->brake_session_id = 0U;
    gate->brake_hold_us = gate->config.brake_min_us;
    gate->neutral_active = 0U;
    gate->neutral_confirmed = 0U;
    gate->neutral_start_ms = 0U;
    gate->coast_active = 0U;
    gate->coast_start_ms = 0U;
    gate->coast_target_mps = 0.0f;
    gate->release_predict_count = 0U;
    gate->release_predict_sample_id = 0U;
    gate->release_target_mps = 0.0f;
    gate->release_purpose = MODE2_DRIVE_BRAKE_PURPOSE_NONE;
    gate->last_purpose = ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL;
    bump_session_nonce(gate);
    gate->has_output_context = 0U;
    gate->last_pwm_us = gate->config.center_pwm_us;
    gate->last_commit_ms = 0U;
    gate->last_target_speed_mps = 0.0f;
    gate->has_last_target = 0U;
}

void Mode2DriveGate_Init(Mode2DriveGate_t *gate,
                         const Mode2DriveGateConfig_t *config)
{
    if (gate == NULL)
    {
        return;
    }

    memset(gate, 0, sizeof(*gate));
    gate->config_reason = MODE2_DRIVE_REASON_CONFIG_INVALID;
    gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
    gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
    gate->last_purpose = ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL;
    gate->current_session_id = 0U;
    if (config != NULL)
    {
        (void)Mode2DriveGate_SetConfig(gate, config);
    }
}

Mode2DriveReason_t Mode2DriveGate_SetConfig(Mode2DriveGate_t *gate,
                                            const Mode2DriveGateConfig_t *config)
{
    Mode2DriveReason_t reason = MODE2_DRIVE_REASON_OK;

    if (gate == NULL || config == NULL)
    {
        return MODE2_DRIVE_REASON_INVALID_ARGUMENT;
    }

    gate->config = *config;
    gate->config_valid = Mode2DriveGate_ConfigIsValid(config, &reason);
    gate->config_reason = reason;
    Mode2DriveGate_ResetHistory(gate);
    return reason;
}

static void clear_stop_evidence(Mode2DriveGate_t *gate)
{
    gate->stop_evidence_valid = 0U;
    gate->stop_sample_count = 0U;
    gate->stop_first_tick_ms = 0U;
    gate->stop_last_tick_ms = 0U;
    gate->stop_established_tick_ms = 0U;
}

static uint8_t is_new_sample(const Mode2DriveGate_t *gate,
                             const Mode2DriveMotionObservation_t *observation)
{
    return (gate->has_last_observed_sample_id == 0U ||
            gate->last_observed_sample_id != observation->sample_id) ? 1U : 0U;
}

static uint8_t observation_is_usable_for_evidence(
    const Mode2DriveGate_t *gate,
    const Mode2DriveMotionObservation_t *observation)
{
    if (observation->available == 0U ||
        observation->speed_magnitude_valid == 0U ||
        observation->context_valid == 0U ||
        finite_f32(observation->speed_magnitude_mps) == 0U ||
        observation->speed_magnitude_mps < 0.0f ||
        (observation->direction_known != 0U &&
         finite_f32(observation->signed_speed_mps) == 0U) ||
        (observation->acceleration_valid != 0U &&
         finite_f32(observation->acceleration_mps2) == 0U))
    {
        return 0U;
    }

    if (gate->has_observation != 0U &&
        tick_at_or_after(observation->sample_tick_ms,
                         gate->latest_observation.sample_tick_ms) == 0U)
    {
        return 0U;
    }

    if (is_new_sample(gate, observation) == 0U)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t stop_evidence_context_valid(
    const Mode2DriveGate_t *gate,
    const Mode2DriveMotionObservation_t *observation)
{
    if (gate->full_pwm_active != 0U)
    {
        return (observation->context_session_id == gate->brake_session_id &&
                observation->context_purpose ==
                    ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE &&
                observation->context_pwm_us <= gate->config.reverse_pwm_us &&
                observation->esc_action == MODE2_DRIVE_ESC_ACTION_BRAKE &&
                tick_at_or_after(observation->sample_tick_ms,
                                 gate->brake_full_start_ms) != 0U) ? 1U : 0U;
    }

    if (gate->brake_active != 0U)
    {
        return (observation->context_session_id == gate->brake_session_id &&
                observation->context_purpose ==
                    ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE &&
                observation->esc_action == MODE2_DRIVE_ESC_ACTION_BRAKE) ? 1U : 0U;
    }

    if (gate->neutral_active != 0U ||
        gate->has_output_context != 0U)
    {
        return (observation->context_session_id == gate->current_session_id &&
                observation->context_purpose ==
                    ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL &&
                observation->esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL) ? 1U : 0U;
    }

    return (observation->context_purpose ==
                ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL &&
            observation->esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL) ? 1U : 0U;
}

static void update_stop_evidence(Mode2DriveGate_t *gate,
                                 const Mode2DriveMotionObservation_t *observation)
{
    uint8_t stopped_sample;
    uint32_t coverage_ms;

    if (observation->available == 0U ||
        observation->speed_magnitude_valid == 0U)
    {
        clear_stop_evidence(gate);
        return;
    }

    if (stop_evidence_context_valid(gate, observation) == 0U)
    {
        if (gate->brake_active != 0U &&
            gate->brake_purpose == MODE2_DRIVE_BRAKE_PURPOSE_STOP &&
            gate->stop_evidence_valid != 0U &&
            observation->context_session_id == gate->brake_session_id &&
            observation->context_purpose ==
                ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE &&
            observation->esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL)
        {
            return;
        }
        clear_stop_evidence(gate);
        return;
    }

    stopped_sample =
        (observation->speed_magnitude_mps <=
         gate->config.stopped_speed_threshold_mps) ? 1U : 0U;
    if (stopped_sample == 0U)
    {
        clear_stop_evidence(gate);
        return;
    }

    if (gate->stop_sample_count == 0U)
    {
        gate->stop_first_tick_ms = observation->sample_tick_ms;
    }
    gate->stop_last_tick_ms = observation->sample_tick_ms;
    if (gate->stop_sample_count < 0xFFFFFFFFUL)
    {
        ++gate->stop_sample_count;
    }

    coverage_ms = gate->stop_last_tick_ms - gate->stop_first_tick_ms;
    if (gate->stop_sample_count >= (uint32_t)gate->config.stopped_min_samples &&
        coverage_ms >= gate->config.stopped_min_coverage_ms)
    {
        gate->stop_evidence_valid = 1U;
        gate->stop_established_tick_ms = observation->sample_tick_ms;
    }
}

static uint8_t context_is_current(const Mode2DriveGate_t *gate,
                                  const Mode2DriveMotionObservation_t *observation)
{
    return (observation->context_valid != 0U &&
            observation->context_session_id == gate->current_session_id) ? 1U : 0U;
}

static uint8_t context_is_brake_session(const Mode2DriveGate_t *gate,
                                        const Mode2DriveMotionObservation_t *observation)
{
    return (observation->context_valid != 0U &&
            observation->context_session_id == gate->brake_session_id &&
            observation->context_purpose ==
                ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE) ? 1U : 0U;
}

static uint8_t moving_forward(const Mode2DriveGate_t *gate,
                              const Mode2DriveMotionObservation_t *observation)
{
    return (observation->direction_known != 0U &&
            observation->direction == MODE2_DRIVE_TARGET_FORWARD &&
            observation->speed_magnitude_mps >
                gate->config.stopped_speed_threshold_mps) ? 1U : 0U;
}

static uint8_t moving_reverse(const Mode2DriveGate_t *gate,
                              const Mode2DriveMotionObservation_t *observation)
{
    return (observation->direction_known != 0U &&
            observation->direction == MODE2_DRIVE_TARGET_REVERSE &&
            observation->speed_magnitude_mps >
                gate->config.stopped_speed_threshold_mps) ? 1U : 0U;
}

static uint8_t stopped_now(const Mode2DriveGate_t *gate);
static void clear_coast(Mode2DriveGate_t *gate);
static void clear_brake(Mode2DriveGate_t *gate);
static void recover_fault_if_neutral_stopped(Mode2DriveGate_t *gate);

static void observe_drive_permission(Mode2DriveGate_t *gate,
                                     const Mode2DriveMotionObservation_t *observation)
{
    if (observation->esc_action != MODE2_DRIVE_ESC_ACTION_DRIVE ||
        observation->context_valid == 0U ||
        observation->context_session_id != gate->current_session_id)
    {
        return;
    }

    if (observation->context_purpose ==
            ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST &&
        moving_forward(gate, observation) != 0U)
    {
        gate->permission = MODE2_DRIVE_PERMISSION_F_READY;
        gate->phase = MODE2_DRIVE_PHASE_F_DRIVE;
        gate->brake_rearm_required = 0U;
    }
    else if (observation->context_purpose ==
                 ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST &&
             moving_reverse(gate, observation) != 0U)
    {
        gate->permission = MODE2_DRIVE_PERMISSION_R_READY;
        gate->phase = MODE2_DRIVE_PHASE_R_DRIVE;
    }
}

static void observe_brake_action(Mode2DriveGate_t *gate,
                                 const Mode2DriveMotionObservation_t *observation)
{
    if (gate->brake_active == 0U)
    {
        if (observation->esc_action == MODE2_DRIVE_ESC_ACTION_BRAKE &&
            observation->context_valid != 0U &&
            observation->context_session_id == gate->current_session_id &&
            observation->context_purpose ==
                ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST &&
            observation->context_pwm_us < gate->config.center_pwm_us)
        {
            gate->brake_active = 1U;
            gate->brake_purpose = MODE2_DRIVE_BRAKE_PURPOSE_REVERSE;
            gate->brake_confirmed = 1U;
            gate->brake_ack_pending = 0U;
            gate->brake_session_id = observation->context_session_id;
            gate->brake_start_ms = observation->sample_tick_ms;
            gate->brake_hold_us = gate->config.brake_min_us;
            gate->permission = MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE;
            gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_REVERSE;
        }
        return;
    }

    if (context_is_brake_session(gate, observation) == 0U)
    {
        return;
    }

    if (observation->esc_action == MODE2_DRIVE_ESC_ACTION_BRAKE)
    {
        gate->brake_confirmed = 1U;
        gate->brake_ack_pending = 0U;
        if (gate->full_pwm_active != 0U &&
            observation->context_pwm_us <= gate->config.reverse_pwm_us &&
            tick_at_or_after(observation->sample_tick_ms,
                             gate->brake_full_start_ms) != 0U)
        {
            gate->brake_full_confirmed = 1U;
        }
    }
    else if (gate->brake_confirmed != 0U &&
             (observation->esc_action == MODE2_DRIVE_ESC_ACTION_DRIVE ||
              observation->esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL))
    {
        if (observation->esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL &&
            gate->brake_purpose == MODE2_DRIVE_BRAKE_PURPOSE_STOP &&
            stopped_now(gate) != 0U)
        {
            return;
        }
        gate->action_conflict = 1U;
        gate->fault_reason = MODE2_DRIVE_REASON_ACTION_CONFLICT;
    }
}

static void observe_neutral_action(Mode2DriveGate_t *gate,
                                   const Mode2DriveMotionObservation_t *observation)
{
    if (gate->neutral_active != 0U &&
        observation->esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL &&
        context_is_current(gate, observation) != 0U &&
        observation->context_purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL)
    {
        gate->neutral_confirmed = 1U;
    }
}

static void observe_track_release_prediction(Mode2DriveGate_t *gate,
                                             const Mode2DriveMotionObservation_t *observation,
                                             uint32_t now_tick_ms)
{
    float horizon_s;
    float release_v;
    float off;

    if (gate->brake_active == 0U ||
        gate->brake_purpose != MODE2_DRIVE_BRAKE_PURPOSE_TRACK ||
        gate->release_purpose != MODE2_DRIVE_BRAKE_PURPOSE_TRACK)
    {
        return;
    }

    if (observation->direction_known == 0U ||
        observation->direction != MODE2_DRIVE_TARGET_FORWARD ||
        observation->acceleration_valid == 0U ||
        observation->acceleration_mps2 >= 0.0f)
    {
        gate->release_predict_count = 0U;
        gate->release_predict_sample_id = observation->sample_id;
        return;
    }

    horizon_s = ((float)elapsed_ms(now_tick_ms, observation->sample_tick_ms) +
                 20.0f + (float)gate->config.release_delay_ms) / 1000.0f;
    release_v = observation->signed_speed_mps +
        observation->acceleration_mps2 * horizon_s;
    if (release_v < 0.0f)
    {
        release_v = 0.0f;
    }

    off = max_f32(gate->config.e_off_abs_mps,
                  gate->config.e_off_ratio * gate->release_target_mps);
    if (release_v <= gate->release_target_mps + off)
    {
        ++gate->release_predict_count;
        gate->release_predict_sample_id = observation->sample_id;
    }
    else
    {
        gate->release_predict_count = 0U;
        gate->release_predict_sample_id = observation->sample_id;
    }
}

void Mode2DriveGate_Observe(Mode2DriveGate_t *gate,
                            const Mode2DriveMotionObservation_t *observation,
                            uint32_t now_tick_ms)
{
    if (gate == NULL || observation == NULL)
    {
        return;
    }

    if (observation_is_usable_for_evidence(gate, observation) == 0U)
    {
        return;
    }

    if (observation->esc_action == MODE2_DRIVE_ESC_ACTION_UNKNOWN)
    {
        gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
        gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
        gate->has_observation = 0U;
        gate->last_observed_sample_id = observation->sample_id;
        gate->has_last_observed_sample_id = 1U;
        clear_stop_evidence(gate);
        clear_brake(gate);
        clear_coast(gate);
        return;
    }

    gate->latest_observation = *observation;
    gate->has_observation = 1U;

    update_stop_evidence(gate, observation);
    gate->last_observed_sample_id = observation->sample_id;
    gate->has_last_observed_sample_id = 1U;

    observe_track_release_prediction(gate, observation, now_tick_ms);
    observe_brake_action(gate, observation);
    observe_neutral_action(gate, observation);
    recover_fault_if_neutral_stopped(gate);
    if (gate->action_conflict != 0U)
    {
        return;
    }
    observe_drive_permission(gate, observation);
}

static uint8_t feedback_fresh(const Mode2DriveGate_t *gate, uint32_t now_ms)
{
    return (gate->has_observation != 0U &&
            gate->latest_observation.available != 0U &&
            gate->latest_observation.speed_magnitude_valid != 0U &&
            elapsed_ms(now_ms, gate->latest_observation.sample_tick_ms) <=
                gate->config.feedback_timeout_ms) ? 1U : 0U;
}

static uint8_t command_fresh(const Mode2DriveGate_t *gate,
                             const Mode2DriveGateInput_t *input,
                             uint32_t now_ms)
{
    return (input->command_valid != 0U &&
            elapsed_ms(now_ms, input->command_tick_ms) <=
                gate->config.command_timeout_ms) ? 1U : 0U;
}

static uint8_t stopped_now(const Mode2DriveGate_t *gate)
{
    return gate->stop_evidence_valid;
}

static uint8_t stopped_after(const Mode2DriveGate_t *gate, uint32_t tick_ms)
{
    return (gate->stop_evidence_valid != 0U &&
            tick_at_or_after(gate->stop_first_tick_ms, tick_ms) != 0U) ? 1U : 0U;
}

static uint8_t nonzero_unknown_motion(const Mode2DriveGate_t *gate)
{
    return (gate->latest_observation.direction_known == 0U &&
            stopped_now(gate) == 0U &&
            gate->latest_observation.speed_magnitude_mps >
                gate->config.stopped_speed_threshold_mps) ? 1U : 0U;
}

static void clear_coast(Mode2DriveGate_t *gate)
{
    gate->coast_active = 0U;
    gate->coast_start_ms = 0U;
    gate->coast_target_mps = 0.0f;
}

static void clear_release_prediction(Mode2DriveGate_t *gate)
{
    gate->release_predict_count = 0U;
    gate->release_predict_sample_id = 0U;
    gate->release_target_mps = 0.0f;
    gate->release_purpose = MODE2_DRIVE_BRAKE_PURPOSE_NONE;
}

static void clear_brake(Mode2DriveGate_t *gate)
{
    gate->brake_active = 0U;
    gate->brake_purpose = MODE2_DRIVE_BRAKE_PURPOSE_NONE;
    gate->brake_confirmed = 0U;
    gate->brake_ack_pending = 0U;
    gate->brake_full_confirmed = 0U;
    gate->full_pwm_active = 0U;
    gate->brake_start_ms = 0U;
    gate->brake_full_start_ms = 0U;
    gate->brake_session_id = 0U;
    gate->brake_hold_us = gate->config.brake_min_us;
}

static uint8_t target_changed(Mode2DriveGate_t *gate, float target)
{
    if (gate->has_last_target == 0U ||
        abs_f32(gate->last_target_speed_mps - target) > 0.0001f)
    {
        gate->last_target_speed_mps = target;
        gate->has_last_target = 1U;
        clear_release_prediction(gate);
        return 1U;
    }
    return 0U;
}

static float raw_pid_without_integral(const Mode2DriveGateInput_t *input)
{
    if (finite_f32(input->pid_integral_us) == 0U)
    {
        return input->pid_raw_us;
    }
    return input->pid_raw_us - input->pid_integral_us;
}

static float e_on(const Mode2DriveGate_t *gate, float target)
{
    return max_f32(gate->config.e_on_abs_mps,
                   gate->config.e_on_ratio * target);
}

static float e_off(const Mode2DriveGate_t *gate, float target)
{
    return max_f32(gate->config.e_off_abs_mps,
                   gate->config.e_off_ratio * target);
}

static uint8_t track_brake_release_ready(Mode2DriveGate_t *gate,
                                         const Mode2DriveGateInput_t *input,
                                         float target,
                                         uint32_t now_ms)
{
    (void)input;
    (void)now_ms;
    if (gate->latest_observation.signed_speed_mps <= target + e_off(gate, target))
    {
        return 1U;
    }
    if (gate->release_purpose == MODE2_DRIVE_BRAKE_PURPOSE_TRACK &&
        abs_f32(gate->release_target_mps - target) <= 0.0001f &&
        gate->release_predict_count >= 2U)
    {
        return 1U;
    }
    return 0U;
}

static uint8_t brake_ack_timeout(const Mode2DriveGate_t *gate, uint32_t now_ms)
{
    return (gate->brake_active != 0U &&
            gate->brake_ack_pending != 0U &&
            gate->brake_confirmed == 0U &&
            elapsed_ms(now_ms, gate->brake_start_ms) >
                gate->config.action_ack_ms) ? 1U : 0U;
}

static void enter_fault_recovery(Mode2DriveGate_t *gate, Mode2DriveReason_t reason)
{
    gate->action_conflict = 1U;
    gate->fault_reason = reason;
    gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
    gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
}

static void recover_fault_if_neutral_stopped(Mode2DriveGate_t *gate)
{
    if (gate->action_conflict != 0U &&
        gate->neutral_confirmed != 0U &&
        stopped_now(gate) != 0U)
    {
        gate->action_conflict = 0U;
        gate->fault_reason = MODE2_DRIVE_REASON_OK;
        gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
        gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
        gate->brake_rearm_required = 0U;
    }
}

static Mode2DriveGateOutput_t conflict_output(Mode2DriveGate_t *gate)
{
    const Mode2DriveReason_t reason =
        (gate->fault_reason == MODE2_DRIVE_REASON_OK) ?
        MODE2_DRIVE_REASON_ACTION_CONFLICT : gate->fault_reason;

    gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
    gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
    clear_brake(gate);
    clear_coast(gate);
    clear_release_prediction(gate);
    return neutral_output(gate,
                          reason,
                          MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
}

static Mode2DriveGateOutput_t startup_hold_output(Mode2DriveGate_t *gate)
{
    Mode2DriveGateOutput_t output;

    if (gate->last_purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST)
    {
        output = make_output(gate,
                             MODE2_DRIVE_ACTION_FORWARD,
                             MODE2_DRIVE_REASON_STARTUP_HOLD,
                             MODE2_DRIVE_PHASE_F_START,
                             gate->last_purpose,
                             MODE2_DRIVE_BRAKE_PURPOSE_NONE,
                             gate->last_pwm_us);
        output.pid_active = 0U;
        return output;
    }
    if (gate->last_purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST)
    {
        output = make_output(gate,
                             MODE2_DRIVE_ACTION_REVERSE,
                             MODE2_DRIVE_REASON_STARTUP_HOLD,
                             MODE2_DRIVE_PHASE_R_START,
                             gate->last_purpose,
                             MODE2_DRIVE_BRAKE_PURPOSE_NONE,
                             gate->last_pwm_us);
        output.pid_active = 0U;
        return output;
    }
    return neutral_output(gate,
                          MODE2_DRIVE_REASON_DIRECTION_UNKNOWN,
                          MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
}

static uint8_t startup_hold_context_matches(const Mode2DriveGate_t *gate,
                                            Mode2DriveTargetDirection_t target,
                                            uint32_t now_ms)
{
    const EscTelemetryOutputPurpose_t expected_purpose =
        (target == MODE2_DRIVE_TARGET_FORWARD) ?
        ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST :
        ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST;

    return (target != MODE2_DRIVE_TARGET_NEUTRAL &&
            gate->has_output_context != 0U &&
            gate->last_purpose == expected_purpose &&
            gate->latest_observation.context_valid != 0U &&
            gate->latest_observation.context_purpose == expected_purpose &&
            gate->latest_observation.context_session_id ==
                gate->current_session_id &&
            elapsed_ms(now_ms, gate->last_commit_ms) <=
                gate->config.feedback_timeout_ms) ? 1U : 0U;
}

static Mode2DriveGateOutput_t evaluate_brake_active(Mode2DriveGate_t *gate,
                                                    const Mode2DriveGateInput_t *input,
                                                    Mode2DriveTargetDirection_t target,
                                                    uint32_t now_ms)
{
    const float target_abs = abs_f32(input->target_speed_mps);

    if (target == MODE2_DRIVE_TARGET_REVERSE)
    {
        gate->brake_purpose = MODE2_DRIVE_BRAKE_PURPOSE_REVERSE;
        gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_REVERSE;
        if (gate->brake_full_confirmed != 0U &&
            gate->full_pwm_active != 0U &&
            elapsed_ms(now_ms, gate->brake_full_start_ms) >=
                gate->config.qualify_ms &&
            stopped_after(gate, gate->brake_full_start_ms) != 0U)
        {
            gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
            gate->phase = MODE2_DRIVE_PHASE_R_DWELL;
            clear_brake(gate);
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                                  MODE2_DRIVE_PHASE_R_DWELL);
        }
        if (brake_ack_timeout(gate, now_ms) != 0U)
        {
            enter_fault_recovery(gate, MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED);
            clear_brake(gate);
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED,
                                  MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
        }
        if (gate->full_pwm_active != 0U &&
            gate->brake_full_confirmed == 0U &&
            elapsed_ms(now_ms, gate->brake_full_start_ms) >
                gate->config.action_ack_ms)
        {
            enter_fault_recovery(gate, MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED);
            clear_brake(gate);
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED,
                                  MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
        }
        return forward_brake_output(gate,
                                    MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                    MODE2_DRIVE_PHASE_F_BRAKE_REVERSE,
                                    MODE2_DRIVE_BRAKE_PURPOSE_REVERSE,
                                    0.0f,
                                    1U);
    }

    if (target == MODE2_DRIVE_TARGET_NEUTRAL)
    {
        gate->brake_purpose = MODE2_DRIVE_BRAKE_PURPOSE_STOP;
        gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_STOP;
        if (stopped_now(gate) != 0U &&
            (gate->latest_observation.esc_action == MODE2_DRIVE_ESC_ACTION_BRAKE ||
             gate->latest_observation.esc_action == MODE2_DRIVE_ESC_ACTION_NEUTRAL))
        {
            gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
            clear_brake(gate);
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                                  MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
        }
        if (brake_ack_timeout(gate, now_ms) != 0U && stopped_now(gate) == 0U)
        {
            enter_fault_recovery(gate, MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED);
            clear_brake(gate);
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED,
                                  MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
        }
        return forward_brake_output(gate,
                                    MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                                    MODE2_DRIVE_PHASE_F_BRAKE_STOP,
                                    MODE2_DRIVE_BRAKE_PURPOSE_STOP,
                                    (input->pid_valid != 0U &&
                                     finite_f32(input->pid_raw_us) != 0U) ?
                                        input->pid_raw_us :
                                        -(float)gate->config.brake_min_us,
                                    0U);
    }

    if (input->pid_valid == 0U || finite_f32(input->pid_raw_us) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_PID_INVALID,
                              MODE2_DRIVE_PHASE_F_BRAKE_TRACK);
    }
    gate->release_target_mps = target_abs;
    gate->release_purpose = MODE2_DRIVE_BRAKE_PURPOSE_TRACK;
    gate->brake_purpose = MODE2_DRIVE_BRAKE_PURPOSE_TRACK;
    gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_TRACK;
    if (track_brake_release_ready(gate, input, target_abs, now_ms) != 0U)
    {
        gate->brake_rearm_required = 1U;
        gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
        gate->phase = MODE2_DRIVE_PHASE_F_RECOVER;
        clear_brake(gate);
        clear_release_prediction(gate);
        if (input->pid_raw_us > 0.0f)
        {
            return forward_output(gate,
                                  MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                                  MODE2_DRIVE_PHASE_F_RECOVER,
                                  input->pid_raw_us);
        }
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                              MODE2_DRIVE_PHASE_F_RECOVER);
    }
    if (brake_ack_timeout(gate, now_ms) != 0U)
    {
        enter_fault_recovery(gate, MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED);
        clear_brake(gate);
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    return forward_brake_output(gate,
                                MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                MODE2_DRIVE_PHASE_F_BRAKE_TRACK,
                                MODE2_DRIVE_BRAKE_PURPOSE_TRACK,
                                input->pid_raw_us,
                                0U);
}

static uint8_t forward_needs_brake(Mode2DriveGate_t *gate,
                                   float target,
                                   uint32_t now_ms)
{
    const Mode2DriveMotionObservation_t *obs = &gate->latest_observation;
    const float delta = obs->signed_speed_mps - target;
    const float off = e_off(gate, target);
    uint32_t coast_ms;
    float t_need_s;
    float t_remaining_s;

    if (delta <= off)
    {
        clear_coast(gate);
        return 0U;
    }

    if (gate->coast_active == 0U ||
        target > gate->coast_target_mps + 0.0001f)
    {
        gate->coast_active = 1U;
        gate->coast_start_ms = now_ms;
        gate->coast_target_mps = target;
    }
    else
    {
        gate->coast_target_mps = target;
    }

    if (delta >= e_on(gate, target))
    {
        return 1U;
    }

    coast_ms = elapsed_ms(now_ms, gate->coast_start_ms);
    if (coast_ms < gate->config.coast_eval_ms)
    {
        return 0U;
    }

    if (obs->acceleration_valid != 0U &&
        obs->acceleration_mps2 < -gate->config.coast_progress_mps2)
    {
        t_need_s = (delta - off) / (-obs->acceleration_mps2);
    }
    else
    {
        t_need_s = 1000000.0f;
    }

    t_remaining_s =
        ((float)gate->config.coast_budget_ms - (float)coast_ms) / 1000.0f;
    if (t_remaining_s < 0.020f)
    {
        t_remaining_s = 0.020f;
    }
    return (coast_ms >= gate->config.coast_budget_ms ||
            t_need_s > t_remaining_s) ? 1U : 0U;
}

static Mode2DriveGateOutput_t evaluate_forward_ready(Mode2DriveGate_t *gate,
                                                     const Mode2DriveGateInput_t *input,
                                                     Mode2DriveTargetDirection_t target,
                                                     uint32_t now_ms)
{
    const float target_abs = (input->target_speed_mps > 0.0f) ?
        input->target_speed_mps : 0.0f;

    if (target == MODE2_DRIVE_TARGET_REVERSE)
    {
        gate->permission = MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE;
        gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_REVERSE;
        gate->brake_hold_us = gate->config.brake_min_us;
        return forward_brake_output(gate,
                                    MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                    MODE2_DRIVE_PHASE_F_BRAKE_REVERSE,
                                    MODE2_DRIVE_BRAKE_PURPOSE_REVERSE,
                                    0.0f,
                                    1U);
    }

    if (target == MODE2_DRIVE_TARGET_NEUTRAL)
    {
        if (stopped_now(gate) != 0U)
        {
            gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
            gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                                  MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
        }
        gate->permission = MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE;
        gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_STOP;
        gate->brake_hold_us = gate->config.brake_min_us;
        {
            Mode2DriveGateOutput_t output =
                forward_brake_output(gate,
                                     MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                                     MODE2_DRIVE_PHASE_F_BRAKE_STOP,
                                     MODE2_DRIVE_BRAKE_PURPOSE_STOP,
                                     (input->pid_valid != 0U &&
                                      finite_f32(input->pid_raw_us) != 0U) ?
                                        raw_pid_without_integral(input) :
                                        -(float)gate->config.brake_min_us,
                                     0U);
            output.integral_reset = 1U;
            return output;
        }
    }

    if (input->pid_valid == 0U || finite_f32(input->pid_raw_us) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_PID_INVALID,
                              MODE2_DRIVE_PHASE_F_DRIVE);
    }

    if (gate->latest_observation.direction_known == 0U &&
        stopped_now(gate) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_DIRECTION_UNKNOWN,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }

    if (gate->latest_observation.direction == MODE2_DRIVE_TARGET_FORWARD &&
        forward_needs_brake(gate, target_abs, now_ms) != 0U)
    {
        if (gate->brake_rearm_required != 0U)
        {
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_BRAKE_REARM_REQUIRED,
                                  MODE2_DRIVE_PHASE_F_RECOVER);
        }
        gate->permission = MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE;
        gate->phase = MODE2_DRIVE_PHASE_F_BRAKE_TRACK;
        gate->brake_hold_us = gate->config.brake_min_us;
        {
            Mode2DriveGateOutput_t output =
                forward_brake_output(gate,
                                     MODE2_DRIVE_REASON_BRAKING_FORWARD,
                                     MODE2_DRIVE_PHASE_F_BRAKE_TRACK,
                                     MODE2_DRIVE_BRAKE_PURPOSE_TRACK,
                                     raw_pid_without_integral(input),
                                     0U);
            output.integral_reset = 1U;
            return output;
        }
    }

    if (gate->latest_observation.direction == MODE2_DRIVE_TARGET_FORWARD &&
        gate->coast_active != 0U)
    {
        if (gate->latest_observation.signed_speed_mps >
            target_abs + e_off(gate, target_abs))
        {
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_FORWARD_COAST,
                                  MODE2_DRIVE_PHASE_F_COAST);
        }
        clear_coast(gate);
    }

    gate->phase = MODE2_DRIVE_PHASE_F_DRIVE;
    return forward_output(gate,
                          MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                          MODE2_DRIVE_PHASE_F_DRIVE,
                          input->pid_raw_us);
}

static Mode2DriveGateOutput_t evaluate_reverse_ready(Mode2DriveGate_t *gate,
                                                     const Mode2DriveGateInput_t *input,
                                                     Mode2DriveTargetDirection_t target)
{
    if (target == MODE2_DRIVE_TARGET_REVERSE)
    {
        if (input->pid_valid == 0U || finite_f32(input->pid_raw_us) == 0U)
        {
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_PID_INVALID,
                                  MODE2_DRIVE_PHASE_R_DRIVE);
        }
        if (input->pid_raw_us < 0.0f)
        {
            gate->phase = MODE2_DRIVE_PHASE_R_DRIVE;
            return reverse_output(gate,
                                  MODE2_DRIVE_REASON_REVERSE_PERMITTED,
                                  MODE2_DRIVE_PHASE_R_DRIVE,
                                  input->pid_raw_us);
        }
        gate->phase = MODE2_DRIVE_PHASE_R_COAST;
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_REVERSE_COAST,
                              MODE2_DRIVE_PHASE_R_COAST);
    }

    gate->phase = MODE2_DRIVE_PHASE_R_COAST;
    if (stopped_now(gate) != 0U)
    {
        gate->permission = MODE2_DRIVE_PERMISSION_UNKNOWN;
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    return neutral_output(gate,
                          MODE2_DRIVE_REASON_REVERSE_COAST,
                          MODE2_DRIVE_PHASE_R_COAST);
}

static uint8_t neutral_dwell_ready(const Mode2DriveGate_t *gate, uint32_t now_ms)
{
    return (gate->neutral_active != 0U &&
            gate->neutral_confirmed != 0U &&
            stopped_now(gate) != 0U &&
            elapsed_ms(now_ms, gate->neutral_start_ms) >=
                gate->config.neutral_dwell_ms) ? 1U : 0U;
}

static Mode2DriveGateOutput_t evaluate_unknown(Mode2DriveGate_t *gate,
                                               const Mode2DriveGateInput_t *input,
                                               Mode2DriveTargetDirection_t target,
                                               uint32_t now_ms)
{
    if (target == MODE2_DRIVE_TARGET_NEUTRAL)
    {
        gate->phase = MODE2_DRIVE_PHASE_NEUTRAL_WAIT;
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_TARGET_NEUTRAL,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }

    if (gate->phase == MODE2_DRIVE_PHASE_R_DWELL)
    {
        if (neutral_dwell_ready(gate, now_ms) != 0U)
        {
            if (target == MODE2_DRIVE_TARGET_REVERSE)
            {
                if (input->pid_valid == 0U ||
                    finite_f32(input->pid_raw_us) == 0U)
                {
                    return neutral_output(gate,
                                          MODE2_DRIVE_REASON_PID_INVALID,
                                          MODE2_DRIVE_PHASE_R_START);
                }
                gate->permission = MODE2_DRIVE_PERMISSION_R_READY;
                gate->phase = MODE2_DRIVE_PHASE_R_START;
                return reverse_output(gate,
                                      MODE2_DRIVE_REASON_REVERSE_PERMITTED,
                                      MODE2_DRIVE_PHASE_R_START,
                                      input->pid_raw_us);
            }
            if (input->pid_valid == 0U ||
                finite_f32(input->pid_raw_us) == 0U)
            {
                return neutral_output(gate,
                                      MODE2_DRIVE_REASON_PID_INVALID,
                                      MODE2_DRIVE_PHASE_F_START);
            }
            gate->phase = MODE2_DRIVE_PHASE_F_START;
            return forward_output(gate,
                                  MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                                  MODE2_DRIVE_PHASE_F_START,
                                  input->pid_raw_us);
        }
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                              MODE2_DRIVE_PHASE_R_DWELL);
    }

    if (nonzero_unknown_motion(gate) != 0U)
    {
        if (startup_hold_context_matches(gate, target, now_ms) != 0U)
        {
            return startup_hold_output(gate);
        }
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_DIRECTION_UNKNOWN,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }

    /*
     * The first propulsion command may be reported by the ESC as NEUTRAL
     * until the motor crosses its electrical dead zone. Keep the just-issued
     * request alive for the full feedback_timeout_ms window; otherwise the
     * gate emits one propulsion frame, immediately returns to 1500 us, and
     * can never reach DRIVE.
     *
     * This check must come before neutral_dwell_ready so that a fresh dwell
     * cannot restart the single-frame cycle while the ESC is still responding.
     * The ESC reports at 10 Hz when stopped (100 ms), so the hold must span
     * at least two 10-Hz intervals to guarantee a DRIVE reply arrives while
     * the session is still open. A DRIVE sample still has to confirm the
     * direction before the permission becomes F_READY/R_READY.
     */
    if ((gate->phase == MODE2_DRIVE_PHASE_F_START ||
         gate->phase == MODE2_DRIVE_PHASE_R_QUERY ||
         gate->phase == MODE2_DRIVE_PHASE_R_START) &&
        gate->has_output_context != 0U &&
        ((target == MODE2_DRIVE_TARGET_FORWARD &&
          input->pid_raw_us > 0.0f) ||
         (target == MODE2_DRIVE_TARGET_REVERSE &&
          input->pid_raw_us < 0.0f)) &&
        elapsed_ms(now_ms, gate->last_commit_ms) <=
            gate->config.feedback_timeout_ms)
    {
        if (startup_hold_context_matches(gate, target, now_ms) != 0U)
        {
            return startup_hold_output(gate);
        }
        /* Context not yet confirmed: re-emit the live PID propulsion PWM to keep
         * the ESC busy until it responds. pid_active is kept alive so the
         * integral can accumulate if the initial proportional jump is too weak
         * to break the ESC deadband on small commands. */
        if (target == MODE2_DRIVE_TARGET_FORWARD)
        {
            return forward_output(gate,
                                  MODE2_DRIVE_REASON_STARTUP_HOLD,
                                  MODE2_DRIVE_PHASE_F_START,
                                  input->pid_raw_us);
        }
        else
        {
            return reverse_output(gate,
                                  MODE2_DRIVE_REASON_STARTUP_HOLD,
                                  gate->phase,
                                  input->pid_raw_us);
        }
    }

    if (target == MODE2_DRIVE_TARGET_FORWARD &&
        gate->phase == MODE2_DRIVE_PHASE_F_RECOVER &&
        gate->latest_observation.direction_known != 0U &&
        gate->latest_observation.direction == MODE2_DRIVE_TARGET_FORWARD)
    {
        if (input->pid_valid == 0U || finite_f32(input->pid_raw_us) == 0U)
        {
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_PID_INVALID,
                                  MODE2_DRIVE_PHASE_F_RECOVER);
        }
        if (input->pid_raw_us <= 0.0f)
        {
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_BRAKE_REARM_REQUIRED,
                                  MODE2_DRIVE_PHASE_F_RECOVER);
        }
        return forward_output(gate,
                              MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                              MODE2_DRIVE_PHASE_F_RECOVER,
                              input->pid_raw_us);
    }

    if (stopped_now(gate) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_WAITING_FOR_STOP,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    if (neutral_dwell_ready(gate, now_ms) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }

    if (target == MODE2_DRIVE_TARGET_FORWARD)
    {
        if (input->pid_valid == 0U || finite_f32(input->pid_raw_us) == 0U)
        {
            return neutral_output(gate,
                                  MODE2_DRIVE_REASON_PID_INVALID,
                                  MODE2_DRIVE_PHASE_F_START);
        }
        gate->phase = MODE2_DRIVE_PHASE_F_START;
        return forward_output(gate,
                              MODE2_DRIVE_REASON_FORWARD_PERMITTED,
                              MODE2_DRIVE_PHASE_F_START,
                              input->pid_raw_us);
    }

    gate->phase = MODE2_DRIVE_PHASE_R_QUERY;
    if (input->pid_valid == 0U || finite_f32(input->pid_raw_us) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_PID_INVALID,
                              MODE2_DRIVE_PHASE_R_QUERY);
    }
    return reverse_output(gate,
                          MODE2_DRIVE_REASON_STARTUP_QUERY,
                          MODE2_DRIVE_PHASE_R_QUERY,
                          input->pid_raw_us);
}

Mode2DriveGateOutput_t Mode2DriveGate_EvaluateControl(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    uint32_t now_tick_ms)
{
    Mode2DriveTargetDirection_t target;

    if (gate == NULL || input == NULL)
    {
        return make_output(gate,
                           MODE2_DRIVE_ACTION_NEUTRAL,
                           MODE2_DRIVE_REASON_INVALID_ARGUMENT,
                           MODE2_DRIVE_PHASE_NEUTRAL_WAIT,
                           ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL,
                           MODE2_DRIVE_BRAKE_PURPOSE_NONE,
                           1500U);
    }

    if (gate->config_valid == 0U)
    {
        return neutral_output(gate,
                              gate->config_reason,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    if (input->propulsion_authorized == 0U)
    {
        Mode2DriveGate_ResetHistory(gate);
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_NOT_AUTHORIZED,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    if (command_fresh(gate, input, now_tick_ms) == 0U)
    {
        Mode2DriveGate_ResetHistory(gate);
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_COMMAND_STALE,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    if (feedback_fresh(gate, now_tick_ms) == 0U)
    {
        Mode2DriveGate_ResetHistory(gate);
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_MOTION_UNAVAILABLE,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }
    if (finite_f32(input->target_speed_mps) == 0U)
    {
        return neutral_output(gate,
                              MODE2_DRIVE_REASON_PID_INVALID,
                              MODE2_DRIVE_PHASE_NEUTRAL_WAIT);
    }

    target = target_direction(input->target_speed_mps, &gate->config);
    (void)target_changed(gate, input->target_speed_mps);

    if (gate->action_conflict != 0U)
    {
        return conflict_output(gate);
    }

    if (gate->brake_active != 0U ||
        gate->permission == MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE)
    {
        return evaluate_brake_active(gate, input, target, now_tick_ms);
    }
    if ((gate->permission == MODE2_DRIVE_PERMISSION_F_READY ||
         gate->permission == MODE2_DRIVE_PERMISSION_R_READY) &&
        nonzero_unknown_motion(gate) != 0U &&
        startup_hold_context_matches(gate, target, now_tick_ms) != 0U)
    {
        return startup_hold_output(gate);
    }
    if (gate->permission == MODE2_DRIVE_PERMISSION_F_READY)
    {
        return evaluate_forward_ready(gate, input, target, now_tick_ms);
    }
    if (gate->permission == MODE2_DRIVE_PERMISSION_R_READY)
    {
        return evaluate_reverse_ready(gate, input, target);
    }

    return evaluate_unknown(gate, input, target, now_tick_ms);
}

static uint8_t continue_negative_session(const Mode2DriveGate_t *gate,
                                         EscTelemetryOutputPurpose_t purpose,
                                         uint16_t pwm_us)
{
    return (gate->has_output_context != 0U &&
            gate->last_purpose ==
                ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST &&
            purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE &&
            gate->last_pwm_us < gate->config.center_pwm_us &&
            pwm_us < gate->config.center_pwm_us) ? 1U : 0U;
}

static void update_session(Mode2DriveGate_t *gate,
                           EscTelemetryOutputPurpose_t purpose,
                           uint16_t pwm_us)
{
    if (gate->has_output_context == 0U)
    {
        gate->has_output_context = 1U;
    }
    else if (gate->last_purpose != purpose &&
             continue_negative_session(gate, purpose, pwm_us) == 0U)
    {
        bump_session_nonce(gate);
    }
    gate->last_purpose = purpose;
}

static void begin_brake_if_needed(Mode2DriveGate_t *gate,
                                  const Mode2DriveGateOutput_t *output,
                                  uint16_t pwm_us,
                                  uint32_t now_ms)
{
    const uint8_t was_active = gate->brake_active;
    const Mode2DriveBrakePurpose_t old_purpose = gate->brake_purpose;

    if (output->purpose != ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE ||
        pwm_us >= gate->config.center_pwm_us)
    {
        return;
    }

    if (gate->brake_active == 0U)
    {
        gate->brake_active = 1U;
        gate->brake_purpose = output->brake_purpose;
        gate->brake_confirmed = 0U;
        gate->brake_ack_pending = 1U;
        gate->brake_full_confirmed = 0U;
        gate->brake_start_ms = now_ms;
        gate->brake_session_id = gate->current_session_id;
        gate->brake_hold_us = gate->config.brake_min_us;
        gate->permission = MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE;
        gate->neutral_active = 0U;
        gate->neutral_confirmed = 0U;
    }
    else
    {
        gate->brake_purpose = output->brake_purpose;
        gate->brake_session_id = gate->current_session_id;
    }

    if (output->brake_purpose == MODE2_DRIVE_BRAKE_PURPOSE_REVERSE &&
        pwm_us <= gate->config.reverse_pwm_us)
    {
        if (gate->full_pwm_active == 0U)
        {
            gate->full_pwm_active = 1U;
            gate->brake_full_start_ms = now_ms;
            gate->brake_full_confirmed = 0U;
            clear_stop_evidence(gate);
        }
    }
    else if (output->brake_purpose == MODE2_DRIVE_BRAKE_PURPOSE_STOP &&
             (was_active == 0U || old_purpose != MODE2_DRIVE_BRAKE_PURPOSE_STOP))
    {
        gate->full_pwm_active = 0U;
        gate->brake_full_start_ms = 0U;
        gate->brake_full_confirmed = 0U;
        clear_stop_evidence(gate);
    }
    else
    {
        gate->full_pwm_active = 0U;
        gate->brake_full_start_ms = 0U;
        gate->brake_full_confirmed = 0U;
    }
}

void Mode2DriveGate_CommitApplied(Mode2DriveGate_t *gate,
                                  Mode2DriveGateOutput_t *output,
                                  uint16_t final_applied_pwm_us,
                                  uint32_t now_tick_ms)
{
    EscTelemetryOutputPurpose_t effective_purpose;

    if (gate == NULL || output == NULL)
    {
        return;
    }

    effective_purpose =
        (final_applied_pwm_us == gate->config.center_pwm_us) ?
        ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL : output->purpose;

    {
        const uint8_t old_has_output_context = gate->has_output_context;
        const uint32_t old_session = gate->current_session_id;
        const EscTelemetryOutputPurpose_t old_purpose = gate->last_purpose;
        const uint8_t preserve_brake_stop =
            (gate->action_conflict == 0U &&
             old_purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE &&
             gate->stop_evidence_valid != 0U) ? 1U : 0U;
        update_session(gate, effective_purpose, final_applied_pwm_us);
        if (effective_purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL)
        {
            if (gate->neutral_active == 0U ||
                old_purpose != ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL ||
                old_session != gate->current_session_id)
            {
                gate->neutral_start_ms = now_tick_ms;
                gate->neutral_confirmed = 0U;
                if (preserve_brake_stop == 0U)
                {
                    clear_stop_evidence(gate);
                }
            }
            gate->neutral_active = 1U;
            if (gate->brake_active != 0U)
            {
                clear_brake(gate);
            }
        }
        else
        {
            if (old_has_output_context == 0U ||
                old_purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL)
            {
                clear_stop_evidence(gate);
            }
            gate->neutral_active = 0U;
            gate->neutral_confirmed = 0U;
        }
    }
    output->purpose = effective_purpose;
    output->session_id = gate->current_session_id;
    output->final_pwm_us = final_applied_pwm_us;

    gate->last_pwm_us = final_applied_pwm_us;
    gate->last_commit_ms = now_tick_ms;

    begin_brake_if_needed(gate, output, final_applied_pwm_us, now_tick_ms);
    gate->phase = output->phase;
    output->permission = gate->permission;
    output->action_conflict = gate->action_conflict;
    output->brake_confirmed = gate->brake_confirmed;
}

uint8_t Mode2DriveGate_HasActionConflict(const Mode2DriveGate_t *gate)
{
    return (gate != NULL && gate->action_conflict != 0U) ? 1U : 0U;
}
