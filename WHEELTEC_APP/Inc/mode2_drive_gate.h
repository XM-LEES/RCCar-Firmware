#ifndef __MODE2_DRIVE_GATE_H
#define __MODE2_DRIVE_GATE_H

#include <stdint.h>

#include "esc_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MODE2_DRIVE_TARGET_NEUTRAL = 0,
    MODE2_DRIVE_TARGET_FORWARD = 1,
    MODE2_DRIVE_TARGET_REVERSE = -1
} Mode2DriveTargetDirection_t;

typedef enum
{
    MODE2_DRIVE_ACTION_NEUTRAL = 0,
    MODE2_DRIVE_ACTION_FORWARD,
    MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE,
    MODE2_DRIVE_ACTION_REVERSE
} Mode2DriveAction_t;

typedef enum
{
    MODE2_DRIVE_ESC_ACTION_UNKNOWN = 0,
    MODE2_DRIVE_ESC_ACTION_NEUTRAL,
    MODE2_DRIVE_ESC_ACTION_DRIVE,
    MODE2_DRIVE_ESC_ACTION_BRAKE
} Mode2DriveEscAction_t;

typedef enum
{
    MODE2_DRIVE_PERMISSION_UNKNOWN = 0,
    MODE2_DRIVE_PERMISSION_F_READY,
    MODE2_DRIVE_PERMISSION_F_BRAKE_ACTIVE,
    MODE2_DRIVE_PERMISSION_R_READY
} Mode2DrivePermission_t;

typedef enum
{
    MODE2_DRIVE_BRAKE_PURPOSE_NONE = 0,
    MODE2_DRIVE_BRAKE_PURPOSE_TRACK,
    MODE2_DRIVE_BRAKE_PURPOSE_STOP,
    MODE2_DRIVE_BRAKE_PURPOSE_REVERSE
} Mode2DriveBrakePurpose_t;

typedef enum
{
    MODE2_DRIVE_REASON_OK = 0,
    MODE2_DRIVE_REASON_INVALID_ARGUMENT,
    MODE2_DRIVE_REASON_CONFIG_INVALID,
    MODE2_DRIVE_REASON_NOT_AUTHORIZED,
    MODE2_DRIVE_REASON_MOTION_UNAVAILABLE,
    MODE2_DRIVE_REASON_COMMAND_STALE,
    MODE2_DRIVE_REASON_TARGET_NEUTRAL,
    MODE2_DRIVE_REASON_BRAKING_FORWARD,
    MODE2_DRIVE_REASON_WAITING_FOR_STOP,
    MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
    MODE2_DRIVE_REASON_FORWARD_PERMITTED,
    MODE2_DRIVE_REASON_REVERSE_PERMITTED,
    MODE2_DRIVE_REASON_FORWARD_COAST,
    MODE2_DRIVE_REASON_REVERSE_COAST,
    MODE2_DRIVE_REASON_BRAKE_REARM_REQUIRED,
    MODE2_DRIVE_REASON_BRAKE_NOT_CONFIRMED,
    MODE2_DRIVE_REASON_ACTION_CONFLICT,
    MODE2_DRIVE_REASON_DIRECTION_UNKNOWN,
    MODE2_DRIVE_REASON_STARTUP_QUERY,
    MODE2_DRIVE_REASON_STARTUP_HOLD,
    MODE2_DRIVE_REASON_PID_INVALID
} Mode2DriveReason_t;

typedef enum
{
    MODE2_DRIVE_PHASE_NEUTRAL_WAIT = 0,
    MODE2_DRIVE_PHASE_F_START,
    MODE2_DRIVE_PHASE_F_DRIVE,
    MODE2_DRIVE_PHASE_F_COAST,
    MODE2_DRIVE_PHASE_F_BRAKE_TRACK,
    MODE2_DRIVE_PHASE_F_BRAKE_STOP,
    MODE2_DRIVE_PHASE_F_BRAKE_REVERSE,
    MODE2_DRIVE_PHASE_F_RECOVER,
    MODE2_DRIVE_PHASE_R_DWELL,
    MODE2_DRIVE_PHASE_R_QUERY,
    MODE2_DRIVE_PHASE_R_START,
    MODE2_DRIVE_PHASE_R_DRIVE,
    MODE2_DRIVE_PHASE_R_COAST
} Mode2DrivePhase_t;

typedef struct
{
    uint16_t center_pwm_us;
    uint16_t forward_pwm_us;
    uint16_t reverse_pwm_us;
    uint16_t brake_min_us;
    uint32_t action_ack_ms;
    uint32_t qualify_ms;
    uint32_t neutral_dwell_ms;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    uint32_t coast_eval_ms;
    uint32_t coast_budget_ms;
    uint32_t release_delay_ms;
    float e_on_abs_mps;
    float e_on_ratio;
    float e_off_abs_mps;
    float e_off_ratio;
    float coast_progress_mps2;
    float stopped_speed_threshold_mps;
    uint8_t stopped_min_samples;
    uint32_t stopped_min_coverage_ms;
} Mode2DriveGateConfig_t;

typedef struct
{
    uint8_t command_valid;
    uint8_t propulsion_authorized;
    uint8_t pid_valid;
    uint32_t command_tick_ms;
    float target_speed_mps;
    float pid_raw_us;
    float pid_integral_us;
} Mode2DriveGateInput_t;

typedef struct
{
    uint8_t available;
    uint8_t speed_magnitude_valid;
    uint8_t direction_known;
    Mode2DriveTargetDirection_t direction;
    float speed_magnitude_mps;
    float signed_speed_mps;
    uint32_t sample_id;
    uint32_t sample_tick_ms;
    Mode2DriveEscAction_t esc_action;
    uint8_t context_valid;
    uint16_t context_pwm_us;
    EscTelemetryOutputPurpose_t context_purpose;
    uint32_t context_session_id;
    uint8_t acceleration_valid;
    float acceleration_mps2;
} Mode2DriveMotionObservation_t;

typedef struct
{
    Mode2DriveAction_t action;
    Mode2DriveReason_t reason;
    Mode2DrivePhase_t phase;
    Mode2DrivePermission_t permission;
    EscTelemetryOutputPurpose_t purpose;
    Mode2DriveBrakePurpose_t brake_purpose;
    uint32_t session_id;
    uint16_t final_pwm_us;
    uint8_t pid_active;
    uint8_t integral_reset;
    uint8_t incomplete;
    uint8_t inhibited;
    uint8_t action_conflict;
    uint8_t startup_pending;
    uint8_t brake_confirmed;
    float pid_min_us;
    float pid_max_us;
    float pid_raw_us;
    float pid_applied_us;
} Mode2DriveGateOutput_t;

typedef struct
{
    Mode2DriveGateConfig_t config;
    uint8_t config_valid;
    Mode2DriveReason_t config_reason;

    Mode2DrivePhase_t phase;
    Mode2DrivePermission_t permission;

    Mode2DriveMotionObservation_t latest_observation;
    uint8_t has_observation;
    uint32_t last_observed_sample_id;
    uint8_t has_last_observed_sample_id;

    uint8_t stop_evidence_valid;
    uint32_t stop_sample_count;
    uint32_t stop_first_tick_ms;
    uint32_t stop_last_tick_ms;
    uint32_t stop_established_tick_ms;

    uint8_t brake_active;
    Mode2DriveBrakePurpose_t brake_purpose;
    uint8_t brake_confirmed;
    uint8_t brake_ack_pending;
    uint8_t brake_full_confirmed;
    uint8_t brake_rearm_required;
    uint8_t action_conflict;
    Mode2DriveReason_t fault_reason;
    uint8_t full_pwm_active;
    uint32_t brake_start_ms;
    uint32_t brake_full_start_ms;
    uint32_t brake_session_id;
    uint16_t brake_hold_us;

    uint8_t neutral_active;
    uint8_t neutral_confirmed;
    uint32_t neutral_start_ms;

    uint8_t coast_active;
    uint32_t coast_start_ms;
    float coast_target_mps;

    uint8_t release_predict_count;
    uint32_t release_predict_sample_id;
    float release_target_mps;
    Mode2DriveBrakePurpose_t release_purpose;

    EscTelemetryOutputPurpose_t last_purpose;
    uint32_t current_session_id;
    uint8_t has_output_context;
    uint16_t last_pwm_us;
    uint32_t last_commit_ms;
    float last_target_speed_mps;
    uint8_t has_last_target;
} Mode2DriveGate_t;

uint8_t Mode2DriveGate_ConfigIsValid(const Mode2DriveGateConfig_t *config,
                                     Mode2DriveReason_t *reason);
void Mode2DriveGate_Init(Mode2DriveGate_t *gate,
                         const Mode2DriveGateConfig_t *config);
Mode2DriveReason_t Mode2DriveGate_SetConfig(Mode2DriveGate_t *gate,
                                            const Mode2DriveGateConfig_t *config);
void Mode2DriveGate_ResetHistory(Mode2DriveGate_t *gate);
void Mode2DriveGate_Observe(Mode2DriveGate_t *gate,
                            const Mode2DriveMotionObservation_t *observation,
                            uint32_t now_tick_ms);
Mode2DriveGateOutput_t Mode2DriveGate_EvaluateControl(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    uint32_t now_tick_ms);
void Mode2DriveGate_CommitApplied(Mode2DriveGate_t *gate,
                                  Mode2DriveGateOutput_t *output,
                                  uint16_t final_applied_pwm_us,
                                  uint32_t now_tick_ms);
uint8_t Mode2DriveGate_HasActionConflict(const Mode2DriveGate_t *gate);

#ifdef __cplusplus
}
#endif

#endif
