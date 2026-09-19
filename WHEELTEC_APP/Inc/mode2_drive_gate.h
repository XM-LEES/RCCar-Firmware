#ifndef __MODE2_DRIVE_GATE_H
#define __MODE2_DRIVE_GATE_H

#include <stdint.h>

#include "esc_motion_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MODE2_DRIVE_TARGET_NEUTRAL = 0,
    MODE2_DRIVE_TARGET_FORWARD,
    MODE2_DRIVE_TARGET_REVERSE
} Mode2DriveTargetDirection_t;

typedef enum
{
    MODE2_DRIVE_ACTION_NEUTRAL = 0,
    MODE2_DRIVE_ACTION_FORWARD,
    MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE,
    MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE,
    MODE2_DRIVE_ACTION_REVERSE
} Mode2DriveAction_t;

typedef enum
{
    MODE2_DRIVE_REASON_OK = 0,
    MODE2_DRIVE_REASON_INVALID_ARGUMENT,
    MODE2_DRIVE_REASON_CONFIG_INVALID,
    MODE2_DRIVE_REASON_NOT_AUTHORIZED,
    MODE2_DRIVE_REASON_MOTION_UNAVAILABLE,
    MODE2_DRIVE_REASON_TARGET_NEUTRAL,
    MODE2_DRIVE_REASON_BRAKING_FORWARD,
    MODE2_DRIVE_REASON_BRAKING_REVERSE,
    MODE2_DRIVE_REASON_WAITING_FOR_STOP,
    MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
    MODE2_DRIVE_REASON_ARMING_UNCERTAIN,
    MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID,
    MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
    MODE2_DRIVE_REASON_FORWARD_PERMITTED,
    MODE2_DRIVE_REASON_REVERSE_PERMITTED,
    MODE2_DRIVE_REASON_FORWARD_RECOVERY
} Mode2DriveReason_t;

typedef enum
{
    MODE2_DRIVE_PHASE_IDLE = 0,
    MODE2_DRIVE_PHASE_BRAKING_TO_REVERSE,
    MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_REVERSE,
    MODE2_DRIVE_PHASE_BRAKING_TO_FORWARD,
    MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_FORWARD
} Mode2DrivePhase_t;

typedef enum
{
    MODE2_DRIVE_STATE_UNKNOWN_SAFE = 0,
    MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING,
    MODE2_DRIVE_STATE_FORWARD_TRACKING,
    MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS,
    MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED,
    MODE2_DRIVE_STATE_REVERSE_ARMED,
    MODE2_DRIVE_STATE_REVERSE_TRACKING,
    MODE2_DRIVE_STATE_REVERSE_BRAKE_CONTINUOUS,
    MODE2_DRIVE_STATE_FORWARD_MAYBE_ARMED,
    MODE2_DRIVE_STATE_FORWARD_ARMED
} Mode2DriveState_t;

typedef struct
{
    float fwd_to_rev_brake_request;
    float rev_to_fwd_brake_request;
    uint32_t fwd_to_rev_brake_min_ms;
    uint32_t rev_to_fwd_brake_min_ms;
    uint32_t neutral_dwell_ms;
    uint16_t center_pwm_us;
    uint16_t fwd_to_rev_brake_full_pwm_us;
    uint16_t rev_to_fwd_brake_full_pwm_us;
    uint16_t fwd_to_rev_qualify_delta_us;
    uint16_t rev_to_fwd_qualify_delta_us;
} Mode2DriveGateConfig_t;

typedef struct
{
    Mode2DriveTargetDirection_t target_direction;
    uint8_t decel_or_stop_requested;
    uint8_t stop_requested;
    uint8_t propulsion_authorized;
    uint8_t brake_request_valid;
    uint8_t forward_recovery_authorized;
    float normalized_brake_request;
} Mode2DriveGateInput_t;

typedef struct
{
    uint8_t available;
    uint8_t stopped;
    uint8_t stop_established_valid;
    uint8_t moving_observed;
    uint32_t stop_established_tick_ms;
    uint32_t sample_tick_ms;
} Mode2DriveMotionObservation_t;

typedef struct
{
    Mode2DriveAction_t action;
    Mode2DriveReason_t reason;
    Mode2DrivePhase_t phase;
    Mode2DriveState_t state;
    uint8_t propulsion_permitted;
    uint8_t forward_permitted;
    uint8_t reverse_permitted;
    uint8_t brake_permitted;
    uint8_t opposite_direction_armed;
    uint8_t uncertain;
    float normalized_brake_request;
} Mode2DriveGateOutput_t;

typedef struct
{
    Mode2DriveGateConfig_t config;
    uint8_t config_valid;
    Mode2DriveReason_t config_reason;

    Mode2DriveState_t state;
    Mode2DriveAction_t last_applied_action;
    uint8_t has_applied_action;
    Mode2DriveTargetDirection_t last_propulsion_direction;

    uint8_t brake_threshold_active;
    uint8_t brake_sufficient;
    Mode2DriveTargetDirection_t brake_target_direction;
    uint32_t brake_session_start_ms;
    uint32_t brake_threshold_start_ms;

    uint8_t neutral_active;
    uint32_t neutral_start_ms;
    uint8_t armed_stop_valid;
    uint32_t armed_stop_tick_ms;

    uint8_t expect_neutral_to_arm;
    Mode2DriveTargetDirection_t expected_arm_direction;
    uint32_t expected_arm_stop_tick_ms;

    uint8_t forward_recovery_active;
    uint32_t forward_recovery_start_ms;
} Mode2DriveGate_t;

uint8_t Mode2DriveGate_ConfigIsValid(const Mode2DriveGateConfig_t *config,
                                     Mode2DriveReason_t *reason);
void Mode2DriveGate_Init(Mode2DriveGate_t *gate,
                         const Mode2DriveGateConfig_t *config);
Mode2DriveReason_t Mode2DriveGate_SetConfig(Mode2DriveGate_t *gate,
                                            const Mode2DriveGateConfig_t *config);
Mode2DriveGateOutput_t Mode2DriveGate_EvaluateWithObservation(
    Mode2DriveGate_t *gate,
    const Mode2DriveGateInput_t *input,
    const Mode2DriveMotionObservation_t *observation,
    uint32_t now_tick_ms);
Mode2DriveGateOutput_t Mode2DriveGate_Evaluate(Mode2DriveGate_t *gate,
                                               const Mode2DriveGateInput_t *input,
                                               const EscMotionEstimate_t *motion,
                                               uint32_t now_tick_ms);
void Mode2DriveGate_CommitAppliedActionWithPwmEvidence(
    Mode2DriveGate_t *gate,
    Mode2DriveAction_t action,
    uint32_t now_tick_ms,
    uint16_t final_applied_pwm_us);
void Mode2DriveGate_InvalidateAppliedHistory(Mode2DriveGate_t *gate);
EscMotionAppliedAction_t Mode2DriveGate_ToMotionAction(Mode2DriveAction_t action);

#ifdef __cplusplus
}
#endif

#endif
