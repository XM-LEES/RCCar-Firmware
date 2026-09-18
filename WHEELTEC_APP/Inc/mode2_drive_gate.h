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
    MODE2_DRIVE_ACTION_BRAKE,
    MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE,
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
    MODE2_DRIVE_REASON_COASTING_REVERSE,
    MODE2_DRIVE_REASON_WAITING_FOR_STOP,
    MODE2_DRIVE_REASON_WAITING_FOR_NEUTRAL_DWELL,
    MODE2_DRIVE_REASON_FIRST_STRIKE_REQUIRED,
    MODE2_DRIVE_REASON_FIRST_STRIKE_HOLD,
    MODE2_DRIVE_REASON_FIRST_STRIKE_COMPLETE,
    MODE2_DRIVE_REASON_FIRST_STRIKE_UNCERTAIN,
    MODE2_DRIVE_REASON_BRAKE_REQUEST_INVALID,
    MODE2_DRIVE_REASON_ESC_STATE_UNKNOWN,
    MODE2_DRIVE_REASON_REVERSAL_TIMEOUT,
    MODE2_DRIVE_REASON_FORWARD_PERMITTED,
    MODE2_DRIVE_REASON_REVERSE_PERMITTED
} Mode2DriveReason_t;

typedef enum
{
    MODE2_DRIVE_PHASE_IDLE = 0,
    MODE2_DRIVE_PHASE_BRAKING_TO_REVERSE,
    MODE2_DRIVE_PHASE_FIRST_STRIKE,
    MODE2_DRIVE_PHASE_NEUTRAL_DWELL_TO_REVERSE,
    MODE2_DRIVE_PHASE_WAIT_STOP_TO_FORWARD,
    MODE2_DRIVE_PHASE_FAULT
} Mode2DrivePhase_t;

typedef struct
{
    uint8_t calibration_valid;
    uint8_t brake_calibration_valid;
    uint8_t first_strike_calibration_valid;
    uint8_t neutral_dwell_valid;
    uint8_t reversal_timeout_valid;

    float brake_request;
    float reverse_first_strike_request;
    uint32_t reverse_first_strike_min_ms;
    uint32_t neutral_dwell_ms;
    uint32_t reversal_timeout_ms;
} Mode2DriveGateConfig_t;

typedef struct
{
    Mode2DriveTargetDirection_t target_direction;
    uint8_t decel_or_stop_requested;
    uint8_t stop_requested;
    uint8_t propulsion_authorized;
    uint8_t brake_request_valid;
    float normalized_brake_request;
} Mode2DriveGateInput_t;

typedef struct
{
    Mode2DriveAction_t action;
    Mode2DriveReason_t reason;
    Mode2DrivePhase_t phase;
    uint8_t propulsion_permitted;
    uint8_t forward_permitted;
    uint8_t reverse_permitted;
    uint8_t brake_permitted;
    uint8_t fault_latched;
    float normalized_brake_request;
} Mode2DriveGateOutput_t;

typedef enum
{
    MODE2_DRIVE_TRANSITION_NONE = 0,
    MODE2_DRIVE_TRANSITION_TO_REVERSE,
    MODE2_DRIVE_TRANSITION_TO_FORWARD
} Mode2DriveTransition_t;

typedef struct
{
    Mode2DriveGateConfig_t config;
    uint8_t config_valid;
    Mode2DriveReason_t config_reason;

    Mode2DriveAction_t last_applied_action;
    uint8_t has_applied_action;
    Mode2DriveTargetDirection_t last_propulsion_direction;
    uint8_t forward_propulsion_available;

    uint8_t reverse_unlocked_by_forward_brake;
    uint8_t forward_brake_session_active;
    uint8_t forward_brake_sufficient_active;
    uint32_t forward_brake_sufficient_start_ms;
    uint8_t first_strike_active;
    uint8_t first_strike_sufficient;
    uint8_t first_strike_uncertain;
    uint32_t first_strike_start_ms;
    uint32_t first_strike_uncertain_since_ms;
    uint8_t neutral_active;
    uint32_t neutral_start_ms;

    uint8_t expect_forward_brake_unlock;
    uint8_t expect_first_strike;

    uint8_t transition_active;
    Mode2DriveTransition_t transition;
    uint32_t transition_start_ms;

    uint8_t esc_state_unknown;
    uint8_t fault_latched;
    Mode2DriveReason_t fault_reason;
} Mode2DriveGate_t;

uint8_t Mode2DriveGate_ConfigIsValid(const Mode2DriveGateConfig_t *config,
                                     Mode2DriveReason_t *reason);
void Mode2DriveGate_Init(Mode2DriveGate_t *gate,
                         const Mode2DriveGateConfig_t *config);
Mode2DriveReason_t Mode2DriveGate_SetConfig(Mode2DriveGate_t *gate,
                                            const Mode2DriveGateConfig_t *config);
Mode2DriveGateOutput_t Mode2DriveGate_Evaluate(Mode2DriveGate_t *gate,
                                               const Mode2DriveGateInput_t *input,
                                               const EscMotionEstimate_t *motion,
                                               uint32_t now_tick_ms);
/* Call after the requested action has really been applied. For BRAKE and
 * REVERSE_FIRST_STRIKE, applied_normalized_brake must be the actual normalized
 * output after all limiting/override logic, not the requested value.
 */
void Mode2DriveGate_CommitAppliedActionWithEvidence(Mode2DriveGate_t *gate,
                                                    Mode2DriveAction_t action,
                                                    uint32_t now_tick_ms,
                                                    float applied_normalized_brake);
void Mode2DriveGate_InvalidateAppliedHistory(Mode2DriveGate_t *gate);
EscMotionAppliedAction_t Mode2DriveGate_ToMotionAction(Mode2DriveAction_t action);

#ifdef __cplusplus
}
#endif

#endif
