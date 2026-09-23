#ifndef __LONGITUDINAL_CONTROLLER_H
#define __LONGITUDINAL_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LONGITUDINAL_DIRECTION_REVERSE = -1,
    LONGITUDINAL_DIRECTION_UNKNOWN = 0,
    LONGITUDINAL_DIRECTION_FORWARD = 1
} LongitudinalDirection_t;

typedef enum
{
    LONGITUDINAL_REASON_OK = 0,
    LONGITUDINAL_REASON_INVALID_ARGUMENT,
    LONGITUDINAL_REASON_CONFIG_INVALID,
    LONGITUDINAL_REASON_NOT_ENABLED,
    LONGITUDINAL_REASON_FEEDBACK_INVALID,
    LONGITUDINAL_REASON_FEEDBACK_STALE,
    LONGITUDINAL_REASON_TIMEBASE_INVALID
} LongitudinalReason_t;

typedef struct
{
    float kp_us_per_mps;
    float ki_us_per_mps_s;
    float kd_us_per_mps2;
    float derivative_tau_s;
    float antiwindup_tau_s;
    float min_output_us;
    float max_output_us;
    uint32_t feedback_freshness_ms;
} LongitudinalControllerConfig_t;

typedef struct
{
    uint8_t valid;
    uint32_t sample_id;
    uint32_t tick_ms;
    float signed_speed_mps;
    LongitudinalDirection_t direction;
} LongitudinalFeedbackSample_t;

typedef struct
{
    uint8_t enabled;
    float target_speed_mps;
    uint32_t now_tick_ms;
} LongitudinalControllerInput_t;

typedef struct
{
    uint8_t pid_active;
    uint8_t reset_integral;
    float applied_output_us;
    float min_output_us;
    float max_output_us;
} LongitudinalAppliedOutput_t;

typedef struct
{
    uint8_t config_valid;
    uint8_t inhibited;
    uint8_t feedback_valid;
    uint8_t feedback_fresh;
    uint8_t new_sample;
    uint8_t duplicate_sample;
    uint8_t derivative_valid;
    uint8_t pid_active;
    uint8_t output_saturated;
    uint8_t integral_saturated;
    uint8_t antiwindup_active;
    uint8_t error_integral_skipped;
    uint8_t target_changed;
    uint8_t target_sign_changed;
    LongitudinalReason_t reason;

    uint32_t feedback_sample_id;
    uint32_t feedback_sample_tick_ms;
    uint32_t feedback_age_ms;

    float requested_target_mps;
    float feedback_signed_mps;
    float speed_error_mps;
    float measured_accel_mps2;
    float proportional_us;
    float integral_us;
    float derivative_us;
    float raw_output_us;
    float bounded_output_us;
    float applied_output_us;
    float min_output_us;
    float max_output_us;
} LongitudinalControllerDiagnostics_t;

typedef struct
{
    LongitudinalReason_t reason;
    LongitudinalDirection_t target_direction;
    float raw_output_us;
    float bounded_output_us;
    uint8_t valid;
    LongitudinalControllerDiagnostics_t diagnostics;
} LongitudinalControllerOutput_t;

typedef struct
{
    LongitudinalControllerConfig_t config;
    uint8_t config_valid;
    LongitudinalReason_t config_reason;

    uint8_t feedback_valid;
    uint8_t feedback_duplicate;
    uint8_t derivative_valid;
    uint32_t feedback_sample_id;
    uint32_t feedback_sample_tick_ms;
    float feedback_signed_mps;
    float previous_feedback_signed_mps;
    LongitudinalDirection_t feedback_direction;
    float measured_accel_mps2;

    uint8_t pid_sample_committed;
    uint32_t pid_committed_sample_id;
    uint32_t pid_committed_sample_tick_ms;

    uint8_t have_target;
    float last_target_speed_mps;
    LongitudinalDirection_t last_target_direction;
    uint8_t skip_error_integral_once;

    float integral_us;
    uint8_t last_integral_saturated;
    uint8_t last_antiwindup_active;
    float last_applied_output_us;

    uint8_t pending_valid;
    uint8_t pending_new_sample;
    uint8_t pending_skip_error_integral;
    float pending_dt_s;
    float pending_error_mps;
    float pending_raw_output_us;
} LongitudinalController_t;

uint8_t LongitudinalController_ConfigIsValid(
    const LongitudinalControllerConfig_t *config,
    LongitudinalReason_t *reason);
void LongitudinalController_Init(LongitudinalController_t *controller,
                                 const LongitudinalControllerConfig_t *config);
LongitudinalReason_t LongitudinalController_SetConfig(
    LongitudinalController_t *controller,
    const LongitudinalControllerConfig_t *config);
void LongitudinalController_Reset(LongitudinalController_t *controller);
void LongitudinalController_ClearIntegral(LongitudinalController_t *controller);
LongitudinalReason_t LongitudinalController_ObserveFeedback(
    LongitudinalController_t *controller,
    const LongitudinalFeedbackSample_t *sample);
LongitudinalControllerOutput_t LongitudinalController_Evaluate(
    LongitudinalController_t *controller,
    const LongitudinalControllerInput_t *input);
LongitudinalReason_t LongitudinalController_CommitApplied(
    LongitudinalController_t *controller,
    const LongitudinalAppliedOutput_t *applied);

#ifdef __cplusplus
}
#endif

#endif
