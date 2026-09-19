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
    LONGITUDINAL_INTENT_NEUTRAL = 0,
    LONGITUDINAL_INTENT_DRIVE,
    LONGITUDINAL_INTENT_TRACKING_BRAKE,
    LONGITUDINAL_INTENT_STOP_BRAKE,
    LONGITUDINAL_INTENT_REVERSAL_REQUEST
} LongitudinalIntent_t;

typedef enum
{
    LONGITUDINAL_REASON_OK = 0,
    LONGITUDINAL_REASON_INVALID_ARGUMENT,
    LONGITUDINAL_REASON_CONFIG_INVALID,
    LONGITUDINAL_REASON_NOT_ENABLED,
    LONGITUDINAL_REASON_FEEDBACK_INVALID,
    LONGITUDINAL_REASON_TARGET_NEUTRAL,
    LONGITUDINAL_REASON_STOP_REQUESTED,
    LONGITUDINAL_REASON_TRACKING_BRAKE,
    LONGITUDINAL_REASON_REVERSAL_REQUIRED
} LongitudinalReason_t;

typedef struct
{
    uint8_t config_valid;

    uint16_t center_pwm_us;
    uint16_t min_pwm_us;
    uint16_t max_pwm_us;
    uint16_t forward_limit_pwm_us;
    uint16_t reverse_limit_pwm_us;

    float min_control_speed_mps;
    float forward_speed_cap_mps;
    float reverse_speed_cap_mps;
    float target_slew_rate_mps2;

    uint8_t pi_enabled;
    float pi_kp_us_per_mps;
    float pi_ki_us_per_mps_s;
    uint16_t pi_trim_limit_us;

    uint8_t tracking_brake_enabled;
    float tracking_brake_kp;
    float tracking_brake_max;
    float tracking_brake_enter_error_mps;
    float tracking_brake_release_error_mps;

    float stop_brake_request;
} LongitudinalControllerConfig_t;

typedef struct
{
    uint8_t automatic_enabled;
    uint8_t stop_requested;
    float target_speed_mps;

    uint8_t feedback_valid;
    uint8_t stopped;
    LongitudinalDirection_t current_direction;
    float speed_magnitude_mps;
    uint32_t feedback_sample_id;
    uint32_t feedback_sample_tick_ms;
    uint32_t now_tick_ms;
} LongitudinalControllerInput_t;

typedef struct
{
    uint8_t config_valid;
    uint8_t inhibited;
    uint8_t feedback_valid;
    uint8_t target_limited;
    uint8_t slew_limited;
    uint8_t duplicate_sample;
    uint8_t pi_active;
    uint8_t pi_saturated;
    uint8_t tracking_brake_active;
    LongitudinalReason_t reason;

    float requested_target_mps;
    float limited_target_mps;
    float slewed_target_mps;
    float feedback_signed_mps;
    float speed_error_mps;
    float tracking_error_mps;
    float pi_integral_mps_s;
    float pi_trim_us;
} LongitudinalControllerDiagnostics_t;

typedef struct
{
    LongitudinalIntent_t intent;
    LongitudinalReason_t reason;
    LongitudinalDirection_t target_direction;
    uint16_t drive_pwm_us;
    float drive_effort;
    float normalized_brake_request;
    uint8_t propulsion_permitted;
    uint8_t brake_permitted;
    LongitudinalControllerDiagnostics_t diagnostics;
} LongitudinalControllerOutput_t;

typedef struct
{
    LongitudinalControllerConfig_t config;
    uint8_t config_valid;
    LongitudinalReason_t config_reason;

    uint8_t has_update_tick;
    uint32_t last_update_tick_ms;
    float slewed_target_mps;

    uint8_t have_feedback_sample;
    uint32_t last_feedback_sample_id;
    uint32_t last_feedback_sample_tick_ms;
    float pi_integral_mps_s;

    uint8_t tracking_brake_active;
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
LongitudinalControllerOutput_t LongitudinalController_Evaluate(
    LongitudinalController_t *controller,
    const LongitudinalControllerInput_t *input);

#ifdef __cplusplus
}
#endif

#endif
