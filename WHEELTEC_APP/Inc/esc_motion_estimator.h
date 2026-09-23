#ifndef __ESC_MOTION_ESTIMATOR_H
#define __ESC_MOTION_ESTIMATOR_H

#include <stdint.h>

#include "esc_fe32_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ESC_MOTION_DIRECTION_UNKNOWN = 0,
    ESC_MOTION_DIRECTION_FORWARD = 1,
    ESC_MOTION_DIRECTION_REVERSE = -1
} EscMotionDirection_t;

typedef enum
{
    ESC_MOTION_APPLIED_ACTION_NEUTRAL = 0,
    ESC_MOTION_APPLIED_ACTION_FORWARD,
    ESC_MOTION_APPLIED_ACTION_BRAKE,
    ESC_MOTION_APPLIED_ACTION_REVERSE,
    ESC_MOTION_APPLIED_ACTION_EXTERNAL_OVERRIDE
} EscMotionAppliedAction_t;

typedef enum
{
    ESC_MOTION_REASON_OK = 0,
    ESC_MOTION_REASON_INVALID_ARGUMENT,
    ESC_MOTION_REASON_CONFIG_INVALID,
    ESC_MOTION_REASON_NO_SAMPLE,
    ESC_MOTION_REASON_SAMPLE_STALE,
    ESC_MOTION_REASON_RPM_INVALID,
    ESC_MOTION_REASON_SAMPLE_OUT_OF_ORDER,
    ESC_MOTION_REASON_CALCULATION_INVALID,
    ESC_MOTION_REASON_DIRECTION_UNKNOWN
} EscMotionReason_t;

typedef struct
{
    float wheel_rpm_per_raw;
    float wheel_radius_m;
    uint32_t telemetry_timeout_ms;
    float stopped_speed_threshold_mps;
    uint8_t stopped_min_samples;
    uint32_t stopped_min_coverage_ms;
} EscMotionEstimatorConfig_t;

typedef struct
{
    uint8_t config_valid;
    uint8_t magnitude_config_valid;
    uint8_t stop_config_valid;
    uint8_t has_sample;
    uint8_t sample_fresh;
    uint8_t rpm_valid;
    uint8_t magnitude_valid;
    uint8_t stop_valid;
    uint8_t direction_valid;
    uint8_t signed_speed_valid;
    uint8_t stopped;
    uint8_t moving_observed;

    EscMotionDirection_t direction;
    EscMotionReason_t reason;
    uint32_t last_sample_id;
    uint32_t last_sample_tick_ms;
    uint32_t sample_age_ms;
    uint32_t stop_sample_count;
    uint32_t stop_coverage_ms;
    uint32_t stop_evidence_start_tick_ms;
    uint32_t stop_established_tick_ms;
    float speed_magnitude_mps;
    float signed_speed_mps;
} EscMotionEstimate_t;

typedef struct
{
    EscMotionEstimatorConfig_t config;
    uint8_t config_valid;
    uint8_t stop_config_valid;
    EscMotionReason_t config_reason;
    EscMotionReason_t stop_config_reason;
    float meters_per_raw;

    uint8_t has_sample;
    uint32_t last_sample_id;
    uint32_t last_sample_tick_ms;
    uint8_t last_rpm_valid;
    float last_speed_magnitude_mps;

    EscMotionDirection_t direction;

    uint8_t stop_evidence_valid;
    uint32_t stop_sample_count;
    uint32_t stop_first_tick_ms;
    uint32_t stop_last_tick_ms;
    uint32_t stop_established_tick_ms;
    uint8_t stop_epoch_valid;
    uint32_t stop_epoch_tick_ms;

    uint8_t has_applied_action;
    EscMotionAppliedAction_t last_applied_action;
} EscMotionEstimator_t;

uint8_t EscMotionEstimator_ConfigIsValid(const EscMotionEstimatorConfig_t *config,
                                         EscMotionReason_t *reason);
uint8_t EscMotionEstimator_StopConfigIsValid(const EscMotionEstimatorConfig_t *config,
                                             EscMotionReason_t *reason);
void EscMotionEstimator_Init(EscMotionEstimator_t *estimator,
                             const EscMotionEstimatorConfig_t *config);
EscMotionReason_t EscMotionEstimator_SetConfig(EscMotionEstimator_t *estimator,
                                               const EscMotionEstimatorConfig_t *config);
EscMotionReason_t EscMotionEstimator_ObserveSample(EscMotionEstimator_t *estimator,
                                                   const EscFe32Sample_t *sample,
                                                   uint32_t now_tick_ms);
EscMotionEstimate_t EscMotionEstimator_GetEstimate(const EscMotionEstimator_t *estimator,
                                                   uint32_t now_tick_ms);
/* Call after every real output application. The estimator uses applied_tick_ms
 * to reject older stop evidence; continuous applications share the first tick
 * of that action until the applied action changes. A BRAKE->NEUTRAL edge also
 * requires new stop evidence after neutral.
 */
void EscMotionEstimator_CommitAppliedActionAt(EscMotionEstimator_t *estimator,
                                              EscMotionAppliedAction_t action,
                                              uint32_t applied_tick_ms);
/* Cache clearing only: these helpers do not establish a receive-time fence.
 * At an authority, configuration, or RX-loss boundary, use CommitAppliedActionAt
 * with EXTERNAL_OVERRIDE and the actual boundary tick to reject older samples. */
void EscMotionEstimator_InvalidateDirectionAndStop(EscMotionEstimator_t *estimator);
void EscMotionEstimator_InvalidateStopEvidence(EscMotionEstimator_t *estimator);

#ifdef __cplusplus
}
#endif

#endif
