/**
 * @file servo_basic_control.h
 * @brief Protocol adapter for servo/ESC control with RC override and Orin Ackermann mapping.
 *
 * The module clamps inputs, supports RC takeover, and maps Orin Ackermann
 * speed/steering commands into ESC/servo PWM.
 */

#ifndef SERVO_BASIC_CONTROL_H
#define SERVO_BASIC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ESC_PWM_MIN_PULSE_US        1000U
#define ESC_PWM_NEUTRAL_PULSE_US    1500U
#define ESC_PWM_MAX_PULSE_US        2000U

#define SERVO_MIN_PULSE_US          ESC_PWM_MIN_PULSE_US
#define SERVO_MAX_PULSE_US          2000U

typedef enum
{
	SERVO_CTRL_MODE_RC_PASSTHROUGH = 0U,
	SERVO_CTRL_MODE_AUTONOMOUS     = 1U,
} servo_control_mode_t;

typedef enum
{
	SERVO_ESC_ACTION_UNKNOWN = 0U,
	SERVO_ESC_ACTION_NEUTRAL = 1U,
	SERVO_ESC_ACTION_DRIVE = 2U,
	SERVO_ESC_ACTION_BRAKE = 3U,
} servo_esc_action_t;

typedef struct
{
	uint16_t esc_pulse_us;
	uint16_t servo_pulse_us;
	servo_control_mode_t control_mode;
	uint8_t rc_takeover_pending;
} servo_basic_state_t;

typedef struct
{
	uint8_t speed_saturated;
	uint8_t steering_saturated;
	uint8_t accel_limited;
	uint8_t steering_rate_limited;
	uint8_t steering_fault;
	uint8_t esc_feedback_valid;
	uint8_t esc_stop_confirmed;
	uint8_t esc_motion_config_valid;
	uint8_t esc_fe32_fresh;
	uint8_t esc_rpm_raw_valid;
	uint8_t esc_speed_magnitude_valid;
	uint8_t esc_speed_calibration_valid;
	uint8_t esc_action;
	uint8_t vehicle_direction_known;
	uint8_t esc_soft_uart_rx_error;
	uint8_t mode2_config_valid;
	uint8_t auto_propulsion_authorized;
	uint8_t closed_loop_active;
	uint8_t tracking_brake_active;
	uint8_t mode2_opposite_armed;
	uint8_t mode2_state_ambiguous;
	uint8_t mode2_control_inhibited;
	uint8_t mode2_state;
	uint8_t mode2_reason;
	uint8_t longitudinal_intent;
	uint8_t longitudinal_reason;
	float longitudinal_slewed_target_mps;
	uint8_t esc_sample_stale;
	uint8_t esc_rx_invalidated;
	int8_t esc_feedback_direction;
	uint16_t esc_rpm_raw;
	uint32_t esc_sample_id;
	uint32_t esc_receive_epoch;
	uint32_t esc_valid_frame_count;
	uint32_t esc_soft_uart_error_count;
	uint32_t esc_soft_uart_error_flags;
} servo_basic_diagnostics_t;

typedef struct
{
	servo_basic_state_t state;
	servo_basic_diagnostics_t diagnostics;
	float speed_mps;
	float esc_uplink_speed_mps;
	float esc_speed_magnitude_mps;
	float steering_angle_rad;
	float yaw_rate_rad_s;
	uint8_t signed_speed_valid;
	uint8_t esc_uplink_speed_valid;
	uint8_t esc_direction_known;
	uint8_t rc_override_active;
	uint8_t orin_command_timeout;
	uint8_t orin_auto_enabled;
	uint8_t orin_brake_active;
	uint8_t orin_emergency_active;
	uint32_t control_tick_ms;
	uint32_t publish_sequence;
	uint32_t esc_sample_id;
	uint32_t esc_receive_epoch;
	uint32_t esc_sample_tick_ms;
} servo_basic_control_snapshot_t;

void ServoBasic_Init(void);
void ServoBasic_ProcessControl(void);
uint8_t ServoBasic_GetControlSnapshot(servo_basic_control_snapshot_t *snapshot);
uint8_t ServoBasic_IsRcOverrideActive(void);
uint8_t ServoBasic_IsOrinCommandTimeout(void);
uint8_t ServoBasic_IsOrinAutoEnabled(void);
uint8_t ServoBasic_IsOrinBrakeActive(void);
uint8_t ServoBasic_IsOrinEmergencyActive(void);
uint8_t ServoBasic_GetAckermannFeedback(float *speed_mps,
                                        float *steering_angle_rad,
                                        float *yaw_rate_rad_s);
servo_basic_diagnostics_t ServoBasic_GetDiagnostics(void);

void ServoBasic_OutputEscPulse(uint16_t pulse_us);
void ServoBasic_OutputServoPulse(uint16_t pulse_us);
void ServoBasic_UpdateAckermannFromOrin(float speed_mps,
                                        float steering_angle_rad,
                                        uint8_t enable,
                                        uint8_t brake,
                                        uint8_t emergency_stop);
void ServoBasic_SetSpeedPiEnable(uint8_t enable);
void ServoBasic_SetSpeedPiKp(float kp_us_per_mps);
void ServoBasic_SetSpeedPiKi(float ki_us_per_mps_s);
void ServoBasic_SetSpeedPiTrimLimitUs(uint32_t limit_us);
void ServoBasic_ResetSpeedPi(void);

#ifdef __cplusplus
}
#endif

#endif
