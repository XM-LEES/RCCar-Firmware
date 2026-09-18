/**
 * @file servo_basic_control.c
 * @brief Protocol adapter for servo/ESC control commands.
 */

#include "servo_basic_control.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_vehicle_config.h"
#include "esc_motion_estimator.h"
#include "esc_telemetry.h"
#include "hall_speed.h"
#include "mode2_drive_gate.h"
#include "servo_rc_capture.h"

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#endif

#ifndef SERVO_BASIC_LOG
#define SERVO_BASIC_LOG(...) ((void)0)
#endif

#if defined(STM32F407xx)
typedef uint32_t ServoBasicIrqState_t;

static ServoBasicIrqState_t servo_basic_enter_critical(void)
{
	ServoBasicIrqState_t state = __get_PRIMASK();

	__disable_irq();
	return state;
}

static void servo_basic_exit_critical(ServoBasicIrqState_t state)
{
	__set_PRIMASK(state);
}
#else
typedef uint8_t ServoBasicIrqState_t;

static ServoBasicIrqState_t servo_basic_enter_critical(void)
{
	return 0U;
}

static void servo_basic_exit_critical(ServoBasicIrqState_t state)
{
	(void)state;
}
#endif

static servo_basic_state_t g_state = {
	ESC_PWM_NEUTRAL_PULSE_US,
	ESC_PWM_NEUTRAL_PULSE_US,
	SERVO_CTRL_MODE_AUTONOMOUS,
	0U,
	0U
};
static servo_basic_control_snapshot_t s_control_snapshot;
static uint32_t s_control_snapshot_next_sequence = 0U;

#define RC_OVERRIDE_CENTER_DEFAULT_US            APP_RC_OVERRIDE_CENTER_US
#define RC_OVERRIDE_ENTER_THRESHOLD_DEFAULT_US   APP_RC_OVERRIDE_ENTER_THRESHOLD_US
#define RC_OVERRIDE_EXIT_THRESHOLD_DEFAULT_US    APP_RC_OVERRIDE_EXIT_THRESHOLD_US
#define RC_OVERRIDE_ENTER_SAMPLES_DEFAULT        APP_RC_OVERRIDE_ENTER_SAMPLES
#define RC_OVERRIDE_RELEASE_HOLD_DEFAULT_MS      APP_RC_OVERRIDE_RELEASE_HOLD_MS
#define RC_GUARD_ACTIVE_LOW_THRESHOLD_DEFAULT_US APP_RC_GUARD_ACTIVE_LOW_THRESHOLD_US
#define RC_GUARD_ACTIVE_HIGH_THRESHOLD_DEFAULT_US APP_RC_GUARD_ACTIVE_HIGH_THRESHOLD_US
#define ORIN_ACKERMANN_WHEELBASE_DEFAULT_MM      APP_ORIN_ACKERMANN_WHEELBASE_MM
#define ORIN_ACKERMANN_TRACK_WIDTH_DEFAULT_MM    APP_ORIN_ACKERMANN_TRACK_WIDTH_MM
#define ORIN_ACKERMANN_WHEEL_RADIUS_DEFAULT_MM   APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM
#define ORIN_ACKERMANN_MAX_STEERING_DEFAULT_MRAD APP_ORIN_ACKERMANN_MAX_STEERING_MRAD
#define ORIN_ACKERMANN_MIN_VX_DEFAULT_MMPS       APP_ORIN_ACKERMANN_MIN_VX_MMPS
#define ORIN_VX_SCALE_DEFAULT_PERMILLE           APP_ORIN_VX_SCALE_PERMILLE
#define ORIN_VX_FORWARD_CAP_DEFAULT_MMPS         APP_ORIN_VX_FORWARD_CAP_MMPS
#define ORIN_VX_REVERSE_CAP_DEFAULT_MMPS         APP_ORIN_VX_REVERSE_CAP_MMPS
#define ORIN_VX_DEADBAND_DEFAULT_MMPS            APP_ORIN_VX_DEADBAND_MMPS
#define ORIN_ESC_FORWARD_START_DEFAULT_US        APP_ORIN_ESC_FORWARD_START_US
#define ORIN_ESC_REVERSE_START_DEFAULT_US        APP_ORIN_ESC_REVERSE_START_US
#define ORIN_ESC_FORWARD_MAX_DEFAULT_US          APP_ORIN_ESC_FORWARD_MAX_US
#define ORIN_ESC_REVERSE_MAX_DEFAULT_US          APP_ORIN_ESC_REVERSE_MAX_US
#define ORIN_SERVO_RANGE_DEFAULT_US              APP_ORIN_SERVO_RANGE_US
#define ORIN_MIN_FORWARD_VX_DEFAULT_MMPS         APP_ORIN_MIN_FORWARD_VX_MMPS
#define ORIN_MIN_REVERSE_VX_DEFAULT_MMPS         APP_ORIN_MIN_REVERSE_VX_MMPS
#define ESC_SPEED_LIMIT_DEFAULT_MMPS             APP_ESC_SPEED_LIMIT_MMPS
#define ESC_SPEED_LIMIT_RELEASE_DEFAULT_MMPS     APP_ESC_SPEED_LIMIT_RELEASE_MMPS
#define ESC_SPEED_LIMIT_CONFIRM_SAMPLES          APP_ESC_SPEED_LIMIT_CONFIRM_SAMPLES
#define ORIN_ACCEL_LIMIT_DEFAULT_MMPS2           APP_ORIN_ACCEL_LIMIT_MMPS2
#define ORIN_STEERING_RATE_LIMIT_DEFAULT_MRADPS  APP_ORIN_STEERING_RATE_LIMIT_MRADPS
#define SPEED_PI_ENABLE_DEFAULT                  APP_SPEED_PI_ENABLE_DEFAULT
#define SPEED_PI_KP_DEFAULT_US_PER_MPS           APP_SPEED_PI_KP_DEFAULT_US_PER_MPS
#define SPEED_PI_KI_DEFAULT_US_PER_MPS_S         APP_SPEED_PI_KI_DEFAULT_US_PER_MPS_S
#define SPEED_PI_TRIM_LIMIT_DEFAULT_US           APP_SPEED_PI_TRIM_LIMIT_US
#define SPEED_PI_KP_MAX_US_PER_MPS               120.0f
#define SPEED_PI_KI_MAX_US_PER_MPS_S             40.0f
#define SPEED_PI_TRIM_LIMIT_PARAM_MAX_US         60U
#define SERVO_BASIC_PI_F                         3.14159265358979f
#define RC_VALID_MIN_DEFAULT_US                  APP_RC_VALID_MIN_US
#define RC_VALID_MAX_DEFAULT_US                  APP_RC_VALID_MAX_US
#define RC_FRAME_MIN_DEFAULT_US                  APP_RC_FRAME_MIN_US
#define RC_FRAME_MAX_DEFAULT_US                  APP_RC_FRAME_MAX_US
#define RC_GLITCH_FREEZE_DEFAULT_MS              APP_RC_GLITCH_FREEZE_MS
#define RC_THROTTLE_NEUTRAL_HOLD_DEFAULT_US      APP_RC_THROTTLE_NEUTRAL_HOLD_US
#define RC_THROTTLE_JUMP_CONFIRM_DEFAULT_US      APP_RC_THROTTLE_JUMP_CONFIRM_US
#define RC_STEERING_JUMP_CONFIRM_DEFAULT_US      APP_RC_STEERING_JUMP_CONFIRM_US
#define RC_JUMP_CONFIRM_SAMPLES_DEFAULT          APP_RC_JUMP_CONFIRM_SAMPLES

// Debug trigger variables (set from Keil Watch/Command).
volatile uint32_t g_debug_servo_trigger = 0U;
volatile uint32_t g_debug_servo_cmd = SERVO_CMD_SET_SERVO_ANGLE;
volatile uint32_t g_debug_servo_value = 90U;

// RC raw PWM follow (bypass 1000-2000 us clamp when non-zero).
volatile uint32_t g_rc_pwm_follow_raw = APP_RC_PWM_FOLLOW_RAW_DEFAULT;
// RC signal timeout in milliseconds (0 uses default 100 ms).
volatile uint32_t g_rc_signal_timeout_ms = APP_RC_SIGNAL_TIMEOUT_MS;
// Orin kinematics to PWM settings (set from Keil Watch).
volatile uint32_t g_orin_pwm_enable = APP_ORIN_PWM_ENABLE_DEFAULT;
volatile uint32_t g_orin_pwm_timeout_ms = APP_ORIN_PWM_TIMEOUT_DEFAULT_MS;
volatile uint32_t g_orin_ackermann_wheelbase_mm = ORIN_ACKERMANN_WHEELBASE_DEFAULT_MM;
volatile uint32_t g_orin_ackermann_track_width_mm = ORIN_ACKERMANN_TRACK_WIDTH_DEFAULT_MM;
volatile uint32_t g_orin_ackermann_wheel_radius_mm = ORIN_ACKERMANN_WHEEL_RADIUS_DEFAULT_MM;
volatile uint32_t g_orin_ackermann_max_steering_millirad = ORIN_ACKERMANN_MAX_STEERING_DEFAULT_MRAD;
volatile uint32_t g_orin_ackermann_min_vx_mmps = ORIN_ACKERMANN_MIN_VX_DEFAULT_MMPS;
volatile uint32_t g_orin_vx_scale = ORIN_VX_SCALE_DEFAULT_PERMILLE;
volatile uint32_t g_orin_vx_forward_cap_mmps = ORIN_VX_FORWARD_CAP_DEFAULT_MMPS;
volatile uint32_t g_orin_vx_reverse_cap_mmps = ORIN_VX_REVERSE_CAP_DEFAULT_MMPS;
volatile uint32_t g_orin_vx_deadband_mmps = ORIN_VX_DEADBAND_DEFAULT_MMPS;
volatile uint32_t g_orin_vx_max_mmps = APP_ORIN_VX_MAX_DEFAULT_MMPS;
volatile uint32_t g_orin_esc_center_us = APP_ORIN_ESC_CENTER_US;
volatile uint32_t g_orin_esc_range_us = APP_ORIN_ESC_RANGE_US;
volatile uint32_t g_orin_esc_forward_start_us = ORIN_ESC_FORWARD_START_DEFAULT_US;
volatile uint32_t g_orin_esc_reverse_start_us = ORIN_ESC_REVERSE_START_DEFAULT_US;
volatile uint32_t g_orin_esc_forward_max_us = ORIN_ESC_FORWARD_MAX_DEFAULT_US;
volatile uint32_t g_orin_esc_reverse_max_us = ORIN_ESC_REVERSE_MAX_DEFAULT_US;
volatile uint32_t g_orin_servo_center_us = APP_ORIN_SERVO_CENTER_US;
volatile uint32_t g_orin_servo_range_us = ORIN_SERVO_RANGE_DEFAULT_US;
volatile uint32_t g_esc_speed_limit_mmps = ESC_SPEED_LIMIT_DEFAULT_MMPS;
volatile uint32_t g_esc_speed_limit_release_mmps = ESC_SPEED_LIMIT_RELEASE_DEFAULT_MMPS;
volatile uint32_t g_esc_speed_limit_active = 0U;
volatile uint32_t g_esc_tracking_brake_valid = APP_ESC_TRACKING_BRAKE_VALID_DEFAULT;
volatile float g_esc_tracking_brake_kp = APP_ESC_TRACKING_BRAKE_KP_DEFAULT;
volatile float g_esc_tracking_brake_max = APP_ESC_TRACKING_BRAKE_MAX_DEFAULT;
volatile float g_esc_tracking_brake_enter_error_mps =
	APP_ESC_TRACKING_BRAKE_ENTER_ERROR_MPS_DEFAULT;
volatile float g_esc_tracking_brake_release_error_mps =
	APP_ESC_TRACKING_BRAKE_RELEASE_ERROR_MPS_DEFAULT;
volatile uint32_t g_esc_motion_calibration_valid = APP_ESC_MOTION_CALIBRATION_VALID_DEFAULT;
volatile uint32_t g_esc_motion_pole_pairs_valid = APP_ESC_MOTION_POLE_PAIRS_VALID_DEFAULT;
volatile uint32_t g_esc_motion_gear_ratio_valid = APP_ESC_MOTION_GEAR_RATIO_VALID_DEFAULT;
volatile uint32_t g_esc_motion_wheel_ratio_valid = APP_ESC_MOTION_WHEEL_RATIO_VALID_DEFAULT;
volatile uint32_t g_esc_motion_wheel_circumference_valid = APP_ESC_MOTION_WHEEL_CIRCUMFERENCE_VALID_DEFAULT;
volatile uint32_t g_esc_motion_telemetry_timeout_valid = APP_ESC_MOTION_TELEMETRY_TIMEOUT_VALID_DEFAULT;
volatile uint32_t g_esc_motion_stopped_threshold_valid = APP_ESC_MOTION_STOPPED_THRESHOLD_VALID_DEFAULT;
volatile uint32_t g_esc_motion_stopped_samples_valid = APP_ESC_MOTION_STOPPED_SAMPLES_VALID_DEFAULT;
volatile uint32_t g_esc_motion_stopped_coverage_valid = APP_ESC_MOTION_STOPPED_COVERAGE_VALID_DEFAULT;
volatile uint32_t g_esc_motion_motor_pole_pairs = APP_ESC_MOTION_MOTOR_POLE_PAIRS_DEFAULT;
volatile float g_esc_motion_gear_ratio = APP_ESC_MOTION_GEAR_RATIO_DEFAULT;
volatile float g_esc_motion_wheel_ratio = APP_ESC_MOTION_WHEEL_RATIO_DEFAULT;
volatile float g_esc_motion_wheel_circumference_m = APP_ESC_MOTION_WHEEL_CIRCUMFERENCE_M_DEFAULT;
volatile uint32_t g_esc_motion_telemetry_timeout_ms = APP_ESC_MOTION_TELEMETRY_TIMEOUT_MS_DEFAULT;
volatile float g_esc_motion_stopped_speed_threshold_mps = APP_ESC_MOTION_STOPPED_THRESHOLD_MPS_DEFAULT;
volatile uint32_t g_esc_motion_stopped_min_samples = APP_ESC_MOTION_STOPPED_MIN_SAMPLES_DEFAULT;
volatile uint32_t g_esc_motion_stopped_min_coverage_ms = APP_ESC_MOTION_STOPPED_MIN_COVERAGE_MS_DEFAULT;
volatile uint32_t g_esc_speed_calibration_valid =
	APP_ESC_SPEED_CALIBRATION_VALID_DEFAULT;
volatile float g_esc_low_gear_wheel_rpm_per_raw =
	APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT;
volatile uint32_t g_esc_speed_fresh_timeout_ms =
	APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT;
volatile uint32_t g_mode2_drive_calibration_valid = APP_MODE2_DRIVE_CALIBRATION_VALID_DEFAULT;
volatile uint32_t g_mode2_drive_brake_calibration_valid = APP_MODE2_DRIVE_BRAKE_CALIBRATION_VALID_DEFAULT;
volatile uint32_t g_mode2_drive_first_strike_calibration_valid =
	APP_MODE2_DRIVE_FIRST_STRIKE_CALIBRATION_VALID_DEFAULT;
volatile uint32_t g_mode2_drive_neutral_dwell_valid = APP_MODE2_DRIVE_NEUTRAL_DWELL_VALID_DEFAULT;
volatile uint32_t g_mode2_drive_reversal_timeout_valid = APP_MODE2_DRIVE_REVERSAL_TIMEOUT_VALID_DEFAULT;
volatile float g_mode2_drive_brake_request = APP_MODE2_DRIVE_BRAKE_REQUEST_DEFAULT;
volatile float g_mode2_drive_reverse_first_strike_request =
	APP_MODE2_DRIVE_REVERSE_FIRST_STRIKE_REQUEST_DEFAULT;
volatile uint32_t g_mode2_drive_reverse_first_strike_min_ms =
	APP_MODE2_DRIVE_REVERSE_FIRST_STRIKE_MIN_MS_DEFAULT;
volatile uint32_t g_mode2_drive_neutral_dwell_ms = APP_MODE2_DRIVE_NEUTRAL_DWELL_MS_DEFAULT;
volatile uint32_t g_mode2_drive_reversal_timeout_ms = APP_MODE2_DRIVE_REVERSAL_TIMEOUT_MS_DEFAULT;
volatile uint32_t g_mode2_brake_pwm_valid = APP_MODE2_BRAKE_PWM_VALID_DEFAULT;
volatile uint32_t g_mode2_brake_pwm_center_us = APP_MODE2_BRAKE_PWM_CENTER_US_DEFAULT;
volatile uint32_t g_mode2_brake_pwm_full_us = APP_MODE2_BRAKE_PWM_FULL_US_DEFAULT;
volatile uint32_t g_orin_accel_limit_mmps2 = ORIN_ACCEL_LIMIT_DEFAULT_MMPS2;
volatile uint32_t g_orin_steering_rate_limit_mradps = ORIN_STEERING_RATE_LIMIT_DEFAULT_MRADPS;
volatile uint32_t g_speed_pi_enable = SPEED_PI_ENABLE_DEFAULT;
volatile float g_speed_pi_kp = SPEED_PI_KP_DEFAULT_US_PER_MPS;
volatile float g_speed_pi_ki = SPEED_PI_KI_DEFAULT_US_PER_MPS_S;
volatile uint32_t g_speed_pi_trim_limit_us = SPEED_PI_TRIM_LIMIT_DEFAULT_US;
volatile float g_speed_pi_target_vx_mps = 0.0f;
volatile float g_speed_pi_feedback_vx_mps = 0.0f;
volatile float g_speed_pi_error_mps = 0.0f;
volatile float g_speed_pi_integral = 0.0f;
volatile int32_t g_speed_pi_base_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile int32_t g_speed_pi_trim_us = 0;
volatile int32_t g_speed_pi_final_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile uint32_t g_speed_pi_feedback_valid = 0U;
volatile uint32_t g_speed_pi_saturated = 0U;
// RC debounce parameters (set from Keil Watch).
volatile uint32_t g_rc_debounce_enable = APP_RC_DEBOUNCE_ENABLE_DEFAULT;
volatile uint32_t g_rc_debounce_deadband_us = APP_RC_DEBOUNCE_DEADBAND_US;
volatile uint32_t g_rc_debounce_smooth_div = APP_RC_DEBOUNCE_SMOOTH_DIV;
volatile uint32_t g_rc_valid_min_us = RC_VALID_MIN_DEFAULT_US;
volatile uint32_t g_rc_valid_max_us = RC_VALID_MAX_DEFAULT_US;
volatile uint32_t g_rc_frame_min_us = RC_FRAME_MIN_DEFAULT_US;
volatile uint32_t g_rc_frame_max_us = RC_FRAME_MAX_DEFAULT_US;
volatile uint32_t g_rc_glitch_freeze_ms = RC_GLITCH_FREEZE_DEFAULT_MS;
volatile uint32_t g_rc_throttle_neutral_hold_us = RC_THROTTLE_NEUTRAL_HOLD_DEFAULT_US;
volatile uint32_t g_rc_throttle_jump_confirm_us = RC_THROTTLE_JUMP_CONFIRM_DEFAULT_US;
volatile uint32_t g_rc_steering_jump_confirm_us = RC_STEERING_JUMP_CONFIRM_DEFAULT_US;
volatile uint32_t g_rc_jump_confirm_samples = RC_JUMP_CONFIRM_SAMPLES_DEFAULT;
// RC override thresholds (set from Keil Watch).
volatile uint32_t g_rc_override_center_us = RC_OVERRIDE_CENTER_DEFAULT_US;
volatile uint32_t g_rc_override_enter_threshold_us = RC_OVERRIDE_ENTER_THRESHOLD_DEFAULT_US;
volatile uint32_t g_rc_override_exit_threshold_us = RC_OVERRIDE_EXIT_THRESHOLD_DEFAULT_US;
volatile uint32_t g_rc_override_enter_samples = RC_OVERRIDE_ENTER_SAMPLES_DEFAULT;
volatile uint32_t g_rc_override_release_hold_ms = RC_OVERRIDE_RELEASE_HOLD_DEFAULT_MS;
volatile uint32_t g_rc_guard_enable = APP_RC_GUARD_ENABLE_DEFAULT;
volatile uint32_t g_rc_guard_active_high = APP_RC_GUARD_ACTIVE_HIGH_DEFAULT;
volatile uint32_t g_rc_guard_active_low_threshold_us = RC_GUARD_ACTIVE_LOW_THRESHOLD_DEFAULT_US;
volatile uint32_t g_rc_guard_active_high_threshold_us = RC_GUARD_ACTIVE_HIGH_THRESHOLD_DEFAULT_US;
volatile uint32_t g_rc_throttle_last_good_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile uint32_t g_rc_steering_last_good_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile uint32_t g_rc_throttle_glitch_active = 0U;
volatile uint32_t g_rc_steering_glitch_active = 0U;
volatile uint32_t g_rc_input_fault_active = 0U;

typedef struct
{
	uint16_t speed_mmps;
	uint16_t pulse_us;
} orin_speed_ff_point_t;

typedef struct
{
	uint16_t esc_pulse_us;
	uint16_t servo_pulse_us;
	uint32_t last_update_ms;
	float target_speed_mps;
	float feedback_vx_mps;
	float feedback_vz_rad_s;
	float target_steering_angle_rad;
	float feedback_steering_angle_rad;
	uint8_t active;
	uint8_t stop;
	uint8_t auto_enabled;
	uint8_t brake_active;
	uint8_t emergency_stop;
	uint8_t speed_saturated;
	uint8_t steering_saturated;
	uint8_t accel_limited;
	uint8_t steering_rate_limited;
} orin_pwm_state_t;

typedef struct
{
	uint16_t filter_state;
	uint16_t last_good_us;
	uint16_t candidate_us;
	uint16_t output_us;
	uint32_t invalid_since_ms;
	uint8_t candidate_count;
	uint8_t glitch_active;
	uint8_t stable_present;
} rc_channel_filter_state_t;

typedef struct
{
	uint8_t valid;
	float kp;
	float max_request;
	float enter_error_mps;
	float release_error_mps;
} esc_tracking_brake_config_t;

typedef struct
{
	uint8_t valid;
	uint32_t center_us;
	uint32_t full_us;
	uint32_t neutral_us;
} mode2_brake_pwm_runtime_config_t;

static orin_pwm_state_t g_orin_state = {
	.esc_pulse_us = ESC_PWM_NEUTRAL_PULSE_US,
	.servo_pulse_us = ESC_PWM_NEUTRAL_PULSE_US,
	.last_update_ms = 0U,
	.target_speed_mps = 0.0f,
	.feedback_vx_mps = 0.0f,
	.feedback_vz_rad_s = 0.0f,
	.target_steering_angle_rad = 0.0f,
	.feedback_steering_angle_rad = 0.0f,
	.active = 0U,
	.stop = 0U,
	.auto_enabled = 0U,
	.brake_active = 0U,
	.emergency_stop = 0U,
	.speed_saturated = 0U,
	.steering_saturated = 0U,
	.accel_limited = 0U,
	.steering_rate_limited = 0U
};
static volatile uint8_t g_rc_override_active = 0U;
static volatile uint8_t g_rc_guard_active = 0U;
static uint8_t g_rc_override_enter_count = 0U;
static uint32_t g_rc_override_release_start_ms = 0U;
static uint8_t g_rc_override_release_hold_required = 0U;
static uint16_t g_rc_throttle_current = 0U;
static uint16_t g_rc_steering_current = 0U;
static uint16_t g_rc_guard_current = 0U;
static uint8_t g_rc_throttle_present = 0U;
static uint8_t g_rc_steering_present = 0U;
static uint8_t g_rc_guard_present = 0U;
static rc_channel_filter_state_t g_rc_throttle_state = {0U};
static rc_channel_filter_state_t g_rc_steering_state = {0U};
static uint32_t g_speed_pi_last_update_ms = 0U;
static uint32_t g_speed_pi_last_sample_id = 0U;
static uint8_t g_speed_pi_have_sample = 0U;
static int8_t g_speed_pi_last_target_direction = 0;
static uint32_t s_esc_speed_limit_over_count = 0U;
static uint32_t s_esc_speed_limit_last_sample_id = 0U;
static uint8_t s_esc_tracking_brake_active = 0U;
static EscMotionEstimator_t s_esc_motion_estimator;
static Mode2DriveGate_t s_mode2_drive_gate;
static EscMotionEstimatorConfig_t s_esc_motion_config;
static Mode2DriveGateConfig_t s_mode2_drive_config;
static esc_tracking_brake_config_t s_esc_tracking_brake_config;
static mode2_brake_pwm_runtime_config_t s_mode2_brake_pwm_config;
static uint8_t s_esc_motion_config_initialized = 0U;
static uint8_t s_mode2_drive_config_initialized = 0U;
static uint8_t s_esc_tracking_brake_config_initialized = 0U;
static uint8_t s_mode2_brake_pwm_config_initialized = 0U;
static EscMotionEstimate_t s_esc_motion_estimate;
static Mode2DriveGateOutput_t s_mode2_drive_output;
static uint8_t s_esc_feedback_available = 0U;
static uint8_t s_esc_stop_confirmed = 0U;
static uint8_t s_esc_sample_stale = 0U;
static uint8_t s_esc_rx_invalidated = 0U;
static uint8_t s_esc_receive_epoch_valid = 0U;
static uint32_t s_esc_receive_epoch = 0U;
static uint32_t s_esc_last_observed_sample_id = 0U;
static uint32_t s_esc_last_observed_epoch = 0U;
static EscFe32Sample_t s_esc_latest_raw_sample;
static uint8_t s_esc_latest_raw_sample_valid = 0U;
static EscTelemetryReceiverHealth_t s_esc_receiver_health;
static uint8_t s_vehicle_direction_known = 0U;
static int8_t s_vehicle_direction = 0;
static uint8_t s_auto_history_boundary_valid = 0U;
static uint32_t s_auto_history_boundary_ms = 0U;

#define ESC_DIAGNOSTIC_FRESH_TIMEOUT_MS 250U

static const orin_speed_ff_point_t s_orin_forward_ff_table[] = {
	{70U, 1546U},
	{100U, 1547U},
	{200U, 1548U},
	{300U, 1550U},
	{500U, 1554U},
	{800U, 1559U},
	{1000U, 1562U},
	{1500U, 1568U},
	{2000U, 1574U},
	{2500U, 1580U},
	{3000U, 1585U},
	{3500U, 1590U},
	{4000U, 1596U},
	{4500U, 1601U},
	{10000U, APP_ORIN_ESC_FORWARD_MAX_US},
};

static const orin_speed_ff_point_t s_orin_reverse_ff_table[] = {
	{180U, 1444U},
	{300U, 1442U},
	{500U, 1438U},
	{800U, 1433U},
	{1000U, 1430U},
	{1500U, 1423U},
	{2000U, 1417U},
	{2500U, 1412U},
	{3000U, 1407U},
	{3500U, 1402U},
	{4000U, 1397U},
	{4500U, 1391U},
};

static uint16_t limit_esc_safe_pulse(uint16_t pulse_us);
static uint16_t limit_servo_safe_pulse(uint16_t pulse_us);
static void speed_pi_reset_controller(void);
static uint32_t get_rc_override_center_us(void);
static uint8_t pulse_is_inside_center(uint16_t pulse_us, uint32_t center_us, uint32_t threshold_us);
static uint8_t orin_pwm_is_active(void);
static void servo_basic_apply_debug_command(uint32_t cmd, uint32_t value);
static void servo_basic_refresh_esc_configs(uint32_t now_ms);
static void servo_basic_update_esc_feedback(uint32_t now_ms);
static void servo_basic_invalidate_auto_history(uint32_t now_ms);
static uint8_t tracking_brake_config_is_valid(const esc_tracking_brake_config_t *config);
static uint8_t servo_basic_tick_is_after(uint32_t tick_ms, uint32_t reference_ms);
static uint8_t servo_basic_tick_delta_ms(uint32_t tick_ms, uint32_t reference_ms, uint32_t *delta_ms);
static uint8_t servo_basic_mode2_application_config_valid(void);
static void servo_basic_clear_vehicle_direction(void);
static uint8_t orin_pwm_is_active_at(uint32_t now_ms);
static servo_basic_diagnostics_t servo_basic_collect_diagnostics(uint32_t now_ms);
static void servo_basic_publish_control_snapshot(uint32_t now_ms);

__attribute__((weak)) void ServoBasic_OutputEscPulse(uint16_t pulse_us)
{
	(void)pulse_us;
}

__attribute__((weak)) void ServoBasic_OutputServoPulse(uint16_t pulse_us)
{
	(void)pulse_us;
}

static uint16_t clamp_servo_angle(uint16_t angle_deg)
{
	return (angle_deg > 180U) ? 180U : angle_deg;
}

static uint16_t servo_angle_to_pulse(uint16_t angle_deg)
{
	const uint32_t servo_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
	return (uint16_t)(SERVO_MIN_PULSE_US +
					  ((uint32_t)angle_deg * servo_range) / 180U);
}

static uint8_t clamp_servo_step(uint8_t step)
{
	const uint8_t max_step = (uint8_t)((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / SERVO_PULSE_STEP_US);
	return (step > max_step) ? max_step : step;
}

static uint16_t servo_step_to_pulse(uint8_t step)
{
	return (uint16_t)(SERVO_MIN_PULSE_US + (uint32_t)step * SERVO_PULSE_STEP_US);
}

static uint16_t clamp_esc_pulse(uint16_t pulse_us)
{
	if (pulse_us < ESC_PWM_MIN_PULSE_US)
	{
		return ESC_PWM_MIN_PULSE_US;
	}
	if (pulse_us > ESC_PWM_MAX_PULSE_US)
	{
		return ESC_PWM_MAX_PULSE_US;
	}
	return pulse_us;
}

static uint16_t clamp_servo_pulse(uint16_t pulse_us)
{
	if (pulse_us < SERVO_MIN_PULSE_US)
	{
		return SERVO_MIN_PULSE_US;
	}
	if (pulse_us > SERVO_MAX_PULSE_US)
	{
		return SERVO_MAX_PULSE_US;
	}
	return pulse_us;
}

static uint16_t rc_select_pulse(uint16_t pulse_us, uint8_t is_esc)
{
	if (is_esc != 0U)
	{
		return limit_esc_safe_pulse(pulse_us);
	}
	if (g_rc_pwm_follow_raw != 0U)
	{
		return limit_servo_safe_pulse(pulse_us);
	}
	return limit_servo_safe_pulse(clamp_servo_pulse(pulse_us));
}

static void rc_debounce_reset(void)
{
	memset(&g_rc_throttle_state, 0, sizeof(g_rc_throttle_state));
	memset(&g_rc_steering_state, 0, sizeof(g_rc_steering_state));
	g_rc_throttle_current = 0U;
	g_rc_steering_current = 0U;
	g_rc_guard_current = 0U;
	g_rc_throttle_last_good_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_rc_steering_last_good_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_rc_throttle_glitch_active = 0U;
	g_rc_steering_glitch_active = 0U;
	g_rc_input_fault_active = 0U;
}

static uint16_t rc_debounce_apply(uint16_t raw, uint16_t *state)
{
	if (raw == 0U)
	{
		return 0U;
	}
	if (g_rc_debounce_enable == 0U)
	{
		*state = raw;
		return raw;
	}
	if (*state == 0U)
	{
		*state = raw;
		return raw;
	}

	uint32_t deadband = g_rc_debounce_deadband_us;
	uint32_t diff = (raw > *state) ? (raw - *state) : (*state - raw);
	if (deadband != 0U && diff <= deadband)
	{
		return *state;
	}

	uint32_t div = g_rc_debounce_smooth_div;
	if (div < 1U)
	{
		div = 1U;
	}
	else if (div > 16U)
	{
		div = 16U;
	}

	int32_t delta = (int32_t)raw - (int32_t)*state;
	int32_t step = delta / (int32_t)div;
	if (step == 0)
	{
		step = (delta > 0) ? 1 : -1;
	}
	*state = (uint16_t)((int32_t)*state + step);
	return *state;
}

static uint32_t get_rc_glitch_freeze_ms(void)
{
	return (g_rc_glitch_freeze_ms == 0U) ? RC_GLITCH_FREEZE_DEFAULT_MS : g_rc_glitch_freeze_ms;
}

static uint32_t get_rc_throttle_neutral_hold_us(void)
{
	return (g_rc_throttle_neutral_hold_us == 0U) ? RC_THROTTLE_NEUTRAL_HOLD_DEFAULT_US :
		g_rc_throttle_neutral_hold_us;
}

static uint32_t get_rc_throttle_jump_confirm_us(void)
{
	return (g_rc_throttle_jump_confirm_us == 0U) ? RC_THROTTLE_JUMP_CONFIRM_DEFAULT_US :
		g_rc_throttle_jump_confirm_us;
}

static uint32_t get_rc_steering_jump_confirm_us(void)
{
	return (g_rc_steering_jump_confirm_us == 0U) ? RC_STEERING_JUMP_CONFIRM_DEFAULT_US :
		g_rc_steering_jump_confirm_us;
}

static uint32_t get_rc_jump_confirm_samples(void)
{
	return (g_rc_jump_confirm_samples == 0U) ? RC_JUMP_CONFIRM_SAMPLES_DEFAULT :
		g_rc_jump_confirm_samples;
}

static uint32_t pulse_diff(uint16_t a, uint16_t b)
{
	return (a > b) ? ((uint32_t)a - (uint32_t)b) : ((uint32_t)b - (uint32_t)a);
}

static uint16_t finalize_rc_channel_pulse(rc_channel_filter_state_t *state, uint16_t pulse_us, uint8_t is_throttle)
{
	uint16_t filtered;

	if (state == NULL || pulse_us == 0U)
	{
		return 0U;
	}

	if (is_throttle != 0U)
	{
		filtered = rc_debounce_apply(pulse_us, &state->filter_state);
		if (pulse_is_inside_center(filtered, get_rc_override_center_us(), get_rc_throttle_neutral_hold_us()) != 0U)
		{
			filtered = (uint16_t)get_rc_override_center_us();
			state->filter_state = filtered;
		}
	}
	else
	{
		filtered = rc_debounce_apply(pulse_us, &state->filter_state);
	}

	state->last_good_us = filtered;
	return filtered;
}

static void update_rc_channel_state(rc_channel_filter_state_t *state,
									uint16_t raw,
									uint8_t raw_present,
									uint8_t fault_active,
									uint32_t now_ms,
									uint8_t is_throttle)
{
	uint32_t jump_threshold;
	uint32_t confirm_samples;

	if (state == NULL)
	{
		return;
	}

	state->stable_present = 0U;

	if (fault_active != 0U)
	{
		state->candidate_us = 0U;
		state->candidate_count = 0U;
		state->glitch_active = 1U;
		if (state->invalid_since_ms == 0U)
		{
			state->invalid_since_ms = now_ms;
		}

		if (state->last_good_us != 0U &&
			(now_ms - state->invalid_since_ms) < get_rc_glitch_freeze_ms())
		{
			state->output_us = state->last_good_us;
			state->stable_present = 1U;
		}
		else
		{
			state->output_us = 0U;
			state->stable_present = 0U;
		}
		return;
	}

	if (raw_present == 0U || raw == 0U)
	{
		state->filter_state = 0U;
		state->candidate_us = 0U;
		state->candidate_count = 0U;
		state->output_us = 0U;
		state->invalid_since_ms = 0U;
		state->glitch_active = 0U;
		return;
	}

	state->invalid_since_ms = 0U;
	jump_threshold = (is_throttle != 0U) ? get_rc_throttle_jump_confirm_us() :
		get_rc_steering_jump_confirm_us();
	confirm_samples = get_rc_jump_confirm_samples();
	if (confirm_samples < 1U)
	{
		confirm_samples = 1U;
	}

	if (state->last_good_us == 0U || state->filter_state == 0U)
	{
		state->candidate_us = 0U;
		state->candidate_count = 0U;
		state->glitch_active = 0U;
		state->filter_state = raw;
		state->output_us = finalize_rc_channel_pulse(state, raw, is_throttle);
		state->stable_present = 1U;
		return;
	}

	if (jump_threshold != 0U && pulse_diff(raw, state->last_good_us) > jump_threshold)
	{
		if (state->candidate_count == 0U || pulse_diff(raw, state->candidate_us) > jump_threshold)
		{
			state->candidate_us = raw;
			state->candidate_count = 1U;
			state->glitch_active = 1U;
			state->output_us = state->last_good_us;
			state->stable_present = 1U;
			return;
		}

		if (state->candidate_count < confirm_samples)
		{
			state->candidate_count++;
		}

		if (state->candidate_count < confirm_samples)
		{
			state->glitch_active = 1U;
			state->output_us = state->last_good_us;
			state->stable_present = 1U;
			return;
		}

		state->filter_state = raw;
		raw = state->candidate_us;
	}

	state->candidate_us = 0U;
	state->candidate_count = 0U;
	state->glitch_active = 0U;
	state->output_us = finalize_rc_channel_pulse(state, raw, is_throttle);
	state->stable_present = 1U;
}

static uint8_t rc_channel_fault_is_persistent(const rc_channel_filter_state_t *state,
											  uint32_t now_ms)
{
	if (state == NULL || state->invalid_since_ms == 0U)
	{
		return 0U;
	}

	return ((now_ms - state->invalid_since_ms) >= get_rc_glitch_freeze_ms()) ? 1U : 0U;
}

static uint32_t get_rc_signal_timeout_ms(void)
{
	return (g_rc_signal_timeout_ms == 0U) ? 100U : g_rc_signal_timeout_ms;
}

static uint32_t get_rc_override_center_us(void)
{
	return (g_rc_override_center_us == 0U) ? RC_OVERRIDE_CENTER_DEFAULT_US : g_rc_override_center_us;
}

static uint32_t get_rc_override_enter_threshold_us(void)
{
	return (g_rc_override_enter_threshold_us == 0U) ? RC_OVERRIDE_ENTER_THRESHOLD_DEFAULT_US : g_rc_override_enter_threshold_us;
}

static uint32_t get_rc_override_exit_threshold_us(void)
{
	return (g_rc_override_exit_threshold_us == 0U) ? RC_OVERRIDE_EXIT_THRESHOLD_DEFAULT_US : g_rc_override_exit_threshold_us;
}

static uint32_t get_rc_override_enter_samples(void)
{
	return (g_rc_override_enter_samples == 0U) ? RC_OVERRIDE_ENTER_SAMPLES_DEFAULT : g_rc_override_enter_samples;
}

static uint32_t get_rc_override_release_hold_ms(void)
{
	return (g_rc_override_release_hold_ms == 0U) ? RC_OVERRIDE_RELEASE_HOLD_DEFAULT_MS : g_rc_override_release_hold_ms;
}

static uint32_t get_orin_ackermann_wheelbase_mm(void)
{
	return (g_orin_ackermann_wheelbase_mm == 0U) ? ORIN_ACKERMANN_WHEELBASE_DEFAULT_MM : g_orin_ackermann_wheelbase_mm;
}

static uint32_t get_orin_ackermann_track_width_mm(void)
{
	return (g_orin_ackermann_track_width_mm == 0U) ? ORIN_ACKERMANN_TRACK_WIDTH_DEFAULT_MM : g_orin_ackermann_track_width_mm;
}

static uint32_t get_orin_ackermann_wheel_radius_mm(void)
{
	return (g_orin_ackermann_wheel_radius_mm == 0U) ? ORIN_ACKERMANN_WHEEL_RADIUS_DEFAULT_MM : g_orin_ackermann_wheel_radius_mm;
}

static uint32_t get_orin_ackermann_max_steering_millirad(void)
{
	return (g_orin_ackermann_max_steering_millirad == 0U) ? ORIN_ACKERMANN_MAX_STEERING_DEFAULT_MRAD :
		g_orin_ackermann_max_steering_millirad;
}

static uint32_t get_orin_ackermann_min_vx_mmps(void)
{
	return (g_orin_ackermann_min_vx_mmps == 0U) ? ORIN_ACKERMANN_MIN_VX_DEFAULT_MMPS : g_orin_ackermann_min_vx_mmps;
}

static uint32_t get_orin_vx_scale_permille(void)
{
	return (g_orin_vx_scale == 0U) ? ORIN_VX_SCALE_DEFAULT_PERMILLE : g_orin_vx_scale;
}

static uint32_t get_orin_vx_forward_cap_mmps(void)
{
	if (g_orin_vx_forward_cap_mmps != 0U)
	{
		return g_orin_vx_forward_cap_mmps;
	}
	return (g_orin_vx_max_mmps == 0U) ? ORIN_VX_FORWARD_CAP_DEFAULT_MMPS : g_orin_vx_max_mmps;
}

static uint32_t get_orin_vx_reverse_cap_mmps(void)
{
	if (g_orin_vx_reverse_cap_mmps != 0U)
	{
		return g_orin_vx_reverse_cap_mmps;
	}
	return (g_orin_vx_max_mmps == 0U) ? ORIN_VX_REVERSE_CAP_DEFAULT_MMPS : g_orin_vx_max_mmps;
}

static uint32_t get_orin_vx_deadband_mmps(void)
{
	return (g_orin_vx_deadband_mmps == 0U) ? ORIN_VX_DEADBAND_DEFAULT_MMPS : g_orin_vx_deadband_mmps;
}

static uint32_t get_orin_min_forward_vx_mmps(void)
{
	return ORIN_MIN_FORWARD_VX_DEFAULT_MMPS;
}

static uint32_t get_orin_min_reverse_vx_mmps(void)
{
	return ORIN_MIN_REVERSE_VX_DEFAULT_MMPS;
}

static uint16_t get_orin_esc_center_pulse(void)
{
	uint32_t pulse = g_orin_esc_center_us;
	if (pulse < ESC_PWM_MIN_PULSE_US)
	{
		pulse = ESC_PWM_MIN_PULSE_US;
	}
	if (pulse > ESC_PWM_MAX_PULSE_US)
	{
		pulse = ESC_PWM_MAX_PULSE_US;
	}
	return (uint16_t)pulse;
}

static uint16_t get_orin_servo_center_pulse(void)
{
	uint32_t pulse = g_orin_servo_center_us;
	if (pulse < SERVO_MIN_PULSE_US)
	{
		pulse = SERVO_MIN_PULSE_US;
	}
	if (pulse > SERVO_MAX_PULSE_US)
	{
		pulse = SERVO_MAX_PULSE_US;
	}
	return (uint16_t)pulse;
}

static uint16_t get_orin_esc_forward_start_pulse(void)
{
	uint16_t center = get_orin_esc_center_pulse();
	uint16_t pulse = clamp_esc_pulse((uint16_t)((g_orin_esc_forward_start_us == 0U) ?
		ORIN_ESC_FORWARD_START_DEFAULT_US : g_orin_esc_forward_start_us));
	return (pulse < center) ? center : pulse;
}

static uint16_t get_orin_esc_reverse_start_pulse(void)
{
	uint16_t center = get_orin_esc_center_pulse();
	uint16_t pulse = clamp_esc_pulse((uint16_t)((g_orin_esc_reverse_start_us == 0U) ?
		ORIN_ESC_REVERSE_START_DEFAULT_US : g_orin_esc_reverse_start_us));
	return (pulse > center) ? center : pulse;
}

static uint16_t get_orin_esc_forward_limit_pulse(void)
{
	uint16_t start = get_orin_esc_forward_start_pulse();
	uint16_t pulse = clamp_esc_pulse((uint16_t)((g_orin_esc_forward_max_us == 0U) ?
		ORIN_ESC_FORWARD_MAX_DEFAULT_US : g_orin_esc_forward_max_us));
	return (pulse < start) ? start : pulse;
}

static uint16_t get_orin_esc_reverse_limit_pulse(void)
{
	uint16_t start = get_orin_esc_reverse_start_pulse();
	uint16_t pulse = clamp_esc_pulse((uint16_t)((g_orin_esc_reverse_max_us == 0U) ?
		ORIN_ESC_REVERSE_MAX_DEFAULT_US : g_orin_esc_reverse_max_us));
	return (pulse > start) ? start : pulse;
}

static float get_orin_velocity_neutral_threshold_mps(void)
{
	float deadband_mps = (float)get_orin_vx_deadband_mmps() / 1000.0f;
	float min_vx_mps = (float)get_orin_ackermann_min_vx_mmps() / 1000.0f;
	return (deadband_mps > min_vx_mps) ? deadband_mps : min_vx_mps;
}

static float scale_and_limit_orin_vx(float vx_mps)
{
	float scaled_vx = vx_mps * ((float)get_orin_vx_scale_permille() / 1000.0f);
	const float forward_cap_mps = (float)get_orin_vx_forward_cap_mmps() / 1000.0f;
	const float reverse_cap_mps = (float)get_orin_vx_reverse_cap_mmps() / 1000.0f;

	if (scaled_vx > forward_cap_mps)
	{
		scaled_vx = forward_cap_mps;
	}
	else if (scaled_vx < -reverse_cap_mps)
	{
		scaled_vx = -reverse_cap_mps;
	}

	return scaled_vx;
}

static int8_t get_vx_direction(float vx_mps)
{
	if (vx_mps > 0.0f)
	{
		return 1;
	}
	if (vx_mps < 0.0f)
	{
		return -1;
	}
	return 0;
}

static int8_t get_rc_throttle_direction(void)
{
	const uint32_t center_us = get_rc_override_center_us();
	const uint32_t neutral_hold_us = get_rc_throttle_neutral_hold_us();
	const uint32_t throttle_us = (uint32_t)g_rc_throttle_current;

	if (g_rc_throttle_present == 0U || throttle_us == 0U)
	{
		return 0;
	}
	if (throttle_us > (center_us + neutral_hold_us))
	{
		return 1;
	}
	if ((throttle_us + neutral_hold_us) < center_us)
	{
		return -1;
	}
	return 0;
}

static uint32_t speed_mps_abs_to_mmps(float speed_mps)
{
	float abs_speed_mps = fabsf(speed_mps);
	float mmps = abs_speed_mps * 1000.0f;

	if (mmps > 65535.0f)
	{
		mmps = 65535.0f;
	}
	return (uint32_t)(mmps + 0.5f);
}

static uint8_t orin_target_is_below_min_control(float vx_mps)
{
	if (vx_mps > 0.0f)
	{
		return (speed_mps_abs_to_mmps(vx_mps) < get_orin_min_forward_vx_mmps()) ? 1U : 0U;
	}
	if (vx_mps < 0.0f)
	{
		return (speed_mps_abs_to_mmps(vx_mps) < get_orin_min_reverse_vx_mmps()) ? 1U : 0U;
	}
	return 1U;
}

static uint16_t interpolate_speed_ff_table(const orin_speed_ff_point_t *table,
										   uint32_t table_count,
										   uint32_t speed_mmps)
{
	uint32_t i;

	if (table == NULL || table_count == 0U)
	{
		return get_orin_esc_center_pulse();
	}

	if (speed_mmps <= table[0].speed_mmps)
	{
		return table[0].pulse_us;
	}

	for (i = 1U; i < table_count; ++i)
	{
		const uint32_t low_speed = table[i - 1U].speed_mmps;
		const uint32_t high_speed = table[i].speed_mmps;

		if (speed_mmps <= high_speed)
		{
			const int32_t low_pulse = (int32_t)table[i - 1U].pulse_us;
			const int32_t high_pulse = (int32_t)table[i].pulse_us;
			const int32_t delta_pulse = high_pulse - low_pulse;
			const uint32_t delta_speed = high_speed - low_speed;
			const uint32_t target_delta = speed_mmps - low_speed;
			int32_t pulse = low_pulse;

			if (delta_speed != 0U)
			{
				pulse += (int32_t)(((int64_t)delta_pulse * (int64_t)target_delta) /
					(int64_t)delta_speed);
			}
			return clamp_esc_pulse((uint16_t)pulse);
		}
	}

	return table[table_count - 1U].pulse_us;
}

static uint16_t limit_esc_safe_pulse(uint16_t pulse_us)
{
	uint16_t center;
	uint16_t forward_limit;
	uint16_t reverse_limit;

	if (pulse_us == 0U)
	{
		return 0U;
	}

	pulse_us = clamp_esc_pulse(pulse_us);
	center = get_orin_esc_center_pulse();
	forward_limit = get_orin_esc_forward_limit_pulse();
	reverse_limit = get_orin_esc_reverse_limit_pulse();

	if (pulse_us >= center)
	{
		return (pulse_us > forward_limit) ? forward_limit : pulse_us;
	}
	return (pulse_us < reverse_limit) ? reverse_limit : pulse_us;
}

static uint16_t limit_servo_safe_pulse(uint16_t pulse_us)
{
	uint16_t center;
	uint32_t range;
	uint16_t low;
	uint16_t high;

	if (pulse_us == 0U)
	{
		return 0U;
	}

	pulse_us = clamp_servo_pulse(pulse_us);
	center = get_orin_servo_center_pulse();
	range = (g_orin_servo_range_us == 0U) ? ORIN_SERVO_RANGE_DEFAULT_US : g_orin_servo_range_us;
	low = (center > range) ? (uint16_t)(center - range) : SERVO_MIN_PULSE_US;
	high = (uint16_t)(center + range);
	if (high > SERVO_MAX_PULSE_US)
	{
		high = SERVO_MAX_PULSE_US;
	}

	if (pulse_us < low)
	{
		return low;
	}
	if (pulse_us > high)
	{
		return high;
	}
	return pulse_us;
}

static uint32_t get_esc_speed_limit_mmps(void)
{
	return (g_esc_speed_limit_mmps == 0U) ?
		ESC_SPEED_LIMIT_DEFAULT_MMPS : g_esc_speed_limit_mmps;
}

static uint32_t get_esc_speed_limit_release_mmps(void)
{
	uint32_t limit_mmps = get_esc_speed_limit_mmps();
	uint32_t release_mmps = (g_esc_speed_limit_release_mmps == 0U) ?
		ESC_SPEED_LIMIT_RELEASE_DEFAULT_MMPS : g_esc_speed_limit_release_mmps;

	return (release_mmps > limit_mmps) ? limit_mmps : release_mmps;
}

static uint32_t get_orin_accel_limit_mmps2(void)
{
	return (g_orin_accel_limit_mmps2 == 0U) ?
		ORIN_ACCEL_LIMIT_DEFAULT_MMPS2 : g_orin_accel_limit_mmps2;
}

static uint32_t get_orin_steering_rate_limit_mradps(void)
{
	return (g_orin_steering_rate_limit_mradps == 0U) ?
		ORIN_STEERING_RATE_LIMIT_DEFAULT_MRADPS : g_orin_steering_rate_limit_mradps;
}

static uint8_t orin_vx_would_saturate(float vx_mps)
{
	const float scaled_vx = vx_mps * ((float)get_orin_vx_scale_permille() / 1000.0f);
	const float forward_cap_mps = (float)get_orin_vx_forward_cap_mmps() / 1000.0f;
	const float reverse_cap_mps = (float)get_orin_vx_reverse_cap_mmps() / 1000.0f;

	if (scaled_vx > forward_cap_mps || scaled_vx < -reverse_cap_mps)
	{
		return 1U;
	}
	return 0U;
}

static float apply_rate_limit_float(float target,
									float previous,
									float limit_per_s,
									float dt_s,
									uint8_t *limited)
{
	const float max_delta = limit_per_s * dt_s;
	float delta = target - previous;

	if (limited != NULL)
	{
		*limited = 0U;
	}
	if (limit_per_s <= 0.001f || dt_s <= 0.0f)
	{
		return target;
	}
	if (delta > max_delta)
	{
		if (limited != NULL)
		{
			*limited = 1U;
		}
		return previous + max_delta;
	}
	if (delta < -max_delta)
	{
		if (limited != NULL)
		{
			*limited = 1U;
		}
		return previous - max_delta;
	}
	return target;
}

static float clamp_unit_float(float value)
{
	if (value < 0.0f)
	{
		return 0.0f;
	}
	if (value > 1.0f)
	{
		return 1.0f;
	}
	return value;
}

static uint8_t esc_motion_direction_is_forward(void)
{
	return (s_esc_motion_estimate.direction_valid != 0U &&
		s_esc_motion_estimate.direction == ESC_MOTION_DIRECTION_FORWARD) ? 1U : 0U;
}

static uint8_t esc_motion_direction_is_reverse(void)
{
	return (s_esc_motion_estimate.direction_valid != 0U &&
		s_esc_motion_estimate.direction == ESC_MOTION_DIRECTION_REVERSE) ? 1U : 0U;
}

static uint8_t esc_speed_limit_protection_active(void)
{
	const float limit_mps = (float)get_esc_speed_limit_mmps() / 1000.0f;
	const float release_mps = (float)get_esc_speed_limit_release_mmps() / 1000.0f;
	const uint32_t confirm_samples = ESC_SPEED_LIMIT_CONFIRM_SAMPLES;
	float speed_abs_mps;

	if (s_esc_feedback_available == 0U ||
		s_esc_motion_estimate.magnitude_valid == 0U)
	{
		g_esc_speed_limit_active = 0U;
		s_esc_speed_limit_over_count = 0U;
		s_esc_speed_limit_last_sample_id = 0U;
		return 0U;
	}

	speed_abs_mps = s_esc_motion_estimate.speed_magnitude_mps;
	if (g_esc_speed_limit_active != 0U)
	{
		if (speed_abs_mps <= release_mps)
		{
			g_esc_speed_limit_active = 0U;
			s_esc_speed_limit_over_count = 0U;
			s_esc_speed_limit_last_sample_id = 0U;
		}
	}
	else if (speed_abs_mps >= limit_mps)
	{
		if (s_esc_speed_limit_last_sample_id != s_esc_motion_estimate.last_sample_id)
		{
			s_esc_speed_limit_last_sample_id = s_esc_motion_estimate.last_sample_id;
			if (s_esc_speed_limit_over_count < confirm_samples)
			{
				s_esc_speed_limit_over_count++;
			}
			if (s_esc_speed_limit_over_count >= confirm_samples)
			{
				g_esc_speed_limit_active = 1U;
			}
		}
	}
	else
	{
		s_esc_speed_limit_over_count = 0U;
		s_esc_speed_limit_last_sample_id = s_esc_motion_estimate.last_sample_id;
	}

	return (g_esc_speed_limit_active != 0U) ? 1U : 0U;
}

static float esc_tracking_brake_reference_mps(Mode2DriveTargetDirection_t target_direction,
											  float target_vx_mps,
											  uint8_t stop_like_request,
											  uint8_t absolute_speed_limit_active)
{
	float reference_mps = 0.0f;

	if (stop_like_request == 0U)
	{
		if (target_direction == MODE2_DRIVE_TARGET_FORWARD &&
			esc_motion_direction_is_forward() != 0U)
		{
			reference_mps = fabsf(target_vx_mps);
		}
		else if (target_direction == MODE2_DRIVE_TARGET_REVERSE &&
				 esc_motion_direction_is_reverse() != 0U)
		{
			reference_mps = fabsf(target_vx_mps);
		}
	}

	if (absolute_speed_limit_active != 0U)
	{
		const float release_mps = (float)get_esc_speed_limit_release_mmps() / 1000.0f;
		if (reference_mps <= 0.0f || reference_mps > release_mps)
		{
			reference_mps = release_mps;
		}
	}

	return reference_mps;
}

static uint8_t esc_tracking_brake_update(Mode2DriveTargetDirection_t target_direction,
										 float target_vx_mps,
										 uint8_t stop_like_request,
										 uint8_t absolute_speed_limit_active,
										 uint8_t *brake_request_valid,
										 float *normalized_brake_request)
{
	const float speed_abs_mps = s_esc_motion_estimate.speed_magnitude_mps;
	const float reference_mps = esc_tracking_brake_reference_mps(target_direction,
		target_vx_mps,
		stop_like_request,
		absolute_speed_limit_active);
	const float speed_error_mps = speed_abs_mps - reference_mps;
	uint8_t decel_requested = 0U;
	uint8_t release_to_center = 0U;

	if (brake_request_valid != NULL)
	{
		*brake_request_valid = 0U;
	}
	if (normalized_brake_request != NULL)
	{
		*normalized_brake_request = 0.0f;
	}

	if (s_esc_feedback_available == 0U ||
		s_esc_motion_estimate.magnitude_valid == 0U ||
		tracking_brake_config_is_valid(&s_esc_tracking_brake_config) == 0U)
	{
		s_esc_tracking_brake_active = 0U;
		return 0U;
	}

	if (absolute_speed_limit_active != 0U)
	{
		s_esc_tracking_brake_active = 1U;
	}
	else if (s_esc_tracking_brake_active != 0U)
	{
		if (speed_error_mps <= s_esc_tracking_brake_config.release_error_mps)
		{
			s_esc_tracking_brake_active = 0U;
			release_to_center = 1U;
		}
	}
	else if (speed_error_mps >= s_esc_tracking_brake_config.enter_error_mps)
	{
		s_esc_tracking_brake_active = 1U;
	}

	decel_requested =
		(s_esc_tracking_brake_active != 0U || release_to_center != 0U) ? 1U : 0U;
	if (decel_requested == 0U)
	{
		return 0U;
	}

	if (esc_motion_direction_is_forward() != 0U)
	{
		float request = (release_to_center != 0U) ? 0.0f :
			s_esc_tracking_brake_config.kp * speed_error_mps;
		if (request > s_esc_tracking_brake_config.max_request)
		{
			request = s_esc_tracking_brake_config.max_request;
		}
		request = clamp_unit_float(request);
		if (brake_request_valid != NULL)
		{
			*brake_request_valid = 1U;
		}
		if (normalized_brake_request != NULL)
		{
			*normalized_brake_request = request;
		}
	}

	return 1U;
}

static uint8_t pulse_is_outside_center(uint16_t pulse_us, uint32_t center_us, uint32_t threshold_us)
{
	uint32_t diff;

	if (pulse_us == 0U)
	{
		return 0U;
	}

	diff = (pulse_us > center_us) ? ((uint32_t)pulse_us - center_us) : (center_us - (uint32_t)pulse_us);
	return (diff > threshold_us) ? 1U : 0U;
}

static uint8_t pulse_is_inside_center(uint16_t pulse_us, uint32_t center_us, uint32_t threshold_us)
{
	uint32_t diff;

	if (pulse_us == 0U)
	{
		return 1U;
	}

	diff = (pulse_us > center_us) ? ((uint32_t)pulse_us - center_us) : (center_us - (uint32_t)pulse_us);
	return (diff <= threshold_us) ? 1U : 0U;
}

static void refresh_rc_inputs(void)
{
	const uint32_t timeout_ms = get_rc_signal_timeout_ms();
	const uint32_t now_ms = HAL_GetTick();
	const uint16_t raw_throttle = ServoRC_GetThrottlePulse();
	const uint16_t raw_steering = ServoRC_GetSteeringPulse();
	const uint8_t raw_throttle_present = ServoRC_IsThrottleActive(timeout_ms);
	const uint8_t raw_steering_present = ServoRC_IsSteeringActive(timeout_ms);
	const uint8_t throttle_fault = ServoRC_HasThrottleFault();
	const uint8_t steering_fault = ServoRC_HasSteeringFault();
	const uint8_t guard_fault = ServoRC_HasGuardFault();
	uint8_t throttle_fault_persistent;
	uint8_t steering_fault_persistent;

	g_rc_guard_present = ServoRC_IsGuardActive(timeout_ms);

	update_rc_channel_state(&g_rc_throttle_state, raw_throttle, raw_throttle_present, throttle_fault, now_ms, 1U);
	update_rc_channel_state(&g_rc_steering_state, raw_steering, raw_steering_present, steering_fault, now_ms, 0U);
	throttle_fault_persistent = rc_channel_fault_is_persistent(&g_rc_throttle_state, now_ms);
	steering_fault_persistent = rc_channel_fault_is_persistent(&g_rc_steering_state, now_ms);

	g_rc_throttle_present = g_rc_throttle_state.stable_present;
	g_rc_steering_present = g_rc_steering_state.stable_present;
	g_rc_throttle_current = g_rc_throttle_state.output_us;
	g_rc_steering_current = g_rc_steering_state.output_us;
	g_rc_throttle_last_good_us = (g_rc_throttle_state.last_good_us != 0U) ?
		g_rc_throttle_state.last_good_us : ESC_PWM_NEUTRAL_PULSE_US;
	g_rc_steering_last_good_us = (g_rc_steering_state.last_good_us != 0U) ?
		g_rc_steering_state.last_good_us : ESC_PWM_NEUTRAL_PULSE_US;
	g_rc_throttle_glitch_active = g_rc_throttle_state.glitch_active;
	g_rc_steering_glitch_active = g_rc_steering_state.glitch_active;
	g_rc_input_fault_active = (throttle_fault_persistent != 0U ||
		steering_fault_persistent != 0U ||
		(g_rc_guard_enable != 0U && guard_fault != 0U)) ? 1U : 0U;

	g_rc_guard_current = (g_rc_guard_present != 0U) ? ServoRC_GetGuardPulse() : 0U;
}

static uint8_t rc_guard_input_is_active(void)
{
	if (g_rc_guard_enable == 0U || g_rc_guard_present == 0U || g_rc_guard_current == 0U)
	{
		return 0U;
	}

	if (g_rc_guard_active_high != 0U)
	{
		const uint32_t threshold = (g_rc_guard_active_high_threshold_us == 0U) ?
			RC_GUARD_ACTIVE_HIGH_THRESHOLD_DEFAULT_US : g_rc_guard_active_high_threshold_us;
		return (g_rc_guard_current >= threshold) ? 1U : 0U;
	}

	{
		const uint32_t threshold = (g_rc_guard_active_low_threshold_us == 0U) ?
			RC_GUARD_ACTIVE_LOW_THRESHOLD_DEFAULT_US : g_rc_guard_active_low_threshold_us;
		return (g_rc_guard_current <= threshold) ? 1U : 0U;
	}
}

static uint8_t rc_manual_override_requested(void)
{
	const uint32_t center_us = get_rc_override_center_us();
	const uint32_t threshold_us = get_rc_override_enter_threshold_us();

	if ((g_rc_throttle_present != 0U) &&
		(pulse_is_outside_center(g_rc_throttle_current, center_us, threshold_us) != 0U))
	{
		return 1U;
	}

	if ((g_rc_steering_present != 0U) &&
		(pulse_is_outside_center(g_rc_steering_current, center_us, threshold_us) != 0U))
	{
		return 1U;
	}

	return 0U;
}

static uint8_t rc_inputs_are_centered(void)
{
	const uint32_t center_us = get_rc_override_center_us();
	const uint32_t threshold_us = get_rc_override_exit_threshold_us();
	const uint8_t throttle_centered = (g_rc_throttle_present == 0U) ? 1U :
		pulse_is_inside_center(g_rc_throttle_current, center_us, threshold_us);
	const uint8_t steering_centered = (g_rc_steering_present == 0U) ? 1U :
		pulse_is_inside_center(g_rc_steering_current, center_us, threshold_us);

	return (throttle_centered != 0U && steering_centered != 0U) ? 1U : 0U;
}

static uint8_t rc_passthrough_is_available(void)
{
	return (g_rc_throttle_present != 0U || g_rc_steering_present != 0U) ? 1U : 0U;
}

static void set_rc_override_state(
	uint8_t override_active, uint8_t guard_active, uint8_t release_hold_required)
{
	g_rc_override_active = (override_active != 0U) ? 1U : 0U;
	g_rc_guard_active = (guard_active != 0U) ? 1U : 0U;
	g_rc_override_release_hold_required =
		(g_rc_override_active != 0U && release_hold_required != 0U) ? 1U : 0U;
	g_state.control_mode = (g_rc_override_active != 0U) ? SERVO_CTRL_MODE_RC_PASSTHROUGH :
		SERVO_CTRL_MODE_AUTONOMOUS;
	g_state.rc_takeover_pending = 0U;
	g_state.emergency_stop = g_rc_guard_active;
	g_rc_override_release_start_ms = 0U;
	g_rc_override_enter_count = 0U;
	if (g_rc_override_active != 0U)
	{
		HallSpeed_SetCommandDirection(get_rc_throttle_direction());
	}
}

static void apply_esc_pulse(uint16_t pulse_us)
{
	g_state.esc_pulse_us = pulse_us;
	ServoBasic_OutputEscPulse(pulse_us);
}

static void speed_pi_reset_controller(void)
{
	g_speed_pi_last_update_ms = 0U;
	g_speed_pi_last_sample_id = 0U;
	g_speed_pi_have_sample = 0U;
	g_speed_pi_last_target_direction = 0;
	g_speed_pi_feedback_valid = 0U;
	g_speed_pi_saturated = 0U;
	g_speed_pi_error_mps = 0.0f;
	g_speed_pi_integral = 0.0f;
	g_speed_pi_trim_us = 0;
}

static uint8_t uint32_to_u8_flag(uint32_t value)
{
	return (value != 0U) ? 1U : 0U;
}

static EscMotionEstimatorConfig_t servo_basic_read_esc_motion_config(void)
{
	EscMotionEstimatorConfig_t config;

	memset(&config, 0, sizeof(config));
	config.calibration_valid = uint32_to_u8_flag(g_esc_motion_calibration_valid);
	config.pole_pairs_valid = uint32_to_u8_flag(g_esc_motion_pole_pairs_valid);
	config.gear_ratio_valid = uint32_to_u8_flag(g_esc_motion_gear_ratio_valid);
	config.wheel_ratio_valid = uint32_to_u8_flag(g_esc_motion_wheel_ratio_valid);
	config.wheel_circumference_valid = uint32_to_u8_flag(g_esc_motion_wheel_circumference_valid);
	config.telemetry_timeout_valid = uint32_to_u8_flag(g_esc_motion_telemetry_timeout_valid);
	config.stopped_threshold_valid = uint32_to_u8_flag(g_esc_motion_stopped_threshold_valid);
	config.stopped_samples_valid = uint32_to_u8_flag(g_esc_motion_stopped_samples_valid);
	config.stopped_coverage_valid = uint32_to_u8_flag(g_esc_motion_stopped_coverage_valid);
	config.motor_pole_pairs = (uint16_t)g_esc_motion_motor_pole_pairs;
	config.gear_ratio = g_esc_motion_gear_ratio;
	config.wheel_ratio = g_esc_motion_wheel_ratio;
	config.wheel_circumference_m = g_esc_motion_wheel_circumference_m;
	config.telemetry_timeout_ms = g_esc_motion_telemetry_timeout_ms;
	config.stopped_speed_threshold_mps = g_esc_motion_stopped_speed_threshold_mps;
	config.stopped_min_samples = (uint8_t)g_esc_motion_stopped_min_samples;
	config.stopped_min_coverage_ms = g_esc_motion_stopped_min_coverage_ms;
	return config;
}

static Mode2DriveGateConfig_t servo_basic_read_mode2_config(void)
{
	Mode2DriveGateConfig_t config;

	memset(&config, 0, sizeof(config));
	config.calibration_valid = uint32_to_u8_flag(g_mode2_drive_calibration_valid);
	config.brake_calibration_valid = uint32_to_u8_flag(g_mode2_drive_brake_calibration_valid);
	config.first_strike_calibration_valid =
		uint32_to_u8_flag(g_mode2_drive_first_strike_calibration_valid);
	config.neutral_dwell_valid = uint32_to_u8_flag(g_mode2_drive_neutral_dwell_valid);
	config.reversal_timeout_valid = uint32_to_u8_flag(g_mode2_drive_reversal_timeout_valid);
	config.brake_request = g_mode2_drive_brake_request;
	config.reverse_first_strike_request = g_mode2_drive_reverse_first_strike_request;
	config.reverse_first_strike_min_ms = g_mode2_drive_reverse_first_strike_min_ms;
	config.neutral_dwell_ms = g_mode2_drive_neutral_dwell_ms;
	config.reversal_timeout_ms = g_mode2_drive_reversal_timeout_ms;
	return config;
}

static esc_tracking_brake_config_t servo_basic_read_tracking_brake_config(void)
{
	esc_tracking_brake_config_t config;

	config.valid = uint32_to_u8_flag(g_esc_tracking_brake_valid);
	config.kp = g_esc_tracking_brake_kp;
	config.max_request = g_esc_tracking_brake_max;
	config.enter_error_mps = g_esc_tracking_brake_enter_error_mps;
	config.release_error_mps = g_esc_tracking_brake_release_error_mps;
	return config;
}

static uint8_t tracking_brake_config_is_valid(const esc_tracking_brake_config_t *config)
{
	if (config == NULL)
	{
		return 0U;
	}

	return (config->valid != 0U &&
		isfinite(config->kp) && config->kp > 0.0f &&
		isfinite(config->max_request) &&
		config->max_request > 0.0f && config->max_request <= 1.0f &&
		isfinite(config->enter_error_mps) && config->enter_error_mps > 0.0f &&
		isfinite(config->release_error_mps) && config->release_error_mps >= 0.0f &&
		config->release_error_mps < config->enter_error_mps) ? 1U : 0U;
}

static uint8_t tracking_brake_config_equal(const esc_tracking_brake_config_t *left,
										   const esc_tracking_brake_config_t *right)
{
	return (left->valid == right->valid &&
		left->kp == right->kp &&
		left->max_request == right->max_request &&
		left->enter_error_mps == right->enter_error_mps &&
		left->release_error_mps == right->release_error_mps) ? 1U : 0U;
}

static uint8_t servo_basic_tick_is_after(uint32_t tick_ms, uint32_t reference_ms)
{
	const uint32_t delta_ms = tick_ms - reference_ms;

	return (delta_ms != 0U && delta_ms < 0x80000000UL) ? 1U : 0U;
}

static uint8_t servo_basic_tick_delta_ms(uint32_t tick_ms, uint32_t reference_ms, uint32_t *delta_ms)
{
	const uint32_t local_delta_ms = tick_ms - reference_ms;

	if (local_delta_ms >= 0x80000000UL)
	{
		return 0U;
	}
	if (delta_ms != NULL)
	{
		*delta_ms = local_delta_ms;
	}
	return 1U;
}

static void servo_basic_clear_vehicle_direction(void)
{
	s_vehicle_direction_known = 0U;
	s_vehicle_direction = 0;
}

static mode2_brake_pwm_runtime_config_t servo_basic_read_mode2_brake_pwm_config(void)
{
	mode2_brake_pwm_runtime_config_t config;

	config.valid = uint32_to_u8_flag(g_mode2_brake_pwm_valid);
	config.center_us = g_mode2_brake_pwm_center_us;
	config.full_us = g_mode2_brake_pwm_full_us;
	config.neutral_us = get_orin_esc_center_pulse();
	return config;
}

static uint8_t mode2_brake_pwm_runtime_config_equal(
	const mode2_brake_pwm_runtime_config_t *left,
	const mode2_brake_pwm_runtime_config_t *right)
{
	return (left->valid == right->valid &&
		left->center_us == right->center_us &&
		left->full_us == right->full_us &&
		left->neutral_us == right->neutral_us) ? 1U : 0U;
}

static uint8_t esc_motion_config_equal(const EscMotionEstimatorConfig_t *left,
									   const EscMotionEstimatorConfig_t *right)
{
	return (left->calibration_valid == right->calibration_valid &&
		left->pole_pairs_valid == right->pole_pairs_valid &&
		left->gear_ratio_valid == right->gear_ratio_valid &&
		left->wheel_ratio_valid == right->wheel_ratio_valid &&
		left->wheel_circumference_valid == right->wheel_circumference_valid &&
		left->telemetry_timeout_valid == right->telemetry_timeout_valid &&
		left->stopped_threshold_valid == right->stopped_threshold_valid &&
		left->stopped_samples_valid == right->stopped_samples_valid &&
		left->stopped_coverage_valid == right->stopped_coverage_valid &&
		left->motor_pole_pairs == right->motor_pole_pairs &&
		left->gear_ratio == right->gear_ratio &&
		left->wheel_ratio == right->wheel_ratio &&
		left->wheel_circumference_m == right->wheel_circumference_m &&
		left->telemetry_timeout_ms == right->telemetry_timeout_ms &&
		left->stopped_speed_threshold_mps == right->stopped_speed_threshold_mps &&
		left->stopped_min_samples == right->stopped_min_samples &&
		left->stopped_min_coverage_ms == right->stopped_min_coverage_ms) ? 1U : 0U;
}

static uint8_t mode2_config_equal(const Mode2DriveGateConfig_t *left,
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

static void servo_basic_clear_esc_observation_state(void)
{
	s_esc_feedback_available = 0U;
	s_esc_stop_confirmed = 0U;
	s_esc_sample_stale = 0U;
	s_esc_rx_invalidated = 0U;
	s_esc_receive_epoch_valid = 0U;
	s_esc_receive_epoch = 0U;
	s_esc_last_observed_sample_id = 0U;
	s_esc_last_observed_epoch = 0U;
	memset(&s_esc_latest_raw_sample, 0, sizeof(s_esc_latest_raw_sample));
	s_esc_latest_raw_sample_valid = 0U;
	memset(&s_esc_receiver_health, 0, sizeof(s_esc_receiver_health));
	s_vehicle_direction_known = 0U;
	s_vehicle_direction = 0;
	s_auto_history_boundary_valid = 0U;
	s_auto_history_boundary_ms = 0U;
	memset(&s_esc_motion_estimate, 0, sizeof(s_esc_motion_estimate));
	memset(&s_mode2_drive_output, 0, sizeof(s_mode2_drive_output));
	s_mode2_drive_output.action = MODE2_DRIVE_ACTION_NEUTRAL;
	g_esc_speed_limit_active = 0U;
	s_esc_speed_limit_over_count = 0U;
	s_esc_speed_limit_last_sample_id = 0U;
	s_esc_tracking_brake_active = 0U;
}

static void servo_basic_invalidate_auto_history(uint32_t now_ms)
{
	Mode2DriveGate_InvalidateAppliedHistory(&s_mode2_drive_gate);
	EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
		ESC_MOTION_APPLIED_ACTION_EXTERNAL_OVERRIDE,
		now_ms);
	s_auto_history_boundary_valid = 1U;
	s_auto_history_boundary_ms = now_ms;
	speed_pi_reset_controller();
	g_esc_speed_limit_active = 0U;
	s_esc_speed_limit_over_count = 0U;
	s_esc_speed_limit_last_sample_id = 0U;
	s_esc_tracking_brake_active = 0U;
}

static void servo_basic_refresh_esc_configs(uint32_t now_ms)
{
	const EscMotionEstimatorConfig_t motion_config =
		servo_basic_read_esc_motion_config();
	const Mode2DriveGateConfig_t mode2_config =
		servo_basic_read_mode2_config();
	const esc_tracking_brake_config_t tracking_brake_config =
		servo_basic_read_tracking_brake_config();
	const mode2_brake_pwm_runtime_config_t brake_pwm_config =
		servo_basic_read_mode2_brake_pwm_config();
	uint8_t changed = 0U;

	if (s_esc_motion_config_initialized == 0U)
	{
		EscMotionEstimator_Init(&s_esc_motion_estimator, &motion_config);
		s_esc_motion_config = motion_config;
		s_esc_motion_config_initialized = 1U;
		changed = 1U;
	}
	else if (esc_motion_config_equal(&s_esc_motion_config, &motion_config) == 0U)
	{
		(void)EscMotionEstimator_SetConfig(&s_esc_motion_estimator, &motion_config);
		s_esc_motion_config = motion_config;
		changed = 1U;
	}

	if (s_mode2_drive_config_initialized == 0U)
	{
		Mode2DriveGate_Init(&s_mode2_drive_gate, &mode2_config);
		s_mode2_drive_config = mode2_config;
		s_mode2_drive_config_initialized = 1U;
		changed = 1U;
	}
	else if (mode2_config_equal(&s_mode2_drive_config, &mode2_config) == 0U)
	{
		(void)Mode2DriveGate_SetConfig(&s_mode2_drive_gate, &mode2_config);
		s_mode2_drive_config = mode2_config;
		changed = 1U;
	}

	if (s_esc_tracking_brake_config_initialized == 0U)
	{
		s_esc_tracking_brake_config = tracking_brake_config;
		s_esc_tracking_brake_config_initialized = 1U;
		changed = 1U;
	}
	else if (tracking_brake_config_equal(&s_esc_tracking_brake_config,
										 &tracking_brake_config) == 0U)
	{
		s_esc_tracking_brake_config = tracking_brake_config;
		changed = 1U;
	}

	if (s_mode2_brake_pwm_config_initialized == 0U)
	{
		s_mode2_brake_pwm_config = brake_pwm_config;
		s_mode2_brake_pwm_config_initialized = 1U;
		changed = 1U;
	}
	else if (mode2_brake_pwm_runtime_config_equal(&s_mode2_brake_pwm_config,
												  &brake_pwm_config) == 0U)
	{
		s_mode2_brake_pwm_config = brake_pwm_config;
		changed = 1U;
	}

	if (changed != 0U)
	{
		servo_basic_invalidate_auto_history(now_ms);
		s_esc_rx_invalidated = 1U;
	}
}

static void servo_basic_update_cached_estimate(uint32_t now_ms)
{
	uint8_t sample_after_boundary = 1U;

	s_esc_motion_estimate =
		EscMotionEstimator_GetEstimate(&s_esc_motion_estimator, now_ms);
	s_esc_sample_stale =
		(s_esc_motion_estimate.reason == ESC_MOTION_REASON_SAMPLE_STALE) ? 1U : 0U;
	if (s_auto_history_boundary_valid != 0U)
	{
		sample_after_boundary =
			(s_esc_motion_estimate.has_sample != 0U &&
			 servo_basic_tick_is_after(s_esc_motion_estimate.last_sample_tick_ms,
				s_auto_history_boundary_ms) != 0U) ? 1U : 0U;
	}
	s_esc_feedback_available =
		(s_esc_rx_invalidated == 0U &&
		 sample_after_boundary != 0U &&
		 s_esc_motion_estimate.config_valid != 0U &&
		 s_esc_motion_estimate.has_sample != 0U &&
		 s_esc_motion_estimate.sample_fresh != 0U &&
		 s_esc_motion_estimate.rpm_valid != 0U &&
		 s_esc_motion_estimate.magnitude_valid != 0U) ? 1U : 0U;
	s_esc_stop_confirmed =
		(s_esc_feedback_available != 0U &&
		 s_esc_motion_estimate.stopped != 0U) ? 1U : 0U;
	if (s_esc_stop_confirmed != 0U)
	{
		servo_basic_clear_vehicle_direction();
	}
	if (s_esc_feedback_available == 0U)
	{
		g_esc_speed_limit_active = 0U;
		s_esc_speed_limit_over_count = 0U;
		s_esc_speed_limit_last_sample_id = 0U;
		s_esc_tracking_brake_active = 0U;
	}
}

static void servo_basic_update_esc_feedback(uint32_t now_ms)
{
	EscTelemetrySnapshot_t snapshot;
	uint8_t new_sample = 0U;

	s_esc_rx_invalidated = 0U;
	s_esc_sample_stale = 0U;
	EscTelemetry_GetReceiverHealth(&s_esc_receiver_health);

	if (EscTelemetry_GetSnapshot(&snapshot) == 0U || snapshot.has_sample == 0U)
	{
		s_esc_rx_invalidated = 1U;
		s_esc_latest_raw_sample_valid = 0U;
		servo_basic_invalidate_auto_history(now_ms);
		servo_basic_update_cached_estimate(now_ms);
		return;
	}
	s_esc_latest_raw_sample = snapshot.sample;
	s_esc_latest_raw_sample_valid = 1U;

	if (s_esc_receive_epoch_valid == 0U)
	{
		s_esc_receive_epoch_valid = 1U;
		s_esc_receive_epoch = snapshot.receive_epoch;
	}
	else if (snapshot.receive_epoch != s_esc_receive_epoch)
	{
		s_esc_receive_epoch = snapshot.receive_epoch;
		s_esc_last_observed_sample_id = 0U;
		s_esc_last_observed_epoch = snapshot.receive_epoch;
		s_esc_rx_invalidated = 1U;
		servo_basic_invalidate_auto_history(now_ms);
		servo_basic_update_cached_estimate(now_ms);
		return;
	}

	if (s_esc_last_observed_sample_id != snapshot.sample.sample_id ||
		s_esc_last_observed_epoch != snapshot.receive_epoch)
	{
		new_sample = 1U;
	}

	if (new_sample != 0U)
	{
		(void)EscMotionEstimator_ObserveSample(&s_esc_motion_estimator,
			&snapshot.sample,
			now_ms);
		s_esc_last_observed_sample_id = snapshot.sample.sample_id;
		s_esc_last_observed_epoch = snapshot.receive_epoch;
	}

	servo_basic_update_cached_estimate(now_ms);
	if (s_esc_feedback_available == 0U)
	{
		servo_basic_invalidate_auto_history(now_ms);
		servo_basic_update_cached_estimate(now_ms);
	}
}

static float clamp_speed_pi_param_float(float value, float max_value)
{
	if (value < 0.0f)
	{
		return 0.0f;
	}
	if (value > max_value)
	{
		return max_value;
	}
	return value;
}

void ServoBasic_ResetSpeedPi(void)
{
	speed_pi_reset_controller();
}

void ServoBasic_SetSpeedPiEnable(uint8_t enable)
{
	g_speed_pi_enable = (enable != 0U) ? 1U : 0U;
	speed_pi_reset_controller();
}

void ServoBasic_SetSpeedPiKp(float kp_us_per_mps)
{
	g_speed_pi_kp = clamp_speed_pi_param_float(kp_us_per_mps, SPEED_PI_KP_MAX_US_PER_MPS);
	speed_pi_reset_controller();
}

void ServoBasic_SetSpeedPiKi(float ki_us_per_mps_s)
{
	g_speed_pi_ki = clamp_speed_pi_param_float(ki_us_per_mps_s, SPEED_PI_KI_MAX_US_PER_MPS_S);
	speed_pi_reset_controller();
}

void ServoBasic_SetSpeedPiTrimLimitUs(uint32_t limit_us)
{
	if (limit_us > SPEED_PI_TRIM_LIMIT_PARAM_MAX_US)
	{
		limit_us = SPEED_PI_TRIM_LIMIT_PARAM_MAX_US;
	}
	g_speed_pi_trim_limit_us = limit_us;
	speed_pi_reset_controller();
}

static void apply_servo_pulse(uint16_t pulse_us)
{
	g_state.servo_pulse_us = pulse_us;
	ServoBasic_OutputServoPulse(pulse_us);
}

static void set_esc_target(uint16_t pulse_us)
{
	g_state.esc_pulse_us = pulse_us;
}

static void set_servo_target(uint16_t pulse_us)
{
	g_state.servo_pulse_us = pulse_us;
}

void ServoBasic_Init(void)
{
	g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
	g_state.rc_takeover_pending = 0U;
	g_state.emergency_stop = 0U;
	g_rc_override_active = 0U;
	g_rc_guard_active = 0U;
	g_rc_override_enter_count = 0U;
	g_rc_override_release_start_ms = 0U;
	g_rc_override_release_hold_required = 0U;
	g_rc_throttle_present = 0U;
	g_rc_steering_present = 0U;
	g_rc_guard_present = 0U;
	rc_debounce_reset();
	g_orin_state.esc_pulse_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_orin_state.servo_pulse_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_orin_state.last_update_ms = 0U;
	g_orin_state.target_speed_mps = 0.0f;
	g_orin_state.feedback_vx_mps = 0.0f;
	g_orin_state.feedback_vz_rad_s = 0.0f;
	g_orin_state.target_steering_angle_rad = 0.0f;
	g_orin_state.feedback_steering_angle_rad = 0.0f;
	g_orin_state.active = 0U;
	g_orin_state.stop = 0U;
	g_orin_state.auto_enabled = 0U;
	g_orin_state.brake_active = 0U;
	g_orin_state.emergency_stop = 0U;
	g_orin_state.speed_saturated = 0U;
	g_orin_state.steering_saturated = 0U;
	g_orin_state.accel_limited = 0U;
	g_orin_state.steering_rate_limited = 0U;
	g_speed_pi_target_vx_mps = 0.0f;
	g_speed_pi_feedback_vx_mps = 0.0f;
	g_speed_pi_base_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_speed_pi_final_us = ESC_PWM_NEUTRAL_PULSE_US;
	s_esc_motion_config_initialized = 0U;
	s_mode2_drive_config_initialized = 0U;
	s_esc_tracking_brake_config_initialized = 0U;
	s_mode2_brake_pwm_config_initialized = 0U;
	servo_basic_clear_esc_observation_state();
	servo_basic_refresh_esc_configs(HAL_GetTick());
	speed_pi_reset_controller();
	ServoRC_Capture_Init();
	apply_esc_pulse(get_orin_esc_center_pulse());
	apply_servo_pulse(get_orin_servo_center_pulse());
	s_control_snapshot_next_sequence = 0U;
	servo_basic_publish_control_snapshot(HAL_GetTick());
}

static uint16_t orin_map_limited_vx_to_esc(float vx_mps)
{
	uint32_t speed_mmps;

	if (orin_target_is_below_min_control(vx_mps) != 0U)
	{
		return get_orin_esc_center_pulse();
	}

	speed_mmps = speed_mps_abs_to_mmps(vx_mps);
	if (vx_mps > 0.0f)
	{
		return interpolate_speed_ff_table(s_orin_forward_ff_table,
			(uint32_t)(sizeof(s_orin_forward_ff_table) / sizeof(s_orin_forward_ff_table[0])),
			speed_mmps);
	}
	return interpolate_speed_ff_table(s_orin_reverse_ff_table,
		(uint32_t)(sizeof(s_orin_reverse_ff_table) / sizeof(s_orin_reverse_ff_table[0])),
		speed_mmps);
}

static int32_t float_to_i32_nearest(float value)
{
	if (value >= 0.0f)
	{
		return (int32_t)(value + 0.5f);
	}
	return (int32_t)(value - 0.5f);
}

static uint16_t clamp_esc_pulse_i32(int32_t pulse_us)
{
	if (pulse_us < (int32_t)ESC_PWM_MIN_PULSE_US)
	{
		return ESC_PWM_MIN_PULSE_US;
	}
	if (pulse_us > (int32_t)ESC_PWM_MAX_PULSE_US)
	{
		return ESC_PWM_MAX_PULSE_US;
	}
	return (uint16_t)pulse_us;
}

static uint32_t get_speed_pi_trim_limit_us(void)
{
	uint32_t limit_us = (g_speed_pi_trim_limit_us == 0U) ?
		SPEED_PI_TRIM_LIMIT_DEFAULT_US : g_speed_pi_trim_limit_us;

	if (limit_us > 100U)
	{
		limit_us = 100U;
	}
	return limit_us;
}

static uint16_t orin_compute_speed_control_esc(Mode2DriveAction_t action)
{
	const float target_vx_mps = g_orin_state.target_speed_mps;
	const uint16_t base_us = orin_map_limited_vx_to_esc(target_vx_mps);
	const int8_t target_direction = get_vx_direction(target_vx_mps);
	float feedback_vx_mps = 0.0f;
	uint8_t feedback_valid = 0U;
	uint8_t new_feedback_sample = 0U;

	g_speed_pi_target_vx_mps = target_vx_mps;
	g_speed_pi_base_us = base_us;

	if (orin_target_is_below_min_control(target_vx_mps) != 0U ||
		(action != MODE2_DRIVE_ACTION_FORWARD &&
		 action != MODE2_DRIVE_ACTION_REVERSE))
	{
		g_speed_pi_feedback_vx_mps = 0.0f;
		g_speed_pi_final_us = get_orin_esc_center_pulse();
		speed_pi_reset_controller();
		return get_orin_esc_center_pulse();
	}

	if (g_speed_pi_last_target_direction != 0 && target_direction != g_speed_pi_last_target_direction)
	{
		speed_pi_reset_controller();
	}
	g_speed_pi_last_target_direction = target_direction;

	if (s_esc_feedback_available != 0U &&
		s_esc_motion_estimate.signed_speed_valid != 0U)
	{
		feedback_vx_mps = s_esc_motion_estimate.signed_speed_mps;
		feedback_valid = 1U;
	}
	else if (s_esc_stop_confirmed != 0U)
	{
		feedback_vx_mps = 0.0f;
		feedback_valid = 1U;
	}
	g_speed_pi_feedback_valid = feedback_valid;
	g_speed_pi_feedback_vx_mps = feedback_vx_mps;

	if (g_speed_pi_enable == 0U || feedback_valid == 0U)
	{
		const uint16_t final_pulse = limit_esc_safe_pulse(base_us);

		g_speed_pi_error_mps = 0.0f;
		g_speed_pi_trim_us = 0;
		g_speed_pi_saturated = (final_pulse != base_us) ? 1U : 0U;
		g_speed_pi_final_us = final_pulse;
		if (feedback_valid == 0U)
		{
			g_speed_pi_last_update_ms = 0U;
			g_speed_pi_integral = 0.0f;
		}
		return final_pulse;
	}

	{
		float dt_s = 0.0f;
		const float kp = g_speed_pi_kp;
		const float ki = g_speed_pi_ki;
		const float trim_limit = (float)get_speed_pi_trim_limit_us();
		float error_mps;
		float integral;
		float trim_us;
		int32_t trim_i32;
		int32_t final_i32;

		new_feedback_sample = (g_speed_pi_have_sample == 0U ||
			g_speed_pi_last_sample_id != s_esc_motion_estimate.last_sample_id) ? 1U : 0U;
		if (new_feedback_sample != 0U && g_speed_pi_have_sample != 0U)
		{
			const uint32_t dt_ms = s_esc_motion_estimate.last_sample_tick_ms -
				g_speed_pi_last_update_ms;
			dt_s = (float)dt_ms / 1000.0f;
			if (dt_s > 0.100f)
			{
				dt_s = 0.100f;
			}
		}
		if (new_feedback_sample != 0U)
		{
			g_speed_pi_last_update_ms = s_esc_motion_estimate.last_sample_tick_ms;
			g_speed_pi_last_sample_id = s_esc_motion_estimate.last_sample_id;
			g_speed_pi_have_sample = 1U;
		}

		error_mps = target_vx_mps - feedback_vx_mps;
		integral = (new_feedback_sample != 0U) ?
			(g_speed_pi_integral + error_mps * dt_s) :
			g_speed_pi_integral;
		trim_us = kp * error_mps + ki * integral;
		g_speed_pi_saturated = 0U;

		if (trim_us > trim_limit)
		{
			trim_us = trim_limit;
			g_speed_pi_saturated = 1U;
			if (ki > 0.001f)
			{
				integral = (trim_limit - kp * error_mps) / ki;
			}
		}
		else if (trim_us < -trim_limit)
		{
			trim_us = -trim_limit;
			g_speed_pi_saturated = 1U;
			if (ki > 0.001f)
			{
				integral = (-trim_limit - kp * error_mps) / ki;
			}
		}

		trim_i32 = float_to_i32_nearest(trim_us);
		final_i32 = (int32_t)base_us + trim_i32;
		if (target_direction > 0 && final_i32 < (int32_t)get_orin_esc_center_pulse())
		{
			final_i32 = (int32_t)get_orin_esc_center_pulse();
			g_speed_pi_saturated = 1U;
		}
		else if (target_direction < 0 && final_i32 > (int32_t)get_orin_esc_center_pulse())
		{
			final_i32 = (int32_t)get_orin_esc_center_pulse();
			g_speed_pi_saturated = 1U;
		}
		if (final_i32 < (int32_t)ESC_PWM_MIN_PULSE_US || final_i32 > (int32_t)ESC_PWM_MAX_PULSE_US)
		{
			g_speed_pi_saturated = 1U;
		}

		g_speed_pi_error_mps = error_mps;
		g_speed_pi_integral = integral;
		g_speed_pi_trim_us = trim_i32;
		{
			const uint16_t hard_limited_pulse = clamp_esc_pulse_i32(final_i32);
			const uint16_t final_pulse = limit_esc_safe_pulse(hard_limited_pulse);

			if (final_pulse != hard_limited_pulse)
			{
				g_speed_pi_saturated = 1U;
			}
			g_speed_pi_final_us = final_pulse;
		}
		return (uint16_t)g_speed_pi_final_us;
	}
}

static Mode2DriveTargetDirection_t mode2_target_direction_from_vx(float vx_mps)
{
	if (orin_target_is_below_min_control(vx_mps) != 0U)
	{
		return MODE2_DRIVE_TARGET_NEUTRAL;
	}
	return (vx_mps > 0.0f) ? MODE2_DRIVE_TARGET_FORWARD : MODE2_DRIVE_TARGET_REVERSE;
}

static uint8_t mode2_brake_pwm_config_is_valid(void)
{
	const uint32_t center_us = g_mode2_brake_pwm_center_us;
	const uint32_t full_us = g_mode2_brake_pwm_full_us;

	return (g_mode2_brake_pwm_valid != 0U &&
		center_us == (uint32_t)get_orin_esc_center_pulse() &&
		center_us >= ESC_PWM_MIN_PULSE_US &&
		center_us <= ESC_PWM_MAX_PULSE_US &&
		full_us >= ESC_PWM_MIN_PULSE_US &&
		full_us <= ESC_PWM_MAX_PULSE_US &&
		full_us < center_us) ? 1U : 0U;
}

static uint8_t servo_basic_mode2_application_config_valid(void)
{
	return (s_mode2_drive_gate.config_valid != 0U &&
		mode2_brake_pwm_config_is_valid() != 0U &&
		tracking_brake_config_is_valid(&s_esc_tracking_brake_config) != 0U) ? 1U : 0U;
}

static uint16_t mode2_brake_request_to_pwm(float request,
										   float *actual_normalized_brake)
{
	const uint16_t center_us = (uint16_t)get_orin_esc_center_pulse();
	int32_t final_us;
	int32_t brake_span_us;
	int32_t actual_delta_us;

	if (actual_normalized_brake != NULL)
	{
		*actual_normalized_brake = 0.0f;
	}
	request = clamp_unit_float(request);
	if (request <= 0.0f || mode2_brake_pwm_config_is_valid() == 0U)
	{
		return center_us;
	}

	brake_span_us = (int32_t)g_mode2_brake_pwm_full_us -
		(int32_t)center_us;
	final_us = (int32_t)center_us +
		float_to_i32_nearest((float)brake_span_us * request);
	final_us = (int32_t)clamp_esc_pulse_i32(final_us);
	if (final_us >= (int32_t)center_us)
	{
		final_us = (int32_t)center_us;
		actual_delta_us = 0;
	}
	else
	{
		actual_delta_us = (int32_t)center_us - final_us;
	}
	if (actual_normalized_brake != NULL)
	{
		const int32_t span_abs_us = (int32_t)center_us -
			(int32_t)g_mode2_brake_pwm_full_us;
		*actual_normalized_brake = (actual_delta_us == 0 || span_abs_us <= 0) ? 0.0f :
			clamp_unit_float((float)actual_delta_us / (float)span_abs_us);
	}

	return (uint16_t)final_us;
}

static Mode2DriveAction_t mode2_actual_action_from_output(Mode2DriveAction_t requested_action,
														  uint16_t final_esc_pulse,
														  float actual_normalized_brake)
{
	const uint16_t center_us = (uint16_t)get_orin_esc_center_pulse();

	switch (requested_action)
	{
	case MODE2_DRIVE_ACTION_FORWARD:
		return (final_esc_pulse > center_us) ?
			MODE2_DRIVE_ACTION_FORWARD : MODE2_DRIVE_ACTION_NEUTRAL;
	case MODE2_DRIVE_ACTION_REVERSE:
		return (final_esc_pulse < center_us) ?
			MODE2_DRIVE_ACTION_REVERSE : MODE2_DRIVE_ACTION_NEUTRAL;
	case MODE2_DRIVE_ACTION_BRAKE:
		return (final_esc_pulse < center_us && actual_normalized_brake > 0.0f) ?
			MODE2_DRIVE_ACTION_BRAKE : MODE2_DRIVE_ACTION_NEUTRAL;
	case MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE:
		return (final_esc_pulse < center_us && actual_normalized_brake > 0.0f) ?
			MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE : MODE2_DRIVE_ACTION_NEUTRAL;
	case MODE2_DRIVE_ACTION_NEUTRAL:
	default:
		return MODE2_DRIVE_ACTION_NEUTRAL;
	}
}

static uint16_t mode2_drive_output_to_esc_pwm(Mode2DriveAction_t action,
											  float normalized_brake_request,
											  float *actual_normalized_brake)
{
	if (actual_normalized_brake != NULL)
	{
		*actual_normalized_brake = 0.0f;
	}

	switch (action)
	{
	case MODE2_DRIVE_ACTION_FORWARD:
	case MODE2_DRIVE_ACTION_REVERSE:
		return orin_compute_speed_control_esc(action);
	case MODE2_DRIVE_ACTION_BRAKE:
	case MODE2_DRIVE_ACTION_REVERSE_FIRST_STRIKE:
		speed_pi_reset_controller();
		return mode2_brake_request_to_pwm(normalized_brake_request,
			actual_normalized_brake);
	case MODE2_DRIVE_ACTION_NEUTRAL:
	default:
		speed_pi_reset_controller();
		return get_orin_esc_center_pulse();
	}
}

static void servo_basic_commit_auto_action(Mode2DriveAction_t requested_action,
										   uint16_t final_esc_pulse,
										   float actual_normalized_brake,
										   uint32_t now_ms)
{
	const Mode2DriveAction_t actual_action =
		mode2_actual_action_from_output(requested_action,
			final_esc_pulse,
			actual_normalized_brake);

	Mode2DriveGate_CommitAppliedActionWithEvidence(&s_mode2_drive_gate,
		actual_action,
		now_ms,
		actual_normalized_brake);
	EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
		Mode2DriveGate_ToMotionAction(actual_action),
		now_ms);
	if (actual_action == MODE2_DRIVE_ACTION_FORWARD)
	{
		s_vehicle_direction_known = 1U;
		s_vehicle_direction = 1;
	}
	else if (actual_action == MODE2_DRIVE_ACTION_REVERSE)
	{
		s_vehicle_direction_known = 1U;
		s_vehicle_direction = -1;
	}
}

static uint8_t servo_basic_auto_propulsion_authorized(void)
{
	return (g_orin_state.auto_enabled != 0U &&
		s_esc_feedback_available != 0U &&
		s_esc_motion_estimator.config_valid != 0U &&
		servo_basic_mode2_application_config_valid() != 0U) ? 1U : 0U;
}

static float clamp_orin_steering_angle(float steering_angle_rad)
{
	const float max_steering_rad = (float)get_orin_ackermann_max_steering_millirad() / 1000.0f;

	if (max_steering_rad <= 0.001f)
	{
		return 0.0f;
	}
	if (steering_angle_rad > max_steering_rad)
	{
		return max_steering_rad;
	}
	if (steering_angle_rad < -max_steering_rad)
	{
		return -max_steering_rad;
	}
	return steering_angle_rad;
}

static uint16_t orin_map_steering_to_servo(float steering_angle_rad)
{
	const float max_steering_rad = (float)get_orin_ackermann_max_steering_millirad() / 1000.0f;
	float ratio;

	if (max_steering_rad <= 0.001f)
	{
		return get_orin_servo_center_pulse();
	}

	steering_angle_rad = clamp_orin_steering_angle(steering_angle_rad);
	ratio = (steering_angle_rad / max_steering_rad) *
		(float)APP_ORIN_STEERING_PWM_DIRECTION_SIGN;
	return limit_servo_safe_pulse((uint16_t)((int32_t)get_orin_servo_center_pulse() +
			(int32_t)(ratio * (float)((g_orin_servo_range_us == 0U) ? ORIN_SERVO_RANGE_DEFAULT_US : g_orin_servo_range_us))));
}

static float telemetry_estimate_steering_rad_from_servo_pulse(uint16_t pulse_us)
{
	const uint16_t center_pulse = get_orin_servo_center_pulse();
	const uint16_t pulse_range = (g_orin_servo_range_us == 0U) ? ORIN_SERVO_RANGE_DEFAULT_US : (uint16_t)g_orin_servo_range_us;
	const float max_steering_rad = (float)get_orin_ackermann_max_steering_millirad() / 1000.0f;
	float ratio;

	if (pulse_us == 0U || pulse_range == 0U || max_steering_rad <= 0.001f)
	{
		return 0.0f;
	}

	ratio = (float)((int32_t)pulse_us - (int32_t)center_pulse) / (float)pulse_range;
	if (ratio > 1.0f)
	{
		ratio = 1.0f;
	}
	else if (ratio < -1.0f)
	{
		ratio = -1.0f;
	}
	return ratio * (float)APP_ORIN_STEERING_PWM_DIRECTION_SIGN *
		max_steering_rad;
}

static float telemetry_estimate_vz_from_pwm(float vx_mps, uint16_t servo_pulse_us)
{
	const float min_vx_mps = (float)get_orin_ackermann_min_vx_mmps() / 1000.0f;
	const float wheelbase_m = (float)get_orin_ackermann_wheelbase_mm() / 1000.0f;
	const float steering_rad = telemetry_estimate_steering_rad_from_servo_pulse(servo_pulse_us);

	if (fabsf(vx_mps) < min_vx_mps || fabsf(steering_rad) < 0.001f || wheelbase_m <= 0.001f)
	{
		return 0.0f;
	}

	return tanf(steering_rad) * vx_mps / wheelbase_m;
}

static void update_ackermann_from_orin(float speed_mps,
									   float steering_angle_rad,
									   uint8_t enable,
									   uint8_t brake,
									   uint8_t emergency_stop)
{
	float requested_speed_mps;
	float limited_speed_mps;
	float clamped_steering_angle_rad;
	float feedback_speed_mps;
	float dt_s = 0.0f;
	uint32_t now_ms;
	uint8_t speed_below_min;
	uint8_t accel_limited = 0U;
	uint8_t steering_saturated = 0U;
	uint8_t steering_rate_limited = 0U;
	const uint8_t auto_enabled = (enable != 0U) ? 1U : 0U;
	const uint8_t brake_active = (brake != 0U) ? 1U : 0U;
	const uint8_t estop_active = (emergency_stop != 0U) ? 1U : 0U;
	const uint8_t force_zero_speed = (auto_enabled == 0U || brake_active != 0U || estop_active != 0U) ? 1U : 0U;

	(void)get_orin_ackermann_track_width_mm();
	(void)get_orin_ackermann_wheel_radius_mm();
	if (g_orin_pwm_enable == 0U)
	{
		HallSpeed_SetCommandDirection(0);
		speed_pi_reset_controller();
			return;
		}

	now_ms = HAL_GetTick();
	if (g_orin_state.active != 0U && now_ms >= g_orin_state.last_update_ms)
	{
		dt_s = (float)(now_ms - g_orin_state.last_update_ms) / 1000.0f;
		if (dt_s > 0.250f)
		{
			dt_s = 0.250f;
		}
	}

	clamped_steering_angle_rad = clamp_orin_steering_angle(steering_angle_rad);
	steering_saturated = (fabsf(clamped_steering_angle_rad - steering_angle_rad) > 0.0005f) ? 1U : 0U;
	requested_speed_mps = (force_zero_speed != 0U) ? 0.0f : scale_and_limit_orin_vx(speed_mps);
	limited_speed_mps = requested_speed_mps;
	if (force_zero_speed == 0U && g_orin_state.active != 0U && dt_s > 0.0f)
	{
		limited_speed_mps = apply_rate_limit_float(
			requested_speed_mps,
			g_orin_state.target_speed_mps,
			(float)get_orin_accel_limit_mmps2() / 1000.0f,
			dt_s,
			&accel_limited);
		clamped_steering_angle_rad = apply_rate_limit_float(
			clamped_steering_angle_rad,
			g_orin_state.target_steering_angle_rad,
			(float)get_orin_steering_rate_limit_mradps() / 1000.0f,
			dt_s,
			&steering_rate_limited);
	}

	speed_below_min = orin_target_is_below_min_control(limited_speed_mps);
	feedback_speed_mps = (speed_below_min != 0U ||
		fabsf(limited_speed_mps) < get_orin_velocity_neutral_threshold_mps()) ? 0.0f : limited_speed_mps;

	if (estop_active != 0U)
	{
		g_orin_state.esc_pulse_us = get_orin_esc_center_pulse();
		g_orin_state.servo_pulse_us = get_orin_servo_center_pulse();
		speed_pi_reset_controller();
	}
		else
		{
			if (force_zero_speed != 0U || speed_below_min != 0U)
			{
				g_orin_state.esc_pulse_us = get_orin_esc_center_pulse();
				speed_pi_reset_controller();
			}
			else
			{
				g_orin_state.esc_pulse_us = orin_map_limited_vx_to_esc(limited_speed_mps);
			}
			g_orin_state.servo_pulse_us = orin_map_steering_to_servo(clamped_steering_angle_rad);
		}

	g_orin_state.target_speed_mps = (force_zero_speed != 0U) ? 0.0f : limited_speed_mps;
	if (s_esc_feedback_available != 0U &&
		s_esc_motion_estimate.signed_speed_valid != 0U)
	{
		feedback_speed_mps = s_esc_motion_estimate.signed_speed_mps;
	}
	else if (s_esc_stop_confirmed != 0U)
	{
		feedback_speed_mps = 0.0f;
	}
	else
	{
		feedback_speed_mps = 0.0f;
	}
	g_orin_state.feedback_vx_mps = feedback_speed_mps;
	g_orin_state.feedback_steering_angle_rad =
		telemetry_estimate_steering_rad_from_servo_pulse(g_orin_state.servo_pulse_us);
	g_orin_state.feedback_vz_rad_s = telemetry_estimate_vz_from_pwm(
		feedback_speed_mps,
		g_orin_state.servo_pulse_us);
	g_orin_state.target_steering_angle_rad = clamped_steering_angle_rad;
	g_orin_state.last_update_ms = now_ms;
	g_orin_state.active = 1U;
	g_orin_state.stop = estop_active;
	g_orin_state.auto_enabled = auto_enabled;
	g_orin_state.brake_active = brake_active;
	g_orin_state.emergency_stop = estop_active;
	g_orin_state.speed_saturated = (force_zero_speed == 0U && orin_vx_would_saturate(speed_mps) != 0U) ? 1U : 0U;
	g_orin_state.steering_saturated = steering_saturated;
	g_orin_state.accel_limited = accel_limited;
	g_orin_state.steering_rate_limited = steering_rate_limited;
}

void ServoBasic_UpdateAckermannFromOrin(float speed_mps,
										float steering_angle_rad,
										uint8_t enable,
										uint8_t brake,
										uint8_t emergency_stop)
{
	update_ackermann_from_orin(speed_mps, steering_angle_rad, enable, brake, emergency_stop);
}

static uint8_t orin_pwm_is_active_at(uint32_t now_ms)
{
	if (g_orin_pwm_enable == 0U || g_orin_state.active == 0U)
	{
		return 0U;
	}
	const uint32_t timeout_ms = (g_orin_pwm_timeout_ms == 0U) ? APP_ORIN_PWM_TIMEOUT_DEFAULT_MS :
		g_orin_pwm_timeout_ms;
	return (now_ms - g_orin_state.last_update_ms) <= timeout_ms;
}

static uint8_t orin_pwm_is_active(void)
{
	return orin_pwm_is_active_at(HAL_GetTick());
}

static uint8_t servo_basic_esc_fe32_is_fresh(uint32_t now_ms)
{
	uint32_t age_ms;

	if (s_esc_latest_raw_sample_valid == 0U)
	{
		return 0U;
	}
	if (servo_basic_tick_delta_ms(now_ms,
								  s_esc_latest_raw_sample.received_tick_ms,
								  &age_ms) == 0U)
	{
		return 0U;
	}
	return (age_ms <= ESC_DIAGNOSTIC_FRESH_TIMEOUT_MS) ? 1U : 0U;
}

static uint8_t servo_basic_uplink_speed_calibration_is_valid(void)
{
	return (g_esc_speed_calibration_valid != 0U &&
		isfinite(g_esc_low_gear_wheel_rpm_per_raw) &&
		g_esc_low_gear_wheel_rpm_per_raw > 0.0f &&
		get_orin_ackermann_wheel_radius_mm() != 0U &&
		g_esc_speed_fresh_timeout_ms != 0U &&
		g_esc_speed_fresh_timeout_ms < 0x80000000UL) ? 1U : 0U;
}

static uint8_t servo_basic_compute_uplink_speed_magnitude(uint32_t now_ms,
														  float *speed_magnitude_mps)
{
	uint32_t age_ms;
	float wheel_axle_rpm;
	float wheel_circumference_m;
	float speed_mps;

	if (speed_magnitude_mps != NULL)
	{
		*speed_magnitude_mps = 0.0f;
	}
	if (servo_basic_uplink_speed_calibration_is_valid() == 0U ||
		servo_basic_esc_fe32_is_fresh(now_ms) == 0U ||
		s_esc_rx_invalidated != 0U ||
		s_esc_latest_raw_sample_valid == 0U ||
		s_esc_latest_raw_sample.rpm_valid == 0U ||
		servo_basic_tick_delta_ms(now_ms,
			s_esc_latest_raw_sample.received_tick_ms,
			&age_ms) == 0U ||
		age_ms > g_esc_speed_fresh_timeout_ms)
	{
		return 0U;
	}

	wheel_axle_rpm = (float)s_esc_latest_raw_sample.rpm_raw *
		g_esc_low_gear_wheel_rpm_per_raw;
	wheel_circumference_m = 2.0f * SERVO_BASIC_PI_F *
		((float)get_orin_ackermann_wheel_radius_mm() / 1000.0f);
	speed_mps = wheel_axle_rpm *
		wheel_circumference_m / 60.0f;
	if (isfinite(speed_mps) == 0 || speed_mps < 0.0f)
	{
		return 0U;
	}
	if (speed_magnitude_mps != NULL)
	{
		*speed_magnitude_mps = speed_mps;
	}
	return 1U;
}

static int8_t servo_basic_estimated_vehicle_direction(void)
{
	if (s_vehicle_direction_known == 0U)
	{
		return 0;
	}
	return s_vehicle_direction;
}

static uint8_t servo_basic_collect_esc_uplink_speed(
	uint32_t now_ms,
	float *speed_mps,
	float *speed_magnitude_mps,
	uint8_t *direction_known)
{
	const int8_t direction = servo_basic_estimated_vehicle_direction();
	float local_speed_magnitude_mps = 0.0f;

	if (speed_mps != NULL)
	{
		*speed_mps = 0.0f;
	}
	if (speed_magnitude_mps != NULL)
	{
		*speed_magnitude_mps = 0.0f;
	}
	if (direction_known != NULL)
	{
		*direction_known = (direction != 0) ? 1U : 0U;
	}

	if (servo_basic_compute_uplink_speed_magnitude(now_ms,
			&local_speed_magnitude_mps) == 0U ||
		s_esc_stop_confirmed != 0U)
	{
		return 0U;
	}

	if (speed_magnitude_mps != NULL)
	{
		*speed_magnitude_mps = local_speed_magnitude_mps;
	}
	if (speed_mps != NULL)
	{
		*speed_mps = (direction < 0) ?
			-local_speed_magnitude_mps : local_speed_magnitude_mps;
	}
	return 1U;
}

static uint8_t servo_basic_collect_ackermann_feedback(
	const servo_basic_state_t *state,
	float *speed_mps,
	float *steering_angle_rad,
	float *yaw_rate_rad_s)
{
	float feedback_speed = 0.0f;
	uint8_t speed_valid = 0U;
	const uint16_t servo_pulse = (state != NULL) ?
		state->servo_pulse_us : get_orin_servo_center_pulse();
	const float steering_rad =
		telemetry_estimate_steering_rad_from_servo_pulse(servo_pulse);
	float yaw_rate;

	if (s_esc_feedback_available != 0U &&
		s_esc_motion_estimate.signed_speed_valid != 0U)
	{
		feedback_speed = s_esc_motion_estimate.signed_speed_mps;
		speed_valid = 1U;
	}
	if (speed_valid == 0U)
	{
		feedback_speed = 0.0f;
	}
	yaw_rate = telemetry_estimate_vz_from_pwm(feedback_speed, servo_pulse);
	if (speed_mps != NULL)
	{
		*speed_mps = feedback_speed;
	}
	if (steering_angle_rad != NULL)
	{
		*steering_angle_rad = steering_rad;
	}
	if (yaw_rate_rad_s != NULL)
	{
		*yaw_rate_rad_s = (speed_valid != 0U) ? yaw_rate : 0.0f;
	}

	return speed_valid;
}

static servo_basic_diagnostics_t servo_basic_collect_diagnostics(uint32_t now_ms)
{
	servo_basic_diagnostics_t diagnostics;
	const uint8_t fe32_fresh = servo_basic_esc_fe32_is_fresh(now_ms);
	const int8_t direction = servo_basic_estimated_vehicle_direction();
	float uplink_speed_magnitude_mps = 0.0f;
	const uint8_t uplink_speed_valid =
		servo_basic_compute_uplink_speed_magnitude(now_ms,
			&uplink_speed_magnitude_mps);

	diagnostics.speed_saturated = (g_orin_state.speed_saturated != 0U ||
		g_speed_pi_saturated != 0U ||
		g_esc_speed_limit_active != 0U) ? 1U : 0U;
	diagnostics.steering_saturated = g_orin_state.steering_saturated;
	diagnostics.accel_limited = g_orin_state.accel_limited;
	diagnostics.steering_rate_limited = g_orin_state.steering_rate_limited;
	diagnostics.steering_fault = (g_rc_input_fault_active != 0U) ? 1U : 0U;
	diagnostics.esc_feedback_valid = (s_esc_feedback_available != 0U &&
		s_esc_motion_estimate.signed_speed_valid != 0U) ? 1U : 0U;
	diagnostics.esc_stop_confirmed = s_esc_stop_confirmed;
	diagnostics.esc_motion_config_valid =
		(s_esc_motion_estimator.config_valid != 0U) ? 1U : 0U;
	diagnostics.esc_fe32_fresh = fe32_fresh;
	diagnostics.esc_rpm_raw_valid =
		(fe32_fresh != 0U &&
		 s_esc_latest_raw_sample_valid != 0U &&
		 s_esc_latest_raw_sample.rpm_valid != 0U) ? 1U : 0U;
	diagnostics.esc_speed_magnitude_valid =
		(uplink_speed_valid != 0U &&
		 s_esc_stop_confirmed == 0U) ? 1U : 0U;
	diagnostics.esc_speed_calibration_valid =
		(servo_basic_uplink_speed_calibration_is_valid() != 0U) ? 1U : 0U;
	diagnostics.vehicle_direction_known = (direction != 0) ? 1U : 0U;
	diagnostics.esc_soft_uart_rx_error =
		(s_esc_receiver_health.rx_error_count != 0U) ? 1U : 0U;
	diagnostics.mode2_config_valid =
		(servo_basic_mode2_application_config_valid() != 0U) ? 1U : 0U;
	diagnostics.esc_sample_stale = s_esc_sample_stale;
	diagnostics.esc_rx_invalidated = s_esc_rx_invalidated;
	if (diagnostics.esc_feedback_valid != 0U)
	{
		diagnostics.esc_feedback_direction =
			(s_esc_motion_estimate.direction == ESC_MOTION_DIRECTION_FORWARD) ? 1 : -1;
	}
	else
	{
		diagnostics.esc_feedback_direction = 0;
	}
	diagnostics.esc_rpm_raw =
		(s_esc_latest_raw_sample_valid != 0U) ? s_esc_latest_raw_sample.rpm_raw : 0U;
	diagnostics.esc_sample_id =
		(s_esc_latest_raw_sample_valid != 0U) ?
		s_esc_latest_raw_sample.sample_id : s_esc_motion_estimate.last_sample_id;
	diagnostics.esc_receive_epoch = s_esc_receive_epoch;
	diagnostics.esc_valid_frame_count = s_esc_receiver_health.samples_published;
	diagnostics.esc_soft_uart_error_count = s_esc_receiver_health.rx_error_count;
	diagnostics.esc_soft_uart_error_flags = s_esc_receiver_health.last_rx_error_flags;

	return diagnostics;
}

static void servo_basic_publish_control_snapshot(uint32_t now_ms)
{
	servo_basic_control_snapshot_t snapshot;
	ServoBasicIrqState_t irq_state;
	const uint8_t orin_active = orin_pwm_is_active_at(now_ms);

	servo_basic_update_cached_estimate(now_ms);
	memset(&snapshot, 0, sizeof(snapshot));
	snapshot.state = g_state;
	snapshot.diagnostics = servo_basic_collect_diagnostics(now_ms);
	snapshot.signed_speed_valid = servo_basic_collect_ackermann_feedback(
		&snapshot.state,
		&snapshot.speed_mps,
		&snapshot.steering_angle_rad,
		&snapshot.yaw_rate_rad_s);
	snapshot.esc_uplink_speed_valid = servo_basic_collect_esc_uplink_speed(
		now_ms,
		&snapshot.esc_uplink_speed_mps,
		&snapshot.esc_speed_magnitude_mps,
		&snapshot.esc_direction_known);
	snapshot.rc_override_active = g_rc_override_active;
	snapshot.rc_emergency_active = g_rc_guard_active;
	if (g_orin_pwm_enable == 0U)
	{
		snapshot.orin_command_timeout = 0U;
	}
	else if (g_orin_state.active == 0U)
	{
		snapshot.orin_command_timeout = 1U;
	}
	else
	{
		snapshot.orin_command_timeout = (orin_active == 0U) ? 1U : 0U;
	}
	snapshot.orin_auto_enabled =
		(g_orin_pwm_enable != 0U &&
		 orin_active != 0U &&
		 g_rc_override_active == 0U &&
		 g_orin_state.emergency_stop == 0U &&
		 g_orin_state.auto_enabled != 0U) ? 1U : 0U;
	snapshot.orin_brake_active =
		(orin_active != 0U && g_orin_state.brake_active != 0U) ? 1U : 0U;
	snapshot.orin_emergency_active =
		(orin_active != 0U && g_orin_state.emergency_stop != 0U) ? 1U : 0U;
	snapshot.control_tick_ms = now_ms;
	snapshot.esc_sample_id =
		(s_esc_latest_raw_sample_valid != 0U) ?
		s_esc_latest_raw_sample.sample_id : s_esc_motion_estimate.last_sample_id;
	snapshot.esc_receive_epoch = s_esc_receive_epoch;
	snapshot.esc_sample_tick_ms =
		(s_esc_latest_raw_sample_valid != 0U) ?
		s_esc_latest_raw_sample.received_tick_ms : s_esc_motion_estimate.last_sample_tick_ms;
	snapshot.publish_sequence = ++s_control_snapshot_next_sequence;

	irq_state = servo_basic_enter_critical();
	s_control_snapshot = snapshot;
	servo_basic_exit_critical(irq_state);
}

static void update_control_mode_from_rc(void)
{
	const uint8_t guard_active = rc_guard_input_is_active();
	const uint8_t manual_override = rc_manual_override_requested();
	const uint8_t centered = rc_inputs_are_centered();
	const uint8_t serial_active = orin_pwm_is_active();
	const uint8_t rc_available = rc_passthrough_is_available();

	if (guard_active != 0U)
	{
		if (g_rc_override_active == 0U || g_rc_guard_active == 0U)
		{
			set_rc_override_state(1U, 1U, 1U);
		}
		return;
	}

	/*
	 * Keep ordinary RC behavior unchanged when there is no active serial PWM source:
	 * with a valid receiver connected, RC should directly drive ESC/servo without
	 * requiring a large stick delta just to "take over" an idle autonomous path.
	 *
	 * The enter-threshold / sample-count gate is only used while a serial command
	 * stream is actively driving the PWM path.
	 */
	if (serial_active == 0U)
	{
		if (rc_available != 0U)
		{
			if (g_rc_override_active == 0U || g_rc_guard_active != 0U)
			{
				set_rc_override_state(1U, 0U, 0U);
			}
		}
		else if (g_rc_override_active != 0U)
		{
			set_rc_override_state(0U, 0U, 0U);
		}
		else
		{
			g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
			g_state.emergency_stop = 0U;
		}
		return;
	}

	/*
	 * When serial becomes active and the receiver sticks are already centered,
	 * release the "idle RC passthrough" state immediately so serial control can
	 * take effect. From this point on, RC must exceed the configured threshold to
	 * reclaim control.
	 */
	if (g_rc_override_active != 0U && manual_override != 0U)
	{
		g_rc_override_release_hold_required = 1U;
	}

	if (g_rc_override_active != 0U && g_rc_guard_active == 0U &&
		g_rc_override_release_hold_required == 0U && centered != 0U)
	{
		set_rc_override_state(0U, 0U, 0U);
	}

	if (g_rc_override_active == 0U)
	{
		if (manual_override != 0U)
		{
			const uint32_t samples_req = get_rc_override_enter_samples();
			if (g_rc_override_enter_count < samples_req)
			{
				g_rc_override_enter_count++;
			}
			if (g_rc_override_enter_count >= samples_req)
			{
				set_rc_override_state(1U, 0U, 1U);
			}
		}
		else
		{
			g_rc_override_enter_count = 0U;
			g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
			g_state.emergency_stop = 0U;
		}
		return;
	}

	if (g_rc_guard_active != 0U)
	{
		set_rc_override_state(1U, 0U, 1U);
	}

	if (manual_override != 0U)
	{
		g_rc_override_release_start_ms = 0U;
		return;
	}

	if (centered == 0U)
	{
		g_rc_override_release_start_ms = 0U;
		return;
	}

	if (g_rc_override_release_start_ms == 0U)
	{
		g_rc_override_release_start_ms = HAL_GetTick();
		return;
	}

	if ((HAL_GetTick() - g_rc_override_release_start_ms) >= get_rc_override_release_hold_ms())
	{
		set_rc_override_state(0U, 0U, 0U);
	}
}

static void apply_rc_passthrough_outputs(void)
{
	const uint16_t esc_pulse = (g_rc_throttle_present != 0U) ?
		rc_select_pulse(g_rc_throttle_current, 1U) : get_orin_esc_center_pulse();
	const uint16_t servo_pulse = (g_rc_steering_present != 0U) ?
		rc_select_pulse(g_rc_steering_current, 0U) : get_orin_servo_center_pulse();

	apply_esc_pulse(esc_pulse);
	apply_servo_pulse(servo_pulse);
}

void ServoBasic_ProcessControl(void)
{
	uint8_t orin_active;
	uint32_t now_ms;

	now_ms = HAL_GetTick();
	servo_basic_refresh_esc_configs(now_ms);
	servo_basic_update_esc_feedback(now_ms);
	refresh_rc_inputs();
	update_control_mode_from_rc();
	orin_active = orin_pwm_is_active();
	if (g_rc_override_active != 0U)
	{
		HallSpeed_SetCommandDirection(get_rc_throttle_direction());
		servo_basic_invalidate_auto_history(now_ms);
		servo_basic_clear_vehicle_direction();
		speed_pi_reset_controller();
	}
	else if (orin_active == 0U)
	{
		HallSpeed_SetCommandDirection(0);
		speed_pi_reset_controller();
		if (g_orin_state.active != 0U)
		{
			servo_basic_invalidate_auto_history(now_ms);
		}
	}

	if (g_state.emergency_stop != 0U)
	{
		servo_basic_invalidate_auto_history(now_ms);
		speed_pi_reset_controller();
		apply_esc_pulse(get_orin_esc_center_pulse());
		apply_servo_pulse(get_orin_servo_center_pulse());
		servo_basic_publish_control_snapshot(now_ms);
		return;
	}
	if (g_rc_override_active != 0U)
	{
		apply_rc_passthrough_outputs();
	}
	else if (orin_active != 0U)
	{
		if (g_orin_state.stop != 0U)
		{
			servo_basic_invalidate_auto_history(now_ms);
			speed_pi_reset_controller();
			apply_esc_pulse(get_orin_esc_center_pulse());
			apply_servo_pulse(get_orin_servo_center_pulse());
		}
		else
		{
			Mode2DriveGateInput_t input;
			Mode2DriveTargetDirection_t target_direction;
			float normalized_brake_request = 0.0f;
			float actual_normalized_brake = 0.0f;
			uint16_t final_esc_pulse;
			uint8_t brake_request_valid = 0U;
			const uint8_t speed_limit_active =
				esc_speed_limit_protection_active();
			uint8_t tracking_decel_active;
			uint8_t stop_like_request;

			memset(&input, 0, sizeof(input));
			target_direction = mode2_target_direction_from_vx(g_orin_state.target_speed_mps);
			stop_like_request =
				(g_orin_state.brake_active != 0U ||
				 target_direction == MODE2_DRIVE_TARGET_NEUTRAL) ? 1U : 0U;
			tracking_decel_active = esc_tracking_brake_update(target_direction,
				g_orin_state.target_speed_mps,
				stop_like_request,
				speed_limit_active,
				&brake_request_valid,
				&normalized_brake_request);
			input.target_direction = target_direction;
			if (tracking_decel_active != 0U && esc_motion_direction_is_reverse() != 0U)
			{
				input.target_direction = MODE2_DRIVE_TARGET_NEUTRAL;
			}
			input.decel_or_stop_requested =
				(g_orin_state.brake_active != 0U ||
				 speed_limit_active != 0U ||
				 tracking_decel_active != 0U ||
				 input.target_direction == MODE2_DRIVE_TARGET_NEUTRAL) ? 1U : 0U;
			input.stop_requested =
				(input.target_direction == MODE2_DRIVE_TARGET_NEUTRAL) ? 1U : 0U;
			input.propulsion_authorized = servo_basic_auto_propulsion_authorized();
			input.brake_request_valid = brake_request_valid;
			input.normalized_brake_request = normalized_brake_request;

			s_mode2_drive_output = Mode2DriveGate_Evaluate(&s_mode2_drive_gate,
				&input,
				&s_esc_motion_estimate,
				now_ms);
			final_esc_pulse = mode2_drive_output_to_esc_pwm(
				s_mode2_drive_output.action,
				s_mode2_drive_output.normalized_brake_request,
				&actual_normalized_brake);
			apply_esc_pulse(final_esc_pulse);
			apply_servo_pulse(limit_servo_safe_pulse(clamp_servo_pulse(g_orin_state.servo_pulse_us)));
			servo_basic_commit_auto_action(s_mode2_drive_output.action,
				final_esc_pulse,
				actual_normalized_brake,
				now_ms);
		}
	}
	else
	{
		apply_esc_pulse(get_orin_esc_center_pulse());
		apply_servo_pulse(get_orin_servo_center_pulse());
		Mode2DriveGate_CommitAppliedActionWithEvidence(&s_mode2_drive_gate,
			MODE2_DRIVE_ACTION_NEUTRAL,
			now_ms,
			0.0f);
		EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
			ESC_MOTION_APPLIED_ACTION_NEUTRAL,
			now_ms);
	}

	servo_basic_publish_control_snapshot(now_ms);
}

void ServoBasic_Task(void *param)
{
	(void)param;

	TickType_t last_wake = xTaskGetTickCount();
	const TickType_t period_ticks = pdMS_TO_TICKS(20U);
	for (;;)
	{
		if (g_debug_servo_trigger != 0U)
		{
			const uint32_t cmd = g_debug_servo_cmd;
			const uint32_t value = g_debug_servo_value;

			g_debug_servo_trigger = 0U;
			servo_basic_apply_debug_command(cmd, value);
		}

		ServoBasic_ProcessControl();
		vTaskDelayUntil(&last_wake, period_ticks);
	}
}

static void servo_basic_apply_debug_command(uint32_t cmd, uint32_t value)
{
	switch (cmd)
	{
	case SERVO_CMD_SET_SERVO_ANGLE:
	{
		const uint16_t angle = clamp_servo_angle((uint16_t)value);
		const uint16_t pulse = servo_angle_to_pulse(angle);
		set_servo_target(pulse);
		SERVO_BASIC_LOG("debug servo angle=%u, pulse=%u\n", (unsigned int)angle, (unsigned int)pulse);
		break;
	}

	case SERVO_CMD_SET_SERVO_PULSE:
	{
		const uint8_t step = clamp_servo_step((uint16_t)value);
		const uint16_t pulse = servo_step_to_pulse(step);
		set_servo_target(pulse);
		SERVO_BASIC_LOG("debug servo step=%u, pulse=%u\n", (unsigned int)step, (unsigned int)pulse);
		break;
	}

	case SERVO_CMD_SET_ESC_PULSE:
	{
		const uint16_t pulse = clamp_esc_pulse((uint16_t)value);
		set_esc_target(pulse);
		SERVO_BASIC_LOG("debug esc pulse=%u\n", (unsigned int)pulse);
		break;
	}

	default:
		break;
	}
}

uint8_t ServoBasic_GetControlSnapshot(servo_basic_control_snapshot_t *snapshot)
{
	ServoBasicIrqState_t irq_state;

	if (snapshot == NULL)
	{
		return 0U;
	}

	irq_state = servo_basic_enter_critical();
	*snapshot = s_control_snapshot;
	servo_basic_exit_critical(irq_state);

	return (snapshot->publish_sequence != 0U) ? 1U : 0U;
}

uint8_t ServoBasic_IsRcOverrideActive(void)
{
	servo_basic_control_snapshot_t snapshot;

	return (ServoBasic_GetControlSnapshot(&snapshot) != 0U) ?
		snapshot.rc_override_active : 0U;
}

uint8_t ServoBasic_IsRcEmergencyActive(void)
{
	servo_basic_control_snapshot_t snapshot;

	return (ServoBasic_GetControlSnapshot(&snapshot) != 0U) ?
		snapshot.rc_emergency_active : 0U;
}

uint8_t ServoBasic_IsOrinCommandTimeout(void)
{
	servo_basic_control_snapshot_t snapshot;

	return (ServoBasic_GetControlSnapshot(&snapshot) != 0U) ?
		snapshot.orin_command_timeout : 0U;
}

uint8_t ServoBasic_IsOrinAutoEnabled(void)
{
	servo_basic_control_snapshot_t snapshot;

	return (ServoBasic_GetControlSnapshot(&snapshot) != 0U) ?
		snapshot.orin_auto_enabled : 0U;
}

uint8_t ServoBasic_IsOrinBrakeActive(void)
{
	servo_basic_control_snapshot_t snapshot;

	return (ServoBasic_GetControlSnapshot(&snapshot) != 0U) ?
		snapshot.orin_brake_active : 0U;
}

uint8_t ServoBasic_IsOrinEmergencyActive(void)
{
	servo_basic_control_snapshot_t snapshot;

	return (ServoBasic_GetControlSnapshot(&snapshot) != 0U) ?
		snapshot.orin_emergency_active : 0U;
}

uint8_t ServoBasic_GetAckermannFeedback(float *speed_mps,
										float *steering_angle_rad,
										float *yaw_rate_rad_s)
{
	servo_basic_control_snapshot_t snapshot;

	if (ServoBasic_GetControlSnapshot(&snapshot) == 0U)
	{
		if (speed_mps != NULL)
		{
			*speed_mps = 0.0f;
		}
		if (steering_angle_rad != NULL)
		{
			*steering_angle_rad = 0.0f;
		}
		if (yaw_rate_rad_s != NULL)
		{
			*yaw_rate_rad_s = 0.0f;
		}
		return 0U;
	}

	if (speed_mps != NULL)
	{
		*speed_mps = snapshot.speed_mps;
	}
	if (steering_angle_rad != NULL)
	{
		*steering_angle_rad = snapshot.steering_angle_rad;
	}
	if (yaw_rate_rad_s != NULL)
	{
		*yaw_rate_rad_s = snapshot.yaw_rate_rad_s;
	}

	return snapshot.signed_speed_valid;
}

servo_basic_diagnostics_t ServoBasic_GetDiagnostics(void)
{
	servo_basic_control_snapshot_t snapshot;

	if (ServoBasic_GetControlSnapshot(&snapshot) == 0U)
	{
		servo_basic_diagnostics_t diagnostics;

		memset(&diagnostics, 0, sizeof(diagnostics));
		return diagnostics;
	}

	return snapshot.diagnostics;
}
