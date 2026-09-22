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
#include "longitudinal_controller.h"
#include "mode2_drive_gate.h"
#include "rc_direction_observer.h"
#include "servo_rc_capture.h"

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#endif

#if defined(STM32F407xx)
typedef uint32_t ServoBasicIrqState_t;

static ServoBasicIrqState_t servo_basic_enter_critical(void)
{
	ServoBasicIrqState_t state = __get_BASEPRI();

	/* Protect task snapshots without delaying priority-3 PD15 bit sampling. */
	__set_BASEPRI_MAX(configMAX_SYSCALL_INTERRUPT_PRIORITY);
	__DSB();
	__ISB();
	return state;
}

static void servo_basic_exit_critical(ServoBasicIrqState_t state)
{
	__set_BASEPRI(state);
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
	0U
};
static servo_basic_control_snapshot_t s_control_snapshot;
static uint32_t s_control_snapshot_next_sequence = 0U;

#define RC_OVERRIDE_CENTER_DEFAULT_US            APP_RC_OVERRIDE_CENTER_US
#define RC_OVERRIDE_ENTER_THRESHOLD_DEFAULT_US   APP_RC_OVERRIDE_ENTER_THRESHOLD_US
#define RC_OVERRIDE_EXIT_THRESHOLD_DEFAULT_US    APP_RC_OVERRIDE_EXIT_THRESHOLD_US
#define RC_OVERRIDE_ENTER_SAMPLES_DEFAULT        APP_RC_OVERRIDE_ENTER_SAMPLES
#define RC_OVERRIDE_RELEASE_HOLD_DEFAULT_MS      APP_RC_OVERRIDE_RELEASE_HOLD_MS
#define ORIN_ACKERMANN_WHEELBASE_DEFAULT_MM      APP_ORIN_ACKERMANN_WHEELBASE_MM
#define ORIN_ACKERMANN_TRACK_WIDTH_DEFAULT_MM    APP_ORIN_ACKERMANN_TRACK_WIDTH_MM
#define ORIN_ACKERMANN_WHEEL_RADIUS_DEFAULT_MM   APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM
#define ORIN_ACKERMANN_MAX_STEERING_DEFAULT_MRAD APP_ORIN_ACKERMANN_MAX_STEERING_MRAD
#define ORIN_ESC_FORWARD_MAX_DEFAULT_US          APP_ORIN_ESC_FORWARD_MAX_US
#define ORIN_ESC_REVERSE_MAX_DEFAULT_US          APP_ORIN_ESC_REVERSE_MAX_US
#define ORIN_SERVO_RANGE_DEFAULT_US              APP_ORIN_SERVO_RANGE_US
#define ORIN_ACCEL_LIMIT_DEFAULT_MMPS2           APP_ORIN_ACCEL_LIMIT_MMPS2
#define ORIN_STEERING_RATE_LIMIT_DEFAULT_MRADPS  APP_ORIN_STEERING_RATE_LIMIT_MRADPS
#define SPEED_PI_ENABLE_DEFAULT                  APP_SPEED_PI_ENABLE_DEFAULT
#define SPEED_PI_KP_DEFAULT_US_PER_MPS           APP_SPEED_PI_KP_DEFAULT_US_PER_MPS
#define SPEED_PI_KI_DEFAULT_US_PER_MPS_S         APP_SPEED_PI_KI_DEFAULT_US_PER_MPS_S
#define SPEED_PI_TRIM_LIMIT_DEFAULT_US           APP_SPEED_PI_TRIM_LIMIT_US
#define SPEED_PI_KP_MAX_US_PER_MPS               120.0f
#define SPEED_PI_KI_MAX_US_PER_MPS_S             40.0f
#define SPEED_PI_TRIM_LIMIT_PARAM_MAX_US         60U
#define RC_VALID_MIN_DEFAULT_US                  APP_RC_VALID_MIN_US
#define RC_VALID_MAX_DEFAULT_US                  APP_RC_VALID_MAX_US
#define RC_FRAME_MIN_DEFAULT_US                  APP_RC_FRAME_MIN_US
#define RC_FRAME_MAX_DEFAULT_US                  APP_RC_FRAME_MAX_US
#define RC_GLITCH_FREEZE_DEFAULT_MS              APP_RC_GLITCH_FREEZE_MS
#define RC_THROTTLE_NEUTRAL_HOLD_DEFAULT_US      APP_RC_THROTTLE_NEUTRAL_HOLD_US
#define RC_THROTTLE_JUMP_CONFIRM_DEFAULT_US      APP_RC_THROTTLE_JUMP_CONFIRM_US
#define RC_STEERING_JUMP_CONFIRM_DEFAULT_US      APP_RC_STEERING_JUMP_CONFIRM_US
#define RC_JUMP_CONFIRM_SAMPLES_DEFAULT          APP_RC_JUMP_CONFIRM_SAMPLES

// RC signal timeout in milliseconds (0 uses default 100 ms).
volatile uint32_t g_rc_signal_timeout_ms = APP_RC_SIGNAL_TIMEOUT_MS;
// Orin kinematics to PWM settings (set from Keil Watch).
volatile uint32_t g_orin_pwm_timeout_ms = APP_ORIN_PWM_TIMEOUT_DEFAULT_MS;
volatile uint32_t g_orin_ackermann_wheelbase_mm = ORIN_ACKERMANN_WHEELBASE_DEFAULT_MM;
volatile uint32_t g_orin_ackermann_track_width_mm = ORIN_ACKERMANN_TRACK_WIDTH_DEFAULT_MM;
volatile uint32_t g_orin_ackermann_wheel_radius_mm = ORIN_ACKERMANN_WHEEL_RADIUS_DEFAULT_MM;
volatile uint32_t g_orin_ackermann_max_steering_millirad = ORIN_ACKERMANN_MAX_STEERING_DEFAULT_MRAD;
volatile uint32_t g_orin_esc_center_us = APP_ORIN_ESC_CENTER_US;
volatile uint32_t g_orin_esc_forward_max_us = ORIN_ESC_FORWARD_MAX_DEFAULT_US;
volatile uint32_t g_orin_esc_reverse_max_us = ORIN_ESC_REVERSE_MAX_DEFAULT_US;
volatile uint32_t g_orin_servo_center_us = APP_ORIN_SERVO_CENTER_US;
volatile uint32_t g_orin_servo_range_us = ORIN_SERVO_RANGE_DEFAULT_US;
volatile float g_esc_tracking_brake_kp = APP_ESC_TRACKING_BRAKE_KP_DEFAULT;
volatile float g_esc_tracking_brake_max = APP_ESC_TRACKING_BRAKE_MAX_DEFAULT;
volatile float g_esc_tracking_brake_enter_error_mps =
	APP_ESC_TRACKING_BRAKE_ENTER_ERROR_MPS_DEFAULT;
volatile float g_esc_tracking_brake_release_error_mps =
	APP_ESC_TRACKING_BRAKE_RELEASE_ERROR_MPS_DEFAULT;
volatile float g_esc_motion_stopped_speed_threshold_mps = APP_ESC_STOPPED_THRESHOLD_MPS_DEFAULT;
volatile uint32_t g_esc_motion_stopped_min_samples = APP_ESC_STOPPED_MIN_SAMPLES_DEFAULT;
volatile uint32_t g_esc_motion_stopped_min_coverage_ms = APP_ESC_STOPPED_MIN_COVERAGE_MS_DEFAULT;
volatile float g_esc_low_gear_wheel_rpm_per_raw =
	APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT;
volatile uint32_t g_esc_speed_fresh_timeout_ms =
	APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT;
volatile float g_mode2_fwd_to_rev_brake_request =
	APP_MODE2_FWD_TO_REV_BRAKE_REQUEST_DEFAULT;
volatile float g_mode2_rev_to_fwd_brake_request =
	APP_MODE2_REV_TO_FWD_BRAKE_REQUEST_DEFAULT;
volatile uint32_t g_mode2_fwd_to_rev_brake_hold_ms =
	APP_MODE2_FWD_TO_REV_BRAKE_HOLD_MS_DEFAULT;
volatile uint32_t g_mode2_rev_to_fwd_brake_hold_ms =
	APP_MODE2_REV_TO_FWD_BRAKE_HOLD_MS_DEFAULT;
volatile uint32_t g_mode2_drive_neutral_dwell_ms = APP_MODE2_DRIVE_NEUTRAL_DWELL_MS_DEFAULT;
volatile uint32_t g_mode2_fwd_to_rev_qualify_delta_us =
	APP_MODE2_FWD_TO_REV_QUALIFY_DELTA_US_DEFAULT;
volatile uint32_t g_mode2_rev_to_fwd_qualify_delta_us =
	APP_MODE2_REV_TO_FWD_QUALIFY_DELTA_US_DEFAULT;
volatile uint32_t g_mode2_fwd_to_rev_brake_full_pwm_us = APP_MODE2_FWD_TO_REV_BRAKE_FULL_PWM_US_DEFAULT;
volatile uint32_t g_mode2_rev_to_fwd_brake_full_pwm_us = APP_MODE2_REV_TO_FWD_BRAKE_FULL_PWM_US_DEFAULT;
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
volatile uint32_t g_rc_throttle_last_good_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile uint32_t g_rc_steering_last_good_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile uint32_t g_rc_throttle_glitch_active = 0U;
volatile uint32_t g_rc_steering_glitch_active = 0U;
volatile uint32_t g_rc_input_fault_active = 0U;

typedef struct
{
	uint16_t servo_pulse_us;
	uint32_t last_update_ms;
	float target_speed_mps;
	float target_steering_angle_rad;
	uint8_t active;
	uint8_t software_stop;
	uint8_t auto_enabled;
	uint8_t brake_active;
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

static orin_pwm_state_t g_orin_state = {
	.servo_pulse_us = ESC_PWM_NEUTRAL_PULSE_US,
	.last_update_ms = 0U,
	.target_speed_mps = 0.0f,
	.target_steering_angle_rad = 0.0f,
	.active = 0U,
	.software_stop = 0U,
	.auto_enabled = 0U,
	.brake_active = 0U,
	.steering_saturated = 0U,
	.accel_limited = 0U,
	.steering_rate_limited = 0U
};
static volatile uint8_t g_rc_override_active = 0U;
static uint8_t g_rc_override_enter_count = 0U;
static uint32_t g_rc_override_release_start_ms = 0U;
static uint8_t g_rc_override_release_hold_required = 0U;
static uint16_t g_rc_throttle_current = 0U;
static uint16_t g_rc_steering_current = 0U;
static uint8_t g_rc_throttle_present = 0U;
static uint8_t g_rc_steering_present = 0U;
static rc_channel_filter_state_t g_rc_throttle_state = {0U};
static rc_channel_filter_state_t g_rc_steering_state = {0U};
static uint32_t g_speed_pi_last_update_ms = 0U;
static uint32_t g_speed_pi_last_sample_id = 0U;
static uint8_t g_speed_pi_have_sample = 0U;
static int8_t g_speed_pi_last_target_direction = 0;
static EscMotionEstimator_t s_esc_motion_estimator;
static Mode2DriveGate_t s_mode2_drive_gate;
static LongitudinalController_t s_longitudinal_controller;
static EscMotionEstimatorConfig_t s_esc_motion_config;
static Mode2DriveGateConfig_t s_mode2_drive_config;
static LongitudinalControllerConfig_t s_longitudinal_config;
static uint8_t s_esc_motion_config_initialized = 0U;
static uint8_t s_mode2_drive_config_initialized = 0U;
static uint8_t s_longitudinal_config_initialized = 0U;
static EscMotionEstimate_t s_esc_motion_estimate;
static Mode2DriveGateOutput_t s_mode2_drive_output;
static LongitudinalControllerOutput_t s_longitudinal_output;
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
static RcDirectionObserver_t s_rc_direction_observer;
static RcDirectionObserverResult_t s_rc_direction_result;
static uint32_t s_rc_output_context;
static uint32_t s_rc_delivery_epoch;
static servo_basic_observation_diagnostics_t s_observation_diagnostics;
static uint32_t s_rc_last_published_sample_id;
static uint32_t s_rc_last_published_receive_epoch;
static uint8_t s_vehicle_direction_known = 0U;
static int8_t s_vehicle_direction = 0;
static uint8_t s_auto_history_boundary_valid = 0U;
static uint32_t s_auto_history_boundary_ms = 0U;

static uint16_t limit_auto_propulsion_pulse(uint16_t pulse_us);
static uint16_t limit_servo_safe_pulse(uint16_t pulse_us);
static void speed_pi_reset_controller(void);
static uint32_t get_rc_override_center_us(void);
static uint8_t pulse_is_inside_center(uint16_t pulse_us, uint32_t center_us, uint32_t threshold_us);
static uint8_t orin_pwm_is_active(void);
static void servo_basic_refresh_esc_configs(uint32_t now_ms);
static void servo_basic_update_esc_feedback(uint32_t now_ms);
static void servo_basic_invalidate_auto_history(uint32_t now_ms);
static uint32_t get_speed_pi_trim_limit_us(void);
static uint8_t longitudinal_config_equal(const LongitudinalControllerConfig_t *left,
										 const LongitudinalControllerConfig_t *right);
static uint8_t servo_basic_tick_is_after(uint32_t tick_ms, uint32_t reference_ms);
static uint8_t servo_basic_tick_delta_ms(uint32_t tick_ms, uint32_t reference_ms, uint32_t *delta_ms);
static uint8_t servo_basic_mode2_application_config_valid(void);
static uint8_t servo_basic_auto_propulsion_authorized(void);
static void servo_basic_clear_vehicle_direction(void);
static uint8_t servo_basic_esc_fe32_is_fresh(uint32_t now_ms);
static servo_esc_action_t servo_basic_current_esc_action(uint32_t now_ms);
static uint8_t orin_pwm_is_active_at(uint32_t now_ms);
static servo_basic_diagnostics_t servo_basic_collect_diagnostics(uint32_t now_ms);
static void servo_basic_publish_control_snapshot(uint32_t now_ms);
static void servo_basic_update_rc_feedback(uint32_t now_ms);
static void servo_basic_reset_rc_evidence(void);

__attribute__((weak)) void ServoBasic_OutputEscPulse(uint16_t pulse_us)
{
	(void)pulse_us;
}

__attribute__((weak)) void ServoBasic_OutputServoPulse(uint16_t pulse_us)
{
	(void)pulse_us;
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
		/* A valid receiver throttle pulse is already bounded by the capture
		 * contract. Preserve its complete travel instead of applying the
		 * automatic controller's calibrated propulsion endpoints. */
		return pulse_us;
	}
	return limit_servo_safe_pulse(clamp_servo_pulse(pulse_us));
}

static void rc_debounce_reset(void)
{
	memset(&g_rc_throttle_state, 0, sizeof(g_rc_throttle_state));
	memset(&g_rc_steering_state, 0, sizeof(g_rc_steering_state));
	g_rc_throttle_current = 0U;
	g_rc_steering_current = 0U;
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
	if (*state == 0U)
	{
		*state = raw;
		return raw;
	}

	uint32_t deadband = g_rc_debounce_deadband_us;
	uint32_t diff = (raw > *state) ? (raw - *state) : (*state - raw);
	if (diff == 0U)
	{
		return *state;
	}
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
		filtered = pulse_us;
		state->filter_state = filtered;
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

static uint16_t get_orin_esc_forward_limit_pulse(void)
{
	uint16_t center = get_orin_esc_center_pulse();
	uint16_t pulse = clamp_esc_pulse((uint16_t)((g_orin_esc_forward_max_us == 0U) ?
		ORIN_ESC_FORWARD_MAX_DEFAULT_US : g_orin_esc_forward_max_us));
	return (pulse < center) ? center : pulse;
}

static uint16_t get_orin_esc_reverse_limit_pulse(void)
{
	uint16_t center = get_orin_esc_center_pulse();
	uint16_t pulse = clamp_esc_pulse((uint16_t)((g_orin_esc_reverse_max_us == 0U) ?
		ORIN_ESC_REVERSE_MAX_DEFAULT_US : g_orin_esc_reverse_max_us));
	return (pulse > center) ? center : pulse;
}

static uint16_t limit_auto_propulsion_pulse(uint16_t pulse_us)
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
	uint8_t throttle_fault_persistent;
	uint8_t steering_fault_persistent;

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
		steering_fault_persistent != 0U) ? 1U : 0U;

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
	uint8_t override_active, uint8_t release_hold_required)
{
	const uint8_t was_active = g_rc_override_active;

	g_rc_override_active = (override_active != 0U) ? 1U : 0U;
	g_rc_override_release_hold_required =
		(g_rc_override_active != 0U && release_hold_required != 0U) ? 1U : 0U;
	g_state.control_mode = (g_rc_override_active != 0U) ? SERVO_CTRL_MODE_RC_PASSTHROUGH :
		SERVO_CTRL_MODE_AUTONOMOUS;
	g_state.rc_takeover_pending = 0U;
	g_rc_override_release_start_ms = 0U;
	g_rc_override_enter_count = 0U;
	if (was_active != g_rc_override_active)
	{
		servo_basic_invalidate_auto_history(HAL_GetTick());
		EscTelemetry_DiscardSamples();
		s_rc_delivery_epoch = EscTelemetry_GetDeliveryEpoch();
		servo_basic_reset_rc_evidence();
		/* A previous source's sample must not establish this source's stop. */
		EscMotionEstimator_Init(&s_esc_motion_estimator, &s_esc_motion_config);
		s_esc_latest_raw_sample_valid = 0U;
		s_esc_last_observed_sample_id = 0U;
	}
	if (was_active != 0U && g_rc_override_active == 0U)
	{
		RcDirectionObserver_ResetDirection(&s_rc_direction_observer);
		memset(&s_rc_direction_result, 0, sizeof(s_rc_direction_result));
		servo_basic_clear_vehicle_direction();
		HallSpeed_SetCommandDirection(0);
	}
}

static void apply_esc_pulse(uint16_t pulse_us)
{
	g_state.esc_pulse_us = pulse_us;
	ServoBasic_OutputEscPulse(pulse_us);
	s_rc_output_context = EscTelemetry_PublishOutputContext(pulse_us, g_rc_override_active);
}

static void speed_pi_reset_controller(void)
{
	if (s_longitudinal_config_initialized != 0U)
	{
		LongitudinalController_Reset(&s_longitudinal_controller);
	}
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
	config.wheel_rpm_per_raw = g_esc_low_gear_wheel_rpm_per_raw;
	config.wheel_radius_m = (float)get_orin_ackermann_wheel_radius_mm() / 1000.0f;
	config.telemetry_timeout_ms = g_esc_speed_fresh_timeout_ms;
	config.stopped_speed_threshold_mps = g_esc_motion_stopped_speed_threshold_mps;
	config.stopped_min_samples = (uint8_t)g_esc_motion_stopped_min_samples;
	config.stopped_min_coverage_ms = g_esc_motion_stopped_min_coverage_ms;
	return config;
}

static Mode2DriveGateConfig_t servo_basic_read_mode2_config(void)
{
	Mode2DriveGateConfig_t config;

	memset(&config, 0, sizeof(config));
	config.fwd_to_rev_brake_request = g_mode2_fwd_to_rev_brake_request;
	config.rev_to_fwd_brake_request = g_mode2_rev_to_fwd_brake_request;
	config.fwd_to_rev_brake_min_ms = g_mode2_fwd_to_rev_brake_hold_ms;
	config.rev_to_fwd_brake_min_ms = g_mode2_rev_to_fwd_brake_hold_ms;
	config.neutral_dwell_ms = g_mode2_drive_neutral_dwell_ms;
	config.center_pwm_us = get_orin_esc_center_pulse();
	config.fwd_to_rev_brake_full_pwm_us =
		(uint16_t)g_mode2_fwd_to_rev_brake_full_pwm_us;
	config.rev_to_fwd_brake_full_pwm_us =
		(uint16_t)g_mode2_rev_to_fwd_brake_full_pwm_us;
	config.fwd_to_rev_qualify_delta_us =
		(uint16_t)g_mode2_fwd_to_rev_qualify_delta_us;
	config.rev_to_fwd_qualify_delta_us =
		(uint16_t)g_mode2_rev_to_fwd_qualify_delta_us;
	return config;
}

static LongitudinalControllerConfig_t servo_basic_read_longitudinal_config(void)
{
	LongitudinalControllerConfig_t config;

	memset(&config, 0, sizeof(config));
	config.center_pwm_us = get_orin_esc_center_pulse();
	config.min_pwm_us = ESC_PWM_MIN_PULSE_US;
	config.max_pwm_us = ESC_PWM_MAX_PULSE_US;
	config.forward_limit_pwm_us = get_orin_esc_forward_limit_pulse();
	config.reverse_limit_pwm_us = get_orin_esc_reverse_limit_pulse();
	config.target_slew_rate_mps2 = (float)get_orin_accel_limit_mmps2() / 1000.0f;
	config.pi_enabled = uint32_to_u8_flag(g_speed_pi_enable);
	config.pi_kp_us_per_mps = g_speed_pi_kp;
	config.pi_ki_us_per_mps_s = g_speed_pi_ki;
	config.pi_trim_limit_us = (uint16_t)get_speed_pi_trim_limit_us();
	config.tracking_brake_kp = g_esc_tracking_brake_kp;
	config.tracking_brake_max = g_esc_tracking_brake_max;
	config.tracking_brake_enter_error_mps = g_esc_tracking_brake_enter_error_mps;
	config.tracking_brake_release_error_mps = g_esc_tracking_brake_release_error_mps;
	return config;
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

static uint8_t esc_motion_config_equal(const EscMotionEstimatorConfig_t *left,
									   const EscMotionEstimatorConfig_t *right)
{
	return (left->wheel_rpm_per_raw == right->wheel_rpm_per_raw &&
		left->wheel_radius_m == right->wheel_radius_m &&
		left->telemetry_timeout_ms == right->telemetry_timeout_ms &&
		left->stopped_speed_threshold_mps == right->stopped_speed_threshold_mps &&
		left->stopped_min_samples == right->stopped_min_samples &&
		left->stopped_min_coverage_ms == right->stopped_min_coverage_ms) ? 1U : 0U;
}

static uint8_t mode2_config_equal(const Mode2DriveGateConfig_t *left,
								  const Mode2DriveGateConfig_t *right)
{
	return (left->fwd_to_rev_brake_request == right->fwd_to_rev_brake_request &&
		left->rev_to_fwd_brake_request == right->rev_to_fwd_brake_request &&
		left->fwd_to_rev_brake_min_ms == right->fwd_to_rev_brake_min_ms &&
		left->rev_to_fwd_brake_min_ms == right->rev_to_fwd_brake_min_ms &&
		left->neutral_dwell_ms == right->neutral_dwell_ms &&
		left->center_pwm_us == right->center_pwm_us &&
		left->fwd_to_rev_brake_full_pwm_us == right->fwd_to_rev_brake_full_pwm_us &&
		left->rev_to_fwd_brake_full_pwm_us == right->rev_to_fwd_brake_full_pwm_us &&
		left->fwd_to_rev_qualify_delta_us == right->fwd_to_rev_qualify_delta_us &&
		left->rev_to_fwd_qualify_delta_us == right->rev_to_fwd_qualify_delta_us) ? 1U : 0U;
}

static uint8_t longitudinal_config_equal(const LongitudinalControllerConfig_t *left,
										 const LongitudinalControllerConfig_t *right)
{
	return (left->center_pwm_us == right->center_pwm_us &&
		left->min_pwm_us == right->min_pwm_us &&
		left->max_pwm_us == right->max_pwm_us &&
		left->forward_limit_pwm_us == right->forward_limit_pwm_us &&
		left->reverse_limit_pwm_us == right->reverse_limit_pwm_us &&
		left->target_slew_rate_mps2 == right->target_slew_rate_mps2 &&
		left->pi_enabled == right->pi_enabled &&
		left->pi_kp_us_per_mps == right->pi_kp_us_per_mps &&
		left->pi_ki_us_per_mps_s == right->pi_ki_us_per_mps_s &&
		left->pi_trim_limit_us == right->pi_trim_limit_us &&
		left->tracking_brake_kp == right->tracking_brake_kp &&
		left->tracking_brake_max == right->tracking_brake_max &&
		left->tracking_brake_enter_error_mps == right->tracking_brake_enter_error_mps &&
		left->tracking_brake_release_error_mps == right->tracking_brake_release_error_mps) ? 1U : 0U;
}

static void servo_basic_clear_esc_observation_state(void)
{
	s_rc_output_context = 0U;
	s_rc_delivery_epoch = 0U;
	s_rc_last_published_sample_id = 0U;
	s_rc_last_published_receive_epoch = 0U;
	memset(&s_observation_diagnostics, 0, sizeof(s_observation_diagnostics));
	RcDirectionObserver_Init(&s_rc_direction_observer);
	s_rc_direction_result =
		RcDirectionObserver_GetResult(&s_rc_direction_observer);
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
	memset(&s_longitudinal_output, 0, sizeof(s_longitudinal_output));
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
}

static void servo_basic_refresh_esc_configs(uint32_t now_ms)
{
	const EscMotionEstimatorConfig_t motion_config =
		servo_basic_read_esc_motion_config();
	const Mode2DriveGateConfig_t mode2_config =
		servo_basic_read_mode2_config();
	const LongitudinalControllerConfig_t longitudinal_config =
		servo_basic_read_longitudinal_config();
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

	if (s_longitudinal_config_initialized == 0U)
	{
		LongitudinalController_Init(&s_longitudinal_controller,
			&longitudinal_config);
		s_longitudinal_config = longitudinal_config;
		s_longitudinal_config_initialized = 1U;
		changed = 1U;
	}
	else if (longitudinal_config_equal(&s_longitudinal_config,
									  &longitudinal_config) == 0U)
	{
		(void)LongitudinalController_SetConfig(&s_longitudinal_controller,
			&longitudinal_config);
		s_longitudinal_config = longitudinal_config;
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
		 s_esc_motion_estimate.stop_valid != 0U &&
		 s_esc_motion_estimate.stopped != 0U) ? 1U : 0U;
	if (s_esc_stop_confirmed != 0U)
	{
		servo_basic_clear_vehicle_direction();
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

void ServoBasic_Init(void)
{
	g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
	g_state.rc_takeover_pending = 0U;
	g_rc_override_active = 0U;
	g_rc_override_enter_count = 0U;
	g_rc_override_release_start_ms = 0U;
	g_rc_override_release_hold_required = 0U;
	g_rc_throttle_present = 0U;
	g_rc_steering_present = 0U;
	rc_debounce_reset();
	g_orin_state.servo_pulse_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_orin_state.last_update_ms = 0U;
	g_orin_state.target_speed_mps = 0.0f;
	g_orin_state.target_steering_angle_rad = 0.0f;
	g_orin_state.active = 0U;
	g_orin_state.software_stop = 0U;
	g_orin_state.auto_enabled = 0U;
	g_orin_state.brake_active = 0U;
	g_orin_state.steering_saturated = 0U;
	g_orin_state.accel_limited = 0U;
	g_orin_state.steering_rate_limited = 0U;
	g_speed_pi_target_vx_mps = 0.0f;
	g_speed_pi_feedback_vx_mps = 0.0f;
	g_speed_pi_base_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_speed_pi_final_us = ESC_PWM_NEUTRAL_PULSE_US;
	s_esc_motion_config_initialized = 0U;
	s_mode2_drive_config_initialized = 0U;
	s_longitudinal_config_initialized = 0U;
	servo_basic_clear_esc_observation_state();
	servo_basic_refresh_esc_configs(HAL_GetTick());
	speed_pi_reset_controller();
	ServoRC_Capture_Init();
	apply_esc_pulse(get_orin_esc_center_pulse());
	apply_servo_pulse(get_orin_servo_center_pulse());
	s_control_snapshot_next_sequence = 0U;
	servo_basic_publish_control_snapshot(HAL_GetTick());
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

static uint16_t longitudinal_drive_output_to_pwm(
	const LongitudinalControllerOutput_t *output)
{
	uint16_t final_pulse;
	uint16_t base_pulse = get_orin_esc_center_pulse();

	if (output != NULL)
	{
		base_pulse = output->diagnostics.feedforward_pwm_us;
	}

	if (output == NULL || output->intent != LONGITUDINAL_INTENT_DRIVE)
	{
		g_speed_pi_base_us = base_pulse;
		g_speed_pi_final_us = get_orin_esc_center_pulse();
		g_speed_pi_saturated = 0U;
		return get_orin_esc_center_pulse();
	}

	final_pulse = limit_auto_propulsion_pulse(output->drive_pwm_us);
	g_speed_pi_base_us = base_pulse;
	g_speed_pi_final_us = final_pulse;
	g_speed_pi_saturated =
		(output->diagnostics.pi_saturated != 0U ||
		 final_pulse != output->drive_pwm_us) ? 1U : 0U;
	return final_pulse;
}

static void update_speed_watch_from_longitudinal(
	const LongitudinalControllerOutput_t *output)
{
	if (output == NULL)
	{
		speed_pi_reset_controller();
		return;
	}

	g_speed_pi_target_vx_mps = output->diagnostics.requested_target_mps;
	g_speed_pi_feedback_vx_mps = output->diagnostics.feedback_signed_mps;
	g_speed_pi_feedback_valid = output->diagnostics.feedback_valid;
	g_speed_pi_error_mps = output->diagnostics.speed_error_mps;
	g_speed_pi_integral = output->diagnostics.pi_integral_mps_s;
	g_speed_pi_trim_us = float_to_i32_nearest(output->diagnostics.pi_trim_us);
	g_orin_state.accel_limited = output->diagnostics.slew_limited;
	g_speed_pi_last_target_direction =
		(output->target_direction == LONGITUDINAL_DIRECTION_FORWARD) ? 1 :
		((output->target_direction == LONGITUDINAL_DIRECTION_REVERSE) ? -1 : 0);
	if (output->diagnostics.duplicate_sample == 0U &&
		output->diagnostics.feedback_valid != 0U)
	{
		g_speed_pi_have_sample = 1U;
		g_speed_pi_last_sample_id = s_esc_motion_estimate.last_sample_id;
		g_speed_pi_last_update_ms = s_esc_motion_estimate.last_sample_tick_ms;
	}
}

static Mode2DriveTargetDirection_t mode2_target_from_longitudinal(
	LongitudinalDirection_t direction)
{
	if (direction == LONGITUDINAL_DIRECTION_FORWARD)
	{
		return MODE2_DRIVE_TARGET_FORWARD;
	}
	if (direction == LONGITUDINAL_DIRECTION_REVERSE)
	{
		return MODE2_DRIVE_TARGET_REVERSE;
	}
	return MODE2_DRIVE_TARGET_NEUTRAL;
}

static LongitudinalDirection_t longitudinal_direction_from_gate(void)
{
	if (s_esc_stop_confirmed != 0U)
	{
		return LONGITUDINAL_DIRECTION_UNKNOWN;
	}

	switch (s_mode2_drive_gate.state)
	{
	case MODE2_DRIVE_STATE_FORWARD_TRACKING:
	case MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS:
	case MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED:
		return LONGITUDINAL_DIRECTION_FORWARD;
	case MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING:
		return (s_esc_motion_estimate.moving_observed != 0U &&
			servo_basic_tick_is_after(s_esc_motion_estimate.last_sample_tick_ms,
				s_mode2_drive_gate.forward_recovery_start_ms) != 0U) ?
			LONGITUDINAL_DIRECTION_FORWARD : LONGITUDINAL_DIRECTION_UNKNOWN;
	case MODE2_DRIVE_STATE_REVERSE_TRACKING:
	case MODE2_DRIVE_STATE_REVERSE_BRAKE_CONTINUOUS:
	case MODE2_DRIVE_STATE_FORWARD_MAYBE_ARMED:
		return LONGITUDINAL_DIRECTION_REVERSE;
	default:
		break;
	}

	if (s_vehicle_direction_known != 0U)
	{
		return (s_vehicle_direction < 0) ?
			LONGITUDINAL_DIRECTION_REVERSE : LONGITUDINAL_DIRECTION_FORWARD;
	}
	return LONGITUDINAL_DIRECTION_UNKNOWN;
}

static LongitudinalControllerInput_t servo_basic_build_longitudinal_input(
	uint32_t now_ms)
{
	LongitudinalControllerInput_t input;

	memset(&input, 0, sizeof(input));
	input.automatic_enabled = servo_basic_auto_propulsion_authorized();
	input.stop_requested = g_orin_state.brake_active;
	input.target_speed_mps = g_orin_state.target_speed_mps;
	input.feedback_valid = s_esc_feedback_available;
	input.stopped = (s_esc_stop_confirmed != 0U ||
		s_mode2_drive_gate.state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING) ? 1U : 0U;
	input.current_direction = longitudinal_direction_from_gate();
	input.speed_magnitude_mps =
		(s_esc_feedback_available != 0U) ?
		s_esc_motion_estimate.speed_magnitude_mps : 0.0f;
	input.feedback_sample_id = s_esc_motion_estimate.last_sample_id;
	input.feedback_sample_tick_ms = s_esc_motion_estimate.last_sample_tick_ms;
	input.now_tick_ms = now_ms;
	return input;
}

static Mode2DriveGateInput_t servo_basic_mode2_input_from_longitudinal(
	const LongitudinalControllerOutput_t *control_output)
{
	Mode2DriveGateInput_t input;

	memset(&input, 0, sizeof(input));
	if (control_output == NULL)
	{
		return input;
	}

	input.propulsion_authorized = servo_basic_auto_propulsion_authorized();
	input.target_direction =
		mode2_target_from_longitudinal(control_output->target_direction);
	switch (control_output->intent)
	{
	case LONGITUDINAL_INTENT_DRIVE:
		break;
	case LONGITUDINAL_INTENT_TRACKING_BRAKE:
		input.decel_or_stop_requested = 1U;
		input.brake_request_valid = 1U;
		input.normalized_brake_request =
			control_output->normalized_brake_request;
		break;
	case LONGITUDINAL_INTENT_STOP_BRAKE:
		input.target_direction = MODE2_DRIVE_TARGET_NEUTRAL;
		input.decel_or_stop_requested = 1U;
		input.stop_requested = 1U;
		break;
	case LONGITUDINAL_INTENT_REVERSAL_REQUEST:
		input.decel_or_stop_requested = 1U;
		break;
	case LONGITUDINAL_INTENT_NEUTRAL:
	default:
		input.target_direction = MODE2_DRIVE_TARGET_NEUTRAL;
		break;
	}

	input.forward_recovery_authorized =
		(input.target_direction == MODE2_DRIVE_TARGET_FORWARD &&
		 control_output->intent == LONGITUDINAL_INTENT_DRIVE &&
		 s_esc_stop_confirmed != 0U &&
		 s_mode2_drive_gate.state == MODE2_DRIVE_STATE_UNKNOWN_SAFE) ? 1U : 0U;
	return input;
}

static Mode2DriveEscAction_t servo_basic_mode2_esc_action(uint32_t now_ms)
{
	switch (servo_basic_current_esc_action(now_ms))
	{
	case SERVO_ESC_ACTION_NEUTRAL:
		return MODE2_DRIVE_ESC_ACTION_NEUTRAL;
	case SERVO_ESC_ACTION_DRIVE:
		return MODE2_DRIVE_ESC_ACTION_DRIVE;
	case SERVO_ESC_ACTION_BRAKE:
		return MODE2_DRIVE_ESC_ACTION_BRAKE;
	case SERVO_ESC_ACTION_UNKNOWN:
	default:
		return MODE2_DRIVE_ESC_ACTION_UNKNOWN;
	}
}

static Mode2DriveMotionObservation_t servo_basic_build_mode2_observation(
	uint32_t now_ms)
{
	Mode2DriveMotionObservation_t observation;

	memset(&observation, 0, sizeof(observation));
	observation.available =
		(s_esc_motion_estimate.config_valid != 0U &&
		 s_esc_motion_estimate.has_sample != 0U &&
		 s_esc_motion_estimate.sample_fresh != 0U &&
		 s_esc_motion_estimate.magnitude_valid != 0U) ? 1U : 0U;
	observation.stopped = s_esc_motion_estimate.stopped;
	observation.stop_established_valid = s_esc_motion_estimate.stopped;
	observation.moving_observed = s_esc_motion_estimate.moving_observed;
	observation.stop_established_tick_ms =
		s_esc_motion_estimate.stop_established_tick_ms;
	observation.sample_tick_ms = s_esc_motion_estimate.last_sample_tick_ms;
	observation.esc_action = servo_basic_mode2_esc_action(now_ms);
	return observation;
}

static uint8_t servo_basic_mode2_application_config_valid(void)
{
	return (s_mode2_drive_gate.config_valid != 0U &&
		s_longitudinal_controller.config_valid != 0U) ? 1U : 0U;
}

static uint16_t mode2_brake_request_to_pwm(Mode2DriveAction_t action,
										   Mode2DrivePhase_t phase,
										   float request,
										   float *actual_normalized_brake)
{
	const uint16_t center_us = s_mode2_drive_gate.config.center_pwm_us;
	uint16_t full_pwm_us;
	int32_t final_us;
	int32_t brake_span_us;
	int32_t actual_delta_us;
	int32_t span_abs_us;

	if (actual_normalized_brake != NULL)
	{
		*actual_normalized_brake = 0.0f;
	}
	request = clamp_unit_float(request);
	if (request <= 0.0f || s_mode2_drive_gate.config_valid == 0U)
	{
		return center_us;
	}

	if (action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE)
	{
		full_pwm_us = (phase == MODE2_DRIVE_PHASE_IDLE) ?
			get_orin_esc_forward_limit_pulse() :
			s_mode2_drive_gate.config.rev_to_fwd_brake_full_pwm_us;
		brake_span_us = (int32_t)full_pwm_us -
			(int32_t)center_us;
	}
	else
	{
		full_pwm_us = (phase == MODE2_DRIVE_PHASE_IDLE) ?
			get_orin_esc_reverse_limit_pulse() :
			s_mode2_drive_gate.config.fwd_to_rev_brake_full_pwm_us;
		brake_span_us = (int32_t)full_pwm_us -
			(int32_t)center_us;
	}
	final_us = (int32_t)center_us +
		float_to_i32_nearest((float)brake_span_us * request);
	final_us = (int32_t)clamp_esc_pulse_i32(final_us);
	if (action == MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE)
	{
		if (final_us <= (int32_t)center_us)
		{
			final_us = (int32_t)center_us;
			actual_delta_us = 0;
		}
		else
		{
			actual_delta_us = final_us - (int32_t)center_us;
		}
		span_abs_us = (int32_t)full_pwm_us -
			(int32_t)center_us;
	}
	else if (final_us >= (int32_t)center_us)
	{
		final_us = (int32_t)center_us;
		actual_delta_us = 0;
		span_abs_us = (int32_t)center_us - (int32_t)full_pwm_us;
	}
	else
	{
		actual_delta_us = (int32_t)center_us - final_us;
		span_abs_us = (int32_t)center_us - (int32_t)full_pwm_us;
	}
	if (actual_normalized_brake != NULL)
	{
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
	case MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE:
		return (final_esc_pulse < center_us && actual_normalized_brake > 0.0f) ?
			MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE : MODE2_DRIVE_ACTION_NEUTRAL;
	case MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE:
		return (final_esc_pulse > center_us && actual_normalized_brake > 0.0f) ?
			MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE : MODE2_DRIVE_ACTION_NEUTRAL;
	case MODE2_DRIVE_ACTION_NEUTRAL:
	default:
		return MODE2_DRIVE_ACTION_NEUTRAL;
	}
}

static uint16_t mode2_drive_output_to_esc_pwm(Mode2DriveAction_t action,
											  Mode2DrivePhase_t phase,
											  float normalized_brake_request,
											  const LongitudinalControllerOutput_t *control_output,
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
		return longitudinal_drive_output_to_pwm(control_output);
	case MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE:
	case MODE2_DRIVE_ACTION_REV_TO_FWD_BRAKE:
		return mode2_brake_request_to_pwm(action,
			phase,
			normalized_brake_request,
			actual_normalized_brake);
	case MODE2_DRIVE_ACTION_NEUTRAL:
	default:
		g_speed_pi_final_us = get_orin_esc_center_pulse();
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

	Mode2DriveGate_CommitAppliedActionWithPwmEvidence(&s_mode2_drive_gate,
		actual_action,
		now_ms,
		final_esc_pulse);
	EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
		Mode2DriveGate_ToMotionAction(actual_action),
		now_ms);
	if (s_mode2_drive_gate.state == MODE2_DRIVE_STATE_FORWARD_TRACKING ||
		s_mode2_drive_gate.state == MODE2_DRIVE_STATE_FORWARD_BRAKE_CONTINUOUS)
	{
		s_vehicle_direction_known = 1U;
		s_vehicle_direction = 1;
	}
	else if (s_mode2_drive_gate.state == MODE2_DRIVE_STATE_REVERSE_TRACKING ||
		s_mode2_drive_gate.state == MODE2_DRIVE_STATE_REVERSE_BRAKE_CONTINUOUS)
	{
		s_vehicle_direction_known = 1U;
		s_vehicle_direction = -1;
	}
}

static uint8_t servo_basic_auto_propulsion_authorized(void)
{
	return (g_orin_state.auto_enabled != 0U &&
		s_esc_feedback_available != 0U &&
		servo_basic_current_esc_action(HAL_GetTick()) != SERVO_ESC_ACTION_UNKNOWN &&
		s_esc_motion_estimator.config_valid != 0U &&
		s_esc_motion_estimator.stop_config_valid != 0U &&
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
	const float min_vx_mps = g_esc_motion_stopped_speed_threshold_mps;
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
	float clamped_steering_angle_rad;
	float dt_s = 0.0f;
	uint32_t now_ms;
	uint8_t steering_saturated = 0U;
	uint8_t steering_rate_limited = 0U;
	const uint8_t auto_enabled = (enable != 0U) ? 1U : 0U;
	const uint8_t brake_active = (brake != 0U) ? 1U : 0U;
	const uint8_t estop_active = (emergency_stop != 0U) ? 1U : 0U;
	const uint8_t force_zero_speed = (auto_enabled == 0U || brake_active != 0U || estop_active != 0U) ? 1U : 0U;

	(void)get_orin_ackermann_track_width_mm();
	(void)get_orin_ackermann_wheel_radius_mm();

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
	requested_speed_mps = (force_zero_speed != 0U) ? 0.0f : speed_mps;
	if (force_zero_speed == 0U && g_orin_state.active != 0U && dt_s > 0.0f)
	{
		clamped_steering_angle_rad = apply_rate_limit_float(
			clamped_steering_angle_rad,
			g_orin_state.target_steering_angle_rad,
			(float)get_orin_steering_rate_limit_mradps() / 1000.0f,
			dt_s,
			&steering_rate_limited);
	}

	if (estop_active != 0U)
	{
		g_orin_state.servo_pulse_us = get_orin_servo_center_pulse();
		speed_pi_reset_controller();
	}
	else
	{
		g_orin_state.servo_pulse_us = orin_map_steering_to_servo(clamped_steering_angle_rad);
	}

	g_orin_state.target_speed_mps = (force_zero_speed != 0U) ? 0.0f : requested_speed_mps;
	g_orin_state.target_steering_angle_rad = clamped_steering_angle_rad;
	g_orin_state.last_update_ms = now_ms;
	g_orin_state.active = 1U;
	g_orin_state.software_stop = estop_active;
	g_orin_state.auto_enabled = auto_enabled;
	g_orin_state.brake_active = brake_active;
	g_orin_state.steering_saturated = steering_saturated;
	g_orin_state.accel_limited = 0U;
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
	if (g_orin_state.active == 0U)
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
	return (g_esc_speed_fresh_timeout_ms != 0U &&
		age_ms <= g_esc_speed_fresh_timeout_ms) ? 1U : 0U;
}

static servo_esc_action_t servo_basic_current_esc_action(uint32_t now_ms)
{
	if (servo_basic_esc_fe32_is_fresh(now_ms) == 0U ||
		s_esc_latest_raw_sample_valid == 0U ||
		s_esc_latest_raw_sample.state_candidate_valid == 0U)
	{
		return SERVO_ESC_ACTION_UNKNOWN;
	}

	switch (s_esc_latest_raw_sample.state_candidate)
	{
	case ESC_FE32_STATE_CANDIDATE_NEUTRAL:
		return SERVO_ESC_ACTION_NEUTRAL;
	case ESC_FE32_STATE_CANDIDATE_DRIVE_AMBIGUOUS:
		return SERVO_ESC_ACTION_DRIVE;
	case ESC_FE32_STATE_CANDIDATE_BRAKE:
		return SERVO_ESC_ACTION_BRAKE;
	default:
		return SERVO_ESC_ACTION_UNKNOWN;
	}
}

static uint8_t servo_basic_compute_uplink_speed_magnitude(uint32_t now_ms,
												  float *speed_magnitude_mps)
{
	(void)now_ms;

	if (speed_magnitude_mps != NULL)
	{
		*speed_magnitude_mps = 0.0f;
	}
	if (s_esc_rx_invalidated != 0U ||
		s_esc_motion_estimate.config_valid == 0U ||
		s_esc_motion_estimate.has_sample == 0U ||
		s_esc_motion_estimate.sample_fresh == 0U ||
		s_esc_motion_estimate.rpm_valid == 0U ||
		s_esc_motion_estimate.magnitude_valid == 0U)
	{
		return 0U;
	}
	if (speed_magnitude_mps != NULL)
	{
		*speed_magnitude_mps = s_esc_motion_estimate.speed_magnitude_mps;
	}
	return 1U;
}

static int8_t servo_basic_estimated_vehicle_direction(void)
{
	if (g_rc_override_active != 0U)
	{
		return (s_rc_direction_result.direction_known != 0U) ?
			(int8_t)s_rc_direction_result.direction : 0;
	}
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
	const int8_t direction = servo_basic_estimated_vehicle_direction();
	const uint16_t servo_pulse = (state != NULL) ?
		state->servo_pulse_us : get_orin_servo_center_pulse();
	const float steering_rad =
		telemetry_estimate_steering_rad_from_servo_pulse(servo_pulse);
	float yaw_rate;

	if (s_esc_stop_confirmed != 0U)
	{
		feedback_speed = 0.0f;
		speed_valid = 1U;
	}
	else if (s_esc_feedback_available != 0U &&
		s_esc_motion_estimate.magnitude_valid != 0U &&
		direction != 0)
	{
		feedback_speed = (direction < 0) ?
			-s_esc_motion_estimate.speed_magnitude_mps :
			s_esc_motion_estimate.speed_magnitude_mps;
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
	const Mode2DriveState_t mode2_state = s_mode2_drive_gate.state;
	const uint8_t mode2_opposite_armed =
		(mode2_state == MODE2_DRIVE_STATE_FORWARD_ARMED ||
		 mode2_state == MODE2_DRIVE_STATE_REVERSE_ARMED) ? 1U : 0U;
	const uint8_t mode2_state_ambiguous =
		(mode2_state == MODE2_DRIVE_STATE_UNKNOWN_SAFE ||
		 mode2_state == MODE2_DRIVE_STATE_FORWARD_MAYBE_ARMED ||
		 mode2_state == MODE2_DRIVE_STATE_REVERSE_MAYBE_ARMED ||
		 mode2_state == MODE2_DRIVE_STATE_FORWARD_RECOVERY_PENDING) ? 1U : 0U;
	float uplink_speed_magnitude_mps = 0.0f;
	const uint8_t uplink_speed_valid =
		servo_basic_compute_uplink_speed_magnitude(now_ms,
			&uplink_speed_magnitude_mps);

	diagnostics.speed_saturated = (g_speed_pi_saturated != 0U) ? 1U : 0U;
	diagnostics.steering_saturated = g_orin_state.steering_saturated;
	diagnostics.accel_limited = g_orin_state.accel_limited;
	diagnostics.steering_rate_limited = g_orin_state.steering_rate_limited;
	diagnostics.steering_fault = (g_rc_input_fault_active != 0U) ? 1U : 0U;
	diagnostics.esc_feedback_valid =
		(s_esc_stop_confirmed != 0U ||
		 (s_esc_feedback_available != 0U &&
		  s_esc_motion_estimate.magnitude_valid != 0U && direction != 0)) ? 1U : 0U;
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
		(s_esc_motion_estimator.config_valid != 0U) ? 1U : 0U;
	diagnostics.esc_action = (uint8_t)servo_basic_current_esc_action(now_ms);
	diagnostics.vehicle_direction_known = (direction != 0) ? 1U : 0U;
	diagnostics.esc_soft_uart_rx_error =
		(s_esc_receiver_health.rx_error_count != 0U) ? 1U : 0U;
	diagnostics.mode2_config_valid =
		(servo_basic_mode2_application_config_valid() != 0U) ? 1U : 0U;
	diagnostics.auto_propulsion_authorized = servo_basic_auto_propulsion_authorized();
	diagnostics.closed_loop_active =
		(diagnostics.auto_propulsion_authorized != 0U &&
		 g_rc_override_active == 0U && orin_pwm_is_active_at(now_ms) != 0U) ? 1U : 0U;
	diagnostics.tracking_brake_active =
		(s_longitudinal_output.intent == LONGITUDINAL_INTENT_TRACKING_BRAKE) ? 1U : 0U;
	diagnostics.mode2_opposite_armed = mode2_opposite_armed;
	diagnostics.mode2_state_ambiguous = mode2_state_ambiguous;
	diagnostics.mode2_control_inhibited =
		(g_orin_state.auto_enabled != 0U &&
		 (diagnostics.auto_propulsion_authorized == 0U ||
		  mode2_state_ambiguous != 0U)) ? 1U : 0U;
	diagnostics.mode2_state = (uint8_t)mode2_state;
	diagnostics.mode2_reason = (uint8_t)s_mode2_drive_output.reason;
	diagnostics.longitudinal_intent = (uint8_t)s_longitudinal_output.intent;
	diagnostics.longitudinal_reason = (uint8_t)s_longitudinal_output.reason;
	diagnostics.longitudinal_slewed_target_mps =
		s_longitudinal_output.diagnostics.slewed_target_mps;
	diagnostics.esc_sample_stale = s_esc_sample_stale;
	diagnostics.esc_rx_invalidated = s_esc_rx_invalidated;
	if (diagnostics.esc_feedback_valid != 0U)
	{
		diagnostics.esc_feedback_direction = direction;
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
	if (g_orin_state.active == 0U)
	{
		snapshot.orin_command_timeout = 1U;
	}
	else
	{
		snapshot.orin_command_timeout = (orin_active == 0U) ? 1U : 0U;
	}
	snapshot.orin_auto_enabled =
		(orin_active != 0U &&
		 g_rc_override_active == 0U &&
		 g_orin_state.software_stop == 0U &&
		 g_orin_state.auto_enabled != 0U) ? 1U : 0U;
	snapshot.orin_brake_active =
		(orin_active != 0U && g_orin_state.brake_active != 0U) ? 1U : 0U;
	snapshot.orin_emergency_active =
		(orin_active != 0U && g_orin_state.software_stop != 0U) ? 1U : 0U;
	snapshot.control_tick_ms = now_ms;
	snapshot.esc_sample_id =
		(s_esc_latest_raw_sample_valid != 0U) ?
		s_esc_latest_raw_sample.sample_id : s_esc_motion_estimate.last_sample_id;
	snapshot.esc_receive_epoch = s_esc_receive_epoch;
	snapshot.esc_sample_tick_ms =
		(s_esc_latest_raw_sample_valid != 0U) ?
		s_esc_latest_raw_sample.received_tick_ms : s_esc_motion_estimate.last_sample_tick_ms;
	snapshot.publish_sequence = ++s_control_snapshot_next_sequence;
	if (g_rc_override_active != 0U && s_esc_latest_raw_sample_valid != 0U &&
		(snapshot.esc_sample_id != s_rc_last_published_sample_id ||
		 snapshot.esc_receive_epoch != s_rc_last_published_receive_epoch))
	{
		uint32_t age_ms;
		if (servo_basic_tick_delta_ms(HAL_GetTick(), snapshot.esc_sample_tick_ms,
			&age_ms) != 0U && age_ms > s_observation_diagnostics.max_publish_age_ms)
		{
			s_observation_diagnostics.max_publish_age_ms = age_ms;
		}
		s_rc_last_published_sample_id = snapshot.esc_sample_id;
		s_rc_last_published_receive_epoch = snapshot.esc_receive_epoch;
	}

	irq_state = servo_basic_enter_critical();
	s_control_snapshot = snapshot;
	servo_basic_exit_critical(irq_state);
}

static void update_control_mode_from_rc(void)
{
	const uint8_t manual_override = rc_manual_override_requested();
	const uint8_t centered = rc_inputs_are_centered();
	const uint8_t serial_active = orin_pwm_is_active();
	const uint8_t rc_available = rc_passthrough_is_available();

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
			if (g_rc_override_active == 0U)
			{
				set_rc_override_state(1U, 0U);
			}
		}
		else if (g_rc_override_active != 0U)
		{
			set_rc_override_state(0U, 0U);
		}
		else
		{
			g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
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

	if (g_rc_override_active != 0U &&
		g_rc_override_release_hold_required == 0U && centered != 0U)
	{
		set_rc_override_state(0U, 0U);
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
				set_rc_override_state(1U, 1U);
			}
		}
		else
		{
			g_rc_override_enter_count = 0U;
			g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
		}
		return;
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
		set_rc_override_state(0U, 0U);
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

static void servo_basic_update_rc_direction_observer(uint32_t now_ms, uint16_t pwm_us)
{
	RcDirectionObserverInput_t input;

	memset(&input, 0, sizeof(input));
	input.rc_active = g_rc_override_active;
	input.applied_pwm_us = pwm_us;
	if (s_esc_latest_raw_sample_valid != 0U &&
		s_esc_rx_invalidated == 0U &&
		servo_basic_esc_fe32_is_fresh(now_ms) != 0U)
	{
		input.telemetry_fresh = 1U;
		input.sample_id = s_esc_latest_raw_sample.sample_id;
		input.state_raw =
			(s_esc_latest_raw_sample.state_candidate_valid != 0U) ?
			s_esc_latest_raw_sample.state_raw : 0xFFU;
		input.rpm_valid = s_esc_latest_raw_sample.rpm_valid;
		input.rpm_raw = s_esc_latest_raw_sample.rpm_raw;
		input.moving_evidence =
			(s_esc_motion_estimate.last_sample_id ==
			 s_esc_latest_raw_sample.sample_id &&
			 s_esc_motion_estimate.moving_observed != 0U) ? 1U : 0U;
	}

	s_rc_direction_result = RcDirectionObserver_Update(
		&s_rc_direction_observer,
		&input);
}

static void servo_basic_reset_rc_evidence(void)
{
	RcDirectionObserver_ResetDirection(&s_rc_direction_observer);
	s_rc_direction_result = RcDirectionObserver_GetResult(&s_rc_direction_observer);
	EscMotionEstimator_InvalidateStopEvidence(&s_esc_motion_estimator);
}

static uint32_t servo_basic_observation_cycles(void)
{
#if defined(STM32F407xx)
	return DWT->CYCCNT;
#else
	return 0U;
#endif
}

static void servo_basic_update_rc_feedback(uint32_t now_ms)
{
	EscTelemetrySnapshot_t latest;
	EscTelemetryObservedSample_t item;
	size_t remaining = EscTelemetry_PendingSamples();
	uint32_t epoch = EscTelemetry_GetDeliveryEpoch();
	uint32_t processed = 0U;
	const uint32_t start_cycles = servo_basic_observation_cycles();
	uint32_t elapsed_cycles;

	EscTelemetry_GetReceiverHealth(&s_esc_receiver_health);
	if (s_rc_delivery_epoch != epoch)
	{
		s_observation_diagnostics.delivery_gaps++;
		servo_basic_reset_rc_evidence();
		s_rc_delivery_epoch = epoch;
	}
	if (EscTelemetry_GetSnapshot(&latest) == 0U)
	{
		s_esc_rx_invalidated = 1U;
		s_esc_latest_raw_sample_valid = 0U;
		servo_basic_reset_rc_evidence();
		servo_basic_update_cached_estimate(now_ms);
		return;
	}
	s_esc_rx_invalidated = 0U;
	s_esc_receive_epoch = latest.receive_epoch;
	s_esc_receive_epoch_valid = 1U;

	/* Bound work to the batch available on entry; do not chase the producer. */
	while (remaining-- != 0U && EscTelemetry_PopSample(&item) != 0U)
	{
		uint32_t age_ms;
		const int8_t old_direction = s_rc_direction_result.direction_known != 0U ?
			(int8_t)s_rc_direction_result.direction : 0;
		if (item.receive_epoch != latest.receive_epoch ||
			EscTelemetry_ContextSource(item.output_context) !=
				EscTelemetry_ContextSource(s_rc_output_context) ||
			EscTelemetry_ContextRcActive(item.output_context) == 0U)
		{
			s_observation_diagnostics.samples_wrong_source++;
			continue;
		}
		if (s_rc_delivery_epoch != item.delivery_epoch)
		{
			s_observation_diagnostics.delivery_gaps++;
			servo_basic_reset_rc_evidence();
			s_rc_delivery_epoch = item.delivery_epoch;
		}
		if (servo_basic_tick_delta_ms(now_ms, item.sample.received_tick_ms, &age_ms) == 0U ||
			g_esc_speed_fresh_timeout_ms == 0U || age_ms > g_esc_speed_fresh_timeout_ms)
		{
			s_observation_diagnostics.samples_expired++;
			servo_basic_reset_rc_evidence();
			continue;
		}
		if (item.sample.sample_id == s_esc_last_observed_sample_id &&
			item.receive_epoch == s_esc_last_observed_epoch)
		{
			continue;
		}
		s_esc_latest_raw_sample = item.sample;
		s_esc_latest_raw_sample_valid = 1U;
		(void)EscMotionEstimator_ObserveSample(&s_esc_motion_estimator, &item.sample, now_ms);
		s_esc_last_observed_sample_id = item.sample.sample_id;
		s_esc_last_observed_epoch = item.receive_epoch;
		servo_basic_update_cached_estimate(now_ms);
		servo_basic_update_rc_direction_observer(now_ms,
			EscTelemetry_ContextPwm(item.output_context));
		/* Preserve Hall period invalidation even if a batch changes direction twice. */
		if (s_rc_direction_result.direction_known != 0U &&
			(int8_t)s_rc_direction_result.direction != old_direction &&
			s_esc_stop_confirmed == 0U)
		{
			HallSpeed_SetCommandDirection((int8_t)s_rc_direction_result.direction);
		}
		s_observation_diagnostics.samples_processed++;
		s_observation_diagnostics.last_sample_id = item.sample.sample_id;
		if (age_ms > s_observation_diagnostics.max_sample_age_ms)
		{
			s_observation_diagnostics.max_sample_age_ms = age_ms;
		}
		processed++;
	}
	/* Context rejection must not hide a valid RPM measurement. Raw snapshot
	 * and FIFO are published atomically; an unqueued latest sample has no
	 * usable RC context, so publish magnitude only and reset sign/stop proof. */
	if ((latest.sample.sample_id != s_esc_last_observed_sample_id ||
		 latest.receive_epoch != s_esc_last_observed_epoch) &&
		EscTelemetry_PendingSamples() == 0U)
	{
		uint32_t age_ms;
		servo_basic_reset_rc_evidence();
		if (servo_basic_tick_delta_ms(now_ms, latest.sample.received_tick_ms, &age_ms) != 0U &&
			g_esc_speed_fresh_timeout_ms != 0U && age_ms <= g_esc_speed_fresh_timeout_ms)
		{
			s_esc_latest_raw_sample = latest.sample;
			s_esc_latest_raw_sample_valid = 1U;
			(void)EscMotionEstimator_ObserveSample(&s_esc_motion_estimator, &latest.sample, now_ms);
			EscMotionEstimator_InvalidateStopEvidence(&s_esc_motion_estimator);
			s_esc_last_observed_sample_id = latest.sample.sample_id;
			s_esc_last_observed_epoch = latest.receive_epoch;
			s_observation_diagnostics.samples_magnitude_only++;
		}
	}
	/* Silence/receiver errors still invalidate evidence without a new frame. */
	if (servo_basic_esc_fe32_is_fresh(now_ms) == 0U)
	{
		servo_basic_reset_rc_evidence();
	}
	epoch = EscTelemetry_GetDeliveryEpoch();
	if (epoch != s_rc_delivery_epoch)
	{
		s_observation_diagnostics.delivery_gaps++;
		servo_basic_reset_rc_evidence();
		s_rc_delivery_epoch = epoch;
	}
	servo_basic_update_cached_estimate(now_ms);
	if (processed > s_observation_diagnostics.max_batch_samples)
	{
		s_observation_diagnostics.max_batch_samples = processed;
	}
	elapsed_cycles = servo_basic_observation_cycles() - start_cycles;
	if (elapsed_cycles > s_observation_diagnostics.max_batch_cycles)
	{
		s_observation_diagnostics.max_batch_cycles = elapsed_cycles;
	}
}

void ServoBasic_ProcessEscObservation(void)
{
	const uint32_t now_ms = HAL_GetTick();
	if (g_rc_override_active == 0U)
	{
		/* AUTO's estimator/PI/gate must run only at the control deadline. */
		return;
	}
	servo_basic_update_rc_feedback(now_ms);
	HallSpeed_SetCommandDirection(
		(s_rc_direction_result.direction_known != 0U && s_esc_stop_confirmed == 0U) ?
		(int8_t)s_rc_direction_result.direction : 0);
	servo_basic_publish_control_snapshot(now_ms);
}

servo_basic_observation_diagnostics_t ServoBasic_GetObservationDiagnostics(void)
{
	servo_basic_observation_diagnostics_t result;
	ServoBasicIrqState_t irq_state = servo_basic_enter_critical();
	result = s_observation_diagnostics;
	servo_basic_exit_critical(irq_state);
	return result;
}

void ServoBasic_ProcessControl(void)
{
	uint8_t orin_active;
	uint32_t now_ms;

	now_ms = HAL_GetTick();
	servo_basic_refresh_esc_configs(now_ms);
	if (g_rc_override_active != 0U)
	{
		servo_basic_update_rc_feedback(now_ms);
	}
	else
	{
		EscTelemetry_DiscardSamples();
		servo_basic_update_esc_feedback(now_ms);
	}
	refresh_rc_inputs();
	update_control_mode_from_rc();
	orin_active = orin_pwm_is_active();
	if (g_rc_override_active != 0U)
	{
		speed_pi_reset_controller();
	}
	else if (orin_active == 0U)
	{
		speed_pi_reset_controller();
		if (g_orin_state.active != 0U)
		{
			servo_basic_invalidate_auto_history(now_ms);
		}
	}

	if (g_rc_override_active != 0U)
	{
		apply_rc_passthrough_outputs();
	}
	else if (orin_active != 0U)
	{
		if (g_orin_state.software_stop != 0U)
		{
			servo_basic_invalidate_auto_history(now_ms);
			speed_pi_reset_controller();
			apply_esc_pulse(get_orin_esc_center_pulse());
			apply_servo_pulse(get_orin_servo_center_pulse());
		}
		else
		{
			LongitudinalControllerInput_t control_input;
			LongitudinalControllerOutput_t control_output;
			Mode2DriveGateInput_t mode2_input;
			Mode2DriveMotionObservation_t mode2_observation;
			float actual_normalized_brake = 0.0f;
			uint16_t final_esc_pulse;
			control_input = servo_basic_build_longitudinal_input(now_ms);
			control_output = LongitudinalController_Evaluate(
				&s_longitudinal_controller,
				&control_input);
			s_longitudinal_output = control_output;
			update_speed_watch_from_longitudinal(&control_output);
			mode2_input = servo_basic_mode2_input_from_longitudinal(
					&control_output);
			mode2_observation = servo_basic_build_mode2_observation(now_ms);

			s_mode2_drive_output = Mode2DriveGate_EvaluateWithObservation(&s_mode2_drive_gate,
					&mode2_input,
					&mode2_observation,
					now_ms);
			final_esc_pulse = mode2_drive_output_to_esc_pwm(
				s_mode2_drive_output.action,
				s_mode2_drive_output.phase,
				s_mode2_drive_output.normalized_brake_request,
				&control_output,
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
		Mode2DriveGate_CommitAppliedActionWithPwmEvidence(&s_mode2_drive_gate,
			MODE2_DRIVE_ACTION_NEUTRAL,
			now_ms,
			get_orin_esc_center_pulse());
		EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
			ESC_MOTION_APPLIED_ACTION_NEUTRAL,
			now_ms);
	}

	if (g_rc_override_active != 0U)
	{
		HallSpeed_SetCommandDirection(
			(s_rc_direction_result.direction_known != 0U &&
			 s_esc_stop_confirmed == 0U) ?
			(int8_t)s_rc_direction_result.direction : 0);
	}
	else if (orin_active != 0U &&
		g_orin_state.software_stop == 0U &&
		s_vehicle_direction_known != 0U)
	{
		HallSpeed_SetCommandDirection(s_vehicle_direction);
	}
	else
	{
		HallSpeed_SetCommandDirection(0);
	}

	servo_basic_publish_control_snapshot(now_ms);
}

void ServoBasic_Task(void *param)
{
	(void)param;

	TickType_t next_control = xTaskGetTickCount();
	const TickType_t period_ticks = pdMS_TO_TICKS(20U);
	for (;;)
	{
		TickType_t now = xTaskGetTickCount();
		const TickType_t lateness = now - next_control;
		if (lateness < (TickType_t)0x80000000UL)
		{
			TickType_t periods;
			if (lateness > s_observation_diagnostics.max_control_lateness_ticks)
			{
				s_observation_diagnostics.max_control_lateness_ticks = lateness;
			}
			ServoBasic_ProcessControl();
			/* Advance the original deadline, without replaying missed commands. */
			now = xTaskGetTickCount();
			periods = (now - next_control) / period_ticks + 1U;
			s_observation_diagnostics.control_deadlines_skipped += periods - 1U;
			next_control += periods * period_ticks;
		}
		else
		{
			ServoBasic_ProcessEscObservation();
		}
		now = xTaskGetTickCount();
		if ((TickType_t)(next_control - now) < (TickType_t)0x80000000UL &&
			next_control != now)
		{
			(void)ulTaskNotifyTake(pdTRUE, next_control - now);
		}
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
