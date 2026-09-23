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
#define ORIN_SERVO_RANGE_DEFAULT_US              APP_ORIN_SERVO_RANGE_US
#define ORIN_STEERING_RATE_LIMIT_DEFAULT_MRADPS  APP_ORIN_STEERING_RATE_LIMIT_MRADPS
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
volatile uint32_t g_orin_servo_center_us = APP_ORIN_SERVO_CENTER_US;
volatile uint32_t g_orin_servo_range_us = ORIN_SERVO_RANGE_DEFAULT_US;
volatile float g_esc_motion_stopped_speed_threshold_mps = APP_ESC_STOPPED_THRESHOLD_MPS_DEFAULT;
volatile uint32_t g_esc_motion_stopped_min_samples = APP_ESC_STOPPED_MIN_SAMPLES_DEFAULT;
volatile uint32_t g_esc_motion_stopped_min_coverage_ms = APP_ESC_STOPPED_MIN_COVERAGE_MS_DEFAULT;
volatile float g_esc_low_gear_wheel_rpm_per_raw =
	APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT;
volatile uint32_t g_esc_speed_fresh_timeout_ms =
	APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT;
volatile uint32_t g_orin_steering_rate_limit_mradps = ORIN_STEERING_RATE_LIMIT_DEFAULT_MRADPS;
volatile float g_speed_pid_kp = APP_SPEED_PID_KP_DEFAULT_US_PER_MPS;
volatile float g_speed_pid_ki = APP_SPEED_PID_KI_DEFAULT_US_PER_MPS_S;
volatile float g_speed_pid_kd = APP_SPEED_PID_KD_DEFAULT_US_PER_MPS2;
volatile uint32_t g_speed_pid_derivative_tau_ms = APP_SPEED_PID_DERIVATIVE_TAU_MS;
volatile uint32_t g_speed_pid_tracking_tau_ms = APP_SPEED_PID_TRACKING_TAU_MS;
volatile float g_speed_pid_target_mps;
volatile float g_speed_pid_feedback_mps;
volatile float g_speed_pid_error_mps;
volatile float g_speed_pid_integral_us;
volatile float g_speed_pid_raw_output_us;
volatile uint32_t g_speed_pid_final_us = ESC_PWM_NEUTRAL_PULSE_US;
volatile uint32_t g_speed_pid_feedback_valid;
volatile uint32_t g_speed_pid_saturated;
volatile uint32_t g_auto_brake_min_us = APP_AUTO_BRAKE_MIN_US;
volatile float g_auto_brake_enter_error_mps = APP_AUTO_BRAKE_ENTER_ERROR_MPS;
volatile float g_auto_brake_enter_ratio = APP_AUTO_BRAKE_ENTER_RATIO;
volatile float g_auto_brake_release_error_mps = APP_AUTO_BRAKE_RELEASE_ERROR_MPS;
volatile float g_auto_brake_release_ratio = APP_AUTO_BRAKE_RELEASE_RATIO;
volatile uint32_t g_auto_coast_eval_ms = APP_AUTO_COAST_EVAL_MS;
volatile uint32_t g_auto_coast_budget_ms = APP_AUTO_COAST_BUDGET_MS;
volatile float g_auto_a_progress_mps2 = APP_AUTO_A_PROGRESS_MPS2;
volatile uint32_t g_auto_brake_release_delay_ms = APP_AUTO_BRAKE_RELEASE_DELAY_MS;
volatile uint32_t g_auto_action_ack_ms = APP_AUTO_ACTION_ACK_MS;
volatile uint32_t g_auto_qualify_ms = APP_AUTO_QUALIFY_MS;
volatile uint32_t g_auto_neutral_dwell_ms = APP_AUTO_NEUTRAL_DWELL_MS;
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
static RcDirectionObserver_t s_auto_direction_observer;
static RcDirectionObserverResult_t s_auto_direction_result;
static uint32_t s_auto_delivery_epoch;
static uint8_t s_auto_authorized_previous;
static uint8_t s_auto_fault_pending;
static uint8_t s_pid_reset_requested;
static uint16_t s_active_esc_center = ESC_PWM_NEUTRAL_PULSE_US;

/* Serial producer publishes a mailbox; only Servo changes control state. */
typedef struct
{
    float speed_mps;
    float steering_angle_rad;
    uint32_t received_ms;
    uint32_t sequence;
    uint8_t enable;
    uint8_t brake;
    uint8_t software_stop;
    uint8_t authority_cancel_seen;
} ServoBasicCommand_t;
static ServoBasicCommand_t s_pending_command;
static uint32_t s_consumed_command_sequence;

static uint16_t limit_servo_safe_pulse(uint16_t pulse_us);
static void speed_pid_reset_controller(void);
static uint32_t get_rc_override_center_us(void);
static uint8_t pulse_is_inside_center(uint16_t pulse_us, uint32_t center_us, uint32_t threshold_us);
static uint8_t orin_pwm_is_active(void);
static void servo_basic_refresh_esc_configs(uint32_t now_ms);
static void servo_basic_update_esc_feedback(uint32_t now_ms);
static void servo_basic_invalidate_auto_history(uint32_t now_ms);
static uint8_t longitudinal_config_equal(const LongitudinalControllerConfig_t *left,
										 const LongitudinalControllerConfig_t *right);
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
static uint32_t servo_basic_observation_cycles(void);

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
    return s_active_esc_center;
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
        if (g_rc_override_active == 0U)
            EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
                ESC_MOTION_APPLIED_ACTION_EXTERNAL_OVERRIDE, HAL_GetTick());
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
    EscTelemetryPreparedOutputContext_t prepared;
    EscTelemetryOutputPurpose_t purpose;
    uint32_t session;
#if defined(STM32F407xx)
    uint32_t irq_mask;
#endif
    if (g_rc_override_active == 0U &&
        (s_mode2_drive_output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL ||
         s_mode2_drive_gate.config_valid == 0U))
    {
        /* Invalid configuration must never turn an invalid center into an endpoint. */
        pulse_us = get_orin_esc_center_pulse();
        s_mode2_drive_output.purpose = ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL;
    }
    if (g_rc_override_active == 0U && s_esc_stop_confirmed != 0U &&
        s_auto_direction_result.direction_known != 0U &&
        ((s_mode2_drive_output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST &&
          pulse_us < get_orin_esc_center_pulse() &&
          s_auto_direction_result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD) ||
         (s_mode2_drive_output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST &&
          pulse_us > get_orin_esc_center_pulse() &&
          s_auto_direction_result.direction == RC_DIRECTION_OBSERVER_DIRECTION_REVERSE)))
    {
        /* Old motion ended. Opposite propulsion must obtain new DRIVE evidence. */
        RcDirectionObserver_ResetDirection(&s_auto_direction_observer);
        s_auto_direction_result = RcDirectionObserver_GetResult(&s_auto_direction_observer);
        servo_basic_clear_vehicle_direction();
        LongitudinalController_Reset(&s_longitudinal_controller);
    }
    if (g_rc_override_active != 0U)
    {
        purpose = ESC_TELEMETRY_OUTPUT_PURPOSE_RC_DIRECT;
        session = 0U;
    }
    else
    {
        Mode2DriveGate_CommitApplied(&s_mode2_drive_gate, &s_mode2_drive_output,
            pulse_us, HAL_GetTick());
        purpose = s_mode2_drive_output.purpose;
        session = s_mode2_drive_output.session_id;
    }
    (void)EscTelemetry_PrepareOutputContext(pulse_us, g_rc_override_active,
        purpose, session, &prepared);
    g_state.esc_pulse_us = pulse_us;
    /* Keep generation arithmetic and gate decisions outside this tiny window.
     * PWM is started in Init before PD15 reception is enabled; later calls
     * update only the timer compare. The byte ISR sees one committed pair. */
#if defined(STM32F407xx)
    irq_mask = __get_PRIMASK();
    __disable_irq();
#endif
    ServoBasic_OutputEscPulse(pulse_us);
    EscTelemetry_CommitOutputContext(&prepared);
#if defined(STM32F407xx)
    __set_PRIMASK(irq_mask);
#endif
    s_rc_output_context = prepared.output_context;
}

static void speed_pid_reset_controller(void)
{
    if (s_longitudinal_config_initialized != 0U)
    {
        LongitudinalController_Reset(&s_longitudinal_controller);
    }
    memset(&s_longitudinal_output, 0, sizeof(s_longitudinal_output));
    s_mode2_drive_gate.release_predict_count = 0U;
    s_mode2_drive_gate.latest_observation.acceleration_valid = 0U;
    g_speed_pid_feedback_valid = 0U;
    g_speed_pid_saturated = 0U;
    g_speed_pid_error_mps = 0.0f;
    g_speed_pid_integral_us = 0.0f;
    g_speed_pid_raw_output_us = 0.0f;
}

void ServoBasic_ResetSpeedPid(void)
{
    ServoBasicIrqState_t state = servo_basic_enter_critical();
    s_pid_reset_requested = 1U;
    servo_basic_exit_critical(state);
}

void ServoBasic_SetSpeedPidGains(float kp, float ki, float kd)
{
    ServoBasicIrqState_t state = servo_basic_enter_critical();
    g_speed_pid_kp = kp;
    g_speed_pid_ki = ki;
    g_speed_pid_kd = kd;
    s_pid_reset_requested = 1U;
    servo_basic_exit_critical(state);
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
    config.center_pwm_us = (g_orin_esc_center_us <= UINT16_MAX) ?
        (uint16_t)g_orin_esc_center_us : 0U;
    config.forward_pwm_us = ESC_PWM_MAX_PULSE_US;
    config.reverse_pwm_us = ESC_PWM_MIN_PULSE_US;
    config.brake_min_us = (g_auto_brake_min_us <= UINT16_MAX) ?
        (uint16_t)g_auto_brake_min_us : 0U;
    /* Electrical calibration cannot change an active continuous session. */
    if (s_mode2_drive_config_initialized != 0U && s_mode2_drive_gate.config_valid != 0U &&
        (s_auto_authorized_previous != 0U || s_esc_stop_confirmed == 0U))
    {
        config.center_pwm_us = s_mode2_drive_config.center_pwm_us;
        config.brake_min_us = s_mode2_drive_config.brake_min_us;
    }
    config.action_ack_ms = g_auto_action_ack_ms;
    config.qualify_ms = g_auto_qualify_ms;
    config.neutral_dwell_ms = g_auto_neutral_dwell_ms;
    config.feedback_timeout_ms = g_esc_speed_fresh_timeout_ms;
    config.command_timeout_ms = (g_orin_pwm_timeout_ms != 0U) ?
        g_orin_pwm_timeout_ms : APP_ORIN_PWM_TIMEOUT_DEFAULT_MS;
    config.coast_eval_ms = g_auto_coast_eval_ms;
    config.coast_budget_ms = g_auto_coast_budget_ms;
    config.release_delay_ms = g_auto_brake_release_delay_ms;
    config.e_on_abs_mps = g_auto_brake_enter_error_mps;
    config.e_on_ratio = g_auto_brake_enter_ratio;
    config.e_off_abs_mps = g_auto_brake_release_error_mps;
    config.e_off_ratio = g_auto_brake_release_ratio;
    config.coast_progress_mps2 = g_auto_a_progress_mps2;
    config.stopped_speed_threshold_mps = g_esc_motion_stopped_speed_threshold_mps;
    config.stopped_min_samples = (g_esc_motion_stopped_min_samples <= UINT8_MAX) ?
        (uint8_t)g_esc_motion_stopped_min_samples : 0U;
    config.stopped_min_coverage_ms = g_esc_motion_stopped_min_coverage_ms;
    return config;
}

static LongitudinalControllerConfig_t servo_basic_read_longitudinal_config(void)
{
    LongitudinalControllerConfig_t config;
    memset(&config, 0, sizeof(config));
    config.kp_us_per_mps = g_speed_pid_kp;
    config.ki_us_per_mps_s = g_speed_pid_ki;
    config.kd_us_per_mps2 = g_speed_pid_kd;
    config.derivative_tau_s = (float)g_speed_pid_derivative_tau_ms * 0.001f;
    config.antiwindup_tau_s = (float)g_speed_pid_tracking_tau_ms * 0.001f;
    config.min_output_us = (float)ESC_PWM_MIN_PULSE_US - (float)s_active_esc_center;
    config.max_output_us = (float)ESC_PWM_MAX_PULSE_US - (float)s_active_esc_center;
    config.feedback_freshness_ms = g_esc_speed_fresh_timeout_ms;
    return config;
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

static uint8_t mode2_config_equal(const Mode2DriveGateConfig_t *a,
                                  const Mode2DriveGateConfig_t *b)
{
    return (a->center_pwm_us == b->center_pwm_us &&
        a->forward_pwm_us == b->forward_pwm_us && a->reverse_pwm_us == b->reverse_pwm_us &&
        a->brake_min_us == b->brake_min_us && a->action_ack_ms == b->action_ack_ms &&
        a->qualify_ms == b->qualify_ms && a->neutral_dwell_ms == b->neutral_dwell_ms &&
        a->feedback_timeout_ms == b->feedback_timeout_ms &&
        a->command_timeout_ms == b->command_timeout_ms &&
        a->coast_eval_ms == b->coast_eval_ms && a->coast_budget_ms == b->coast_budget_ms &&
        a->release_delay_ms == b->release_delay_ms &&
        a->e_on_abs_mps == b->e_on_abs_mps && a->e_on_ratio == b->e_on_ratio &&
        a->e_off_abs_mps == b->e_off_abs_mps && a->e_off_ratio == b->e_off_ratio &&
        a->coast_progress_mps2 == b->coast_progress_mps2 &&
        a->stopped_speed_threshold_mps == b->stopped_speed_threshold_mps &&
        a->stopped_min_samples == b->stopped_min_samples &&
        a->stopped_min_coverage_ms == b->stopped_min_coverage_ms) ? 1U : 0U;
}

static uint8_t longitudinal_config_equal(const LongitudinalControllerConfig_t *a,
                                         const LongitudinalControllerConfig_t *b)
{
    return (a->kp_us_per_mps == b->kp_us_per_mps &&
        a->ki_us_per_mps_s == b->ki_us_per_mps_s &&
        a->kd_us_per_mps2 == b->kd_us_per_mps2 &&
        a->derivative_tau_s == b->derivative_tau_s &&
        a->antiwindup_tau_s == b->antiwindup_tau_s &&
        a->min_output_us == b->min_output_us && a->max_output_us == b->max_output_us &&
        a->feedback_freshness_ms == b->feedback_freshness_ms) ? 1U : 0U;
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
	s_esc_receive_epoch = 0U;
	s_esc_last_observed_sample_id = 0U;
	s_esc_last_observed_epoch = 0U;
	memset(&s_esc_latest_raw_sample, 0, sizeof(s_esc_latest_raw_sample));
	s_esc_latest_raw_sample_valid = 0U;
	memset(&s_esc_receiver_health, 0, sizeof(s_esc_receiver_health));
	s_vehicle_direction_known = 0U;
	s_vehicle_direction = 0;
    RcDirectionObserver_Init(&s_auto_direction_observer);
    s_auto_direction_result = RcDirectionObserver_GetResult(&s_auto_direction_observer);
    s_auto_delivery_epoch = 0U;
    s_auto_authorized_previous = 0U;
    s_auto_fault_pending = 0U;
	memset(&s_esc_motion_estimate, 0, sizeof(s_esc_motion_estimate));
	memset(&s_mode2_drive_output, 0, sizeof(s_mode2_drive_output));
	s_mode2_drive_output.action = MODE2_DRIVE_ACTION_NEUTRAL;
	memset(&s_longitudinal_output, 0, sizeof(s_longitudinal_output));
}

static void servo_basic_invalidate_auto_history(uint32_t now_ms)
{
    Mode2DriveGate_ResetHistory(&s_mode2_drive_gate);
    RcDirectionObserver_ResetDirection(&s_auto_direction_observer);
    s_auto_direction_result = RcDirectionObserver_GetResult(&s_auto_direction_observer);
    memset(&s_mode2_drive_output, 0, sizeof(s_mode2_drive_output));
    speed_pid_reset_controller();
    if (g_rc_override_active == 0U)
    {
        servo_basic_clear_vehicle_direction();
        EscMotionEstimator_CommitAppliedActionAt(&s_esc_motion_estimator,
            ESC_MOTION_APPLIED_ACTION_EXTERNAL_OVERRIDE, now_ms);
    }
}

static void servo_basic_refresh_esc_configs(uint32_t now_ms)
{
    const EscMotionEstimatorConfig_t motion = servo_basic_read_esc_motion_config();
    Mode2DriveGateConfig_t gate;
    LongitudinalControllerConfig_t pid;
    uint8_t gate_changed = 0U;
    uint8_t reset_requested;
    ServoBasicIrqState_t state;

    if (s_esc_motion_config_initialized == 0U)
    {
        EscMotionEstimator_Init(&s_esc_motion_estimator, &motion);
        s_esc_motion_config = motion;
        s_esc_motion_config_initialized = 1U;
    }
    else if (esc_motion_config_equal(&s_esc_motion_config, &motion) == 0U)
    {
        (void)EscMotionEstimator_SetConfig(&s_esc_motion_estimator, &motion);
        s_esc_motion_config = motion;
        servo_basic_reset_rc_evidence();
        gate_changed = 1U;
    }
    gate = servo_basic_read_mode2_config();
    if (s_mode2_drive_config_initialized == 0U)
    {
        Mode2DriveGate_Init(&s_mode2_drive_gate, &gate);
        s_mode2_drive_config = gate;
        s_mode2_drive_config_initialized = 1U;
    }
    else if (mode2_config_equal(&s_mode2_drive_config, &gate) == 0U)
    {
        if (s_mode2_drive_config.center_pwm_us != gate.center_pwm_us)
            RcDirectionObserver_Init(&s_auto_direction_observer);
        (void)Mode2DriveGate_SetConfig(&s_mode2_drive_gate, &gate);
        s_mode2_drive_config = gate;
        gate_changed = 1U;
    }
    if (s_mode2_drive_gate.config_valid != 0U)
    {
        s_active_esc_center = gate.center_pwm_us;
    }

    state = servo_basic_enter_critical();
    pid = servo_basic_read_longitudinal_config();
    reset_requested = s_pid_reset_requested;
    s_pid_reset_requested = 0U;
    servo_basic_exit_critical(state);
    if (s_longitudinal_config_initialized == 0U)
    {
        LongitudinalController_Init(&s_longitudinal_controller, &pid);
        s_longitudinal_config = pid;
        s_longitudinal_config_initialized = 1U;
    }
    else if (longitudinal_config_equal(&s_longitudinal_config, &pid) == 0U)
    {
        (void)LongitudinalController_SetConfig(&s_longitudinal_controller, &pid);
        s_longitudinal_config = pid;
        s_mode2_drive_gate.release_predict_count = 0U;
        s_mode2_drive_gate.latest_observation.acceleration_valid = 0U;
        /* Re-seed from the next feedback event, not a fabricated direction. */
    }
    if (reset_requested != 0U)
    {
        speed_pid_reset_controller();
    }
    if (gate_changed != 0U)
    {
        servo_basic_invalidate_auto_history(now_ms);
        s_auto_fault_pending = 1U;
    }
}

static void servo_basic_update_cached_estimate(uint32_t now_ms)
{
    s_esc_motion_estimate = EscMotionEstimator_GetEstimate(&s_esc_motion_estimator, now_ms);
    s_esc_sample_stale =
        (s_esc_motion_estimate.reason == ESC_MOTION_REASON_SAMPLE_STALE) ? 1U : 0U;
    s_esc_feedback_available =
        (s_esc_rx_invalidated == 0U && s_esc_motion_estimate.config_valid != 0U &&
         s_esc_motion_estimate.has_sample != 0U && s_esc_motion_estimate.sample_fresh != 0U &&
         s_esc_motion_estimate.rpm_valid != 0U && s_esc_motion_estimate.magnitude_valid != 0U) ? 1U : 0U;
    s_esc_stop_confirmed =
        (s_esc_feedback_available != 0U && s_esc_motion_estimate.stop_valid != 0U &&
         s_esc_motion_estimate.stopped != 0U) ? 1U : 0U;
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
    g_speed_pid_target_mps = 0.0f;
    g_speed_pid_feedback_mps = 0.0f;
    g_speed_pid_final_us = ESC_PWM_NEUTRAL_PULSE_US;
    s_active_esc_center = ESC_PWM_NEUTRAL_PULSE_US;
    memset(&s_pending_command, 0, sizeof(s_pending_command));
    s_consumed_command_sequence = 0U;
    s_pid_reset_requested = 0U;
	s_esc_motion_config_initialized = 0U;
	s_mode2_drive_config_initialized = 0U;
	s_longitudinal_config_initialized = 0U;
	servo_basic_clear_esc_observation_state();
	servo_basic_refresh_esc_configs(HAL_GetTick());
	speed_pid_reset_controller();
	ServoRC_Capture_Init();
	apply_esc_pulse(get_orin_esc_center_pulse());
	apply_servo_pulse(get_orin_servo_center_pulse());
	s_control_snapshot_next_sequence = 0U;
	servo_basic_publish_control_snapshot(HAL_GetTick());
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

static uint8_t servo_basic_mode2_application_config_valid(void)
{
	return (s_mode2_drive_gate.config_valid != 0U &&
		s_longitudinal_controller.config_valid != 0U) ? 1U : 0U;
}

static uint8_t servo_basic_auto_propulsion_authorized(void)
{
    return (g_rc_override_active == 0U && orin_pwm_is_active() != 0U &&
        g_orin_state.software_stop == 0U && g_orin_state.auto_enabled != 0U &&
        s_esc_feedback_available != 0U && servo_basic_mode2_application_config_valid() != 0U &&
        s_auto_fault_pending == 0U) ? 1U : 0U;
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
		LongitudinalController_ClearIntegral(&s_longitudinal_controller);
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
    ServoBasicIrqState_t state = servo_basic_enter_critical();
    s_pending_command.speed_mps = speed_mps;
    s_pending_command.steering_angle_rad = steering_angle_rad;
    s_pending_command.enable = enable;
    s_pending_command.brake = brake;
    s_pending_command.software_stop = emergency_stop;
    if (enable == 0U || emergency_stop != 0U)
        s_pending_command.authority_cancel_seen = 1U;
    s_pending_command.received_ms = HAL_GetTick();
    s_pending_command.sequence++;
    if (s_pending_command.sequence == 0U) s_pending_command.sequence = 1U;
    servo_basic_exit_critical(state);
}

static void servo_basic_consume_command(void)
{
    ServoBasicCommand_t command;
    ServoBasicIrqState_t state = servo_basic_enter_critical();
    command = s_pending_command;
    s_pending_command.authority_cancel_seen = 0U;
    servo_basic_exit_critical(state);
    if (command.sequence == s_consumed_command_sequence) return;
    s_consumed_command_sequence = command.sequence;
    if (command.authority_cancel_seen != 0U)
    {
        servo_basic_invalidate_auto_history(HAL_GetTick());
        s_auto_fault_pending = 1U;
    }
    if (isfinite(command.speed_mps) == 0 || isfinite(command.steering_angle_rad) == 0)
    {
        command.enable = 0U;
        command.software_stop = 1U;
        command.speed_mps = 0.0f;
        command.steering_angle_rad = 0.0f;
    }
    update_ackermann_from_orin(command.speed_mps, command.steering_angle_rad,
        command.enable, command.brake, command.software_stop);
    g_orin_state.last_update_ms = command.received_ms;
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
	float uplink_speed_magnitude_mps = 0.0f;
	const uint8_t uplink_speed_valid =
		servo_basic_compute_uplink_speed_magnitude(now_ms,
			&uplink_speed_magnitude_mps);

	memset(&diagnostics, 0, sizeof(diagnostics));
	diagnostics.speed_saturated = (g_speed_pid_saturated != 0U) ? 1U : 0U;
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
        (diagnostics.auto_propulsion_authorized != 0U && s_mode2_drive_output.pid_active != 0U) ? 1U : 0U;
    diagnostics.tracking_brake_active =
        (g_rc_override_active == 0U &&
         s_mode2_drive_gate.phase == MODE2_DRIVE_PHASE_F_BRAKE_TRACK) ? 1U : 0U;
    diagnostics.mode2_opposite_armed =
        (s_mode2_drive_gate.permission == MODE2_DRIVE_PERMISSION_R_READY) ? 1U : 0U;
    diagnostics.mode2_state_ambiguous = s_mode2_drive_output.incomplete;
    diagnostics.mode2_control_inhibited =
        (g_rc_override_active == 0U && orin_pwm_is_active_at(now_ms) != 0U &&
         g_orin_state.auto_enabled != 0U &&
         (diagnostics.auto_propulsion_authorized == 0U || s_mode2_drive_output.inhibited != 0U)) ? 1U : 0U;
    diagnostics.mode2_state = (uint8_t)s_mode2_drive_gate.phase;
    diagnostics.mode2_reason = (uint8_t)s_mode2_drive_output.reason;
    diagnostics.auto_brake_purpose = (uint8_t)s_mode2_drive_gate.brake_purpose;
    diagnostics.longitudinal_reason = (uint8_t)s_longitudinal_output.reason;
    diagnostics.longitudinal_target_mps = s_longitudinal_output.diagnostics.requested_target_mps;
    diagnostics.pid_raw_output_us = s_longitudinal_output.raw_output_us;
    diagnostics.pid_integral_us = s_longitudinal_controller.integral_us;
    diagnostics.acceleration_mps2 = s_longitudinal_controller.measured_accel_mps2;
    diagnostics.auto_permission = (uint8_t)s_mode2_drive_gate.permission;
    diagnostics.auto_session_id = s_mode2_drive_gate.current_session_id;
    diagnostics.auto_brake_confirmed = s_mode2_drive_gate.brake_confirmed;
    diagnostics.auto_neutral_confirmed = s_mode2_drive_gate.neutral_confirmed;
    diagnostics.auto_coast_elapsed_ms = (s_mode2_drive_gate.coast_active != 0U) ?
        now_ms - s_mode2_drive_gate.coast_start_ms : 0U;
    if (s_esc_feedback_available != 0U && direction > 0 &&
        s_longitudinal_controller.derivative_valid != 0U)
    {
        const float horizon_s = ((float)(now_ms - s_esc_motion_estimate.last_sample_tick_ms) +
            20.0f + (float)s_mode2_drive_config.release_delay_ms) * 0.001f;
        diagnostics.predicted_release_speed_mps = fmaxf(0.0f,
            s_esc_motion_estimate.speed_magnitude_mps + diagnostics.acceleration_mps2 * horizon_s);
    }
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
if (s_esc_latest_raw_sample_valid != 0U &&
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

static void servo_basic_feed_auto_pid(void)
{
    LongitudinalFeedbackSample_t sample;
    const int8_t direction = (s_auto_direction_result.direction_known != 0U) ?
        (int8_t)s_auto_direction_result.direction : 0;
    memset(&sample, 0, sizeof(sample));
    sample.sample_id = s_esc_latest_raw_sample.sample_id;
    sample.tick_ms = s_esc_latest_raw_sample.received_tick_ms;
    sample.direction = (LongitudinalDirection_t)direction;
    sample.valid = (s_esc_feedback_available != 0U &&
        (direction != 0 ||
         s_esc_latest_raw_sample.rpm_raw == 0U ||
         s_esc_stop_confirmed != 0U)) ? 1U : 0U;
    if (sample.valid != 0U)
    {
        sample.signed_speed_mps = (s_esc_stop_confirmed != 0U) ? 0.0f :
            ((direction < 0) ?
             -s_esc_motion_estimate.speed_magnitude_mps :
              s_esc_motion_estimate.speed_magnitude_mps);
    }
    (void)LongitudinalController_ObserveFeedback(&s_longitudinal_controller, &sample);
}

static void servo_basic_observe_auto_sample(const EscTelemetryObservedSample_t *item,
                                            uint32_t now_ms)
{
    RcDirectionObserverInput_t direction_input;
    Mode2DriveMotionObservation_t observation;
    const EscTelemetryOutputPurpose_t purpose =
        EscTelemetry_MetadataPurpose(item->output_metadata);
    const uint32_t session = EscTelemetry_MetadataSession(item->output_metadata);
    const uint16_t pwm = EscTelemetry_ContextPwm(item->output_context);
    const uint8_t matching_output = (session == s_mode2_drive_gate.current_session_id &&
        purpose == s_mode2_drive_gate.last_purpose) ? 1U : 0U;

    memset(&direction_input, 0, sizeof(direction_input));
    direction_input.rc_active = 1U; /* Separate instance; this flag means source present. */
    direction_input.telemetry_fresh = s_esc_feedback_available;
    direction_input.sample_id = item->sample.sample_id;
    direction_input.state_raw = item->sample.state_candidate_valid != 0U ?
        item->sample.state_raw : 0xFFU;
    direction_input.rpm_valid = item->sample.rpm_valid;
    direction_input.rpm_raw = item->sample.rpm_raw;
    direction_input.moving_evidence = s_esc_motion_estimate.moving_observed;
    direction_input.applied_pwm_us = pwm;
    if (direction_input.state_raw == 0U && purpose != ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL)
    {
        /* A weak reverse request reporting NEUTRAL is not center calibration. */
        direction_input.applied_pwm_us = 0U;
    }
    if (direction_input.state_raw == 1U &&
        (matching_output == 0U ||
         (purpose != ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_REQUEST &&
          purpose != ESC_TELEMETRY_OUTPUT_PURPOSE_REVERSE_REQUEST)))
    {
        /* Preserve existing motion; a brake/neutral request cannot prove new direction. */
        direction_input.applied_pwm_us = s_auto_direction_result.neutral_pwm_us;
    }
    s_auto_direction_result = RcDirectionObserver_Update(&s_auto_direction_observer,
        &direction_input);
    s_vehicle_direction_known = s_auto_direction_result.direction_known;
    s_vehicle_direction = (s_vehicle_direction_known != 0U) ?
        (int8_t)s_auto_direction_result.direction : 0;

    servo_basic_feed_auto_pid();
    memset(&observation, 0, sizeof(observation));
    observation.available = s_esc_feedback_available;
    observation.speed_magnitude_valid = s_esc_motion_estimate.magnitude_valid;
    observation.speed_magnitude_mps = s_esc_motion_estimate.speed_magnitude_mps;
    observation.direction_known = s_vehicle_direction_known;
    observation.direction = (Mode2DriveTargetDirection_t)s_vehicle_direction;
    observation.signed_speed_mps = (s_vehicle_direction < 0) ?
        -observation.speed_magnitude_mps : observation.speed_magnitude_mps;
    observation.sample_id = item->sample.sample_id;
    observation.sample_tick_ms = item->sample.received_tick_ms;
    observation.esc_action = servo_basic_mode2_esc_action(now_ms);
    observation.context_valid = EscTelemetry_MetadataAutoContext(item->output_metadata);
    observation.context_pwm_us = pwm;
    observation.context_purpose = purpose;
    observation.context_session_id = session;
    observation.acceleration_valid = s_longitudinal_controller.derivative_valid;
    observation.acceleration_mps2 = s_longitudinal_controller.measured_accel_mps2;
    Mode2DriveGate_Observe(&s_mode2_drive_gate, &observation, now_ms);
    if (Mode2DriveGate_HasActionConflict(&s_mode2_drive_gate) != 0U)
    {
        /* Keep the gate's reason and stop request; discard only unsupported sign. */
        RcDirectionObserver_ResetDirection(&s_auto_direction_observer);
        s_auto_direction_result = RcDirectionObserver_GetResult(&s_auto_direction_observer);
        servo_basic_clear_vehicle_direction();
        speed_pid_reset_controller();
    }
}

static void servo_basic_update_esc_feedback(uint32_t now_ms)
{
    EscTelemetrySnapshot_t latest;
    EscTelemetryObservedSample_t item;
    size_t remaining = EscTelemetry_PendingSamples();
    uint32_t epoch = EscTelemetry_GetDeliveryEpoch();
    uint32_t processed = 0U;
    uint32_t start_cycles = servo_basic_observation_cycles();
    uint32_t cycles;

    EscTelemetry_GetReceiverHealth(&s_esc_receiver_health);
    if (epoch != s_auto_delivery_epoch)
    {
        s_observation_diagnostics.delivery_gaps++;
        /* A delivery boundary drops event evidence, but a still-fresh latest
         * FE32 sample remains usable for the running PID/output session. */
        s_auto_delivery_epoch = epoch;
    }
    if (EscTelemetry_GetSnapshot(&latest) == 0U)
    {
        s_esc_rx_invalidated = 1U;
        s_esc_latest_raw_sample_valid = 0U;
        servo_basic_invalidate_auto_history(now_ms);
        s_auto_fault_pending = 1U;
        servo_basic_update_cached_estimate(now_ms);
        return;
    }
    s_esc_rx_invalidated = 0U;
    /* A receive epoch marks a real receiver/parser boundary. A delivery
     * boundary alone may be tolerated, but samples from a new receive epoch
     * must not extend the old AUTO direction/startup session. */
    if (s_esc_receive_epoch != 0U &&
        latest.receive_epoch != s_esc_receive_epoch)
    {
        servo_basic_invalidate_auto_history(now_ms);
        s_auto_fault_pending = 1U;
    }
    s_esc_receive_epoch = latest.receive_epoch;
    while (remaining-- != 0U && EscTelemetry_PopSample(&item) != 0U)
    {
        uint32_t age_ms;
        int8_t previous_direction = s_vehicle_direction_known != 0U ? s_vehicle_direction : 0;
        if (item.receive_epoch != latest.receive_epoch ||
            EscTelemetry_ContextSource(item.output_context) != EscTelemetry_ContextSource(s_rc_output_context) ||
            EscTelemetry_ContextRcActive(item.output_context) != 0U ||
            EscTelemetry_MetadataAutoContext(item.output_metadata) == 0U)
        {
            s_observation_diagnostics.samples_wrong_source++;
            continue;
        }
        if (s_auto_delivery_epoch != item.delivery_epoch)
        {
            s_observation_diagnostics.delivery_gaps++;
            s_auto_delivery_epoch = item.delivery_epoch;
        }
        if (servo_basic_tick_delta_ms(now_ms, item.sample.received_tick_ms, &age_ms) == 0U ||
            s_esc_motion_config.telemetry_timeout_ms == 0U ||
            age_ms > s_esc_motion_config.telemetry_timeout_ms)
        {
            s_observation_diagnostics.samples_expired++;
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
        if (s_esc_feedback_available == 0U)
        {
            /* The estimator will revoke freshness at the configured timeout;
             * one malformed event must not cancel an otherwise fresh session. */
        }
        else
        {
            servo_basic_observe_auto_sample(&item, now_ms);
        }
        if (s_vehicle_direction_known != 0U && s_vehicle_direction != previous_direction &&
            s_esc_stop_confirmed == 0U)
        {
            HallSpeed_SetCommandDirection(s_vehicle_direction);
        }
        s_observation_diagnostics.samples_processed++;
        s_observation_diagnostics.last_sample_id = item.sample.sample_id;
        if (age_ms > s_observation_diagnostics.max_sample_age_ms)
            s_observation_diagnostics.max_sample_age_ms = age_ms;
        processed++;
    }
    if ((latest.sample.sample_id != s_esc_last_observed_sample_id ||
         latest.receive_epoch != s_esc_last_observed_epoch) && EscTelemetry_PendingSamples() == 0U)
    {
        uint32_t age_ms;
        if (servo_basic_tick_delta_ms(now_ms, latest.sample.received_tick_ms, &age_ms) != 0U &&
            s_esc_motion_config.telemetry_timeout_ms != 0U &&
            age_ms <= s_esc_motion_config.telemetry_timeout_ms)
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
    if (servo_basic_esc_fe32_is_fresh(now_ms) == 0U)
    {
        servo_basic_invalidate_auto_history(now_ms);
        s_auto_fault_pending = 1U;
    }
    epoch = EscTelemetry_GetDeliveryEpoch();
    if (epoch != s_auto_delivery_epoch)
    {
        s_observation_diagnostics.delivery_gaps++;
        s_auto_delivery_epoch = epoch;
    }
    servo_basic_update_cached_estimate(now_ms);
    if (processed > s_observation_diagnostics.max_batch_samples)
        s_observation_diagnostics.max_batch_samples = processed;
    cycles = servo_basic_observation_cycles() - start_cycles;
    if (cycles > s_observation_diagnostics.max_batch_cycles)
        s_observation_diagnostics.max_batch_cycles = cycles;
}

void ServoBasic_ProcessEscObservation(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (g_rc_override_active != 0U)
        servo_basic_update_rc_feedback(now_ms);
    else
        servo_basic_update_esc_feedback(now_ms);
    HallSpeed_SetCommandDirection(s_esc_stop_confirmed == 0U ?
        servo_basic_estimated_vehicle_direction() : 0);
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
    const uint32_t now_ms = HAL_GetTick();
    uint8_t orin_active;
    uint8_t command_authorized;
    LongitudinalControllerInput_t pid_input;
    LongitudinalAppliedOutput_t applied;
    Mode2DriveGateInput_t gate_input;

    servo_basic_consume_command();
    servo_basic_refresh_esc_configs(now_ms);
    if (g_rc_override_active != 0U)
        servo_basic_update_rc_feedback(now_ms);
    else
        servo_basic_update_esc_feedback(now_ms);
    refresh_rc_inputs();
    update_control_mode_from_rc();
    orin_active = orin_pwm_is_active_at(now_ms);
    command_authorized = (g_rc_override_active == 0U && orin_active != 0U &&
        g_orin_state.auto_enabled != 0U && g_orin_state.software_stop == 0U) ? 1U : 0U;

    if (s_auto_authorized_previous != command_authorized)
    {
        servo_basic_invalidate_auto_history(now_ms);
        s_auto_authorized_previous = command_authorized;
    }
    if (g_rc_override_active != 0U)
    {
        speed_pid_reset_controller();
        apply_rc_passthrough_outputs();
    }
    else
    {
        /* A PID-only gain update can re-seed from this same real measurement. */
        if (s_longitudinal_controller.feedback_valid == 0U &&
            s_esc_latest_raw_sample_valid != 0U && s_esc_feedback_available != 0U)
            servo_basic_feed_auto_pid();

        memset(&pid_input, 0, sizeof(pid_input));
        pid_input.enabled = servo_basic_auto_propulsion_authorized();
        pid_input.target_speed_mps = g_orin_state.brake_active != 0U ?
            0.0f : g_orin_state.target_speed_mps;
        pid_input.now_tick_ms = now_ms;
        s_longitudinal_output = LongitudinalController_Evaluate(&s_longitudinal_controller, &pid_input);

        memset(&gate_input, 0, sizeof(gate_input));
        gate_input.command_valid = orin_active;
        gate_input.command_tick_ms = g_orin_state.last_update_ms;
        gate_input.propulsion_authorized = servo_basic_auto_propulsion_authorized();
        gate_input.target_speed_mps = pid_input.target_speed_mps;
        gate_input.pid_valid = s_longitudinal_output.valid;
        gate_input.pid_raw_us = s_longitudinal_output.raw_output_us;
        gate_input.pid_integral_us = s_longitudinal_output.diagnostics.integral_us;
        s_mode2_drive_output = Mode2DriveGate_EvaluateControl(&s_mode2_drive_gate,
            &gate_input, now_ms);
        if (Mode2DriveGate_HasActionConflict(&s_mode2_drive_gate) != 0U)
        {
            RcDirectionObserver_ResetDirection(&s_auto_direction_observer);
            s_auto_direction_result = RcDirectionObserver_GetResult(&s_auto_direction_observer);
            servo_basic_clear_vehicle_direction();
        }

        apply_esc_pulse(clamp_esc_pulse(s_mode2_drive_output.final_pwm_us));
        apply_servo_pulse((orin_active != 0U && g_orin_state.software_stop == 0U) ?
            limit_servo_safe_pulse(clamp_servo_pulse(g_orin_state.servo_pulse_us)) :
            get_orin_servo_center_pulse());

        memset(&applied, 0, sizeof(applied));
        applied.applied_output_us = (float)g_state.esc_pulse_us - (float)get_orin_esc_center_pulse();
        applied.min_output_us = s_mode2_drive_output.pid_min_us;
        applied.max_output_us = s_mode2_drive_output.pid_max_us;
        applied.pid_active = s_mode2_drive_output.pid_active;
        applied.reset_integral = s_mode2_drive_output.integral_reset;
        (void)LongitudinalController_CommitApplied(&s_longitudinal_controller, &applied);

        g_speed_pid_target_mps = pid_input.target_speed_mps;
        g_speed_pid_feedback_mps = s_longitudinal_output.diagnostics.feedback_signed_mps;
        g_speed_pid_feedback_valid = s_longitudinal_output.diagnostics.feedback_valid;
        g_speed_pid_error_mps = s_longitudinal_output.diagnostics.speed_error_mps;
        g_speed_pid_integral_us = s_longitudinal_controller.integral_us;
        g_speed_pid_raw_output_us = s_longitudinal_output.raw_output_us;
        g_speed_pid_final_us = g_state.esc_pulse_us;
        g_speed_pid_saturated = (s_mode2_drive_output.pid_active != 0U &&
            (s_longitudinal_output.raw_output_us < applied.min_output_us ||
             s_longitudinal_output.raw_output_us > applied.max_output_us)) ? 1U : 0U;
    }
    s_auto_fault_pending = 0U; /* The invalidation has now reached a physical neutral output. */
    HallSpeed_SetCommandDirection(s_esc_stop_confirmed == 0U ?
        servo_basic_estimated_vehicle_direction() : 0);
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
