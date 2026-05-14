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
#include "hall_speed.h"
#include "servo_rc_capture.h"

#ifndef SERVO_BASIC_LOG
#define SERVO_BASIC_LOG(...) ((void)0)
#endif

static servo_basic_state_t g_state = {
	ESC_PWM_NEUTRAL_PULSE_US,
	ESC_PWM_NEUTRAL_PULSE_US,
	SERVO_CTRL_MODE_AUTONOMOUS,
	0U,
	0U
};

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
#define ORIN_MIN_FORWARD_VX_DEFAULT_MMPS         APP_ORIN_MIN_FORWARD_VX_MMPS
#define ORIN_MIN_REVERSE_VX_DEFAULT_MMPS         APP_ORIN_MIN_REVERSE_VX_MMPS
#define HALL_SPEED_LIMIT_DEFAULT_MMPS            APP_HALL_SPEED_LIMIT_MMPS
#define HALL_SPEED_LIMIT_RELEASE_DEFAULT_MMPS    APP_HALL_SPEED_LIMIT_RELEASE_MMPS
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
volatile uint32_t g_orin_servo_range_us = APP_ORIN_SERVO_RANGE_US;
volatile uint32_t g_hall_speed_limit_mmps = HALL_SPEED_LIMIT_DEFAULT_MMPS;
volatile uint32_t g_hall_speed_limit_release_mmps = HALL_SPEED_LIMIT_RELEASE_DEFAULT_MMPS;
volatile uint32_t g_hall_speed_limit_active = 0U;
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
	ESC_PWM_NEUTRAL_PULSE_US,
	ESC_PWM_NEUTRAL_PULSE_US,
	0U,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0.0f,
	0U,
	0U,
	0U,
	0U,
	0U
};
static volatile uint8_t g_rc_override_active = 0U;
static volatile uint8_t g_rc_guard_active = 0U;
static uint8_t g_rc_override_enter_count = 0U;
static uint32_t g_rc_override_release_start_ms = 0U;
static uint16_t g_rc_throttle_current = 0U;
static uint16_t g_rc_steering_current = 0U;
static uint16_t g_rc_guard_current = 0U;
static uint8_t g_rc_throttle_present = 0U;
static uint8_t g_rc_steering_present = 0U;
static uint8_t g_rc_guard_present = 0U;
static rc_channel_filter_state_t g_rc_throttle_state = {0U};
static rc_channel_filter_state_t g_rc_steering_state = {0U};
static uint8_t g_autonomous_target_valid = 0U;
static uint32_t g_speed_pi_last_update_ms = 0U;
static int8_t g_speed_pi_last_target_direction = 0;

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
static void apply_orin_esc_pulse(uint16_t pulse_us);
static void speed_pi_reset_controller(void);
static uint32_t get_rc_override_center_us(void);
static uint8_t pulse_is_inside_center(uint16_t pulse_us, uint32_t center_us, uint32_t threshold_us);
static uint8_t orin_pwm_is_active(void);
static void servo_basic_apply_debug_command(uint32_t cmd, uint32_t value);

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
	range = (g_orin_servo_range_us == 0U) ? 500U : g_orin_servo_range_us;
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

static uint32_t get_hall_speed_limit_mmps(void)
{
	return (g_hall_speed_limit_mmps == 0U) ?
		HALL_SPEED_LIMIT_DEFAULT_MMPS : g_hall_speed_limit_mmps;
}

static uint32_t get_hall_speed_limit_release_mmps(void)
{
	uint32_t limit_mmps = get_hall_speed_limit_mmps();
	uint32_t release_mmps = (g_hall_speed_limit_release_mmps == 0U) ?
		HALL_SPEED_LIMIT_RELEASE_DEFAULT_MMPS : g_hall_speed_limit_release_mmps;

	return (release_mmps > limit_mmps) ? limit_mmps : release_mmps;
}

static uint8_t hall_speed_limit_should_block(void)
{
	float feedback_vx_mps = 0.0f;
	const float limit_mps = (float)get_hall_speed_limit_mmps() / 1000.0f;
	const float release_mps = (float)get_hall_speed_limit_release_mmps() / 1000.0f;
	float speed_abs_mps;

	if (HallSpeed_GetSignedSpeedMps(&feedback_vx_mps) == 0U)
	{
		g_hall_speed_limit_active = 0U;
		return 0U;
	}

	speed_abs_mps = fabsf(feedback_vx_mps);
	if (g_hall_speed_limit_active != 0U)
	{
		if (speed_abs_mps <= release_mps)
		{
			g_hall_speed_limit_active = 0U;
		}
	}
	else if (speed_abs_mps >= limit_mps)
	{
		g_hall_speed_limit_active = 1U;
	}

	return (g_hall_speed_limit_active != 0U) ? 1U : 0U;
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

	g_rc_guard_present = ServoRC_IsGuardActive(timeout_ms);

	update_rc_channel_state(&g_rc_throttle_state, raw_throttle, raw_throttle_present, throttle_fault, now_ms, 1U);
	update_rc_channel_state(&g_rc_steering_state, raw_steering, raw_steering_present, steering_fault, now_ms, 0U);

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
	g_rc_input_fault_active = (g_rc_throttle_state.invalid_since_ms != 0U ||
		g_rc_steering_state.invalid_since_ms != 0U ||
		throttle_fault != 0U ||
		steering_fault != 0U ||
		guard_fault != 0U) ? 1U : 0U;

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

static void set_rc_override_state(uint8_t override_active, uint8_t guard_active)
{
	g_rc_override_active = (override_active != 0U) ? 1U : 0U;
	g_rc_guard_active = (guard_active != 0U) ? 1U : 0U;
	g_state.control_mode = (g_rc_override_active != 0U) ? SERVO_CTRL_MODE_RC_PASSTHROUGH :
		SERVO_CTRL_MODE_AUTONOMOUS;
	g_state.rc_takeover_pending = 0U;
	g_state.emergency_stop = g_rc_guard_active;
	g_rc_override_release_start_ms = 0U;
	g_rc_override_enter_count = 0U;
	if (g_rc_override_active != 0U)
	{
		HallSpeed_SetCommandDirection(0);
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
	g_speed_pi_last_target_direction = 0;
	g_speed_pi_feedback_valid = 0U;
	g_speed_pi_saturated = 0U;
	g_speed_pi_error_mps = 0.0f;
	g_speed_pi_integral = 0.0f;
	g_speed_pi_trim_us = 0;
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

static void apply_orin_esc_pulse(uint16_t pulse_us)
{
	if (pulse_us != 0U && hall_speed_limit_should_block() != 0U)
	{
		pulse_us = get_orin_esc_center_pulse();
		speed_pi_reset_controller();
	}
	apply_esc_pulse(pulse_us);
}

static void apply_servo_pulse(uint16_t pulse_us)
{
	g_state.servo_pulse_us = pulse_us;
	ServoBasic_OutputServoPulse(pulse_us);
}

static void set_esc_target(uint16_t pulse_us)
{
	g_state.esc_pulse_us = pulse_us;
	g_autonomous_target_valid = 1U;
}

static void set_servo_target(uint16_t pulse_us)
{
	g_state.servo_pulse_us = pulse_us;
	g_autonomous_target_valid = 1U;
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
	g_rc_throttle_present = 0U;
	g_rc_steering_present = 0U;
	g_rc_guard_present = 0U;
	g_autonomous_target_valid = 0U;
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
	g_speed_pi_target_vx_mps = 0.0f;
	g_speed_pi_feedback_vx_mps = 0.0f;
	g_speed_pi_base_us = ESC_PWM_NEUTRAL_PULSE_US;
	g_speed_pi_final_us = ESC_PWM_NEUTRAL_PULSE_US;
	speed_pi_reset_controller();
	ServoRC_Capture_Init();
	apply_esc_pulse(get_orin_esc_center_pulse());
	apply_servo_pulse(get_orin_servo_center_pulse());
}

static uint16_t orin_map_vx_to_esc(float vx_mps)
{
	float scaled_vx_mps = scale_and_limit_orin_vx(vx_mps);
	uint32_t speed_mmps;

	if (orin_target_is_below_min_control(scaled_vx_mps) != 0U)
	{
		return get_orin_esc_center_pulse();
	}

	speed_mmps = speed_mps_abs_to_mmps(scaled_vx_mps);
	if (scaled_vx_mps > 0.0f)
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

static uint16_t orin_compute_speed_control_esc(void)
{
	const float target_vx_mps = scale_and_limit_orin_vx(g_orin_state.target_speed_mps);
	const uint16_t base_us = orin_map_vx_to_esc(target_vx_mps);
	const int8_t target_direction = get_vx_direction(target_vx_mps);
	const uint32_t now_ms = HAL_GetTick();
	float feedback_vx_mps = 0.0f;
	uint8_t feedback_valid = 0U;

	g_speed_pi_target_vx_mps = target_vx_mps;
	g_speed_pi_base_us = base_us;

	if (orin_target_is_below_min_control(target_vx_mps) != 0U)
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

	feedback_valid = HallSpeed_GetSignedSpeedMps(&feedback_vx_mps);
	g_speed_pi_feedback_valid = feedback_valid;
	g_speed_pi_feedback_vx_mps = feedback_vx_mps;

	if (g_speed_pi_enable == 0U || feedback_valid == 0U)
	{
		g_speed_pi_error_mps = 0.0f;
		g_speed_pi_trim_us = 0;
		g_speed_pi_final_us = base_us;
		if (feedback_valid == 0U)
		{
			g_speed_pi_last_update_ms = 0U;
			g_speed_pi_integral = 0.0f;
		}
		return base_us;
	}

	{
		float dt_s = 0.02f;
		const float kp = g_speed_pi_kp;
		const float ki = g_speed_pi_ki;
		const float trim_limit = (float)get_speed_pi_trim_limit_us();
		float error_mps;
		float integral;
		float trim_us;
		int32_t trim_i32;
		int32_t final_i32;

		if (g_speed_pi_last_update_ms != 0U)
		{
			dt_s = (float)(now_ms - g_speed_pi_last_update_ms) / 1000.0f;
			if (dt_s < 0.001f)
			{
				dt_s = 0.001f;
			}
			else if (dt_s > 0.100f)
			{
				dt_s = 0.100f;
			}
		}
		g_speed_pi_last_update_ms = now_ms;

		error_mps = target_vx_mps - feedback_vx_mps;
		integral = g_speed_pi_integral + error_mps * dt_s;
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
		if (final_i32 < (int32_t)ESC_PWM_MIN_PULSE_US || final_i32 > (int32_t)ESC_PWM_MAX_PULSE_US)
		{
			g_speed_pi_saturated = 1U;
		}

		g_speed_pi_error_mps = error_mps;
		g_speed_pi_integral = integral;
		g_speed_pi_trim_us = trim_i32;
		g_speed_pi_final_us = clamp_esc_pulse_i32(final_i32);
		return (uint16_t)g_speed_pi_final_us;
	}
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
	ratio = steering_angle_rad / max_steering_rad;
	return limit_servo_safe_pulse((uint16_t)((int32_t)get_orin_servo_center_pulse() +
		(int32_t)(ratio * (float)((g_orin_servo_range_us == 0U) ? 500U : g_orin_servo_range_us))));
}

static float telemetry_estimate_steering_rad_from_servo_pulse(uint16_t pulse_us)
{
	const uint16_t center_pulse = get_orin_servo_center_pulse();
	const uint16_t pulse_range = (g_orin_servo_range_us == 0U) ? 500U : (uint16_t)g_orin_servo_range_us;
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
	return ratio * max_steering_rad;
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
	float scaled_speed_mps;
	float feedback_speed_mps;
	int8_t command_direction = 0;
	uint8_t speed_below_min;
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

	steering_angle_rad = clamp_orin_steering_angle(steering_angle_rad);
	scaled_speed_mps = (force_zero_speed != 0U) ? 0.0f : scale_and_limit_orin_vx(speed_mps);
	speed_below_min = orin_target_is_below_min_control(scaled_speed_mps);
	feedback_speed_mps = (speed_below_min != 0U ||
		fabsf(scaled_speed_mps) < get_orin_velocity_neutral_threshold_mps()) ? 0.0f : scaled_speed_mps;

	if (force_zero_speed == 0U && speed_below_min == 0U)
	{
		command_direction = get_vx_direction(scaled_speed_mps);
	}
	HallSpeed_SetCommandDirection(command_direction);

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
			g_orin_state.esc_pulse_us = orin_map_vx_to_esc(speed_mps);
		}
		g_orin_state.servo_pulse_us = orin_map_steering_to_servo(steering_angle_rad);
	}

	g_orin_state.target_speed_mps = (force_zero_speed != 0U) ? 0.0f : speed_mps;
	g_orin_state.feedback_vx_mps = feedback_speed_mps;
	g_orin_state.feedback_steering_angle_rad =
		telemetry_estimate_steering_rad_from_servo_pulse(g_orin_state.servo_pulse_us);
	g_orin_state.feedback_vz_rad_s = telemetry_estimate_vz_from_pwm(
		feedback_speed_mps,
		g_orin_state.servo_pulse_us);
	g_orin_state.target_steering_angle_rad = steering_angle_rad;
	g_orin_state.last_update_ms = HAL_GetTick();
	g_orin_state.active = 1U;
	g_orin_state.stop = estop_active;
	g_orin_state.auto_enabled = auto_enabled;
	g_orin_state.brake_active = brake_active;
	g_orin_state.emergency_stop = estop_active;
}

void ServoBasic_UpdateAckermannFromOrin(float speed_mps,
										float steering_angle_rad,
										uint8_t enable,
										uint8_t brake,
										uint8_t emergency_stop)
{
	update_ackermann_from_orin(speed_mps, steering_angle_rad, enable, brake, emergency_stop);
}

static uint8_t orin_pwm_is_active(void)
{
	if (g_orin_pwm_enable == 0U || g_orin_state.active == 0U)
	{
		return 0U;
	}
	const uint32_t timeout_ms = (g_orin_pwm_timeout_ms == 0U) ? APP_ORIN_PWM_TIMEOUT_DEFAULT_MS :
		g_orin_pwm_timeout_ms;
	return (HAL_GetTick() - g_orin_state.last_update_ms) <= timeout_ms;
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
			set_rc_override_state(1U, 1U);
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
	if (g_rc_override_active != 0U && g_rc_guard_active == 0U && centered != 0U)
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
				set_rc_override_state(1U, 0U);
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
		set_rc_override_state(1U, 0U);
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

static void apply_autonomous_outputs(void)
{
	apply_orin_esc_pulse(limit_esc_safe_pulse(clamp_esc_pulse(g_state.esc_pulse_us)));
	apply_servo_pulse(limit_servo_safe_pulse(clamp_servo_pulse(g_state.servo_pulse_us)));
}

void ServoBasic_ProcessControl(void)
{
	uint8_t orin_active;

	refresh_rc_inputs();
	update_control_mode_from_rc();
	orin_active = orin_pwm_is_active();
	if (g_rc_override_active != 0U || orin_active == 0U)
	{
		HallSpeed_SetCommandDirection(0);
		speed_pi_reset_controller();
	}

	if (g_state.emergency_stop != 0U)
	{
		speed_pi_reset_controller();
		apply_esc_pulse(ESC_PWM_MIN_PULSE_US);
		apply_servo_pulse(get_orin_servo_center_pulse());
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
			speed_pi_reset_controller();
			apply_esc_pulse(0U);
			apply_servo_pulse(0U);
		}
		else
		{
			apply_orin_esc_pulse(orin_compute_speed_control_esc());
			apply_servo_pulse(limit_servo_safe_pulse(clamp_servo_pulse(g_orin_state.servo_pulse_us)));
		}
	}
	else
	{
		if (g_autonomous_target_valid != 0U)
		{
			apply_autonomous_outputs();
		}
		else
		{
			apply_esc_pulse(get_orin_esc_center_pulse());
			apply_servo_pulse(get_orin_servo_center_pulse());
		}
	}
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

const servo_basic_state_t *ServoBasic_GetState(void)
{
	return &g_state;
}

uint8_t ServoBasic_IsRcOverrideActive(void)
{
	return g_rc_override_active;
}

uint8_t ServoBasic_IsRcEmergencyActive(void)
{
	return g_rc_guard_active;
}

uint8_t ServoBasic_IsOrinCommandTimeout(void)
{
	if (g_orin_pwm_enable == 0U)
	{
		return 0U;
	}
	if (g_orin_state.active == 0U)
	{
		return 1U;
	}
	return (orin_pwm_is_active() == 0U) ? 1U : 0U;
}

uint8_t ServoBasic_IsOrinAutoEnabled(void)
{
	if (g_orin_pwm_enable == 0U || orin_pwm_is_active() == 0U)
	{
		return 0U;
	}
	if (g_rc_override_active != 0U || g_orin_state.emergency_stop != 0U)
	{
		return 0U;
	}
	return (g_orin_state.auto_enabled != 0U) ? 1U : 0U;
}

uint8_t ServoBasic_IsOrinBrakeActive(void)
{
	if (orin_pwm_is_active() == 0U)
	{
		return 0U;
	}
	return (g_orin_state.brake_active != 0U) ? 1U : 0U;
}

uint8_t ServoBasic_IsOrinEmergencyActive(void)
{
	if (orin_pwm_is_active() == 0U)
	{
		return 0U;
	}
	return (g_orin_state.emergency_stop != 0U) ? 1U : 0U;
}

uint8_t ServoBasic_GetAckermannFeedback(float *speed_mps,
										float *steering_angle_rad,
										float *yaw_rate_rad_s)
{
	float feedback_speed = 0.0f;
	const uint8_t speed_valid = HallSpeed_GetSignedSpeedMps(&feedback_speed);
	const float steering_rad = telemetry_estimate_steering_rad_from_servo_pulse(g_state.servo_pulse_us);
	const float yaw_rate = telemetry_estimate_vz_from_pwm(feedback_speed, g_state.servo_pulse_us);

	if (speed_valid == 0U)
	{
		feedback_speed = 0.0f;
	}
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
		*yaw_rate_rad_s = yaw_rate;
	}

	return speed_valid;
}
