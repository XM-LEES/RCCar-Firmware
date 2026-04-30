/**
 * @file servo_rc_capture.c
 * @brief RC input capture using TIM4 CH1~CH3 (throttle, steering, guard).
 */

#include "servo_rc_capture.h"

extern volatile uint32_t g_rc_valid_min_us;
extern volatile uint32_t g_rc_valid_max_us;
volatile uint32_t g_rc_capture_active_high = 1U;

typedef struct
{
	uint32_t last_capture;
	uint16_t pulse_ticks;
	uint8_t waiting_for_falling;
	uint32_t last_update_ms;
	uint8_t fault_active;
} servo_rc_channel_state_t;

static servo_rc_channel_state_t g_rc_throttle = {0};
static servo_rc_channel_state_t g_rc_steering = {0};
static servo_rc_channel_state_t g_rc_guard = {0};

static uint32_t rc_capture_first_polarity(void);
static uint32_t rc_capture_second_polarity(void);

void ServoRC_Capture_Init(void)
{
	g_rc_throttle.last_capture = 0U;
	g_rc_throttle.pulse_ticks = 1500U;
	g_rc_throttle.waiting_for_falling = 0U;
	g_rc_throttle.last_update_ms = 0U;
	g_rc_throttle.fault_active = 0U;

	g_rc_steering.last_capture = 0U;
	g_rc_steering.pulse_ticks = 1500U;
	g_rc_steering.waiting_for_falling = 0U;
	g_rc_steering.last_update_ms = 0U;
	g_rc_steering.fault_active = 0U;

	g_rc_guard.last_capture = 0U;
	g_rc_guard.pulse_ticks = 1500U;
	g_rc_guard.waiting_for_falling = 0U;
	g_rc_guard.last_update_ms = 0U;
	g_rc_guard.fault_active = 0U;

	TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1);
	TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, rc_capture_first_polarity());
	TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2);
	TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2, rc_capture_first_polarity());
	TIM_RESET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_3);
	TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_3, rc_capture_first_polarity());
}

uint16_t ServoRC_GetThrottlePulse(void)
{
	return g_rc_throttle.pulse_ticks;
}

uint16_t ServoRC_GetSteeringPulse(void)
{
	return g_rc_steering.pulse_ticks;
}

uint16_t ServoRC_GetGuardPulse(void)
{
	return g_rc_guard.pulse_ticks;
}

static uint32_t rc_capture_diff(uint32_t start, uint32_t end, uint32_t period)
{
	if (end >= start)
	{
		return end - start;
	}
	return (period + 1U - start) + end;
}

static uint32_t get_rc_valid_min_pulse_us(void)
{
	return (g_rc_valid_min_us == 0U) ? 900U : g_rc_valid_min_us;
}

static uint32_t get_rc_valid_max_pulse_us(void)
{
	const uint32_t min_us = get_rc_valid_min_pulse_us();
	if (g_rc_valid_max_us < min_us)
	{
		return 2100U;
	}
	return (g_rc_valid_max_us == 0U) ? 2100U : g_rc_valid_max_us;
}

static uint8_t rc_pulse_is_valid(uint32_t pulse_ticks)
{
	const uint32_t min_us = get_rc_valid_min_pulse_us();
	const uint32_t max_us = get_rc_valid_max_pulse_us();
	return (pulse_ticks >= min_us && pulse_ticks <= max_us) ? 1U : 0U;
}

static uint32_t rc_capture_first_polarity(void)
{
	return (g_rc_capture_active_high != 0U) ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
}

static uint32_t rc_capture_second_polarity(void)
{
	return (g_rc_capture_active_high != 0U) ? TIM_ICPOLARITY_FALLING : TIM_ICPOLARITY_RISING;
}

static void ServoRC_HandleChannel(servo_rc_channel_state_t *state,
								  TIM_HandleTypeDef *htim,
								  uint32_t channel)
{
	const uint32_t captured = HAL_TIM_ReadCapturedValue(htim, channel);
	const uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
	const uint32_t now_ms = HAL_GetTick();

	if (!state->waiting_for_falling)
	{
		state->last_capture = captured;
		state->waiting_for_falling = 1U;
		TIM_RESET_CAPTUREPOLARITY(htim, channel);
		TIM_SET_CAPTUREPOLARITY(htim, channel, rc_capture_second_polarity());
		return;
	}

	{
		const uint32_t pulse_ticks = rc_capture_diff(state->last_capture, captured, period);
		if (pulse_ticks <= 0xFFFFU &&
			rc_pulse_is_valid(pulse_ticks) != 0U)
		{
			state->pulse_ticks = (uint16_t)pulse_ticks;
			state->last_update_ms = now_ms;
			state->fault_active = 0U;
		}
		else
		{
			state->fault_active = 1U;
		}
	}

	state->waiting_for_falling = 0U;
	TIM_RESET_CAPTUREPOLARITY(htim, channel);
	TIM_SET_CAPTUREPOLARITY(htim, channel, rc_capture_first_polarity());
}

static uint8_t ServoRC_IsActive(const servo_rc_channel_state_t *state, uint32_t timeout_ms)
{
	if (state->last_update_ms == 0U)
	{
		return 0U;
	}
	return (HAL_GetTick() - state->last_update_ms) <= timeout_ms;
}

uint8_t ServoRC_IsThrottleActive(uint32_t timeout_ms)
{
	return ServoRC_IsActive(&g_rc_throttle, timeout_ms);
}

uint8_t ServoRC_IsSteeringActive(uint32_t timeout_ms)
{
	return ServoRC_IsActive(&g_rc_steering, timeout_ms);
}

uint8_t ServoRC_IsGuardActive(uint32_t timeout_ms)
{
	return ServoRC_IsActive(&g_rc_guard, timeout_ms);
}

uint8_t ServoRC_HasThrottleFault(void)
{
	return g_rc_throttle.fault_active;
}

uint8_t ServoRC_HasSteeringFault(void)
{
	return g_rc_steering.fault_active;
}

uint8_t ServoRC_HasGuardFault(void)
{
	return g_rc_guard.fault_active;
}

void ServoRC_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance != TIM4)
	{
		return;
	}

	switch (htim->Channel)
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
			ServoRC_HandleChannel(&g_rc_throttle, htim, TIM_CHANNEL_1);
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			ServoRC_HandleChannel(&g_rc_steering, htim, TIM_CHANNEL_2);
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			ServoRC_HandleChannel(&g_rc_guard, htim, TIM_CHANNEL_3);
			break;
		default:
			break;
	}
}
