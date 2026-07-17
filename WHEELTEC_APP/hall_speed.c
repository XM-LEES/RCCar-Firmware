/**
 * @file hall_speed.c
 * @brief Hall speed measurement with configurable count and direction channels.
 */

#include "hall_speed.h"

#include "main.h"
#include "bsp_dwt.h"

#define HALL_WHEEL_DIAMETER_M            0.230f
#define HALL_WHEEL_CIRCUMFERENCE_M       (HALL_WHEEL_DIAMETER_M * 3.14159265358979f)
#define HALL_COUNT_EVENTS_PER_REV        10U
#define HALL_MIN_EVENT_INTERVAL_US       1500U
#define HALL_TIMEOUT_MIN_US           500000U
#define HALL_TIMEOUT_MAX_US          4000000U
#define HALL_COUNT_USE_CHANNEL_B         1U

volatile hall_speed_state_t g_hall_speed_state = {0};

static uint16_t hall_count_pin = HallA_Pin;
static volatile uint32_t g_hall_speed_started_cycles = 0U;

static uint32_t hall_speed_get_cycles(void)
{
	return DWT_CYCCNT;
}

static uint32_t hall_speed_elapsed_us(uint32_t now_cycles, uint32_t start_cycles)
{
	const uint32_t elapsed_cycles = now_cycles - start_cycles;
	const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
	if (cycles_per_us == 0U)
	{
		return 0U;
	}
	return elapsed_cycles / cycles_per_us;
}

static uint32_t hall_speed_get_timeout_us(uint32_t last_period_us)
{
	uint64_t timeout_us = (uint64_t)last_period_us * 2ULL;

	if (timeout_us < HALL_TIMEOUT_MIN_US)
	{
		timeout_us = HALL_TIMEOUT_MIN_US;
	}
	if (timeout_us > HALL_TIMEOUT_MAX_US)
	{
		timeout_us = HALL_TIMEOUT_MAX_US;
	}
	return (uint32_t)timeout_us;
}

static int8_t hall_speed_clamp_direction(int8_t direction)
{
	if (direction > 0)
	{
		return 1;
	}
	if (direction < 0)
	{
		return -1;
	}
	return 0;
}

void HallSpeed_Init(void)
{
	const uint32_t now_cycles = hall_speed_get_cycles();

	__disable_irq();
	if (HALL_COUNT_USE_CHANNEL_B != 0U)
	{
		hall_count_pin = HallB_Pin;
	}
	else
	{
		hall_count_pin = HallA_Pin;
	}
	g_hall_speed_state.event_count_total = 0;
	g_hall_speed_state.last_event_cycles = 0U;
	g_hall_speed_state.last_period_us = 0U;
	g_hall_speed_state.zero_command_since_cycles = now_cycles;
	g_hall_speed_state.fault_count = 0U;
	g_hall_speed_state.direction = 0;
	g_hall_speed_state.command_direction = 0;
	g_hall_speed_state.speed_valid = 0U;
	g_hall_speed_state.timeout_active = 0U;
	g_hall_speed_state.stationary_confirmed = 0U;
	g_hall_speed_state.period_origin_valid = 0U;
	g_hall_speed_started_cycles = now_cycles;
	__enable_irq();
}

void HallSpeed_SetCommandDirection(int8_t direction)
{
	const int8_t command_direction = hall_speed_clamp_direction(direction);
	const uint32_t now_cycles = hall_speed_get_cycles();
	int8_t previous_command_direction;
	uint8_t previous_stationary_confirmed;

	__disable_irq();
	previous_command_direction = g_hall_speed_state.command_direction;
	previous_stationary_confirmed = g_hall_speed_state.stationary_confirmed;
	g_hall_speed_state.command_direction = command_direction;
	if (command_direction == 0 && previous_command_direction != 0)
	{
		/* A stop request must earn a fresh no-pulse confirmation window. */
		g_hall_speed_state.zero_command_since_cycles = now_cycles;
	}
	if (command_direction != 0)
	{
		const int8_t previous_measurement_direction =
			g_hall_speed_state.direction;
		/* A new motion request invalidates an earlier no-pulse standstill. */
		g_hall_speed_state.direction = command_direction;
		if (command_direction != previous_command_direction ||
			g_hall_speed_state.stationary_confirmed != 0U)
		{
			g_hall_speed_state.speed_valid = 0U;
			g_hall_speed_state.timeout_active = 0U;
			g_hall_speed_state.stationary_confirmed = 0U;
		}
		if (previous_stationary_confirmed != 0U ||
			(previous_measurement_direction != 0 &&
			 previous_measurement_direction != command_direction))
		{
			g_hall_speed_state.last_period_us = 0U;
			g_hall_speed_state.period_origin_valid = 0U;
		}
	}
	/*
	 * A zero request deliberately retains direction until Hall silence is
	 * confirmed. This preserves the command-derived sign while the wheel
	 * coasts; it is still not an independently measured direction.
	 */
	__enable_irq();
}

void HallSpeed_OnCountEvent(void)
{
	const uint32_t now_cycles = hall_speed_get_cycles();
	const uint32_t last_event_cycles = g_hall_speed_state.last_event_cycles;
	uint8_t period_accepted = 0U;

	if (g_hall_speed_state.period_origin_valid != 0U &&
		g_hall_speed_state.timeout_active == 0U)
	{
		const uint32_t elapsed_us =
			hall_speed_elapsed_us(now_cycles, last_event_cycles);
		if (elapsed_us < HALL_MIN_EVENT_INTERVAL_US)
		{
			g_hall_speed_state.fault_count++;
			return;
		}
		if (elapsed_us <= HALL_TIMEOUT_MAX_US)
		{
			g_hall_speed_state.last_period_us = elapsed_us;
			g_hall_speed_state.speed_valid = 1U;
			period_accepted = 1U;
		}
	}

	if (period_accepted == 0U)
	{
		/* A first edge after acquisition or timeout only establishes an origin. */
		g_hall_speed_state.last_period_us = 0U;
		g_hall_speed_state.speed_valid = 0U;
	}
	g_hall_speed_state.last_event_cycles = now_cycles;
	g_hall_speed_state.period_origin_valid = 1U;
	g_hall_speed_state.timeout_active = 0U;
	g_hall_speed_state.stationary_confirmed = 0U;
	g_hall_speed_state.event_count_total++;
}

void HallSpeed_ClearFaultCount(void)
{
	__disable_irq();
	g_hall_speed_state.fault_count = 0U;
	__enable_irq();
}

hall_speed_state_t HallSpeed_GetState(void)
{
	hall_speed_state_t snapshot;

	for (;;)
	{
		uint32_t quiet_reference_cycles;
		uint32_t quiet_timeout_us;
		uint32_t now_cycles;
		uint32_t quiet_elapsed_us;
		uint32_t zero_command_elapsed_us;
		uint8_t zero_command_quiet;
		uint8_t state_changed;

		__disable_irq();
		snapshot = g_hall_speed_state;
		__enable_irq();

		now_cycles = hall_speed_get_cycles();
		quiet_reference_cycles = (snapshot.event_count_total == 0) ?
			g_hall_speed_started_cycles : snapshot.last_event_cycles;
		quiet_timeout_us = (snapshot.last_period_us == 0U) ?
			HALL_TIMEOUT_MIN_US : hall_speed_get_timeout_us(snapshot.last_period_us);
		quiet_elapsed_us = hall_speed_elapsed_us(now_cycles, quiet_reference_cycles);
		zero_command_elapsed_us = hall_speed_elapsed_us(
			now_cycles, snapshot.zero_command_since_cycles);
		zero_command_quiet =
			(snapshot.command_direction == 0 &&
			 zero_command_elapsed_us >= quiet_timeout_us) ? 1U : 0U;

		if (snapshot.stationary_confirmed != 0U && snapshot.command_direction == 0)
		{
			snapshot.direction = 0;
			snapshot.speed_valid = 0U;
			snapshot.timeout_active = 1U;
		}
		else if (snapshot.timeout_active != 0U && snapshot.command_direction != 0)
		{
			/* A timed-out measurement stays unavailable until a fresh Hall edge. */
			snapshot.speed_valid = 0U;
			snapshot.timeout_active = 1U;
			snapshot.stationary_confirmed = 0U;
		}
		else if (quiet_elapsed_us >= quiet_timeout_us)
		{
			snapshot.speed_valid = 0U;
			snapshot.timeout_active = 1U;
			if (zero_command_quiet != 0U)
			{
				snapshot.direction = 0;
				snapshot.stationary_confirmed = 1U;
			}
			else
			{
				/* No pulses under a non-zero request is unknown, not zero. */
				snapshot.stationary_confirmed = 0U;
			}
		}
		else
		{
			snapshot.speed_valid =
				(snapshot.period_origin_valid != 0U && snapshot.last_period_us != 0U) ? 1U : 0U;
			snapshot.timeout_active = 0U;
			snapshot.stationary_confirmed = 0U;
		}

		__disable_irq();
		state_changed =
			(g_hall_speed_state.event_count_total != snapshot.event_count_total ||
			 g_hall_speed_state.last_event_cycles != snapshot.last_event_cycles ||
			 g_hall_speed_state.command_direction != snapshot.command_direction) ? 1U : 0U;
		if (state_changed == 0U)
		{
			g_hall_speed_state.direction = snapshot.direction;
			g_hall_speed_state.speed_valid = snapshot.speed_valid;
			g_hall_speed_state.timeout_active = snapshot.timeout_active;
			g_hall_speed_state.stationary_confirmed = snapshot.stationary_confirmed;
		}
		__enable_irq();

		if (state_changed == 0U)
		{
			return snapshot;
		}
	}
}

uint8_t HallSpeed_GetSignedSpeedMps(float *speed_mps)
{
	const hall_speed_state_t snapshot = HallSpeed_GetState();
	float speed = 0.0f;

	if (snapshot.speed_valid == 0U || snapshot.direction == 0 || snapshot.last_period_us == 0U)
	{
		if (speed_mps != NULL)
		{
			*speed_mps = 0.0f;
		}
		return 0U;
	}

	speed = (1000000.0f / (float)snapshot.last_period_us) *
		(HALL_WHEEL_CIRCUMFERENCE_M / (float)HALL_COUNT_EVENTS_PER_REV);
	if (snapshot.direction < 0)
	{
		speed = -speed;
	}

	if (speed_mps != NULL)
	{
		*speed_mps = speed;
	}
	return 1U;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == hall_count_pin)
	{
		HallSpeed_OnCountEvent();
	}
}
