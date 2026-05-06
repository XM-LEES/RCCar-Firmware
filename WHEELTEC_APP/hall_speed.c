/**
 * @file hall_speed.c
 * @brief Hall speed measurement with configurable count and direction channels.
 */

#include "hall_speed.h"

#include "main.h"
#include "bsp_dwt.h"

#define HALL_WHEEL_DIAMETER_M            0.235f
#define HALL_WHEEL_CIRCUMFERENCE_M       (HALL_WHEEL_DIAMETER_M * 3.14159265358979f)
#define HALL_COUNT_EVENTS_PER_REV        10U
#define HALL_MIN_EVENT_INTERVAL_US       1500U
#define HALL_TIMEOUT_MIN_US           500000U
#define HALL_TIMEOUT_MAX_US          4000000U
#define HALL_COUNT_USE_CHANNEL_B         1U

volatile hall_speed_state_t g_hall_speed_state = {0};

static uint16_t hall_count_pin = HallA_Pin;

static uint32_t hall_speed_get_time_us(void)
{
	const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
	if (cycles_per_us == 0U)
	{
		return 0U;
	}
	return DWT_CYCCNT / cycles_per_us;
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
	g_hall_speed_state.last_event_us = 0U;
	g_hall_speed_state.last_period_us = 0U;
	g_hall_speed_state.fault_count = 0U;
	g_hall_speed_state.direction = 0;
	g_hall_speed_state.speed_valid = 0U;
	g_hall_speed_state.timeout_active = 1U;
	g_hall_speed_state.reserved0 = 0U;
	g_hall_speed_state.reserved1 = 0U;
	__enable_irq();
}

void HallSpeed_SetCommandDirection(int8_t direction)
{
	__disable_irq();
	g_hall_speed_state.direction = hall_speed_clamp_direction(direction);
	__enable_irq();
}

void HallSpeed_OnCountEvent(void)
{
	const uint32_t now_us = hall_speed_get_time_us();
	const uint32_t last_event_us = g_hall_speed_state.last_event_us;

	if (last_event_us != 0U)
	{
		const uint32_t elapsed_us = now_us - last_event_us;
		if (elapsed_us < HALL_MIN_EVENT_INTERVAL_US)
		{
			g_hall_speed_state.fault_count++;
			return;
		}
		g_hall_speed_state.last_period_us = elapsed_us;
		g_hall_speed_state.speed_valid = 1U;
	}

	g_hall_speed_state.last_event_us = now_us;
	g_hall_speed_state.timeout_active = 0U;
	g_hall_speed_state.event_count_total++;
}

hall_speed_state_t HallSpeed_GetState(void)
{
	hall_speed_state_t snapshot;
	const uint32_t now_us = hall_speed_get_time_us();

	__disable_irq();
	snapshot = g_hall_speed_state;
	__enable_irq();

	if (snapshot.last_period_us == 0U || snapshot.last_event_us == 0U)
	{
		snapshot.speed_valid = 0U;
		snapshot.timeout_active = 1U;
	}
	else if ((now_us - snapshot.last_event_us) > hall_speed_get_timeout_us(snapshot.last_period_us))
	{
		snapshot.speed_valid = 0U;
		snapshot.timeout_active = 1U;
	}
	else
	{
		snapshot.speed_valid = 1U;
		snapshot.timeout_active = 0U;
	}

	__disable_irq();
	g_hall_speed_state.speed_valid = snapshot.speed_valid;
	g_hall_speed_state.timeout_active = snapshot.timeout_active;
	__enable_irq();

	return snapshot;
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
