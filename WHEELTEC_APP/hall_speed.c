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
#define HALL_DIR_INVERT                  1U
#define HALL_COUNT_USE_CHANNEL_B         1U
#define HALL_STATE_A_HIGH             0x02U
#define HALL_STATE_B_HIGH             0x01U
#define HALL_STATE_00                 0x00U
#define HALL_STATE_01                 0x01U
#define HALL_STATE_10                 0x02U
#define HALL_STATE_11                 0x03U
#define HALL_SEQUENCE_NONE            0xFFU

volatile hall_speed_state_t g_hall_speed_state = {0};

static uint16_t hall_count_pin = HallA_Pin;
static uint8_t hall_last_state = HALL_STATE_11;
static uint8_t hall_sequence_origin = HALL_SEQUENCE_NONE;
static uint8_t hall_state_initialized = 0U;

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

static uint8_t hall_speed_read_state(void)
{
	uint8_t state = 0U;

	if (HAL_GPIO_ReadPin(HallA_GPIO_Port, HallA_Pin) != GPIO_PIN_RESET)
	{
		state |= HALL_STATE_A_HIGH;
	}
	if (HAL_GPIO_ReadPin(HallB_GPIO_Port, HallB_Pin) != GPIO_PIN_RESET)
	{
		state |= HALL_STATE_B_HIGH;
	}
	return state;
}

static uint8_t hall_speed_count_pin_is_low(uint8_t state)
{
	if (hall_count_pin == HallB_Pin)
	{
		return ((state & HALL_STATE_B_HIGH) == 0U) ? 1U : 0U;
	}
	return ((state & HALL_STATE_A_HIGH) == 0U) ? 1U : 0U;
}

static uint8_t hall_speed_changed_bit_count(uint8_t previous_state, uint8_t state)
{
	const uint8_t diff = (previous_state ^ state) & HALL_STATE_11;

	if (diff == 0U)
	{
		return 0U;
	}
	if (diff == HALL_STATE_A_HIGH || diff == HALL_STATE_B_HIGH)
	{
		return 1U;
	}
	return 2U;
}

static int8_t hall_speed_apply_dir_invert(int8_t direction)
{
#if HALL_DIR_INVERT != 0U
	return (int8_t)-direction;
#else
	return direction;
#endif
}

static void hall_speed_accept_direction(int8_t direction)
{
	g_hall_speed_state.direction = hall_speed_apply_dir_invert(direction);
}

static void hall_speed_record_state_fault(void)
{
	g_hall_speed_state.fault_count++;
	hall_sequence_origin = HALL_SEQUENCE_NONE;
}

static uint8_t hall_speed_state_is_endpoint(uint8_t state)
{
	return (state == HALL_STATE_01 || state == HALL_STATE_10) ? 1U : 0U;
}

static uint8_t hall_speed_state_is_bridge(uint8_t state)
{
	return (state == HALL_STATE_00 || state == HALL_STATE_11) ? 1U : 0U;
}

static void hall_speed_observe_state(uint8_t state)
{
	if (hall_state_initialized == 0U)
	{
		hall_last_state = state;
		hall_state_initialized = 1U;
		return;
	}

	if (state == hall_last_state)
	{
		return;
	}

	if (hall_speed_changed_bit_count(hall_last_state, state) > 1U)
	{
		hall_speed_record_state_fault();
		hall_last_state = state;
		return;
	}

	if (hall_speed_state_is_endpoint(hall_last_state) != 0U &&
		hall_speed_state_is_bridge(state) != 0U)
	{
		hall_sequence_origin = hall_last_state;
	}
	else if (hall_speed_state_is_bridge(hall_last_state) != 0U &&
		hall_speed_state_is_endpoint(state) != 0U)
	{
		if (hall_sequence_origin == HALL_STATE_01 && state == HALL_STATE_10)
		{
			hall_speed_accept_direction(-1);
		}
		else if (hall_sequence_origin == HALL_STATE_10 && state == HALL_STATE_01)
		{
			hall_speed_accept_direction(1);
		}
		else if (hall_sequence_origin == HALL_STATE_01 || hall_sequence_origin == HALL_STATE_10)
		{
			hall_speed_record_state_fault();
		}
		hall_sequence_origin = HALL_SEQUENCE_NONE;
	}

	hall_last_state = state;
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
	hall_last_state = hall_speed_read_state();
	hall_sequence_origin = HALL_SEQUENCE_NONE;
	hall_state_initialized = 1U;
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

void HallSpeed_OnGpioEdge(uint16_t GPIO_Pin)
{
	const uint8_t state = hall_speed_read_state();

	if (GPIO_Pin != HallA_Pin && GPIO_Pin != HallB_Pin)
	{
		return;
	}

	hall_speed_observe_state(state);
	if (GPIO_Pin == hall_count_pin && hall_speed_count_pin_is_low(state) != 0U)
	{
		HallSpeed_OnCountEvent();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HallSpeed_OnGpioEdge(GPIO_Pin);
}
