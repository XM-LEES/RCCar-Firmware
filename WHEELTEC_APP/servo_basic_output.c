/**
 * @brief Hardware binding for servo/ESC PWM outputs on STM32F407.
 *
 * ESC PWM  -> TIM8 CH1 (PC6)
 * Servo PWM-> TIM8 CH2 (PC7)
 *
 * Timer tick: 1 MHz (1 us), period: 2630 (about 380 Hz frame)
 */

#include "servo_basic_control.h"
#include "tim.h"

static void start_pwm_once(void)
{
	static uint8_t started = 0U;
	if (started)
	{
		return;
	}
	started = 1U;

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // BP: start TIM8 PWM outputs
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

void ServoBasic_OutputEscPulse(uint16_t pulse_us)
{
	start_pwm_once();
	if (pulse_us > __HAL_TIM_GET_AUTORELOAD(&htim8))
	{
		pulse_us = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim8);
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_us); // BP: update ESC pulse
}

void ServoBasic_OutputServoPulse(uint16_t pulse_us)
{
	start_pwm_once();
	if (pulse_us > __HAL_TIM_GET_AUTORELOAD(&htim8))
	{
		pulse_us = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim8);
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pulse_us); // BP: update servo pulse
}
