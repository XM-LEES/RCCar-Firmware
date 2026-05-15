/**
 * @file SerialControl_task.c
 * @brief ROS UART command parser for the Ackermann-only control path.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#include "bsp_buzzer.h"
#include "main.h"
#include "app_runtime_state.h"
#include "servo_basic_control.h"

#define ROS_CMD_FRAME_LEN 11U
#define ROS_CMD_ACKERMANN 0x01U

#define ROS_CMD_FLAG_ENABLE         0x01U
#define ROS_CMD_FLAG_BRAKE          0x02U
#define ROS_CMD_FLAG_CLEAR_FAULT    0x04U
#define ROS_CMD_FLAG_EMERGENCY_STOP 0x80U

#if ROS_CMD_FRAME_LEN != 11U
#error "ROS downlink command frame must remain 11 bytes for the firmware parser."
#endif

uint8_t Calculate_BCC(const uint8_t *checkdata, uint16_t datalen)
{
	uint8_t bccval = 0U;
	uint16_t i = 0U;
	for (i = 0U; i < datalen; i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}

static int16_t serial_control_read_i16_be(const uint8_t *buffer)
{
	return (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
}

static uint8_t serial_control_is_zero_command(int16_t speed_mmps, int16_t steering_mrad)
{
	return (speed_mmps == 0 && steering_mrad == 0) ? 1U : 0U;
}

static void serial_control_send_zero_command(void)
{
	ServoBasic_UpdateAckermannFromOrin(0.0f, 0.0f, 1U, 1U, 0U);
}

static void serial_control_try_clear_diagnostics(uint8_t enable,
												 uint8_t brake,
												 uint8_t emergency_stop,
												 int16_t speed_mmps)
{
	if (emergency_stop != 0U || speed_mmps != 0)
	{
		return;
	}
	if (brake == 0U && enable != 0U)
	{
		return;
	}
	if (ServoBasic_IsRcEmergencyActive() != 0U)
	{
		return;
	}
	if (ServoBasic_IsRcOverrideActive() != 0U)
	{
		return;
	}

	g_app_runtime_state.uart4_rx_frame_error_seen = 0U;
	AppRuntime_TryClearFaultLatch();
}

static void serial_control_check_reset(char uart_recv)
{
	static char res_buf[5] = {0};
	static uint8_t res_count = 0U;

	res_buf[res_count] = uart_recv;

	if (uart_recv == 'r' || res_count > 0U)
	{
		res_count++;
	}
	else
	{
		res_count = 0U;
	}

	if (res_count != 5U)
	{
		return;
	}

	res_count = 0U;
	if (res_buf[0] == 'r' && res_buf[1] == 'e' && res_buf[2] == 's' && res_buf[3] == 'e' && res_buf[4] == 't')
	{
		NVIC_SystemReset();
	}
}

static void serial_control_set_debug_level(char uart_recv)
{
	static char log_buf[4] = {0};
	static uint8_t log_count = 0U;

	log_buf[log_count] = uart_recv;

	if (uart_recv == 'L' || log_count > 0U)
	{
		log_count++;
	}
	else
	{
		log_count = 0U;
	}

	if (log_count != 4U)
	{
		return;
	}

	log_count = 0U;
	if (log_buf[0] != 'L' || log_buf[1] != 'O' || log_buf[2] != 'G')
	{
		return;
	}

	switch (log_buf[3])
	{
	case '0': g_app_runtime_state.debug_level = 0U; break;
	case '1': g_app_runtime_state.debug_level = 1U; break;
	case '2': g_app_runtime_state.debug_level = 2U; break;
	case '3': g_app_runtime_state.debug_level = 3U; break;
	default:  g_app_runtime_state.debug_level = 0U; break;
	}

	{
		pBuzzerInterface_t tips = &UserBuzzer;
		tips->AddTask(1U, 200U);
	}
}

void SerialControlTask(void *param)
{
	extern QueueHandle_t g_xQueueROSserial;

	uint8_t recv = 0U;
	uint8_t roscmdBuf[20] = {0U};
	uint8_t roscmdCount = 0U;
	const uint8_t cmdLen = ROS_CMD_FRAME_LEN;
	static uint8_t s_rc_override_blocking = 0U;

	(void)param;

	for (;;)
	{
		if (pdPASS != xQueueReceive(g_xQueueROSserial, &recv, portMAX_DELAY))
		{
			continue;
		}

		serial_control_check_reset((char)recv);
		serial_control_set_debug_level((char)recv);

		if (recv == 0x7BU || roscmdCount > 0U)
		{
			roscmdBuf[roscmdCount++] = recv;
		}
		else
		{
			roscmdCount = 0U;
		}

		if (roscmdCount != cmdLen)
		{
			continue;
		}

		roscmdCount = 0U;

		if (roscmdBuf[cmdLen - 1U] != 0x7DU || roscmdBuf[cmdLen - 2U] != Calculate_BCC(roscmdBuf, cmdLen - 2U))
		{
			g_app_runtime_state.uart4_rx_frame_error_seen = 1U;
			continue;
		}

		if (roscmdBuf[1] != ROS_CMD_ACKERMANN)
		{
			continue;
		}

		{
			const uint8_t flags = roscmdBuf[2];
			const int16_t speed_mmps = serial_control_read_i16_be(&roscmdBuf[3]);
			const int16_t steering_mrad = serial_control_read_i16_be(&roscmdBuf[5]);
			const uint8_t enable = (flags & ROS_CMD_FLAG_ENABLE) ? 1U : 0U;
			const uint8_t brake = (flags & ROS_CMD_FLAG_BRAKE) ? 1U : 0U;
			const uint8_t clear_fault = (flags & ROS_CMD_FLAG_CLEAR_FAULT) ? 1U : 0U;
			const uint8_t emergency_stop = (flags & ROS_CMD_FLAG_EMERGENCY_STOP) ? 1U : 0U;
			const float speed_mps = (float)speed_mmps / 1000.0f;
			const float steering_angle_rad = (float)steering_mrad / 1000.0f;
			const uint8_t command_is_zero = serial_control_is_zero_command(speed_mmps, steering_mrad);
			uint8_t allow_serial_motion = 1U;
			const uint8_t rc_override_active = ServoBasic_IsRcOverrideActive();

			if (clear_fault != 0U)
			{
				serial_control_try_clear_diagnostics(enable, brake, emergency_stop, speed_mmps);
			}

			if (rc_override_active != 0U)
			{
				if (s_rc_override_blocking == 0U)
				{
					serial_control_send_zero_command();
					s_rc_override_blocking = 1U;
				}
				allow_serial_motion = (command_is_zero != 0U || brake != 0U ||
					enable == 0U || emergency_stop != 0U) ? 1U : 0U;
			}
			else
			{
				s_rc_override_blocking = 0U;
			}

			if (allow_serial_motion != 0U)
			{
				ServoBasic_UpdateAckermannFromOrin(speed_mps, steering_angle_rad, enable, brake, emergency_stop);
			}
		}
	}
}
