/**
 * @file SerialControl_task.c
 * @brief ROS UART command parser for the Ackermann-only control path.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#include "app_runtime_state.h"
#include "servo_basic_control.h"

#define ROS_CMD_FRAME_LEN 11U
#define ROS_CMD_ACKERMANN 0x01U

#define ROS_CMD_FLAG_ENABLE         0x01U
#define ROS_CMD_FLAG_BRAKE          0x02U
#define ROS_CMD_FLAG_CLEAR_FAULT    0x04U
#define ROS_CMD_FLAG_SOFTWARE_STOP  0x80U
#define ROS_CMD_FLAG_ALLOWED_MASK   0x87U

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

static uint8_t serial_control_retain_next_header(uint8_t *buffer, uint8_t length)
{
	uint8_t start;

	for (start = 1U; start < length; ++start)
	{
		uint8_t index;
		uint8_t remaining;

		if (buffer[start] != 0x7BU)
		{
			continue;
		}

		remaining = (uint8_t)(length - start);
		for (index = 0U; index < remaining; ++index)
		{
			buffer[index] = buffer[start + index];
		}
		return remaining;
	}

	return 0U;
}

static uint8_t serial_control_is_zero_command(int16_t speed_mmps, int16_t steering_mrad)
{
	return (speed_mmps == 0 && steering_mrad == 0) ? 1U : 0U;
}

static void serial_control_send_zero_command(void)
{
	ServoBasic_UpdateAckermannFromOrin(0.0f, 0.0f, 1U, 0U, 0U);
}

static void serial_control_try_clear_diagnostics(uint8_t enable,
												 uint8_t brake,
												 uint8_t software_stop,
												 int16_t speed_mmps,
												 int16_t steering_mrad)
{
	if (software_stop != 0U || speed_mmps != 0 || steering_mrad != 0)
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

void SerialControlTask(void *param)
{
	extern QueueHandle_t g_xQueueROSserial;

	uint8_t recv = 0U;
	uint8_t roscmdBuf[ROS_CMD_FRAME_LEN] = {0U};
	uint8_t roscmdCount = 0U;
	const uint8_t cmdLen = ROS_CMD_FRAME_LEN;

	(void)param;

	for (;;)
	{
		if (pdPASS != xQueueReceive(g_xQueueROSserial, &recv, portMAX_DELAY))
		{
			continue;
		}

		if (roscmdCount == 0U)
		{
			if (recv != 0x7BU)
			{
				continue;
			}
			roscmdBuf[roscmdCount++] = recv;
		}
		else
		{
			roscmdBuf[roscmdCount++] = recv;
		}

		if (roscmdCount != cmdLen)
		{
			continue;
		}

		if (roscmdBuf[cmdLen - 1U] != 0x7DU || roscmdBuf[cmdLen - 2U] != Calculate_BCC(roscmdBuf, cmdLen - 2U))
		{
			g_app_runtime_state.uart4_rx_frame_error_seen = 1U;
			roscmdCount = serial_control_retain_next_header(roscmdBuf, cmdLen);
			continue;
		}

		roscmdCount = 0U;

		if (roscmdBuf[1] != ROS_CMD_ACKERMANN || roscmdBuf[7] != 0U || roscmdBuf[8] != 0U)
		{
			g_app_runtime_state.uart4_rx_frame_error_seen = 1U;
			continue;
		}

		{
			const uint8_t flags = roscmdBuf[2];
			const int16_t speed_mmps = serial_control_read_i16_be(&roscmdBuf[3]);
			const int16_t steering_mrad = serial_control_read_i16_be(&roscmdBuf[5]);
			const uint8_t enable = (flags & ROS_CMD_FLAG_ENABLE) ? 1U : 0U;
			const uint8_t brake = (flags & ROS_CMD_FLAG_BRAKE) ? 1U : 0U;
			const uint8_t clear_fault = (flags & ROS_CMD_FLAG_CLEAR_FAULT) ? 1U : 0U;
			const uint8_t software_stop = (flags & ROS_CMD_FLAG_SOFTWARE_STOP) ? 1U : 0U;
			const float speed_mps = (float)speed_mmps / 1000.0f;
			const float steering_angle_rad = (float)steering_mrad / 1000.0f;
			const uint8_t command_is_zero = serial_control_is_zero_command(speed_mmps, steering_mrad);
			uint8_t allow_serial_motion = 1U;
			const uint8_t rc_override_active = ServoBasic_IsRcOverrideActive();

			if ((flags & (uint8_t)(~ROS_CMD_FLAG_ALLOWED_MASK)) != 0U)
			{
				g_app_runtime_state.uart4_rx_frame_error_seen = 1U;
				continue;
			}

			if (clear_fault != 0U)
			{
				serial_control_try_clear_diagnostics(
					enable, brake, software_stop, speed_mmps, steering_mrad);
			}

			if (rc_override_active != 0U)
			{
				allow_serial_motion = (command_is_zero != 0U || software_stop != 0U) ? 1U : 0U;
				if (allow_serial_motion == 0U)
				{
					/*
					 * RC owns the outputs, but keep the serial source fresh with an
					 * explicit zero target. This lets the unchanged 500 ms centered
					 * release hold complete even though the command timeout is 250 ms,
					 * without caching a non-zero automatic target for release.
					 */
					serial_control_send_zero_command();
				}
			}

			if (allow_serial_motion != 0U)
			{
				ServoBasic_UpdateAckermannFromOrin(
					speed_mps, steering_angle_rad, enable, brake, software_stop);
			}
		}
	}
}
