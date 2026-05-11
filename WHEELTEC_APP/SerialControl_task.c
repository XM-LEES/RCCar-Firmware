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
#define ROS_CMD_TYPE_MOTION 0U
#define ROS_CMD_TYPE_LEGACY_IGNORE 4U
#define ROS_CMD_TYPE_PID_PARAM 5U
#define ROS_PID_PARAM_ENABLE 1U
#define ROS_PID_PARAM_KP_X100 2U
#define ROS_PID_PARAM_KI_X100 3U
#define ROS_PID_PARAM_TRIM_LIMIT_US 4U
#define ROS_PID_PARAM_RESET_I 5U
#define ROS_PID_PARAM_MAGIC0 0x50U
#define ROS_PID_PARAM_MAGIC1 0x49U
#define ROS_PID_PARAM_MAGIC2 0x44U
#define ROS_PID_PARAM_MAGIC3 0x31U

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

static uint8_t serial_control_is_zero_motion(float vx_mps, float vy_mps, float vz_rad_s)
{
	return (vx_mps == 0.0f && vy_mps == 0.0f && vz_rad_s == 0.0f) ? 1U : 0U;
}

static int16_t serial_control_read_i16_be(const uint8_t *buf)
{
	return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static void serial_control_send_zero_command(void)
{
	ServoBasic_UpdateFromOrin(0.0f, 0.0f, 0.0f, 1U);
}

static void serial_control_apply_pid_param_frame(const uint8_t *frame)
{
	const uint8_t param_id = frame[2];
	const int16_t value = serial_control_read_i16_be(&frame[3]);

	if (frame[5] != ROS_PID_PARAM_MAGIC0 ||
		frame[6] != ROS_PID_PARAM_MAGIC1 ||
		frame[7] != ROS_PID_PARAM_MAGIC2 ||
		frame[8] != ROS_PID_PARAM_MAGIC3)
	{
		return;
	}

	switch (param_id)
	{
	case ROS_PID_PARAM_ENABLE:
		ServoBasic_SetSpeedPiEnable((value != 0) ? 1U : 0U);
		break;
	case ROS_PID_PARAM_KP_X100:
		ServoBasic_SetSpeedPiKp((float)value / 100.0f);
		break;
	case ROS_PID_PARAM_KI_X100:
		ServoBasic_SetSpeedPiKi((float)value / 100.0f);
		break;
	case ROS_PID_PARAM_TRIM_LIMIT_US:
		ServoBasic_SetSpeedPiTrimLimitUs((value > 0) ? (uint32_t)value : 0U);
		break;
	case ROS_PID_PARAM_RESET_I:
		ServoBasic_ResetSpeedPi();
		break;
	default:
		break;
	}
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
	static uint8_t idleCount = 0U;

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
			continue;
		}

		if (roscmdBuf[1] == ROS_CMD_TYPE_PID_PARAM)
		{
			serial_control_apply_pid_param_frame(roscmdBuf);
			continue;
		}

		if (roscmdBuf[1] == ROS_CMD_TYPE_LEGACY_IGNORE)
		{
			continue;
		}

		if (roscmdBuf[1] != ROS_CMD_TYPE_MOTION)
		{
			continue;
		}

		{
			const float vx_mps = (float)serial_control_read_i16_be(&roscmdBuf[3]) / 1000.0f;
			const float vy_mps = (float)serial_control_read_i16_be(&roscmdBuf[5]) / 1000.0f;
			const float vz_rad_s = (float)serial_control_read_i16_be(&roscmdBuf[7]) / 1000.0f;
			const uint8_t flag_stop = (roscmdBuf[2] & 0x80U) ? 1U : 0U;
			const uint8_t motion_is_zero = serial_control_is_zero_motion(vx_mps, vy_mps, vz_rad_s);
			uint8_t allow_serial_motion = 1U;
			const uint8_t rc_override_active = ServoBasic_IsRcOverrideActive();

			if (motion_is_zero != 0U)
			{
				if (idleCount < 10U)
				{
					idleCount++;
				}
			}
			else
			{
				idleCount = 0U;
			}

			if (rc_override_active != 0U)
			{
				if (s_rc_override_blocking == 0U)
				{
					serial_control_send_zero_command();
					s_rc_override_blocking = 1U;
				}
				allow_serial_motion = (motion_is_zero != 0U || flag_stop != 0U) ? 1U : 0U;
			}
			else
			{
				s_rc_override_blocking = 0U;
			}

			if (allow_serial_motion != 0U && (idleCount < 10U || motion_is_zero != 0U || flag_stop != 0U))
			{
				ServoBasic_UpdateFromOrin(vx_mps, vy_mps, vz_rad_s, flag_stop);
			}
		}
	}
}
