#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "bsp_oled.h"
#include "bsp_adc.h"
#include "hall_speed.h"
#include "app_runtime_state.h"
#include "app_vehicle_config.h"
#include "servo_basic_control.h"
#include <string.h>

static pOLEDInterface_t oled = &UserOLED;

static void show_u4_zero_padded(uint8_t x, uint8_t y, uint32_t value)
{
    value %= 10000U;
    oled->ShowNumber(x, y, (value / 1000U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 8U), y, (value / 100U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 16U), y, (value / 10U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 24U), y, value % 10U, 1, 12);
}

static void show_i4_zero_padded(uint8_t x, uint8_t y, int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        oled->ShowString(x, y, "-");
        magnitude = (uint32_t)(-value);
    }
    else
    {
        oled->ShowString(x, y, "+");
        magnitude = (uint32_t)value;
    }

    show_u4_zero_padded((uint8_t)(x + 8U), y, magnitude);
}

static void show_debug_status(void)
{
    servo_basic_control_snapshot_t control_snapshot;
    const uint16_t adc_vol = USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL);
    hall_speed_state_t hall = HallSpeed_GetState();

    if (ServoBasic_GetControlSnapshot(&control_snapshot) == 0U)
    {
        memset(&control_snapshot, 0, sizeof(control_snapshot));
        control_snapshot.state.esc_pulse_us = APP_ORIN_ESC_CENTER_US;
        control_snapshot.state.servo_pulse_us = APP_ORIN_SERVO_CENTER_US;
        control_snapshot.state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
    }

    oled->ShowString(0, 0, "                ");
    oled->ShowString(0, 0, "M:");
    oled->ShowString(16, 0, (control_snapshot.state.control_mode == SERVO_CTRL_MODE_RC_PASSTHROUGH) ? "RC  " : "AUTO");
    oled->ShowString(56, 0, "R");
    oled->ShowNumber(64, 0, control_snapshot.rc_override_active, 1, 12);
    oled->ShowString(80, 0, "G");
    oled->ShowNumber(88, 0, control_snapshot.rc_emergency_active, 1, 12);

    oled->ShowString(0, 12, "                ");
    oled->ShowString(0, 12, "F");
    show_u4_zero_padded(8, 12, control_snapshot.diagnostics.esc_valid_frame_count);
    oled->ShowString(48, 12, "R");
    oled->ShowNumber(56, 12, control_snapshot.diagnostics.esc_rpm_raw, 5, 12);

    oled->ShowString(0, 24, "                ");
    oled->ShowString(0, 24, "U");
    show_u4_zero_padded(8, 24, control_snapshot.diagnostics.esc_soft_uart_error_count);
    oled->ShowString(48, 24, "X");
    show_u4_zero_padded(56, 24, control_snapshot.diagnostics.esc_soft_uart_error_flags);
    oled->ShowString(88, 24, "A");
    show_u4_zero_padded(96, 24, adc_vol);

    oled->ShowString(0, 36, "                ");
    oled->ShowString(0, 36, "SP");
    oled->ShowFloat(16, 36, control_snapshot.esc_uplink_speed_mps, 1, 2);
    oled->ShowString(64, 36, "ST");
    oled->ShowFloat(88, 36, control_snapshot.steering_angle_rad, 1, 2);

    oled->ShowString(0, 48, "                ");
    oled->ShowString(0, 48, "A");
    oled->ShowNumber(8, 48, (HAL_GPIO_ReadPin(HallA_GPIO_Port, HallA_Pin) == GPIO_PIN_RESET) ? 0U : 1U, 1, 12);
    oled->ShowString(24, 48, "B");
    oled->ShowNumber(32, 48, (HAL_GPIO_ReadPin(HallB_GPIO_Port, HallB_Pin) == GPIO_PIN_RESET) ? 0U : 1U, 1, 12);
    oled->ShowString(48, 48, "D");
    oled->ShowString(56, 48, (hall.direction < 0) ? "-" : ((hall.direction > 0) ? "+" : "0"));
    oled->ShowString(72, 48, "C");
    show_i4_zero_padded(80, 48, hall.event_count_total);

    oled->RefreshGram();
}

void show_task(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    const uint16_t TaskPeriodMs = 100U;
    (void)param;

    while (1)
    {
        show_debug_status();
        vTaskDelayUntil(&preTime, pdMS_TO_TICKS(TaskPeriodMs));
    }
}
