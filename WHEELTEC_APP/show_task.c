#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "bsp_oled.h"
#include "hall_speed.h"
#include "app_runtime_state.h"
#include "app_vehicle_config.h"
#include "servo_basic_control.h"
#include <string.h>

static pOLEDInterface_t oled = &UserOLED;

#define SHOW_KEY_POLL_MS       20U
#define SHOW_KEY_DEBOUNCE_MS   40U
#define SHOW_REFRESH_MS       100U
#define SHOW_PAGE_RUNTIME       0U
#define SHOW_PAGE_DIAGNOSTIC    1U

static void show_u4_zero_padded(uint8_t x, uint8_t y, uint32_t value)
{
    value %= 10000U;
    oled->ShowNumber(x, y, (value / 1000U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 8U), y, (value / 100U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 16U), y, (value / 10U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 24U), y, value % 10U, 1, 12);
}

static void show_u2_zero_padded(uint8_t x, uint8_t y, uint32_t value)
{
    value %= 100U;
    oled->ShowNumber(x, y, (value / 10U) % 10U, 1, 12);
    oled->ShowNumber((uint8_t)(x + 8U), y, value % 10U, 1, 12);
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

static void show_clear_line(uint8_t y)
{
    oled->ShowString(0, y, "                ");
}

static void show_runtime_page(
    const servo_basic_control_snapshot_t *control_snapshot,
    const hall_speed_state_t *hall)
{
    show_clear_line(0);
    oled->ShowString(0, 0, "M:");
    oled->ShowString(16, 0,
        (control_snapshot->state.control_mode == SERVO_CTRL_MODE_RC_PASSTHROUGH) ?
        "RC  " : "AUTO");
    oled->ShowString(56, 0, "R");
    oled->ShowNumber(64, 0, control_snapshot->rc_override_active, 1, 12);
    oled->ShowString(80, 0, "G");
    oled->ShowNumber(88, 0, control_snapshot->rc_emergency_active, 1, 12);

    show_clear_line(12);
    oled->ShowString(0, 12, "E");
    oled->ShowNumber(8, 12, control_snapshot->state.esc_pulse_us, 4, 12);
    oled->ShowString(56, 12, "S");
    oled->ShowNumber(64, 12, control_snapshot->state.servo_pulse_us, 4, 12);

    show_clear_line(24);
    oled->ShowString(0, 24, "SP");
    oled->ShowFloat(16, 24, control_snapshot->esc_uplink_speed_mps, 1, 2);
    oled->ShowString(64, 24, "ST");
    oled->ShowFloat(88, 24, control_snapshot->steering_angle_rad, 1, 2);

    show_clear_line(36);
    oled->ShowString(0, 36, "V");
    oled->ShowFloat(8, 36, g_app_runtime_state.voltage_v, 2, 2);
    oled->ShowString(64, 36, "F");
    show_u4_zero_padded(72, 36,
        control_snapshot->diagnostics.esc_valid_frame_count);

    show_clear_line(48);
    oled->ShowString(0, 48, "A");
    oled->ShowNumber(8, 48,
        (HAL_GPIO_ReadPin(HallA_GPIO_Port, HallA_Pin) == GPIO_PIN_RESET) ? 0U : 1U,
        1, 12);
    oled->ShowString(24, 48, "B");
    oled->ShowNumber(32, 48,
        (HAL_GPIO_ReadPin(HallB_GPIO_Port, HallB_Pin) == GPIO_PIN_RESET) ? 0U : 1U,
        1, 12);
    oled->ShowString(48, 48, "D");
    oled->ShowString(56, 48,
        (hall->direction < 0) ? "-" : ((hall->direction > 0) ? "+" : "0"));
    oled->ShowString(72, 48, "C");
    show_i4_zero_padded(80, 48, hall->event_count_total);
}

static void show_diagnostic_page(
    const servo_basic_control_snapshot_t *control_snapshot,
    const hall_speed_state_t *hall)
{
    float hall_speed_mps = 0.0f;

    (void)HallSpeed_GetSnapshotSpeedMps(hall, &hall_speed_mps);

    show_clear_line(0);
    oled->ShowString(0, 0, "F");
    show_u4_zero_padded(8, 0,
        control_snapshot->diagnostics.esc_valid_frame_count);
    oled->ShowString(48, 0, "R");
    oled->ShowNumber(56, 0, control_snapshot->diagnostics.esc_rpm_raw, 5, 12);

    show_clear_line(12);
    oled->ShowString(0, 12, "U");
    show_u2_zero_padded(8, 12,
        control_snapshot->diagnostics.esc_soft_uart_error_count);
    oled->ShowString(32, 12, "X");
    show_u2_zero_padded(40, 12,
        control_snapshot->diagnostics.esc_soft_uart_error_flags);
    oled->ShowString(64, 12, "M");
    show_u2_zero_padded(72, 12,
        control_snapshot->diagnostics.mode2_state);
    oled->ShowString(96, 12, "Q");
    show_u2_zero_padded(104, 12,
        control_snapshot->diagnostics.mode2_reason);

    show_clear_line(24);
    oled->ShowString(0, 24, "T");
    oled->ShowNumber(8, 24, control_snapshot->diagnostics.esc_fe32_fresh, 1, 12);
    oled->ShowString(24, 24, "P");
    oled->ShowNumber(32, 24,
        control_snapshot->diagnostics.esc_rpm_raw_valid, 1, 12);
    oled->ShowString(48, 24, "K");
    oled->ShowNumber(56, 24,
        control_snapshot->diagnostics.esc_speed_calibration_valid, 1, 12);
    oled->ShowString(72, 24, "D");
    oled->ShowNumber(80, 24,
        control_snapshot->diagnostics.vehicle_direction_known, 1, 12);

    show_clear_line(36);
    oled->ShowString(0, 36, "SP");
    oled->ShowFloat(16, 36, control_snapshot->esc_uplink_speed_mps, 1, 2);
    oled->ShowString(64, 36, "H");
    oled->ShowFloat(72, 36, hall_speed_mps, 1, 2);

    show_clear_line(48);
    oled->ShowString(0, 48, "A");
    oled->ShowNumber(8, 48,
        (HAL_GPIO_ReadPin(HallA_GPIO_Port, HallA_Pin) == GPIO_PIN_RESET) ? 0U : 1U,
        1, 12);
    oled->ShowString(24, 48, "B");
    oled->ShowNumber(32, 48,
        (HAL_GPIO_ReadPin(HallB_GPIO_Port, HallB_Pin) == GPIO_PIN_RESET) ? 0U : 1U,
        1, 12);
    oled->ShowString(48, 48, "C");
    show_i4_zero_padded(56, 48, hall->event_count_total);
}

static void show_status_page(uint8_t page)
{
    servo_basic_control_snapshot_t control_snapshot;
    hall_speed_state_t hall = HallSpeed_GetState();

    if (ServoBasic_GetControlSnapshot(&control_snapshot) == 0U)
    {
        memset(&control_snapshot, 0, sizeof(control_snapshot));
        control_snapshot.state.esc_pulse_us = APP_ORIN_ESC_CENTER_US;
        control_snapshot.state.servo_pulse_us = APP_ORIN_SERVO_CENTER_US;
        control_snapshot.state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS;
    }

    if (page == SHOW_PAGE_DIAGNOSTIC)
    {
        show_diagnostic_page(&control_snapshot, &hall);
    }
    else
    {
        show_runtime_page(&control_snapshot, &hall);
    }

    oled->RefreshGram();
}

void show_task(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    TickType_t last_refresh = preTime - pdMS_TO_TICKS(SHOW_REFRESH_MS);
    TickType_t candidate_since = preTime;
    uint8_t key_initialized = 0U;
    uint8_t candidate_level = 0U;
    uint8_t stable_level = 0U;
    (void)param;

    while (1)
    {
        const TickType_t now = xTaskGetTickCount();
        const uint8_t raw_level =
            (HAL_GPIO_ReadPin(UserKey_GPIO_Port, UserKey_Pin) == GPIO_PIN_SET) ? 1U : 0U;

        if (key_initialized == 0U)
        {
            candidate_level = raw_level;
            stable_level = raw_level;
            candidate_since = now;
            key_initialized = 1U;
        }
        else if (raw_level != candidate_level)
        {
            candidate_level = raw_level;
            candidate_since = now;
        }
        else if (candidate_level != stable_level &&
                 (now - candidate_since) >= pdMS_TO_TICKS(SHOW_KEY_DEBOUNCE_MS))
        {
            stable_level = candidate_level;
        }

        if ((now - last_refresh) >= pdMS_TO_TICKS(SHOW_REFRESH_MS))
        {
            const uint8_t page = (stable_level == 0U) ?
                SHOW_PAGE_RUNTIME : SHOW_PAGE_DIAGNOSTIC;
            show_status_page(page);
            last_refresh = now;
        }

        vTaskDelayUntil(&preTime, pdMS_TO_TICKS(SHOW_KEY_POLL_MS));
    }
}
