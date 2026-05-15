#include "usart.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_adc.h"
#include "app_runtime_state.h"
#include "app_vehicle_config.h"
#include "hall_speed.h"
#include "servo_basic_control.h"

extern uint8_t Calculate_BCC(const uint8_t* checkdata, uint16_t datalen);

static UART_HandleTypeDef *serial = &huart4;

#define BaseFRAME_HEAD 0x7B
#define BaseFRAME_TAIL 0x7D
#define BaseFRAME_LEN  24U

#define TELEMETRY_FLAG_AUTO_ENABLED        0x01U
#define TELEMETRY_FLAG_RC_OVERRIDE_ACTIVE  0x02U
#define TELEMETRY_FLAG_ESTOP_ACTIVE        0x04U
#define TELEMETRY_FLAG_COMMAND_TIMEOUT     0x08U
#define TELEMETRY_FLAG_BRAKE_ACTIVE        0x10U
#define TELEMETRY_FLAG_FAULT_LATCHED       0x20U
#define TELEMETRY_FLAG_STEERING_MEASURED   0x40U

#define STATUS_BIT_FAULT_LATCHED           (1UL << 0)
#define STATUS_BIT_COMMAND_TIMEOUT         (1UL << 1)
#define STATUS_BIT_RC_OVERRIDE_ACTIVE      (1UL << 2)
#define STATUS_BIT_ESTOP_ACTIVE            (1UL << 3)
#define STATUS_BIT_BRAKE_ACTIVE            (1UL << 4)
#define STATUS_BIT_AUTO_ENABLED            (1UL << 5)
#define STATUS_BIT_HALL_FEEDBACK_VALID     (1UL << 6)
#define STATUS_BIT_HALL_FAULT              (1UL << 7)
#define STATUS_BIT_STEERING_FEEDBACK_VALID (1UL << 8)
#define STATUS_BIT_STEERING_IS_MEASURED    (1UL << 9)
#define STATUS_BIT_STEERING_FAULT          (1UL << 10)
#define STATUS_BIT_BATTERY_VALID           (1UL << 11)
#define STATUS_BIT_BATTERY_LOW             (1UL << 12)
#define STATUS_BIT_BATTERY_CRITICAL        (1UL << 13)
#define STATUS_BIT_SPEED_SATURATED         (1UL << 14)
#define STATUS_BIT_STEERING_SATURATED      (1UL << 15)
#define STATUS_BIT_ACCEL_LIMITED           (1UL << 16)
#define STATUS_BIT_STEERING_RATE_LIMITED   (1UL << 17)
#define STATUS_BIT_FRAME_ERROR_SEEN        (1UL << 18)

#if BaseFRAME_LEN != 24U
#error "UART4 ROS telemetry frame must remain 24 bytes for the upper computer parser."
#endif

static void update_power_state(void)
{
    g_app_runtime_state.voltage_v = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL) / 4095.0f * 3.3f * 11.0f;
}

static uint8_t battery_mv_is_low(uint16_t battery_mv)
{
    return (battery_mv > 0U && battery_mv <= APP_BATTERY_LOW_MV) ? 1U : 0U;
}

static uint8_t battery_mv_is_critical(uint16_t battery_mv)
{
    return (battery_mv > 0U && battery_mv <= APP_BATTERY_CRITICAL_MV) ? 1U : 0U;
}

static void record_uart_tx_status(HAL_StatusTypeDef status, uint32_t *busy_count, uint32_t *error_count)
{
    if (status == HAL_BUSY)
    {
        (*busy_count)++;
    }
    else if (status != HAL_OK)
    {
        (*error_count)++;
    }
}

static int16_t clamp_float_to_i16(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }
    if (value < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t clamp_float_to_u16(float value)
{
    if (value < 0.0f)
    {
        return 0U;
    }
    if (value > 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)value;
}

static void write_i16_be(uint8_t *buffer, int16_t value)
{
    buffer[0] = (uint8_t)((uint16_t)value >> 8);
    buffer[1] = (uint8_t)((uint16_t)value);
}

static void write_u16_be(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value >> 8);
    buffer[1] = (uint8_t)value;
}

static void write_i32_be(uint8_t *buffer, int32_t value)
{
    const uint32_t encoded = (uint32_t)value;
    buffer[0] = (uint8_t)(encoded >> 24);
    buffer[1] = (uint8_t)(encoded >> 16);
    buffer[2] = (uint8_t)(encoded >> 8);
    buffer[3] = (uint8_t)encoded;
}

static void write_u32_be(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)(value >> 24);
    buffer[1] = (uint8_t)(value >> 16);
    buffer[2] = (uint8_t)(value >> 8);
    buffer[3] = (uint8_t)value;
}

static int32_t get_signed_hall_delta(const hall_speed_state_t *snapshot)
{
    static int32_t s_last_count = 0;
    static uint8_t s_have_last_count = 0U;
    int32_t delta = 0;

    if (snapshot == NULL)
    {
        return 0;
    }

    if (s_have_last_count != 0U)
    {
        delta = snapshot->event_count_total - s_last_count;
        if (snapshot->direction < 0 && delta > 0)
        {
            delta = -delta;
        }
        else if (snapshot->direction == 0)
        {
            delta = 0;
        }
    }

    s_last_count = snapshot->event_count_total;
    s_have_last_count = 1U;
    return delta;
}

void RobotDataTransmitTask(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    TickType_t lastTelemetryTick = preTime;
    const uint16_t TaskFreq = 20U;
    uint8_t basebuffer[BaseFRAME_LEN];
    uint8_t seq = 0U;

    (void)param;

    for (;;)
    {
        const TickType_t nowTick = xTaskGetTickCount();
        uint32_t dt_ms = (uint32_t)((nowTick - lastTelemetryTick) * portTICK_PERIOD_MS);
        hall_speed_state_t hall_snapshot;
        float speed_mps = 0.0f;
        float steering_angle_rad = 0.0f;
        float yaw_rate_rad_s = 0.0f;
        uint8_t status_flags = 0U;
        uint32_t status_bits = 0U;
        int32_t hall_delta_count;
        uint16_t battery_mv;
        uint32_t active_fault_sources = 0U;
        servo_basic_diagnostics_t servo_diagnostics;
        const uint8_t rc_override_active = ServoBasic_IsRcOverrideActive();
        const uint8_t estop_active = (ServoBasic_IsRcEmergencyActive() != 0U ||
            ServoBasic_IsOrinEmergencyActive() != 0U) ? 1U : 0U;
        const uint8_t command_timeout = ServoBasic_IsOrinCommandTimeout();
        const uint8_t brake_active = ServoBasic_IsOrinBrakeActive();
        const uint8_t auto_enabled = ServoBasic_IsOrinAutoEnabled();

        update_power_state();
        battery_mv = clamp_float_to_u16(g_app_runtime_state.voltage_v * 1000.0f);
        hall_snapshot = HallSpeed_GetState();
        hall_delta_count = get_signed_hall_delta(&hall_snapshot);
        if (dt_ms == 0U)
        {
            dt_ms = 1U;
        }
        lastTelemetryTick = nowTick;

        (void)ServoBasic_GetAckermannFeedback(&speed_mps, &steering_angle_rad, &yaw_rate_rad_s);
        servo_diagnostics = ServoBasic_GetDiagnostics();

        if (hall_snapshot.fault_count != 0U) { active_fault_sources |= APP_FAULT_SOURCE_HALL; }
        if (servo_diagnostics.steering_fault != 0U) { active_fault_sources |= APP_FAULT_SOURCE_STEERING; }
        if (battery_mv_is_low(battery_mv) != 0U) { active_fault_sources |= APP_FAULT_SOURCE_BATTERY_LOW; }
        if (battery_mv_is_critical(battery_mv) != 0U) { active_fault_sources |= APP_FAULT_SOURCE_BATTERY_CRITICAL; }
        if (g_app_runtime_state.uart4_rx_frame_error_seen != 0U) { active_fault_sources |= APP_FAULT_SOURCE_FRAME_ERROR; }
        AppRuntime_UpdateFaultSources(active_fault_sources);

        if (auto_enabled != 0U) { status_flags |= TELEMETRY_FLAG_AUTO_ENABLED; }
        if (rc_override_active != 0U) { status_flags |= TELEMETRY_FLAG_RC_OVERRIDE_ACTIVE; }
        if (estop_active != 0U) { status_flags |= TELEMETRY_FLAG_ESTOP_ACTIVE; }
        if (command_timeout != 0U) { status_flags |= TELEMETRY_FLAG_COMMAND_TIMEOUT; }
        if (brake_active != 0U) { status_flags |= TELEMETRY_FLAG_BRAKE_ACTIVE; }
        if (g_app_runtime_state.fault_latched != 0U) { status_flags |= TELEMETRY_FLAG_FAULT_LATCHED; }

        if (g_app_runtime_state.fault_latched != 0U) { status_bits |= STATUS_BIT_FAULT_LATCHED; }
        if (command_timeout != 0U) { status_bits |= STATUS_BIT_COMMAND_TIMEOUT; }
        if (rc_override_active != 0U) { status_bits |= STATUS_BIT_RC_OVERRIDE_ACTIVE; }
        if (estop_active != 0U) { status_bits |= STATUS_BIT_ESTOP_ACTIVE; }
        if (brake_active != 0U) { status_bits |= STATUS_BIT_BRAKE_ACTIVE; }
        if (auto_enabled != 0U) { status_bits |= STATUS_BIT_AUTO_ENABLED; }
        if (hall_snapshot.speed_valid != 0U) { status_bits |= STATUS_BIT_HALL_FEEDBACK_VALID; }
        if (hall_snapshot.fault_count != 0U) { status_bits |= STATUS_BIT_HALL_FAULT; }
        status_bits |= STATUS_BIT_STEERING_FEEDBACK_VALID;
        if (servo_diagnostics.steering_fault != 0U) { status_bits |= STATUS_BIT_STEERING_FAULT; }
        if (g_app_runtime_state.voltage_v > 0.1f) { status_bits |= STATUS_BIT_BATTERY_VALID; }
        if (battery_mv_is_low(battery_mv) != 0U) { status_bits |= STATUS_BIT_BATTERY_LOW; }
        if (battery_mv_is_critical(battery_mv) != 0U) { status_bits |= STATUS_BIT_BATTERY_CRITICAL; }
        if (servo_diagnostics.speed_saturated != 0U) { status_bits |= STATUS_BIT_SPEED_SATURATED; }
        if (servo_diagnostics.steering_saturated != 0U) { status_bits |= STATUS_BIT_STEERING_SATURATED; }
        if (servo_diagnostics.accel_limited != 0U) { status_bits |= STATUS_BIT_ACCEL_LIMITED; }
        if (servo_diagnostics.steering_rate_limited != 0U) { status_bits |= STATUS_BIT_STEERING_RATE_LIMITED; }
        if (g_app_runtime_state.uart4_rx_frame_error_seen != 0U) { status_bits |= STATUS_BIT_FRAME_ERROR_SEEN; }

        basebuffer[0] = BaseFRAME_HEAD;
        basebuffer[1] = status_flags;
        basebuffer[2] = seq++;
        write_i32_be(&basebuffer[3], hall_delta_count);
        write_i16_be(&basebuffer[7], clamp_float_to_i16(speed_mps * 1000.0f));
        write_i16_be(&basebuffer[9], clamp_float_to_i16(steering_angle_rad * 1000.0f));
        write_i16_be(&basebuffer[11], clamp_float_to_i16(yaw_rate_rad_s * 1000.0f));
        write_u16_be(&basebuffer[13], battery_mv);
        write_u16_be(&basebuffer[15], (dt_ms > 65535U) ? 65535U : (uint16_t)dt_ms);
        write_u32_be(&basebuffer[17], status_bits);
        basebuffer[21] = 0U;
        basebuffer[22] = Calculate_BCC(basebuffer, 22U);
        basebuffer[23] = BaseFRAME_TAIL;

        record_uart_tx_status(
            HAL_UART_Transmit_DMA(serial, basebuffer, BaseFRAME_LEN),
            &g_app_runtime_state.uart4_tx_busy_count,
            &g_app_runtime_state.uart4_tx_error_count);

        if (g_app_runtime_state.debug_level == 0U)
        {
            record_uart_tx_status(
                HAL_UART_Transmit_DMA(&huart1, basebuffer, BaseFRAME_LEN),
                &g_app_runtime_state.usart1_debug_tx_busy_count,
                &g_app_runtime_state.usart1_debug_tx_error_count);
        }

        vTaskDelayUntil(&preTime, pdMS_TO_TICKS((1000.0f / (float)TaskFreq)));
    }
}
