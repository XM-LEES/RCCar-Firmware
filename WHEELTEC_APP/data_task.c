#include "usart.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
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
#define TELEMETRY_PROTOCOL_ID 0xA1U

#define TELEMETRY_FLAG_AUTO_ENABLED        0x01U
#define TELEMETRY_FLAG_RC_OVERRIDE_ACTIVE  0x02U
#define TELEMETRY_FLAG_STOP_OVERRIDE_ACTIVE 0x04U
#define TELEMETRY_FLAG_COMMAND_TIMEOUT     0x08U
#define TELEMETRY_FLAG_BRAKE_ACTIVE        0x10U
#define TELEMETRY_FLAG_FAULT_LATCHED       0x20U
#define TELEMETRY_FLAG_ESC_ACTION_SHIFT         6U
#define TELEMETRY_FLAG_ESC_ACTION_MASK       0xC0U

#define STATUS_BIT_FAULT_LATCHED           (1UL << 0)
#define STATUS_BIT_COMMAND_TIMEOUT         (1UL << 1)
#define STATUS_BIT_RC_OVERRIDE_ACTIVE      (1UL << 2)
#define STATUS_BIT_STOP_OVERRIDE_ACTIVE    (1UL << 3)
#define STATUS_BIT_BRAKE_ACTIVE            (1UL << 4)
#define STATUS_BIT_AUTO_ENABLED            (1UL << 5)
#define STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID (1UL << 6)
#define STATUS_BIT_HALL_FAULT              (1UL << 7)
#define STATUS_BIT_STEERING_ESTIMATE_VALID (1UL << 8)
#define STATUS_BIT_STEERING_IS_MEASURED    (1UL << 9)
#define STATUS_BIT_RC_INPUT_FAULT          (1UL << 10)
#define STATUS_BIT_BATTERY_VALID           (1UL << 11)
#define STATUS_BIT_HALL_STANDSTILL_CONFIRMED (1UL << 12)
#define STATUS_BIT_HALL_SPEED_VALID       (1UL << 13)
#define STATUS_BIT_SPEED_SATURATED         (1UL << 14)
#define STATUS_BIT_STEERING_SATURATED      (1UL << 15)
#define STATUS_BIT_ACCEL_LIMITED           (1UL << 16)
#define STATUS_BIT_STEERING_RATE_LIMITED   (1UL << 17)
#define STATUS_BIT_FRAME_ERROR_SEEN        (1UL << 18)
#define STATUS_BIT_HALL_STOP_CONFIRMED     (1UL << 19)
#define STATUS_BIT_ESC_FE32_FRESH          (1UL << 20)
#define STATUS_BIT_ESC_RPM_RAW_VALID       (1UL << 21)
#define STATUS_BIT_ESC_SPEED_CALIBRATION_VALID (1UL << 22)
#define STATUS_BIT_VEHICLE_DIRECTION_KNOWN (1UL << 23)
#define STATUS_BIT_ESC_SOFT_UART_RX_ERROR  (1UL << 24)
#define STATUS_BIT_AUTO_PROPULSION_AUTHORIZED (1UL << 25)
#define STATUS_BIT_CLOSED_LOOP_ACTIVE      (1UL << 26)
#define STATUS_BIT_TRACKING_BRAKE_ACTIVE   (1UL << 27)
#define STATUS_BIT_MODE2_OPPOSITE_ARMED    (1UL << 28)
#define STATUS_BIT_MODE2_STATE_AMBIGUOUS   (1UL << 29)
#define STATUS_BIT_MODE2_CONTROL_INHIBITED (1UL << 30)
#define STATUS_BIT_MODE2_CONFIG_VALID      (1UL << 31)

#if BaseFRAME_LEN != 24U
#error "UART4 ROS telemetry frame must remain 24 bytes for the upper computer parser."
#endif

static void update_power_state(void)
{
    g_app_runtime_state.voltage_v = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL) / 4095.0f * 3.3f * 11.0f;
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

void RobotDataTransmitTask(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    TickType_t lastTelemetryTick = preTime;
    const uint16_t TaskFreq = 20U;
    uint8_t basebuffer[BaseFRAME_LEN];
    uint8_t seq = 0U;
    uint32_t handled_clear_request_count = 0U;

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
        float hall_speed_mps = 0.0f;
        uint8_t hall_speed_valid;
        uint16_t esc_pwm_us = ESC_PWM_NEUTRAL_PULSE_US;
        uint8_t esc_speed_magnitude_valid = 0U;
        uint8_t esc_direction_known = 0U;
        uint32_t active_fault_sources = 0U;
        uint8_t clear_fault_requested = 0U;
        servo_basic_control_snapshot_t control_snapshot;
        servo_basic_diagnostics_t servo_diagnostics;
        uint8_t rc_override_active = 0U;
        uint8_t stop_override_active = 0U;
        uint8_t command_timeout = 0U;
        uint8_t brake_active = 0U;
        uint8_t auto_enabled = 0U;

        const uint32_t clear_request_count = AppRuntime_GetFaultClearRequestCount();
        if (clear_request_count != handled_clear_request_count)
        {
            HallSpeed_ClearFaultCount();
            g_app_runtime_state.uart4_rx_frame_error_seen = 0U;
            handled_clear_request_count = clear_request_count;
            clear_fault_requested = 1U;
        }

        update_power_state();
        hall_snapshot = HallSpeed_GetState();
        hall_speed_valid = HallSpeed_GetSnapshotSpeedMps(&hall_snapshot, &hall_speed_mps);
        if (hall_snapshot.stationary_confirmed != 0U)
        {
            hall_speed_mps = 0.0f;
            status_bits |= STATUS_BIT_HALL_STOP_CONFIRMED;
        }
        else if (hall_speed_valid != 0U)
        {
            status_bits |= STATUS_BIT_HALL_SPEED_VALID;
        }
        if (dt_ms == 0U)
        {
            dt_ms = 1U;
        }
        lastTelemetryTick = nowTick;

        if (ServoBasic_GetControlSnapshot(&control_snapshot) != 0U)
        {
            servo_diagnostics = control_snapshot.diagnostics;
            esc_pwm_us = control_snapshot.state.esc_pulse_us;
            esc_speed_magnitude_valid = control_snapshot.esc_uplink_speed_valid;
            esc_direction_known = control_snapshot.esc_direction_known;
            speed_mps = control_snapshot.esc_uplink_speed_mps;
            steering_angle_rad = control_snapshot.steering_angle_rad;
            yaw_rate_rad_s = (control_snapshot.signed_speed_valid != 0U) ?
                control_snapshot.yaw_rate_rad_s : 0.0f;
            rc_override_active = control_snapshot.rc_override_active;
            stop_override_active = control_snapshot.orin_emergency_active;
            command_timeout = control_snapshot.orin_command_timeout;
            brake_active = control_snapshot.orin_brake_active;
            auto_enabled = control_snapshot.orin_auto_enabled;
        }
        else
        {
            memset(&servo_diagnostics, 0, sizeof(servo_diagnostics));
            esc_speed_magnitude_valid = 0U;
            esc_direction_known = 0U;
        }
        if (servo_diagnostics.esc_stop_confirmed != 0U)
        {
            speed_mps = 0.0f;
            yaw_rate_rad_s = 0.0f;
            esc_speed_magnitude_valid = 0U;
            esc_direction_known = 0U;
        }
        if (hall_snapshot.fault_count != 0U) { active_fault_sources |= APP_FAULT_SOURCE_HALL; }
        if (servo_diagnostics.steering_fault != 0U) { active_fault_sources |= APP_FAULT_SOURCE_STEERING; }
        if (g_app_runtime_state.uart4_rx_frame_error_seen != 0U) { active_fault_sources |= APP_FAULT_SOURCE_FRAME_ERROR; }
        AppRuntime_UpdateFaultSources(active_fault_sources);
        if (clear_fault_requested != 0U)
        {
            AppRuntime_TryClearFaultLatch();
        }

        if (auto_enabled != 0U) { status_flags |= TELEMETRY_FLAG_AUTO_ENABLED; }
        if (rc_override_active != 0U) { status_flags |= TELEMETRY_FLAG_RC_OVERRIDE_ACTIVE; }
        if (stop_override_active != 0U) { status_flags |= TELEMETRY_FLAG_STOP_OVERRIDE_ACTIVE; }
        if (command_timeout != 0U) { status_flags |= TELEMETRY_FLAG_COMMAND_TIMEOUT; }
        if (brake_active != 0U) { status_flags |= TELEMETRY_FLAG_BRAKE_ACTIVE; }
        if (g_app_runtime_state.fault_latched != 0U) { status_flags |= TELEMETRY_FLAG_FAULT_LATCHED; }
        status_flags |= (uint8_t)(((uint8_t)servo_diagnostics.esc_action <<
            TELEMETRY_FLAG_ESC_ACTION_SHIFT) & TELEMETRY_FLAG_ESC_ACTION_MASK);

        if (g_app_runtime_state.fault_latched != 0U) { status_bits |= STATUS_BIT_FAULT_LATCHED; }
        if (command_timeout != 0U) { status_bits |= STATUS_BIT_COMMAND_TIMEOUT; }
        if (rc_override_active != 0U) { status_bits |= STATUS_BIT_RC_OVERRIDE_ACTIVE; }
        if (stop_override_active != 0U) { status_bits |= STATUS_BIT_STOP_OVERRIDE_ACTIVE; }
        if (brake_active != 0U) { status_bits |= STATUS_BIT_BRAKE_ACTIVE; }
        if (auto_enabled != 0U) { status_bits |= STATUS_BIT_AUTO_ENABLED; }
        if (servo_diagnostics.esc_stop_confirmed != 0U)
        {
            status_bits |= STATUS_BIT_HALL_STANDSTILL_CONFIRMED;
        }
        else if (esc_speed_magnitude_valid != 0U)
        {
            status_bits |= STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID;
        }
        if (hall_snapshot.fault_count != 0U) { status_bits |= STATUS_BIT_HALL_FAULT; }
        status_bits |= STATUS_BIT_STEERING_ESTIMATE_VALID;
        if (servo_diagnostics.steering_fault != 0U) { status_bits |= STATUS_BIT_RC_INPUT_FAULT; }
        if (g_app_runtime_state.voltage_v > 0.1f) { status_bits |= STATUS_BIT_BATTERY_VALID; }
        if (servo_diagnostics.speed_saturated != 0U) { status_bits |= STATUS_BIT_SPEED_SATURATED; }
        if (servo_diagnostics.steering_saturated != 0U) { status_bits |= STATUS_BIT_STEERING_SATURATED; }
        if (servo_diagnostics.accel_limited != 0U) { status_bits |= STATUS_BIT_ACCEL_LIMITED; }
        if (servo_diagnostics.steering_rate_limited != 0U) { status_bits |= STATUS_BIT_STEERING_RATE_LIMITED; }
        if (g_app_runtime_state.uart4_rx_frame_error_seen != 0U) { status_bits |= STATUS_BIT_FRAME_ERROR_SEEN; }
        if (servo_diagnostics.esc_fe32_fresh != 0U) { status_bits |= STATUS_BIT_ESC_FE32_FRESH; }
        if (servo_diagnostics.esc_rpm_raw_valid != 0U) { status_bits |= STATUS_BIT_ESC_RPM_RAW_VALID; }
        if (servo_diagnostics.esc_speed_calibration_valid != 0U) { status_bits |= STATUS_BIT_ESC_SPEED_CALIBRATION_VALID; }
        if (esc_direction_known != 0U) { status_bits |= STATUS_BIT_VEHICLE_DIRECTION_KNOWN; }
        if (servo_diagnostics.esc_soft_uart_rx_error != 0U) { status_bits |= STATUS_BIT_ESC_SOFT_UART_RX_ERROR; }
        if (servo_diagnostics.auto_propulsion_authorized != 0U) { status_bits |= STATUS_BIT_AUTO_PROPULSION_AUTHORIZED; }
        if (servo_diagnostics.closed_loop_active != 0U) { status_bits |= STATUS_BIT_CLOSED_LOOP_ACTIVE; }
        if (servo_diagnostics.tracking_brake_active != 0U) { status_bits |= STATUS_BIT_TRACKING_BRAKE_ACTIVE; }
        if (servo_diagnostics.mode2_opposite_armed != 0U) { status_bits |= STATUS_BIT_MODE2_OPPOSITE_ARMED; }
        if (servo_diagnostics.mode2_state_ambiguous != 0U) { status_bits |= STATUS_BIT_MODE2_STATE_AMBIGUOUS; }
        if (servo_diagnostics.mode2_control_inhibited != 0U) { status_bits |= STATUS_BIT_MODE2_CONTROL_INHIBITED; }
        if (servo_diagnostics.mode2_config_valid != 0U) { status_bits |= STATUS_BIT_MODE2_CONFIG_VALID; }

        basebuffer[0] = BaseFRAME_HEAD;
        basebuffer[1] = status_flags;
        basebuffer[2] = seq++;
        /* Hall speed is independent of ESC calibration and direction validity. */
        write_i32_be(&basebuffer[3], (int32_t)(hall_speed_mps * 1000.0f));
        write_i16_be(&basebuffer[7], clamp_float_to_i16(speed_mps * 1000.0f));
        write_i16_be(&basebuffer[9], clamp_float_to_i16(steering_angle_rad * 1000.0f));
        write_i16_be(&basebuffer[11], clamp_float_to_i16(yaw_rate_rad_s * 1000.0f));
        write_u16_be(&basebuffer[13], esc_pwm_us);
        write_u16_be(&basebuffer[15], (dt_ms > 65535U) ? 65535U : (uint16_t)dt_ms);
        write_u32_be(&basebuffer[17], status_bits);
        basebuffer[21] = TELEMETRY_PROTOCOL_ID;
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
