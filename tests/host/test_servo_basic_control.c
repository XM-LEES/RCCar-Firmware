#include "servo_basic_control.h"
#include "app_runtime_state.h"
#include "app_vehicle_config.h"
#include "esc_telemetry.h"
#include "queue.h"
#include "hall_speed.h"
#include "usart.h"

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ROS_CMD_FRAME_LEN 11U
#define ROS_CMD_ACKERMANN 0x01U
#define ROS_CMD_FLAG_ENABLE 0x01U
#define ROS_CMD_FLAG_BRAKE 0x02U
#define ROS_CMD_FLAG_SOFTWARE_STOP 0x80U
#define BASE_TELEMETRY_FRAME_LEN 24U
#define TELEMETRY_FLAG_AUTO_ENABLED 0x01U
#define TELEMETRY_FLAG_RC_OVERRIDE_ACTIVE 0x02U
#define TELEMETRY_FLAG_STOP_OVERRIDE_ACTIVE 0x04U
#define TELEMETRY_FLAG_COMMAND_TIMEOUT 0x08U
#define TELEMETRY_FLAG_BRAKE_ACTIVE 0x10U
#define STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID (1UL << 6)
#define STATUS_BIT_ESC_STOP_CONFIRMED (1UL << 12)
#define STATUS_BIT_ESC_FE32_FRESH (1UL << 20)
#define STATUS_BIT_ESC_RPM_RAW_VALID (1UL << 21)
#define STATUS_BIT_ESC_SPEED_CALIBRATION_VALID (1UL << 22)
#define STATUS_BIT_VEHICLE_DIRECTION_KNOWN (1UL << 23)
#define STATUS_BIT_ESC_SOFT_UART_RX_ERROR (1UL << 24)
#define STATUS_BIT_AUTO_PROPULSION_AUTHORIZED (1UL << 25)
#define STATUS_BIT_CLOSED_LOOP_ACTIVE (1UL << 26)
#define STATUS_BIT_TRACKING_BRAKE_ACTIVE (1UL << 27)
#define STATUS_BIT_MODE2_OPPOSITE_ARMED (1UL << 28)
#define STATUS_BIT_MODE2_STATE_AMBIGUOUS (1UL << 29)
#define STATUS_BIT_MODE2_CONTROL_INHIBITED (1UL << 30)
#define STATUS_BIT_MODE2_CONFIG_VALID (1UL << 31)

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

#define EXPECT_EQ_U16(actual, expected) \
    do { \
        const uint16_t actual_value__ = (uint16_t)(actual); \
        const uint16_t expected_value__ = (uint16_t)(expected); \
        if (actual_value__ != expected_value__) { \
            fprintf(stderr, "%s:%d: expected %s=%u, got %u\n", \
                    __FILE__, __LINE__, #actual, \
                    (unsigned int)expected_value__, \
                    (unsigned int)actual_value__); \
            return 1; \
        } \
    } while (0)

#define EXPECT_EQ_I32(actual, expected) \
    do { \
        const int32_t actual_value__ = (int32_t)(actual); \
        const int32_t expected_value__ = (int32_t)(expected); \
        if (actual_value__ != expected_value__) { \
            fprintf(stderr, "%s:%d: expected %s=%ld, got %ld\n", \
                    __FILE__, __LINE__, #actual, \
                    (long)expected_value__, (long)actual_value__); \
            return 1; \
        } \
    } while (0)

typedef struct
{
    uint16_t throttle_us;
    uint16_t steering_us;
    uint16_t aux_us;
    uint8_t throttle_active;
    uint8_t steering_active;
    uint8_t aux_active;
    uint8_t throttle_fault;
    uint8_t steering_fault;
    uint8_t aux_fault;
} HostRcState_t;

extern volatile uint32_t g_rc_signal_timeout_ms;
extern volatile uint32_t g_rc_debounce_deadband_us;
extern volatile uint32_t g_rc_debounce_smooth_div;
extern volatile uint32_t g_rc_valid_min_us;
extern volatile uint32_t g_rc_valid_max_us;
extern volatile uint32_t g_rc_frame_min_us;
extern volatile uint32_t g_rc_frame_max_us;
extern volatile uint32_t g_rc_glitch_freeze_ms;
extern volatile uint32_t g_rc_throttle_neutral_hold_us;
extern volatile uint32_t g_rc_throttle_jump_confirm_us;
extern volatile uint32_t g_rc_steering_jump_confirm_us;
extern volatile uint32_t g_rc_jump_confirm_samples;
extern volatile uint32_t g_rc_override_center_us;
extern volatile uint32_t g_rc_override_enter_threshold_us;
extern volatile uint32_t g_rc_override_exit_threshold_us;
extern volatile uint32_t g_rc_override_enter_samples;
extern volatile uint32_t g_rc_override_release_hold_ms;
extern volatile uint32_t g_rc_aux_pulse_us;
extern volatile uint32_t g_rc_aux_present;
extern volatile uint32_t g_rc_aux_fault;
extern volatile uint32_t g_orin_pwm_timeout_ms;
extern volatile uint32_t g_orin_ackermann_wheelbase_mm;
extern volatile uint32_t g_orin_ackermann_track_width_mm;
extern volatile uint32_t g_orin_ackermann_wheel_radius_mm;
extern volatile uint32_t g_orin_ackermann_max_steering_millirad;
extern volatile uint32_t g_orin_esc_center_us;
extern volatile uint32_t g_orin_esc_forward_max_us;
extern volatile uint32_t g_orin_esc_reverse_max_us;
extern volatile uint32_t g_orin_servo_center_us;
extern volatile uint32_t g_orin_servo_range_us;
extern volatile float g_esc_tracking_brake_kp;
extern volatile float g_esc_tracking_brake_max;
extern volatile float g_esc_tracking_brake_enter_error_mps;
extern volatile float g_esc_tracking_brake_release_error_mps;
extern volatile float g_esc_motion_stopped_speed_threshold_mps;
extern volatile uint32_t g_esc_motion_stopped_min_samples;
extern volatile uint32_t g_esc_motion_stopped_min_coverage_ms;
extern volatile float g_esc_low_gear_wheel_rpm_per_raw;
extern volatile uint32_t g_esc_speed_fresh_timeout_ms;
extern volatile float g_mode2_fwd_to_rev_brake_request;
extern volatile float g_mode2_rev_to_fwd_brake_request;
extern volatile uint32_t g_mode2_fwd_to_rev_brake_hold_ms;
extern volatile uint32_t g_mode2_rev_to_fwd_brake_hold_ms;
extern volatile uint32_t g_mode2_drive_neutral_dwell_ms;
extern volatile uint32_t g_mode2_fwd_to_rev_qualify_delta_us;
extern volatile uint32_t g_mode2_rev_to_fwd_qualify_delta_us;
extern volatile uint32_t g_mode2_fwd_to_rev_brake_full_pwm_us;
extern volatile uint32_t g_mode2_rev_to_fwd_brake_full_pwm_us;
extern volatile uint32_t g_orin_accel_limit_mmps2;
extern volatile uint32_t g_orin_steering_rate_limit_mradps;
extern volatile uint32_t g_speed_pi_enable;
extern volatile float g_speed_pi_kp;
extern volatile float g_speed_pi_ki;
extern volatile uint32_t g_speed_pi_trim_limit_us;
extern volatile int32_t g_speed_pi_trim_us;
extern volatile int32_t g_speed_pi_final_us;
extern volatile float g_speed_pi_integral;
extern volatile uint32_t g_speed_pi_saturated;

QueueHandle_t g_xQueueROSserial = (QueueHandle_t)1;
volatile hall_speed_state_t g_hall_speed_state;
volatile uint32_t g_rc_capture_active_high = 1U;

static HostRcState_t s_rc;
static uint32_t s_fake_tick_ms;
static uint16_t s_last_esc_pulse;
static uint16_t s_last_servo_pulse;
static uint32_t s_esc_output_count;
static uint32_t s_servo_output_count;
static uint8_t s_hall_speed_valid;
static float s_hall_speed_mps;
static int8_t s_last_hall_command_direction;
static const uint8_t *s_queue_bytes;
static size_t s_queue_len;
static size_t s_queue_pos;
static jmp_buf s_queue_done;
static uint8_t s_queue_done_armed;
static EscTelemetrySnapshot_t s_esc_snapshot;
static uint8_t s_esc_snapshot_present;
static EscTelemetryDiagnostics_t s_esc_diagnostics;
static uint8_t s_base_telemetry_frame[BASE_TELEMETRY_FRAME_LEN];
static uint16_t s_base_telemetry_len;
static jmp_buf s_transmit_done;
static uint8_t s_transmit_done_armed;
static uint16_t s_fake_adc_raw;

UART_HandleTypeDef huart1 = {1U};
UART_HandleTypeDef huart4 = {4U};

extern uint8_t Calculate_BCC(const uint8_t *checkdata, uint16_t datalen);
void SerialControlTask(void *param);
void RobotDataTransmitTask(void *param);

uint32_t HAL_GetTick(void)
{
    return s_fake_tick_ms;
}

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)s_fake_tick_ms;
}

void vTaskDelayUntil(TickType_t *previous_wake_time, TickType_t time_increment)
{
    if (previous_wake_time != NULL)
    {
        *previous_wake_time += time_increment;
    }
}

void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

BaseType_t xQueueReceive(QueueHandle_t queue,
                         void *buffer,
                         TickType_t ticks_to_wait)
{
    (void)queue;
    (void)ticks_to_wait;

    if (s_queue_pos < s_queue_len)
    {
        *(uint8_t *)buffer = s_queue_bytes[s_queue_pos++];
        return pdPASS;
    }

    if (s_queue_done_armed != 0U)
    {
        longjmp(s_queue_done, 1);
    }
    return pdFAIL;
}

void ServoBasic_OutputEscPulse(uint16_t pulse_us)
{
    s_last_esc_pulse = pulse_us;
    s_esc_output_count++;
}

void ServoBasic_OutputServoPulse(uint16_t pulse_us)
{
    s_last_servo_pulse = pulse_us;
    s_servo_output_count++;
}

uint8_t EscTelemetry_GetSnapshot(EscTelemetrySnapshot_t *snapshot)
{
    if (snapshot == NULL || s_esc_snapshot_present == 0U)
    {
        return 0U;
    }

    *snapshot = s_esc_snapshot;
    return 1U;
}

void EscTelemetry_GetDiagnostics(EscTelemetryDiagnostics_t *diagnostics)
{
    if (diagnostics != NULL)
    {
        *diagnostics = s_esc_diagnostics;
    }
}

void EscTelemetry_GetReceiverHealth(EscTelemetryReceiverHealth_t *health)
{
    if (health != NULL)
    {
        health->samples_published = s_esc_diagnostics.samples_published;
        health->rx_error_count = s_esc_diagnostics.rx_error_count;
        health->last_rx_error_flags = s_esc_diagnostics.last_rx_error_flags;
    }
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *data,
                                        uint16_t size)
{
    if (huart == &huart4 && size == BASE_TELEMETRY_FRAME_LEN)
    {
        memcpy(s_base_telemetry_frame, data, size);
        s_base_telemetry_len = size;
        if (s_transmit_done_armed != 0U)
        {
            longjmp(s_transmit_done, 1);
        }
    }

    return HAL_OK;
}

uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
    (void)channel;
    return s_fake_adc_raw;
}

void ServoRC_Capture_Init(void)
{
}

uint16_t ServoRC_GetThrottlePulse(void)
{
    return s_rc.throttle_us;
}

uint16_t ServoRC_GetSteeringPulse(void)
{
    return s_rc.steering_us;
}

uint16_t ServoRC_GetAuxPulse(void)
{
    return s_rc.aux_us;
}

uint8_t ServoRC_IsThrottleActive(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return s_rc.throttle_active;
}

uint8_t ServoRC_IsSteeringActive(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return s_rc.steering_active;
}

uint8_t ServoRC_IsAuxActive(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return s_rc.aux_active;
}

uint8_t ServoRC_HasThrottleFault(void)
{
    return s_rc.throttle_fault;
}

uint8_t ServoRC_HasSteeringFault(void)
{
    return s_rc.steering_fault;
}

uint8_t ServoRC_HasAuxFault(void)
{
    return s_rc.aux_fault;
}

void HallSpeed_Init(void)
{
    memset((void *)&g_hall_speed_state, 0, sizeof(g_hall_speed_state));
    s_last_hall_command_direction = 0;
}

void HallSpeed_SetCommandDirection(int8_t direction)
{
    s_last_hall_command_direction = direction;
    g_hall_speed_state.command_direction = direction;
}

void HallSpeed_OnCountEvent(void)
{
}

void HallSpeed_ClearFaultCount(void)
{
    g_hall_speed_state.fault_count = 0U;
}

uint8_t HallSpeed_GetSignedSpeedMps(float *speed_mps)
{
    if (speed_mps != NULL)
    {
        *speed_mps = s_hall_speed_mps;
    }
    return s_hall_speed_valid;
}

hall_speed_state_t HallSpeed_GetState(void)
{
    return g_hall_speed_state;
}

uint8_t HallSpeed_GetSnapshotSpeedMps(const hall_speed_state_t *snapshot, float *speed_mps)
{
    (void)snapshot;
    *speed_mps = (s_hall_speed_valid != 0U) ? s_hall_speed_mps : 0.0f;
    return s_hall_speed_valid;
}

static void set_rc(uint16_t throttle_us,
                   uint16_t steering_us,
                   uint8_t active)
{
    s_rc.throttle_us = throttle_us;
    s_rc.steering_us = steering_us;
    s_rc.throttle_active = active;
    s_rc.steering_active = active;
}

static void reset_outputs(void)
{
    s_last_esc_pulse = 0U;
    s_last_servo_pulse = 0U;
    s_esc_output_count = 0U;
    s_servo_output_count = 0U;
}

static void reset_tunables_to_defaults(void)
{
    g_rc_signal_timeout_ms = APP_RC_SIGNAL_TIMEOUT_MS;
    g_rc_debounce_deadband_us = APP_RC_DEBOUNCE_DEADBAND_US;
    g_rc_debounce_smooth_div = APP_RC_DEBOUNCE_SMOOTH_DIV;
    g_rc_valid_min_us = APP_RC_VALID_MIN_US;
    g_rc_valid_max_us = APP_RC_VALID_MAX_US;
    g_rc_frame_min_us = APP_RC_FRAME_MIN_US;
    g_rc_frame_max_us = APP_RC_FRAME_MAX_US;
    g_rc_glitch_freeze_ms = APP_RC_GLITCH_FREEZE_MS;
    g_rc_throttle_neutral_hold_us = APP_RC_THROTTLE_NEUTRAL_HOLD_US;
    g_rc_throttle_jump_confirm_us = APP_RC_THROTTLE_JUMP_CONFIRM_US;
    g_rc_steering_jump_confirm_us = APP_RC_STEERING_JUMP_CONFIRM_US;
    g_rc_jump_confirm_samples = APP_RC_JUMP_CONFIRM_SAMPLES;
    g_rc_override_center_us = APP_RC_OVERRIDE_CENTER_US;
    g_rc_override_enter_threshold_us = APP_RC_OVERRIDE_ENTER_THRESHOLD_US;
    g_rc_override_exit_threshold_us = APP_RC_OVERRIDE_EXIT_THRESHOLD_US;
    g_rc_override_enter_samples = APP_RC_OVERRIDE_ENTER_SAMPLES;
    g_rc_override_release_hold_ms = APP_RC_OVERRIDE_RELEASE_HOLD_MS;
    g_rc_aux_pulse_us = 0U;
    g_rc_aux_present = 0U;
    g_rc_aux_fault = 0U;

    g_orin_pwm_timeout_ms = APP_ORIN_PWM_TIMEOUT_DEFAULT_MS;
    g_orin_ackermann_wheelbase_mm = APP_ORIN_ACKERMANN_WHEELBASE_MM;
    g_orin_ackermann_track_width_mm = APP_ORIN_ACKERMANN_TRACK_WIDTH_MM;
    g_orin_ackermann_wheel_radius_mm = APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM;
    g_orin_ackermann_max_steering_millirad = APP_ORIN_ACKERMANN_MAX_STEERING_MRAD;
    g_orin_esc_center_us = APP_ORIN_ESC_CENTER_US;
    g_orin_esc_forward_max_us = APP_ORIN_ESC_FORWARD_MAX_US;
    g_orin_esc_reverse_max_us = APP_ORIN_ESC_REVERSE_MAX_US;
    g_orin_servo_center_us = APP_ORIN_SERVO_CENTER_US;
    g_orin_servo_range_us = APP_ORIN_SERVO_RANGE_US;

    g_esc_tracking_brake_kp = APP_ESC_TRACKING_BRAKE_KP_DEFAULT;
    g_esc_tracking_brake_max = APP_ESC_TRACKING_BRAKE_MAX_DEFAULT;
    g_esc_tracking_brake_enter_error_mps = APP_ESC_TRACKING_BRAKE_ENTER_ERROR_MPS_DEFAULT;
    g_esc_tracking_brake_release_error_mps = APP_ESC_TRACKING_BRAKE_RELEASE_ERROR_MPS_DEFAULT;
    g_esc_motion_stopped_speed_threshold_mps = APP_ESC_STOPPED_THRESHOLD_MPS_DEFAULT;
    g_esc_motion_stopped_min_samples = APP_ESC_STOPPED_MIN_SAMPLES_DEFAULT;
    g_esc_motion_stopped_min_coverage_ms = APP_ESC_STOPPED_MIN_COVERAGE_MS_DEFAULT;
    g_esc_low_gear_wheel_rpm_per_raw = APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT;
    g_esc_speed_fresh_timeout_ms = APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT;

    g_mode2_fwd_to_rev_brake_request = APP_MODE2_FWD_TO_REV_BRAKE_REQUEST_DEFAULT;
    g_mode2_rev_to_fwd_brake_request = APP_MODE2_REV_TO_FWD_BRAKE_REQUEST_DEFAULT;
    g_mode2_fwd_to_rev_brake_hold_ms = APP_MODE2_FWD_TO_REV_BRAKE_HOLD_MS_DEFAULT;
    g_mode2_rev_to_fwd_brake_hold_ms = APP_MODE2_REV_TO_FWD_BRAKE_HOLD_MS_DEFAULT;
    g_mode2_drive_neutral_dwell_ms = APP_MODE2_DRIVE_NEUTRAL_DWELL_MS_DEFAULT;
    g_mode2_fwd_to_rev_qualify_delta_us = APP_MODE2_FWD_TO_REV_QUALIFY_DELTA_US_DEFAULT;
    g_mode2_rev_to_fwd_qualify_delta_us = APP_MODE2_REV_TO_FWD_QUALIFY_DELTA_US_DEFAULT;
    g_mode2_fwd_to_rev_brake_full_pwm_us = APP_MODE2_FWD_TO_REV_BRAKE_FULL_PWM_US_DEFAULT;
    g_mode2_rev_to_fwd_brake_full_pwm_us = APP_MODE2_REV_TO_FWD_BRAKE_FULL_PWM_US_DEFAULT;

    g_orin_accel_limit_mmps2 = APP_ORIN_ACCEL_LIMIT_MMPS2;
    g_orin_steering_rate_limit_mradps = APP_ORIN_STEERING_RATE_LIMIT_MRADPS;
    g_speed_pi_enable = APP_SPEED_PI_ENABLE_DEFAULT;
    g_speed_pi_kp = APP_SPEED_PI_KP_DEFAULT_US_PER_MPS;
    g_speed_pi_ki = APP_SPEED_PI_KI_DEFAULT_US_PER_MPS_S;
    g_speed_pi_trim_limit_us = APP_SPEED_PI_TRIM_LIMIT_US;
}

static void reset_fixture(void)
{
    memset(&s_rc, 0, sizeof(s_rc));
    memset(&g_app_runtime_state, 0, sizeof(g_app_runtime_state));
    memset((void *)&g_hall_speed_state, 0, sizeof(g_hall_speed_state));
    s_fake_tick_ms = 1000U;
    s_hall_speed_valid = 0U;
    s_hall_speed_mps = 0.0f;
    s_last_hall_command_direction = 0;
    s_queue_bytes = NULL;
    s_queue_len = 0U;
    s_queue_pos = 0U;
    s_queue_done_armed = 0U;
    memset(&s_esc_snapshot, 0, sizeof(s_esc_snapshot));
    s_esc_snapshot_present = 0U;
    memset(&s_esc_diagnostics, 0, sizeof(s_esc_diagnostics));
    memset(s_base_telemetry_frame, 0, sizeof(s_base_telemetry_frame));
    s_base_telemetry_len = 0U;
    s_transmit_done_armed = 0U;
    s_fake_adc_raw = 1024U;
    reset_tunables_to_defaults();
    ServoBasic_Init();
    reset_outputs();
}

static void run_control_at(uint32_t tick_ms)
{
    s_fake_tick_ms = tick_ms;
    ServoBasic_ProcessControl();
}

static void enable_valid_esc_configs(void)
{
    g_esc_low_gear_wheel_rpm_per_raw = APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT;
    g_esc_speed_fresh_timeout_ms = 200U;
    g_esc_motion_stopped_speed_threshold_mps = 0.05f;
    g_esc_motion_stopped_min_samples = 2U;
    g_esc_motion_stopped_min_coverage_ms = 20U;
    g_esc_tracking_brake_kp = 0.50f;
    g_esc_tracking_brake_max = 0.70f;
    g_esc_tracking_brake_enter_error_mps = 0.20f;
    g_esc_tracking_brake_release_error_mps = 0.10f;

    g_mode2_fwd_to_rev_brake_request = 0.70f;
    g_mode2_rev_to_fwd_brake_request = 0.70f;
    g_mode2_fwd_to_rev_brake_hold_ms = 40U;
    g_mode2_rev_to_fwd_brake_hold_ms = 40U;
    g_mode2_drive_neutral_dwell_ms = 40U;
    g_mode2_fwd_to_rev_qualify_delta_us = 100U;
    g_mode2_rev_to_fwd_qualify_delta_us = 100U;
    g_mode2_fwd_to_rev_brake_full_pwm_us = ESC_PWM_MIN_PULSE_US;
    g_mode2_rev_to_fwd_brake_full_pwm_us = ESC_PWM_MAX_PULSE_US;
}

static void set_esc_sample(uint32_t epoch,
                           uint32_t sample_id,
                           uint32_t received_tick_ms,
                           uint32_t erpm_candidate,
                           uint8_t rpm_valid)
{
    memset(&s_esc_snapshot, 0, sizeof(s_esc_snapshot));
    s_esc_snapshot_present = 1U;
    s_esc_snapshot.has_sample = 1U;
    s_esc_snapshot.publish_sequence = sample_id;
    s_esc_snapshot.receive_epoch = epoch;
    s_esc_snapshot.sample.sample_id = sample_id;
    s_esc_snapshot.sample.received_tick_ms = received_tick_ms;
    s_esc_snapshot.sample.rpm_valid = rpm_valid;
    s_esc_snapshot.sample.erpm_candidate = erpm_candidate;
    s_esc_snapshot.sample.rpm_raw = (uint16_t)(erpm_candidate / 10UL);
    s_esc_diagnostics.samples_published = sample_id;
    s_esc_diagnostics.receive_epoch = epoch;
}

static void clear_esc_sample(void)
{
    memset(&s_esc_snapshot, 0, sizeof(s_esc_snapshot));
    s_esc_snapshot_present = 0U;
}

static void establish_fresh_stop(uint32_t epoch,
                                 uint32_t first_sample_id,
                                 uint32_t first_tick_ms)
{
    run_control_at(first_tick_ms - 20U);
    set_esc_sample(epoch, first_sample_id, first_tick_ms, 0U, 1U);
    run_control_at(first_tick_ms);
    set_esc_sample(epoch, first_sample_id + 1U, first_tick_ms + 20U, 0U, 1U);
    run_control_at(first_tick_ms + 20U);
}

static void run_data_task_once(uint32_t tick_ms)
{
    s_fake_tick_ms = tick_ms;
    if (setjmp(s_transmit_done) == 0)
    {
        s_transmit_done_armed = 1U;
        RobotDataTransmitTask(NULL);
    }
    s_transmit_done_armed = 0U;
}

static uint32_t read_u32_be_from_frame(size_t offset)
{
    return ((uint32_t)s_base_telemetry_frame[offset] << 24) |
        ((uint32_t)s_base_telemetry_frame[offset + 1U] << 16) |
        ((uint32_t)s_base_telemetry_frame[offset + 2U] << 8) |
        ((uint32_t)s_base_telemetry_frame[offset + 3U]);
}

static int32_t read_i32_be_from_frame(size_t offset)
{
    return (int32_t)read_u32_be_from_frame(offset);
}

static int16_t read_i16_be_from_frame(size_t offset)
{
    return (int16_t)(((uint16_t)s_base_telemetry_frame[offset] << 8) |
        (uint16_t)s_base_telemetry_frame[offset + 1U]);
}

static void send_serial_command(int16_t speed_mmps,
                                int16_t steering_mrad,
                                uint8_t flags)
{
    static uint8_t frame[ROS_CMD_FRAME_LEN];

    frame[0] = 0x7BU;
    frame[1] = ROS_CMD_ACKERMANN;
    frame[2] = flags;
    frame[3] = (uint8_t)((uint16_t)speed_mmps >> 8);
    frame[4] = (uint8_t)((uint16_t)speed_mmps);
    frame[5] = (uint8_t)((uint16_t)steering_mrad >> 8);
    frame[6] = (uint8_t)((uint16_t)steering_mrad);
    frame[7] = 0U;
    frame[8] = 0U;
    frame[9] = Calculate_BCC(frame, ROS_CMD_FRAME_LEN - 2U);
    frame[10] = 0x7DU;

    s_queue_bytes = frame;
    s_queue_len = sizeof(frame);
    s_queue_pos = 0U;

    if (setjmp(s_queue_done) == 0)
    {
        s_queue_done_armed = 1U;
        SerialControlTask(NULL);
    }
    s_queue_done_armed = 0U;
}

static int test_idle_and_software_stop_use_center_not_zero(void)
{
    reset_fixture();

    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    EXPECT_EQ_U16(s_last_servo_pulse, APP_ORIN_SERVO_CENTER_US);
    EXPECT_TRUE(s_last_esc_pulse != 0U);
    EXPECT_TRUE(s_last_servo_pulse != 0U);

    s_fake_tick_ms = 1010U;
    send_serial_command(1200, 200,
                        (uint8_t)(ROS_CMD_FLAG_ENABLE |
                                  ROS_CMD_FLAG_SOFTWARE_STOP));
    run_control_at(1010U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    EXPECT_EQ_U16(s_last_servo_pulse, APP_ORIN_SERVO_CENTER_US);
    EXPECT_TRUE(ServoBasic_IsOrinEmergencyActive() == 1U);

    return 0;
}

static int test_rc_priority_and_software_stop_is_serial_only(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    set_rc(1600U, 1580U, 1U);

    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1600U);
    EXPECT_EQ_U16(s_last_servo_pulse, 1580U);

    s_fake_tick_ms = 1010U;
    send_serial_command(1500, 250, ROS_CMD_FLAG_ENABLE);
    run_control_at(1010U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1600U);
    EXPECT_EQ_U16(s_last_servo_pulse, 1580U);

    s_fake_tick_ms = 1020U;
    send_serial_command(0, 0,
                        (uint8_t)(ROS_CMD_FLAG_ENABLE |
                                  ROS_CMD_FLAG_SOFTWARE_STOP));
    run_control_at(1020U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_TRUE(ServoBasic_IsOrinEmergencyActive() == 1U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1600U);
    EXPECT_EQ_U16(s_last_servo_pulse, 1580U);

    return 0;
}

static int test_nonzero_serial_during_rc_releases_to_zero_not_cached_motion(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    set_rc(1600U, 1580U, 1U);

    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);

    s_fake_tick_ms = 1010U;
    send_serial_command(2000, 250, ROS_CMD_FLAG_ENABLE);
    run_control_at(1010U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1600U);
    EXPECT_EQ_U16(s_last_servo_pulse, 1580U);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    s_fake_tick_ms = 1020U;
    send_serial_command(2000, 250, ROS_CMD_FLAG_ENABLE);
    run_control_at(1020U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);

    s_fake_tick_ms = 1510U;
    send_serial_command(2000, 250, ROS_CMD_FLAG_ENABLE);
    run_control_at(1510U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);

    s_fake_tick_ms = 2030U;
    send_serial_command(2000, 250, ROS_CMD_FLAG_ENABLE);
    run_control_at(2030U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 0U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    EXPECT_EQ_U16(s_last_servo_pulse, APP_ORIN_SERVO_CENTER_US);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    return 0;
}

static int test_rc_hall_direction_forward_brake_neutral_reverse(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    g_hall_speed_state.stationary_confirmed = 0U;
    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1020U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1400U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    g_hall_speed_state.stationary_confirmed = 1U;
    run_control_at(1040U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    run_control_at(1060U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1080U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1100U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1400U);
    EXPECT_EQ_I32(s_last_hall_command_direction, -1);

    return 0;
}

static int test_rc_hall_direction_reverse_brake_neutral_forward(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_EQ_I32(s_last_hall_command_direction, -1);

    g_hall_speed_state.stationary_confirmed = 0U;
    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1020U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1600U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    g_hall_speed_state.stationary_confirmed = 1U;
    run_control_at(1040U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1060U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1080U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1600U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    return 0;
}

static int test_rc_hall_direction_cancel_brake_keeps_current_direction(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    g_hall_speed_state.stationary_confirmed = 0U;
    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1020U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1040U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    return 0;
}

static int test_rc_hall_direction_requires_stop_before_neutral_unlock(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    g_hall_speed_state.stationary_confirmed = 0U;
    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1020U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1040U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    g_hall_speed_state.stationary_confirmed = 1U;
    run_control_at(1060U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1080U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    run_control_at(1100U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1120U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1140U);
    EXPECT_EQ_I32(s_last_hall_command_direction, -1);

    return 0;
}

static int test_rc_hall_direction_resets_when_rc_exits(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    set_rc(0U, 0U, 0U);
    run_control_at(1020U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 0U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1040U);
    EXPECT_TRUE(ServoBasic_IsRcOverrideActive() == 1U);
    EXPECT_EQ_I32(s_last_hall_command_direction, -1);

    return 0;
}

static int test_rc_hall_direction_renewed_motion_revokes_brake_stop(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1020U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    g_hall_speed_state.stationary_confirmed = 1U;
    run_control_at(1040U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    g_hall_speed_state.stationary_confirmed = 0U;
    run_control_at(1060U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    g_hall_speed_state.stationary_confirmed = 1U;
    run_control_at(1080U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1100U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    return 0;
}

static int test_rc_aux_does_not_change_partial_hall_reversal_sequence(void)
{
    reset_fixture();
    g_rc_debounce_deadband_us = 0U;
    g_rc_debounce_smooth_div = 1U;
    g_rc_throttle_jump_confirm_us = 1000U;

    set_rc(1600U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1000U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 1);

    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1020U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    g_hall_speed_state.stationary_confirmed = 1U;
    run_control_at(1040U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    s_rc.aux_us = 2000U;
    s_rc.aux_active = 1U;
    run_control_at(1060U);
    EXPECT_TRUE(g_rc_aux_present != 0U);
    EXPECT_TRUE(g_rc_aux_pulse_us == 2000U);

    set_rc(APP_RC_OVERRIDE_CENTER_US, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1080U);
    set_rc(1400U, APP_RC_OVERRIDE_CENTER_US, 1U);
    run_control_at(1100U);
    EXPECT_EQ_I32(s_last_hall_command_direction, -1);

    return 0;
}

static int test_steering_direction_limit_and_rate_limit(void)
{
    servo_basic_diagnostics_t diagnostics;

    reset_fixture();
    ServoBasic_SetSpeedPiEnable(0U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_servo_pulse, APP_ORIN_SERVO_CENTER_US);

    s_fake_tick_ms = 1100U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, 1.0f, 1U, 0U, 0U);
    run_control_at(1100U);
    EXPECT_TRUE(s_last_servo_pulse > APP_ORIN_SERVO_CENTER_US);
    EXPECT_TRUE(s_last_servo_pulse >= 1600U && s_last_servo_pulse <= 1602U);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.steering_saturated == 1U);
    EXPECT_TRUE(diagnostics.steering_rate_limited == 1U);

    s_fake_tick_ms = 2100U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, 1.0f, 1U, 0U, 0U);
    run_control_at(2100U);
    EXPECT_TRUE(s_last_servo_pulse > 1855U && s_last_servo_pulse < 1858U);

    s_fake_tick_ms = 2400U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, 1.0f, 1U, 0U, 0U);
    run_control_at(2400U);
    EXPECT_EQ_U16(s_last_servo_pulse,
                  (uint16_t)(APP_ORIN_SERVO_CENTER_US + APP_ORIN_SERVO_RANGE_US));

    s_fake_tick_ms = 3400U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, -1.0f, 1U, 0U, 0U);
    run_control_at(3400U);
    EXPECT_TRUE(s_last_servo_pulse > APP_ORIN_SERVO_CENTER_US);

    s_fake_tick_ms = 3700U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, -1.0f, 1U, 0U, 0U);
    run_control_at(3700U);
    EXPECT_TRUE(s_last_servo_pulse < APP_ORIN_SERVO_CENTER_US);

    s_fake_tick_ms = 4000U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, -1.0f, 1U, 0U, 0U);
    run_control_at(4000U);
    EXPECT_TRUE(s_last_servo_pulse < 1135U);

    s_fake_tick_ms = 4300U;
    ServoBasic_UpdateAckermannFromOrin(0.0f, -1.0f, 1U, 0U, 0U);
    run_control_at(4300U);
    EXPECT_EQ_U16(s_last_servo_pulse,
                  (uint16_t)(APP_ORIN_SERVO_CENTER_US - APP_ORIN_SERVO_RANGE_US));

    return 0;
}

static int test_missing_esc_or_mode2_parameters_keep_propulsion_off(void)
{
    servo_basic_diagnostics_t diagnostics;

    reset_fixture();
    g_mode2_fwd_to_rev_brake_request = 0.0f;
    set_esc_sample(1U, 1U, 1000U, 0U, 1U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);

    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    EXPECT_TRUE(ServoBasic_IsOrinAutoEnabled() == 1U);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.esc_motion_config_valid == 1U);
    EXPECT_TRUE(diagnostics.mode2_config_valid == 0U);

    reset_fixture();
    enable_valid_esc_configs();
    g_esc_tracking_brake_kp = 0.0f;
    establish_fresh_stop(1U, 1U, 960U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.esc_motion_config_valid == 1U);
    EXPECT_TRUE(diagnostics.mode2_config_valid == 0U);

    return 0;
}

static int test_feedforward_table_and_pi_microsecond_parameters(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    establish_fresh_stop(1U, 1U, 960U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    set_esc_sample(1U, 3U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1562U);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    establish_fresh_stop(1U, 1U, 960U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    EXPECT_EQ_I32(s_last_hall_command_direction, 0);

    reset_fixture();
    enable_valid_esc_configs();
    g_orin_accel_limit_mmps2 = 100000U;
    EXPECT_TRUE(g_speed_pi_enable == APP_SPEED_PI_ENABLE_DEFAULT);
    EXPECT_TRUE(g_speed_pi_kp == APP_SPEED_PI_KP_DEFAULT_US_PER_MPS);
    EXPECT_TRUE(g_speed_pi_ki == APP_SPEED_PI_KI_DEFAULT_US_PER_MPS_S);
    EXPECT_TRUE(g_speed_pi_trim_limit_us == APP_SPEED_PI_TRIM_LIMIT_US);

    establish_fresh_stop(1U, 1U, 960U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    set_esc_sample(1U, 3U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1574U);
    EXPECT_EQ_I32(g_speed_pi_trim_us, 12);
    EXPECT_TRUE(g_speed_pi_saturated == 1U);

    ServoBasic_SetSpeedPiKp(1000.0f);
    ServoBasic_SetSpeedPiKi(1000.0f);
    ServoBasic_SetSpeedPiTrimLimitUs(1000U);
    EXPECT_TRUE(g_speed_pi_kp == 120.0f);
    EXPECT_TRUE(g_speed_pi_ki == 40.0f);
    EXPECT_TRUE(g_speed_pi_trim_limit_us == 60U);

    return 0;
}

static int test_operational_defaults_authorize_speed_control(void)
{
    servo_basic_control_snapshot_t snapshot;

    reset_fixture();
    EXPECT_TRUE(g_orin_esc_forward_max_us == APP_ORIN_ESC_FORWARD_MAX_US);
    EXPECT_TRUE(g_orin_esc_reverse_max_us == APP_ORIN_ESC_REVERSE_MAX_US);

    set_esc_sample(1U, 1U, 1020U, 0U, 1U);
    run_control_at(1020U);
    set_esc_sample(1U, 2U, 1120U, 0U, 1U);
    run_control_at(1120U);
    set_esc_sample(1U, 3U, 1220U, 0U, 1U);
    run_control_at(1220U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);
    EXPECT_TRUE(snapshot.diagnostics.mode2_config_valid != 0U);

    s_fake_tick_ms = 1240U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1240U);
    run_control_at(1260U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.auto_propulsion_authorized != 0U);

    set_esc_sample(1U, 4U, 1280U, 1000U, 1U);
    run_control_at(1280U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.closed_loop_active != 0U);
    EXPECT_TRUE(snapshot.esc_direction_known != 0U);

    return 0;
}

static int test_epoch_invalidation_immediately_blocks_and_recovers_on_fresh_stop(void)
{
    servo_basic_diagnostics_t diagnostics;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    establish_fresh_stop(1U, 1U, 960U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    set_esc_sample(2U, 3U, 1040U, 0U, 1U);
    run_control_at(1040U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.esc_rx_invalidated == 1U);

    set_esc_sample(2U, 4U, 1060U, 0U, 1U);
    run_control_at(1060U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    set_esc_sample(2U, 5U, 1080U, 0U, 1U);
    run_control_at(1080U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    run_control_at(1100U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_duplicate_sample_recomputes_p_without_advancing_i(void)
{
    int32_t trim_after_new_sample;
    float integral_after_new_sample;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiKp(0.0f);
    ServoBasic_SetSpeedPiKi(40.0f);
    establish_fresh_stop(1U, 1U, 900U);

    set_esc_sample(1U, 3U, 1000U, 0U, 1U);
    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    EXPECT_EQ_I32(g_speed_pi_trim_us, 0);

    set_esc_sample(1U, 4U, 1100U, 0U, 1U);
    run_control_at(1100U);
    EXPECT_EQ_I32(g_speed_pi_trim_us, 0);

    set_esc_sample(1U, 5U, 1200U, 0U, 1U);
    run_control_at(1200U);
    trim_after_new_sample = g_speed_pi_trim_us;
    integral_after_new_sample = g_speed_pi_integral;
    EXPECT_TRUE(trim_after_new_sample > 0);

    run_control_at(1240U);
    EXPECT_EQ_I32(g_speed_pi_trim_us, trim_after_new_sample);
    EXPECT_TRUE(g_speed_pi_integral == integral_after_new_sample);

    return 0;
}

static int test_tick_wrap_keeps_fresh_esc_sample_valid(void)
{
    servo_basic_diagnostics_t diagnostics;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    run_control_at(0xFFFFFFB0UL);
    set_esc_sample(1U, 1U, 0xFFFFFFD0UL, 0U, 1U);
    run_control_at(0xFFFFFFD0UL);
    set_esc_sample(1U, 2U, 0xFFFFFFF0UL, 0U, 1U);
    run_control_at(0xFFFFFFF0UL);

    s_fake_tick_ms = 0x00000010UL;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(0x00000010UL);
    run_control_at(0x00000030UL);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.esc_sample_stale == 0U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_high_target_uses_full_pwm_without_absolute_speed_guard(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(15.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    set_esc_sample(1U, 3U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    run_control_at(1150U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_FORWARD_MAX_US);

    return 0;
}

static int test_tracking_brake_releases_to_drive_and_can_reenter(void)
{
    uint32_t status_bits;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 3U, 1040U, 12000U, 1U);
    run_control_at(1040U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);
    run_data_task_once(1040U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_TRUE((status_bits & STATUS_BIT_CLOSED_LOOP_ACTIVE) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_TRACKING_BRAKE_ACTIVE) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_VEHICLE_DIRECTION_KNOWN) != 0U);

    set_esc_sample(1U, 4U, 1060U, 6000U, 1U);
    run_control_at(1060U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);
    run_data_task_once(1060U);
    EXPECT_TRUE((read_u32_be_from_frame(17U) & STATUS_BIT_TRACKING_BRAKE_ACTIVE) == 0U);

    set_esc_sample(1U, 5U, 1080U, 12000U, 1U);
    run_control_at(1080U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_propulsion_outputs_keep_legacy_soft_limits_after_pi(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiKp(120.0f);
    ServoBasic_SetSpeedPiTrimLimitUs(100U);
    g_orin_accel_limit_mmps2 = 100000U;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(10.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    set_esc_sample(1U, 3U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    run_control_at(1060U);
    run_control_at(1080U);
    run_control_at(1100U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_FORWARD_MAX_US);
    EXPECT_EQ_I32(g_speed_pi_final_us, APP_ORIN_ESC_FORWARD_MAX_US);
    EXPECT_TRUE(g_speed_pi_saturated == 1U);

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(1U);
    ServoBasic_SetSpeedPiKp(120.0f);
    ServoBasic_SetSpeedPiTrimLimitUs(100U);
    g_orin_accel_limit_mmps2 = 100000U;
    g_esc_tracking_brake_kp = 1.0f;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    set_esc_sample(1U, 3U, 1040U, 1000U, 1U);
    run_control_at(1040U);

    s_fake_tick_ms = 1060U;
    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1060U);
    set_esc_sample(1U, 4U, 1080U, 1000U, 1U);
    run_control_at(1080U);
    set_esc_sample(1U, 5U, 1100U, 1000U, 1U);
    run_control_at(1100U);
    set_esc_sample(1U, 6U, 1120U, 1000U, 1U);
    run_control_at(1120U);
    set_esc_sample(1U, 7U, 1140U, 0U, 1U);
    run_control_at(1140U);
    set_esc_sample(1U, 8U, 1160U, 0U, 1U);
    run_control_at(1160U);
    run_control_at(1200U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);

    ServoBasic_UpdateAckermannFromOrin(-4.5f, 0.0f, 1U, 0U, 0U);
    run_control_at(1220U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_REVERSE_MAX_US);
    EXPECT_EQ_I32(g_speed_pi_final_us, APP_ORIN_ESC_REVERSE_MAX_US);
    EXPECT_TRUE(g_speed_pi_saturated == 1U);

    return 0;
}

static int test_invalid_brake_pwm_configs_disable_auto_propulsion(void)
{
    servo_basic_diagnostics_t diagnostics;

    reset_fixture();
    enable_valid_esc_configs();
    g_mode2_fwd_to_rev_brake_full_pwm_us = 1800U;
    establish_fresh_stop(1U, 1U, 900U);
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.mode2_config_valid == 0U);

    reset_fixture();
    enable_valid_esc_configs();
    g_mode2_rev_to_fwd_brake_full_pwm_us = APP_ORIN_ESC_CENTER_US - 10U;
    establish_fresh_stop(1U, 1U, 900U);
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.mode2_config_valid == 0U);

    return 0;
}

static int test_brake_endpoint_below_qualification_delta_disables_auto(void)
{
    servo_basic_diagnostics_t diagnostics;

    reset_fixture();
    enable_valid_esc_configs();
    g_mode2_fwd_to_rev_brake_full_pwm_us = APP_ORIN_ESC_CENTER_US - 1U;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    diagnostics = ServoBasic_GetDiagnostics();
    EXPECT_TRUE(diagnostics.mode2_config_valid == 0U);
    EXPECT_TRUE(diagnostics.auto_propulsion_authorized == 0U);

    return 0;
}

static int test_application_mode2_brakes_symmetrically_before_reversal(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    g_esc_tracking_brake_kp = 1.0f;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 3U, 1020U, 1000U, 1U);
    run_control_at(1020U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    s_fake_tick_ms = 1040U;
    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 4U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 5U, 1060U, 1000U, 1U);
    run_control_at(1060U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 6U, 1080U, 1000U, 1U);
    run_control_at(1080U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 7U, 1100U, 0U, 1U);
    run_control_at(1100U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 8U, 1120U, 0U, 1U);
    run_control_at(1120U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    run_control_at(1160U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1430U);

    s_fake_tick_ms = 1200U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 9U, 1200U, 1000U, 1U);
    run_control_at(1200U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 10U, 1220U, 0U, 1U);
    run_control_at(1220U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);
    set_esc_sample(1U, 11U, 1240U, 0U, 1U);
    run_control_at(1240U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);
    run_control_at(1260U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    run_control_at(1300U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_reversal_slew_never_reapplies_source_direction_after_stop(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);

    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1020U);
    s_fake_tick_ms = 1240U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 3U, 1250U, 1000U, 1U);
    run_control_at(1250U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 4U, 1270U, 1000U, 1U);
    run_control_at(1270U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 5U, 1290U, 0U, 1U);
    run_control_at(1290U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);
    set_esc_sample(1U, 6U, 1310U, 0U, 1U);
    run_control_at(1310U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);

    run_control_at(1330U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    run_control_at(1350U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    run_control_at(1370U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_reverse_tracking_decel_brakes_actively(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    g_esc_tracking_brake_kp = 1.0f;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 3U, 1020U, 1000U, 1U);
    run_control_at(1020U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    s_fake_tick_ms = 1040U;
    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 4U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 5U, 1060U, 1000U, 1U);
    run_control_at(1060U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 6U, 1080U, 1000U, 1U);
    run_control_at(1080U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 7U, 1100U, 0U, 1U);
    run_control_at(1100U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1150U);

    set_esc_sample(1U, 8U, 1120U, 0U, 1U);
    run_control_at(1120U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    run_control_at(1160U);
    EXPECT_EQ_U16(s_last_esc_pulse, 1430U);

    set_esc_sample(1U, 9U, 1180U, 12000U, 1U);
    run_control_at(1180U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_data_task_reports_independent_hall_speed(void)
{
    uint32_t status_bits;
    int32_t hall_speed_mmps;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    run_data_task_once(950U);
    EXPECT_TRUE(s_base_telemetry_len == BASE_TELEMETRY_FRAME_LEN);
    EXPECT_TRUE(s_base_telemetry_frame[21] == 0xA1U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_STOP_CONFIRMED) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_CONFIG_VALID) != 0U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), 0);
    EXPECT_EQ_I32(read_i16_be_from_frame(11U), 0);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    set_esc_sample(1U, 3U, 1020U, 1000U, 1U);
    run_control_at(1020U);
    g_hall_speed_state.event_count_total += 5;
    s_hall_speed_valid = 1U;
    s_hall_speed_mps = 0.75f;
    run_data_task_once(1030U);
    status_bits = read_u32_be_from_frame(17U);
    hall_speed_mmps = read_i32_be_from_frame(3U);
    EXPECT_TRUE((s_base_telemetry_frame[1] & TELEMETRY_FLAG_AUTO_ENABLED) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_STOP_CONFIRMED) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_AUTO_PROPULSION_AUTHORIZED) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_CLOSED_LOOP_ACTIVE) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_STATE_AMBIGUOUS) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_CONTROL_INHIBITED) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_CONFIG_VALID) != 0U);
    EXPECT_EQ_I32(hall_speed_mmps, 750);
    EXPECT_TRUE((status_bits & (1UL << 13)) != 0U);

    clear_esc_sample();
    run_control_at(1240U);
    g_hall_speed_state.event_count_total += 5;
    s_hall_speed_mps = -0.5f;
    run_data_task_once(1240U);
    status_bits = read_u32_be_from_frame(17U);
    hall_speed_mmps = read_i32_be_from_frame(3U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_STOP_CONFIRMED) == 0U);
    EXPECT_EQ_I32(hall_speed_mmps, -500);
    EXPECT_TRUE((status_bits & (1UL << 13)) != 0U);

    s_hall_speed_valid = 0U;
    run_data_task_once(1290U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i32_be_from_frame(3U), 0);
    EXPECT_TRUE((status_bits & ((1UL << 13) | (1UL << 19))) == 0U);

    g_hall_speed_state.stationary_confirmed = 1U;
    run_data_task_once(1340U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i32_be_from_frame(3U), 0);
    EXPECT_TRUE((status_bits & (1UL << 13)) == 0U);
    EXPECT_TRUE((status_bits & (1UL << 19)) != 0U);

    return 0;
}

static int test_data_task_reports_unknown_direction_as_positive_esc_magnitude(void)
{
    servo_basic_control_snapshot_t snapshot;
    uint32_t status_bits;

    reset_fixture();
    enable_valid_esc_configs();
    run_control_at(1000U);
    set_esc_sample(1U, 1U, 1020U, 3000U, 1U);
    run_control_at(1020U);

    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.esc_uplink_speed_valid != 0U);
    EXPECT_TRUE(snapshot.esc_direction_known == 0U);
    EXPECT_TRUE(snapshot.signed_speed_valid == 0U);
    EXPECT_TRUE(snapshot.esc_uplink_speed_mps > 0.509f);
    EXPECT_TRUE(snapshot.esc_uplink_speed_mps < 0.511f);

    run_data_task_once(1020U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), 509);
    EXPECT_EQ_I32(read_i16_be_from_frame(11U), 0);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_FE32_FRESH) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_RPM_RAW_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_CALIBRATION_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_VEHICLE_DIRECTION_KNOWN) == 0U);

    return 0;
}

static int test_data_task_reports_known_reverse_as_negative_esc_speed(void)
{
    uint32_t status_bits;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    g_esc_tracking_brake_kp = 1.0f;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    set_esc_sample(1U, 3U, 1020U, 1000U, 1U);
    run_control_at(1020U);

    s_fake_tick_ms = 1040U;
    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 4U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    set_esc_sample(1U, 5U, 1060U, 1000U, 1U);
    run_control_at(1060U);
    set_esc_sample(1U, 6U, 1080U, 1000U, 1U);
    run_control_at(1080U);
    set_esc_sample(1U, 7U, 1100U, 0U, 1U);
    run_control_at(1100U);
    set_esc_sample(1U, 8U, 1120U, 0U, 1U);
    run_control_at(1120U);
    run_control_at(1160U);
    EXPECT_TRUE(s_last_esc_pulse < APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 9U, 1180U, 3000U, 1U);
    run_control_at(1180U);
    run_data_task_once(1180U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), -509);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_VEHICLE_DIRECTION_KNOWN) != 0U);

    return 0;
}

static int test_uplink_speed_uses_low_gear_raw_calibration_without_control_config(void)
{
    servo_basic_control_snapshot_t snapshot;
    uint32_t status_bits;

    reset_fixture();
    g_mode2_fwd_to_rev_brake_request = 0.0f;
    set_esc_sample(1U, 1U, 1000U, 3000U, 1U);
    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.esc_uplink_speed_valid != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_motion_config_valid == 1U);
    EXPECT_TRUE(snapshot.diagnostics.mode2_config_valid == 0U);
    EXPECT_TRUE(snapshot.orin_auto_enabled == 0U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    run_data_task_once(1000U);

    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), 509);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_FE32_FRESH) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_RPM_RAW_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_CALIBRATION_VALID) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_VEHICLE_DIRECTION_KNOWN) == 0U);

    return 0;
}

static int test_uplink_speed_rejects_stale_or_invalid_rpm(void)
{
    uint32_t status_bits;

    reset_fixture();
    set_esc_sample(1U, 1U, 1000U, 3000U, 0U);
    run_control_at(1000U);
    run_data_task_once(1000U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), 0);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_FE32_FRESH) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_RPM_RAW_VALID) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_CALIBRATION_VALID) != 0U);

    reset_fixture();
    set_esc_sample(1U, 1U, 1000U, 3000U, 1U);
    g_esc_speed_fresh_timeout_ms = 500U;
    run_control_at(1501U);
    run_data_task_once(1501U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), 0);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_FE32_FRESH) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_RPM_RAW_VALID) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_CALIBRATION_VALID) != 0U);

    return 0;
}

static int test_uplink_speed_does_not_authorize_auto_propulsion(void)
{
    servo_basic_control_snapshot_t snapshot;
    uint32_t status_bits;

    reset_fixture();
    g_mode2_fwd_to_rev_brake_request = 0.0f;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    set_esc_sample(1U, 1U, 1000U, 3000U, 1U);
    run_control_at(1000U);

    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.esc_uplink_speed_valid != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_motion_config_valid == 1U);
    EXPECT_TRUE(snapshot.diagnostics.mode2_config_valid == 0U);
    EXPECT_TRUE(snapshot.signed_speed_valid == 0U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    run_data_task_once(1000U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_TRUE((status_bits & STATUS_BIT_AUTO_PROPULSION_AUTHORIZED) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_CLOSED_LOOP_ACTIVE) == 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_STATE_AMBIGUOUS) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_CONTROL_INHIBITED) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_MODE2_CONFIG_VALID) == 0U);

    return 0;
}

static int test_data_task_stop_priority_clears_wire_speed_and_yaw(void)
{
    servo_basic_control_snapshot_t snapshot;
    uint32_t status_bits;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    g_orin_accel_limit_mmps2 = 100000U;
    g_esc_tracking_brake_kp = 1.0f;
    establish_fresh_stop(1U, 1U, 900U);

    s_fake_tick_ms = 1000U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.3f, 1U, 0U, 0U);
    run_control_at(1000U);
    set_esc_sample(1U, 3U, 1020U, 1000U, 1U);
    run_control_at(1020U);

    s_fake_tick_ms = 1040U;
    ServoBasic_UpdateAckermannFromOrin(-1.0f, 0.3f, 1U, 0U, 0U);
    set_esc_sample(1U, 4U, 1040U, 1000U, 1U);
    run_control_at(1040U);
    set_esc_sample(1U, 5U, 1060U, 1000U, 1U);
    run_control_at(1060U);
    set_esc_sample(1U, 6U, 1080U, 1000U, 1U);
    run_control_at(1080U);
    set_esc_sample(1U, 7U, 1100U, 40U, 1U);
    run_control_at(1100U);
    set_esc_sample(1U, 8U, 1120U, 40U, 1U);
    run_control_at(1120U);

    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);
    EXPECT_TRUE(snapshot.signed_speed_valid != 0U);
    EXPECT_TRUE(snapshot.speed_mps == 0.0f);
    EXPECT_TRUE(snapshot.esc_direction_known == 0U);

    run_data_task_once(1130U);
    status_bits = read_u32_be_from_frame(17U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_STOP_CONFIRMED) != 0U);
    EXPECT_TRUE((status_bits & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) == 0U);
    EXPECT_EQ_I32(read_i16_be_from_frame(7U), 0);
    EXPECT_EQ_I32(read_i16_be_from_frame(11U), 0);

    return 0;
}

static int test_control_snapshot_copy_is_stable_across_later_control_tick(void)
{
    servo_basic_control_snapshot_t before;
    servo_basic_control_snapshot_t after;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.2f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);

    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&before) != 0U);
    EXPECT_TRUE(before.publish_sequence != 0U);
    EXPECT_TRUE(before.control_tick_ms == 1010U);
    EXPECT_TRUE(before.state.esc_pulse_us > APP_ORIN_ESC_CENTER_US);

    ServoBasic_UpdateAckermannFromOrin(0.0f, -0.2f, 1U, 1U, 0U);
    run_control_at(1020U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&after) != 0U);
    EXPECT_TRUE(after.publish_sequence > before.publish_sequence);
    EXPECT_TRUE(after.control_tick_ms == 1020U);
    EXPECT_TRUE(before.control_tick_ms == 1010U);
    EXPECT_TRUE(before.state.esc_pulse_us > APP_ORIN_ESC_CENTER_US);
    EXPECT_TRUE(before.steering_angle_rad > 0.0f);

    return 0;
}

static int test_control_snapshot_rx_invalidation_clears_valid_and_stop(void)
{
    servo_basic_control_snapshot_t snapshot;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);

    clear_esc_sample();
    run_control_at(1120U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_rx_invalidated != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_feedback_valid == 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed == 0U);
    EXPECT_TRUE(snapshot.signed_speed_valid == 0U);
    EXPECT_TRUE(snapshot.speed_mps == 0.0f);
    EXPECT_TRUE(snapshot.yaw_rate_rad_s == 0.0f);

    run_data_task_once(1120U);
    EXPECT_TRUE((read_u32_be_from_frame(17U) & STATUS_BIT_ESC_STOP_CONFIRMED) == 0U);
    EXPECT_TRUE((read_u32_be_from_frame(17U) & STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID) == 0U);

    return 0;
}

static int test_control_snapshot_records_rc_aux_without_changing_authority(void)
{
    servo_basic_control_snapshot_t before;
    servo_basic_control_snapshot_t after;

    reset_fixture();
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&before) != 0U);
    s_rc.aux_us = 2000U;
    s_rc.aux_active = 1U;
    run_control_at(1000U);

    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&after) != 0U);
    EXPECT_TRUE(after.publish_sequence > before.publish_sequence);
    EXPECT_TRUE(after.control_tick_ms == 1000U);
    EXPECT_TRUE(after.rc_override_active == 0U);
    EXPECT_TRUE(g_rc_aux_present != 0U);
    EXPECT_TRUE(g_rc_aux_pulse_us == 2000U);
    EXPECT_EQ_U16(after.state.esc_pulse_us, APP_ORIN_ESC_CENTER_US);
    EXPECT_EQ_U16(after.state.servo_pulse_us, APP_ORIN_SERVO_CENTER_US);

    run_data_task_once(1000U);
    EXPECT_TRUE((s_base_telemetry_frame[1] & TELEMETRY_FLAG_RC_OVERRIDE_ACTIVE) == 0U);
    EXPECT_TRUE((s_base_telemetry_frame[1] & TELEMETRY_FLAG_STOP_OVERRIDE_ACTIVE) == 0U);

    return 0;
}

static int test_forward_recovery_requires_fresh_motion_before_signed_speed(void)
{
    servo_basic_control_snapshot_t snapshot;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);

    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.control_tick_ms == 1010U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed == 0U);
    EXPECT_TRUE(snapshot.signed_speed_valid == 0U);
    EXPECT_TRUE(snapshot.esc_direction_known == 0U);

    set_esc_sample(1U, 3U, 1020U, 1000U, 1U);
    run_control_at(1020U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.signed_speed_valid != 0U);
    EXPECT_TRUE(snapshot.esc_direction_known != 0U);

    return 0;
}

static int test_forward_recovery_stops_on_command_or_feedback_timeout(void)
{
    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    set_esc_sample(1U, 3U, 1260U, 0U, 1U);
    run_control_at(1260U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1000U);
    run_control_at(1010U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    s_fake_tick_ms = 1260U;
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1260U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_invalidated_feedback_requires_post_boundary_stop_samples(void)
{
    servo_basic_control_snapshot_t snapshot;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);

    clear_esc_sample();
    run_control_at(1000U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_rx_invalidated != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed == 0U);

    set_esc_sample(1U, 3U, 980U, 0U, 1U);
    run_control_at(1020U);
    set_esc_sample(1U, 4U, 1000U, 0U, 1U);
    run_control_at(1040U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_feedback_valid == 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed == 0U);

    set_esc_sample(1U, 5U, 1060U, 0U, 1U);
    run_control_at(1060U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed == 0U);
    set_esc_sample(1U, 6U, 1080U, 0U, 1U);
    run_control_at(1080U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);

    return 0;
}

static int test_brake_pwm_mapping_change_requires_fresh_post_change_samples(void)
{
    servo_basic_control_snapshot_t snapshot;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiEnable(0U);
    establish_fresh_stop(1U, 1U, 900U);
    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(940U);
    run_control_at(960U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    ServoBasic_UpdateAckermannFromOrin(0.0f, 0.0f, 0U, 0U, 0U);
    g_mode2_fwd_to_rev_brake_full_pwm_us = APP_ORIN_ESC_CENTER_US - 1U;
    run_control_at(1000U);
    EXPECT_EQ_U16(s_last_esc_pulse, APP_ORIN_ESC_CENTER_US);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.mode2_config_valid == 0U);

    g_mode2_fwd_to_rev_brake_full_pwm_us = APP_MODE2_FWD_TO_REV_BRAKE_FULL_PWM_US_DEFAULT;
    set_esc_sample(1U, 3U, 1000U, 0U, 1U);
    run_control_at(1020U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.mode2_config_valid != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_feedback_valid == 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed == 0U);

    set_esc_sample(1U, 4U, 1040U, 0U, 1U);
    run_control_at(1040U);
    set_esc_sample(1U, 5U, 1060U, 0U, 1U);
    run_control_at(1060U);
    EXPECT_TRUE(ServoBasic_GetControlSnapshot(&snapshot) != 0U);
    EXPECT_TRUE(snapshot.diagnostics.esc_stop_confirmed != 0U);

    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(1080U);
    run_control_at(1100U);
    EXPECT_TRUE(s_last_esc_pulse > APP_ORIN_ESC_CENTER_US);

    return 0;
}

static int test_speed_pi_uses_zero_dt_for_same_tick_and_wrap_delta(void)
{
    float integral_after_wrapped_sample;

    reset_fixture();
    enable_valid_esc_configs();
    ServoBasic_SetSpeedPiKp(0.0f);
    ServoBasic_SetSpeedPiKi(40.0f);
    establish_fresh_stop(2U, 1U, 0xFFFFFFB0UL);

    ServoBasic_UpdateAckermannFromOrin(1.0f, 0.0f, 1U, 0U, 0U);
    run_control_at(0xFFFFFFF0UL);
    run_control_at(0x00000010UL);

    set_esc_sample(2U, 3U, 0x00000020UL, 0U, 1U);
    run_control_at(0x00000020UL);
    integral_after_wrapped_sample = g_speed_pi_integral;
    EXPECT_TRUE(integral_after_wrapped_sample > 0.0f);

    set_esc_sample(2U, 4U, 0x00000020UL, 0U, 1U);
    run_control_at(0x00000030UL);
    EXPECT_TRUE(g_speed_pi_integral == integral_after_wrapped_sample);

    return 0;
}


int main(void)
{
    if (test_idle_and_software_stop_use_center_not_zero() != 0)
    {
        return 1;
    }
    if (test_rc_priority_and_software_stop_is_serial_only() != 0)
    {
        return 1;
    }
    if (test_nonzero_serial_during_rc_releases_to_zero_not_cached_motion() != 0)
    {
        return 1;
    }
    if (test_rc_hall_direction_forward_brake_neutral_reverse() != 0)
    {
        return 1;
    }
    if (test_rc_hall_direction_reverse_brake_neutral_forward() != 0)
    {
        return 1;
    }
    if (test_rc_hall_direction_cancel_brake_keeps_current_direction() != 0)
    {
        return 1;
    }
    if (test_rc_hall_direction_requires_stop_before_neutral_unlock() != 0)
    {
        return 1;
    }
    if (test_rc_hall_direction_resets_when_rc_exits() != 0)
    {
        return 1;
    }
    if (test_rc_hall_direction_renewed_motion_revokes_brake_stop() != 0)
    {
        return 1;
    }
    if (test_rc_aux_does_not_change_partial_hall_reversal_sequence() != 0)
    {
        return 1;
    }
    if (test_steering_direction_limit_and_rate_limit() != 0)
    {
        return 1;
    }
    if (test_missing_esc_or_mode2_parameters_keep_propulsion_off() != 0)
    {
        return 1;
    }
    if (test_feedforward_table_and_pi_microsecond_parameters() != 0)
    {
        return 1;
    }
    if (test_operational_defaults_authorize_speed_control() != 0)
    {
        return 1;
    }
    if (test_epoch_invalidation_immediately_blocks_and_recovers_on_fresh_stop() != 0)
    {
        return 1;
    }
    if (test_duplicate_sample_recomputes_p_without_advancing_i() != 0)
    {
        return 1;
    }
    if (test_tick_wrap_keeps_fresh_esc_sample_valid() != 0)
    {
        return 1;
    }
    if (test_high_target_uses_full_pwm_without_absolute_speed_guard() != 0)
    {
        return 1;
    }
    if (test_tracking_brake_releases_to_drive_and_can_reenter() != 0)
    {
        return 1;
    }
    if (test_propulsion_outputs_keep_legacy_soft_limits_after_pi() != 0)
    {
        return 1;
    }
    if (test_invalid_brake_pwm_configs_disable_auto_propulsion() != 0)
    {
        return 1;
    }
    if (test_brake_endpoint_below_qualification_delta_disables_auto() != 0)
    {
        return 1;
    }
    if (test_application_mode2_brakes_symmetrically_before_reversal() != 0)
    {
        return 1;
    }
    if (test_reversal_slew_never_reapplies_source_direction_after_stop() != 0)
    {
        return 1;
    }
    if (test_reverse_tracking_decel_brakes_actively() != 0)
    {
        return 1;
    }
    if (test_data_task_reports_independent_hall_speed() != 0)
    {
        return 1;
    }
    if (test_data_task_reports_unknown_direction_as_positive_esc_magnitude() != 0)
    {
        return 1;
    }
    if (test_data_task_reports_known_reverse_as_negative_esc_speed() != 0)
    {
        return 1;
    }
    if (test_uplink_speed_uses_low_gear_raw_calibration_without_control_config() != 0)
    {
        return 1;
    }
    if (test_uplink_speed_rejects_stale_or_invalid_rpm() != 0)
    {
        return 1;
    }
    if (test_uplink_speed_does_not_authorize_auto_propulsion() != 0)
    {
        return 1;
    }
    if (test_data_task_stop_priority_clears_wire_speed_and_yaw() != 0)
    {
        return 1;
    }
    if (test_control_snapshot_copy_is_stable_across_later_control_tick() != 0)
    {
        return 1;
    }
    if (test_control_snapshot_rx_invalidation_clears_valid_and_stop() != 0)
    {
        return 1;
    }
    if (test_control_snapshot_records_rc_aux_without_changing_authority() != 0)
    {
        return 1;
    }
    if (test_forward_recovery_requires_fresh_motion_before_signed_speed() != 0)
    {
        return 1;
    }
    if (test_forward_recovery_stops_on_command_or_feedback_timeout() != 0)
    {
        return 1;
    }
    if (test_invalidated_feedback_requires_post_boundary_stop_samples() != 0)
    {
        return 1;
    }
    if (test_brake_pwm_mapping_change_requires_fresh_post_change_samples() != 0)
    {
        return 1;
    }
    if (test_speed_pi_uses_zero_dt_for_same_tick_and_wrap_delta() != 0)
    {
        return 1;
    }

    return 0;
}
