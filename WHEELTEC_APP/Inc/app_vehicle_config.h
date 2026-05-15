#ifndef __APP_VEHICLE_CONFIG_H
#define __APP_VEHICLE_CONFIG_H

#include <stdint.h>

/*
 * Current-product vehicle defaults.
 *
 * Runtime `g_orin_*` variables still exist for Keil Watch tuning. These macros
 * are the single source for reset-time defaults when those variables are zero.
 */
#define APP_ORIN_PWM_ENABLE_DEFAULT                 1U
#define APP_ORIN_PWM_TIMEOUT_DEFAULT_MS           250U

#define APP_ORIN_ACKERMANN_WHEELBASE_MM           600U
#define APP_ORIN_ACKERMANN_TRACK_WIDTH_MM         480U
#define APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM        110U
#define APP_ORIN_ACKERMANN_MAX_STEERING_MRAD      393U
#define APP_ORIN_ACKERMANN_MIN_VX_MMPS             50U

#define APP_ORIN_VX_SCALE_PERMILLE               1000U
#define APP_ORIN_VX_FORWARD_CAP_MMPS              500U
#define APP_ORIN_VX_REVERSE_CAP_MMPS              300U
#define APP_ORIN_VX_DEADBAND_MMPS                  50U
#define APP_ORIN_VX_MAX_DEFAULT_MMPS              500U

#define APP_ORIN_MIN_FORWARD_VX_MMPS               70U
#define APP_ORIN_MIN_REVERSE_VX_MMPS              180U

#define APP_HALL_SPEED_LIMIT_MMPS                5000U
#define APP_HALL_SPEED_LIMIT_RELEASE_MMPS        4500U

#define APP_ORIN_ACCEL_LIMIT_MMPS2                600U
#define APP_ORIN_STEERING_RATE_LIMIT_MRADPS       900U

#define APP_SPEED_PI_ENABLE_DEFAULT                 1U
#define APP_SPEED_PI_KP_DEFAULT_US_PER_MPS       20.0f
#define APP_SPEED_PI_KI_DEFAULT_US_PER_MPS_S      0.0f
#define APP_SPEED_PI_TRIM_LIMIT_US                 12U

#define APP_ORIN_ESC_CENTER_US                   1500U
#define APP_ORIN_ESC_RANGE_US                     500U
#define APP_ORIN_ESC_FORWARD_START_US            1560U
#define APP_ORIN_ESC_REVERSE_START_US            1440U
#define APP_ORIN_ESC_FORWARD_MAX_US              1650U
#define APP_ORIN_ESC_REVERSE_MAX_US              1350U

#define APP_ORIN_SERVO_CENTER_US                 1500U
#define APP_ORIN_SERVO_RANGE_US                   500U

#define APP_RC_VALID_MIN_US                       900U
#define APP_RC_VALID_MAX_US                      2100U
#define APP_RC_FRAME_MIN_US                      5000U
#define APP_RC_FRAME_MAX_US                     30000U
#define APP_RC_GLITCH_FREEZE_MS                   100U
#define APP_RC_SIGNAL_TIMEOUT_MS                  100U
#define APP_RC_PWM_FOLLOW_RAW_DEFAULT               0U

#define APP_RC_DEBOUNCE_ENABLE_DEFAULT              1U
#define APP_RC_DEBOUNCE_DEADBAND_US                 5U
#define APP_RC_DEBOUNCE_SMOOTH_DIV                  4U
#define APP_RC_THROTTLE_NEUTRAL_HOLD_US            25U
#define APP_RC_THROTTLE_JUMP_CONFIRM_US            40U
#define APP_RC_STEERING_JUMP_CONFIRM_US            80U
#define APP_RC_JUMP_CONFIRM_SAMPLES                 2U

#define APP_RC_OVERRIDE_CENTER_US                 1500U
#define APP_RC_OVERRIDE_ENTER_THRESHOLD_US          60U
#define APP_RC_OVERRIDE_EXIT_THRESHOLD_US           40U
#define APP_RC_OVERRIDE_ENTER_SAMPLES               2U
#define APP_RC_OVERRIDE_RELEASE_HOLD_MS           500U

#define APP_RC_GUARD_ENABLE_DEFAULT                 1U
#define APP_RC_GUARD_ACTIVE_HIGH_DEFAULT            1U
#define APP_RC_GUARD_ACTIVE_LOW_THRESHOLD_US     1300U
#define APP_RC_GUARD_ACTIVE_HIGH_THRESHOLD_US    1700U

#define APP_BATTERY_LOW_MV                       7000U
#define APP_BATTERY_CRITICAL_MV                  6600U

#endif
