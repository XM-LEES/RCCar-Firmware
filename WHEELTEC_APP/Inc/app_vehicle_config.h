#ifndef __APP_VEHICLE_CONFIG_H
#define __APP_VEHICLE_CONFIG_H

#include <stdint.h>

/*
 * Current-product vehicle defaults.
 *
 * Runtime `g_orin_*` variables still exist for Keil Watch tuning. These macros
 * are the single source for reset-time defaults when those variables are zero.
 */
#define APP_ORIN_PWM_TIMEOUT_DEFAULT_MS           250U

#define APP_ORIN_ACKERMANN_WHEELBASE_MM           600U
#define APP_ORIN_ACKERMANN_TRACK_WIDTH_MM         500U
#define APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM        115U
#define APP_ORIN_ACKERMANN_MAX_STEERING_MRAD      349U
#define APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS         3U

#define APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT 0.14115f
#define APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT      250U

#define APP_ESC_STOPPED_THRESHOLD_MPS_DEFAULT      0.05f
#define APP_ESC_STOPPED_MIN_SAMPLES_DEFAULT           3U
#define APP_ESC_STOPPED_MIN_COVERAGE_MS_DEFAULT     150U

#define APP_ORIN_STEERING_RATE_LIMIT_MRADPS       900U

#define APP_SPEED_PID_KP_DEFAULT_US_PER_MPS       120.0f
#define APP_SPEED_PID_KI_DEFAULT_US_PER_MPS_S      20.0f
#define APP_SPEED_PID_KD_DEFAULT_US_PER_MPS2        0.0f
#define APP_SPEED_PID_DERIVATIVE_TAU_MS              80U
#define APP_SPEED_PID_TRACKING_TAU_MS               100U
#define APP_AUTO_BRAKE_MIN_US                        50U
#define APP_AUTO_BRAKE_ENTER_ERROR_MPS              0.20f
#define APP_AUTO_BRAKE_ENTER_RATIO                  0.10f
#define APP_AUTO_BRAKE_RELEASE_ERROR_MPS            0.05f
#define APP_AUTO_BRAKE_RELEASE_RATIO                0.02f
#define APP_AUTO_COAST_EVAL_MS                       200U
#define APP_AUTO_COAST_BUDGET_MS                     600U
#define APP_AUTO_A_PROGRESS_MPS2                   0.05f
#define APP_AUTO_BRAKE_RELEASE_DELAY_MS               80U
#define APP_AUTO_ACTION_ACK_MS                      300U
#define APP_AUTO_QUALIFY_MS                         100U
#define APP_AUTO_NEUTRAL_DWELL_MS                   100U

#define APP_ORIN_ESC_CENTER_US                   1500U

#define APP_ORIN_SERVO_CENTER_US                 1500U
#define APP_ORIN_SERVO_RANGE_US                   395U
/* Standard positive tire angle is left; this chassis needs increasing PWM. */
#define APP_ORIN_STEERING_PWM_DIRECTION_SIGN        (+1)

#if (APP_ORIN_STEERING_PWM_DIRECTION_SIGN != 1) && \
	(APP_ORIN_STEERING_PWM_DIRECTION_SIGN != -1)
#error "APP_ORIN_STEERING_PWM_DIRECTION_SIGN must be +1 or -1"
#endif

#if APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS < 2U
#error "APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS must reject a single glitch"
#endif

#define APP_RC_VALID_MIN_US                       900U
#define APP_RC_VALID_MAX_US                      2100U
#define APP_RC_FRAME_MIN_US                      5000U
#define APP_RC_FRAME_MAX_US                     30000U
#define APP_RC_GLITCH_FREEZE_MS                   100U
#define APP_RC_SIGNAL_TIMEOUT_MS                  100U
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

#endif
