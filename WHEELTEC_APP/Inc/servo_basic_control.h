/**
 * @file servo_basic_control.h
 * @brief Protocol adapter for servo/ESC control with RC override and Orin Ackermann mapping.
 *
 * The module clamps inputs, supports RC takeover, and maps Orin Ackermann
 * speed/steering commands into ESC/servo PWM.
 */

#ifndef SERVO_BASIC_CONTROL_H
#define SERVO_BASIC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SERVO_CMD_SET_SERVO_ANGLE 0x53U
#define SERVO_CMD_SET_SERVO_PULSE 0x50U
#define SERVO_CMD_SET_ESC_PULSE   0x45U

#define ESC_PWM_MIN_PULSE_US        1000U
#define ESC_PWM_NEUTRAL_PULSE_US    1500U
#define ESC_PWM_MAX_PULSE_US        2000U
#define ESC_PULSE_STEP_US           1U

#define SERVO_MIN_PULSE_US          ESC_PWM_MIN_PULSE_US
#define SERVO_MAX_PULSE_US          2000U
#define SERVO_PULSE_STEP_US         5U

typedef enum
{
	SERVO_CTRL_MODE_RC_PASSTHROUGH = 0U,
	SERVO_CTRL_MODE_AUTONOMOUS     = 1U,
} servo_control_mode_t;

typedef struct
{
	uint16_t esc_pulse_us;
	uint16_t servo_pulse_us;
	servo_control_mode_t control_mode;
	uint8_t rc_takeover_pending;
	uint8_t emergency_stop;
} servo_basic_state_t;

void ServoBasic_Init(void);
void ServoBasic_ProcessControl(void);
const servo_basic_state_t *ServoBasic_GetState(void);
uint8_t ServoBasic_IsRcOverrideActive(void);
uint8_t ServoBasic_IsRcEmergencyActive(void);
uint8_t ServoBasic_IsOrinCommandTimeout(void);
uint8_t ServoBasic_IsOrinAutoEnabled(void);
uint8_t ServoBasic_IsOrinBrakeActive(void);
uint8_t ServoBasic_IsOrinEmergencyActive(void);
uint8_t ServoBasic_GetAckermannFeedback(float *speed_mps,
                                        float *steering_angle_rad,
                                        float *yaw_rate_rad_s);

void ServoBasic_OutputEscPulse(uint16_t pulse_us);
void ServoBasic_OutputServoPulse(uint16_t pulse_us);
void ServoBasic_UpdateAckermannFromOrin(float speed_mps,
                                        float steering_angle_rad,
                                        uint8_t enable,
                                        uint8_t brake,
                                        uint8_t emergency_stop);
void ServoBasic_SetSpeedPiEnable(uint8_t enable);
void ServoBasic_SetSpeedPiKp(float kp_us_per_mps);
void ServoBasic_SetSpeedPiKi(float ki_us_per_mps_s);
void ServoBasic_SetSpeedPiTrimLimitUs(uint32_t limit_us);
void ServoBasic_ResetSpeedPi(void);

#ifdef __cplusplus
}
#endif

#endif
