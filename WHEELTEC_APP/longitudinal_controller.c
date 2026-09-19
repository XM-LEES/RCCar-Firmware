#include "longitudinal_controller.h"

#include <math.h>
#include <string.h>

#define LONGITUDINAL_TICK_HALF_RANGE 0x80000000UL

typedef struct
{
    uint16_t speed_mmps;
    uint16_t pulse_us;
} LongitudinalFeedForwardPoint_t;

static const LongitudinalFeedForwardPoint_t s_forward_ff_table[] = {
    {70U, 1546U},
    {100U, 1547U},
    {200U, 1548U},
    {300U, 1550U},
    {500U, 1554U},
    {800U, 1559U},
    {1000U, 1562U},
    {1500U, 1568U},
    {2000U, 1574U},
    {2500U, 1580U},
    {3000U, 1585U},
    {3500U, 1590U},
    {4000U, 1596U},
    {4500U, 1601U},
    {10000U, 1650U},
};

static const LongitudinalFeedForwardPoint_t s_reverse_ff_table[] = {
    {180U, 1444U},
    {300U, 1442U},
    {500U, 1438U},
    {800U, 1433U},
    {1000U, 1430U},
    {1500U, 1423U},
    {2000U, 1417U},
    {2500U, 1412U},
    {3000U, 1407U},
    {3500U, 1402U},
    {4000U, 1397U},
    {4500U, 1391U},
};

static uint8_t finite_positive(float value)
{
    return (isfinite(value) && value > 0.0f) ? 1U : 0U;
}

static uint8_t finite_nonnegative(float value)
{
    return (isfinite(value) && value >= 0.0f) ? 1U : 0U;
}

static uint8_t unit_value(float value)
{
    return (isfinite(value) && value >= 0.0f && value <= 1.0f) ? 1U : 0U;
}

static uint8_t direction_is_valid(LongitudinalDirection_t direction)
{
    return (direction == LONGITUDINAL_DIRECTION_FORWARD ||
            direction == LONGITUDINAL_DIRECTION_REVERSE ||
            direction == LONGITUDINAL_DIRECTION_UNKNOWN) ? 1U : 0U;
}

static uint8_t tick_delta_ms(uint32_t later_tick_ms,
                             uint32_t earlier_tick_ms,
                             uint32_t *delta_ms)
{
    const uint32_t local_delta_ms = later_tick_ms - earlier_tick_ms;

    if (local_delta_ms >= LONGITUDINAL_TICK_HALF_RANGE)
    {
        return 0U;
    }
    if (delta_ms != NULL)
    {
        *delta_ms = local_delta_ms;
    }
    return 1U;
}

static uint16_t clamp_pwm_i32(const LongitudinalControllerConfig_t *config,
                              int32_t pwm_us)
{
    if (pwm_us < (int32_t)config->min_pwm_us)
    {
        return config->min_pwm_us;
    }
    if (pwm_us > (int32_t)config->max_pwm_us)
    {
        return config->max_pwm_us;
    }
    return (uint16_t)pwm_us;
}

static uint16_t limit_drive_pwm(const LongitudinalControllerConfig_t *config,
                                LongitudinalDirection_t direction,
                                int32_t pwm_us)
{
    const int32_t center = (int32_t)config->center_pwm_us;

    if (direction == LONGITUDINAL_DIRECTION_FORWARD)
    {
        if (pwm_us < center)
        {
            pwm_us = center;
        }
        if (pwm_us > (int32_t)config->forward_limit_pwm_us)
        {
            pwm_us = (int32_t)config->forward_limit_pwm_us;
        }
    }
    else if (direction == LONGITUDINAL_DIRECTION_REVERSE)
    {
        if (pwm_us > center)
        {
            pwm_us = center;
        }
        if (pwm_us < (int32_t)config->reverse_limit_pwm_us)
        {
            pwm_us = (int32_t)config->reverse_limit_pwm_us;
        }
    }
    else
    {
        pwm_us = center;
    }

    return clamp_pwm_i32(config, pwm_us);
}

static uint8_t target_is_valid(float target_mps)
{
    return isfinite(target_mps) ? 1U : 0U;
}

static LongitudinalDirection_t direction_from_target(float target_mps)
{
    if (target_mps > 0.0f)
    {
        return LONGITUDINAL_DIRECTION_FORWARD;
    }
    if (target_mps < 0.0f)
    {
        return LONGITUDINAL_DIRECTION_REVERSE;
    }
    return LONGITUDINAL_DIRECTION_UNKNOWN;
}

static float signed_feedback_mps(const LongitudinalControllerInput_t *input)
{
    if (input->stopped != 0U)
    {
        return 0.0f;
    }
    if (input->current_direction == LONGITUDINAL_DIRECTION_REVERSE)
    {
        return -input->speed_magnitude_mps;
    }
    if (input->current_direction == LONGITUDINAL_DIRECTION_FORWARD)
    {
        return input->speed_magnitude_mps;
    }
    return 0.0f;
}

static uint8_t feedback_is_valid(const LongitudinalControllerInput_t *input)
{
    if (input == NULL ||
        input->feedback_valid == 0U ||
        finite_nonnegative(input->speed_magnitude_mps) == 0U ||
        direction_is_valid(input->current_direction) == 0U)
    {
        return 0U;
    }

    if (input->stopped != 0U)
    {
        return 1U;
    }

    return (input->current_direction != LONGITUDINAL_DIRECTION_UNKNOWN) ? 1U : 0U;
}

static uint16_t interpolate_ff_table(
    const LongitudinalFeedForwardPoint_t *table,
    uint32_t table_count,
    uint32_t speed_mmps)
{
    uint32_t i;

    if (speed_mmps <= table[0].speed_mmps)
    {
        return table[0].pulse_us;
    }

    for (i = 1U; i < table_count; ++i)
    {
        const uint32_t low_speed = table[i - 1U].speed_mmps;
        const uint32_t high_speed = table[i].speed_mmps;

        if (speed_mmps <= high_speed)
        {
            const int32_t low_pulse = (int32_t)table[i - 1U].pulse_us;
            const int32_t high_pulse = (int32_t)table[i].pulse_us;
            const int32_t delta_pulse = high_pulse - low_pulse;
            const uint32_t delta_speed = high_speed - low_speed;
            const uint32_t target_delta = speed_mmps - low_speed;
            int32_t pulse = low_pulse;

            if (delta_speed != 0U)
            {
                pulse += (int32_t)(((int64_t)delta_pulse *
                                    (int64_t)target_delta) /
                                   (int64_t)delta_speed);
            }
            return (uint16_t)pulse;
        }
    }

    return table[table_count - 1U].pulse_us;
}

static uint32_t abs_speed_mmps(float speed_mps)
{
    float mmps = fabsf(speed_mps) * 1000.0f;

    if (mmps > 100000.0f)
    {
        mmps = 100000.0f;
    }
    return (uint32_t)(mmps + 0.5f);
}

static uint16_t feed_forward_pwm(const LongitudinalControllerConfig_t *config,
                                 float target_mps)
{
    const LongitudinalDirection_t direction = direction_from_target(target_mps);
    const uint32_t speed_mmps = abs_speed_mmps(target_mps);

    if (direction == LONGITUDINAL_DIRECTION_FORWARD)
    {
        return limit_drive_pwm(config,
                               direction,
                               (int32_t)interpolate_ff_table(
                                   s_forward_ff_table,
                                   (uint32_t)(sizeof(s_forward_ff_table) /
                                              sizeof(s_forward_ff_table[0])),
                                   speed_mmps));
    }
    if (direction == LONGITUDINAL_DIRECTION_REVERSE)
    {
        return limit_drive_pwm(config,
                               direction,
                               (int32_t)interpolate_ff_table(
                                   s_reverse_ff_table,
                                   (uint32_t)(sizeof(s_reverse_ff_table) /
                                              sizeof(s_reverse_ff_table[0])),
                                   speed_mmps));
    }
    return config->center_pwm_us;
}

static int32_t float_to_i32_nearest(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static float drive_effort_from_pwm(const LongitudinalControllerConfig_t *config,
                                   LongitudinalDirection_t direction,
                                   uint16_t pwm_us)
{
    if (direction == LONGITUDINAL_DIRECTION_FORWARD)
    {
        const uint32_t span = (uint32_t)config->forward_limit_pwm_us -
            (uint32_t)config->center_pwm_us;
        return (span == 0U) ? 0.0f :
            (float)((uint32_t)pwm_us - (uint32_t)config->center_pwm_us) /
            (float)span;
    }
    if (direction == LONGITUDINAL_DIRECTION_REVERSE)
    {
        const uint32_t span = (uint32_t)config->center_pwm_us -
            (uint32_t)config->reverse_limit_pwm_us;
        return (span == 0U) ? 0.0f :
            (float)((uint32_t)config->center_pwm_us - (uint32_t)pwm_us) /
            (float)span;
    }
    return 0.0f;
}

static float step_toward(float current,
                         float target,
                         float max_delta,
                         uint8_t *limited)
{
    const float delta = target - current;

    if (limited != NULL)
    {
        *limited = 0U;
    }
    if (max_delta <= 0.0f)
    {
        return target;
    }
    if (delta > max_delta)
    {
        if (limited != NULL)
        {
            *limited = 1U;
        }
        return current + max_delta;
    }
    if (delta < -max_delta)
    {
        if (limited != NULL)
        {
            *limited = 1U;
        }
        return current - max_delta;
    }
    return target;
}

static void reset_dynamic_state(LongitudinalController_t *controller)
{
    controller->has_update_tick = 0U;
    controller->last_update_tick_ms = 0U;
    controller->slewed_target_mps = 0.0f;
    controller->have_feedback_sample = 0U;
    controller->last_feedback_sample_id = 0U;
    controller->last_feedback_sample_tick_ms = 0U;
    controller->pi_integral_mps_s = 0.0f;
    controller->tracking_brake_active = 0U;
}

static LongitudinalControllerOutput_t make_output(
    const LongitudinalController_t *controller,
    LongitudinalIntent_t intent,
    LongitudinalReason_t reason)
{
    LongitudinalControllerOutput_t output;

    memset(&output, 0, sizeof(output));
    output.intent = intent;
    output.reason = reason;
    output.target_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    output.drive_pwm_us = (controller != NULL) ?
        controller->config.center_pwm_us : 0U;
    output.diagnostics.reason = reason;
    output.diagnostics.config_valid =
        (controller != NULL && controller->config_valid != 0U) ? 1U : 0U;
    output.diagnostics.inhibited =
        (reason == LONGITUDINAL_REASON_NOT_ENABLED ||
         reason == LONGITUDINAL_REASON_CONFIG_INVALID ||
         reason == LONGITUDINAL_REASON_FEEDBACK_INVALID) ? 1U : 0U;
    if (controller != NULL)
    {
        output.diagnostics.slewed_target_mps = controller->slewed_target_mps;
        output.diagnostics.pi_integral_mps_s = controller->pi_integral_mps_s;
        output.diagnostics.tracking_brake_active =
            controller->tracking_brake_active;
        output.diagnostics.feedforward_pwm_us =
            controller->config.center_pwm_us;
    }
    return output;
}

uint8_t LongitudinalController_ConfigIsValid(
    const LongitudinalControllerConfig_t *config,
    LongitudinalReason_t *reason)
{
    LongitudinalReason_t local_reason = LONGITUDINAL_REASON_OK;
    uint8_t valid = 1U;

    if (config == NULL)
    {
        if (reason != NULL)
        {
            *reason = LONGITUDINAL_REASON_INVALID_ARGUMENT;
        }
        return 0U;
    }

    if (config->min_pwm_us >= config->center_pwm_us ||
        config->max_pwm_us <= config->center_pwm_us ||
        config->forward_limit_pwm_us < config->center_pwm_us ||
        config->forward_limit_pwm_us > config->max_pwm_us ||
        config->reverse_limit_pwm_us > config->center_pwm_us ||
        config->reverse_limit_pwm_us < config->min_pwm_us ||
        finite_positive(config->target_slew_rate_mps2) == 0U ||
        finite_nonnegative(config->pi_kp_us_per_mps) == 0U ||
        finite_nonnegative(config->pi_ki_us_per_mps_s) == 0U)
    {
        valid = 0U;
    }
    else if (config->pi_enabled != 0U && config->pi_trim_limit_us == 0U)
    {
        valid = 0U;
    }
    else if (finite_positive(config->tracking_brake_kp) == 0U ||
             unit_value(config->tracking_brake_max) == 0U ||
             config->tracking_brake_max <= 0.0f ||
             finite_positive(config->tracking_brake_enter_error_mps) == 0U ||
             finite_nonnegative(config->tracking_brake_release_error_mps) == 0U ||
             config->tracking_brake_release_error_mps >=
                config->tracking_brake_enter_error_mps)
    {
        valid = 0U;
    }

    if (valid == 0U)
    {
        local_reason = LONGITUDINAL_REASON_CONFIG_INVALID;
    }

    if (reason != NULL)
    {
        *reason = local_reason;
    }
    return valid;
}

void LongitudinalController_Init(LongitudinalController_t *controller,
                                 const LongitudinalControllerConfig_t *config)
{
    if (controller == NULL)
    {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config_reason = LONGITUDINAL_REASON_CONFIG_INVALID;
    if (config != NULL)
    {
        (void)LongitudinalController_SetConfig(controller, config);
    }
}

LongitudinalReason_t LongitudinalController_SetConfig(
    LongitudinalController_t *controller,
    const LongitudinalControllerConfig_t *config)
{
    LongitudinalReason_t reason = LONGITUDINAL_REASON_OK;
    uint8_t valid;

    if (controller == NULL || config == NULL)
    {
        return LONGITUDINAL_REASON_INVALID_ARGUMENT;
    }

    valid = LongitudinalController_ConfigIsValid(config, &reason);
    controller->config = *config;
    controller->config_valid = valid;
    controller->config_reason = reason;
    reset_dynamic_state(controller);

    return reason;
}

void LongitudinalController_Reset(LongitudinalController_t *controller)
{
    if (controller == NULL)
    {
        return;
    }
    reset_dynamic_state(controller);
}

static void update_slewed_target(LongitudinalController_t *controller,
                                 float target_mps,
                                 const LongitudinalControllerInput_t *input,
                                 LongitudinalControllerOutput_t *output)
{
    uint32_t dt_ms = 0U;
    float dt_s = 0.0f;
    uint8_t slew_limited = 0U;

    if (target_mps == 0.0f)
    {
        controller->slewed_target_mps = 0.0f;
        controller->has_update_tick = 1U;
        controller->last_update_tick_ms = input->now_tick_ms;
        output->diagnostics.slewed_target_mps = 0.0f;
        return;
    }

    if (controller->has_update_tick != 0U &&
        tick_delta_ms(input->now_tick_ms,
                      controller->last_update_tick_ms,
                      &dt_ms) != 0U)
    {
        dt_s = (float)dt_ms / 1000.0f;
        if (dt_s > 0.250f)
        {
            dt_s = 0.250f;
        }
    }

    if (controller->has_update_tick == 0U)
    {
        controller->slewed_target_mps = 0.0f;
    }
    else
    {
        controller->slewed_target_mps = step_toward(
            controller->slewed_target_mps,
            target_mps,
            controller->config.target_slew_rate_mps2 * dt_s,
            &slew_limited);
    }
    controller->has_update_tick = 1U;
    controller->last_update_tick_ms = input->now_tick_ms;
    output->diagnostics.slew_limited = slew_limited;
    output->diagnostics.slewed_target_mps = controller->slewed_target_mps;
}

static uint8_t feedback_sample_is_new(LongitudinalController_t *controller,
                                      const LongitudinalControllerInput_t *input,
                                      float *dt_s)
{
    uint32_t dt_ms = 0U;
    const uint8_t sample_changed =
        (controller->have_feedback_sample == 0U ||
         controller->last_feedback_sample_id != input->feedback_sample_id ||
         controller->last_feedback_sample_tick_ms !=
            input->feedback_sample_tick_ms) ? 1U : 0U;

    if (dt_s != NULL)
    {
        *dt_s = 0.0f;
    }

    if (sample_changed != 0U &&
        controller->have_feedback_sample != 0U &&
        tick_delta_ms(input->feedback_sample_tick_ms,
                      controller->last_feedback_sample_tick_ms,
                      &dt_ms) != 0U)
    {
        float local_dt_s = (float)dt_ms / 1000.0f;
        if (local_dt_s > 0.100f)
        {
            local_dt_s = 0.100f;
        }
        if (dt_s != NULL)
        {
            *dt_s = local_dt_s;
        }
    }

    if (sample_changed != 0U)
    {
        controller->have_feedback_sample = 1U;
        controller->last_feedback_sample_id = input->feedback_sample_id;
        controller->last_feedback_sample_tick_ms =
            input->feedback_sample_tick_ms;
    }

    return sample_changed;
}

static void reset_pi(LongitudinalController_t *controller)
{
    controller->have_feedback_sample = 0U;
    controller->last_feedback_sample_id = 0U;
    controller->last_feedback_sample_tick_ms = 0U;
    controller->pi_integral_mps_s = 0.0f;
}

static void apply_drive_output(LongitudinalController_t *controller,
                               const LongitudinalControllerInput_t *input,
                               float target_mps,
                               float feedback_mps,
                               LongitudinalControllerOutput_t *output)
{
    const LongitudinalControllerConfig_t *config = &controller->config;
    const LongitudinalDirection_t target_direction =
        direction_from_target(target_mps);
    const uint16_t base_pwm = feed_forward_pwm(config, target_mps);
    int32_t final_pwm = (int32_t)base_pwm;
    float error_mps = target_mps - feedback_mps;
    float dt_s = 0.0f;
    uint8_t new_sample;

    output->intent = LONGITUDINAL_INTENT_DRIVE;
    output->reason = LONGITUDINAL_REASON_OK;
    output->target_direction = target_direction;
    output->propulsion_permitted = 1U;
    output->diagnostics.pi_active = (config->pi_enabled != 0U) ? 1U : 0U;
    output->diagnostics.feedforward_pwm_us = base_pwm;
    output->diagnostics.speed_error_mps = error_mps;

    new_sample = feedback_sample_is_new(controller, input, &dt_s);
    output->diagnostics.duplicate_sample = (new_sample == 0U) ? 1U : 0U;

    if (config->pi_enabled != 0U)
    {
        const float kp = config->pi_kp_us_per_mps;
        const float ki = config->pi_ki_us_per_mps_s;
        const float trim_limit = (float)config->pi_trim_limit_us;
        float integral = controller->pi_integral_mps_s;
        float trim_us;

        if (new_sample != 0U)
        {
            integral += error_mps * dt_s;
        }

        trim_us = kp * error_mps + ki * integral;
        if (trim_us > trim_limit)
        {
            trim_us = trim_limit;
            output->diagnostics.pi_saturated = 1U;
            if (ki > 0.0001f)
            {
                integral = (trim_limit - kp * error_mps) / ki;
            }
        }
        else if (trim_us < -trim_limit)
        {
            trim_us = -trim_limit;
            output->diagnostics.pi_saturated = 1U;
            if (ki > 0.0001f)
            {
                integral = (-trim_limit - kp * error_mps) / ki;
            }
        }

        final_pwm += float_to_i32_nearest(trim_us);
        controller->pi_integral_mps_s = integral;
        output->diagnostics.pi_integral_mps_s = integral;
        output->diagnostics.pi_trim_us = trim_us;
    }
    else
    {
        reset_pi(controller);
    }

    output->drive_pwm_us = limit_drive_pwm(config, target_direction, final_pwm);
    if (output->drive_pwm_us != (uint16_t)final_pwm)
    {
        output->diagnostics.pi_saturated =
            (config->pi_enabled != 0U) ? 1U : output->diagnostics.pi_saturated;
    }
    output->drive_effort = drive_effort_from_pwm(config,
                                                 target_direction,
                                                 output->drive_pwm_us);
}

LongitudinalControllerOutput_t LongitudinalController_Evaluate(
    LongitudinalController_t *controller,
    const LongitudinalControllerInput_t *input)
{
    LongitudinalControllerOutput_t output;
    float target_mps;
    float feedback_mps;
    LongitudinalDirection_t target_direction;
    LongitudinalDirection_t requested_direction;

    if (controller == NULL || input == NULL)
    {
        return make_output(controller,
                           LONGITUDINAL_INTENT_NEUTRAL,
                           LONGITUDINAL_REASON_INVALID_ARGUMENT);
    }

    if (controller->config_valid == 0U)
    {
        output = make_output(controller,
                             LONGITUDINAL_INTENT_NEUTRAL,
                             controller->config_reason);
        reset_dynamic_state(controller);
        return output;
    }

    output = make_output(controller,
                         LONGITUDINAL_INTENT_NEUTRAL,
                         LONGITUDINAL_REASON_OK);
    output.diagnostics.requested_target_mps = input->target_speed_mps;

    if (input->automatic_enabled == 0U)
    {
        reset_dynamic_state(controller);
        return make_output(controller,
                           LONGITUDINAL_INTENT_NEUTRAL,
                           LONGITUDINAL_REASON_NOT_ENABLED);
    }

    if (feedback_is_valid(input) == 0U)
    {
        reset_dynamic_state(controller);
        return make_output(controller,
                           LONGITUDINAL_INTENT_NEUTRAL,
                           LONGITUDINAL_REASON_FEEDBACK_INVALID);
    }

    output.diagnostics.feedback_valid = 1U;
    feedback_mps = signed_feedback_mps(input);
    output.diagnostics.feedback_signed_mps = feedback_mps;

    if (target_is_valid(input->target_speed_mps) == 0U)
    {
        reset_dynamic_state(controller);
        return make_output(controller,
                           LONGITUDINAL_INTENT_NEUTRAL,
                           LONGITUDINAL_REASON_INVALID_ARGUMENT);
    }

    target_mps = (input->stop_requested != 0U) ? 0.0f :
        input->target_speed_mps;
    output.diagnostics.command_target_mps = target_mps;

    requested_direction = direction_from_target(target_mps);
    if (input->stopped == 0U &&
        input->current_direction != LONGITUDINAL_DIRECTION_UNKNOWN &&
        requested_direction != LONGITUDINAL_DIRECTION_UNKNOWN &&
        input->current_direction != requested_direction)
    {
        /* A sign-changing command is a reversal request immediately. Holding
         * the slew reference at zero prevents a fast stop from briefly
         * re-applying propulsion in the old direction while the reference is
         * still crossing zero. After stop confirmation the new direction
         * starts slewing from zero behind the Mode2 neutral-dwell gate. */
        controller->slewed_target_mps = 0.0f;
        controller->has_update_tick = 1U;
        controller->last_update_tick_ms = input->now_tick_ms;
        reset_pi(controller);
        controller->tracking_brake_active = 0U;
        output.intent = LONGITUDINAL_INTENT_REVERSAL_REQUEST;
        output.reason = LONGITUDINAL_REASON_REVERSAL_REQUIRED;
        output.target_direction = requested_direction;
        output.brake_permitted = 1U;
        output.diagnostics.slew_limited = 1U;
        output.diagnostics.slewed_target_mps = 0.0f;
        output.diagnostics.reason = output.reason;
        return output;
    }

    update_slewed_target(controller, target_mps, input, &output);
    target_mps = controller->slewed_target_mps;
    target_direction = direction_from_target(target_mps);
    output.target_direction = target_direction;

    if (target_direction == LONGITUDINAL_DIRECTION_UNKNOWN)
    {
        reset_pi(controller);
        controller->tracking_brake_active = 0U;
        output.diagnostics.tracking_brake_active = 0U;

        if (input->stopped != 0U)
        {
            output.reason = (input->stop_requested != 0U) ?
                LONGITUDINAL_REASON_STOP_REQUESTED :
                LONGITUDINAL_REASON_TARGET_NEUTRAL;
            output.diagnostics.reason = output.reason;
            return output;
        }

        output.intent = LONGITUDINAL_INTENT_STOP_BRAKE;
        output.reason = LONGITUDINAL_REASON_STOP_REQUESTED;
        output.brake_permitted = 1U;
        output.diagnostics.reason = output.reason;
        return output;
    }

    {
        const float tracking_error_mps =
            input->speed_magnitude_mps - fabsf(target_mps);
        output.diagnostics.tracking_error_mps = tracking_error_mps;

        if (controller->tracking_brake_active != 0U)
        {
            if (tracking_error_mps <=
                controller->config.tracking_brake_release_error_mps)
            {
                controller->tracking_brake_active = 0U;
            }
        }
        else if (tracking_error_mps >=
                 controller->config.tracking_brake_enter_error_mps)
        {
            controller->tracking_brake_active = 1U;
        }

        if (controller->tracking_brake_active != 0U)
        {
            float brake_request =
                controller->config.tracking_brake_kp * tracking_error_mps;

            if (brake_request < 0.0f)
            {
                brake_request = 0.0f;
            }
            if (brake_request > controller->config.tracking_brake_max)
            {
                brake_request = controller->config.tracking_brake_max;
            }

            reset_pi(controller);
            output.intent = LONGITUDINAL_INTENT_TRACKING_BRAKE;
            output.reason = LONGITUDINAL_REASON_TRACKING_BRAKE;
            output.target_direction = target_direction;
            output.brake_permitted = 1U;
            output.normalized_brake_request = brake_request;
            output.diagnostics.reason = output.reason;
            output.diagnostics.tracking_brake_active = 1U;
            return output;
        }
    }

    controller->tracking_brake_active = 0U;
    apply_drive_output(controller, input, target_mps, feedback_mps, &output);
    output.diagnostics.reason = output.reason;
    output.diagnostics.tracking_brake_active = 0U;
    return output;
}
