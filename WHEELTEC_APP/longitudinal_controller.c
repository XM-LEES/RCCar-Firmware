#include "longitudinal_controller.h"

#include <math.h>
#include <string.h>

#define LONGITUDINAL_TICK_HALF_RANGE 0x80000000UL
#define LONGITUDINAL_EPSILON 0.0001f

static uint8_t finite_value(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static uint8_t finite_nonnegative(float value)
{
    return (isfinite(value) && value >= 0.0f) ? 1U : 0U;
}

static uint8_t finite_positive(float value)
{
    return (isfinite(value) && value > 0.0f) ? 1U : 0U;
}

static uint8_t direction_is_valid(LongitudinalDirection_t direction)
{
    return (direction == LONGITUDINAL_DIRECTION_FORWARD ||
            direction == LONGITUDINAL_DIRECTION_REVERSE ||
            direction == LONGITUDINAL_DIRECTION_UNKNOWN) ? 1U : 0U;
}

static LongitudinalDirection_t direction_from_signed(float value)
{
    if (value > LONGITUDINAL_EPSILON)
    {
        return LONGITUDINAL_DIRECTION_FORWARD;
    }
    if (value < -LONGITUDINAL_EPSILON)
    {
        return LONGITUDINAL_DIRECTION_REVERSE;
    }
    return LONGITUDINAL_DIRECTION_UNKNOWN;
}

static uint8_t direction_matches_speed(LongitudinalDirection_t direction,
                                       float signed_speed_mps)
{
    if (direction == LONGITUDINAL_DIRECTION_FORWARD)
    {
        return (signed_speed_mps >= -LONGITUDINAL_EPSILON) ? 1U : 0U;
    }
    if (direction == LONGITUDINAL_DIRECTION_REVERSE)
    {
        return (signed_speed_mps <= LONGITUDINAL_EPSILON) ? 1U : 0U;
    }
    return (fabsf(signed_speed_mps) <= LONGITUDINAL_EPSILON) ? 1U : 0U;
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

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint8_t output_bounds_are_valid(float minimum, float maximum)
{
    return (isfinite(minimum) && isfinite(maximum) &&
            minimum < 0.0f && maximum > 0.0f &&
            minimum < maximum) ? 1U : 0U;
}

static uint8_t applied_output_bounds_are_valid(
    const LongitudinalController_t *controller,
    float minimum,
    float maximum)
{
    return (controller != NULL &&
            isfinite(minimum) && isfinite(maximum) &&
            minimum <= maximum &&
            minimum >= controller->config.min_output_us &&
            maximum <= controller->config.max_output_us) ? 1U : 0U;
}

static uint8_t same_feedback_sample(const LongitudinalController_t *controller,
                                    const LongitudinalFeedbackSample_t *sample)
{
    return (controller->feedback_valid != 0U &&
            controller->feedback_sample_id == sample->sample_id &&
            controller->feedback_sample_tick_ms == sample->tick_ms) ? 1U : 0U;
}

static uint8_t latest_sample_is_committed(
    const LongitudinalController_t *controller)
{
    return (controller->pid_sample_committed != 0U &&
            controller->pid_committed_sample_id ==
                controller->feedback_sample_id &&
            controller->pid_committed_sample_tick_ms ==
                controller->feedback_sample_tick_ms) ? 1U : 0U;
}

static void clear_dynamic_state(LongitudinalController_t *controller)
{
    controller->feedback_valid = 0U;
    controller->feedback_duplicate = 0U;
    controller->derivative_valid = 0U;
    controller->feedback_sample_id = 0U;
    controller->feedback_sample_tick_ms = 0U;
    controller->feedback_signed_mps = 0.0f;
    controller->previous_feedback_signed_mps = 0.0f;
    controller->feedback_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    controller->measured_accel_mps2 = 0.0f;
    controller->pid_sample_committed = 0U;
    controller->pid_committed_sample_id = 0U;
    controller->pid_committed_sample_tick_ms = 0U;
    controller->have_target = 0U;
    controller->last_target_speed_mps = 0.0f;
    controller->last_target_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    controller->skip_error_integral_once = 0U;
    controller->integral_us = 0.0f;
    controller->last_integral_saturated = 0U;
    controller->last_antiwindup_active = 0U;
    controller->last_applied_output_us = 0.0f;
    controller->pending_valid = 0U;
    controller->pending_new_sample = 0U;
    controller->pending_skip_error_integral = 0U;
    controller->pending_dt_s = 0.0f;
    controller->pending_error_mps = 0.0f;
    controller->pending_raw_output_us = 0.0f;
}

static void invalidate_feedback(LongitudinalController_t *controller)
{
    controller->feedback_valid = 0U;
    controller->feedback_duplicate = 0U;
    controller->derivative_valid = 0U;
    controller->measured_accel_mps2 = 0.0f;
    controller->pid_sample_committed = 0U;
    controller->pid_committed_sample_id = 0U;
    controller->pid_committed_sample_tick_ms = 0U;
    controller->pending_valid = 0U;
    controller->pending_new_sample = 0U;
}

static LongitudinalControllerOutput_t make_output(
    const LongitudinalController_t *controller,
    LongitudinalReason_t reason)
{
    LongitudinalControllerOutput_t output;

    memset(&output, 0, sizeof(output));
    output.reason = reason;
    output.target_direction = LONGITUDINAL_DIRECTION_UNKNOWN;
    output.diagnostics.reason = reason;
    output.diagnostics.config_valid =
        (controller != NULL && controller->config_valid != 0U) ? 1U : 0U;
    output.diagnostics.inhibited =
        (reason == LONGITUDINAL_REASON_OK) ? 0U : 1U;
    if (controller != NULL)
    {
        output.diagnostics.integral_us = controller->integral_us;
        output.diagnostics.integral_saturated =
            controller->last_integral_saturated;
        output.diagnostics.antiwindup_active =
            controller->last_antiwindup_active;
        output.diagnostics.applied_output_us =
            controller->last_applied_output_us;
        output.diagnostics.measured_accel_mps2 =
            controller->measured_accel_mps2;
        output.diagnostics.derivative_valid =
            controller->derivative_valid;
        output.diagnostics.min_output_us =
            controller->config.min_output_us;
        output.diagnostics.max_output_us =
            controller->config.max_output_us;
    }
    return output;
}

uint8_t LongitudinalController_ConfigIsValid(
    const LongitudinalControllerConfig_t *config,
    LongitudinalReason_t *reason)
{
    uint8_t valid = 1U;

    if (config == NULL)
    {
        if (reason != NULL)
        {
            *reason = LONGITUDINAL_REASON_INVALID_ARGUMENT;
        }
        return 0U;
    }

    if (finite_positive(config->kp_us_per_mps) == 0U ||
        finite_nonnegative(config->ki_us_per_mps_s) == 0U ||
        finite_nonnegative(config->kd_us_per_mps2) == 0U ||
        finite_positive(config->derivative_tau_s) == 0U ||
        finite_value(config->antiwindup_tau_s) == 0U ||
        config->antiwindup_tau_s <= 0.0f ||
        output_bounds_are_valid(config->min_output_us,
                                config->max_output_us) == 0U ||
        config->feedback_freshness_ms == 0U ||
        config->feedback_freshness_ms >= LONGITUDINAL_TICK_HALF_RANGE)
    {
        valid = 0U;
    }

    if (reason != NULL)
    {
        *reason = (valid != 0U) ? LONGITUDINAL_REASON_OK :
            LONGITUDINAL_REASON_CONFIG_INVALID;
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

    if (controller == NULL || config == NULL)
    {
        return LONGITUDINAL_REASON_INVALID_ARGUMENT;
    }

    controller->config = *config;
    controller->config_valid =
        LongitudinalController_ConfigIsValid(config, &reason);
    controller->config_reason = reason;
    clear_dynamic_state(controller);

    return reason;
}

void LongitudinalController_Reset(LongitudinalController_t *controller)
{
    if (controller == NULL)
    {
        return;
    }
    clear_dynamic_state(controller);
}

void LongitudinalController_ClearIntegral(LongitudinalController_t *controller)
{
    if (controller == NULL)
    {
        return;
    }
    controller->integral_us = 0.0f;
    controller->skip_error_integral_once = 0U;
}

LongitudinalReason_t LongitudinalController_ObserveFeedback(
    LongitudinalController_t *controller,
    const LongitudinalFeedbackSample_t *sample)
{
    uint32_t dt_ms = 0U;
    uint8_t derivative_can_update = 0U;

    if (controller == NULL || sample == NULL)
    {
        return LONGITUDINAL_REASON_INVALID_ARGUMENT;
    }

    controller->feedback_duplicate = 0U;

    if (controller->config_valid == 0U)
    {
        invalidate_feedback(controller);
        return controller->config_reason;
    }

    if (sample->valid == 0U ||
        finite_value(sample->signed_speed_mps) == 0U ||
        direction_is_valid(sample->direction) == 0U ||
        direction_matches_speed(sample->direction,
                                sample->signed_speed_mps) == 0U)
    {
        invalidate_feedback(controller);
        return LONGITUDINAL_REASON_FEEDBACK_INVALID;
    }

    if (same_feedback_sample(controller, sample) != 0U)
    {
        controller->feedback_duplicate = 1U;
        return LONGITUDINAL_REASON_OK;
    }

    if (controller->feedback_valid != 0U)
    {
        if (tick_delta_ms(sample->tick_ms,
                          controller->feedback_sample_tick_ms,
                          &dt_ms) == 0U)
        {
            invalidate_feedback(controller);
            return LONGITUDINAL_REASON_TIMEBASE_INVALID;
        }
        if (dt_ms > 0U &&
            dt_ms <= controller->config.feedback_freshness_ms &&
            controller->feedback_direction != LONGITUDINAL_DIRECTION_UNKNOWN &&
            sample->direction != LONGITUDINAL_DIRECTION_UNKNOWN &&
            controller->feedback_direction == sample->direction)
        {
            derivative_can_update = 1U;
        }
    }

    controller->previous_feedback_signed_mps =
        controller->feedback_signed_mps;
    controller->feedback_signed_mps = sample->signed_speed_mps;
    controller->feedback_direction = sample->direction;
    controller->feedback_sample_id = sample->sample_id;
    controller->feedback_sample_tick_ms = sample->tick_ms;
    controller->feedback_valid = 1U;

    if (derivative_can_update != 0U)
    {
        const float dt_s = (float)dt_ms / 1000.0f;
        const float raw_accel =
            (controller->feedback_signed_mps -
             controller->previous_feedback_signed_mps) / dt_s;
        float alpha = 1.0f;

        if (controller->config.derivative_tau_s > 0.0f)
        {
            alpha = dt_s / (controller->config.derivative_tau_s + dt_s);
        }
        controller->measured_accel_mps2 +=
            alpha * (raw_accel - controller->measured_accel_mps2);
        controller->derivative_valid = 1U;
    }
    else
    {
        controller->measured_accel_mps2 = 0.0f;
        controller->derivative_valid = 0U;
    }

    return LONGITUDINAL_REASON_OK;
}

static uint8_t feedback_age_is_valid(const LongitudinalController_t *controller,
                                     uint32_t now_tick_ms,
                                     uint32_t *age_ms)
{
    uint32_t local_age_ms = 0U;

    if (tick_delta_ms(now_tick_ms,
                      controller->feedback_sample_tick_ms,
                      &local_age_ms) == 0U)
    {
        return 0U;
    }
    if (age_ms != NULL)
    {
        *age_ms = local_age_ms;
    }
    return 1U;
}

static float pid_sample_dt_s(LongitudinalController_t *controller,
                             uint8_t *new_sample)
{
    uint32_t dt_ms = 0U;

    if (new_sample != NULL)
    {
        *new_sample = 0U;
    }
    if (controller->feedback_valid == 0U ||
        latest_sample_is_committed(controller) != 0U)
    {
        return 0.0f;
    }

    if (new_sample != NULL)
    {
        *new_sample = 1U;
    }
    if (controller->pid_sample_committed == 0U)
    {
        return 0.0f;
    }
    if (tick_delta_ms(controller->feedback_sample_tick_ms,
                      controller->pid_committed_sample_tick_ms,
                      &dt_ms) == 0U)
    {
        return 0.0f;
    }
    return (float)dt_ms / 1000.0f;
}

static void update_target_state(LongitudinalController_t *controller,
                                float target_speed_mps,
                                LongitudinalControllerOutput_t *output)
{
    const LongitudinalDirection_t target_direction =
        direction_from_signed(target_speed_mps);

    if (controller->have_target != 0U &&
        target_speed_mps != controller->last_target_speed_mps)
    {
        output->diagnostics.target_changed = 1U;
        controller->skip_error_integral_once = 1U;
        if (target_direction != LONGITUDINAL_DIRECTION_UNKNOWN &&
            controller->last_target_direction !=
                LONGITUDINAL_DIRECTION_UNKNOWN &&
            target_direction != controller->last_target_direction)
        {
            output->diagnostics.target_sign_changed = 1U;
            controller->integral_us = 0.0f;
        }
    }

    controller->have_target = 1U;
    controller->last_target_speed_mps = target_speed_mps;
    controller->last_target_direction = target_direction;
    output->target_direction = target_direction;
}

LongitudinalControllerOutput_t LongitudinalController_Evaluate(
    LongitudinalController_t *controller,
    const LongitudinalControllerInput_t *input)
{
    LongitudinalControllerOutput_t output;
    uint32_t age_ms = 0U;
    uint8_t new_sample = 0U;
    float dt_s = 0.0f;
    float raw_output_us = 0.0f;
    float derivative_us = 0.0f;

    if (controller == NULL || input == NULL)
    {
        return make_output(controller, LONGITUDINAL_REASON_INVALID_ARGUMENT);
    }

    if (controller->config_valid == 0U)
    {
        controller->pending_valid = 0U;
        return make_output(controller, controller->config_reason);
    }

    output = make_output(controller, LONGITUDINAL_REASON_OK);
    output.diagnostics.requested_target_mps = input->target_speed_mps;

    if (input->enabled == 0U)
    {
        controller->pending_valid = 0U;
        LongitudinalController_ClearIntegral(controller);
        return make_output(controller, LONGITUDINAL_REASON_NOT_ENABLED);
    }
    if (finite_value(input->target_speed_mps) == 0U)
    {
        controller->pending_valid = 0U;
        return make_output(controller, LONGITUDINAL_REASON_INVALID_ARGUMENT);
    }
    if (controller->feedback_valid == 0U)
    {
        controller->pending_valid = 0U;
        LongitudinalController_ClearIntegral(controller);
        return make_output(controller, LONGITUDINAL_REASON_FEEDBACK_INVALID);
    }
    if (feedback_age_is_valid(controller, input->now_tick_ms, &age_ms) == 0U)
    {
        invalidate_feedback(controller);
        LongitudinalController_ClearIntegral(controller);
        return make_output(controller, LONGITUDINAL_REASON_TIMEBASE_INVALID);
    }
    if (age_ms > controller->config.feedback_freshness_ms)
    {
        invalidate_feedback(controller);
        LongitudinalController_ClearIntegral(controller);
        return make_output(controller, LONGITUDINAL_REASON_FEEDBACK_STALE);
    }

    update_target_state(controller, input->target_speed_mps, &output);
    dt_s = pid_sample_dt_s(controller, &new_sample);
    output.valid = 1U;
    output.reason = LONGITUDINAL_REASON_OK;
    output.diagnostics.reason = LONGITUDINAL_REASON_OK;
    output.diagnostics.inhibited = 0U;
    output.diagnostics.feedback_valid = 1U;
    output.diagnostics.feedback_fresh = 1U;
    output.diagnostics.feedback_sample_id = controller->feedback_sample_id;
    output.diagnostics.feedback_sample_tick_ms =
        controller->feedback_sample_tick_ms;
    output.diagnostics.feedback_age_ms = age_ms;
    output.diagnostics.feedback_signed_mps =
        controller->feedback_signed_mps;
    output.diagnostics.new_sample = new_sample;
    output.diagnostics.duplicate_sample = (new_sample == 0U) ? 1U : 0U;
    output.diagnostics.derivative_valid = controller->derivative_valid;
    output.diagnostics.measured_accel_mps2 =
        controller->measured_accel_mps2;
    output.diagnostics.speed_error_mps =
        input->target_speed_mps - controller->feedback_signed_mps;
    output.diagnostics.proportional_us =
        controller->config.kp_us_per_mps *
        output.diagnostics.speed_error_mps;
    if (controller->derivative_valid != 0U)
    {
        derivative_us = -controller->config.kd_us_per_mps2 *
            controller->measured_accel_mps2;
    }
    output.diagnostics.derivative_us = derivative_us;
    output.diagnostics.integral_us = controller->integral_us;
    output.diagnostics.integral_saturated =
        controller->last_integral_saturated;
    output.diagnostics.antiwindup_active =
        controller->last_antiwindup_active;
    output.diagnostics.applied_output_us =
        controller->last_applied_output_us;
    output.diagnostics.pid_active = 1U;

    raw_output_us = output.diagnostics.proportional_us +
        controller->integral_us + derivative_us;
    output.raw_output_us = raw_output_us;
    output.bounded_output_us = clamp_float(raw_output_us,
                                           controller->config.min_output_us,
                                           controller->config.max_output_us);
    output.diagnostics.raw_output_us = raw_output_us;
    output.diagnostics.bounded_output_us = output.bounded_output_us;
    output.diagnostics.output_saturated =
        (output.bounded_output_us != raw_output_us) ? 1U : 0U;
    output.diagnostics.min_output_us = controller->config.min_output_us;
    output.diagnostics.max_output_us = controller->config.max_output_us;

    controller->pending_valid = 1U;
    controller->pending_new_sample = new_sample;
    controller->pending_skip_error_integral =
        controller->skip_error_integral_once;
    controller->pending_dt_s = dt_s;
    controller->pending_error_mps = output.diagnostics.speed_error_mps;
    controller->pending_raw_output_us = raw_output_us;

    if (controller->pending_skip_error_integral != 0U)
    {
        output.diagnostics.error_integral_skipped = 1U;
    }

    return output;
}

LongitudinalReason_t LongitudinalController_CommitApplied(
    LongitudinalController_t *controller,
    const LongitudinalAppliedOutput_t *applied)
{
    float min_output_us;
    float max_output_us;
    float applied_output_us;

    if (controller == NULL || applied == NULL)
    {
        return LONGITUDINAL_REASON_INVALID_ARGUMENT;
    }
    if (controller->config_valid == 0U)
    {
        return controller->config_reason;
    }
    if (applied_output_bounds_are_valid(controller,
                                        applied->min_output_us,
                                        applied->max_output_us) == 0U ||
        finite_value(applied->applied_output_us) == 0U)
    {
        return LONGITUDINAL_REASON_INVALID_ARGUMENT;
    }

    min_output_us = applied->min_output_us;
    max_output_us = applied->max_output_us;
    applied_output_us = clamp_float(applied->applied_output_us,
                                    min_output_us,
                                    max_output_us);
    controller->last_applied_output_us = applied_output_us;
    controller->last_integral_saturated = 0U;
    controller->last_antiwindup_active = 0U;

    if (applied->reset_integral != 0U ||
        applied->pid_active == 0U ||
        controller->config.ki_us_per_mps_s == 0.0f)
    {
        controller->integral_us = 0.0f;
    }
    else if (controller->pending_valid != 0U &&
             controller->pending_new_sample != 0U &&
             controller->pending_dt_s > 0.0f)
    {
        const float dt_s = controller->pending_dt_s;
        const uint8_t error_worsens_high =
            (controller->pending_raw_output_us > max_output_us &&
             controller->pending_error_mps > 0.0f) ? 1U : 0U;
        const uint8_t error_worsens_low =
            (controller->pending_raw_output_us < min_output_us &&
             controller->pending_error_mps < 0.0f) ? 1U : 0U;
        float next_integral = controller->integral_us;
        float unclamped_integral;
        float beta = dt_s / controller->config.antiwindup_tau_s;

        if (beta > 1.0f)
        {
            beta = 1.0f;
        }

        if (controller->pending_skip_error_integral == 0U &&
            error_worsens_high == 0U &&
            error_worsens_low == 0U)
        {
            next_integral += controller->config.ki_us_per_mps_s *
                controller->pending_error_mps * dt_s;
        }
        if (fabsf(applied_output_us -
                  controller->pending_raw_output_us) > LONGITUDINAL_EPSILON)
        {
            controller->last_antiwindup_active = 1U;
        }
        next_integral += beta *
            (applied_output_us - controller->pending_raw_output_us);
        unclamped_integral = next_integral;
        controller->integral_us = clamp_float(unclamped_integral,
                                              min_output_us,
                                              max_output_us);
        controller->last_integral_saturated =
            (controller->integral_us != unclamped_integral) ? 1U : 0U;
    }

    if (controller->pending_valid != 0U &&
        controller->pending_new_sample != 0U)
    {
        controller->pid_sample_committed = 1U;
        controller->pid_committed_sample_id = controller->feedback_sample_id;
        controller->pid_committed_sample_tick_ms =
            controller->feedback_sample_tick_ms;
        if (controller->pending_skip_error_integral != 0U)
        {
            controller->skip_error_integral_once = 0U;
        }
    }

    controller->pending_valid = 0U;
    controller->pending_new_sample = 0U;
    controller->pending_skip_error_integral = 0U;
    controller->pending_dt_s = 0.0f;
    controller->pending_error_mps = 0.0f;
    controller->pending_raw_output_us = 0.0f;

    return LONGITUDINAL_REASON_OK;
}
