#include "rc_direction_observer.h"

#include <stdio.h>
#include <string.h>

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

#define TEST_CENTER_PWM_US 1500U
#define TEST_FORWARD_PWM_US 1600U
#define TEST_REVERSE_PWM_US 1400U

static RcDirectionObserverInput_t input(uint32_t sample_id,
                                        uint8_t state_raw,
                                        uint16_t rpm_raw,
                                        uint16_t applied_pwm_us)
{
    RcDirectionObserverInput_t out;

    memset(&out, 0, sizeof(out));
    out.rc_active = 1U;
    out.telemetry_fresh = 1U;
    out.sample_id = sample_id;
    out.state_raw = state_raw;
    out.rpm_valid = 1U;
    out.rpm_raw = rpm_raw;
    out.moving_evidence = (rpm_raw > 0U) ? 1U : 0U;
    out.applied_pwm_us = applied_pwm_us;
    return out;
}

static RcDirectionObserverResult_t step(RcDirectionObserver_t *observer,
                                        uint32_t sample_id,
                                        uint8_t state_raw,
                                        uint16_t rpm_raw,
                                        uint16_t applied_pwm_us)
{
    RcDirectionObserverInput_t in =
        input(sample_id, state_raw, rpm_raw, applied_pwm_us);

    return RcDirectionObserver_Update(observer, &in);
}

static void init_observer(RcDirectionObserver_t *observer)
{
    RcDirectionObserver_Init(observer);
}

static void learn_center(RcDirectionObserver_t *observer)
{
    (void)step(observer, 1U, 0U, 0U, TEST_CENTER_PWM_US);
    (void)step(observer, 2U, 0U, 0U, TEST_CENTER_PWM_US);
}

static void confirm_forward(RcDirectionObserver_t *observer)
{
    learn_center(observer);
    (void)step(observer, 3U, 1U, 320U, TEST_FORWARD_PWM_US);
    (void)step(observer, 4U, 1U, 330U, TEST_FORWARD_PWM_US);
}

static void confirm_reverse(RcDirectionObserver_t *observer)
{
    learn_center(observer);
    (void)step(observer, 3U, 1U, 650U, TEST_REVERSE_PWM_US);
    (void)step(observer, 4U, 1U, 660U, TEST_REVERSE_PWM_US);
}

static int test_neutral_zero_rpm_learns_center_from_two_new_samples(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    result = step(&observer, 1U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 0U);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.processed_new_sample == 1U);

    result = step(&observer, 1U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 0U);
    EXPECT_TRUE(result.processed_new_sample == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_DUPLICATE_SAMPLE);

    result = step(&observer, 2U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 1U);
    EXPECT_TRUE(result.neutral_pwm_us == TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);

    init_observer(&observer);
    (void)step(&observer, 1U, 0U, 0U, TEST_CENTER_PWM_US);
    (void)step(&observer, 2U, 0U, 20U, TEST_CENTER_PWM_US);
    result = step(&observer, 3U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 0U);
    result = step(&observer, 4U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 1U);

    init_observer(&observer);
    (void)step(&observer, 1U, 0U, 0U, TEST_CENTER_PWM_US);
    {
        RcDirectionObserverInput_t stale =
            input(2U, 0U, 0U, TEST_CENTER_PWM_US);
        stale.telemetry_fresh = 0U;
        (void)RcDirectionObserver_Update(&observer, &stale);
    }
    result = step(&observer, 2U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 0U);
    result = step(&observer, 3U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.neutral_learned == 1U);

    return 0;
}

static int test_forward_drive_requires_two_different_samples(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    learn_center(&observer);

    result = step(&observer, 3U, 1U, 300U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING);

    result = step(&observer, 3U, 1U, 300U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_DUPLICATE_SAMPLE);

    result = step(&observer, 4U, 1U, 320U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_OK);

    return 0;
}

static int test_brake_and_neutral_keep_confirmed_forward_direction(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_forward(&observer);

    result = step(&observer, 5U, 2U, 180U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    result = step(&observer, 6U, 0U, 70U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    result = step(&observer, 7U, 0U, 0U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    return 0;
}

static int test_forward_to_reverse_waits_for_brake_or_neutral_and_two_drive_samples(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_forward(&observer);

    result = step(&observer, 5U, 2U, 220U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    result = step(&observer, 6U, 1U, 600U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING);

    result = step(&observer, 7U, 1U, 620U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_REVERSE);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_OK);

    return 0;
}

static int test_repeated_brake_never_confirms_reverse_without_drive(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_forward(&observer);

    result = step(&observer, 5U, 2U, 200U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);
    result = step(&observer, 6U, 2U, 100U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);
    result = step(&observer, 7U, 2U, 0U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    return 0;
}

static int test_reverse_to_forward_does_not_require_neutral(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_reverse(&observer);

    result = step(&observer, 5U, 2U, 500U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_REVERSE);

    result = step(&observer, 6U, 1U, 300U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING);

    result = step(&observer, 7U, 1U, 320U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    return 0;
}

static int test_opposite_drive_without_non_drive_does_not_flip_direction(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_forward(&observer);

    result = step(&observer, 5U, 1U, 650U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_AWAITING_NON_DRIVE);

    result = step(&observer, 6U, 1U, 680U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_AWAITING_NON_DRIVE);

    result = step(&observer, 7U, 2U, 300U, TEST_REVERSE_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    return 0;
}

static int test_stale_unknown_and_rc_inactive_invalidate_direction(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverInput_t in;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_forward(&observer);

    in = input(5U, 1U, 300U, TEST_FORWARD_PWM_US);
    in.telemetry_fresh = 0U;
    result = RcDirectionObserver_Update(&observer, &in);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_TELEMETRY_STALE);
    EXPECT_TRUE(result.neutral_learned == 1U);

    result = step(&observer, 5U, 1U, 300U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_CANDIDATE_PENDING);
    result = step(&observer, 6U, 1U, 320U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);

    result = step(&observer, 7U, 99U, 300U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_ACTION_UNKNOWN);

    in = input(8U, 1U, 300U, TEST_FORWARD_PWM_US);
    in.rc_active = 0U;
    result = RcDirectionObserver_Update(&observer, &in);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_RC_INACTIVE);

    return 0;
}

static int test_drive_requires_learned_center_pwm_side_and_motion(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    result = step(&observer, 1U, 1U, 300U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_NEUTRAL_NOT_LEARNED);

    init_observer(&observer);
    learn_center(&observer);
    result = step(&observer, 3U, 1U, 300U, TEST_CENTER_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_PWM_AT_NEUTRAL);

    result = step(&observer, 4U, 1U, 0U, TEST_FORWARD_PWM_US);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_RPM_NOT_MOVING);

    return 0;
}

static int test_confirmed_same_side_drive_keeps_direction_below_moving_threshold(void)
{
    RcDirectionObserver_t observer;
    RcDirectionObserverInput_t in;
    RcDirectionObserverResult_t result;

    init_observer(&observer);
    confirm_forward(&observer);

    in = input(5U, 1U, 0U, TEST_FORWARD_PWM_US);
    in.moving_evidence = 0U;
    result = RcDirectionObserver_Update(&observer, &in);
    EXPECT_TRUE(result.direction_known == 1U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_FORWARD);
    EXPECT_TRUE(result.reason == RC_DIRECTION_OBSERVER_REASON_RPM_NOT_MOVING);

    in = input(6U, 1U, 0U, TEST_REVERSE_PWM_US);
    in.moving_evidence = 0U;
    result = RcDirectionObserver_Update(&observer, &in);
    EXPECT_TRUE(result.direction_known == 0U);
    EXPECT_TRUE(result.direction == RC_DIRECTION_OBSERVER_DIRECTION_UNKNOWN);

    return 0;
}

int main(void)
{
    int failures = 0;

    if (test_neutral_zero_rpm_learns_center_from_two_new_samples() != 0)
    {
        failures++;
    }
    if (test_forward_drive_requires_two_different_samples() != 0)
    {
        failures++;
    }
    if (test_brake_and_neutral_keep_confirmed_forward_direction() != 0)
    {
        failures++;
    }
    if (test_forward_to_reverse_waits_for_brake_or_neutral_and_two_drive_samples() != 0)
    {
        failures++;
    }
    if (test_repeated_brake_never_confirms_reverse_without_drive() != 0)
    {
        failures++;
    }
    if (test_reverse_to_forward_does_not_require_neutral() != 0)
    {
        failures++;
    }
    if (test_opposite_drive_without_non_drive_does_not_flip_direction() != 0)
    {
        failures++;
    }
    if (test_stale_unknown_and_rc_inactive_invalidate_direction() != 0)
    {
        failures++;
    }
    if (test_drive_requires_learned_center_pwm_side_and_motion() != 0)
    {
        failures++;
    }
    if (test_confirmed_same_side_drive_keeps_direction_below_moving_threshold() != 0)
    {
        failures++;
    }

    if (failures != 0)
    {
        fprintf(stderr, "test_rc_direction_observer: %d failure(s)\n", failures);
        return 1;
    }

    printf("test_rc_direction_observer: all tests passed\n");
    return 0;
}
