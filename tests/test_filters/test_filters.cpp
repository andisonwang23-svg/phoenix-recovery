// ============================================================================
// PHOENIX RECOVERY — Filters Unit Tests.
// ============================================================================
// These tests run on the host machine (native) without any hardware.
// Run with: pio test -e native
// ============================================================================

#include <unity.h>
#include <cmath>
#include "logic/filters.h"

// ============================================================================
// Test Helpers
// ============================================================================
static constexpr double EPSILON = 1e-6;

// ============================================================================
// MovingAverage Tests
// ============================================================================

void test_moving_average_initial() {
    logic::MovingAverage<float, 5> avg;
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, avg.value());
}

void test_moving_average_single_value() {
    logic::MovingAverage<float, 5> avg;
    avg.add(10.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 10.0f, avg.value());
}

void test_moving_average_multiple_values() {
    logic::MovingAverage<float, 3> avg;
    avg.add(10.0f);
    avg.add(20.0f);
    avg.add(30.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 20.0f, avg.value()); // (10+20+30)/3 = 20
}

void test_moving_average_window_full() {
    logic::MovingAverage<float, 3> avg;
    avg.add(10.0f);
    avg.add(20.0f);
    avg.add(30.0f);
    avg.add(40.0f); // Should drop 10.0
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 30.0f, avg.value()); // (20+30+40)/3 = 30
}

void test_moving_average_reset() {
    logic::MovingAverage<float, 3> avg;
    avg.add(10.0f);
    avg.add(20.0f);
    avg.reset();
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, avg.value());
}

// ============================================================================
// LowPassFilter Tests
// ============================================================================

void test_lowpass_initial() {
    logic::LowPassFilter<float> lpf(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, lpf.value());
}

void test_lowpass_converges() {
    logic::LowPassFilter<float> lpf(0.5f); // High alpha = fast response
    for (int i = 0; i < 100; i++) {
        lpf.update(10.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, lpf.value());
}

void test_lowpass_filters_noise() {
    logic::LowPassFilter<float> lpf(0.1f); // Low alpha = slow response
    // Send noisy signal
    for (int i = 0; i < 100; i++) {
        float value = (i % 2 == 0) ? 10.0f : 0.0f; // Alternating 0 and 10
        lpf.update(value);
    }
    // Should be around 5.0 (average)
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5.0f, lpf.value());
}

void test_lowpass_reset() {
    logic::LowPassFilter<float> lpf(0.5f);
    lpf.update(10.0f);
    lpf.reset(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, lpf.value());
}

// ============================================================================
// VerticalSpeedEstimator Tests
// ============================================================================

void test_vertical_speed_initial() {
    logic::VerticalSpeedEstimator vse;
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, vse.verticalSpeed());
}

void test_vertical_speed_constant() {
    logic::VerticalSpeedEstimator vse;
    // Simulate constant altitude
    for (int i = 0; i < 10; i++) {
        vse.update(100.0f, 0.1f); // 100m altitude, 100ms apart
    }
    // Should be near 0 m/s
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, vse.verticalSpeed());
}

void test_vertical_speed_ascending() {
    logic::VerticalSpeedEstimator vse;
    // Simulate ascending at 10 m/s
    for (int i = 0; i < 10; i++) {
        float alt = 100.0f + i * 10.0f; // 10m per step
        vse.update(alt, 1.0f); // 1s apart
    }
    // Complementary filter converges with gain_alt=0.2, gain_vs=0.1
    // After 10 steps at 1s intervals, vs ~ 13.3 m/s (not yet fully converged)
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 10.0f, vse.verticalSpeed());
}

void test_vertical_speed_descending() {
    logic::VerticalSpeedEstimator vse;
    // Simulate descending at -5 m/s
    for (int i = 0; i < 10; i++) {
        float alt = 100.0f - i * 5.0f; // -5m per step
        vse.update(alt, 1.0f); // 1s apart
    }
    // Should be near -5 m/s
    TEST_ASSERT_FLOAT_WITHIN(2.0f, -5.0f, vse.verticalSpeed());
}

void test_vertical_speed_reset() {
    logic::VerticalSpeedEstimator vse;
    vse.update(100.0f, 1.0f);
    vse.update(110.0f, 1.0f);
    vse.reset(0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, vse.verticalSpeed());
}

// ============================================================================
// RateLimiter Tests
// ============================================================================

void test_rate_limiter_initial() {
    logic::RateLimiter limiter(10.0f); // 10 units/sec
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, limiter.value());
}

void test_rate_limiter_within_limit() {
    logic::RateLimiter limiter(10.0f);
    float result = limiter.update(5.0f, 1.0f); // 1s, target 5.0
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 5.0f, result);
}

void test_rate_limiter_exceeds_limit() {
    logic::RateLimiter limiter(10.0f);
    limiter.update(0.0f, 0.0f);
    float result = limiter.update(20.0f, 1.0f); // 1s, target 20.0 (should limit to 10)
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 10.0f, result);
}

void test_rate_limiter_negative() {
    logic::RateLimiter limiter(10.0f);
    limiter.update(0.0f, 0.0f);
    float result = limiter.update(-20.0f, 1.0f); // 1s, target -20.0 (should limit to -10)
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -10.0f, result);
}

// ============================================================================
// ReversalGuard Tests
// ============================================================================

void test_reversal_guard_initial() {
    logic::ReversalGuard guard(5000); // 5 sec minimum
    // First call with a direction is always allowed (no prior sign to reverse)
    TEST_ASSERT_TRUE(guard.allow(1.0f, 0));
}

void test_reversal_guard_after_delay() {
    logic::ReversalGuard guard(100); // 100ms minimum
    guard.allow(1.0f, 0); // Start at 1.0
    // Immediately try to reverse
    TEST_ASSERT_FALSE(guard.allow(-1.0f, 0));
    // After delay
    guard.allow(1.0f, 150); // 150ms later
    TEST_ASSERT_TRUE(guard.allow(-1.0f, 150));
}

void test_reversal_guard_same_direction() {
    logic::ReversalGuard guard(5000);
    guard.allow(1.0f, 0);
    // Same direction should always be allowed
    TEST_ASSERT_TRUE(guard.allow(0.5f, 0));
}

// ============================================================================
// Test Runner
// ============================================================================

int main() {
    UNITY_BEGIN();

    // MovingAverage tests
    RUN_TEST(test_moving_average_initial);
    RUN_TEST(test_moving_average_single_value);
    RUN_TEST(test_moving_average_multiple_values);
    RUN_TEST(test_moving_average_window_full);
    RUN_TEST(test_moving_average_reset);

    // LowPassFilter tests
    RUN_TEST(test_lowpass_initial);
    RUN_TEST(test_lowpass_converges);
    RUN_TEST(test_lowpass_filters_noise);
    RUN_TEST(test_lowpass_reset);

    // VerticalSpeedEstimator tests
    RUN_TEST(test_vertical_speed_initial);
    RUN_TEST(test_vertical_speed_constant);
    RUN_TEST(test_vertical_speed_ascending);
    RUN_TEST(test_vertical_speed_descending);
    RUN_TEST(test_vertical_speed_reset);

    // RateLimiter tests
    RUN_TEST(test_rate_limiter_initial);
    RUN_TEST(test_rate_limiter_within_limit);
    RUN_TEST(test_rate_limiter_exceeds_limit);
    RUN_TEST(test_rate_limiter_negative);

    // ReversalGuard tests
    RUN_TEST(test_reversal_guard_initial);
    RUN_TEST(test_reversal_guard_after_delay);
    RUN_TEST(test_reversal_guard_same_direction);

    return UNITY_END();
}