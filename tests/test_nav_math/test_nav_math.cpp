// ============================================================================
// PHOENIX RECOVERY — Navigation Math Unit Tests.
// ============================================================================
// These tests run on the host machine (native) without any hardware.
// Run with: pio test -e native
// ============================================================================

#include <unity.h>
#include <cmath>
#include "logic/nav_math.h"
#include "logic/filters.h"

// ============================================================================
// Test Helpers
// ============================================================================
static constexpr float EPSILON = 1e-5f;
static constexpr float BEARING_EPSILON = 0.5f; // degrees (haversine rounding at short distances)

// ============================================================================
// Test Cases — Haversine distance
// ============================================================================

void test_haversine_same_point() {
    float dist = (float)logic::distanceMeters(40.0, -74.0, 40.0, -74.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, dist);
}

void test_haversine_known_distance() {
    // New York to Los Angeles ~ 3944 km (haversine rounding at this scale)
    float dist = (float)logic::distanceMeters(40.7128, -74.0060, 34.0522, -118.2437);
    TEST_ASSERT_FLOAT_WITHIN(20000.0f, 3944000.0f, dist); // Within 20km
}

void test_haversine_short_distance() {
    // 1 degree latitude ~ 111 km
    float dist = (float)logic::distanceMeters(40.0, -74.0, 41.0, -74.0);
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 111000.0f, dist); // Within 1km
}

// ============================================================================
// Test Cases — Bearing
// ============================================================================

void test_bearing_north() {
    float bearing = (float)logic::bearingDegrees(40.0, -74.0, 41.0, -74.0);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_EPSILON, 0.0f, bearing);
}

void test_bearing_east() {
    float bearing = (float)logic::bearingDegrees(40.0, -74.0, 40.0, -73.0);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_EPSILON, 90.0f, bearing);
}

void test_bearing_south() {
    float bearing = (float)logic::bearingDegrees(40.0, -74.0, 39.0, -74.0);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_EPSILON, 180.0f, bearing);
}

void test_bearing_west() {
    float bearing = (float)logic::bearingDegrees(40.0, -74.0, 40.0, -75.0);
    TEST_ASSERT_FLOAT_WITHIN(BEARING_EPSILON, 270.0f, bearing);
}

// ============================================================================
// Test Cases — Heading error
// ============================================================================

void test_heading_error_zero() {
    float error = (float)logic::headingErrorDegrees(90.0, 90.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, error);
}

void test_heading_error_positive() {
    // Target is 90 deg right of current heading
    float error = (float)logic::headingErrorDegrees(0.0, 90.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 90.0f, error);
}

void test_heading_error_negative() {
    // Target is 90 deg left of current heading
    float error = (float)logic::headingErrorDegrees(0.0, 270.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -90.0f, error);
}

void test_heading_error_wraparound() {
    // Test crossing 0/360 boundary
    float error = (float)logic::headingErrorDegrees(350.0, 10.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 20.0f, error);
}

void test_heading_error_wraparound_negative() {
    // Test crossing 0/360 boundary (other direction)
    float error = (float)logic::headingErrorDegrees(10.0, 350.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -20.0f, error);
}

// ============================================================================
// Test Cases — Angle normalization (normalizeAngle returns [-180, +180])
// ============================================================================

void test_normalize_angle_positive() {
    float normalized = (float)logic::normalizeAngle(370.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 10.0f, normalized);
}

void test_normalize_angle_negative() {
    // normalizeAngle returns [-180,+180], so -10 stays -10
    float normalized = (float)logic::normalizeAngle(-10.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -10.0f, normalized);
}

void test_normalize_angle_zero() {
    float normalized = (float)logic::normalizeAngle(0.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, normalized);
}

void test_normalize_angle_full_rotation() {
    float normalized = (float)logic::normalizeAngle(360.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, normalized);
}

void test_wrap_angle_270_to_negative() {
    // 270 normalizes to -90 in [-180,+180] range
    float wrapped = (float)logic::normalizeAngle(270.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -90.0f, wrapped);
}

void test_wrap_angle_90() {
    float wrapped = (float)logic::normalizeAngle(90.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 90.0f, wrapped);
}

void test_wrap_angle_450() {
    // 450 = 360+90 -> 90
    float wrapped = (float)logic::normalizeAngle(450.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 90.0f, wrapped);
}

void test_wrap_angle_neg_270() {
    // -270 = -360+90 -> 90
    float wrapped = (float)logic::normalizeAngle(-270.0);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 90.0f, wrapped);
}

// ============================================================================
// Test Cases — Deadband and clamp
// ============================================================================

void test_deadband_within() {
    float result = logic::applyDeadbandAndClamp(0.05f, 0.1f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, result);
}

void test_deadband_outside() {
    float result = logic::applyDeadbandAndClamp(0.2f, 0.1f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.2f, result);
}

void test_deadband_negative_within() {
    float result = logic::applyDeadbandAndClamp(-0.05f, 0.1f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0f, result);
}

void test_deadband_negative_outside() {
    float result = logic::applyDeadbandAndClamp(-0.2f, 0.1f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -0.2f, result);
}

void test_clamp_within() {
    float result = logic::applyDeadbandAndClamp(0.5f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.5f, result);
}

void test_clamp_above() {
    float result = logic::applyDeadbandAndClamp(1.5f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, 1.0f, result);
}

void test_clamp_below() {
    // -0.5 is within [-1, +1], so it passes through clamped
    float result = logic::applyDeadbandAndClamp(-0.5f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -0.5f, result);
}

void test_clamp_symmetric() {
    float result = logic::applyDeadbandAndClamp(-1.5f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPSILON, -1.0f, result);
}

// ============================================================================
// Test Runner
// ============================================================================

int main() {
    UNITY_BEGIN();

    // Haversine distance tests
    RUN_TEST(test_haversine_same_point);
    RUN_TEST(test_haversine_known_distance);
    RUN_TEST(test_haversine_short_distance);

    // Bearing tests
    RUN_TEST(test_bearing_north);
    RUN_TEST(test_bearing_east);
    RUN_TEST(test_bearing_south);
    RUN_TEST(test_bearing_west);

    // Heading error tests
    RUN_TEST(test_heading_error_zero);
    RUN_TEST(test_heading_error_positive);
    RUN_TEST(test_heading_error_negative);
    RUN_TEST(test_heading_error_wraparound);
    RUN_TEST(test_heading_error_wraparound_negative);

    // Angle normalization tests
    RUN_TEST(test_normalize_angle_positive);
    RUN_TEST(test_normalize_angle_negative);
    RUN_TEST(test_normalize_angle_zero);
    RUN_TEST(test_normalize_angle_full_rotation);

    // Wrap angle tests
    RUN_TEST(test_wrap_angle_270_to_negative);
    RUN_TEST(test_wrap_angle_90);
    RUN_TEST(test_wrap_angle_450);
    RUN_TEST(test_wrap_angle_neg_270);

    // Deadband tests
    RUN_TEST(test_deadband_within);
    RUN_TEST(test_deadband_outside);
    RUN_TEST(test_deadband_negative_within);
    RUN_TEST(test_deadband_negative_outside);

    // Clamp tests
    RUN_TEST(test_clamp_within);
    RUN_TEST(test_clamp_above);
    RUN_TEST(test_clamp_below);
    RUN_TEST(test_clamp_symmetric);

    return UNITY_END();
}
