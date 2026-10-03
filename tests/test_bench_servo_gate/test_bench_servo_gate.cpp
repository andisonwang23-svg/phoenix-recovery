#include <unity.h>

#include "logic/bench_servo_gate.h"

using namespace logic;

static BenchServoGateInput baseInput() {
    BenchServoGateInput in;
    in.flight_state = FlightState::SELF_TEST;
    in.servo_healthy = true;
    return in;
}

void sensor_health_is_not_part_of_bench_gate() {
    // No sensor fields exist in the gate by design.
    TEST_ASSERT_TRUE(benchServoTestAllowed(baseInput()));
}

void sensor_failure_state_is_allowed_only_before_launch() {
    auto in = baseInput();
    in.flight_state = FlightState::FAILSAFE_DESCENT;
    TEST_ASSERT_TRUE(benchServoTestAllowed(in));
    in.launch_ms = 10;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
}

void armed_or_recording_test_is_blocked() {
    auto in = baseInput();
    in.armed = true;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
    in.armed = false;
    in.drop_test_recording = true;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
}

void airborne_and_landed_states_are_blocked() {
    auto in = baseInput();
    in.flight_state = FlightState::ASCENT;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
    in.flight_state = FlightState::GUIDED_DESCENT;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
    in.flight_state = FlightState::LANDED;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
}

void unhealthy_servos_are_blocked_but_uptime_does_not_expire_prelaunch_testing() {
    auto in = baseInput();
    in.servo_healthy = false;
    TEST_ASSERT_FALSE(benchServoTestAllowed(in));
    in.servo_healthy = true;
    // The gate has no uptime field: a supervised prelaunch test remains
    // available after five minutes. Individual commands still expire in the
    // LoRa command handler and return the servos to neutral.
    TEST_ASSERT_TRUE(benchServoTestAllowed(in));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(sensor_health_is_not_part_of_bench_gate);
    RUN_TEST(sensor_failure_state_is_allowed_only_before_launch);
    RUN_TEST(armed_or_recording_test_is_blocked);
    RUN_TEST(airborne_and_landed_states_are_blocked);
    RUN_TEST(unhealthy_servos_are_blocked_but_uptime_does_not_expire_prelaunch_testing);
    return UNITY_END();
}
