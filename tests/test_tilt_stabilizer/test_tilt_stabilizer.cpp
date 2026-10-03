#include <unity.h>

#include "logic/tilt_stabilizer.h"
#include "../../firmware/src/logic/tilt_stabilizer.cpp"

using namespace logic;

static TiltStabilizerInput nominal(uint32_t now_ms = 100) {
    TiltStabilizerInput in;
    in.now_ms = now_ms;
    in.dt_s = 0.1f;
    in.command_fresh = true;
    in.preflight_allowed = true;
    in.imu_valid = true;
    in.barometer_valid = true;
    in.servo_valid = true;
    return in;
}

void disabled_is_neutral() {
    TiltStabilizer controller;
    const auto out = controller.update(nominal());
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_TRUE(out.immediate_neutral);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.left_brake);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.right_brake);
}

void captures_reference_and_uses_deadband() {
    TiltStabilizer controller;
    controller.arm(100, 7.0f);
    auto in = nominal(200);
    in.roll_deg = 8.0f;
    const auto out = controller.update(in);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.0f, out.reference_roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.signed_command);
}

void positive_and_negative_tilt_command_opposite_brakes() {
    TiltStabilizer controller;
    controller.arm(100, 0.0f);
    auto in = nominal(200);
    in.roll_deg = 10.0f;
    auto out = controller.update(in);
    TEST_ASSERT_TRUE(out.left_brake > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.right_brake);

    controller.arm(300, 0.0f);
    in = nominal(400);
    in.roll_deg = -10.0f;
    out = controller.update(in);
    TEST_ASSERT_TRUE(out.right_brake > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out.left_brake);
}

void authority_and_slew_are_limited() {
    TiltStabilizerConfig cfg;
    cfg.max_brake_command = 0.15f;
    cfg.command_rate_limit_per_s = 0.20f;
    TiltStabilizer controller(cfg);
    controller.arm(100, 0.0f);
    auto in = nominal(200);
    in.dt_s = 0.1f;
    in.roll_deg = 20.0f;
    in.roll_rate_dps = 10.0f;
    const auto first = controller.update(in);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.02f, first.signed_command);
    for (uint32_t n = 0; n < 20; ++n) {
        in.now_ms += 100;
        controller.update(in);
    }
    const auto last = controller.update(in);
    TEST_ASSERT_LESS_OR_EQUAL(0.1501f, last.signed_command);
}

void missing_sensor_or_motion_forces_immediate_neutral() {
    TiltStabilizer controller;
    controller.arm(100, 0.0f);
    auto in = nominal(200);
    in.roll_deg = 10.0f;
    TEST_ASSERT_TRUE(controller.update(in).active);
    in.now_ms = 300;
    in.imu_valid = false;
    auto out = controller.update(in);
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::SENSOR_INVALID, (int)out.status);
    TEST_ASSERT_TRUE(out.immediate_neutral);

    controller.arm(400, 0.0f);
    in = nominal(500);
    in.vertical_speed_mps = -1.0f;
    out = controller.update(in);
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::MOTION_BLOCKED, (int)out.status);
    TEST_ASSERT_TRUE(out.immediate_neutral);
}

void launch_like_acceleration_and_excess_tilt_are_blocked() {
    TiltStabilizer controller;
    controller.arm(100, 0.0f);
    auto in = nominal(200);
    in.vertical_accel_mps2 = 3.0f;
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::MOTION_BLOCKED,
                      (int)controller.update(in).status);

    controller.arm(300, 0.0f);
    in = nominal(400);
    in.roll_deg = 30.0f;
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::ATTITUDE_LIMIT,
                      (int)controller.update(in).status);
}

void timeout_duration_and_interlock_are_neutral() {
    TiltStabilizer controller;
    controller.arm(100, 0.0f);
    auto in = nominal(200);
    in.command_fresh = false;
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::COMMAND_TIMEOUT,
                      (int)controller.update(in).status);

    controller.arm(100, 0.0f);
    in = nominal(16000);
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::DURATION_EXPIRED,
                      (int)controller.update(in).status);

    controller.arm(20000, 0.0f);
    in = nominal(20100);
    in.preflight_allowed = false;
    TEST_ASSERT_EQUAL((int)TiltStabilizerStatus::INTERLOCK,
                      (int)controller.update(in).status);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(disabled_is_neutral);
    RUN_TEST(captures_reference_and_uses_deadband);
    RUN_TEST(positive_and_negative_tilt_command_opposite_brakes);
    RUN_TEST(authority_and_slew_are_limited);
    RUN_TEST(missing_sensor_or_motion_forces_immediate_neutral);
    RUN_TEST(launch_like_acceleration_and_excess_tilt_are_blocked);
    RUN_TEST(timeout_duration_and_interlock_are_neutral);
    return UNITY_END();
}
