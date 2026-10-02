#include <unity.h>
#include <limits>

#include "logic/drop_test_controller.h"
#include "../../firmware/src/logic/drop_test_controller.cpp"

using namespace logic;

static DropTestConfig config() {
    DropTestConfig c;
    c.release_speed_mps = -1.0f;
    c.release_altitude_loss_m = 1.0f;
    c.release_confirm_ms = 200;
    c.canopy_observation_delay_ms = 100;
    c.canopy_deceleration_delta_mps = 1.0f;
    c.canopy_accel_signature_mps2 = 3.0f;
    c.stable_descent_window_ms = 200;
    c.stable_descent_max_vs_range_mps = 0.5f;
    c.stable_descent_max_angular_rate_dps = 10.0f;
    c.stable_descent_min_down_speed_mps = 0.2f;
    c.landing_max_vertical_speed_mps = 0.2f;
    c.landing_max_angular_rate_dps = 5.0f;
    c.landing_max_ground_speed_mps = 0.5f;
    c.landing_max_altitude_span_m = 0.3f;
    c.landing_confirm_ms = 200;
    c.post_landing_record_ms = 100;
    c.max_duration_ms = 5000;
    return c;
}

static DropTestInput sample(uint32_t now, float altitude, float vs) {
    DropTestInput in;
    in.now_ms = now;
    in.altitude_agl_m = altitude;
    in.vertical_speed_mps = vs;
    in.vertical_accel_mps2 = 0.0f;
    in.angular_rate_dps = 2.0f;
    in.ground_speed_mps = 0.2f;
    in.barometer_valid = true;
    in.imu_valid = true;
    in.gps_valid = true;
    return in;
}

static void confirmDrop(DropTestController& controller) {
    controller.arm(100, 10.0f);
    controller.step(sample(200, 8.5f, -2.0f));
    auto out = controller.step(sample(400, 8.0f, -2.0f));
    TEST_ASSERT_EQUAL((int)DropTestState::DROP_CONFIRMED, (int)out.state);
    TEST_ASSERT_TRUE((out.events & DROP_EVENT_RELEASE_CONFIRMED) != 0);
}

void arm_always_enables_recording_and_neutral_lock() {
    DropTestController controller(config());
    auto out = controller.arm(100, 10.0f);
    TEST_ASSERT_TRUE(out.recording);
    TEST_ASSERT_TRUE(out.neutral_lock);
    TEST_ASSERT_EQUAL((int)DropTestState::ARMED_RECORDING, (int)out.state);
}

void release_requires_persistent_speed_and_altitude_loss() {
    DropTestController controller(config());
    controller.arm(100, 10.0f);
    auto out = controller.step(sample(200, 8.5f, -2.0f));
    TEST_ASSERT_EQUAL((int)DropTestState::RELEASE_CANDIDATE, (int)out.state);
    out = controller.step(sample(250, 8.5f, 0.0f));
    TEST_ASSERT_EQUAL((int)DropTestState::ARMED_RECORDING, (int)out.state);
    TEST_ASSERT_TRUE((out.events & DROP_EVENT_RELEASE_REJECTED) != 0);
    controller.step(sample(300, 8.5f, -2.0f));
    out = controller.step(sample(501, 8.0f, -2.0f));
    TEST_ASSERT_EQUAL((int)DropTestState::DROP_CONFIRMED, (int)out.state);
}

void impossible_values_cannot_trigger_release() {
    DropTestController controller(config());
    controller.arm(100, 10.0f);
    auto in = sample(200, 8.0f, -2.0f);
    in.vertical_speed_mps = std::numeric_limits<float>::quiet_NaN();
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)DropTestState::ARMED_RECORDING, (int)out.state);
}

void canopy_signature_is_observational_and_timestamped() {
    DropTestController controller(config());
    confirmDrop(controller);
    auto in = sample(550, 7.5f, -0.8f);
    in.vertical_accel_mps2 = 4.0f;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)DropTestState::CANOPY_EVENT_SUSPECTED, (int)out.state);
    TEST_ASSERT_TRUE(out.canopy_signature_observed);
    TEST_ASSERT_EQUAL_UINT32(550, out.canopy_signature_ms);
    TEST_ASSERT_TRUE(out.neutral_lock);
}

void stable_descent_requires_a_quiet_persistent_window() {
    DropTestController controller(config());
    confirmDrop(controller);
    auto noisy = sample(500, 7.5f, -1.0f);
    noisy.angular_rate_dps = 30.0f;
    controller.step(noisy);
    controller.step(sample(600, 7.0f, -1.0f));
    auto out = controller.step(sample(801, 6.8f, -0.9f));
    TEST_ASSERT_EQUAL((int)DropTestState::STABLE_DESCENT_OBSERVED, (int)out.state);
    TEST_ASSERT_TRUE(out.stable_descent_observed);
    TEST_ASSERT_TRUE(out.neutral_lock);
}

void landing_requires_low_motion_altitude_span_and_persistence() {
    DropTestController controller(config());
    confirmDrop(controller);
    auto out = controller.step(sample(600, 0.10f, 0.10f));
    TEST_ASSERT_EQUAL((int)DropTestState::LANDING_CONFIRM, (int)out.state);
    out = controller.step(sample(700, 0.55f, 0.10f));
    TEST_ASSERT_EQUAL((int)DropTestState::LANDING_CONFIRM, (int)out.state);
    out = controller.step(sample(801, 0.55f, 0.10f));
    TEST_ASSERT_EQUAL((int)DropTestState::LANDING_CONFIRM, (int)out.state);
    TEST_ASSERT_EQUAL_UINT32(0, out.landing_confirm_ms);

    auto moving = sample(810, 0.55f, 1.0f);
    out = controller.step(moving);
    TEST_ASSERT_TRUE((out.events & DROP_EVENT_LANDING_REJECTED) != 0);

    controller.step(sample(900, 0.10f, 0.05f));
    out = controller.step(sample(1101, 0.20f, 0.05f));
    TEST_ASSERT_EQUAL((int)DropTestState::POST_LANDING, (int)out.state);
    TEST_ASSERT_EQUAL_UINT32(1101, out.landing_confirm_ms);
    out = controller.step(sample(1202, 0.20f, 0.0f));
    TEST_ASSERT_EQUAL((int)DropTestState::TEST_COMPLETE, (int)out.state);
    TEST_ASSERT_FALSE(out.recording);
    TEST_ASSERT_TRUE(out.close_log);
    TEST_ASSERT_TRUE(out.neutral_lock);
}

void gps_motion_blocks_landing_but_gps_absence_does_not_invent_motion() {
    DropTestController controller(config());
    confirmDrop(controller);
    auto in = sample(600, 0.1f, 0.0f);
    in.ground_speed_mps = 5.0f;
    auto out = controller.step(in);
    TEST_ASSERT_NOT_EQUAL((int)DropTestState::LANDING_CONFIRM, (int)out.state);
    in.now_ms = 700;
    in.gps_valid = false;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)DropTestState::LANDING_CONFIRM, (int)out.state);
}

void sensor_dropout_never_releases_neutral_or_false_completes() {
    DropTestController controller(config());
    confirmDrop(controller);
    auto in = sample(600, 0.0f, 0.0f);
    in.barometer_valid = false;
    in.imu_valid = false;
    auto out = controller.step(in);
    TEST_ASSERT_TRUE(out.recording);
    TEST_ASSERT_TRUE(out.neutral_lock);
    TEST_ASSERT_EQUAL_UINT32(0, out.landing_confirm_ms);
}

void timeout_and_abort_close_the_log_fail_neutral() {
    DropTestController timeout_controller(config());
    timeout_controller.arm(100, 10.0f);
    auto out = timeout_controller.step(sample(5100, 10.0f, 0.0f));
    TEST_ASSERT_TRUE(out.close_log);
    TEST_ASSERT_TRUE(out.neutral_lock);
    TEST_ASSERT_TRUE((out.events & DROP_EVENT_TIMEOUT) != 0);

    DropTestController abort_controller(config());
    abort_controller.arm(100, 10.0f);
    out = abort_controller.abort(200);
    TEST_ASSERT_EQUAL((int)DropTestState::TEST_ABORTED, (int)out.state);
    TEST_ASSERT_TRUE(out.close_log);
    TEST_ASSERT_TRUE(out.neutral_lock);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(arm_always_enables_recording_and_neutral_lock);
    RUN_TEST(release_requires_persistent_speed_and_altitude_loss);
    RUN_TEST(impossible_values_cannot_trigger_release);
    RUN_TEST(canopy_signature_is_observational_and_timestamped);
    RUN_TEST(stable_descent_requires_a_quiet_persistent_window);
    RUN_TEST(landing_requires_low_motion_altitude_span_and_persistence);
    RUN_TEST(gps_motion_blocks_landing_but_gps_absence_does_not_invent_motion);
    RUN_TEST(sensor_dropout_never_releases_neutral_or_false_completes);
    RUN_TEST(timeout_and_abort_close_the_log_fail_neutral);
    return UNITY_END();
}
