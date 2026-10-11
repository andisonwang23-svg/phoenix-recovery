#include <unity.h>

#include "logic/guided_drop_controller.h"
#include "../../firmware/src/logic/guidance_logic.cpp"
#include "../../firmware/src/logic/guided_drop_controller.cpp"

using namespace logic;

static GuidedDropConfig config(bool enabled = true) {
    GuidedDropConfig c;
    c.enabled = enabled;
    c.deployment_neutral_ms = 200;
    c.navigation_confirm_ms = 100;
    c.navigation_loss_latch_ms = 300;
    c.min_activation_altitude_m = 20.0f;
    c.max_activation_angular_rate_dps = 12.0f;
    c.final_approach_radius_m = 10.0f;
    c.sensor_max_age_ms = 1000;
    c.gps_max_position_jump_m = 50.0f;
    c.gps_recovery_fix_count = 3;
    c.guidance.kp = 0.02f;
    c.guidance.deadband_deg = 2.0f;
    c.guidance.max_steering = 0.15f;
    c.guidance.cmd_rate_limit_per_s = 0.5f;
    c.guidance.reversal_guard_ms = 200;
    c.guidance.min_guidance_altitude_m = 5.0f;
    c.guidance.min_course_speed_mps = 1.0f;
    c.guidance.target_accept_radius_m = 2.0f;
    c.guidance.final_approach_altitude_m = 10.0f;
    c.guidance.final_approach_max_command = 0.08f;
    c.guidance.flare_enabled = true; // constructor must force this off
    return c;
}

static GuidedDropInput nominal(uint32_t now_ms) {
    GuidedDropInput in;
    in.now_ms = now_ms;
    in.dt_s = 0.1f;
    in.guided_test_requested = true;
    in.drop_recording = true;
    in.drop_state = DropTestState::STABLE_DESCENT_OBSERVED;
    in.release_confirm_ms = 100;
    in.stable_descent_observed = true;
    in.canopy_confirmed = true;
    in.authority = GuidedAuthorityLevel::TWENTY_PERCENT;
    in.authority_limit = 0.20f;
    in.latitude = 41.0;
    in.longitude = -87.0;
    in.target_latitude = 41.001;
    in.target_longitude = -87.0;
    in.altitude_agl_m = 100.0f;
    in.ground_speed_mps = 8.0f;
    in.gps_course_deg = 90.0f;
    in.imu_heading_deg = 90.0f;
    in.angular_rate_dps = 2.0f;
    in.gps_valid = true;
    in.imu_valid = true;
    in.barometer_valid = true;
    in.servo_valid = true;
    return in;
}

static GuidedDropOutput reachGuidance(GuidedDropController& controller) {
    controller.step(nominal(301));
    return controller.step(nominal(402));
}

void disabled_profile_is_always_neutral() {
    GuidedDropController controller(config(false));
    auto out = controller.step(nominal(500));
    TEST_ASSERT_FALSE(out.available);
    TEST_ASSERT_TRUE(out.neutral_required);
    TEST_ASSERT_FALSE(out.active);
}

void release_and_deployment_delay_are_neutral() {
    GuidedDropController controller(config());
    auto in = nominal(50);
    in.drop_state = DropTestState::ARMED_RECORDING;
    in.release_confirm_ms = 0;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_RELEASE, (int)out.status);
    in.now_ms = 250;
    in.drop_state = DropTestState::DROP_CONFIRMED;
    in.release_confirm_ms = 100;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::DEPLOYMENT_NEUTRAL, (int)out.status);
    TEST_ASSERT_TRUE(out.neutral_required);
}

void stability_navigation_and_altitude_gate_guidance() {
    GuidedDropController controller(config());
    auto in = nominal(301);
    in.stable_descent_observed = false;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_STABLE_DESCENT, (int)out.status);
    in.stable_descent_observed = true;
    in.target_latitude = in.target_longitude = 0.0;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_NAVIGATION, (int)out.status);
    in.target_latitude = 41.001;
    in.target_longitude = -87.0;
    in.altitude_agl_m = 15.0f;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_ALTITUDE, (int)out.status);
}

void guidance_produces_a_limited_nonzero_brake_command() {
    GuidedDropController controller(config());
    auto out = reachGuidance(controller);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::GUIDANCE_ACTIVE, (int)out.status);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_TRUE(out.left_brake > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, out.right_brake);
    TEST_ASSERT_TRUE(out.left_brake <= 0.15f);
}

void short_gps_loss_unwinds_then_latches_failsafe() {
    GuidedDropController controller(config());
    auto out = reachGuidance(controller);
    const float before = out.left_brake + out.right_brake;
    auto in = nominal(450);
    in.gps_valid = false;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_NAVIGATION, (int)out.status);
    TEST_ASSERT_TRUE(out.left_brake + out.right_brake < before);
    TEST_ASSERT_FALSE(out.fault_latched);
    in.now_ms = 751;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::FAULT_NEUTRAL, (int)out.status);
    TEST_ASSERT_TRUE(out.fault_latched);
    TEST_ASSERT_TRUE(out.neutral_required);
}

void critical_sensor_or_servo_fault_latches_neutral() {
    GuidedDropController controller(config());
    auto in = nominal(301);
    in.imu_valid = false;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::FAULT_NEUTRAL, (int)out.status);
    TEST_ASSERT_TRUE(out.fault_latched);
    in.imu_valid = true;
    out = controller.step(in);
    TEST_ASSERT_TRUE(out.fault_latched);
    TEST_ASSERT_TRUE(out.neutral_required);
}

void persistent_instability_after_guidance_latches_neutral() {
    auto c = config();
    c.neutral_angular_rate_dps = 20.0f;
    c.instability_latch_ms = 200;
    GuidedDropController controller(c);
    auto out = reachGuidance(controller);
    TEST_ASSERT_TRUE(out.active);

    auto in = nominal(500);
    in.angular_rate_dps = 25.0f;
    out = controller.step(in);
    TEST_ASSERT_FALSE(out.fault_latched);
    TEST_ASSERT_TRUE(out.left_brake + out.right_brake < 0.15f);

    in.now_ms = 701;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::FAULT_NEUTRAL, (int)out.status);
    TEST_ASSERT_TRUE(out.fault_latched);
    TEST_ASSERT_TRUE(out.neutral_required);
}

void canopy_confirmation_and_authority_are_hard_gates() {
    GuidedDropController controller(config());
    auto in = nominal(301);
    in.canopy_confirmed = false;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_STABLE_DESCENT, (int)out.status);
    TEST_ASSERT_TRUE(out.neutral_required);

    controller.reset();
    in = nominal(301);
    in.authority = GuidedAuthorityLevel::NEUTRAL;
    in.authority_limit = 0.0f;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::FAULT_NEUTRAL, (int)out.status);
}

void gps_jump_requires_plausible_recovery_fixes() {
    GuidedDropController controller(config());
    reachGuidance(controller);
    auto in = nominal(500);
    in.latitude += 0.01;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::GPS_SUSPECT, (int)out.status);
    TEST_ASSERT_TRUE(out.gps_suspect);
    in.latitude = 41.0;
    in.now_ms = 550;
    out = controller.step(in);
    TEST_ASSERT_TRUE(out.gps_suspect);
    in.now_ms = 600;
    out = controller.step(in);
    TEST_ASSERT_TRUE(out.gps_suspect);
    in.now_ms = 650;
    out = controller.step(in);
    TEST_ASSERT_FALSE(out.gps_suspect);
}

void validated_imu_course_hold_is_bounded_and_optional() {
    auto c = config();
    c.imu_course_hold_enabled = true;
    c.imu_course_hold_max_ms = 200;
    c.imu_course_hold_max_command = 0.05f;
    GuidedDropController controller(c);
    reachGuidance(controller);
    auto in = nominal(450);
    in.gps_valid = false;
    in.imu_heading_deg = 90.0f;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::GPS_COURSE_HOLD, (int)out.status);
    TEST_ASSERT_TRUE(out.course_hold_active);
    TEST_ASSERT_LESS_OR_EQUAL(0.051f, out.left_brake + out.right_brake);
    in.now_ms = 701;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::WAITING_NAVIGATION, (int)out.status);
}

void landing_or_abort_always_neutralizes() {
    GuidedDropController controller(config());
    reachGuidance(controller);
    auto in = nominal(500);
    in.drop_state = DropTestState::POST_LANDING;
    auto out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::LANDED, (int)out.status);
    TEST_ASSERT_TRUE(out.neutral_required);
    in.drop_state = DropTestState::TEST_ABORTED;
    out = controller.step(in);
    TEST_ASSERT_EQUAL((int)GuidedDropStatus::LANDED, (int)out.status);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(disabled_profile_is_always_neutral);
    RUN_TEST(release_and_deployment_delay_are_neutral);
    RUN_TEST(stability_navigation_and_altitude_gate_guidance);
    RUN_TEST(guidance_produces_a_limited_nonzero_brake_command);
    RUN_TEST(short_gps_loss_unwinds_then_latches_failsafe);
    RUN_TEST(critical_sensor_or_servo_fault_latches_neutral);
    RUN_TEST(persistent_instability_after_guidance_latches_neutral);
    RUN_TEST(canopy_confirmation_and_authority_are_hard_gates);
    RUN_TEST(gps_jump_requires_plausible_recovery_fixes);
    RUN_TEST(validated_imu_course_hold_is_bounded_and_optional);
    RUN_TEST(landing_or_abort_always_neutralizes);
    return UNITY_END();
}
