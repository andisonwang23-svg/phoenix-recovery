#include <unity.h>
#include <cmath>

#include "logic/flight_coordinator.h"
#include "logic/servo_calibration_logic.h"
#include "../../firmware/src/logic/guidance_logic.cpp"
#include "../../firmware/src/logic/state_machine.cpp"
#include "../../firmware/src/logic/flight_coordinator.cpp"

using namespace logic;

static CoordinatorConfig testConfig() {
    CoordinatorConfig c;
    c.state_machine.launch_confirm_ms = 100;
    c.state_machine.apogee_confirmation_time_ms = 100;
    c.state_machine.deployment_wait_ms = 100;
    c.state_machine.min_stabilization_time_ms = 100;
    c.state_machine.max_stabilization_time_ms = 1000;
    c.state_machine.gps_loss_timeout_ms = 500;
    c.state_machine.landed_confirm_ms = 200;
    c.state_machine.final_approach_altitude_m = 20;
    c.state_machine.final_approach_radius_m = 5;
    c.guidance.kp = .02f;
    c.guidance.deadband_deg = 2;
    c.guidance.max_steering = .35f;
    c.guidance.cmd_rate_limit_per_s = .5f;
    c.guidance.min_guidance_altitude_m = 5;
    c.guidance.target_accept_radius_m = 2;
    c.guidance.min_course_speed_mps = 1;
    c.guidance.flare_enabled = false;
    c.flare_enabled = false;
    return c;
}

static CoordinatorInput nominal(uint32_t now) {
    CoordinatorInput i;
    i.now_ms = now; i.dt_s = .1f;
    i.latitude = 41.0; i.longitude = -87.0;
    i.target_latitude = 41.001; i.target_longitude = -87.0;
    i.altitude_agl_m = 100; i.vertical_speed_mps = 0;
    i.ground_speed_mps = 8; i.gps_course_deg = 90;
    i.angular_rate_dps = 2;
    i.gps_valid = i.imu_valid = i.barometer_valid = i.servo_valid = true;
    return i;
}

static CoordinatorOutput reachGuidance(FlightCoordinator& coordinator, bool target = true) {
    CoordinatorInput i = nominal(1);
    if (!target) i.target_latitude = i.target_longitude = 0;
    coordinator.step(i);                 // BOOT -> SELF_TEST
    i.now_ms = 2; coordinator.step(i);   // SELF_TEST -> PAD_SAFE
    i.vertical_accel_mps2 = 12; i.vertical_speed_mps = 8; i.altitude_agl_m = 20;
    i.now_ms = 10; coordinator.step(i);
    i.now_ms = 111; coordinator.step(i); // launch
    i.vertical_accel_mps2 = 0; i.vertical_speed_mps = -3; i.altitude_agl_m = 100;
    i.now_ms = 200; coordinator.step(i);
    i.now_ms = 301; coordinator.step(i); // apogee
    i.now_ms = 302; coordinator.step(i); // deployment wait
    i.now_ms = 403; coordinator.step(i); // stabilization
    i.now_ms = 404; coordinator.step(i);
    i.now_ms = 505;
    return coordinator.step(i);          // guidance
}

void full_sequence_reaches_guidance_and_commands_brake() {
    FlightCoordinator c(testConfig());
    auto o = reachGuidance(c);
    TEST_ASSERT_EQUAL((int)FlightState::GUIDED_DESCENT, (int)o.state);
    TEST_ASSERT_EQUAL((int)GuidanceMode::HEADING_TO_TARGET, (int)o.mode);
    TEST_ASSERT_TRUE(o.requested_left_brake > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, o.requested_right_brake);
    TEST_ASSERT_TRUE(c.timestamps().launch_ms > 0);
    TEST_ASSERT_TRUE(c.timestamps().apogee_ms > 0);
    TEST_ASSERT_TRUE(c.timestamps().deployment_wait_ms > 0);
    TEST_ASSERT_TRUE(c.timestamps().stabilization_ms > 0);
    TEST_ASSERT_TRUE(c.timestamps().guidance_ms > 0);
}

void missing_target_blocks_guidance_output() {
    FlightCoordinator c(testConfig());
    auto o = reachGuidance(c, false);
    TEST_ASSERT_EQUAL((int)FlightState::GUIDED_DESCENT, (int)o.state);
    TEST_ASSERT_FALSE(o.target_valid);
    TEST_ASSERT_FALSE(o.guidance_ready);
    TEST_ASSERT_EQUAL((int)GuidanceMode::MODE_DISABLED, (int)o.mode);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void bad_preflight_baro_launch_attempt_warns_and_fails_neutral() {
    FlightCoordinator c(testConfig());
    CoordinatorInput i = nominal(1);
    c.step(i);                       // BOOT -> SELF_TEST
    i.barometer_valid = false;
    i.vertical_accel_mps2 = 12;
    i.vertical_speed_mps = 8;
    i.altitude_agl_m = 20;
    i.now_ms = 10; c.step(i);        // launch-like motion begins
    i.now_ms = 111; auto o = c.step(i);
    TEST_ASSERT_TRUE(o.preflight_launch_warning);
    TEST_ASSERT_FALSE(o.launch_readiness_ok);
    TEST_ASSERT_TRUE(o.failsafe_active);
    TEST_ASSERT_EQUAL((int)FlightState::FAILSAFE_DESCENT, (int)o.state);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_BAROMETER, (int)o.failure);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void bad_preflight_gps_launch_attempt_warns_and_fails_neutral() {
    FlightCoordinator c(testConfig());
    CoordinatorInput i = nominal(1);
    i.gps_valid = false;
    c.step(i);                       // BOOT -> SELF_TEST
    i.now_ms = 2; c.step(i);         // SELF_TEST -> PAD_SAFE; GPS is a guidance readiness fault
    i.vertical_accel_mps2 = 12;
    i.vertical_speed_mps = 8;
    i.altitude_agl_m = 20;
    i.now_ms = 10; c.step(i);        // launch-like motion begins
    i.now_ms = 111; auto o = c.step(i);
    TEST_ASSERT_TRUE(o.preflight_launch_warning);
    TEST_ASSERT_FALSE(o.launch_readiness_ok);
    TEST_ASSERT_TRUE(o.failsafe_active);
    TEST_ASSERT_EQUAL((int)FlightState::FAILSAFE_DESCENT, (int)o.state);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_NAV_INVALID, (int)o.failure);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void bad_preflight_servo_launch_attempt_warns_and_fails_neutral() {
    FlightCoordinator c(testConfig());
    CoordinatorInput i = nominal(1);
    i.servo_valid = false;
    c.step(i);                       // BOOT -> SELF_TEST
    i.now_ms = 2; c.step(i);         // SELF_TEST -> PAD_SAFE; servo is checked by coordinator
    i.vertical_accel_mps2 = 12;
    i.vertical_speed_mps = 8;
    i.altitude_agl_m = 20;
    i.now_ms = 10; c.step(i);        // launch-like motion begins
    i.now_ms = 111; auto o = c.step(i);
    TEST_ASSERT_TRUE(o.preflight_launch_warning);
    TEST_ASSERT_FALSE(o.launch_readiness_ok);
    TEST_ASSERT_TRUE(o.failsafe_active);
    TEST_ASSERT_EQUAL((int)FlightState::FAILSAFE_DESCENT, (int)o.state);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_SERVO, (int)o.failure);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void gps_loss_unwinds_then_fails_neutral() {
    FlightCoordinator c(testConfig());
    auto o = reachGuidance(c);
    const float initial = o.requested_left_brake;
    CoordinatorInput i = nominal(605); i.vertical_speed_mps = -3; i.gps_valid = false;
    o = c.step(i);
    TEST_ASSERT_TRUE(o.gps_fallback_active);
    TEST_ASSERT_TRUE(o.degraded_guidance);
    TEST_ASSERT_TRUE(o.requested_left_brake < initial);
    TEST_ASSERT_FALSE(o.failsafe_active);
    i.now_ms = 1110; o = c.step(i);
    TEST_ASSERT_TRUE(o.failsafe_active);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_GPS_TIMEOUT, (int)o.failure);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void imu_loss_after_guidance_uses_reduced_authority_fallback() {
    FlightCoordinator c(testConfig()); reachGuidance(c);
    CoordinatorInput i = nominal(600); i.vertical_speed_mps = -3; i.imu_valid = false;
    auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_NONE, (int)o.failure);
    TEST_ASSERT_EQUAL((int)FlightState::GUIDED_DESCENT, (int)o.state);
    TEST_ASSERT_TRUE(o.imu_fallback_active);
    TEST_ASSERT_TRUE(o.degraded_guidance);
    TEST_ASSERT_TRUE(o.requested_left_brake > 0.0f);
    TEST_ASSERT_TRUE(o.requested_left_brake <= testConfig().guidance.reduced_authority);
}

void barometer_loss_after_guidance_uses_gps_imu_fallback() {
    FlightCoordinator c(testConfig()); reachGuidance(c);
    CoordinatorInput i = nominal(600); i.vertical_speed_mps = -3; i.barometer_valid = false;
    auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_NONE, (int)o.failure);
    TEST_ASSERT_EQUAL((int)FlightState::GUIDED_DESCENT, (int)o.state);
    TEST_ASSERT_TRUE(o.barometer_fallback_active);
    TEST_ASSERT_TRUE(o.degraded_guidance);
    TEST_ASSERT_TRUE(o.requested_left_brake > 0.0f);
    TEST_ASSERT_TRUE(o.requested_left_brake <= testConfig().guidance.reduced_authority);
}

void combined_sensor_loss_latches_and_cannot_be_overwritten() {
    FlightCoordinator c(testConfig()); reachGuidance(c);
    CoordinatorInput i = nominal(600); i.vertical_speed_mps = -3;
    i.imu_valid = false; i.barometer_valid = false;
    auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_SENSOR_DATA, (int)o.failure);
    TEST_ASSERT_EQUAL((int)FlightState::FAILSAFE_DESCENT, (int)o.state);
    i.now_ms = 700; i.imu_valid = true; i.barometer_valid = true;
    o = c.step(i);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_SENSOR_DATA, (int)o.failure);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void pre_guidance_imu_or_barometer_loss_still_fails_safe() {
    FlightCoordinator c(testConfig());
    CoordinatorInput i = nominal(1);
    c.step(i);                 // BOOT -> SELF_TEST
    i.now_ms = 2; c.step(i);   // SELF_TEST -> PAD_SAFE
    i.vertical_accel_mps2 = 12; i.vertical_speed_mps = 8; i.altitude_agl_m = 20;
    i.now_ms = 10; c.step(i);
    i.now_ms = 111; c.step(i); // ASCENT
    i.imu_valid = false;
    i.now_ms = 112; auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_IMU, (int)o.failure);
    TEST_ASSERT_EQUAL((int)FlightState::FAILSAFE_DESCENT, (int)o.state);

    FlightCoordinator b(testConfig());
    i = nominal(1);
    b.step(i);
    i.now_ms = 2; b.step(i);
    i.vertical_accel_mps2 = 12; i.vertical_speed_mps = 8; i.altitude_agl_m = 20;
    i.now_ms = 10; b.step(i);
    i.now_ms = 111; b.step(i);
    i.barometer_valid = false;
    i.now_ms = 112; o = b.step(i);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_BAROMETER, (int)o.failure);
    TEST_ASSERT_EQUAL((int)FlightState::FAILSAFE_DESCENT, (int)o.state);
}

void reboot_restores_calibration_and_flare_stays_disabled() {
    ServoCalibrationValue defaults;
    ServoCalibrationValue saved = defaults;
    saved.neutral_us = 1525; saved.min_us = 1100; saved.max_us = 1900;
    saved.max_brake_us = 300; saved.reversed = true;
    auto restored = restoreServoCalibration(defaults, saved);
    TEST_ASSERT_FLOAT_WITHIN(.01f, 1525, restored.neutral_us);
    TEST_ASSERT_TRUE(restored.reversed);
    FlightCoordinator rebooted(testConfig());
    TEST_ASSERT_FALSE(rebooted.flareEnabled());
}

void landing_closes_log_once_and_neutralizes() {
    FlightCoordinator c(testConfig()); reachGuidance(c);
    CoordinatorInput i = nominal(600); i.altitude_agl_m = 10;
    i.vertical_speed_mps = 0; i.ground_speed_mps = 0; i.angular_rate_dps = 0;
    c.step(i);                         // enter FINAL_APPROACH
    i.now_ms = 601; c.step(i);         // begin sustained landing condition
    i.now_ms = 802; auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FlightState::LANDED, (int)o.state);
    TEST_ASSERT_TRUE(o.close_log);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
    i.now_ms = 900; o = c.step(i);
    TEST_ASSERT_FALSE(o.close_log);
}

void communications_loss_has_no_control_input() {
    FlightCoordinator c(testConfig());
    auto before = reachGuidance(c);
    CoordinatorInput i = nominal(605); i.vertical_speed_mps = -3;
    // CoordinatorInput intentionally contains no Wi-Fi or LoRa fields.
    auto after = c.step(i);
    TEST_ASSERT_EQUAL((int)FlightState::GUIDED_DESCENT, (int)after.state);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_NONE, (int)after.failure);
    TEST_ASSERT_TRUE(before.requested_left_brake > 0 && after.requested_left_brake > 0);
}

void lora_remote_manual_commands_only_in_descent() {
    FlightCoordinator c(testConfig());
    reachGuidance(c);
    CoordinatorInput i = nominal(605);
    i.vertical_speed_mps = -3;
    i.remote_manual_active = true;
    i.remote_servo1_brake = 0.20f;
    i.remote_servo2_brake = 0.05f;
    auto o = c.step(i);
    TEST_ASSERT_TRUE(o.remote_command_allowed);
    TEST_ASSERT_TRUE(o.remote_manual_active);
    TEST_ASSERT_EQUAL((int)GuidanceMode::REMOTE_MANUAL, (int)o.mode);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.20f, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.05f, o.requested_right_brake);

    FlightCoordinator ascent(testConfig());
    i = nominal(1);
    ascent.step(i);
    i.now_ms = 2; ascent.step(i);
    i.vertical_accel_mps2 = 12; i.vertical_speed_mps = 8; i.altitude_agl_m = 20;
    i.now_ms = 10; ascent.step(i);
    i.now_ms = 111;
    i.remote_manual_active = true;
    i.remote_servo1_brake = 0.25f;
    i.remote_servo2_brake = 0.25f;
    o = ascent.step(i);
    TEST_ASSERT_EQUAL((int)FlightState::ASCENT, (int)o.state);
    TEST_ASSERT_FALSE(o.remote_command_allowed);
    TEST_ASSERT_FALSE(o.remote_manual_active);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, o.requested_right_brake);
}

void lora_remote_can_steer_without_target_after_stabilization() {
    FlightCoordinator c(testConfig());
    reachGuidance(c, false);
    CoordinatorInput i = nominal(605);
    i.target_latitude = 0;
    i.target_longitude = 0;
    i.vertical_speed_mps = -3;
    i.remote_manual_active = true;
    i.remote_servo1_brake = 0.12f;
    i.remote_servo2_brake = 0.00f;
    auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FlightState::GUIDED_DESCENT, (int)o.state);
    TEST_ASSERT_FALSE(o.target_valid);
    TEST_ASSERT_TRUE(o.remote_command_allowed);
    TEST_ASSERT_TRUE(o.remote_manual_active);
    TEST_ASSERT_EQUAL((int)GuidanceMode::REMOTE_MANUAL, (int)o.mode);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.12f, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.00f, o.requested_right_brake);
}

void lora_remote_cannot_override_failsafe() {
    FlightCoordinator c(testConfig());
    reachGuidance(c);
    CoordinatorInput i = nominal(600);
    i.vertical_speed_mps = -3;
    i.imu_valid = false;
    i.barometer_valid = false;
    c.step(i);
    i.now_ms = 700;
    i.imu_valid = true;
    i.barometer_valid = true;
    i.remote_manual_active = true;
    i.remote_servo1_brake = 0.25f;
    i.remote_servo2_brake = 0.25f;
    auto o = c.step(i);
    TEST_ASSERT_TRUE(o.failsafe_active);
    TEST_ASSERT_FALSE(o.remote_command_allowed);
    TEST_ASSERT_FALSE(o.remote_manual_active);
    TEST_ASSERT_EQUAL((int)GuidanceMode::MODE_FAILSAFE, (int)o.mode);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, o.requested_right_brake);
}

void failsafe_can_land_but_failure_remains_recorded() {
    FlightCoordinator c(testConfig()); reachGuidance(c);
    CoordinatorInput i = nominal(600); i.vertical_speed_mps = -3; i.servo_valid = false;
    c.step(i);
    i.now_ms = 700; i.servo_valid = true; i.vertical_speed_mps = 0; i.ground_speed_mps = 0; i.angular_rate_dps = 0;
    c.step(i);
    i.now_ms = 901; auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FlightState::LANDED, (int)o.state);
    TEST_ASSERT_EQUAL((int)FailCode::FAIL_SERVO, (int)o.failure);
    TEST_ASSERT_TRUE(o.close_log);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

void barometer_fallback_can_confirm_landing_in_final_approach() {
    FlightCoordinator c(testConfig()); reachGuidance(c);
    CoordinatorInput i = nominal(600);
    i.latitude = 41.001; i.longitude = -87.0;
    i.altitude_agl_m = 50;
    i.vertical_speed_mps = -3;
    c.step(i);                         // close target -> FINAL_APPROACH
    i.now_ms = 700; i.barometer_valid = false; i.ground_speed_mps = 0; i.angular_rate_dps = 0;
    c.step(i);
    i.now_ms = 901; auto o = c.step(i);
    TEST_ASSERT_EQUAL((int)FlightState::LANDED, (int)o.state);
    TEST_ASSERT_TRUE(o.close_log);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_left_brake);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0, o.requested_right_brake);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(full_sequence_reaches_guidance_and_commands_brake);
    RUN_TEST(missing_target_blocks_guidance_output);
    RUN_TEST(bad_preflight_baro_launch_attempt_warns_and_fails_neutral);
    RUN_TEST(bad_preflight_gps_launch_attempt_warns_and_fails_neutral);
    RUN_TEST(bad_preflight_servo_launch_attempt_warns_and_fails_neutral);
    RUN_TEST(gps_loss_unwinds_then_fails_neutral);
    RUN_TEST(imu_loss_after_guidance_uses_reduced_authority_fallback);
    RUN_TEST(barometer_loss_after_guidance_uses_gps_imu_fallback);
    RUN_TEST(combined_sensor_loss_latches_and_cannot_be_overwritten);
    RUN_TEST(pre_guidance_imu_or_barometer_loss_still_fails_safe);
    RUN_TEST(reboot_restores_calibration_and_flare_stays_disabled);
    RUN_TEST(landing_closes_log_once_and_neutralizes);
    RUN_TEST(communications_loss_has_no_control_input);
    RUN_TEST(lora_remote_manual_commands_only_in_descent);
    RUN_TEST(lora_remote_can_steer_without_target_after_stabilization);
    RUN_TEST(lora_remote_cannot_override_failsafe);
    RUN_TEST(failsafe_can_land_but_failure_remains_recorded);
    RUN_TEST(barometer_fallback_can_confirm_landing_in_final_approach);
    return UNITY_END();
}
