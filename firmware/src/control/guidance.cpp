// ============================================================================
// PHOENIX RECOVERY — Guidance Controller Implementation.
// ============================================================================
#include "guidance.h"

namespace control {

GuidanceController::GuidanceController()
    : rate_limiter_(cfg::CMD_RATE_LIMIT_PER_S),
      reversal_guard_(cfg::REVERSAL_GUARD_MS) {
    // Initialize config from cfg namespace
    config_.kp = cfg::GUIDANCE_KP;
    config_.deadband_deg = cfg::GUIDANCE_DEADBAND_DEG;
    config_.max_steering = cfg::MAX_STEERING_COMMAND;
    config_.cmd_rate_limit_per_s = cfg::CMD_RATE_LIMIT_PER_S;
    config_.reversal_guard_ms = cfg::REVERSAL_GUARD_MS;
    config_.final_approach_kp = cfg::FINAL_APPROACH_KP;
    config_.final_approach_max_command = cfg::FINAL_APPROACH_MAX_COMMAND;
    config_.final_approach_altitude_m = cfg::FINAL_APPROACH_ALTITUDE_M;
    config_.flare_enabled = cfg::FLARE_ENABLED;
    config_.flare_altitude_m = cfg::FLARE_ALTITUDE_M;
    config_.flare_max_brake = cfg::FLARE_MAX_BRAKE;
    config_.flare_stall_release_dps = cfg::FLARE_STALL_RELEASE_DPS;
    config_.min_guidance_altitude_m = cfg::MIN_GUIDANCE_ALTITUDE_M;
    config_.target_accept_radius_m = cfg::TARGET_ACCEPT_RADIUS_M;
    config_.min_course_speed_mps = cfg::GPS_MIN_GROUND_SPEED_FOR_COURSE;
    config_.reversal_guard_ms = cfg::MIN_TURN_REVERSAL_TIME_MS;
}

bool GuidanceController::begin(control::ServoController* servos) {
    servos_ = servos;
    rate_limiter_.reset(0.0f);
    reversal_guard_.reset();
    initialized_ = true;
    return true;
}

bool GuidanceController::begin(const logic::GuidanceConfig& config) {
    config_ = config;
    rate_limiter_.reset(0.0f);
    reversal_guard_.reset();
    initialized_ = true;
    return true;
}

void GuidanceController::update(phoenix::VehicleState& state, float dt_s) {
    if (!initialized_ || !servos_) return;

    // Prepare guidance input
    logic::GuidanceInput in;
    in.current_lat = state.latitude;
    in.current_lon = state.longitude;
    in.target_lat = state.target_latitude;
    in.target_lon = state.target_longitude;
    in.altitude_agl_m = state.altitude_agl_m;
    in.ground_speed_mps = state.ground_speed_mps;
    in.gps_course_deg = state.gps_course_deg;
    in.gps_valid = state.gps_valid;
    in.imu_valid = state.imu_valid;
    in.baro_valid = state.barometer_valid;
    in.angular_rate_dps = state.angular_rate_dps;
    in.mode = current_mode_;
    in.now_ms = state.timestamp_ms;
    in.dt_s = dt_s;

    // Compute guidance
    logic::GuidanceOutput out = logic::computeGuidance(in, config_, rate_limiter_, reversal_guard_);

    // Update state with guidance output
    state.target_bearing_deg = out.target_bearing_deg;
    state.heading_error_deg = out.heading_error_deg;
    state.distance_to_target_m = out.distance_to_target_m;
    state.guidance_mode = out.active_mode;
    guidance_active_ = out.guidance_active;

    // Apply steering to servos
    if (out.guidance_active) {
        // Parafoil steering pulls only the brake on the requested side.
        // Negative heading error = target left; positive = target right.
        float left_cmd = out.steering_command < 0.0f ? -out.steering_command : 0.0f;
        float right_cmd = out.steering_command > 0.0f ? out.steering_command : 0.0f;
        servos_->setBrakeCommands(left_cmd, right_cmd);

        state.left_servo_command = left_cmd;
        state.right_servo_command = right_cmd;
    } else {
        // No guidance - neutral
        servos_->setNeutral();
        state.left_servo_command = 0.0f;
        state.right_servo_command = 0.0f;
    }

    // Update servo actual positions
    state.left_servo_us = servos_->getLeftUs();
    state.right_servo_us = servos_->getRightUs();
}

void GuidanceController::update(phoenix::VehicleState& state) {
    // Default dt_s = 1/15 (guidance rate)
    update(state, 1.0f / 15.0f);
}

} // namespace control
