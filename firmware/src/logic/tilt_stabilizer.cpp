#include "tilt_stabilizer.h"

#include <cmath>

namespace logic {

TiltStabilizer::TiltStabilizer(const TiltStabilizerConfig& config) : config_(config) {}

void TiltStabilizer::arm(uint32_t now_ms, float reference_roll_deg) {
    armed_ = std::isfinite(reference_roll_deg);
    armed_ms_ = now_ms;
    reference_roll_deg_ = armed_ ? wrapDegrees(reference_roll_deg) : 0.0f;
    last_status_ = armed_ ? TiltStabilizerStatus::ACTIVE
                          : TiltStabilizerStatus::SENSOR_INVALID;
    resetCommandState();
}

void TiltStabilizer::disarm(TiltStabilizerStatus reason) {
    armed_ = false;
    last_status_ = reason;
    resetCommandState();
}

bool TiltStabilizer::finiteInput(const TiltStabilizerInput& input) {
    return std::isfinite(input.dt_s) &&
           std::isfinite(input.roll_deg) &&
           std::isfinite(input.pitch_deg) &&
           std::isfinite(input.roll_rate_dps) &&
           std::isfinite(input.vertical_speed_mps) &&
           std::isfinite(input.vertical_accel_mps2);
}

float TiltStabilizer::wrapDegrees(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float TiltStabilizer::clamp(float value, float lower, float upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

void TiltStabilizer::resetCommandState() {
    limited_command_ = 0.0f;
    last_direction_ = 0;
    last_direction_command_ms_ = 0;
}

TiltStabilizerOutput TiltStabilizer::neutralOutput(TiltStabilizerStatus status,
                                                   float roll_error_deg) {
    last_status_ = status;
    limited_command_ = 0.0f;
    TiltStabilizerOutput out;
    out.status = status;
    out.armed = armed_;
    out.active = false;
    out.immediate_neutral = true;
    out.reference_roll_deg = reference_roll_deg_;
    out.roll_error_deg = roll_error_deg;
    return out;
}

TiltStabilizerOutput TiltStabilizer::update(const TiltStabilizerInput& input) {
    const float error = wrapDegrees(input.roll_deg - reference_roll_deg_);
    if (!armed_) return neutralOutput(last_status_, error);
    if (!input.command_fresh) {
        disarm(TiltStabilizerStatus::COMMAND_TIMEOUT);
        return neutralOutput(TiltStabilizerStatus::COMMAND_TIMEOUT, error);
    }
    if (input.now_ms - armed_ms_ >= config_.max_duration_ms) {
        disarm(TiltStabilizerStatus::DURATION_EXPIRED);
        return neutralOutput(TiltStabilizerStatus::DURATION_EXPIRED, error);
    }
    if (!input.preflight_allowed || !input.servo_valid) {
        return neutralOutput(TiltStabilizerStatus::INTERLOCK, error);
    }
    if (!input.imu_valid || !input.barometer_valid || !finiteInput(input)) {
        return neutralOutput(TiltStabilizerStatus::SENSOR_INVALID, error);
    }
    if (std::fabs(input.vertical_speed_mps) > config_.max_vertical_speed_mps ||
        std::fabs(input.vertical_accel_mps2) > config_.max_vertical_accel_mps2) {
        return neutralOutput(TiltStabilizerStatus::MOTION_BLOCKED, error);
    }
    if (std::fabs(error) > config_.max_roll_error_deg ||
        std::fabs(input.pitch_deg) > config_.max_abs_pitch_deg ||
        std::fabs(input.roll_rate_dps) > config_.max_roll_rate_dps) {
        return neutralOutput(TiltStabilizerStatus::ATTITUDE_LIMIT, error);
    }

    float desired = 0.0f;
    if (std::fabs(error) >= config_.roll_deadband_deg ||
        std::fabs(input.roll_rate_dps) >= config_.roll_rate_deadband_dps) {
        desired = config_.correction_sign *
                  (config_.kp_per_deg * error + config_.kd_per_dps * input.roll_rate_dps);
        desired = clamp(desired, -config_.max_brake_command, config_.max_brake_command);
    }

    const int desired_direction = desired > 0.0f ? 1 : (desired < 0.0f ? -1 : 0);
    if (desired_direction != 0 && last_direction_ != 0 &&
        desired_direction != last_direction_ &&
        input.now_ms - last_direction_command_ms_ < config_.reversal_guard_ms) {
        desired = 0.0f;
    }

    const float safe_dt = clamp(input.dt_s, 0.0f, 0.1f);
    const float max_delta = config_.command_rate_limit_per_s * safe_dt;
    limited_command_ += clamp(desired - limited_command_, -max_delta, max_delta);
    if (std::fabs(limited_command_) < 0.0005f) limited_command_ = 0.0f;

    const int actual_direction = limited_command_ > 0.0f ? 1 : (limited_command_ < 0.0f ? -1 : 0);
    if (actual_direction != 0) {
        last_direction_ = actual_direction;
        last_direction_command_ms_ = input.now_ms;
    }

    TiltStabilizerOutput out;
    out.status = TiltStabilizerStatus::ACTIVE;
    out.armed = true;
    out.active = true;
    out.immediate_neutral = false;
    out.reference_roll_deg = reference_roll_deg_;
    out.roll_error_deg = error;
    out.signed_command = limited_command_;
    out.left_brake = limited_command_ > 0.0f ? limited_command_ : 0.0f;
    out.right_brake = limited_command_ < 0.0f ? -limited_command_ : 0.0f;
    last_status_ = out.status;
    return out;
}

} // namespace logic
