#include "flight_coordinator.h"

#include <cmath>

namespace logic {

FlightCoordinator::FlightCoordinator(const CoordinatorConfig& config)
    : config_(config),
      rate_limiter_(config.guidance.cmd_rate_limit_per_s),
      reversal_guard_(config.guidance.reversal_guard_ms) {
    // A firmware build must opt in twice before flare can ever be selected.
    config_.state_machine.flare_enabled = flareEnabled();
    reset(0);
}

void FlightCoordinator::reset(uint32_t now_ms) {
    state_ = FlightState::BOOT;
    failure_ = FailCode::FAIL_NONE;
    state_runtime_ = StateMachineRuntime{};
    state_runtime_.enter(now_ms);
    timestamps_ = TransitionTimestamps{};
    timestamps_.state_entry_ms = now_ms;
    rate_limiter_.reset(0.0f);
    reversal_guard_.reset();
    log_closed_ = false;
    preflight_launch_attempt_since_ms_ = 0;
    preflight_launch_blocked_ = false;
}

bool FlightCoordinator::validTarget(double latitude, double longitude) {
    if (!std::isfinite(latitude) || !std::isfinite(longitude)) return false;
    if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) return false;
    // 0,0 is the shipped sentinel and must never arm navigation.
    return !(std::fabs(latitude) < 1e-9 && std::fabs(longitude) < 1e-9);
}

bool FlightCoordinator::preflightState(FlightState state) {
    return state == FlightState::BOOT ||
           state == FlightState::SELF_TEST ||
           state == FlightState::PRE_LAUNCH ||
           state == FlightState::PAD_SAFE ||
           state == FlightState::ARMED;
}

bool FlightCoordinator::launchAttemptDetected(const CoordinatorInput& input,
                                              const StateMachineContext& config) {
    // This is intentionally broader than normal launch confirmation. It exists
    // only as a safety tripwire: if the vehicle appears to be launching while
    // required preflight systems are unhealthy, enter fail-neutral instead of
    // remaining stuck in SELF_TEST/PAD_SAFE.
    const bool imu_launch_signal = input.imu_valid &&
        input.vertical_accel_mps2 > config.launch_accel_mps2;
    const bool baro_launch_signal = input.barometer_valid &&
        (input.vertical_speed_mps > config.launch_vs_mps ||
         input.altitude_agl_m > config.launch_alt_delta_m);
    return imu_launch_signal || baro_launch_signal;
}

FailCode FlightCoordinator::readinessFailure(const CoordinatorInput& input, bool target_valid) {
    (void)target_valid;
    if (!input.servo_valid) return FailCode::FAIL_SERVO;
    if (!input.imu_valid && !input.barometer_valid) return FailCode::FAIL_SENSOR_DATA;
    if (!input.imu_valid) return FailCode::FAIL_IMU;
    if (!input.barometer_valid) return FailCode::FAIL_BAROMETER;
    if (!input.gps_valid) return FailCode::FAIL_NAV_INVALID;
    return FailCode::FAIL_NONE;
}

void FlightCoordinator::recordTransition(FlightState next, uint32_t now_ms) {
    timestamps_.state_entry_ms = now_ms;
    switch (next) {
        case FlightState::ASCENT: timestamps_.launch_ms = now_ms; break;
        case FlightState::APOGEE_CONFIRMED: timestamps_.apogee_ms = now_ms; break;
        case FlightState::DEPLOYMENT_WAIT: timestamps_.deployment_wait_ms = now_ms; break;
        case FlightState::PARAFOIL_STABILIZATION: timestamps_.stabilization_ms = now_ms; break;
        case FlightState::GUIDED_DESCENT: timestamps_.guidance_ms = now_ms; break;
        case FlightState::FINAL_APPROACH: timestamps_.final_approach_ms = now_ms; break;
        case FlightState::FLARE: timestamps_.flare_ms = now_ms; break;
        case FlightState::LANDED: timestamps_.landed_ms = now_ms; break;
        case FlightState::FAILSAFE_DESCENT: timestamps_.failsafe_ms = now_ms; break;
        default: break;
    }
}

void FlightCoordinator::latchFailure(FailCode failure, uint32_t now_ms) {
    if (failure_ != FailCode::FAIL_NONE) return;
    failure_ = failure == FailCode::FAIL_NONE ? FailCode::FAIL_UNKNOWN : failure;
    if (state_ != FlightState::LANDED && state_ != FlightState::FAILSAFE_DESCENT) {
        state_ = FlightState::FAILSAFE_DESCENT;
        state_runtime_.enter(now_ms);
        recordTransition(state_, now_ms);
    }
}

CoordinatorOutput FlightCoordinator::step(const CoordinatorInput& in) {
    CoordinatorOutput out;
    const FlightState starting_state = state_;

    // Critical sensors are required once flight has begun. Communications are
    // deliberately absent: Wi-Fi and LoRa can never change flight control.
    const bool airborne = state_ == FlightState::ASCENT || state_ == FlightState::APOGEE_DETECT ||
        state_ == FlightState::APOGEE_CONFIRMED || state_ == FlightState::DEPLOYMENT_WAIT ||
        state_ == FlightState::PARAFOIL_STABILIZATION || state_ == FlightState::GUIDED_DESCENT ||
        state_ == FlightState::FINAL_APPROACH || state_ == FlightState::FLARE;
    const bool guidance_phase = state_ == FlightState::GUIDED_DESCENT ||
        state_ == FlightState::FINAL_APPROACH;
    out.gps_fallback_active = guidance_phase && !in.gps_valid;
    out.imu_fallback_active = guidance_phase && !in.imu_valid &&
        in.gps_valid && in.barometer_valid && in.servo_valid;
    out.barometer_fallback_active = guidance_phase && !in.barometer_valid &&
        in.gps_valid && in.imu_valid && in.servo_valid;
    out.degraded_guidance = out.gps_fallback_active || out.imu_fallback_active ||
        out.barometer_fallback_active;

    if (airborne && !in.servo_valid) latchFailure(FailCode::FAIL_SERVO, in.now_ms);
    if (airborne && !in.imu_valid && !in.barometer_valid) {
        latchFailure(FailCode::FAIL_SENSOR_DATA, in.now_ms);
    } else {
        if (airborne && !in.imu_valid && !out.imu_fallback_active) {
            latchFailure(FailCode::FAIL_IMU, in.now_ms);
        }
        if (airborne && !in.barometer_valid && !out.barometer_fallback_active) {
            latchFailure(FailCode::FAIL_BAROMETER, in.now_ms);
        }
    }

    StateMachineContext ctx = config_.state_machine;
    ctx.now_ms = in.now_ms;
    ctx.altitude_agl_m = in.altitude_agl_m;
    ctx.vertical_speed_mps = in.vertical_speed_mps;
    ctx.vertical_accel_mps2 = in.vertical_accel_mps2;
    ctx.ground_speed_mps = in.ground_speed_mps;
    ctx.angular_rate_dps = in.angular_rate_dps;
    ctx.vs_jitter_mps = in.vertical_speed_jitter_mps;
    ctx.gps_valid = in.gps_valid;
    ctx.imu_valid = in.imu_valid;
    ctx.baro_valid = in.barometer_valid;
    ctx.flare_enabled = flareEnabled();

    out.target_valid = validTarget(in.target_latitude, in.target_longitude);
    const FailCode preflight_failure = readinessFailure(in, out.target_valid);
    out.launch_readiness_ok = preflight_failure == FailCode::FAIL_NONE;
    const bool launch_attempt = preflightState(state_) && launchAttemptDetected(in, config_.state_machine);
    if (launch_attempt && !out.launch_readiness_ok) {
        if (preflight_launch_attempt_since_ms_ == 0) {
            preflight_launch_attempt_since_ms_ = in.now_ms;
        }
        out.preflight_launch_warning =
            in.now_ms - preflight_launch_attempt_since_ms_ >= config_.state_machine.launch_confirm_ms;
        if (out.preflight_launch_warning) {
            preflight_launch_blocked_ = true;
            latchFailure(preflight_failure, in.now_ms);
        }
    } else {
        preflight_launch_attempt_since_ms_ = 0;
        out.preflight_launch_warning = preflight_launch_blocked_;
    }

    if (out.target_valid && in.gps_valid) {
        ctx.distance_to_target_m = distanceMeters(in.latitude, in.longitude,
                                                  in.target_latitude, in.target_longitude);
    }

    StateMachineResult sm = updateStateMachine(ctx, state_runtime_, state_, failure_);
    if (sm.fail_code != FailCode::FAIL_NONE) latchFailure(sm.fail_code, in.now_ms);
    if ((failure_ == FailCode::FAIL_NONE || sm.new_state == FlightState::LANDED) &&
        sm.new_state != state_) {
        state_ = sm.new_state;
        recordTransition(state_, in.now_ms);
    }

    // Always derive the mode from the current state on every iteration. A
    // missing target disables navigation even after stabilization completes.
    GuidanceMode mode = guidanceModeForState(state_);
    out.guidance_ready = out.target_valid && in.gps_valid && in.servo_valid &&
                         (in.imu_valid || out.imu_fallback_active) &&
                         (in.barometer_valid || out.barometer_fallback_active);
    if ((mode == GuidanceMode::HEADING_TO_TARGET || mode == GuidanceMode::FINAL_APPROACH) &&
        !out.target_valid) mode = GuidanceMode::MODE_DISABLED;
    if (failure_ != FailCode::FAIL_NONE && state_ != FlightState::LANDED)
        mode = GuidanceMode::MODE_FAILSAFE;

    GuidanceInput guidance_in;
    guidance_in.current_lat = in.latitude;
    guidance_in.current_lon = in.longitude;
    guidance_in.target_lat = in.target_latitude;
    guidance_in.target_lon = in.target_longitude;
    guidance_in.altitude_agl_m = in.altitude_agl_m;
    guidance_in.ground_speed_mps = in.ground_speed_mps;
    guidance_in.gps_course_deg = in.gps_course_deg;
    guidance_in.gps_valid = in.gps_valid && out.target_valid;
    guidance_in.imu_valid = in.imu_valid;
    guidance_in.baro_valid = in.barometer_valid;
    guidance_in.angular_rate_dps = in.angular_rate_dps;
    guidance_in.mode = mode;
    guidance_in.now_ms = in.now_ms;
    guidance_in.dt_s = in.dt_s;
    GuidanceOutput guidance = computeGuidance(guidance_in, config_.guidance,
                                              rate_limiter_, reversal_guard_);

    out.state = state_;
    out.mode = mode;
    out.failure = failure_;
    out.state_changed = state_ != starting_state;
    out.failsafe_active = failure_ != FailCode::FAIL_NONE || state_ == FlightState::FAILSAFE_DESCENT;
    out.target_bearing_deg = guidance.target_bearing_deg;
    out.heading_error_deg = guidance.heading_error_deg;
    out.distance_to_target_m = guidance.distance_to_target_m;
    out.steering_command = guidance.steering_command;
    out.servo_saturated = guidance.saturated;

    if (out.failsafe_active || state_ == FlightState::LANDED || mode == GuidanceMode::MODE_DISABLED) {
        // Initial consolidated failsafe policy: direct, immediate neutral only.
        out.requested_left_brake = 0.0f;
        out.requested_right_brake = 0.0f;
    } else {
        out.requested_left_brake = guidance.steering_command < 0.0f ? -guidance.steering_command : 0.0f;
        out.requested_right_brake = guidance.steering_command > 0.0f ? guidance.steering_command : 0.0f;
    }
    if (state_ == FlightState::LANDED && !log_closed_) {
        out.close_log = true;
        log_closed_ = true;
    }
    return out;
}

}  // namespace logic
