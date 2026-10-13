#include "guided_drop_controller.h"

#include <algorithm>
#include <cmath>

namespace logic {

namespace {

float normalized360(float degrees) {
    while (degrees >= 360.0f) degrees -= 360.0f;
    while (degrees < 0.0f) degrees += 360.0f;
    return degrees;
}

float safeAuthority(float requested, float configured) {
    if (!std::isfinite(requested) || requested <= 0.0f) return 0.0f;
    return std::min(std::min(requested, configured), 1.0f);
}

}  // namespace

GuidedDropController::GuidedDropController(const GuidedDropConfig& config)
    : config_(config),
      rate_limiter_(config.guidance.cmd_rate_limit_per_s),
      reversal_guard_(config.guidance.reversal_guard_ms) {
    config_.guidance.flare_enabled = false;
    reset();
}

void GuidedDropController::resetNavigationTimers() {
    navigation_candidate_active_ = false;
    navigation_candidate_since_ms_ = 0;
    navigation_loss_active_ = false;
    navigation_loss_since_ms_ = 0;
}

void GuidedDropController::resetGpsQuality() {
    have_accepted_position_ = false;
    gps_suspect_ = false;
    last_accepted_latitude_ = 0.0;
    last_accepted_longitude_ = 0.0;
    last_accepted_course_deg_ = 0.0f;
    last_accepted_position_ms_ = 0;
    gps_recovery_count_ = 0;
    have_course_hold_reference_ = false;
    imu_to_course_offset_deg_ = 0.0f;
}

void GuidedDropController::reset() {
    output_ = GuidedDropOutput{};
    output_.available = config_.enabled;
    rate_limiter_.reset(0.0f);
    reversal_guard_.reset();
    request_was_active_ = false;
    guidance_started_ = false;
    fault_latched_ = false;
    instability_since_ms_ = 0;
    last_heading_error_sign_ = 0;
    heading_sign_changes_ = 0;
    oscillation_window_start_ms_ = 0;
    resetNavigationTimers();
    resetGpsQuality();
}

bool GuidedDropController::validTarget(double latitude, double longitude) {
    if (!std::isfinite(latitude) || !std::isfinite(longitude)) return false;
    if (latitude < -90.0 || latitude > 90.0 ||
        longitude < -180.0 || longitude > 180.0) return false;
    return !(std::fabs(latitude) < 1e-9 && std::fabs(longitude) < 1e-9);
}

bool GuidedDropController::landedState(DropTestState state) {
    return state == DropTestState::POST_LANDING ||
           state == DropTestState::TEST_COMPLETE ||
           state == DropTestState::TEST_ABORTED;
}

bool GuidedDropController::postReleaseState(DropTestState state) {
    return state == DropTestState::DROP_CONFIRMED ||
           state == DropTestState::CANOPY_EVENT_SUSPECTED ||
           state == DropTestState::STABLE_DESCENT_OBSERVED ||
           state == DropTestState::LANDING_CONFIRM;
}

bool GuidedDropController::sensorsFresh(const GuidedDropInput& in) const {
    return in.imu_valid && in.barometer_valid && in.servo_valid &&
           in.imu_age_ms <= config_.sensor_max_age_ms &&
           in.barometer_age_ms <= config_.sensor_max_age_ms;
}

bool GuidedDropController::acceptGpsPoint(const GuidedDropInput& in) {
    const bool basic_valid = in.gps_valid &&
        in.gps_age_ms <= config_.sensor_max_age_ms &&
        validTarget(in.latitude, in.longitude) &&
        std::isfinite(in.gps_course_deg) &&
        std::isfinite(in.ground_speed_mps);
    if (!basic_valid) return false;

    if (have_accepted_position_) {
        const float jump_m = distanceMeters(last_accepted_latitude_,
                                             last_accepted_longitude_,
                                             in.latitude, in.longitude);
        if (!std::isfinite(jump_m) || jump_m > config_.gps_max_position_jump_m) {
            gps_suspect_ = true;
            gps_recovery_count_ = 0;
            return false;
        }
    }

    if (gps_suspect_) {
        if (gps_recovery_count_ < 255) ++gps_recovery_count_;
        if (gps_recovery_count_ < config_.gps_recovery_fix_count) return false;
        gps_suspect_ = false;
        gps_recovery_count_ = 0;
    }

    have_accepted_position_ = true;
    last_accepted_latitude_ = in.latitude;
    last_accepted_longitude_ = in.longitude;
    last_accepted_course_deg_ = normalized360(in.gps_course_deg);
    last_accepted_position_ms_ = in.now_ms;
    if (in.imu_valid && std::isfinite(in.imu_heading_deg)) {
        imu_to_course_offset_deg_ = headingErrorDegrees(
            in.imu_heading_deg, last_accepted_course_deg_);
        have_course_hold_reference_ = true;
    }
    return true;
}

float GuidedDropController::updateOscillationDamping(float heading_error_deg,
                                                      uint32_t now_ms) {
    if (oscillation_window_start_ms_ == 0 ||
        static_cast<uint32_t>(now_ms - oscillation_window_start_ms_) >
            config_.oscillation_window_ms) {
        oscillation_window_start_ms_ = now_ms;
        heading_sign_changes_ = 0;
        last_heading_error_sign_ = 0;
    }
    const int8_t sign = heading_error_deg > config_.guidance.deadband_deg ? 1 :
        (heading_error_deg < -config_.guidance.deadband_deg ? -1 : 0);
    if (sign != 0 && last_heading_error_sign_ != 0 &&
        sign != last_heading_error_sign_) {
        if (heading_sign_changes_ < 255) ++heading_sign_changes_;
    }
    if (sign != 0) last_heading_error_sign_ = sign;
    return heading_sign_changes_ >= config_.oscillation_sign_change_limit
        ? config_.oscillation_authority_scale : 1.0f;
}

GuidedDropOutput GuidedDropController::neutral(GuidedDropStatus status) {
    output_.status = status;
    output_.mode = status == GuidedDropStatus::FAULT_NEUTRAL
        ? GuidanceMode::MODE_FAILSAFE : GuidanceMode::MODE_DISABLED;
    output_.active = false;
    output_.neutral_required = true;
    output_.fault_latched = fault_latched_;
    output_.left_brake = 0.0f;
    output_.right_brake = 0.0f;
    output_.steering_command = 0.0f;
    output_.gps_suspect = gps_suspect_;
    return output_;
}

GuidedDropOutput GuidedDropController::unwindNavigationLoss(
    const GuidedDropInput& input) {
    const float command = rate_limiter_.update(0.0f, input.dt_s);
    output_.status = gps_suspect_ ? GuidedDropStatus::GPS_SUSPECT
                                  : GuidedDropStatus::WAITING_NAVIGATION;
    output_.mode = GuidanceMode::MODE_DISABLED;
    output_.steering_command = command;
    output_.left_brake = command < 0.0f ? -command : 0.0f;
    output_.right_brake = command > 0.0f ? command : 0.0f;
    output_.active = std::fabs(command) > 0.0005f;
    output_.neutral_required = !output_.active;
    output_.gps_suspect = gps_suspect_;
    return output_;
}

GuidedDropOutput GuidedDropController::runGuidance(
    const GuidedDropInput& in, double latitude, double longitude,
    float course_deg, float ground_speed_mps, GuidanceMode mode,
    float authority_limit, GuidedDropStatus status) {
    GuidanceConfig guidance_config = config_.guidance;
    guidance_config.max_steering = std::min(guidance_config.max_steering,
                                            authority_limit);
    guidance_config.final_approach_max_command =
        std::min(guidance_config.final_approach_max_command, authority_limit);
    guidance_config.low_speed_max_command =
        std::min(guidance_config.low_speed_max_command, authority_limit);
    guidance_config.unreachable_max_command = authority_limit;

    const float target_bearing = bearingDegrees(latitude, longitude,
                                                 in.target_latitude,
                                                 in.target_longitude);
    const float heading_error = headingErrorDegrees(course_deg, target_bearing);
    const float damping_scale = updateOscillationDamping(heading_error, in.now_ms);
    const bool damping = damping_scale < 0.999f;
    guidance_config.max_steering *= damping_scale;
    guidance_config.final_approach_max_command *= damping_scale;
    guidance_config.deadband_deg *= damping ? 1.5f : 1.0f;

    if (in.angular_rate_dps > config_.reduced_authority_angular_rate_dps) {
        guidance_config.max_steering *= 0.5f;
        guidance_config.final_approach_max_command *= 0.5f;
    }

    GuidanceInput guidance_in;
    guidance_in.current_lat = latitude;
    guidance_in.current_lon = longitude;
    guidance_in.target_lat = in.target_latitude;
    guidance_in.target_lon = in.target_longitude;
    guidance_in.altitude_agl_m = in.altitude_agl_m;
    guidance_in.ground_speed_mps = ground_speed_mps;
    guidance_in.gps_course_deg = course_deg;
    guidance_in.gps_valid = true;
    guidance_in.imu_valid = true;
    guidance_in.baro_valid = true;
    guidance_in.angular_rate_dps = in.angular_rate_dps;
    guidance_in.mode = mode;
    guidance_in.now_ms = in.now_ms;
    guidance_in.dt_s = in.dt_s;

    const GuidanceOutput guidance = computeGuidance(
        guidance_in, guidance_config, rate_limiter_, reversal_guard_);
    guidance_started_ = true;
    output_.status = status;
    output_.mode = mode;
    output_.active = guidance.guidance_active;
    output_.neutral_required = !guidance.guidance_active;
    output_.fault_latched = false;
    output_.steering_command = guidance.steering_command;
    output_.target_bearing_deg = guidance.target_bearing_deg;
    output_.heading_error_deg = guidance.heading_error_deg;
    output_.distance_to_target_m = guidance.distance_to_target_m;
    output_.target_reachable = guidance.target_reachable;
    output_.oscillation_damping_active = damping;
    output_.left_brake = guidance.steering_command < 0.0f
        ? -guidance.steering_command : 0.0f;
    output_.right_brake = guidance.steering_command > 0.0f
        ? guidance.steering_command : 0.0f;
    return output_;
}

GuidedDropOutput GuidedDropController::step(const GuidedDropInput& in) {
    output_ = GuidedDropOutput{};
    const bool available = config_.enabled;
    const bool target_valid = validTarget(in.target_latitude, in.target_longitude);
    const float authority_limit = safeAuthority(in.authority_limit,
                                                config_.guidance.max_steering);
    output_.available = available;
    output_.requested = in.guided_test_requested;
    output_.target_valid = target_valid;
    output_.authority = in.authority;
    output_.authority_limit = authority_limit;

    if (!available) {
        if (request_was_active_) reset();
        output_.available = false;
        output_.requested = in.guided_test_requested;
        output_.target_valid = target_valid;
        output_.authority = in.authority;
        return neutral(GuidedDropStatus::FEATURE_DISABLED);
    }
    if (!in.guided_test_requested || !in.drop_recording) {
        if (request_was_active_) reset();
        request_was_active_ = in.guided_test_requested && in.drop_recording;
        output_.available = available;
        output_.requested = in.guided_test_requested;
        output_.target_valid = target_valid;
        output_.authority = in.authority;
        return neutral(GuidedDropStatus::WAITING_RELEASE);
    }
    request_was_active_ = true;

    if (landedState(in.drop_state)) return neutral(GuidedDropStatus::LANDED);
    if (in.abort_latched || fault_latched_) {
        fault_latched_ = true;
        return neutral(GuidedDropStatus::FAULT_NEUTRAL);
    }
    if (in.authority == GuidedAuthorityLevel::NEUTRAL || authority_limit <= 0.0f) {
        fault_latched_ = true;
        return neutral(GuidedDropStatus::FAULT_NEUTRAL);
    }
    if (!sensorsFresh(in)) {
        fault_latched_ = true;
        return neutral(GuidedDropStatus::FAULT_NEUTRAL);
    }
    if (!postReleaseState(in.drop_state) || in.release_confirm_ms == 0) {
        resetNavigationTimers();
        return neutral(GuidedDropStatus::WAITING_RELEASE);
    }
    if (static_cast<uint32_t>(in.now_ms - in.release_confirm_ms) <
        config_.deployment_neutral_ms) {
        resetNavigationTimers();
        return neutral(GuidedDropStatus::DEPLOYMENT_NEUTRAL);
    }
    if (!in.canopy_event_observed || !in.stable_descent_observed ||
        !std::isfinite(in.angular_rate_dps) ||
        (!guidance_started_ &&
         in.angular_rate_dps > config_.max_activation_angular_rate_dps)) {
        navigation_candidate_active_ = false;
        return neutral(GuidedDropStatus::WAITING_STABLE_DESCENT);
    }
    if (!std::isfinite(in.altitude_agl_m) ||
        (!guidance_started_ && in.altitude_agl_m < config_.min_activation_altitude_m)) {
        navigation_candidate_active_ = false;
        return neutral(GuidedDropStatus::WAITING_ALTITUDE);
    }

    if (in.angular_rate_dps > config_.neutral_angular_rate_dps) {
        if (instability_since_ms_ == 0) instability_since_ms_ = in.now_ms;
        if (static_cast<uint32_t>(in.now_ms - instability_since_ms_) >=
            config_.instability_latch_ms) {
            fault_latched_ = true;
            return neutral(GuidedDropStatus::FAULT_NEUTRAL);
        }
        return unwindNavigationLoss(in);
    }
    instability_since_ms_ = 0;

    const bool gps_point_accepted = acceptGpsPoint(in);
    output_.gps_point_accepted = gps_point_accepted;
    output_.gps_suspect = gps_suspect_;
    const bool navigation_valid = target_valid && gps_point_accepted;
    if (!navigation_valid) {
        navigation_candidate_active_ = false;
        if (guidance_started_) {
            if (!navigation_loss_active_) {
                navigation_loss_active_ = true;
                navigation_loss_since_ms_ = in.now_ms;
            }
            const uint32_t loss_age = in.now_ms - navigation_loss_since_ms_;
            if (config_.imu_course_hold_enabled && have_accepted_position_ &&
                have_course_hold_reference_ && in.imu_valid &&
                std::isfinite(in.imu_heading_deg) &&
                loss_age <= config_.imu_course_hold_max_ms) {
                const float estimated_course = normalized360(
                    in.imu_heading_deg + imu_to_course_offset_deg_);
                output_.course_hold_active = true;
                return runGuidance(in, last_accepted_latitude_,
                                   last_accepted_longitude_, estimated_course,
                                   config_.guidance.min_course_speed_mps,
                                   GuidanceMode::HEADING_TO_TARGET,
                                   std::min(authority_limit,
                                            config_.imu_course_hold_max_command),
                                   GuidedDropStatus::GPS_COURSE_HOLD);
            }
            if (loss_age >= config_.navigation_loss_latch_ms) {
                fault_latched_ = true;
                return neutral(GuidedDropStatus::FAULT_NEUTRAL);
            }
            return unwindNavigationLoss(in);
        }
        return neutral(gps_suspect_ ? GuidedDropStatus::GPS_SUSPECT
                                    : GuidedDropStatus::WAITING_NAVIGATION);
    }
    navigation_loss_active_ = false;
    navigation_loss_since_ms_ = 0;

    if (!navigation_candidate_active_) {
        navigation_candidate_active_ = true;
        navigation_candidate_since_ms_ = in.now_ms;
        return neutral(GuidedDropStatus::WAITING_NAVIGATION);
    }
    if (static_cast<uint32_t>(in.now_ms - navigation_candidate_since_ms_) <
        config_.navigation_confirm_ms) {
        return neutral(GuidedDropStatus::WAITING_NAVIGATION);
    }

    const float distance_m = distanceMeters(in.latitude, in.longitude,
                                             in.target_latitude,
                                             in.target_longitude);
    const bool final_approach =
        in.altitude_agl_m <= config_.guidance.final_approach_altitude_m ||
        distance_m <= config_.final_approach_radius_m;
    return runGuidance(in, in.latitude, in.longitude, in.gps_course_deg,
                       in.ground_speed_mps,
                       final_approach ? GuidanceMode::FINAL_APPROACH
                                      : GuidanceMode::HEADING_TO_TARGET,
                       authority_limit,
                       final_approach ? GuidedDropStatus::FINAL_APPROACH
                                      : GuidedDropStatus::GUIDANCE_ACTIVE);
}

}  // namespace logic
