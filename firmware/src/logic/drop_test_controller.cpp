#include "drop_test_controller.h"

#include <algorithm>
#include <cmath>

namespace logic {

DropTestController::DropTestController(const DropTestConfig& config)
    : config_(config) {}

bool DropTestController::elapsed(uint32_t now, uint32_t since, uint32_t duration) {
    return since != 0 && static_cast<uint32_t>(now - since) >= duration;
}

DropTestOutput DropTestController::arm(uint32_t now_ms, float baseline_altitude_m) {
    output_ = DropTestOutput{};
    output_.state = DropTestState::ARMED_RECORDING;
    output_.recording = true;
    output_.neutral_lock = true;
    output_.armed_ms = now_ms;
    arm_altitude_m_ = baseline_altitude_m;
    fastest_descent_mps_ = 0.0f;
    release_candidate_since_ms_ = 0;
    stable_window_start_ms_ = 0;
    landing_candidate_since_ms_ = 0;
    post_landing_until_ms_ = 0;
    return output_;
}

DropTestOutput DropTestController::abort(uint32_t now_ms) {
    if (output_.recording) {
        output_.state = DropTestState::TEST_ABORTED;
        output_.recording = false;
        output_.neutral_lock = true;
        output_.close_log = true;
        output_.log_closed_ms = now_ms;
        output_.events = DROP_EVENT_ABORTED;
    }
    return output_;
}

bool DropTestController::isPostReleaseState() const {
    return output_.state == DropTestState::DROP_CONFIRMED ||
           output_.state == DropTestState::CANOPY_EVENT_SUSPECTED ||
           output_.state == DropTestState::STABLE_DESCENT_OBSERVED ||
           output_.state == DropTestState::LANDING_CONFIRM;
}

void DropTestController::resetStableWindow(const DropTestInput& input) {
    stable_window_start_ms_ = input.now_ms;
    stable_min_vs_mps_ = input.vertical_speed_mps;
    stable_max_vs_mps_ = input.vertical_speed_mps;
}

DropTestOutput DropTestController::step(const DropTestInput& input) {
    output_.events = DROP_EVENT_NONE;
    output_.close_log = false;
    if (!output_.recording) return output_;

    if (static_cast<uint32_t>(input.now_ms - output_.armed_ms) >= config_.max_duration_ms) {
        output_.state = DropTestState::TEST_COMPLETE;
        output_.recording = false;
        output_.neutral_lock = true;
        output_.close_log = true;
        output_.log_closed_ms = input.now_ms;
        output_.events = DROP_EVENT_TIMEOUT | DROP_EVENT_COMPLETE;
        return output_;
    }

    if (output_.state == DropTestState::ARMED_RECORDING ||
        output_.state == DropTestState::RELEASE_CANDIDATE) {
        const float altitude_loss_m = arm_altitude_m_ - input.altitude_agl_m;
        const bool release_candidate = input.barometer_valid &&
            std::isfinite(input.altitude_agl_m) &&
            std::isfinite(input.vertical_speed_mps) &&
            input.vertical_speed_mps <= config_.release_speed_mps &&
            altitude_loss_m >= config_.release_altitude_loss_m;

        if (release_candidate) {
            if (release_candidate_since_ms_ == 0) {
                release_candidate_since_ms_ = input.now_ms;
                output_.release_onset_ms = input.now_ms;
                output_.state = DropTestState::RELEASE_CANDIDATE;
                output_.events |= DROP_EVENT_RELEASE_ONSET;
            }
            if (elapsed(input.now_ms, release_candidate_since_ms_, config_.release_confirm_ms)) {
                output_.state = DropTestState::DROP_CONFIRMED;
                output_.release_confirm_ms = input.now_ms;
                fastest_descent_mps_ = input.vertical_speed_mps;
                resetStableWindow(input);
                output_.events |= DROP_EVENT_RELEASE_CONFIRMED;
            }
        } else if (release_candidate_since_ms_ != 0) {
            release_candidate_since_ms_ = 0;
            output_.release_onset_ms = 0;
            output_.state = DropTestState::ARMED_RECORDING;
            output_.events |= DROP_EVENT_RELEASE_REJECTED;
        }
    }

    if (isPostReleaseState() && output_.landing_confirm_ms == 0) {
        if (input.barometer_valid && std::isfinite(input.vertical_speed_mps)) {
            fastest_descent_mps_ = std::min(fastest_descent_mps_, input.vertical_speed_mps);
        }

        if (!output_.canopy_signature_observed &&
            elapsed(input.now_ms, output_.release_confirm_ms,
                    config_.canopy_observation_delay_ms)) {
            const bool baro_signature = input.barometer_valid &&
                std::isfinite(input.vertical_speed_mps) &&
                input.vertical_speed_mps - fastest_descent_mps_ >=
                    config_.canopy_deceleration_delta_mps;
            const bool imu_signature = input.imu_valid &&
                std::isfinite(input.vertical_accel_mps2) &&
                std::fabs(input.vertical_accel_mps2) >=
                    config_.canopy_accel_signature_mps2;
            if (baro_signature || imu_signature) {
                output_.canopy_signature_observed = true;
                output_.canopy_signature_ms = input.now_ms;
                output_.state = DropTestState::CANOPY_EVENT_SUSPECTED;
                output_.events |= DROP_EVENT_CANOPY_SIGNATURE;
            }
        }

        const bool stable_sample = input.barometer_valid && input.imu_valid &&
            std::isfinite(input.vertical_speed_mps) &&
            std::isfinite(input.angular_rate_dps) &&
            input.vertical_speed_mps <= -config_.stable_descent_min_down_speed_mps &&
            input.angular_rate_dps <= config_.stable_descent_max_angular_rate_dps;
        if (stable_sample) {
            if (stable_window_start_ms_ == 0) resetStableWindow(input);
            stable_min_vs_mps_ = std::min(stable_min_vs_mps_, input.vertical_speed_mps);
            stable_max_vs_mps_ = std::max(stable_max_vs_mps_, input.vertical_speed_mps);
            if (!output_.stable_descent_observed &&
                elapsed(input.now_ms, stable_window_start_ms_,
                        config_.stable_descent_window_ms) &&
                stable_max_vs_mps_ - stable_min_vs_mps_ <=
                    config_.stable_descent_max_vs_range_mps) {
                output_.stable_descent_observed = true;
                output_.stable_descent_ms = input.now_ms;
                output_.state = DropTestState::STABLE_DESCENT_OBSERVED;
                output_.events |= DROP_EVENT_STABLE_DESCENT;
            }
        } else {
            stable_window_start_ms_ = 0;
        }

        const bool landing_candidate = input.barometer_valid && input.imu_valid &&
            std::isfinite(input.altitude_agl_m) &&
            std::isfinite(input.vertical_speed_mps) &&
            std::isfinite(input.angular_rate_dps) &&
            std::fabs(input.vertical_speed_mps) <= config_.landing_max_vertical_speed_mps &&
            input.angular_rate_dps <= config_.landing_max_angular_rate_dps &&
            (!input.gps_valid ||
             (std::isfinite(input.ground_speed_mps) &&
              input.ground_speed_mps <= config_.landing_max_ground_speed_mps));

        if (landing_candidate) {
            if (landing_candidate_since_ms_ == 0) {
                landing_candidate_since_ms_ = input.now_ms;
                output_.landing_candidate_ms = input.now_ms;
                pre_landing_state_ = output_.state;
                landing_min_altitude_m_ = input.altitude_agl_m;
                landing_max_altitude_m_ = input.altitude_agl_m;
                output_.state = DropTestState::LANDING_CONFIRM;
                output_.events |= DROP_EVENT_LANDING_CANDIDATE;
            } else {
                landing_min_altitude_m_ = std::min(landing_min_altitude_m_, input.altitude_agl_m);
                landing_max_altitude_m_ = std::max(landing_max_altitude_m_, input.altitude_agl_m);
            }
            const float altitude_span_m = landing_max_altitude_m_ - landing_min_altitude_m_;
            if (elapsed(input.now_ms, landing_candidate_since_ms_,
                        config_.landing_confirm_ms) &&
                altitude_span_m <= config_.landing_max_altitude_span_m) {
                output_.landing_confirm_ms = input.now_ms;
                output_.state = DropTestState::POST_LANDING;
                post_landing_until_ms_ = input.now_ms + config_.post_landing_record_ms;
                output_.events |= DROP_EVENT_LANDING_CONFIRMED;
            }
        } else if (landing_candidate_since_ms_ != 0) {
            landing_candidate_since_ms_ = 0;
            output_.landing_candidate_ms = 0;
            output_.state = pre_landing_state_;
            output_.events |= DROP_EVENT_LANDING_REJECTED;
        }
    }

    if (output_.state == DropTestState::POST_LANDING &&
        static_cast<int32_t>(input.now_ms - post_landing_until_ms_) >= 0) {
        output_.state = DropTestState::TEST_COMPLETE;
        output_.recording = false;
        output_.neutral_lock = true;
        output_.close_log = true;
        output_.log_closed_ms = input.now_ms;
        output_.events |= DROP_EVENT_COMPLETE;
    }

    // This controller is an inert-test recorder. No state can release neutral.
    output_.neutral_lock = true;
    return output_;
}

} // namespace logic
