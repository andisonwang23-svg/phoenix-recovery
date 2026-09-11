#pragma once

#include <cstdint>
#include "guidance_logic.h"
#include "state_machine.h"

namespace logic {

struct CoordinatorConfig {
    StateMachineContext state_machine{};
    GuidanceConfig guidance{};
    bool flare_enabled = false;
};

struct CoordinatorInput {
    uint32_t now_ms = 0;
    float dt_s = 0.02f;
    double latitude = 0.0;
    double longitude = 0.0;
    double target_latitude = 0.0;
    double target_longitude = 0.0;
    float altitude_agl_m = 0.0f;
    float vertical_speed_mps = 0.0f;
    float vertical_accel_mps2 = 0.0f;
    float ground_speed_mps = 0.0f;
    float gps_course_deg = 0.0f;
    float angular_rate_dps = 0.0f;
    float vertical_speed_jitter_mps = 0.0f;
    bool gps_valid = false;
    bool imu_valid = false;
    bool barometer_valid = false;
    bool servo_valid = true;
};

struct TransitionTimestamps {
    uint32_t state_entry_ms = 0;
    uint32_t launch_ms = 0;
    uint32_t apogee_ms = 0;
    uint32_t deployment_wait_ms = 0;
    uint32_t stabilization_ms = 0;
    uint32_t guidance_ms = 0;
    uint32_t final_approach_ms = 0;
    uint32_t flare_ms = 0;
    uint32_t landed_ms = 0;
    uint32_t failsafe_ms = 0;
};

struct CoordinatorOutput {
    FlightState state = FlightState::BOOT;
    GuidanceMode mode = GuidanceMode::MODE_DISABLED;
    FailCode failure = FailCode::FAIL_NONE;
    float requested_left_brake = 0.0f;
    float requested_right_brake = 0.0f;
    float steering_command = 0.0f;
    float target_bearing_deg = 0.0f;
    float heading_error_deg = 0.0f;
    float distance_to_target_m = 0.0f;
    bool target_valid = false;
    bool guidance_ready = false;
    bool state_changed = false;
    bool failsafe_active = false;
    bool close_log = false;
    bool servo_saturated = false;
    bool gps_fallback_active = false;
    bool imu_fallback_active = false;
    bool barometer_fallback_active = false;
    bool degraded_guidance = false;
    bool launch_readiness_ok = false;
    bool preflight_launch_warning = false;
};

class FlightCoordinator {
public:
    explicit FlightCoordinator(const CoordinatorConfig& config = CoordinatorConfig());
    void reset(uint32_t now_ms = 0);
    CoordinatorOutput step(const CoordinatorInput& input);
    const TransitionTimestamps& timestamps() const { return timestamps_; }
    FlightState state() const { return state_; }
    FailCode failure() const { return failure_; }
    bool flareEnabled() const { return config_.flare_enabled && config_.guidance.flare_enabled; }

private:
    CoordinatorConfig config_;
    StateMachineRuntime state_runtime_;
    RateLimiter rate_limiter_;
    ReversalGuard reversal_guard_;
    TransitionTimestamps timestamps_;
    FlightState state_ = FlightState::BOOT;
    FailCode failure_ = FailCode::FAIL_NONE;
    bool log_closed_ = false;
    uint32_t preflight_launch_attempt_since_ms_ = 0;
    bool preflight_launch_blocked_ = false;

    static bool validTarget(double latitude, double longitude);
    static bool preflightState(FlightState state);
    static bool launchAttemptDetected(const CoordinatorInput& input, const StateMachineContext& config);
    static FailCode readinessFailure(const CoordinatorInput& input, bool target_valid);
    void recordTransition(FlightState next, uint32_t now_ms);
    void latchFailure(FailCode failure, uint32_t now_ms);
};

}  // namespace logic
