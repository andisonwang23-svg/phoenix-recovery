// ============================================================================
// PHOENIX RECOVERY — guarded return-to-target controller for inert drop tests.
//
// This controller never handles launch. It can own the brakes only after an
// explicitly armed drop recorder confirms release, observes an automatic
// canopy load signature, the deployment-neutral delay has elapsed, stable
// descent has persisted, and navigation is valid.
// ============================================================================
#pragma once

#include <cstdint>

#include "drop_test_controller.h"
#include "guided_drop_arm_gate.h"
#include "guidance_logic.h"

namespace logic {

enum class GuidedDropStatus : uint8_t {
    FEATURE_DISABLED = 0,
    WAITING_RELEASE = 1,
    DEPLOYMENT_NEUTRAL = 2,
    WAITING_STABLE_DESCENT = 3,
    WAITING_NAVIGATION = 4,
    WAITING_ALTITUDE = 5,
    GUIDANCE_ACTIVE = 6,
    FINAL_APPROACH = 7,
    FAULT_NEUTRAL = 8,
    LANDED = 9,
    GPS_SUSPECT = 10,
    GPS_COURSE_HOLD = 11
};

inline const char* guidedDropStatusName(GuidedDropStatus status) {
    switch (status) {
        case GuidedDropStatus::FEATURE_DISABLED: return "DISABLED";
        case GuidedDropStatus::WAITING_RELEASE: return "WAITING_RELEASE";
        case GuidedDropStatus::DEPLOYMENT_NEUTRAL: return "DEPLOYMENT_NEUTRAL";
        case GuidedDropStatus::WAITING_STABLE_DESCENT: return "WAITING_STABLE_DESCENT";
        case GuidedDropStatus::WAITING_NAVIGATION: return "WAITING_NAVIGATION";
        case GuidedDropStatus::WAITING_ALTITUDE: return "WAITING_ALTITUDE";
        case GuidedDropStatus::GUIDANCE_ACTIVE: return "GUIDANCE_ACTIVE";
        case GuidedDropStatus::FINAL_APPROACH: return "FINAL_APPROACH";
        case GuidedDropStatus::FAULT_NEUTRAL: return "FAULT_NEUTRAL";
        case GuidedDropStatus::LANDED: return "LANDED";
        case GuidedDropStatus::GPS_SUSPECT: return "GPS_SUSPECT";
        case GuidedDropStatus::GPS_COURSE_HOLD: return "GPS_COURSE_HOLD";
    }
    return "UNKNOWN";
}

struct GuidedDropConfig {
    bool enabled = false;
    uint32_t deployment_neutral_ms = 1500;
    uint32_t navigation_confirm_ms = 2000;
    uint32_t navigation_loss_latch_ms = 2000;
    uint32_t sensor_max_age_ms = 1000;
    float min_activation_altitude_m = 75.0f;
    float max_activation_angular_rate_dps = 12.0f;
    float reduced_authority_angular_rate_dps = 12.0f;
    float neutral_angular_rate_dps = 25.0f;
    uint32_t instability_latch_ms = 2000;
    float final_approach_radius_m = 40.0f;
    float gps_max_position_jump_m = 50.0f;
    uint8_t gps_recovery_fix_count = 3;
    bool imu_course_hold_enabled = false;
    uint32_t imu_course_hold_max_ms = 1500;
    float imu_course_hold_max_command = 0.05f;
    float oscillation_authority_scale = 0.50f;
    uint8_t oscillation_sign_change_limit = 4;
    uint32_t oscillation_window_ms = 4000;
    GuidanceConfig guidance{};
};

struct GuidedDropInput {
    uint32_t now_ms = 0;
    float dt_s = 0.02f;
    bool guided_test_requested = false;
    bool drop_recording = false;
    DropTestState drop_state = DropTestState::IDLE;
    uint32_t release_confirm_ms = 0;
    bool stable_descent_observed = false;
    bool canopy_event_observed = false;
    bool abort_latched = false;
    GuidedAuthorityLevel authority = GuidedAuthorityLevel::NEUTRAL;
    float authority_limit = 0.0f;

    double latitude = 0.0;
    double longitude = 0.0;
    double target_latitude = 0.0;
    double target_longitude = 0.0;
    float altitude_agl_m = 0.0f;
    float ground_speed_mps = 0.0f;
    float gps_course_deg = 0.0f;
    float imu_heading_deg = 0.0f;
    float angular_rate_dps = 0.0f;

    bool gps_valid = false;
    bool imu_valid = false;
    bool barometer_valid = false;
    bool servo_valid = false;
    uint32_t gps_age_ms = 0;
    uint32_t imu_age_ms = 0;
    uint32_t barometer_age_ms = 0;
};

struct GuidedDropOutput {
    GuidedDropStatus status = GuidedDropStatus::FEATURE_DISABLED;
    GuidanceMode mode = GuidanceMode::MODE_DISABLED;
    bool available = false;
    bool requested = false;
    bool active = false;
    bool neutral_required = true;
    bool fault_latched = false;
    bool target_valid = false;
    bool target_reachable = false;
    bool gps_point_accepted = false;
    bool gps_suspect = false;
    bool course_hold_active = false;
    bool oscillation_damping_active = false;
    GuidedAuthorityLevel authority = GuidedAuthorityLevel::NEUTRAL;
    float authority_limit = 0.0f;
    float left_brake = 0.0f;
    float right_brake = 0.0f;
    float steering_command = 0.0f;
    float target_bearing_deg = 0.0f;
    float heading_error_deg = 0.0f;
    float distance_to_target_m = 0.0f;
};

class GuidedDropController {
public:
    explicit GuidedDropController(const GuidedDropConfig& config = GuidedDropConfig());
    void reset();
    GuidedDropOutput step(const GuidedDropInput& input);
    const GuidedDropOutput& output() const { return output_; }

private:
    GuidedDropConfig config_;
    GuidedDropOutput output_;
    RateLimiter rate_limiter_;
    ReversalGuard reversal_guard_;
    bool request_was_active_ = false;
    bool navigation_candidate_active_ = false;
    uint32_t navigation_candidate_since_ms_ = 0;
    bool navigation_loss_active_ = false;
    uint32_t navigation_loss_since_ms_ = 0;
    bool guidance_started_ = false;
    bool fault_latched_ = false;
    bool have_accepted_position_ = false;
    bool gps_suspect_ = false;
    double last_accepted_latitude_ = 0.0;
    double last_accepted_longitude_ = 0.0;
    float last_accepted_course_deg_ = 0.0f;
    uint32_t last_accepted_position_ms_ = 0;
    uint8_t gps_recovery_count_ = 0;
    bool have_course_hold_reference_ = false;
    float imu_to_course_offset_deg_ = 0.0f;
    uint32_t instability_since_ms_ = 0;
    int8_t last_heading_error_sign_ = 0;
    uint8_t heading_sign_changes_ = 0;
    uint32_t oscillation_window_start_ms_ = 0;

    static bool validTarget(double latitude, double longitude);
    static bool landedState(DropTestState state);
    static bool postReleaseState(DropTestState state);
    void resetNavigationTimers();
    void resetGpsQuality();
    bool acceptGpsPoint(const GuidedDropInput& input);
    bool sensorsFresh(const GuidedDropInput& input) const;
    float updateOscillationDamping(float heading_error_deg, uint32_t now_ms);
    GuidedDropOutput neutral(GuidedDropStatus status);
    GuidedDropOutput unwindNavigationLoss(const GuidedDropInput& input);
    GuidedDropOutput runGuidance(const GuidedDropInput& input,
                                 double latitude, double longitude,
                                 float course_deg, float ground_speed_mps,
                                 GuidanceMode mode, float authority_limit,
                                 GuidedDropStatus status);
};

} // namespace logic
