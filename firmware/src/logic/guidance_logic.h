// ============================================================================
// PHOENIX RECOVERY — Guidance logic (pure C++, zero hardware deps).
// Proportional steering, deadband, clamp, slew, reversal guard, mode selection.
// Host-testable.
// ============================================================================
#pragma once

#include "nav_math.h"
#include "filters.h"
#include "state_machine.h"

namespace logic {

// ---------------------------------------------------------------------------
// Guidance configuration (from config.h)
// ---------------------------------------------------------------------------
struct GuidanceConfig {
    float kp = 0.02f;
    float deadband_deg = 5.0f;
    float max_steering = 0.35f;
    float cmd_rate_limit_per_s = 0.5f;
    uint32_t reversal_guard_ms = 800;

    float final_approach_kp = 0.010f;
    float final_approach_max_command = 0.25f;
    float final_approach_altitude_m = 30.0f;

    bool flare_enabled = false;
    float flare_altitude_m = 8.0f;
    float flare_max_brake = 0.6f;
    float flare_stall_release_dps = 60.0f;

    float min_guidance_altitude_m = 12.0f;
    float target_accept_radius_m = 10.0f;
    float min_course_speed_mps = 3.0f;
    float unstable_gyro_rate_dps = 25.0f;
    float reduced_authority = 0.35f;
    float final_deadband_multiplier = 2.0f;
    float low_speed_deadband_multiplier = 3.0f;
    float nominal_glide_ratio = 3.0f; // REQUIRES EXPERIMENTAL CALIBRATION
};

// ---------------------------------------------------------------------------
// Guidance input (from VehicleState)
// ---------------------------------------------------------------------------
struct GuidanceInput {
    double current_lat = 0.0;
    double current_lon = 0.0;
    double target_lat = 0.0;
    double target_lon = 0.0;
    float altitude_agl_m = 0.0f;
    float ground_speed_mps = 0.0f;
    float gps_course_deg = 0.0f;
    bool gps_valid = false;
    bool imu_valid = false;
    bool baro_valid = false;
    float angular_rate_dps = 0.0f;
    GuidanceMode mode = GuidanceMode::MODE_DISABLED;
    uint32_t now_ms = 0;
    float dt_s = 0.0f;
};

// ---------------------------------------------------------------------------
// Guidance output
// ---------------------------------------------------------------------------
struct GuidanceOutput {
    float steering_command = 0.0f;   // [-1, +1], pos = right turn
    float target_bearing_deg = 0.0f;
    float heading_error_deg = 0.0f;
    float distance_to_target_m = 0.0f;
    bool guidance_active = false;
    bool course_reliable = false;
    bool target_reachable = true;
    bool saturated = false;
    GuidanceMode active_mode = GuidanceMode::MODE_DISABLED;
};

// ---------------------------------------------------------------------------
// Compute guidance command
// ---------------------------------------------------------------------------
GuidanceOutput computeGuidance(const GuidanceInput& in, const GuidanceConfig& cfg,
                               RateLimiter& rate_limiter, ReversalGuard& reversal_guard);

} // namespace logic
