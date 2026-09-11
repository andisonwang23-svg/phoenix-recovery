// ============================================================================
// PHOENIX RECOVERY — Guidance logic implementation (pure C++).
// ============================================================================
#include "guidance_logic.h"
#include <cmath>

namespace logic {

GuidanceOutput computeGuidance(const GuidanceInput& in, const GuidanceConfig& cfg,
                               RateLimiter& rate_limiter, ReversalGuard& reversal_guard) {
    GuidanceOutput out;
    out.active_mode = in.mode;

    // No guidance in disabled/failsafe modes
    if (in.mode == GuidanceMode::MODE_DISABLED || in.mode == GuidanceMode::MODE_FAILSAFE) {
        out.guidance_active = false;
        out.steering_command = 0.0f;
        return out;
    }

    if (!in.gps_valid || !std::isfinite(in.current_lat) || !std::isfinite(in.current_lon) ||
        !std::isfinite(in.gps_course_deg)) {
        const float unwind = rate_limiter.update(0.0f, in.dt_s); // actively unwind an old hard turn
        out.guidance_active = false;
        out.steering_command = unwind;
        return out;
    }

    // Below minimum guidance altitude - no steering
    if (in.altitude_agl_m < cfg.min_guidance_altitude_m) {
        out.guidance_active = false;
        out.steering_command = 0.0f;
        return out;
    }

    // Compute navigation
    out.target_bearing_deg = bearingDegrees(in.current_lat, in.current_lon,
                                             in.target_lat, in.target_lon);
    out.distance_to_target_m = distanceMeters(in.current_lat, in.current_lon,
                                               in.target_lat, in.target_lon);
    out.heading_error_deg = headingErrorDegrees(in.gps_course_deg, out.target_bearing_deg);
    out.course_reliable = in.ground_speed_mps >= cfg.min_course_speed_mps;
    out.target_reachable = out.distance_to_target_m <= in.altitude_agl_m * cfg.nominal_glide_ratio;

    // Within target acceptance radius - reduce steering
    if (out.distance_to_target_m < cfg.target_accept_radius_m) {
        out.guidance_active = false;
        out.steering_command = 0.0f;
        return out;
    }

    // Select gains based on mode
    float kp = cfg.kp;
    float max_cmd = cfg.max_steering;
    float deadband = cfg.deadband_deg;

    if (in.mode == GuidanceMode::FINAL_APPROACH) {
        kp = cfg.final_approach_kp;
        max_cmd = cfg.final_approach_max_command;
        deadband *= cfg.final_deadband_multiplier;
    } else if (in.mode == GuidanceMode::FLARE) {
        // Flare is symmetric brake, not differential steering
        kp = 0.0f;
        max_cmd = cfg.flare_max_brake;
    }

    if (!out.course_reliable) { max_cmd = std::min(max_cmd, cfg.reduced_authority); deadband *= cfg.low_speed_deadband_multiplier; }
    if (in.imu_valid && in.angular_rate_dps > cfg.unstable_gyro_rate_dps) max_cmd = 0.0f;
    else if (!in.imu_valid) max_cmd = std::min(max_cmd, cfg.reduced_authority);
    if (!in.baro_valid) max_cmd = std::min(max_cmd, cfg.reduced_authority);
    if (!out.target_reachable) max_cmd = std::min(max_cmd, cfg.reduced_authority);

    // Proportional steering
    float raw_command = std::fabs(out.heading_error_deg) < deadband
                      ? 0.0f : kp * out.heading_error_deg;

    // Apply deadband and clamp
    float cmd = applyDeadbandAndClamp(raw_command, 0.0f, max_cmd);
    out.saturated = std::fabs(raw_command) > max_cmd && max_cmd > 0.0f;

    // Rate limit
    cmd = rate_limiter.update(cmd, in.dt_s);

    // Reversal guard
    if (!reversal_guard.allow(cmd, in.now_ms)) {
        // First unload the current brake; the opposite turn is allowed later.
        cmd = rate_limiter.update(0.0f, in.dt_s);
    }

    out.steering_command = cmd;
    out.guidance_active = true;
    return out;
}

} // namespace logic
