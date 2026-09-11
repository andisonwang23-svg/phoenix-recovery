// ============================================================================
// PHOENIX RECOVERY — Detection logic (launch, apogee, landing).
// Pure C++, host-testable.
// ============================================================================
#pragma once

#include <cstdint>

namespace logic {

// ---------------------------------------------------------------------------
// Launch detection context
// ---------------------------------------------------------------------------
struct LaunchDetectionContext {
    float vertical_accel_mps2 = 0.0f;
    float vertical_speed_mps = 0.0f;
    float altitude_agl_m = 0.0f;
    bool baro_valid = false;
    bool imu_valid = false;
    uint32_t now_ms = 0;
    uint32_t confirm_ms = 400;
    float accel_threshold = 8.0f;
    float vs_threshold = 5.0f;
    float alt_delta_threshold = 10.0f;
    uint32_t confirm_start_ms = 0;
};

// ---------------------------------------------------------------------------
// Apogee detection context
// ---------------------------------------------------------------------------
struct ApogeeDetectionContext {
    float vertical_speed_mps = 0.0f;
    float altitude_agl_m = 0.0f;
    float prev_altitude_m = 0.0f;
    bool baro_valid = false;
    uint32_t now_ms = 0;
    uint32_t confirm_ms = 700;
    float vs_threshold = 1.5f;
    float alt_flat_threshold = 2.0f;
    float min_launch_altitude = 8.0f;
    uint32_t confirm_start_ms = 0;
};

// ---------------------------------------------------------------------------
// Landing detection context
// ---------------------------------------------------------------------------
struct LandingDetectionContext {
    float vertical_speed_mps = 0.0f;
    float altitude_agl_m = 0.0f;
    float ground_speed_mps = 0.0f;
    float accel_jitter_g = 0.0f;
    bool gps_valid = false;
    bool baro_valid = false;
    uint32_t now_ms = 0;
    uint32_t confirm_ms = 4000;
    float alt_change_threshold = 1.0f;
    float vs_threshold = 1.0f;
    float gnd_speed_threshold = 2.0f;
    float accel_jitter_threshold = 0.15f;
    uint32_t confirm_start_ms = 0;
};

// ---------------------------------------------------------------------------
// Launch detection: returns true if launch confirmed
// Requires 2 of 3 signals persisting for confirm_ms
// ---------------------------------------------------------------------------
inline bool detectLaunch(LaunchDetectionContext& ctx) {
    if (!ctx.baro_valid || !ctx.imu_valid) return false;

    int signals = 0;
    if (ctx.vertical_accel_mps2 > ctx.accel_threshold) signals++;
    if (ctx.vertical_speed_mps > ctx.vs_threshold) signals++;
    if (ctx.altitude_agl_m > ctx.alt_delta_threshold) signals++;

    if (signals >= 2) {
        if (ctx.confirm_start_ms == 0) {
            ctx.confirm_start_ms = ctx.now_ms;
            return false;  // not confirmed yet
        }
        return (ctx.now_ms - ctx.confirm_start_ms) >= ctx.confirm_ms;
    } else {
        ctx.confirm_start_ms = 0;
        return false;
    }
}

// ---------------------------------------------------------------------------
// Apogee detection: returns true if apogee confirmed
// VS crosses through zero and altitude stops increasing
// ---------------------------------------------------------------------------
inline bool detectApogee(ApogeeDetectionContext& ctx) {
    if (!ctx.baro_valid) return false;

    bool vs_near_zero = (std::abs(ctx.vertical_speed_mps) < ctx.vs_threshold);
    bool alt_flat = (ctx.altitude_agl_m - ctx.prev_altitude_m) < ctx.alt_flat_threshold;
    bool above_min = ctx.altitude_agl_m > ctx.min_launch_altitude;

    if (vs_near_zero && alt_flat && above_min) {
        if (ctx.confirm_start_ms == 0) {
            ctx.confirm_start_ms = ctx.now_ms;
            return false;
        }
        return (ctx.now_ms - ctx.confirm_start_ms) >= ctx.confirm_ms;
    } else {
        ctx.confirm_start_ms = 0;
        return false;
    }
}

// ---------------------------------------------------------------------------
// Landing detection: returns true if landing confirmed
// Multiple indicators sustained for confirm_ms
// ---------------------------------------------------------------------------
inline bool detectLanding(LandingDetectionContext& ctx) {
    if (!ctx.baro_valid) return false;

    bool alt_stable = (std::abs(ctx.vertical_speed_mps) < ctx.vs_threshold);
    bool gnd_slow = (!ctx.gps_valid || ctx.ground_speed_mps < ctx.gnd_speed_threshold);
    // accel_jitter would come from IMU history
    bool low_jitter = ctx.accel_jitter_g < ctx.accel_jitter_threshold;

    if (alt_stable && gnd_slow && low_jitter) {
        if (ctx.confirm_start_ms == 0) {
            ctx.confirm_start_ms = ctx.now_ms;
            return false;
        }
        return (ctx.now_ms - ctx.confirm_start_ms) >= ctx.confirm_ms;
    } else {
        ctx.confirm_start_ms = 0;
        return false;
    }
}

// ---------------------------------------------------------------------------
// Stabilization detection for parafoil
// ---------------------------------------------------------------------------
struct StabilizationContext {
    float angular_rate_dps = 0.0f;
    float vs_jitter_mps = 0.0f;
    uint32_t now_ms = 0;
    uint32_t min_stable_ms = 3000;
    uint32_t max_wait_ms = 15000;
    float rate_limit_dps = 15.0f;
    float vs_jitter_limit_mps = 0.5f;
    uint32_t stable_start_ms = 0;
};

// Returns: 0 = not stable yet, 1 = stable & ready, -1 = timeout (abandon)
inline int detectStabilization(StabilizationContext& ctx) {
    bool stable = (ctx.angular_rate_dps < ctx.rate_limit_dps &&
                   ctx.vs_jitter_mps < ctx.vs_jitter_limit_mps);

    if (stable) {
        if (ctx.stable_start_ms == 0) {
            ctx.stable_start_ms = ctx.now_ms;
            return 0;
        }
        if ((ctx.now_ms - ctx.stable_start_ms) >= ctx.min_stable_ms) {
            return 1;  // stable and minimum time elapsed
        }
        return 0;  // stable but not long enough
    } else {
        ctx.stable_start_ms = 0;
        // Check timeout
        // (caller must track total time in stabilization state)
        return 0;
    }
}

} // namespace logic