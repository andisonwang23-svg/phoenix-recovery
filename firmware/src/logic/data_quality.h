// ============================================================================
// PHOENIX RECOVERY — Data quality / plausibility filters.
// Pure C++, host-testable. Rejects NaN, impossible jumps, out-of-range.
// ============================================================================
#pragma once

#include <cmath>
#include <cstdint>

namespace logic {

// ---------------------------------------------------------------------------
// GPS data quality
// ---------------------------------------------------------------------------
struct GPSQualityConfig {
    float max_speed_mps = 120.0f;      // absurd ground speed
    float max_jump_m = 50.0f;          // position jump in one fix
    float min_hdop = 0.5f;
    float max_hdop = 20.0f;
    uint8_t min_satellites = 4;
};

struct GPSData {
    double lat = 0.0;
    double lon = 0.0;
    float speed_mps = 0.0f;
    float course_deg = 0.0f;
    float hdop = 0.0f;
    uint8_t satellites = 0;
    bool valid = false;
    uint32_t timestamp_ms = 0;
};

inline bool validateGPS(const GPSData& curr, const GPSData& prev,
                        const GPSQualityConfig& cfg) {
    if (!curr.valid) return false;
    if (std::isnan(curr.lat) || std::isnan(curr.lon)) return false;
    if (std::isnan(curr.speed_mps) || std::isnan(curr.course_deg)) return false;
    if (curr.satellites < cfg.min_satellites) return false;
    if (curr.hdop < cfg.min_hdop || curr.hdop > cfg.max_hdop) return false;
    if (curr.speed_mps > cfg.max_speed_mps) return false;

    if (prev.valid) {
        // Check position jump
        double dlat = (curr.lat - prev.lat) * 111320.0;  // rough m/deg
        double dlon = (curr.lon - prev.lon) * 111320.0 * std::cos(curr.lat * M_PI / 180.0);
        float jump_m = std::sqrt(dlat * dlat + dlon * dlon);
        if (jump_m > cfg.max_jump_m) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Barometer data quality
// ---------------------------------------------------------------------------
struct BaroQualityConfig {
    float max_alt_step_m = 30.0f;    // altitude jump in one sample
    float min_pressure_hpa = 300.0f;
    float max_pressure_hpa = 1200.0f;
};

struct BaroData {
    float pressure_hpa = 0.0f;
    float temperature_c = 0.0f;
    float altitude_m = 0.0f;
    bool valid = false;
    uint32_t timestamp_ms = 0;
};

inline bool validateBaro(const BaroData& curr, const BaroData& prev,
                         const BaroQualityConfig& cfg) {
    if (!curr.valid) return false;
    if (std::isnan(curr.pressure_hpa) || std::isnan(curr.altitude_m)) return false;
    if (curr.pressure_hpa < cfg.min_pressure_hpa ||
        curr.pressure_hpa > cfg.max_pressure_hpa) return false;

    if (prev.valid) {
        float alt_step = std::abs(curr.altitude_m - prev.altitude_m);
        if (alt_step > cfg.max_alt_step_m) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// IMU data quality
// ---------------------------------------------------------------------------
struct IMUQualityConfig {
    float quat_min_norm = 0.85f;
    float quat_max_norm = 1.15f;
    float max_accel_g = 50.0f;       // absurd acceleration
    float max_gyro_dps = 2000.0f;    // absurd rotation rate
};

struct IMUData {
    float quat_w = 1.0f, quat_x = 0.0f, quat_y = 0.0f, quat_z = 0.0f;
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;  // m/s^2
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;     // rad/s
    float linear_accel_x = 0.0f, linear_accel_y = 0.0f, linear_accel_z = 0.0f;
    bool valid = false;
    uint32_t timestamp_ms = 0;
};

inline bool validateIMU(const IMUData& data, const IMUQualityConfig& cfg) {
    if (!data.valid) return false;
    if (std::isnan(data.quat_w) || std::isnan(data.quat_x) ||
        std::isnan(data.quat_y) || std::isnan(data.quat_z)) return false;

    float norm2 = data.quat_w * data.quat_w + data.quat_x * data.quat_x +
                  data.quat_y * data.quat_y + data.quat_z * data.quat_z;
    if (norm2 < cfg.quat_min_norm * cfg.quat_min_norm ||
        norm2 > cfg.quat_max_norm * cfg.quat_max_norm) return false;

    // Check linear acceleration magnitude
    float accel_mag = std::sqrt(data.linear_accel_x * data.linear_accel_x +
                                data.linear_accel_y * data.linear_accel_y +
                                data.linear_accel_z * data.linear_accel_z);
    if (accel_mag > cfg.max_accel_g * 9.81f) return false;

    // Check gyro magnitude
    float gyro_mag = std::sqrt(data.gyro_x * data.gyro_x +
                               data.gyro_y * data.gyro_y +
                               data.gyro_z * data.gyro_z);
    if (gyro_mag > cfg.max_gyro_dps * M_PI / 180.0f) return false;

    return true;
}

// ---------------------------------------------------------------------------
// Servo command validation
// ---------------------------------------------------------------------------
inline bool validateServoCommand(float cmd, float min_cmd = -1.0f, float max_cmd = 1.0f) {
    if (std::isnan(cmd)) return false;
    return (cmd >= min_cmd && cmd <= max_cmd);
}

} // namespace logic