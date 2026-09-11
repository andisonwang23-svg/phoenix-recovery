// ============================================================================
// PHOENIX RECOVERY — Health Monitor (Sensor & System Health Tracking).
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"
#include "vehicle_state.h"

namespace phoenix {
namespace safety {

// Health status for individual subsystems
enum class SubsystemHealth : uint8_t {
    HEALTHY = 0,
    DEGRADED = 1,
    FAILED = 2,
    UNKNOWN = 3
};

// Overall system health
enum class SystemHealth : uint8_t {
    UNKNOWN = 0,
    NOMINAL = 1,
    WARNING = 2,
    CRITICAL = 3,
    FAILSAFE = 4
};

// Health report for a single subsystem
struct SubsystemReport {
    SubsystemHealth health = SubsystemHealth::UNKNOWN;
    uint32_t last_update_ms = 0;
    uint32_t error_count = 0;
    uint32_t warning_count = 0;
    float health_score = 1.0f; // 0.0 = failed, 1.0 = perfect
    char last_error[64] = {0};
};

// Complete health report
struct HealthReport {
    SystemHealth overall = SystemHealth::UNKNOWN;
    uint32_t timestamp_ms = 0;
    uint32_t uptime_ms = 0;

    SubsystemReport imu;
    SubsystemReport barometer;
    SubsystemReport gps;
    SubsystemReport servos;
    SubsystemReport lora;
    SubsystemReport wifi;
    SubsystemReport memory;
    SubsystemReport cpu;

    // Thresholds exceeded
    bool imu_stale = false;
    bool baro_stale = false;
    bool gps_stale = false;
    bool servo_stale = false;
    bool low_memory = false;
    bool high_cpu = false;
    bool lora_failed = false;
    bool wifi_failed = false;
};

// Configuration for health thresholds
struct HealthConfig {
    // Staleness thresholds (ms)
    uint32_t imu_max_age_ms = 100;
    uint32_t baro_max_age_ms = 200;
    uint32_t gps_max_age_ms = 2000;
    uint32_t servo_max_age_ms = 50;

    // Error rate thresholds (errors per second)
    float max_imu_error_rate = 10.0f;
    float max_baro_error_rate = 5.0f;
    float max_gps_error_rate = 2.0f;
    float max_servo_error_rate = 5.0f;

    // System thresholds
    uint32_t min_free_heap_bytes = 10 * 1024; // 10 KB
    float max_cpu_load = 0.90f; // 90%
};

class HealthMonitor {
public:
    HealthMonitor();
    ~HealthMonitor() = default;

    // Initialize with config
    bool begin(const HealthConfig& config = HealthConfig());

    // Update health from VehicleState (call every loop)
    void update(const VehicleState& state);

    // Get current health report
    const HealthReport& getReport() const { return report_; }

    // Check if system is healthy enough for guided flight
    bool isFlightHealthy() const;

    // Check if specific subsystem is healthy by name
    bool isSubsystemHealthy(const char* subsystem) const;

    // Record an error for a subsystem
    void recordError(const char* subsystem, const char* error_msg);

    // Record a warning for a subsystem
    void recordWarning(const char* subsystem, const char* warn_msg);

    // Reset health stats (e.g., after configuration change)
    void reset();

    // Get health score for a subsystem (0.0 - 1.0)
    float getHealthScore(const char* subsystem) const;

private:
    HealthConfig config_;
    HealthReport report_;
    uint32_t start_time_ms_ = 0;
    bool initialized_ = false;

    // Update individual subsystem health
    void updateIMU(const VehicleState& state);
    void updateBarometer(const VehicleState& state);
    void updateGPS(const VehicleState& state);
    void updateServos(const VehicleState& state);
    void updateLoRa(const VehicleState& state);
    void updateWiFi(const VehicleState& state);
    void updateMemory();
    void updateCPU();

    // Compute overall health from subsystems
    void computeOverallHealth();

    // Check staleness
    bool isStale(uint32_t last_update_ms, uint32_t max_age_ms) const;

    // Update health score based on error rate and staleness
    void updateHealthScore(SubsystemReport& report, float error_rate, bool stale);
};

} // namespace safety
} // namespace phoenix