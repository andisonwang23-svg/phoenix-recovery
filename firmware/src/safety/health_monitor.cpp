// ============================================================================
// PHOENIX RECOVERY — Health Monitor Implementation.
// ============================================================================
#include "health_monitor.h"

namespace phoenix {
namespace safety {

HealthMonitor::HealthMonitor() {
    start_time_ms_ = millis();
}

bool HealthMonitor::begin(const HealthConfig& config) {
    config_ = config;
    report_ = HealthReport{}; // zero-initialize
    report_.timestamp_ms = millis();
    report_.uptime_ms = millis() - start_time_ms_;
    initialized_ = true;
    return true;
}

void HealthMonitor::update(const VehicleState& state) {
    if (!initialized_) return;

    report_.timestamp_ms = state.timestamp_ms;
    report_.uptime_ms = millis() - start_time_ms_;

    updateIMU(state);
    updateBarometer(state);
    updateGPS(state);
    updateServos(state);
    updateLoRa(state);
    updateWiFi(state);
    updateMemory();
    updateCPU();

    computeOverallHealth();
}

bool HealthMonitor::isFlightHealthy() const {
    return report_.overall == SystemHealth::NOMINAL ||
           report_.overall == SystemHealth::WARNING;
}

bool HealthMonitor::isSubsystemHealthy(const char* subsystem) const {
    if (strcmp(subsystem, "imu") == 0) return report_.imu.health == SubsystemHealth::HEALTHY;
    if (strcmp(subsystem, "barometer") == 0) return report_.barometer.health == SubsystemHealth::HEALTHY;
    if (strcmp(subsystem, "gps") == 0) return report_.gps.health == SubsystemHealth::HEALTHY;
    if (strcmp(subsystem, "servo") == 0) return report_.servos.health == SubsystemHealth::HEALTHY;
    if (strcmp(subsystem, "lora") == 0) return report_.lora.health == SubsystemHealth::HEALTHY;
    if (strcmp(subsystem, "wifi") == 0) return report_.wifi.health == SubsystemHealth::HEALTHY;
    return false;
}

void HealthMonitor::recordError(const char* subsystem, const char* error_msg) {
    if (!initialized_) return;

    // Map subsystem name to report field
    if (strcmp(subsystem, "imu") == 0) {
        report_.imu.error_count++;
        strncpy(report_.imu.last_error, error_msg, sizeof(report_.imu.last_error) - 1);
    } else if (strcmp(subsystem, "barometer") == 0) {
        report_.barometer.error_count++;
        strncpy(report_.barometer.last_error, error_msg, sizeof(report_.barometer.last_error) - 1);
    } else if (strcmp(subsystem, "gps") == 0) {
        report_.gps.error_count++;
        strncpy(report_.gps.last_error, error_msg, sizeof(report_.gps.last_error) - 1);
    } else if (strcmp(subsystem, "servo") == 0) {
        report_.servos.error_count++;
        strncpy(report_.servos.last_error, error_msg, sizeof(report_.servos.last_error) - 1);
    } else if (strcmp(subsystem, "lora") == 0) {
        report_.lora.error_count++;
        strncpy(report_.lora.last_error, error_msg, sizeof(report_.lora.last_error) - 1);
    } else if (strcmp(subsystem, "wifi") == 0) {
        report_.wifi.error_count++;
        strncpy(report_.wifi.last_error, error_msg, sizeof(report_.wifi.last_error) - 1);
    } else if (strcmp(subsystem, "memory") == 0) {
        report_.memory.error_count++;
        strncpy(report_.memory.last_error, error_msg, sizeof(report_.memory.last_error) - 1);
    } else if (strcmp(subsystem, "cpu") == 0) {
        report_.cpu.error_count++;
        strncpy(report_.cpu.last_error, error_msg, sizeof(report_.cpu.last_error) - 1);
    }
}

void HealthMonitor::recordWarning(const char* subsystem, const char* warn_msg) {
    if (!initialized_) return;

    if (strcmp(subsystem, "imu") == 0) {
        report_.imu.warning_count++;
    } else if (strcmp(subsystem, "barometer") == 0) {
        report_.barometer.warning_count++;
    } else if (strcmp(subsystem, "gps") == 0) {
        report_.gps.warning_count++;
    } else if (strcmp(subsystem, "servo") == 0) {
        report_.servos.warning_count++;
    } else if (strcmp(subsystem, "lora") == 0) {
        report_.lora.warning_count++;
    } else if (strcmp(subsystem, "wifi") == 0) {
        report_.wifi.warning_count++;
    } else if (strcmp(subsystem, "memory") == 0) {
        report_.memory.warning_count++;
    } else if (strcmp(subsystem, "cpu") == 0) {
        report_.cpu.warning_count++;
    }
}

void HealthMonitor::reset() {
    report_ = HealthReport{};
    start_time_ms_ = millis();
}

float HealthMonitor::getHealthScore(const char* subsystem) const {
    if (strcmp(subsystem, "imu") == 0) return report_.imu.health_score;
    if (strcmp(subsystem, "barometer") == 0) return report_.barometer.health_score;
    if (strcmp(subsystem, "gps") == 0) return report_.gps.health_score;
    if (strcmp(subsystem, "servo") == 0) return report_.servos.health_score;
    if (strcmp(subsystem, "lora") == 0) return report_.lora.health_score;
    if (strcmp(subsystem, "wifi") == 0) return report_.wifi.health_score;
    if (strcmp(subsystem, "memory") == 0) return report_.memory.health_score;
    if (strcmp(subsystem, "cpu") == 0) return report_.cpu.health_score;
    return 0.0f;
}

void HealthMonitor::updateIMU(const VehicleState& state) {
    report_.imu.last_update_ms = state.timestamp_ms;
    report_.imu_stale = isStale(state.imu_last_update_ms, config_.imu_max_age_ms);

    if (state.imu_valid) {
        report_.imu.health = SubsystemHealth::HEALTHY;
    } else if (report_.imu_stale) {
        report_.imu.health = SubsystemHealth::FAILED;
    } else {
        report_.imu.health = SubsystemHealth::DEGRADED;
    }

    // Estimate error rate (simplified - would need time window in real impl)
    float error_rate = report_.imu.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.imu, error_rate, report_.imu_stale);
}

void HealthMonitor::updateBarometer(const VehicleState& state) {
    report_.barometer.last_update_ms = state.timestamp_ms;
    report_.baro_stale = isStale(state.baro_last_update_ms, config_.baro_max_age_ms);

    if (state.barometer_valid) {
        report_.barometer.health = SubsystemHealth::HEALTHY;
    } else if (report_.baro_stale) {
        report_.barometer.health = SubsystemHealth::FAILED;
    } else {
        report_.barometer.health = SubsystemHealth::DEGRADED;
    }

    float error_rate = report_.barometer.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.barometer, error_rate, report_.baro_stale);
}

void HealthMonitor::updateGPS(const VehicleState& state) {
    report_.gps.last_update_ms = state.timestamp_ms;
    report_.gps_stale = isStale(state.gps_last_update_ms, config_.gps_max_age_ms);

    if (state.gps_valid && state.satellite_count >= 4) {
        report_.gps.health = SubsystemHealth::HEALTHY;
    } else if (report_.gps_stale || state.satellite_count < 3) {
        report_.gps.health = SubsystemHealth::FAILED;
    } else {
        report_.gps.health = SubsystemHealth::DEGRADED;
    }

    float error_rate = report_.gps.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.gps, error_rate, report_.gps_stale);
}

void HealthMonitor::updateServos(const VehicleState& state) {
    report_.servos.last_update_ms = state.timestamp_ms;
    report_.servo_stale = isStale(state.servo_last_update_ms, config_.servo_max_age_ms);

    // Check if servos are at neutral (safe) or commanded
    bool servos_responsive = (state.left_servo_us > 500 && state.left_servo_us < 2500) &&
                             (state.right_servo_us > 500 && state.right_servo_us < 2500);

    if (servos_responsive && !report_.servo_stale) {
        report_.servos.health = SubsystemHealth::HEALTHY;
    } else if (report_.servo_stale) {
        report_.servos.health = SubsystemHealth::FAILED;
    } else {
        report_.servos.health = SubsystemHealth::DEGRADED;
    }

    float error_rate = report_.servos.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.servos, error_rate, report_.servo_stale);
}

void HealthMonitor::updateLoRa(const VehicleState& state) {
    report_.lora.last_update_ms = state.timestamp_ms;

    if (state.lora_rssi > -120 && state.lora_rssi < 0) {
        report_.lora.health = SubsystemHealth::HEALTHY;
        report_.lora_failed = false;
    } else {
        report_.lora.health = SubsystemHealth::DEGRADED;
        report_.lora_failed = true;
    }

    float error_rate = report_.lora.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.lora, error_rate, false);
}

void HealthMonitor::updateWiFi(const VehicleState& state) {
    report_.wifi.last_update_ms = state.timestamp_ms;

    // WiFi health based on client count (for AP mode)
    if (state.wifi_client_count > 0 || state.gps_ap_active) {
        report_.wifi.health = SubsystemHealth::HEALTHY;
        report_.wifi_failed = false;
    } else {
        report_.wifi.health = SubsystemHealth::DEGRADED;
    }

    float error_rate = report_.wifi.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.wifi, error_rate, false);
}

void HealthMonitor::updateMemory() {
    report_.memory.last_update_ms = millis();

    size_t free_heap = ESP.getFreeHeap();
    report_.low_memory = free_heap < config_.min_free_heap_bytes;

    if (report_.low_memory) {
        report_.memory.health = SubsystemHealth::FAILED;
    } else if (free_heap < config_.min_free_heap_bytes * 2) {
        report_.memory.health = SubsystemHealth::DEGRADED;
    } else {
        report_.memory.health = SubsystemHealth::HEALTHY;
    }

    float error_rate = report_.memory.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.memory, error_rate, report_.low_memory);
}

void HealthMonitor::updateCPU() {
    report_.cpu.last_update_ms = millis();

    // Approximate CPU load from loop timing (would need more sophisticated measurement)
    // For now, just check if loop rate is reasonable
    float loop_rate = 1000.0f / (report_.timestamp_ms - report_.uptime_ms + 1); // placeholder

    if (loop_rate < 10.0f) { // Below 10 Hz is concerning
        report_.cpu.health = SubsystemHealth::DEGRADED;
        report_.high_cpu = true;
    } else {
        report_.cpu.health = SubsystemHealth::HEALTHY;
        report_.high_cpu = false;
    }

    float error_rate = report_.cpu.error_count / (report_.uptime_ms / 1000.0f + 1.0f);
    updateHealthScore(report_.cpu, error_rate, report_.high_cpu);
}

bool HealthMonitor::isStale(uint32_t last_update_ms, uint32_t max_age_ms) const {
    if (last_update_ms == 0) return true; // Never updated
    uint32_t now = millis();
    return (now - last_update_ms) > max_age_ms;
}

void HealthMonitor::updateHealthScore(SubsystemReport& report, float error_rate, bool stale) {
    float score = 1.0f;

    // Reduce score based on error rate
    if (error_rate > 0.1f) {
        score *= 0.5f;
    } else if (error_rate > 0.01f) {
        score *= 0.8f;
    }

    // Reduce score if stale
    if (stale) {
        score *= 0.3f;
    }

    report.health_score = constrain(score, 0.0f, 1.0f);

    // Update health enum based on score
    if (report.health_score >= 0.8f) {
        report.health = SubsystemHealth::HEALTHY;
    } else if (report.health_score >= 0.4f) {
        report.health = SubsystemHealth::DEGRADED;
    } else {
        report.health = SubsystemHealth::FAILED;
    }
}

void HealthMonitor::computeOverallHealth() {
    int healthy_count = 0;
    int degraded_count = 0;
    int failed_count = 0;

    SubsystemReport* reports[] = {
        &report_.imu, &report_.barometer, &report_.gps, &report_.servos,
        &report_.lora, &report_.wifi, &report_.memory, &report_.cpu
    };

    for (auto* r : reports) {
        switch (r->health) {
            case SubsystemHealth::HEALTHY: healthy_count++; break;
            case SubsystemHealth::DEGRADED: degraded_count++; break;
            case SubsystemHealth::FAILED: failed_count++; break;
            default: break;
        }
    }

    // Critical subsystems for flight
    const bool sensor_fallback_available =
        (report_.gps.health != SubsystemHealth::FAILED && report_.imu.health != SubsystemHealth::FAILED) ||
        (report_.gps.health != SubsystemHealth::FAILED && report_.barometer.health != SubsystemHealth::FAILED);
    bool critical_failed = (report_.servos.health == SubsystemHealth::FAILED) ||
                           ((report_.imu.health == SubsystemHealth::FAILED ||
                             report_.barometer.health == SubsystemHealth::FAILED ||
                             report_.gps.health == SubsystemHealth::FAILED) &&
                            !sensor_fallback_available) ||
                           report_.low_memory;

    if (critical_failed) {
        report_.overall = SystemHealth::FAILSAFE;
    } else if (failed_count > 0) {
        report_.overall = SystemHealth::CRITICAL;
    } else if (degraded_count > 2) {
        report_.overall = SystemHealth::WARNING;
    } else {
        report_.overall = SystemHealth::NOMINAL;
    }
}

} // namespace safety
} // namespace phoenix
