// ============================================================================
// PHOENIX RECOVERY — Failsafe Controller (Deterministic Fault Response).
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"
#include "vehicle_state.h"
#include "health_monitor.h"

namespace phoenix {
namespace safety {

// Failsafe trigger reasons
enum class FailsafeReason : uint8_t {
    NONE = 0,
    IMU_FAILURE = 1,
    BAROMETER_FAILURE = 2,
    GPS_FAILURE = 3,
    SERVO_FAILURE = 4,
    LOW_MEMORY = 5,
    HIGH_CPU_LOAD = 6,
    LORA_FAILURE = 7,
    WATCHDOG_TIMEOUT = 8,
    MANUAL_TRIGGER = 9,
    LAUNCH_DETECTION_TIMEOUT = 10,
    APOGEE_DETECTION_TIMEOUT = 11,
    STABILIZATION_TIMEOUT = 12,
    GUIDANCE_LOST_TARGET = 13,
    FLIGHT_TIME_EXCEEDED = 14,
    POWER_LOW = 15
};

// Failsafe action to take
enum class FailsafeAction : uint8_t {
    NONE = 0,
    NEUTRAL_SERVOS = 1,        // Command servos to neutral
    FLARE_AND_LAND = 2,        // Flare (max brake) and land
    SPIRAL_DESCENT = 3,        // Enter spiral descent pattern
    EMERGENCY_CUTOFF = 4,      // Cut power to servos (if hardware supports)
    REBOOT_SYSTEM = 5,         // Reboot ESP32
    HOLD_LAST_COMMAND = 6      // Hold last known good command
};

// Failsafe state
struct FailsafeState {
    bool active = false;
    FailsafeReason reason = FailsafeReason::NONE;
    FailsafeAction action = FailsafeAction::NONE;
    uint32_t triggered_ms = 0;
    uint32_t acknowledged_ms = 0;
    bool requires_reboot = false;
    char detail[128] = {0};
};

// Failsafe configuration
struct FailsafeConfig {
    // Timeouts (ms)
    uint32_t launch_detection_timeout_ms = 30000;     // 30s max wait for launch
    uint32_t apogee_detection_timeout_ms = 120000;    // 2min max wait for apogee
    uint32_t stabilization_timeout_ms = 10000;        // 10s max stabilization
    uint32_t max_flight_time_ms = 600000;             // 10min max flight
    uint32_t servo_timeout_ms = 500;                  // 500ms servo update timeout

    // Thresholds
    float min_battery_voltage = 3.3f;                 // V
    uint32_t min_free_heap = 8192;                    // 8KB
    float max_cpu_load = 0.95f;                       // 95%

    // Behavior
    bool auto_recover = false;                        // Auto-clear failsafe if health restores
    uint32_t recovery_delay_ms = 5000;                // Wait before auto-recover
};

class FailsafeController {
public:
    FailsafeController();
    ~FailsafeController() = default;

    // Initialize with config
    bool begin(const FailsafeConfig& config = FailsafeConfig());

    // Update failsafe logic (call every loop)
    void update(const VehicleState& state, const HealthReport& health);

    // Check if failsafe is active
    bool isActive() const { return state_.active; }

    // Get current failsafe state
    const FailsafeState& getState() const { return state_; }

    // Get commanded failsafe action
    FailsafeAction getAction() const { return state_.action; }

    // Manually trigger failsafe
    void trigger(FailsafeReason reason, const char* detail = "");

    // Acknowledge failsafe (clear it if conditions allow)
    bool acknowledge();

    // Force clear failsafe (use with caution)
    void forceClear();

    // Check if specific condition should trigger failsafe
    bool checkLaunchTimeout(const VehicleState& state) const;
    bool checkApogeeTimeout(const VehicleState& state) const;
    bool checkStabilizationTimeout(const VehicleState& state) const;
    bool checkFlightTimeExceeded(const VehicleState& state) const;
    bool checkServoTimeout(const VehicleState& state) const;
    bool checkHealthFailures(const HealthReport& health) const;

    // Apply failsafe action to vehicle state
    void applyAction(VehicleState& state);

    // Get reason string
    static const char* reasonToString(FailsafeReason reason);
    static const char* actionToString(FailsafeAction action);

private:
    FailsafeConfig config_;
    FailsafeState state_;
    bool initialized_ = false;
    uint32_t last_health_check_ms_ = 0;

    // Determine action based on reason
    FailsafeAction determineAction(FailsafeReason reason) const;

    // Check if we can auto-recover
    bool canAutoRecover(const HealthReport& health) const;
};

} // namespace safety
} // namespace phoenix