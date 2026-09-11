// ============================================================================
// PHOENIX RECOVERY — Failsafe Controller Implementation.
// ============================================================================
#include "failsafe.h"
#include "../logic/state_machine.h"

namespace phoenix {
namespace safety {

FailsafeController::FailsafeController() = default;

bool FailsafeController::begin(const FailsafeConfig& config) {
    config_ = config;
    state_ = FailsafeState{};
    initialized_ = true;
    return true;
}

void FailsafeController::update(const VehicleState& state, const HealthReport& health) {
    if (!initialized_) return;

    // If already in failsafe, check for auto-recovery
    if (state_.active) {
        if (config_.auto_recover && canAutoRecover(health)) {
            uint32_t now = millis();
            if (now - state_.triggered_ms >= config_.recovery_delay_ms) {
                acknowledge();
            }
        }
        return;
    }

    // Check all failsafe conditions
    if (checkHealthFailures(health)) return;
    if (checkLaunchTimeout(state)) return;
    if (checkApogeeTimeout(state)) return;
    if (checkStabilizationTimeout(state)) return;
    if (checkFlightTimeExceeded(state)) return;
    if (checkServoTimeout(state)) return;
}

bool FailsafeController::checkLaunchTimeout(const VehicleState& state) const {
    // Only check if we're in pre-launch states
    if (state.flight_state != logic::FlightState::PRE_LAUNCH &&
        state.flight_state != logic::FlightState::ARMED) {
        return false;
    }

    uint32_t elapsed = state.timestamp_ms - state.armed_ms;
    if (elapsed >= config_.launch_detection_timeout_ms) {
        return true;
    }
    return false;
}

bool FailsafeController::checkApogeeTimeout(const VehicleState& state) const {
    // Check if we've been in ascent too long without detecting apogee
    if (state.flight_state != logic::FlightState::ASCENT &&
        state.flight_state != logic::FlightState::APOGEE_DETECT) {
        return false;
    }

    uint32_t elapsed = state.timestamp_ms - state.launch_ms;
    if (elapsed >= config_.apogee_detection_timeout_ms) {
        return true;
    }
    return false;
}

bool FailsafeController::checkStabilizationTimeout(const VehicleState& state) const {
    if (state.flight_state != logic::FlightState::PARAFOIL_STABILIZATION) {
        return false;
    }

    uint32_t elapsed = state.timestamp_ms - state.stabilization_start_ms;
    if (elapsed >= config_.stabilization_timeout_ms) {
        return true;
    }
    return false;
}

bool FailsafeController::checkFlightTimeExceeded(const VehicleState& state) const {
    if (state.flight_state == logic::FlightState::PRE_LAUNCH ||
        state.flight_state == logic::FlightState::ARMED) {
        return false;
    }

    uint32_t elapsed = state.timestamp_ms - state.launch_ms;
    if (elapsed >= config_.max_flight_time_ms) {
        return true;
    }
    return false;
}

bool FailsafeController::checkServoTimeout(const VehicleState& state) const {
    if (state.servo_last_update_ms == 0) return false;

    uint32_t elapsed = state.timestamp_ms - state.servo_last_update_ms;
    if (elapsed >= config_.servo_timeout_ms) {
        return true;
    }
    return false;
}

bool FailsafeController::checkHealthFailures(const HealthReport& health) const {
    // Check critical subsystem failures
    if (health.imu.health == SubsystemHealth::FAILED) return true;
    if (health.barometer.health == SubsystemHealth::FAILED) return true;
    if (health.gps.health == SubsystemHealth::FAILED) return true;
    if (health.servos.health == SubsystemHealth::FAILED) return true;
    if (health.low_memory) return true;
    if (health.high_cpu) return true;

    return false;
}

void FailsafeController::trigger(FailsafeReason reason, const char* detail) {
    if (state_.active) return; // Already in failsafe

    state_.active = true;
    state_.reason = reason;
    state_.action = determineAction(reason);
    state_.triggered_ms = millis();
    state_.requires_reboot = (reason == FailsafeReason::WATCHDOG_TIMEOUT ||
                              reason == FailsafeReason::LOW_MEMORY ||
                              reason == FailsafeReason::HIGH_CPU_LOAD);

    if (detail && strlen(detail) > 0) {
        strncpy(state_.detail, detail, sizeof(state_.detail) - 1);
    }
}

bool FailsafeController::acknowledge() {
    if (!state_.active) return true;

    // Only acknowledge if we can recover
    // In real implementation, would check health again
    state_.active = false;
    state_.acknowledged_ms = millis();
    state_.reason = FailsafeReason::NONE;
    state_.action = FailsafeAction::NONE;
    state_.detail[0] = '\0';
    state_.requires_reboot = false;

    return true;
}

void FailsafeController::forceClear() {
    state_ = FailsafeState{};
}

void FailsafeController::applyAction(VehicleState& state) {
    if (!state_.active) return;

    switch (state_.action) {
        case FailsafeAction::NEUTRAL_SERVOS:
            state.left_servo_command = 0.0f;
            state.right_servo_command = 0.0f;
            state.guidance_mode = logic::GuidanceMode::MANUAL;
            break;

        case FailsafeAction::FLARE_AND_LAND:
            state.left_servo_command = -1.0f;  // Full brake
            state.right_servo_command = -1.0f; // Full brake
            state.guidance_mode = logic::GuidanceMode::MANUAL;
            break;

        case FailsafeAction::SPIRAL_DESCENT:
            state.left_servo_command = 0.5f;   // Gentle turn
            state.right_servo_command = -0.5f;
            state.guidance_mode = logic::GuidanceMode::MANUAL;
            break;

        case FailsafeAction::EMERGENCY_CUTOFF:
            // Hardware-specific - would cut power to servos
            state.left_servo_command = 0.0f;
            state.right_servo_command = 0.0f;
            break;

        case FailsafeAction::REBOOT_SYSTEM:
            // Trigger reboot
            ESP.restart();
            break;

        case FailsafeAction::HOLD_LAST_COMMAND:
            // Keep last commanded values
            break;

        default:
            break;
    }
}

FailsafeAction FailsafeController::determineAction(FailsafeReason reason) const {
    switch (reason) {
        case FailsafeReason::IMU_FAILURE:
        case FailsafeReason::BAROMETER_FAILURE:
        case FailsafeReason::GPS_FAILURE:
            return FailsafeAction::NEUTRAL_SERVOS;

        case FailsafeReason::SERVO_FAILURE:
            return FailsafeAction::EMERGENCY_CUTOFF;

        case FailsafeReason::LOW_MEMORY:
        case FailsafeReason::HIGH_CPU_LOAD:
            return FailsafeAction::REBOOT_SYSTEM;

        case FailsafeReason::WATCHDOG_TIMEOUT:
            return FailsafeAction::REBOOT_SYSTEM;

        case FailsafeReason::MANUAL_TRIGGER:
            return FailsafeAction::NEUTRAL_SERVOS;

        case FailsafeReason::LAUNCH_DETECTION_TIMEOUT:
        case FailsafeReason::APOGEE_DETECTION_TIMEOUT:
            return FailsafeAction::FLARE_AND_LAND;

        case FailsafeReason::STABILIZATION_TIMEOUT:
            return FailsafeAction::SPIRAL_DESCENT;

        case FailsafeReason::GUIDANCE_LOST_TARGET:
            return FailsafeAction::SPIRAL_DESCENT;

        case FailsafeReason::FLIGHT_TIME_EXCEEDED:
            return FailsafeAction::FLARE_AND_LAND;

        case FailsafeReason::POWER_LOW:
            return FailsafeAction::FLARE_AND_LAND;

        default:
            return FailsafeAction::NEUTRAL_SERVOS;
    }
}

bool FailsafeController::canAutoRecover(const HealthReport& health) const {
    // Can only auto-recover if all critical subsystems are healthy
    return health.imu.health == SubsystemHealth::HEALTHY &&
           health.barometer.health == SubsystemHealth::HEALTHY &&
           health.gps.health == SubsystemHealth::HEALTHY &&
           health.servos.health == SubsystemHealth::HEALTHY &&
           !health.low_memory &&
           !health.high_cpu;
}

const char* FailsafeController::reasonToString(FailsafeReason reason) {
    switch (reason) {
        case FailsafeReason::NONE: return "NONE";
        case FailsafeReason::IMU_FAILURE: return "IMU_FAILURE";
        case FailsafeReason::BAROMETER_FAILURE: return "BAROMETER_FAILURE";
        case FailsafeReason::GPS_FAILURE: return "GPS_FAILURE";
        case FailsafeReason::SERVO_FAILURE: return "SERVO_FAILURE";
        case FailsafeReason::LOW_MEMORY: return "LOW_MEMORY";
        case FailsafeReason::HIGH_CPU_LOAD: return "HIGH_CPU_LOAD";
        case FailsafeReason::LORA_FAILURE: return "LORA_FAILURE";
        case FailsafeReason::WATCHDOG_TIMEOUT: return "WATCHDOG_TIMEOUT";
        case FailsafeReason::MANUAL_TRIGGER: return "MANUAL_TRIGGER";
        case FailsafeReason::LAUNCH_DETECTION_TIMEOUT: return "LAUNCH_DETECTION_TIMEOUT";
        case FailsafeReason::APOGEE_DETECTION_TIMEOUT: return "APOGEE_DETECTION_TIMEOUT";
        case FailsafeReason::STABILIZATION_TIMEOUT: return "STABILIZATION_TIMEOUT";
        case FailsafeReason::GUIDANCE_LOST_TARGET: return "GUIDANCE_LOST_TARGET";
        case FailsafeReason::FLIGHT_TIME_EXCEEDED: return "FLIGHT_TIME_EXCEEDED";
        case FailsafeReason::POWER_LOW: return "POWER_LOW";
        default: return "UNKNOWN";
    }
}

const char* FailsafeController::actionToString(FailsafeAction action) {
    switch (action) {
        case FailsafeAction::NONE: return "NONE";
        case FailsafeAction::NEUTRAL_SERVOS: return "NEUTRAL_SERVOS";
        case FailsafeAction::FLARE_AND_LAND: return "FLARE_AND_LAND";
        case FailsafeAction::SPIRAL_DESCENT: return "SPIRAL_DESCENT";
        case FailsafeAction::EMERGENCY_CUTOFF: return "EMERGENCY_CUTOFF";
        case FailsafeAction::REBOOT_SYSTEM: return "REBOOT_SYSTEM";
        case FailsafeAction::HOLD_LAST_COMMAND: return "HOLD_LAST_COMMAND";
        default: return "UNKNOWN";
    }
}

} // namespace safety
} // namespace phoenix
