// ============================================================================
// PHOENIX RECOVERY — Guidance Controller.
// Selects guidance mode, computes steering, interfaces with servo controller.
// ============================================================================
#pragma once

#include "config.h"
#include "vehicle_state.h"
#include "control/servo_controller.h"
#include "logic/guidance_logic.h"
#include "logic/state_machine.h"

namespace control {

class GuidanceController {
public:
    GuidanceController();
    ~GuidanceController() = default;

    // Initialize guidance
    bool begin(control::ServoController* servos);
    bool begin(const logic::GuidanceConfig& config);

    // Update guidance - call at guidance rate
    void update(phoenix::VehicleState& state);
    void update(phoenix::VehicleState& state, float dt_s);

    // Set guidance mode (called by state machine)
    void setMode(logic::GuidanceMode mode) { current_mode_ = mode; }

    // Get current mode
    logic::GuidanceMode getMode() const { return current_mode_; }

    // Check if guidance is active
    bool isGuidanceActive() const { return guidance_active_; }

private:
    control::ServoController* servos_ = nullptr;
    logic::GuidanceConfig config_;
    logic::GuidanceMode current_mode_ = logic::GuidanceMode::MODE_DISABLED;
    logic::RateLimiter rate_limiter_;
    logic::ReversalGuard reversal_guard_;
    bool guidance_active_ = false;
    bool initialized_ = false;
};

} // namespace control
