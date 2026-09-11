// ============================================================================
// PHOENIX RECOVERY — Hardware Watchdog Timer.
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"
#include "vehicle_state.h"

namespace phoenix {
namespace safety {

// Watchdog configuration
struct WatchdogConfig {
    uint32_t timeout_ms = 5000;      // 5 second timeout
    bool enabled = false;            // Disabled by default for safety
    bool interrupt_mode = false;     // true = interrupt then reset, false = hard reset
    bool panic_reboot = true;        // Use panic() for immediate reset
};

// Watchdog status
struct WatchdogStatus {
    bool enabled = false;
    uint32_t timeout_ms = 0;
    uint32_t last_feed_ms = 0;
    uint32_t feed_count = 0;
    uint32_t reset_count = 0;
    bool needs_reset = false;
};

class HardwareWatchdog {
public:
    HardwareWatchdog();
    ~HardwareWatchdog() = default;

    // Initialize watchdog with config
    bool begin(const WatchdogConfig& config = WatchdogConfig());

    // Enable/disable watchdog
    void enable(bool enabled);

    // Feed the watchdog (call in main loop)
    void feed();

    // Get watchdog status
    const WatchdogStatus& getStatus() const { return status_; }

    // Check if watchdog is enabled
    bool isEnabled() const { return status_.enabled; }

    // Trigger immediate reboot via watchdog
    void triggerReboot();

    // Get time since last feed (ms)
    uint32_t getTimeSinceLastFeed() const;

    // Check if watchdog has timed out
    bool hasTimedOut() const;

private:
    WatchdogConfig config_;
    WatchdogStatus status_;
    bool initialized_ = false;
    uint32_t start_time_ms_ = 0;
};

} // namespace safety
} // namespace phoenix