// ============================================================================
// PHOENIX RECOVERY — Hardware Watchdog Implementation.
// ============================================================================
#include "watchdog.h"

namespace phoenix {
namespace safety {

HardwareWatchdog::HardwareWatchdog() = default;

bool HardwareWatchdog::begin(const WatchdogConfig& config) {
    config_ = config;
    status_ = WatchdogStatus{};
    status_.timeout_ms = config.timeout_ms;
    start_time_ms_ = millis();

    // ESP32-S3 has a hardware watchdog
    // For now, we'll track it in software and use ESP.restart() as fallback
    // Real hardware watchdog would use esp_task_wdt_* APIs

    if (config_.enabled) {
        enable(true);
    }

    initialized_ = true;
    return true;
}

void HardwareWatchdog::enable(bool enabled) {
    if (!initialized_) return;

    status_.enabled = enabled;
    status_.last_feed_ms = millis();

    if (enabled) {
        // In real implementation:
        // esp_task_wdt_init(config_.timeout_ms / 1000, true);
        // esp_task_wdt_add(NULL);
        // For now, we'll use software-based watchdog tracking
    }
}

void HardwareWatchdog::feed() {
    if (!initialized_ || !status_.enabled) return;

    status_.last_feed_ms = millis();
    status_.feed_count++;

    // In real implementation:
    // esp_task_wdt_reset();
}

uint32_t HardwareWatchdog::getTimeSinceLastFeed() const {
    if (!initialized_) return 0;
    return millis() - status_.last_feed_ms;
}

bool HardwareWatchdog::hasTimedOut() const {
    if (!initialized_ || !status_.enabled) return false;
    return getTimeSinceLastFeed() >= status_.timeout_ms;
}

void HardwareWatchdog::triggerReboot() {
    // Immediate reboot
    ESP.restart();
}

} // namespace safety
} // namespace phoenix