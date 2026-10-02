// ============================================================================
// PHOENIX RECOVERY — pure GNSS freshness/recovery decisions.
// Host-testable so a stale GPS fix can never remain accepted by accident.
// ============================================================================
#pragma once

#include <cstdint>

namespace logic {

inline bool gpsNmeaActive(uint32_t now, uint32_t last_nmea_ms,
                          uint32_t nmea_timeout_ms) {
    return last_nmea_ms != 0 && now - last_nmea_ms <= nmea_timeout_ms;
}

inline bool gpsFixUsable(uint32_t now, uint32_t last_nmea_ms,
                         uint32_t nmea_timeout_ms, bool location_valid,
                         uint32_t location_age_ms, uint32_t fix_timeout_ms) {
    return gpsNmeaActive(now, last_nmea_ms, nmea_timeout_ms) &&
           location_valid && location_age_ms < fix_timeout_ms;
}

inline bool gpsSilentRecoveryDue(uint32_t now, uint32_t startup_ms,
                                 uint32_t last_nmea_ms,
                                 uint32_t last_recovery_attempt_ms,
                                 uint32_t silence_grace_ms,
                                 uint32_t retry_interval_ms) {
    const uint32_t reference_ms = last_nmea_ms != 0 ? last_nmea_ms : startup_ms;
    return now - reference_ms >= silence_grace_ms &&
           now - last_recovery_attempt_ms >= retry_interval_ms;
}

} // namespace logic
