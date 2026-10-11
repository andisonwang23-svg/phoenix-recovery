// ============================================================================
// PHOENIX RECOVERY — payload-owned arming gate for inert guided drop tests.
//
// The ground dashboard may request actions, but this gate is authoritative.
// It requires a physical arm input, recorded canopy observation, healthy
// prerequisites, an unlocked authority level, and two commands inside a short
// confirmation window.  It never authorizes powered-flight steering.
// ============================================================================
#pragma once

#include <cstdint>

namespace logic {

enum class GuidedAuthorityLevel : uint8_t {
    NEUTRAL = 0,
    TWENTY_PERCENT = 1,
    FORTY_PERCENT = 2,
    VALIDATED_FULL = 3
};

inline const char* guidedAuthorityLevelName(GuidedAuthorityLevel level) {
    switch (level) {
        case GuidedAuthorityLevel::NEUTRAL: return "NEUTRAL";
        case GuidedAuthorityLevel::TWENTY_PERCENT: return "TWENTY_PERCENT";
        case GuidedAuthorityLevel::FORTY_PERCENT: return "FORTY_PERCENT";
        case GuidedAuthorityLevel::VALIDATED_FULL: return "VALIDATED_FULL";
    }
    return "UNKNOWN";
}

inline uint8_t guidedAuthorityPercent(GuidedAuthorityLevel level) {
    switch (level) {
        case GuidedAuthorityLevel::TWENTY_PERCENT: return 20;
        case GuidedAuthorityLevel::FORTY_PERCENT: return 40;
        case GuidedAuthorityLevel::VALIDATED_FULL: return 100;
        default: return 0;
    }
}

enum class GuidedArmStatus : uint8_t {
    IDLE = 0,
    CANOPY_CONFIRMED = 1,
    PREPARED = 2,
    ARMED = 3,
    REJECTED_FEATURE_LOCK = 4,
    REJECTED_AUTHORITY_LOCK = 5,
    REJECTED_PHYSICAL_ARM = 6,
    REJECTED_CANOPY = 7,
    REJECTED_TARGET = 8,
    REJECTED_SENSORS = 9,
    REJECTED_LOG = 10,
    REJECTED_CONFIRMATION = 11
};

inline const char* guidedArmStatusName(GuidedArmStatus status) {
    switch (status) {
        case GuidedArmStatus::IDLE: return "IDLE";
        case GuidedArmStatus::CANOPY_CONFIRMED: return "CANOPY_CONFIRMED";
        case GuidedArmStatus::PREPARED: return "PREPARED";
        case GuidedArmStatus::ARMED: return "ARMED";
        case GuidedArmStatus::REJECTED_FEATURE_LOCK: return "FEATURE_LOCK";
        case GuidedArmStatus::REJECTED_AUTHORITY_LOCK: return "AUTHORITY_LOCK";
        case GuidedArmStatus::REJECTED_PHYSICAL_ARM: return "PHYSICAL_ARM_REQUIRED";
        case GuidedArmStatus::REJECTED_CANOPY: return "CANOPY_CONFIRMATION_REQUIRED";
        case GuidedArmStatus::REJECTED_TARGET: return "TARGET_NOT_READY";
        case GuidedArmStatus::REJECTED_SENSORS: return "SENSORS_NOT_READY";
        case GuidedArmStatus::REJECTED_LOG: return "LOG_NOT_READY";
        case GuidedArmStatus::REJECTED_CONFIRMATION: return "CONFIRMATION_EXPIRED";
    }
    return "UNKNOWN";
}

struct GuidedDropUnlocks {
    bool feature_enabled = false;
    bool calibration_confirmed = false;
    bool twenty_percent_unlocked = false;
    bool forty_percent_unlocked = false;
    bool validated_full_unlocked = false;
};

struct GuidedArmPrerequisites {
    bool physical_arm_active = false;
    bool target_valid = false;
    bool gps_valid = false;
    bool imu_valid = false;
    bool barometer_valid = false;
    bool servo_valid = false;
    bool log_ready = false;
    bool recorder_idle = true;
};

struct GuidedArmResult {
    GuidedArmStatus status = GuidedArmStatus::IDLE;
    GuidedAuthorityLevel authority = GuidedAuthorityLevel::NEUTRAL;
    bool canopy_confirmed = false;
    bool confirmation_pending = false;
    bool arm_authorized = false;
};

class GuidedDropArmGate {
public:
    explicit GuidedDropArmGate(uint32_t confirmation_window_ms = 5000);

    void reset();
    GuidedArmResult confirmCanopy(uint32_t now_ms, bool recorder_idle);
    GuidedArmResult prepare(uint32_t now_ms, GuidedAuthorityLevel authority,
                            const GuidedDropUnlocks& unlocks,
                            const GuidedArmPrerequisites& prerequisites);
    GuidedArmResult confirmAndArm(uint32_t now_ms,
                                  const GuidedDropUnlocks& unlocks,
                                  const GuidedArmPrerequisites& prerequisites);
    GuidedArmResult snapshot(uint32_t now_ms) const;

private:
    uint32_t confirmation_window_ms_ = 5000;
    uint32_t canopy_confirmed_ms_ = 0;
    uint32_t prepared_ms_ = 0;
    GuidedAuthorityLevel authority_ = GuidedAuthorityLevel::NEUTRAL;
    GuidedArmStatus status_ = GuidedArmStatus::IDLE;

    static bool authorityUnlocked(GuidedAuthorityLevel authority,
                                  const GuidedDropUnlocks& unlocks);
    GuidedArmStatus validate(GuidedAuthorityLevel authority,
                             const GuidedDropUnlocks& unlocks,
                             const GuidedArmPrerequisites& prerequisites) const;
};

}  // namespace logic
