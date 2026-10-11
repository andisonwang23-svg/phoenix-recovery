#include "guided_drop_arm_gate.h"

namespace logic {

GuidedDropArmGate::GuidedDropArmGate(uint32_t confirmation_window_ms)
    : confirmation_window_ms_(confirmation_window_ms) {}

void GuidedDropArmGate::reset() {
    canopy_confirmed_ms_ = 0;
    prepared_ms_ = 0;
    authority_ = GuidedAuthorityLevel::NEUTRAL;
    status_ = GuidedArmStatus::IDLE;
}

bool GuidedDropArmGate::authorityUnlocked(GuidedAuthorityLevel authority,
                                          const GuidedDropUnlocks& unlocks) {
    switch (authority) {
        case GuidedAuthorityLevel::TWENTY_PERCENT:
            return unlocks.twenty_percent_unlocked;
        case GuidedAuthorityLevel::FORTY_PERCENT:
            return unlocks.forty_percent_unlocked;
        case GuidedAuthorityLevel::VALIDATED_FULL:
            return unlocks.validated_full_unlocked;
        default:
            return false;
    }
}

GuidedArmStatus GuidedDropArmGate::validate(
    GuidedAuthorityLevel authority,
    const GuidedDropUnlocks& unlocks,
    const GuidedArmPrerequisites& prerequisites) const {
    if (!unlocks.feature_enabled || !unlocks.calibration_confirmed) {
        return GuidedArmStatus::REJECTED_FEATURE_LOCK;
    }
    if (!authorityUnlocked(authority, unlocks)) {
        return GuidedArmStatus::REJECTED_AUTHORITY_LOCK;
    }
    if (!prerequisites.physical_arm_active) {
        return GuidedArmStatus::REJECTED_PHYSICAL_ARM;
    }
    if (canopy_confirmed_ms_ == 0) {
        return GuidedArmStatus::REJECTED_CANOPY;
    }
    if (!prerequisites.target_valid || !prerequisites.gps_valid) {
        return GuidedArmStatus::REJECTED_TARGET;
    }
    if (!prerequisites.imu_valid || !prerequisites.barometer_valid ||
        !prerequisites.servo_valid || !prerequisites.recorder_idle) {
        return GuidedArmStatus::REJECTED_SENSORS;
    }
    if (!prerequisites.log_ready) {
        return GuidedArmStatus::REJECTED_LOG;
    }
    return GuidedArmStatus::PREPARED;
}

GuidedArmResult GuidedDropArmGate::confirmCanopy(uint32_t now_ms,
                                                 bool recorder_idle) {
    if (!recorder_idle) {
        status_ = GuidedArmStatus::REJECTED_SENSORS;
        return snapshot(now_ms);
    }
    canopy_confirmed_ms_ = now_ms == 0 ? 1 : now_ms;
    prepared_ms_ = 0;
    authority_ = GuidedAuthorityLevel::NEUTRAL;
    status_ = GuidedArmStatus::CANOPY_CONFIRMED;
    return snapshot(now_ms);
}

GuidedArmResult GuidedDropArmGate::prepare(
    uint32_t now_ms, GuidedAuthorityLevel authority,
    const GuidedDropUnlocks& unlocks,
    const GuidedArmPrerequisites& prerequisites) {
    authority_ = authority;
    const GuidedArmStatus validated = validate(authority, unlocks, prerequisites);
    if (validated != GuidedArmStatus::PREPARED) {
        prepared_ms_ = 0;
        status_ = validated;
        return snapshot(now_ms);
    }
    prepared_ms_ = now_ms == 0 ? 1 : now_ms;
    status_ = GuidedArmStatus::PREPARED;
    return snapshot(now_ms);
}

GuidedArmResult GuidedDropArmGate::confirmAndArm(
    uint32_t now_ms, const GuidedDropUnlocks& unlocks,
    const GuidedArmPrerequisites& prerequisites) {
    const GuidedArmStatus validated = validate(authority_, unlocks, prerequisites);
    if (validated != GuidedArmStatus::PREPARED) {
        prepared_ms_ = 0;
        status_ = validated;
        return snapshot(now_ms);
    }
    if (prepared_ms_ == 0 ||
        static_cast<uint32_t>(now_ms - prepared_ms_) > confirmation_window_ms_) {
        prepared_ms_ = 0;
        status_ = GuidedArmStatus::REJECTED_CONFIRMATION;
        return snapshot(now_ms);
    }
    prepared_ms_ = 0;
    status_ = GuidedArmStatus::ARMED;
    GuidedArmResult result = snapshot(now_ms);
    result.arm_authorized = true;
    return result;
}

GuidedArmResult GuidedDropArmGate::snapshot(uint32_t now_ms) const {
    GuidedArmResult result;
    result.status = status_;
    result.authority = authority_;
    result.canopy_confirmed = canopy_confirmed_ms_ != 0;
    result.confirmation_pending = prepared_ms_ != 0 &&
        static_cast<uint32_t>(now_ms - prepared_ms_) <= confirmation_window_ms_;
    result.arm_authorized = status_ == GuidedArmStatus::ARMED;
    return result;
}

}  // namespace logic
