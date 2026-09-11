#pragma once

#include <cmath>

namespace logic {
struct ServoCalibrationValue {
    float neutral_us = 1500.0f;
    float min_us = 1000.0f;
    float max_us = 2000.0f;
    float max_brake_us = 250.0f;
    bool reversed = false;
};

inline bool validServoCalibration(const ServoCalibrationValue& value) {
    return std::isfinite(value.neutral_us) && std::isfinite(value.min_us) &&
        std::isfinite(value.max_us) && std::isfinite(value.max_brake_us) &&
        value.min_us >= 500.0f && value.max_us <= 2500.0f &&
        value.min_us < value.neutral_us && value.neutral_us < value.max_us &&
        value.max_brake_us > 0.0f &&
        value.max_brake_us <= value.max_us - value.neutral_us &&
        value.max_brake_us <= value.neutral_us - value.min_us;
}

inline ServoCalibrationValue restoreServoCalibration(const ServoCalibrationValue& defaults,
                                                     const ServoCalibrationValue& persisted) {
    return validServoCalibration(persisted) ? persisted : defaults;
}
}  // namespace logic
