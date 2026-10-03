// ============================================================================
// PHOENIX RECOVERY — guarded IMU/barometer ground roll-stabilization test.
//
// This is an explicitly armed, short-duration ground/suspended-test controller.
// It is not a flight mode and it does not claim to make a parafoil payload
// "upright." Differential brakes can influence roll/yaw only after the actual
// rig response and sign have been measured experimentally.
// ============================================================================
#pragma once

#include <cstdint>

namespace logic {

enum class TiltStabilizerStatus : uint8_t {
    OFF = 0,
    ACTIVE,
    COMMAND_TIMEOUT,
    DURATION_EXPIRED,
    INTERLOCK,
    SENSOR_INVALID,
    MOTION_BLOCKED,
    ATTITUDE_LIMIT
};

inline const char* tiltStabilizerStatusName(TiltStabilizerStatus status) {
    switch (status) {
        case TiltStabilizerStatus::OFF: return "DISABLED";
        case TiltStabilizerStatus::ACTIVE: return "ACTIVE";
        case TiltStabilizerStatus::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
        case TiltStabilizerStatus::DURATION_EXPIRED: return "DURATION_EXPIRED";
        case TiltStabilizerStatus::INTERLOCK: return "INTERLOCK";
        case TiltStabilizerStatus::SENSOR_INVALID: return "SENSOR_INVALID";
        case TiltStabilizerStatus::MOTION_BLOCKED: return "MOTION_BLOCKED";
        case TiltStabilizerStatus::ATTITUDE_LIMIT: return "ATTITUDE_LIMIT";
    }
    return "UNKNOWN";
}

struct TiltStabilizerConfig {
    float kp_per_deg = 0.008f;
    float kd_per_dps = 0.003f;
    float roll_deadband_deg = 3.0f;
    float roll_rate_deadband_dps = 2.0f;
    float max_brake_command = 0.15f;
    float command_rate_limit_per_s = 0.30f;
    uint32_t reversal_guard_ms = 1000;
    uint32_t max_duration_ms = 15000;
    float max_vertical_speed_mps = 0.75f;
    float max_vertical_accel_mps2 = 2.0f;
    float max_roll_error_deg = 25.0f;
    float max_abs_pitch_deg = 45.0f;
    float max_roll_rate_dps = 45.0f;
    // +1 means positive roll error commands the physical left brake. Change
    // only after an unloaded direction check. REQUIRES EXPERIMENTAL CALIBRATION.
    float correction_sign = 1.0f;
};

struct TiltStabilizerInput {
    uint32_t now_ms = 0;
    float dt_s = 0.0f;
    bool command_fresh = false;
    bool preflight_allowed = false;
    bool imu_valid = false;
    bool barometer_valid = false;
    bool servo_valid = false;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float roll_rate_dps = 0.0f;
    float vertical_speed_mps = 0.0f;
    float vertical_accel_mps2 = 0.0f;
};

struct TiltStabilizerOutput {
    TiltStabilizerStatus status = TiltStabilizerStatus::OFF;
    bool armed = false;
    bool active = false;
    bool immediate_neutral = true;
    float reference_roll_deg = 0.0f;
    float roll_error_deg = 0.0f;
    float signed_command = 0.0f; // positive = physical left brake
    float left_brake = 0.0f;
    float right_brake = 0.0f;
};

class TiltStabilizer {
public:
    explicit TiltStabilizer(const TiltStabilizerConfig& config = TiltStabilizerConfig{});

    void arm(uint32_t now_ms, float reference_roll_deg);
    void disarm(TiltStabilizerStatus reason = TiltStabilizerStatus::OFF);
    TiltStabilizerOutput update(const TiltStabilizerInput& input);
    bool isArmed() const { return armed_; }

private:
    TiltStabilizerConfig config_;
    bool armed_ = false;
    uint32_t armed_ms_ = 0;
    float reference_roll_deg_ = 0.0f;
    float limited_command_ = 0.0f;
    int last_direction_ = 0;
    uint32_t last_direction_command_ms_ = 0;
    TiltStabilizerStatus last_status_ = TiltStabilizerStatus::OFF;

    static bool finiteInput(const TiltStabilizerInput& input);
    static float wrapDegrees(float angle_deg);
    static float clamp(float value, float lower, float upper);
    void resetCommandState();
    TiltStabilizerOutput neutralOutput(TiltStabilizerStatus status, float roll_error_deg);
};

} // namespace logic
