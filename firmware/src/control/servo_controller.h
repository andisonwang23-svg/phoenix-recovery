// ============================================================================
// PHOENIX RECOVERY — Servo Controller (parafoil brake lines).
// Independent calibration per servo, slew limiting, safety.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "vehicle_state.h"

namespace control {

// Servo configuration
struct ServoConfig {
    int left_channel = cfg::SERVO_LEFT_CHANNEL;
    int right_channel = cfg::SERVO_RIGHT_CHANNEL;
    float left_neutral_us = cfg::SERVO_LEFT_NEUTRAL_US;
    float right_neutral_us = cfg::SERVO_RIGHT_NEUTRAL_US;
    float left_min_us = cfg::SERVO_LEFT_MIN_US;
    float left_max_us = cfg::SERVO_LEFT_MAX_US;
    float right_min_us = cfg::SERVO_RIGHT_MIN_US;
    float right_max_us = cfg::SERVO_RIGHT_MAX_US;
};

class ServoController {
public:
    struct Calibration {
        float neutral_us = 1500.0f;
        float min_us = 1000.0f;
        float max_us = 2000.0f;
        float max_brake_us = 500.0f;  // travel from neutral
        bool reversed = false;
    };

    ServoController();
    ~ServoController();

    // Initialize servos
    bool begin();
    bool begin(const ServoConfig& config);

    // Update servos - call at servo refresh rate
    void update();

    // Set physical brake-line commands. Persisted channel-role mapping routes
    // these to Servo 1 and Servo 2 without assuming their installed sides.
    void setBrakeCommands(float left_cmd, float right_cmd);
    void setServoCommands(float servo1_cmd, float servo2_cmd); // bench only

    // Emergency neutral - immediate safe position
    void emergencyNeutral();

    // Set both to neutral
    void setNeutral();

    // Symmetric brake (flare)
    void setSymmetricBrake(float brake_fraction); // 0 to 1

    // Turn left/right (differential)
    void turnLeft(float amount);  // 0 to 1
    void turnRight(float amount); // 0 to 1

    // Get current pulse widths
    float getLeftUs() const { return left_us_; }
    float getRightUs() const { return right_us_; }
    float getLeftAngleDeg() const;
    float getRightAngleDeg() const;
    float getLeftTurnDeg() const;
    float getRightTurnDeg() const;

    // Get current commands
    float getLeftCommand() const { return left_cmd_; }
    float getRightCommand() const { return right_cmd_; }

    // Check if healthy
    bool isHealthy() const { return initialized_ && left_attached_ && right_attached_; }

    // Load/save calibration from NVS
    bool loadCalibration();
    bool saveCalibration();

    // Set calibration (for bench tuning)
    void setLeftCalibration(const Calibration& cal) { left_cal_ = cal; }
    void setRightCalibration(const Calibration& cal) { right_cal_ = cal; }
    const Calibration& getLeftCalibration() const { return left_cal_; }
    const Calibration& getRightCalibration() const { return right_cal_; }
    void setServo1ControlsLeftBrake(bool value) { servo1_controls_left_brake_ = value; }
    bool servo1ControlsLeftBrake() const { return servo1_controls_left_brake_; }

private:
    bool initialized_ = false;
    bool left_attached_ = false;
    bool right_attached_ = false;

    Servo left_servo_;
    Servo right_servo_;

    Calibration left_cal_;
    Calibration right_cal_;

    float left_us_ = 1500.0f;
    float right_us_ = 1500.0f;
    float left_cmd_ = 0.0f;
    float right_cmd_ = 0.0f;

    uint32_t last_update_ms_ = 0;
    uint32_t update_interval_ms_ = cfg::SERVO_REFRESH_MS;

    // Slew limiting
    float left_us_target_ = 1500.0f;
    float right_us_target_ = 1500.0f;
    int last_left_written_us_ = -1;
    int last_right_written_us_ = -1;
    bool servo1_controls_left_brake_ = true;

    float commandToUs(float cmd, const Calibration& cal);
    float usToCommand(float us, const Calibration& cal);
    float usToAngleDeg(float us, const Calibration& cal) const;
    float usToTurnDeg(float us, const Calibration& cal) const;
    void applySlewLimit(float& current, float target, float max_rate_us_per_s, float dt_s);
};

} // namespace control
