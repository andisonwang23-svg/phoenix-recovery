// ============================================================================
// PHOENIX RECOVERY — Servo Controller Implementation.
// ============================================================================
#include "servo_controller.h"

#include <Preferences.h>
#include "logic/servo_calibration_logic.h"

namespace control {

ServoController::ServoController() = default;

ServoController::~ServoController() {
    if (left_attached_) left_servo_.detach();
    if (right_attached_) right_servo_.detach();
}

bool ServoController::begin() {
    // Load calibration from NVS
    loadCalibration();

    // Use defaults if not calibrated
    if (left_cal_.neutral_us == 0) {
        left_cal_.neutral_us = cfg::SERVO_LEFT_NEUTRAL_US;
        left_cal_.min_us = cfg::SERVO_LEFT_MIN_US;
        left_cal_.max_us = cfg::SERVO_LEFT_MAX_US;
        left_cal_.max_brake_us = cfg::SERVO_LEFT_MAX_BRAKE_US;
        left_cal_.reversed = cfg::SERVO_LEFT_REVERSED;
    }
    if (right_cal_.neutral_us == 0) {
        right_cal_.neutral_us = cfg::SERVO_RIGHT_NEUTRAL_US;
        right_cal_.min_us = cfg::SERVO_RIGHT_MIN_US;
        right_cal_.max_us = cfg::SERVO_RIGHT_MAX_US;
        right_cal_.max_brake_us = cfg::SERVO_RIGHT_MAX_BRAKE_US;
        right_cal_.reversed = cfg::SERVO_RIGHT_REVERSED;
    }

    // Attach servos
    left_servo_.setPeriodHertz(50); // Standard 50 Hz
    right_servo_.setPeriodHertz(50);

    // ESP32Servo returns an integer channel from attach(). On ESP32-S3 it can
    // legitimately return 0 when MCPWM was attached successfully, so treating
    // that return value as a bool incorrectly disables the entire controller.
    left_servo_.attach(cfg::PIN_SERVO_LEFT, left_cal_.min_us, left_cal_.max_us);
    right_servo_.attach(cfg::PIN_SERVO_RIGHT, right_cal_.min_us, right_cal_.max_us);
    left_attached_ = left_servo_.attached();
    right_attached_ = right_servo_.attached();

    if (!left_attached_ || !right_attached_) {
        return false;
    }

    // Start at neutral
    emergencyNeutral();

    initialized_ = true;
    last_update_ms_ = millis();
    update_interval_ms_ = cfg::SERVO_REFRESH_MS;

    return true;
}

bool ServoController::begin(const ServoConfig& config) {
    // Seed defaults from the compiled configuration, then restore a persisted
    // bench calibration if one exists. Previously this overload silently
    // discarded NVS every boot because main() always supplies pin channels.
    left_cal_.neutral_us = config.left_neutral_us;
    left_cal_.min_us = config.left_min_us;
    left_cal_.max_us = config.left_max_us;
    left_cal_.max_brake_us = cfg::SERVO_LEFT_MAX_BRAKE_US;
    left_cal_.reversed = cfg::SERVO_LEFT_REVERSED;

    right_cal_.neutral_us = config.right_neutral_us;
    right_cal_.min_us = config.right_min_us;
    right_cal_.max_us = config.right_max_us;
    right_cal_.max_brake_us = cfg::SERVO_RIGHT_MAX_BRAKE_US;
    right_cal_.reversed = cfg::SERVO_RIGHT_REVERSED;

    loadCalibration();

    // Attach servos
    left_servo_.setPeriodHertz(50);
    right_servo_.setPeriodHertz(50);

    left_servo_.attach(config.left_channel, left_cal_.min_us, left_cal_.max_us);
    right_servo_.attach(config.right_channel, right_cal_.min_us, right_cal_.max_us);
    left_attached_ = left_servo_.attached();
    right_attached_ = right_servo_.attached();

    if (!left_attached_ || !right_attached_) {
        return false;
    }

    emergencyNeutral();
    initialized_ = true;
    last_update_ms_ = millis();

    return true;
}

void ServoController::update() {
    if (!initialized_) return;

    uint32_t now = millis();
    if (now - last_update_ms_ < update_interval_ms_) return;
    float dt_s = (now - last_update_ms_) / 1000.0f;
    last_update_ms_ = now;

    // Apply slew limiting
    float max_rate = cfg::SERVO_SLEW_RATE_US_PER_S;
    applySlewLimit(left_us_, left_us_target_, max_rate, dt_s);
    applySlewLimit(right_us_, right_us_target_, max_rate, dt_s);

    // MCPWM keeps producing the last pulse without repeated writes. Updating
    // only when the integer pulse changes avoids needless high-rate timer
    // reconfiguration that can present as audible or visible servo twitch.
    const int left_write_us = static_cast<int>(lroundf(left_us_));
    const int right_write_us = static_cast<int>(lroundf(right_us_));
    if (left_write_us != last_left_written_us_) {
        left_servo_.writeMicroseconds(left_write_us);
        last_left_written_us_ = left_write_us;
    }
    if (right_write_us != last_right_written_us_) {
        right_servo_.writeMicroseconds(right_write_us);
        last_right_written_us_ = right_write_us;
    }
}

void ServoController::setBrakeCommands(float left_cmd, float right_cmd) {
    if (servo1_controls_left_brake_) setServoCommands(left_cmd, right_cmd);
    else setServoCommands(right_cmd, left_cmd);
}

void ServoController::setServoCommands(float left_cmd, float right_cmd) {
    if (!initialized_) return;

    // Clamp commands
    left_cmd = constrain(left_cmd, -1.0f, 1.0f);
    right_cmd = constrain(right_cmd, -1.0f, 1.0f);
    if (fabsf(left_cmd) < 0.005f) left_cmd = 0.0f;
    if (fabsf(right_cmd) < 0.005f) right_cmd = 0.0f;

    // Apply MAX_STEERING_COMMAND limit (from config)
    float max_cmd = cfg::MAX_STEERING_COMMAND;
    left_cmd = constrain(left_cmd, -max_cmd, max_cmd);
    right_cmd = constrain(right_cmd, -max_cmd, max_cmd);

    left_cmd_ = left_cmd;
    right_cmd_ = right_cmd;

    left_us_target_ = commandToUs(left_cmd, left_cal_);
    right_us_target_ = commandToUs(right_cmd, right_cal_);
}

void ServoController::emergencyNeutral() {
    left_us_ = left_cal_.neutral_us;
    right_us_ = right_cal_.neutral_us;
    left_us_target_ = left_cal_.neutral_us;
    right_us_target_ = right_cal_.neutral_us;
    left_cmd_ = 0.0f;
    right_cmd_ = 0.0f;

    if (left_attached_) {
        last_left_written_us_ = static_cast<int>(lroundf(left_us_));
        left_servo_.writeMicroseconds(last_left_written_us_);
    }
    if (right_attached_) {
        last_right_written_us_ = static_cast<int>(lroundf(right_us_));
        right_servo_.writeMicroseconds(last_right_written_us_);
    }
}

void ServoController::setNeutral() {
    emergencyNeutral();
}

void ServoController::setSymmetricBrake(float brake_fraction) {
    brake_fraction = constrain(brake_fraction, 0.0f, 1.0f);
    // Apply flare max brake limit
    brake_fraction = constrain(brake_fraction, 0.0f, cfg::FLARE_MAX_BRAKE);
    setBrakeCommands(brake_fraction, brake_fraction);
}

void ServoController::turnLeft(float amount) {
    amount = constrain(amount, 0.0f, 1.0f);
    setBrakeCommands(-amount, amount);
}

void ServoController::turnRight(float amount) {
    amount = constrain(amount, 0.0f, 1.0f);
    setBrakeCommands(amount, -amount);
}

float ServoController::getLeftAngleDeg() const {
    return usToAngleDeg(left_us_, left_cal_);
}

float ServoController::getRightAngleDeg() const {
    return usToAngleDeg(right_us_, right_cal_);
}

float ServoController::getLeftTurnDeg() const {
    return usToTurnDeg(left_us_, left_cal_);
}

float ServoController::getRightTurnDeg() const {
    return usToTurnDeg(right_us_, right_cal_);
}

float ServoController::commandToUs(float cmd, const Calibration& cal) {
    // cmd: -1 to +1 (fraction of max brake travel)
    // Returns pulse width in microseconds

    float travel = cal.max_brake_us * cmd; // can be negative

    if (cal.reversed) {
        travel = -travel;
    }

    float us = cal.neutral_us + travel;

    // Clamp to servo limits
    us = constrain(us, cal.min_us, cal.max_us);

    return us;
}

float ServoController::usToCommand(float us, const Calibration& cal) {
    float travel = us - cal.neutral_us;
    if (cal.reversed) travel = -travel;
    return travel / cal.max_brake_us;
}

float ServoController::usToAngleDeg(float us, const Calibration& cal) const {
    const float range_us = cal.max_us - cal.min_us;
    if (range_us <= 1.0f) return 90.0f;
    const float clamped_us = constrain(us, cal.min_us, cal.max_us);
    return constrain((clamped_us - cal.min_us) * 180.0f / range_us, 0.0f, 180.0f);
}

float ServoController::usToTurnDeg(float us, const Calibration& cal) const {
    const float range_us = cal.max_us - cal.min_us;
    if (range_us <= 1.0f) return 0.0f;
    float travel_us = us - cal.neutral_us;
    if (cal.reversed) travel_us = -travel_us;
    return travel_us * 180.0f / range_us;
}

void ServoController::applySlewLimit(float& current, float target, float max_rate_us_per_s, float dt_s) {
    float max_delta = max_rate_us_per_s * dt_s;
    float delta = target - current;

    if (delta > max_delta) delta = max_delta;
    else if (delta < -max_delta) delta = -max_delta;

    current += delta;
}

bool ServoController::loadCalibration() {
    Preferences prefs;
    if (!prefs.begin("servo_cal", true)) return false; // read-only

    left_cal_.neutral_us = prefs.getFloat("L_neutral", cfg::SERVO_LEFT_NEUTRAL_US);
    left_cal_.min_us = prefs.getFloat("L_min", cfg::SERVO_LEFT_MIN_US);
    left_cal_.max_us = prefs.getFloat("L_max", cfg::SERVO_LEFT_MAX_US);
    // Versioned key intentionally adopts the enlarged calibrated travel while
    // leaving older 350 us installations recoverable in NVS.
    left_cal_.max_brake_us = prefs.getFloat("L_brake_v2", cfg::SERVO_LEFT_MAX_BRAKE_US);
    left_cal_.reversed = prefs.getBool("L_rev", cfg::SERVO_LEFT_REVERSED);

    right_cal_.neutral_us = prefs.getFloat("R_neutral", cfg::SERVO_RIGHT_NEUTRAL_US);
    right_cal_.min_us = prefs.getFloat("R_min", cfg::SERVO_RIGHT_MIN_US);
    right_cal_.max_us = prefs.getFloat("R_max", cfg::SERVO_RIGHT_MAX_US);
    right_cal_.max_brake_us = prefs.getFloat("R_brake_v2", cfg::SERVO_RIGHT_MAX_BRAKE_US);
    right_cal_.reversed = prefs.getBool("R_rev", cfg::SERVO_RIGHT_REVERSED);
    servo1_controls_left_brake_ = prefs.getBool("S1_is_left", true);

    prefs.end();
    logic::ServoCalibrationValue left;
    left.neutral_us = left_cal_.neutral_us; left.min_us = left_cal_.min_us;
    left.max_us = left_cal_.max_us; left.max_brake_us = left_cal_.max_brake_us;
    left.reversed = left_cal_.reversed;
    logic::ServoCalibrationValue right;
    right.neutral_us = right_cal_.neutral_us; right.min_us = right_cal_.min_us;
    right.max_us = right_cal_.max_us; right.max_brake_us = right_cal_.max_brake_us;
    right.reversed = right_cal_.reversed;
    if (!logic::validServoCalibration(left) || !logic::validServoCalibration(right)) {
        left_cal_.neutral_us = cfg::SERVO_LEFT_NEUTRAL_US;
        left_cal_.min_us = cfg::SERVO_LEFT_MIN_US; left_cal_.max_us = cfg::SERVO_LEFT_MAX_US;
        left_cal_.max_brake_us = cfg::SERVO_LEFT_MAX_BRAKE_US;
        left_cal_.reversed = cfg::SERVO_LEFT_REVERSED;
        right_cal_.neutral_us = cfg::SERVO_RIGHT_NEUTRAL_US;
        right_cal_.min_us = cfg::SERVO_RIGHT_MIN_US; right_cal_.max_us = cfg::SERVO_RIGHT_MAX_US;
        right_cal_.max_brake_us = cfg::SERVO_RIGHT_MAX_BRAKE_US;
        right_cal_.reversed = cfg::SERVO_RIGHT_REVERSED;
        return false;
    }
    return true;
}

bool ServoController::saveCalibration() {
    Preferences prefs;
    if (!prefs.begin("servo_cal", false)) return false; // read-write

    prefs.putFloat("L_neutral", left_cal_.neutral_us);
    prefs.putFloat("L_min", left_cal_.min_us);
    prefs.putFloat("L_max", left_cal_.max_us);
    prefs.putFloat("L_brake_v2", left_cal_.max_brake_us);
    prefs.putBool("L_rev", left_cal_.reversed);

    prefs.putFloat("R_neutral", right_cal_.neutral_us);
    prefs.putFloat("R_min", right_cal_.min_us);
    prefs.putFloat("R_max", right_cal_.max_us);
    prefs.putFloat("R_brake_v2", right_cal_.max_brake_us);
    prefs.putBool("R_rev", right_cal_.reversed);
    prefs.putBool("S1_is_left", servo1_controls_left_brake_);

    prefs.end();
    return true;
}

} // namespace control
