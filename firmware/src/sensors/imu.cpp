// ============================================================================
// PHOENIX RECOVERY — IMU Driver Implementation (BNO08x via SparkFun library).
// ============================================================================
#include "imu.h"

#include <SparkFun_BNO08x_Arduino_Library.h>

namespace sensors {

IMU::IMU() = default;

IMU::~IMU() = default;

bool IMU::begin(TwoWire& wire) {
    wire_ = &wire;

    // Initialize I2C with custom pins
    wire_->begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    wire_->setClock(cfg::I2C_FREQ_HZ);

    // Accept either standard BNO08x address. Breakout-board ADDR straps vary,
    // so a hard-coded address made a healthy module look absent in the UI.
    uint8_t detectedAddress = cfg::BNO08X_ADDR;
    if (!bno_.begin(detectedAddress, wire)) {
        detectedAddress = (cfg::BNO08X_ADDR == 0x4A) ? 0x4B : 0x4A;
        if (!bno_.begin(detectedAddress, wire)) {
            Serial.println("[IMU] BNO08x not found at 0x4A or 0x4B");
            return false;
        }
    }
    Serial.printf("[IMU] BNO08x detected at 0x%02X\n", detectedAddress);

    // Enable required reports at 50 Hz (20ms interval)
    report_interval_ms_ = 1000 / cfg::RATE_IMU_HZ;
    bno_.enableRotationVector(report_interval_ms_);
    bno_.enableLinearAccelerometer(report_interval_ms_);
    bno_.enableAccelerometer(report_interval_ms_);
    bno_.enableGyro(report_interval_ms_);
    bno_.enableGravity(report_interval_ms_);

    initialized_ = true;
    last_report_ms_ = millis();

    return true;
}

bool IMU::update() {
    if (!initialized_) return false;

    uint32_t now = millis();
    if (now - last_report_ms_ < report_interval_ms_) return false;
    last_report_ms_ = now;

    // Read all available reports
    return readReports();
}

void IMU::enableReports() {
    bno_.enableRotationVector(report_interval_ms_);
    bno_.enableLinearAccelerometer(report_interval_ms_);
    bno_.enableAccelerometer(report_interval_ms_);
    bno_.enableGyro(report_interval_ms_);
    bno_.enableGravity(report_interval_ms_);
}

bool IMU::readReports() {
    if (bno_.wasReset()) enableReports();
    if (!bno_.getSensorEvent()) return false;

    // The BNO08x interleaves reports. Getter fields are only meaningful for
    // the current report type; reading every getter for every event corrupted
    // the quaternion whenever an accelerometer/gyro report arrived.
    switch (bno_.getSensorEventID()) {
        case SENSOR_REPORTID_ROTATION_VECTOR:
            data_.quat_x = bno_.getQuatI();
            data_.quat_y = bno_.getQuatJ();
            data_.quat_z = bno_.getQuatK();
            data_.quat_w = bno_.getQuatReal();
            cal_status_ = bno_.getQuatAccuracy();
            quaternionToEuler(data_.quat_w, data_.quat_x, data_.quat_y, data_.quat_z,
                              data_.roll_deg, data_.pitch_deg, data_.yaw_deg);
            has_rotation_vector_ = true;
            break;
        case SENSOR_REPORTID_ACCELEROMETER:
            data_.accel_x = bno_.getAccelX();
            data_.accel_y = bno_.getAccelY();
            data_.accel_z = bno_.getAccelZ();
            break;
        case SENSOR_REPORTID_LINEAR_ACCELERATION:
            data_.linear_accel_x = bno_.getLinAccelX();
            data_.linear_accel_y = bno_.getLinAccelY();
            data_.linear_accel_z = bno_.getLinAccelZ();
            break;
        case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
            data_.gyro_x = bno_.getGyroX();
            data_.gyro_y = bno_.getGyroY();
            data_.gyro_z = bno_.getGyroZ();
            break;
        case SENSOR_REPORTID_GRAVITY:
            data_.gravity_x = bno_.getGravityX();
            data_.gravity_y = bno_.getGravityY();
            data_.gravity_z = bno_.getGravityZ();
            break;
        default:
            break;
    }

    data_.timestamp_ms = millis();
    data_.valid = has_rotation_vector_;

    // Validate
    logic::IMUQualityConfig qcfg;
    logic::IMUData imu_data;
    imu_data.quat_w = data_.quat_w;
    imu_data.quat_x = data_.quat_x;
    imu_data.quat_y = data_.quat_y;
    imu_data.quat_z = data_.quat_z;
    imu_data.linear_accel_x = data_.linear_accel_x;
    imu_data.linear_accel_y = data_.linear_accel_y;
    imu_data.linear_accel_z = data_.linear_accel_z;
    imu_data.gyro_x = data_.gyro_x;
    imu_data.gyro_y = data_.gyro_y;
    imu_data.gyro_z = data_.gyro_z;
    imu_data.valid = data_.valid;

    data_.valid = logic::validateIMU(imu_data, qcfg);

    return data_.valid;
}

void IMU::quaternionToEuler(float w, float x, float y, float z,
                            float& roll, float& pitch, float& yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI / 2.0f, sinp) * 180.0f / M_PI;
    else
        pitch = std::asin(sinp) * 180.0f / M_PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

} // namespace sensors
