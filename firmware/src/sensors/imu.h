// ============================================================================
// PHOENIX RECOVERY — IMU Driver (BNO08x via SparkFun library).
// Hardware abstraction for the BNO08x IMU.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "config.h"
#include "vehicle_state.h"
#include "logic/data_quality.h"

namespace sensors {

// ---------------------------------------------------------------------------
// BNO08x IMU wrapper
// ---------------------------------------------------------------------------
class IMU {
public:
    struct Data {
        float quat_w = 1.0f, quat_x = 0.0f, quat_y = 0.0f, quat_z = 0.0f;
        float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;      // m/s^2
        float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;         // rad/s
        float linear_accel_x = 0.0f, linear_accel_y = 0.0f, linear_accel_z = 0.0f; // m/s^2
        float gravity_x = 0.0f, gravity_y = 0.0f, gravity_z = 0.0f; // m/s^2
        float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
        bool valid = false;
        uint32_t timestamp_ms = 0;
    };

    IMU();
    ~IMU();

    // Initialize the IMU
    // Returns true on success
    bool begin(TwoWire& wire = Wire);

    // Update - call every loop iteration
    // Returns true if new data available
    bool update();

    // Get latest data
    const Data& getData() const { return data_; }
    Data& getData() { return data_; }

    // Check if IMU is initialized and healthy
    bool isHealthy() const { return initialized_ && data_.valid; }

    // Get chip ID for diagnostics
    uint8_t getChipID() const { return chip_id_; }

    // Calibration status
    uint8_t getCalibrationStatus() const { return cal_status_; }

private:
    bool initialized_ = false;
    uint8_t chip_id_ = 0;
    uint8_t cal_status_ = 0;
    Data data_;
    TwoWire* wire_ = nullptr;
    BNO08x bno_;
    uint32_t last_report_ms_ = 0;
    uint32_t report_interval_ms_ = 0;
    bool has_rotation_vector_ = false;

    // Report IDs we enable
    static constexpr uint8_t REPORT_ROTATION_VECTOR = 0x05;
    static constexpr uint8_t REPORT_LINEAR_ACCELERATION = 0x04;
    static constexpr uint8_t REPORT_ACCELEROMETER = 0x01;
    static constexpr uint8_t REPORT_GYROSCOPE = 0x02;
    static constexpr uint8_t REPORT_GRAVITY = 0x06;

    void enableReports();
    bool readReports();
    void quaternionToEuler(float w, float x, float y, float z,
                           float& roll, float& pitch, float& yaw);
};

} // namespace sensors
