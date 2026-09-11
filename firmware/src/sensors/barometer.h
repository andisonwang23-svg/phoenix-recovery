// ============================================================================
// PHOENIX RECOVERY — Barometer Driver (BMP388 via Adafruit library).
// ============================================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "vehicle_state.h"
#include "logic/data_quality.h"

class Adafruit_BMP3XX;

namespace sensors {

class Barometer {
public:
    struct Data {
        float pressure_hpa = 0.0f;
        float temperature_c = 0.0f;
        float altitude_m = 0.0f;
        float raw_pressure_hpa = 0.0f;
        float raw_temperature_c = 0.0f;
        float raw_altitude_m = 0.0f;
        bool valid = false;
        uint32_t timestamp_ms = 0;
        uint32_t hardware_read_successes = 0;
        uint32_t hardware_read_failures = 0;
        uint32_t quality_rejections = 0;
        uint32_t last_hardware_success_ms = 0;
        uint32_t last_hardware_failure_ms = 0;
        uint32_t last_quality_rejection_ms = 0;
        uint32_t recovery_attempts = 0;
        uint32_t successful_recoveries = 0;
        uint16_t consecutive_hardware_failures = 0;
        uint16_t consecutive_quality_rejections = 0;
        uint32_t warmup_samples_discarded = 0;
    };

    Barometer();
    ~Barometer();

    // Initialize the BMP388
    bool begin(TwoWire& wire = Wire);

    // Update - call every loop iteration
    bool update();

    // Get latest data
    const Data& getData() const { return data_; }
    Data& getData() { return data_; }

    // Set ground pressure reference (for AGL altitude)
    void setGroundPressure(float pressure_hpa);
    float getGroundPressure() const { return ground_pressure_hpa_; }

    // Calibrate ground pressure (average multiple samples)
    bool calibrateGroundPressure(uint32_t samples = cfg::BARO_CAL_SAMPLES);

    // Check if healthy
    bool isHealthy() const { return initialized_ && data_.valid; }
    bool isAvailable() const { return initialized_ && bmp_ != nullptr; }

private:
    bool initialized_ = false;
    Data data_;
    logic::BaroData previous_valid_;
    TwoWire* wire_ = nullptr;
    Adafruit_BMP3XX* bmp_ = nullptr;
    float ground_pressure_hpa_ = 1013.25f;
    uint32_t last_update_ms_ = 0;
    uint32_t update_interval_ms_ = 0;
    uint32_t last_recovery_attempt_ms_ = 0;
    uint8_t warmup_samples_remaining_ = 0;
    bool configureDevice();
    bool recoverDevice(uint32_t now_ms);

    // For calibration
    uint32_t cal_samples_remaining_ = 0;
    uint32_t cal_samples_total_ = 0;
    float cal_pressure_sum_ = 0.0f;
    bool calibrating_ = false;
};

} // namespace sensors
