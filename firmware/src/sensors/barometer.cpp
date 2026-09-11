// ============================================================================
// PHOENIX RECOVERY — Barometer Driver Implementation (BMP388).
// ============================================================================
#include "barometer.h"

#include <Adafruit_BMP3XX.h>

namespace sensors {

namespace {
constexpr uint16_t BARO_FAILURES_BEFORE_RECOVERY = 5;
constexpr uint32_t BARO_RECOVERY_COOLDOWN_MS = 1000;
constexpr uint8_t BARO_WARMUP_SAMPLES = 3;
}

Barometer::Barometer() = default;

Barometer::~Barometer() { delete bmp_; }

bool Barometer::begin(TwoWire& wire) {
    wire_ = &wire;

    wire_->begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    wire_->setClock(cfg::I2C_FREQ_HZ);

    delete bmp_;
    bmp_ = new Adafruit_BMP3XX();
    if (!bmp_) return false;

    uint8_t detectedAddress = cfg::BMP388_ADDR;
    if (!bmp_->begin_I2C(detectedAddress, wire_)) {
        detectedAddress = (cfg::BMP388_ADDR == 0x76) ? 0x77 : 0x76;
        if (!bmp_->begin_I2C(detectedAddress, wire_)) {
            Serial.println("[BARO] BMP3xx not found at 0x76 or 0x77");
            delete bmp_;
            bmp_ = nullptr;
            return false;
        }
    }
    Serial.printf("[BARO] BMP3xx detected at 0x%02X\n", detectedAddress);

    if (!configureDevice()) return false;

    update_interval_ms_ = 1000 / cfg::RATE_BARO_HZ;
    initialized_ = true;
    last_update_ms_ = millis();
    warmup_samples_remaining_ = BARO_WARMUP_SAMPLES;

    return true;
}

bool Barometer::configureDevice() {
    if (!bmp_) return false;
    // Configure for stable 25 Hz operation. Configuration is repeated after a
    // bounded device recovery; Wire itself is not restarted because the BNO085
    // shares this bus.
    bmp_->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp_->setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp_->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp_->setOutputDataRate(BMP3_ODR_25_HZ);
    return true;
}

bool Barometer::recoverDevice(uint32_t now_ms) {
    if (!bmp_ || !wire_ || now_ms - last_recovery_attempt_ms_ < BARO_RECOVERY_COOLDOWN_MS)
        return false;
    last_recovery_attempt_ms_ = now_ms;
    data_.recovery_attempts++;
    uint8_t address = cfg::BMP388_ADDR;
    bool recovered = bmp_->begin_I2C(address, wire_);
    if (!recovered) {
        address = (address == 0x76) ? 0x77 : 0x76;
        recovered = bmp_->begin_I2C(address, wire_);
    }
    if (!recovered || !configureDevice()) return false;
    data_.successful_recoveries++;
    data_.consecutive_hardware_failures = 0;
    warmup_samples_remaining_ = BARO_WARMUP_SAMPLES;
    Serial.printf("[BARO] recovered at 0x%02X\n", address);
    return true;
}

bool Barometer::update() {
    if (!initialized_ || !bmp_) return false;

    uint32_t now = millis();
    if (now - last_update_ms_ < update_interval_ms_) return false;
    last_update_ms_ = now;

    if (!bmp_->performReading()) {
        data_.hardware_read_failures++;
        data_.consecutive_hardware_failures++;
        data_.last_hardware_failure_ms = now;
        // A single bus error must not flicker validity. SensorManager owns the
        // freshness timeout and will invalidate the retained sample after 200ms.
        if (data_.consecutive_hardware_failures >= BARO_FAILURES_BEFORE_RECOVERY)
            recoverDevice(now);
        return false;
    }
    data_.hardware_read_successes++;
    data_.consecutive_hardware_failures = 0;
    data_.last_hardware_success_ms = now;
    data_.raw_temperature_c = bmp_->temperature;
    data_.raw_pressure_hpa = bmp_->pressure / 100.0f;
    data_.temperature_c = data_.raw_temperature_c;
    data_.pressure_hpa = data_.raw_pressure_hpa;

    data_.timestamp_ms = now;
    data_.valid = true;

    // Calculate altitude from pressure (relative to ground pressure)
    if (data_.pressure_hpa > 0) {
        data_.altitude_m = 44330.0f * (1.0f - std::pow(data_.pressure_hpa / ground_pressure_hpa_, 1.0f / 5.255f));
    }
    data_.raw_altitude_m = data_.altitude_m;

    // BMP3xx can return a transient first conversion after power-up or device
    // reset. Never establish the plausibility-filter baseline from it.
    if (warmup_samples_remaining_ > 0) {
        warmup_samples_remaining_--;
        data_.warmup_samples_discarded++;
        data_.valid = false;
        return false;
    }

    if (calibrating_ && data_.pressure_hpa > 0) {
        cal_pressure_sum_ += data_.pressure_hpa;
        if (--cal_samples_remaining_ == 0) {
            ground_pressure_hpa_ = cal_pressure_sum_ / static_cast<float>(cal_samples_total_);
            calibrating_ = false;
        }
    }

    // Validate
    logic::BaroQualityConfig qcfg;
    logic::BaroData baro_data;
    baro_data.pressure_hpa = data_.pressure_hpa;
    baro_data.altitude_m = data_.altitude_m;
    baro_data.valid = data_.valid;
    data_.valid = logic::validateBaro(baro_data, previous_valid_, qcfg);
    if (data_.valid) {
        previous_valid_ = baro_data;
        data_.consecutive_quality_rejections = 0;
    } else if (previous_valid_.valid) {
        data_.quality_rejections++;
        data_.consecutive_quality_rejections++;
        data_.last_quality_rejection_ms = now;
        // Retain the last plausible altitude; a single spike must not drive
        // state transitions or flare/landing logic.
        data_.altitude_m = previous_valid_.altitude_m;
        data_.pressure_hpa = previous_valid_.pressure_hpa;
    } else {
        data_.quality_rejections++;
        data_.consecutive_quality_rejections++;
        data_.last_quality_rejection_ms = now;
    }

    return data_.valid;
}

bool Barometer::calibrateGroundPressure(uint32_t samples) {
    if (!initialized_ || samples == 0) return false;

    calibrating_ = true;
    cal_samples_remaining_ = samples;
    cal_samples_total_ = samples;
    cal_pressure_sum_ = 0.0f;

    // Calibration happens over multiple update() calls
    // Caller should loop calling update() until calibrating_ == false
    return true;
}

void Barometer::setGroundPressure(float pressure_hpa) {
    if (!std::isfinite(pressure_hpa) || pressure_hpa <= 0.0f) return;
    ground_pressure_hpa_ = pressure_hpa;
    // Changing the reference legitimately shifts altitude. Clear the old
    // baseline so that the first calibrated AGL sample is accepted rather than
    // misclassified as a pressure spike.
    previous_valid_ = logic::BaroData{};
    data_.valid = false;
}

} // namespace sensors
