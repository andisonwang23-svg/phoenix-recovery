// ============================================================================
// PHOENIX RECOVERY — Heltec L76K GNSS Driver (NMEA via TinyGPSPlus).
// ============================================================================
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "vehicle_state.h"
#include "logic/data_quality.h"

class TinyGPSPlus;

namespace sensors {

class GPS {
public:
    struct Data {
        double latitude = 0.0;
        double longitude = 0.0;
        float altitude_m = 0.0f;
        float speed_mps = 0.0f;
        float course_deg = 0.0f;
        float hdop = 0.0f;
        int satellites = 0;
        int satellites_in_view = 0;
        bool valid = false;
        bool fix_valid = false;
        uint32_t timestamp_ms = 0;
        uint32_t last_uart_byte_ms = 0;
        uint32_t last_nmea_ms = 0;
        uint32_t uart_bytes_received = 0;
        uint32_t valid_nmea_sentences = 0;
        uint32_t failed_nmea_checksums = 0;
        uint32_t recovery_attempts = 0;
        uint32_t last_gsv_ms = 0;
    };

    GPS();
    ~GPS();

    // Initialize GPS on UART2
    bool begin(HardwareSerial& serial = Serial2);

    // Update - call every loop iteration
    bool update();

    // Get latest data
    const Data& getData() const { return data_; }
    Data& getData() { return data_; }

    // Check if healthy
    bool isHealthy() const { return initialized_ && data_.valid; }

    // Inject NMEA sentence for testing
    void injectNMEA(const char* sentence);

private:
    bool initialized_ = false;
    Data data_;
    Data prev_data_;
    HardwareSerial* serial_ = nullptr;
    TinyGPSPlus* gps_ = nullptr;
    uint32_t last_update_ms_ = 0;
    uint32_t update_interval_ms_ = 1000; // GPS native rate ~1 Hz
    uint8_t valid_fix_streak_ = 0;
    uint32_t startup_ms_ = 0;
    uint32_t last_recovery_attempt_ms_ = 0;
    uint32_t gsv_window_start_ms_ = 0;

    void powerCycleModule();
    void restartParser();
    void invalidateStaleFix(uint32_t now);
    void recoverSilentModule(uint32_t now);

    // NMEA buffer
    static constexpr size_t NMEA_BUF_SIZE = 128;
    char nmea_buf_[NMEA_BUF_SIZE];
    size_t nmea_idx_ = 0;
};

} // namespace sensors
