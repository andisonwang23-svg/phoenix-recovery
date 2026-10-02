// ============================================================================
// PHOENIX RECOVERY — Heltec L76K GNSS Driver (NMEA via TinyGPSPlus).
// ============================================================================
#include "gps.h"

#include <TinyGPSPlus.h>
#include "logic/gps_link_health.h"

namespace sensors {

GPS::GPS() = default;

GPS::~GPS() { delete gps_; }

void GPS::powerCycleModule() {
    // Original WiFi LoRa 32 V4/V4.3 mapping: GPIO34 is the active-low GNSS
    // power gate and GPIO42 is the active-low GNSS reset. Hold reset while
    // power is removed, then release it only after the rail is restored.
    pinMode(cfg::PIN_GNSS_POWER, OUTPUT);
    if (cfg::PIN_GNSS_RST >= 0) {
        pinMode(cfg::PIN_GNSS_RST, OUTPUT);
        digitalWrite(cfg::PIN_GNSS_RST, LOW);
    }
    digitalWrite(cfg::PIN_GNSS_POWER, HIGH);
    delay(cfg::GPS_POWER_OFF_TIME_MS);
    digitalWrite(cfg::PIN_GNSS_POWER, LOW);
    delay(50);
    if (cfg::PIN_GNSS_RST >= 0) {
        digitalWrite(cfg::PIN_GNSS_RST, HIGH);
    }
    delay(cfg::GPS_BOOT_WAIT_MS);
}

void GPS::restartParser() {
    delete gps_;
    gps_ = new TinyGPSPlus();
    valid_fix_streak_ = 0;
    data_.valid = false;
    data_.fix_valid = false;
}

bool GPS::begin(HardwareSerial& serial) {
    serial_ = &serial;
    serial_->begin(cfg::GPS_BAUD, SERIAL_8N1, cfg::PIN_GPS_RX, cfg::PIN_GPS_TX);
    powerCycleModule();
    while (serial_->available()) serial_->read();
    data_ = Data{};
    prev_data_ = Data{};
    restartParser();
    if (!gps_) return false;
    initialized_ = true;
    startup_ms_ = millis();
    last_recovery_attempt_ms_ = startup_ms_;
    last_update_ms_ = startup_ms_;
    return true;
}

void GPS::invalidateStaleFix(uint32_t now) {
    const bool usable = gps_ && logic::gpsFixUsable(
        now,
        data_.last_nmea_ms,
        cfg::GPS_NMEA_ACTIVE_TIMEOUT_MS,
        gps_->location.isValid(),
        gps_->location.age(),
        cfg::GPS_LOSS_TIMEOUT_MS);
    if (!usable) {
        data_.valid = false;
        data_.fix_valid = false;
        valid_fix_streak_ = 0;
    }
}

void GPS::recoverSilentModule(uint32_t now) {
    if (!logic::gpsSilentRecoveryDue(
            now, startup_ms_, data_.last_nmea_ms,
            last_recovery_attempt_ms_, cfg::GPS_NMEA_STARTUP_GRACE_MS,
            cfg::GPS_NMEA_RECOVERY_RETRY_MS)) return;

    last_recovery_attempt_ms_ = now;
    ++data_.recovery_attempts;
    if (serial_) serial_->end();
    powerCycleModule();
    if (serial_) {
        serial_->begin(cfg::GPS_BAUD, SERIAL_8N1, cfg::PIN_GPS_RX, cfg::PIN_GPS_TX);
        while (serial_->available()) serial_->read();
    }
    restartParser();
    data_.last_uart_byte_ms = 0;
    data_.last_nmea_ms = 0;
    startup_ms_ = millis();
}

bool GPS::update() {
    if (!initialized_ || !gps_ || !serial_) return false;

    const uint32_t now = millis();
    bool sentence_complete = false;
    while (serial_->available()) {
        const char c = static_cast<char>(serial_->read());
        data_.last_uart_byte_ms = millis();
        ++data_.uart_bytes_received;
        if (gps_->encode(c)) {
            sentence_complete = true;
            data_.last_nmea_ms = data_.last_uart_byte_ms;
            ++data_.valid_nmea_sentences;
        }
    }
    data_.failed_nmea_checksums = gps_->failedChecksum();
    if (!sentence_complete) {
        invalidateStaleFix(now);
        recoverSilentModule(now);
        return false;
    }

    data_.latitude = gps_->location.lat();
    data_.longitude = gps_->location.lng();
    data_.altitude_m = gps_->altitude.isValid() ? gps_->altitude.meters() : 0.0f;
    data_.speed_mps = gps_->speed.isValid() ? gps_->speed.mps() : 0.0f;
    data_.course_deg = gps_->course.isValid() ? gps_->course.deg() : 0.0f;
    data_.hdop = gps_->hdop.isValid() ? gps_->hdop.hdop() : 99.9f;
    data_.satellites = gps_->satellites.isValid() ? static_cast<int>(gps_->satellites.value()) : 0;
    data_.fix_valid = gps_->location.isValid() && gps_->location.age() < cfg::GPS_LOSS_TIMEOUT_MS;
    data_.valid = data_.fix_valid;
    data_.timestamp_ms = now;

    // Validate
    logic::GPSQualityConfig qcfg;
    logic::GPSData gps_data;
    gps_data.lat = data_.latitude;
    gps_data.lon = data_.longitude;
    gps_data.speed_mps = data_.speed_mps;
    gps_data.course_deg = data_.course_deg;
    gps_data.hdop = data_.hdop;
    gps_data.satellites = data_.satellites;
    gps_data.valid = data_.valid;
    gps_data.timestamp_ms = data_.timestamp_ms;

    logic::GPSData prev_gps_data;
    prev_gps_data.lat = prev_data_.latitude;
    prev_gps_data.lon = prev_data_.longitude;
    prev_gps_data.speed_mps = prev_data_.speed_mps;
    prev_gps_data.course_deg = prev_data_.course_deg;
    prev_gps_data.hdop = prev_data_.hdop;
    prev_gps_data.satellites = prev_data_.satellites;
    prev_gps_data.valid = prev_data_.valid;
    prev_gps_data.timestamp_ms = prev_data_.timestamp_ms;

    const bool plausible = logic::validateGPS(gps_data, prev_gps_data, qcfg);
    if (plausible) {
        if (valid_fix_streak_ < cfg::GPS_RECOVERY_FIX_COUNT) ++valid_fix_streak_;
        // Keep the plausible point as the jump-check baseline even while the
        // recovery confirmation streak is accumulating.
        prev_data_ = data_;
        data_.valid = valid_fix_streak_ >= cfg::GPS_RECOVERY_FIX_COUNT;
    } else {
        valid_fix_streak_ = 0;
        data_.valid = false;
        // Never expose an impossible jump as the current navigation position.
        if (prev_data_.fix_valid) {
            data_.latitude = prev_data_.latitude;
            data_.longitude = prev_data_.longitude;
        }
    }

    return data_.valid;
}

void GPS::injectNMEA(const char* sentence) {
    if (!gps_ || !sentence) return;
    while (*sentence) gps_->encode(*sentence++);
}

} // namespace sensors
