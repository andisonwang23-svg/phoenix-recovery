// ============================================================================
// PHOENIX RECOVERY — Heltec L76K GNSS Driver (NMEA via TinyGPSPlus).
// ============================================================================
#include "gps.h"

#include <TinyGPSPlus.h>

namespace sensors {

GPS::GPS() = default;

GPS::~GPS() { delete gps_; }

bool GPS::begin(HardwareSerial& serial) {
    // Enable the Heltec GNSS connector's active-low power gate before UART.
    // PHOENIX uses the Heltec factory GPS-test mapping:
    // GPIO34 LOW powers VGNSS, GPIO42 HIGH releases GNSS reset.
    pinMode(cfg::PIN_GNSS_POWER, OUTPUT);
    digitalWrite(cfg::PIN_GNSS_POWER, LOW);
    if (cfg::PIN_GNSS_RST >= 0) {
        pinMode(cfg::PIN_GNSS_RST, OUTPUT);
        digitalWrite(cfg::PIN_GNSS_RST, HIGH);
    }
    delay(250);

    serial_ = &serial;
    serial_->begin(cfg::GPS_BAUD, SERIAL_8N1, cfg::PIN_GPS_RX, cfg::PIN_GPS_TX);
    delete gps_;
    gps_ = new TinyGPSPlus();
    if (!gps_) return false;
    initialized_ = true;
    last_update_ms_ = millis();
    return true;
}

bool GPS::update() {
    if (!initialized_ || !gps_ || !serial_) return false;

    bool sentence_complete = false;
    while (serial_->available()) {
        sentence_complete = gps_->encode(static_cast<char>(serial_->read())) || sentence_complete;
        data_.last_nmea_ms = millis();
    }
    if (!sentence_complete && !gps_->location.isUpdated()) return false;

    data_.latitude = gps_->location.lat();
    data_.longitude = gps_->location.lng();
    data_.altitude_m = gps_->altitude.isValid() ? gps_->altitude.meters() : 0.0f;
    data_.speed_mps = gps_->speed.isValid() ? gps_->speed.mps() : 0.0f;
    data_.course_deg = gps_->course.isValid() ? gps_->course.deg() : 0.0f;
    data_.hdop = gps_->hdop.isValid() ? gps_->hdop.hdop() : 99.9f;
    data_.satellites = gps_->satellites.isValid() ? static_cast<int>(gps_->satellites.value()) : 0;
    data_.fix_valid = gps_->location.isValid() && gps_->location.age() < cfg::GPS_LOSS_TIMEOUT_MS;
    data_.valid = data_.fix_valid;
    data_.timestamp_ms = millis();

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
