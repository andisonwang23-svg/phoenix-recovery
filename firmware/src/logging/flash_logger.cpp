// ============================================================================
// PHOENIX RECOVERY — Flash Logger Implementation.
// ============================================================================
#include "flash_logger.h"
#include <SPIFFS.h>

namespace phoenix {
namespace logging {

FlashLogger::FlashLogger() = default;

bool FlashLogger::begin(const FlashLoggerConfig& config) {
    config_ = config;

    if (!config_.enabled) {
        return true; // Logging disabled, but initialization successful
    }

    // Initialize SPIFFS for flash storage
    if (!SPIFFS.begin(true)) {
        Serial.println("[FlashLogger] Failed to initialize SPIFFS");
        return false;
    }

    initialized_ = true;
    // Do not silently overwrite flight_0.log after every reboot. Select the
    // first unused slot; retention/archival can be handled after recovery.
    for (uint16_t candidate = 0; candidate < 255; ++candidate) {
        String filename = "/flight_" + String(candidate) + ".log";
        if (!SPIFFS.exists(filename.c_str())) {
            current_flight_.flight_number = static_cast<uint8_t>(candidate);
            break;
        }
    }
    Serial.println("[FlashLogger] Initialized successfully");
    return true;
}

bool FlashLogger::startFlightLog() {
    if (!initialized_ || !config_.enabled) return false;

    logging_active_ = false;
    write_fault_reported_ = false;

    // Create new flight log file
    String filename = "/flight_" + String(current_flight_.flight_number) + ".log";
    File file = SPIFFS.open(filename.c_str(), FILE_WRITE);
    if (!file) {
        Serial.println("[FlashLogger] Failed to create flight log file");
        return false;
    }

    // Initialize flight header
    memcpy(current_flight_.magic, "PRFL", 4);
    current_flight_.version = 2;
    current_flight_.start_time_ms = millis();
    current_flight_.end_time_ms = 0;
    current_flight_.entry_count = 0;
    current_flight_.data_size = 0;
    current_flight_.crc = 0;

    // Write header
    file.seek(0);
    const size_t header_written =
        file.write(reinterpret_cast<const uint8_t*>(&current_flight_), sizeof(FlightLogHeader));
    file.close();

    if (header_written != sizeof(FlightLogHeader)) {
        SPIFFS.remove(filename.c_str());
        Serial.println("[FlashLogger] Flight log storage is full or unwritable; logging disabled");
        write_fault_reported_ = true;
        return false;
    }

    logging_active_ = true;
    entry_count_ = 0;
    data_size_ = 0;
    sequence_ = 0;

    Serial.printf("[FlashLogger] Started flight log %d\n", current_flight_.flight_number);
    return true;
}

bool FlashLogger::endFlightLog() {
    if (!initialized_ || !logging_active_) return false;

    // Update header with final stats
    current_flight_.end_time_ms = millis();
    current_flight_.entry_count = entry_count_;
    current_flight_.data_size = data_size_;

    // Calculate CRC over all data (simplified)
    current_flight_.crc = calculateCRC(reinterpret_cast<const uint8_t*>(&current_flight_),
                                       sizeof(FlightLogHeader) - sizeof(uint16_t));

    // Rewrite header with updated info
    String filename = "/flight_" + String(current_flight_.flight_number) + ".log";
    File file = SPIFFS.open(filename.c_str(), "r+");
    if (!file) {
        Serial.println("[FlashLogger] Failed to update flight log header");
        logging_active_ = false;
        return false;
    }

    file.seek(0);
    file.write(reinterpret_cast<const uint8_t*>(&current_flight_), sizeof(FlightLogHeader));
    file.close();

    logging_active_ = false;
    current_flight_.flight_number++;

    Serial.printf("[FlashLogger] Ended flight log, %d entries, %d bytes\n",
                  entry_count_, data_size_);
    return true;
}

bool FlashLogger::log(LogEntryType type, const uint8_t* data, uint16_t length) {
    if (!initialized_ || !logging_active_) return false;
    if (length > sizeof(LogEntry::data)) return false;

    if (entry_count_ >= config_.max_entries ||
        data_size_ + sizeof(LogEntryHeader) + length > config_.max_data_size) {
        logging_active_ = false;
        if (!write_fault_reported_) {
            Serial.println("[FlashLogger] Configured log limit reached; logging stopped");
            write_fault_reported_ = true;
        }
        return false;
    }

    LogEntry entry;
    entry.header.timestamp_ms = millis();
    entry.header.type = static_cast<uint8_t>(type);
    entry.header.sequence = sequence_++;
    entry.header.length = length;

    memcpy(entry.data, data, length);

    if (!writeEntry(entry)) {
        // A full or damaged filesystem must fail once, not retry at the 10 Hz
        // flight-log rate. Repeated failed opens previously flooded Serial and
        // stole time from the half-duplex LoRa receive window.
        logging_active_ = false;
        if (!write_fault_reported_) {
            Serial.println("[FlashLogger] Log write failed; logging disabled until reboot");
            write_fault_reported_ = true;
        }
        return false;
    }

    entry_count_++;
    data_size_ += sizeof(LogEntryHeader) + length;

    return true;
}

bool FlashLogger::logFlightState(const VehicleState& state) {
    // Pack relevant flight state data
    struct FlightStateData {
        uint8_t flight_state;
        uint8_t guidance_mode;
        float altitude_agl;
        float vertical_speed;
        float distance_to_target;
    } data;

    data.flight_state = static_cast<uint8_t>(state.flight_state);
    data.guidance_mode = static_cast<uint8_t>(state.guidance_mode);
    data.altitude_agl = state.altitude_agl_m;
    data.vertical_speed = state.vertical_speed_mps;
    data.distance_to_target = state.distance_to_target_m;

    return log(LogEntryType::FLIGHT_STATE, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
}

bool FlashLogger::logTelemetry(const VehicleState& state) {
    // One synchronized, replayable snapshot. Requested commands are retained
    // separately from actual outputs so a stalled/saturated actuator is visible.
    struct TelemetryData {
        uint8_t schema_version;
        uint8_t drop_test_state;
        uint16_t drop_test_id;
        uint32_t sample_timestamp_ms;
        uint32_t configuration_version;
        uint8_t flight_state;
        uint8_t guidance_mode;
        uint8_t failure_code;
        uint8_t validity_bits;
        uint32_t gps_age_ms;
        uint32_t baro_age_ms;
        uint32_t imu_age_ms;
        uint32_t gps_last_update_ms;
        uint32_t baro_last_update_ms;
        uint32_t imu_last_update_ms;
        double latitude;
        double longitude;
        float gps_altitude;
        float hdop;
        uint8_t satellites;
        float altitude_agl;
        float barometric_altitude;
        float barometric_pressure_hpa;
        float barometric_temperature_c;
        float vertical_speed;
        float vertical_accel;
        float ground_speed;
        float gps_course;
        float target_bearing;
        float heading_error;
        float distance_to_target;
        float roll;
        float pitch;
        float yaw;
        float angular_rate;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float requested_left;
        float requested_right;
        float actual_left;
        float actual_right;
        float actual_left_us;
        float actual_right_us;
        float battery_voltage;
        float servo_rail_voltage;
        uint32_t drop_release_ms;
        uint32_t canopy_signature_ms;
        uint32_t stable_descent_ms;
        uint32_t landing_confirm_ms;
    } data;

    static_assert(sizeof(TelemetryData) <= sizeof(LogEntry::data),
                  "Telemetry log record exceeds FlashLogger entry size");
    data.schema_version = 2;
    data.drop_test_state = static_cast<uint8_t>(state.drop_test_state);
    data.drop_test_id = state.drop_test_id;
    data.sample_timestamp_ms = state.timestamp_ms;
    data.configuration_version = state.configuration_version;
    data.flight_state = static_cast<uint8_t>(state.flight_state);
    data.guidance_mode = static_cast<uint8_t>(state.guidance_mode);
    data.failure_code = static_cast<uint8_t>(state.failure_code);
    data.validity_bits = (state.gps_valid ? 1U : 0U) |
                         (state.imu_valid ? 2U : 0U) |
                         (state.barometer_valid ? 4U : 0U) |
                         (state.target_valid ? 8U : 0U) |
                         (state.battery_voltage_valid ? 16U : 0U) |
                         (state.servo_rail_voltage_valid ? 32U : 0U);
    data.gps_age_ms = state.gps_age_ms;
    data.baro_age_ms = state.baro_age_ms;
    data.imu_age_ms = state.imu_age_ms;
    data.gps_last_update_ms = state.gps_last_update_ms;
    data.baro_last_update_ms = state.baro_last_update_ms;
    data.imu_last_update_ms = state.imu_last_update_ms;
    data.latitude = state.latitude;
    data.longitude = state.longitude;
    data.gps_altitude = state.gps_altitude_m;
    data.hdop = state.hdop;
    data.satellites = static_cast<uint8_t>(state.satellite_count < 0 ? 0 :
                                           (state.satellite_count > 255 ? 255 : state.satellite_count));
    data.altitude_agl = state.altitude_agl_m;
    data.barometric_altitude = state.barometric_altitude_m;
    data.barometric_pressure_hpa = state.barometric_pressure_hpa;
    data.barometric_temperature_c = state.barometric_temperature_c;
    data.vertical_speed = state.vertical_speed_mps;
    data.vertical_accel = state.vertical_accel_mps2;
    data.ground_speed = state.ground_speed_mps;
    data.gps_course = state.gps_course_deg;
    data.target_bearing = state.target_bearing_deg;
    data.heading_error = state.heading_error_deg;
    data.distance_to_target = state.distance_to_target_m;
    data.roll = state.roll_deg;
    data.pitch = state.pitch_deg;
    data.yaw = state.yaw_deg;
    data.angular_rate = state.angular_rate_dps;
    data.gyro_x = state.gyro_x_dps;
    data.gyro_y = state.gyro_y_dps;
    data.gyro_z = state.gyro_z_dps;
    data.requested_left = state.requested_left_servo_command;
    data.requested_right = state.requested_right_servo_command;
    data.actual_left = state.left_servo_command;
    data.actual_right = state.right_servo_command;
    data.actual_left_us = state.left_servo_us;
    data.actual_right_us = state.right_servo_us;
    data.battery_voltage = state.battery_voltage_v;
    data.servo_rail_voltage = state.servo_rail_voltage_v;
    data.drop_release_ms = state.drop_test_release_confirm_ms;
    data.canopy_signature_ms = state.drop_test_canopy_signature_ms;
    data.stable_descent_ms = state.drop_test_stable_descent_ms;
    data.landing_confirm_ms = state.drop_test_landing_confirm_ms;

    return log(LogEntryType::TELEMETRY, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
}

bool FlashLogger::logEvent(const char* message) {
    uint16_t len = strlen(message);
    if (len > sizeof(LogEntry::data) - 1) {
        len = sizeof(LogEntry::data) - 1;
    }
    return log(LogEntryType::EVENT, reinterpret_cast<const uint8_t*>(message), len);
}

bool FlashLogger::logError(const char* error_msg) {
    uint16_t len = strlen(error_msg);
    if (len > sizeof(LogEntry::data) - 1) {
        len = sizeof(LogEntry::data) - 1;
    }
    return log(LogEntryType::ERROR, reinterpret_cast<const uint8_t*>(error_msg), len);
}

bool FlashLogger::readEntry(uint32_t index, LogEntry& entry) {
    if (!initialized_) return false;

    String filename = "/flight_" + String(current_flight_.flight_number) + ".log";
    File file = SPIFFS.open(filename.c_str(), FILE_READ);
    if (!file) {
        return false;
    }

    // Skip header
    file.seek(sizeof(FlightLogHeader));

    // Read entries until we reach the desired index
    for (uint32_t i = 0; i <= index; i++) {
        if (file.available() < sizeof(LogEntryHeader)) {
            file.close();
            return false;
        }

        file.read(reinterpret_cast<uint8_t*>(&entry.header), sizeof(LogEntryHeader));

        if (file.available() < entry.header.length) {
            file.close();
            return false;
        }

        file.read(entry.data, entry.header.length);

        if (i == index) {
            file.close();
            return true;
        }
    }

    file.close();
    return false;
}

bool FlashLogger::clearAll() {
    if (!initialized_) return false;

    // Remove all flight log files
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String filename = file.name();
        if (filename.startsWith("/flight_")) {
            SPIFFS.remove(filename);
        }
        file = root.openNextFile();
    }
    root.close();

    current_flight_.flight_number = 0;
    Serial.println("[FlashLogger] Cleared all logs");
    return true;
}

uint8_t FlashLogger::getFlightLogCount() const {
    if (!initialized_) return 0;

    uint8_t count = 0;
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String filename = file.name();
        if (filename.startsWith("/flight_")) {
            count++;
        }
        file = root.openNextFile();
    }
    root.close();

    return count;
}

bool FlashLogger::getFlightLogInfo(uint8_t index, FlightLogHeader& info) {
    if (!initialized_) return false;

    // Find the nth flight log file
    uint8_t count = 0;
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String filename = file.name();
        if (filename.startsWith("/flight_")) {
            if (count == index) {
                // Read header
                file.read(reinterpret_cast<uint8_t*>(&info), sizeof(FlightLogHeader));
                file.close();
                root.close();
                return true;
            }
            count++;
        }
        file = root.openNextFile();
    }
    root.close();

    return false;
}

bool FlashLogger::writeEntry(const LogEntry& entry) {
    String filename = "/flight_" + String(current_flight_.flight_number) + ".log";
    File file = SPIFFS.open(filename.c_str(), FILE_APPEND);
    if (!file) {
        return false;
    }

    file.write(reinterpret_cast<const uint8_t*>(&entry.header), sizeof(LogEntryHeader));
    file.write(entry.data, entry.header.length);
    file.close();

    return true;
}

uint16_t FlashLogger::calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

} // namespace logging
} // namespace phoenix
