// ============================================================================
// PHOENIX RECOVERY — Flash Logger (Persistent Flight Data Logging).
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"
#include "vehicle_state.h"

namespace phoenix {
namespace logging {

// Log entry types
enum class LogEntryType : uint8_t {
    FLIGHT_STATE = 0,
    TELEMETRY = 1,
    SENSOR_DATA = 2,
    GUIDANCE = 3,
    SERVO_COMMAND = 4,
    EVENT = 5,
    ERROR = 6,
    BOOT = 7,
    CONFIG_SNAPSHOT = 8
};

#pragma pack(push, 1)
// Stable, host-decodable schema used by log version 3. Do not rely on compiler
// padding when logs must be replayed on a different computer.
struct TelemetryRecordV3 {
    uint8_t schema_version;
    uint8_t drop_test_state;
    uint8_t guided_drop_status;
    uint8_t guided_authority;
    uint8_t flight_state;
    uint8_t guidance_mode;
    uint8_t failure_code;
    uint8_t validity_bits;
    uint8_t control_flags;
    uint8_t satellites;
    uint8_t reset_reason;
    uint8_t reserved;
    uint16_t drop_test_id;
    uint32_t sample_timestamp_ms;
    uint32_t configuration_version;
    uint32_t gps_age_ms;
    uint32_t baro_age_ms;
    uint32_t imu_age_ms;
    double latitude;
    double longitude;
    double target_latitude;
    double target_longitude;
    float gps_altitude;
    float hdop;
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
};

struct ConfigurationSnapshotV1 {
    uint8_t schema_version;
    uint8_t guided_authority;
    uint8_t flare_enabled;
    uint8_t servo1_controls_left_brake;
    uint32_t configuration_version;
    double target_latitude;
    double target_longitude;
    float servo1_neutral_us;
    float servo1_min_us;
    float servo1_max_us;
    float servo1_max_brake_us;
    float servo2_neutral_us;
    float servo2_min_us;
    float servo2_max_us;
    float servo2_max_brake_us;
};
#pragma pack(pop)

static_assert(sizeof(TelemetryRecordV3) == 194,
              "TelemetryRecordV3 changed; update tools/export_drop_log.py");
static_assert(sizeof(ConfigurationSnapshotV1) == 56,
              "ConfigurationSnapshotV1 changed; update tools/export_drop_log.py");

// Flight log entry header
struct LogEntryHeader {
    uint32_t timestamp_ms;
    uint8_t type;
    uint8_t sequence;
    uint16_t length;  // Data length in bytes
};

// Flight log entry
struct LogEntry {
    LogEntryHeader header;
    uint8_t data[256]; // Variable length data
};

// Flight log header (stored at beginning of flash partition)
struct FlightLogHeader {
    char magic[4];           // "PRFL"
    uint8_t version;
    uint8_t flight_number;
    uint8_t reserved[2];
    uint32_t start_time_ms;
    uint32_t end_time_ms;
    uint32_t entry_count;
    uint32_t data_size;
    uint16_t crc;
};

static_assert(sizeof(FlightLogHeader) == 28,
              "FlightLogHeader changed; update tools/export_drop_log.py");

// Flash logger configuration
struct FlashLoggerConfig {
    bool enabled = false;
    uint32_t max_entries = 10000;        // Max log entries
    uint32_t max_data_size = 1024 * 1024; // 1MB max
    bool auto_erase_old = true;          // Auto-erase old flights
    uint8_t max_flight_logs = 5;         // Max flight logs to keep
    bool compress = false;               // Enable compression
};

class FlashLogger {
public:
    FlashLogger();
    ~FlashLogger() = default;

    // Initialize flash logger
    bool begin(const FlashLoggerConfig& config = FlashLoggerConfig());

    // Start a new flight log
    bool startFlightLog();

    // End current flight log
    bool endFlightLog();

    // Log an entry
    bool log(LogEntryType type, const uint8_t* data, uint16_t length);

    // Log flight state change
    bool logFlightState(const VehicleState& state);

    // Log telemetry snapshot
    bool logTelemetry(const VehicleState& state);

    bool logConfigurationSnapshot(const ConfigurationSnapshotV1& snapshot);

    // Log event with message
    bool logEvent(const char* message);

    // Log error
    bool logError(const char* error_msg);

    // Check if logging is active
    bool isLogging() const { return logging_active_; }

    // Get current log stats
    uint32_t getEntryCount() const { return entry_count_; }
    uint32_t getDataSize() const { return data_size_; }

    // Read log entries (for playback)
    bool readEntry(uint32_t index, LogEntry& entry);

    // Clear all logs
    bool clearAll();

    // Get number of stored flight logs
    uint8_t getFlightLogCount() const;

    // Get flight log info
    bool getFlightLogInfo(uint8_t index, FlightLogHeader& info);

private:
    FlashLoggerConfig config_;
    bool initialized_ = false;
    bool logging_active_ = false;
    bool write_fault_reported_ = false;

    uint32_t entry_count_ = 0;
    uint32_t data_size_ = 0;
    uint8_t sequence_ = 0;

    FlightLogHeader current_flight_{};
    uint32_t current_flight_offset_ = 0;

    // Flash memory management
    bool eraseFlash();
    bool writeHeader(const FlightLogHeader& header);
    bool readHeader(FlightLogHeader& header);
    bool writeEntry(const LogEntry& entry);

    // CRC calculation
    uint16_t calculateCRC(const uint8_t* data, size_t length);
};

} // namespace logging
} // namespace phoenix
