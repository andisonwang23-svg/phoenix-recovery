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
    BOOT = 7
};

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

    uint32_t entry_count_ = 0;
    uint32_t data_size_ = 0;
    uint8_t sequence_ = 0;

    FlightLogHeader current_flight_;
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