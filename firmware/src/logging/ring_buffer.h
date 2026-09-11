// ============================================================================
// PHOENIX RECOVERY — RAM Ring Buffer (High-Speed Telemetry Logging).
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"
#include "vehicle_state.h"
#include "../logic/state_machine.h"

namespace phoenix {
namespace logging {

// Ring buffer entry types
enum class BufferEntryType : uint8_t {
    TELEMETRY = 0,
    STATE_CHANGE = 1,
    EVENT = 2,
    ERROR = 3
};

// Ring buffer entry
struct BufferEntry {
    uint32_t timestamp_ms;
    uint8_t type;
    uint16_t length;
    uint8_t data[128];
};

// Ring buffer configuration
struct RingBufferConfig {
    bool enabled = false;
    uint32_t max_entries = 500;        // Max entries in buffer
    uint32_t max_size_bytes = 64 * 1024; // 64KB max
    bool overwrite_old = true;         // Overwrite oldest entries when full
    uint32_t flush_interval_ms = 10000; // Auto-flush to flash every 10s
};

// Ring buffer stats
struct RingBufferStats {
    uint32_t total_entries = 0;
    uint32_t dropped_entries = 0;
    uint32_t current_entries = 0;
    uint32_t current_size_bytes = 0;
    uint32_t write_index = 0;
    uint32_t read_index = 0;
    bool is_full = false;
};

class RingBuffer {
public:
    RingBuffer();
    ~RingBuffer();

    // Initialize ring buffer
    bool begin(const RingBufferConfig& config = RingBufferConfig());

    // Add entry to buffer
    bool push(BufferEntryType type, const uint8_t* data, uint16_t length);

    // Add telemetry snapshot
    bool pushTelemetry(const VehicleState& state);

    // Add state change
    bool pushStateChange(logic::FlightState state, logic::GuidanceMode mode);

    // Add event
    bool pushEvent(const char* message);

    // Add error
    bool pushError(const char* error_msg);

    // Get next entry (for reading)
    bool pop(BufferEntry& entry);

    // Peek at next entry without removing
    bool peek(BufferEntry& entry) const;

    // Check if buffer has data
    bool hasData() const { return stats_.current_entries > 0; }

    // Check if buffer is full
    bool isFull() const { return stats_.is_full; }

    // Get buffer stats
    const RingBufferStats& getStats() const { return stats_; }

    // Clear buffer
    void clear();

    // Get all entries (for batch read)
    uint32_t getAll(BufferEntry* entries, uint32_t max_count);

    // Get entries since timestamp
    uint32_t getSince(uint32_t timestamp_ms, BufferEntry* entries, uint32_t max_count);

private:
    RingBufferConfig config_;
    RingBufferStats stats_;
    BufferEntry* buffer_ = nullptr;
    bool initialized_ = false;

    // Calculate next write index
    uint32_t nextWriteIndex() const;

    // Calculate next read index
    uint32_t nextReadIndex() const;

    // Check if buffer is full
    void updateFullState();
};

} // namespace logging
} // namespace phoenix