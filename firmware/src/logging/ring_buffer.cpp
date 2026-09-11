// ============================================================================
// PHOENIX RECOVERY — Ring Buffer Implementation.
// ============================================================================
#include "ring_buffer.h"

namespace phoenix {
namespace logging {

RingBuffer::RingBuffer() = default;

RingBuffer::~RingBuffer() {
    // Note: destructor is declared = default in header but defined here
    // to handle dynamic memory cleanup
    if (buffer_) {
        free(buffer_);
        buffer_ = nullptr;
    }
}

bool RingBuffer::begin(const RingBufferConfig& config) {
    config_ = config;

    if (!config_.enabled) {
        return true; // Buffer disabled, but initialization successful
    }

    // Allocate buffer memory
    uint32_t buffer_size = config_.max_entries * sizeof(BufferEntry);
    buffer_ = (BufferEntry*)malloc(buffer_size);

    if (!buffer_) {
        Serial.println("[RingBuffer] Failed to allocate buffer memory");
        return false;
    }

    // Clear buffer
    memset(buffer_, 0, buffer_size);

    stats_ = RingBufferStats{};
    initialized_ = true;

    Serial.printf("[RingBuffer] Initialized with %d entries (%d bytes)\n",
                  config_.max_entries, buffer_size);
    return true;
}

bool RingBuffer::push(BufferEntryType type, const uint8_t* data, uint16_t length) {
    if (!initialized_ || !config_.enabled) return false;
    if (length > sizeof(BufferEntry::data)) return false;

    // Check if buffer is full
    if (stats_.is_full && !config_.overwrite_old) {
        stats_.dropped_entries++;
        return false;
    }

    // Get write index
    uint32_t write_idx = nextWriteIndex();

    // If buffer is full and we're overwriting, advance read index
    if (stats_.is_full) {
        stats_.read_index = nextReadIndex();
        stats_.dropped_entries++;
    }

    // Write entry
    BufferEntry& entry = buffer_[write_idx];
    entry.timestamp_ms = millis();
    entry.type = static_cast<uint8_t>(type);
    entry.length = length;
    memcpy(entry.data, data, length);

    // Update stats
    stats_.write_index = write_idx;
    stats_.current_entries++;
    stats_.total_entries++;
    stats_.current_size_bytes += sizeof(BufferEntry) + length;

    // Update full state
    updateFullState();

    return true;
}

bool RingBuffer::pushTelemetry(const VehicleState& state) {
    // Pack telemetry data into compact format
    struct TelemetryData {
        float latitude;
        float longitude;
        float altitude_agl;
        float vertical_speed;
        float heading_error;
        float left_servo;
        float right_servo;
        uint8_t flight_state;
        uint8_t guidance_mode;
    } data;

    data.latitude = state.latitude;
    data.longitude = state.longitude;
    data.altitude_agl = state.altitude_agl_m;
    data.vertical_speed = state.vertical_speed_mps;
    data.heading_error = state.heading_error_deg;
    data.left_servo = state.left_servo_command;
    data.right_servo = state.right_servo_command;
    data.flight_state = static_cast<uint8_t>(state.flight_state);
    data.guidance_mode = static_cast<uint8_t>(state.guidance_mode);

    return push(BufferEntryType::TELEMETRY, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
}

bool RingBuffer::pushStateChange(logic::FlightState state, logic::GuidanceMode mode) {
    struct StateData {
        uint8_t flight_state;
        uint8_t guidance_mode;
    } data;

    data.flight_state = static_cast<uint8_t>(state);
    data.guidance_mode = static_cast<uint8_t>(mode);

    return push(BufferEntryType::STATE_CHANGE, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
}

bool RingBuffer::pushEvent(const char* message) {
    uint16_t len = strlen(message);
    if (len > sizeof(BufferEntry::data) - 1) {
        len = sizeof(BufferEntry::data) - 1;
    }
    return push(BufferEntryType::EVENT, reinterpret_cast<const uint8_t*>(message), len);
}

bool RingBuffer::pushError(const char* error_msg) {
    uint16_t len = strlen(error_msg);
    if (len > sizeof(BufferEntry::data) - 1) {
        len = sizeof(BufferEntry::data) - 1;
    }
    return push(BufferEntryType::ERROR, reinterpret_cast<const uint8_t*>(error_msg), len);
}

bool RingBuffer::pop(BufferEntry& entry) {
    if (!initialized_ || !hasData()) return false;

    // Read from read index
    entry = buffer_[stats_.read_index];

    // Update read index
    stats_.read_index = nextReadIndex();
    stats_.current_entries--;

    // Update size estimate
    if (stats_.current_size_bytes >= sizeof(BufferEntry)) {
        stats_.current_size_bytes -= sizeof(BufferEntry);
    } else {
        stats_.current_size_bytes = 0;
    }

    // Update full state
    updateFullState();

    return true;
}

bool RingBuffer::peek(BufferEntry& entry) const {
    if (!initialized_ || !hasData()) return false;

    entry = buffer_[stats_.read_index];
    return true;
}

void RingBuffer::clear() {
    if (!initialized_) return;

    stats_ = RingBufferStats{};
    if (buffer_) {
        memset(buffer_, 0, config_.max_entries * sizeof(BufferEntry));
    }
}

uint32_t RingBuffer::getAll(BufferEntry* entries, uint32_t max_count) {
    if (!initialized_ || !hasData()) return 0;

    uint32_t count = 0;
    uint32_t read_idx = stats_.read_index;

    while (count < max_count && count < stats_.current_entries) {
        entries[count] = buffer_[read_idx];
        read_idx = (read_idx + 1) % config_.max_entries;
        count++;
    }

    // Update read index
    stats_.read_index = read_idx;
    stats_.current_entries -= count;

    return count;
}

uint32_t RingBuffer::getSince(uint32_t timestamp_ms, BufferEntry* entries, uint32_t max_count) {
    if (!initialized_ || !hasData()) return 0;

    uint32_t count = 0;
    uint32_t read_idx = stats_.read_index;

    for (uint32_t i = 0; i < stats_.current_entries && count < max_count; i++) {
        if (buffer_[read_idx].timestamp_ms >= timestamp_ms) {
            entries[count] = buffer_[read_idx];
            count++;
        }
        read_idx = (read_idx + 1) % config_.max_entries;
    }

    return count;
}

uint32_t RingBuffer::nextWriteIndex() const {
    return (stats_.write_index + 1) % config_.max_entries;
}

uint32_t RingBuffer::nextReadIndex() const {
    return (stats_.read_index + 1) % config_.max_entries;
}

void RingBuffer::updateFullState() {
    stats_.is_full = (stats_.current_entries >= config_.max_entries);
}

} // namespace logging
} // namespace phoenix